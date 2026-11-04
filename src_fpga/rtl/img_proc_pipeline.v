//=============================================================================
// img_proc_pipeline.v
//-----------------------------------------------------------------------------
// 功能：FPGA 侧图像处理流水线顶层 —— 赛题「基础任务 3」的实现
//
// 数据流：
//
//                        ┌──────────────────────────────────────┐
//   RGB565 ──► 灰度化 ──►│ 3x3 窗口生成                          │
//                        └───┬──────────────┬───────────────┬───┘
//                            │              │               │
//                       高斯去噪        Sobel 边缘       帧差运动检测
//                       (去噪)          (感知)          (运动目标)
//                            │              │
//                            │          二值化
//                            │              │
//                            ▼              ▼
//                         o_blur       o_edge_bin
//
//   RGB565 ──────────────────────────────► 红绿灯 ROI 颜色统计（并行支路）
//
//-----------------------------------------------------------------------------
// 各输出相对 i_de 的延迟（时钟周期）：
//   o_gray_de / o_gray            : 2
//   o_blur_de / o_blur            : 5
//   o_edge_de / o_edge_mag/bin    : 5
//   o_bin_de  / o_bin             : 6
//   红绿灯统计 / 运动统计          : 每帧更新
//
//-----------------------------------------------------------------------------
// 使用方法：
//   在顶层模块里例化本模块，把 OV5640 采集出来的 RGB565 数据流接进来，
//   各输出按需接到 PCIe DMA / 显示 / 寄存器。
//
//   不需要的支路直接在综合时优化掉即可（若想显式关闭，
//   把对应模块的例化注释掉，或加 `ifdef 开关）。
//
// 资源估算（1280 宽，PG2L50H）：
//   line_buffer_3x3   : 2 x 1280x8 分布式 RAM  ≈ 400 LUT
//   frame_diff_motion : 80x45x8 分布式 RAM     ≈ 450 LUT
//   其余为普通逻辑 + 移位加法，无乘法器、无 DSP
//   合计约 1000~1500 LUT（占 50K LUT 的 2~3%）
//
// 适用：紫光同创 Logos2 系列，纯 Verilog-2001，无厂商原语
//=============================================================================
`timescale 1ns / 1ps
`default_nettype none

module img_proc_pipeline #(
    parameter IMG_W        = 1280,
    parameter IMG_H        = 720,
    parameter SOBEL_THRESH = 13'd256,     // Sobel 二值边缘阈值 0~2040
    parameter BIN_THRESH   = 8'd128,      // 灰度二值化阈值
    parameter DIFF_THRESH  = 8'd24,       // 帧差运动阈值
    parameter BLOCK_LOG2   = 4            // 运动检测块大小 = 2^BLOCK_LOG2
)(
    input  wire        i_clk,
    input  wire        i_rstn,
    input  wire        i_de,              // 像素有效（来自采集模块）
    input  wire        i_vsync,           // 帧同步（来自采集模块）
    input  wire [15:0] i_rgb565,          // RGB565 像素

    // ---- 灰度化输出（延迟 2）----
    output wire        o_gray_de,
    output wire [7:0]  o_gray,

    // ---- 高斯去噪输出（延迟 5）----
    output wire        o_blur_de,
    output wire [7:0]  o_blur,

    // ---- Sobel 边缘输出（延迟 5）----
    output wire        o_edge_de,
    output wire [7:0]  o_edge_mag,        // 梯度幅值（可当灰度图看）
    output wire        o_edge_bin,        // 二值边缘

    // ---- 灰度二值化输出（延迟 6）----
    output wire        o_bin_de,
    output wire        o_bin,

    // ---- 红绿灯识别（每帧更新）----
    output wire [31:0] o_red_cnt,
    output wire [31:0] o_green_cnt,
    output wire [31:0] o_total_cnt,
    output wire [1:0]  o_light,           // 0=无 1=红 2=绿 3=黄

    // ---- 运动检测（每帧更新）----
    output wire        o_motion_any,
    output wire [15:0] o_motion_cnt
);

    //==================================================================
    // 1) RGB565 → 灰度（延迟 2）
    //==================================================================
    wire        gray_de;
    wire [7:0]  gray;

    rgb565_to_gray u_gray (
        .i_clk    (i_clk),
        .i_rstn   (i_rstn),
        .i_de     (i_de),
        .i_rgb565 (i_rgb565),
        .o_de     (gray_de),
        .o_gray   (gray)
    );

    assign o_gray_de = gray_de;
    assign o_gray    = gray;

    // vsync 延迟 2 拍，与 gray_de 对齐
    reg vs_d1, vs_d2;
    always @(posedge i_clk) begin
        if (!i_rstn) begin
            vs_d1 <= 1'b0;
            vs_d2 <= 1'b0;
        end else begin
            vs_d1 <= i_vsync;
            vs_d2 <= vs_d1;
        end
    end

    wire vsync_aligned = vs_d2;

    // 帧结束脉冲（vsync 上升沿）
    reg  vs_d3;
    always @(posedge i_clk) begin
        if (!i_rstn) vs_d3 <= 1'b0;
        else         vs_d3 <= i_vsync;
    end
    wire frame_end = i_vsync & ~vs_d3;

    //==================================================================
    // 2) 3x3 窗口生成（延迟 1）
    //==================================================================
    wire        w_valid;
    wire [7:0]  p00,p01,p02,p10,p11,p12,p20,p21,p22;

    line_buffer_3x3 #(
        .LINE_LENGTH (IMG_W),
        .DATA_WIDTH  (8)
    ) u_linebuf (
        .i_clk   (i_clk),
        .i_rstn  (i_rstn),
        .i_de    (gray_de),
        .i_vsync (vsync_aligned),
        .i_data  (gray),
        .o_valid (w_valid),
        .o_p00(p00), .o_p01(p01), .o_p02(p02),
        .o_p10(p10), .o_p11(p11), .o_p12(p12),
        .o_p20(p20), .o_p21(p21), .o_p22(p22)
    );

    //==================================================================
    // 3) 高斯去噪（延迟 2）
    //==================================================================
    gaussian_3x3 #(
        .DW (8)
    ) u_gaussian (
        .i_clk (i_clk), .i_rstn (i_rstn), .i_valid (w_valid),
        .i_p00(p00), .i_p01(p01), .i_p02(p02),
        .i_p10(p10), .i_p11(p11), .i_p12(p12),
        .i_p20(p20), .i_p21(p21), .i_p22(p22),
        .o_valid (o_blur_de),
        .o_data  (o_blur)
    );

    //==================================================================
    // 4) Sobel 边缘检测（延迟 2）
    //==================================================================
    sobel_edge #(
        .DW     (8),
        .THRESH (SOBEL_THRESH)
    ) u_sobel (
        .i_clk (i_clk), .i_rstn (i_rstn), .i_valid (w_valid),
        .i_p00(p00), .i_p01(p01), .i_p02(p02),
        .i_p10(p10), .i_p11(p11), .i_p12(p12),
        .i_p20(p20), .i_p21(p21), .i_p22(p22),
        .o_valid (o_edge_de),
        .o_mag   (o_edge_mag),
        .o_edge  (o_edge_bin)
    );

    //==================================================================
    // 5) 灰度二值化（接在 Sobel 幅值后面，延迟 1）
    //    如果想对「灰度图」而不是「梯度图」做二值化，
    //    把 .i_data 从 o_edge_mag 改成 o_blur 并把 i_valid 改成 o_blur_de
    //==================================================================
    binarize #(
        .DW     (8),
        .THRESH (BIN_THRESH)
    ) u_binarize (
        .i_clk   (i_clk),
        .i_rstn  (i_rstn),
        .i_valid (o_edge_de),
        .i_data  (o_edge_mag),
        .o_valid (o_bin_de),
        .o_bin   (o_bin)
    );

    //==================================================================
    // 6) 红绿灯 ROI 颜色统计（并行支路，直接吃 RGB565）
    //==================================================================
    color_detect_roi #(
        .X_MIN (0),
        .X_MAX (IMG_W-1),
        .Y_MIN (0),
        .Y_MAX (IMG_H-1),
        .CNT_THRESH (32'd40)
    ) u_color (
        .i_clk        (i_clk),
        .i_rstn       (i_rstn),
        .i_de         (i_de),
        .i_vsync      (i_vsync),
        .i_frame_end  (frame_end),
        .i_rgb565     (i_rgb565),
        .o_red_cnt    (o_red_cnt),
        .o_green_cnt  (o_green_cnt),
        .o_yellow_cnt (),
        .o_total_cnt  (o_total_cnt),
        .o_light      (o_light),
        .o_frame_done ()
    );

    //==================================================================
    // 7) 帧差运动检测（吃灰度，降采样存上一帧）
    //==================================================================
    frame_diff_motion #(
        .IMG_W       (IMG_W),
        .IMG_H       (IMG_H),
        .BLOCK_LOG2  (BLOCK_LOG2),
        .DIFF_THRESH (DIFF_THRESH)
    ) u_motion (
        .i_clk          (i_clk),
        .i_rstn         (i_rstn),
        .i_de           (gray_de),
        .i_vsync        (vsync_aligned),
        .i_gray         (gray),
        .o_motion_any   (o_motion_any),
        .o_motion_cnt   (o_motion_cnt),
        .o_block_valid  (),
        .o_block_idx    (),
        .o_block_motion ()
    );

endmodule

`default_nettype wire
