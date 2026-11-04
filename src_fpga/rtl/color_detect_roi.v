//=============================================================================
// color_detect_roi.v
//-----------------------------------------------------------------------------
// 功能：在指定 ROI 区域内统计红/绿/黄像素数量，并按阈值判断信号灯颜色
//
// 这是赛题「红绿灯识别」的 FPGA 侧实现 —— 不依赖 NPU，纯硬件并行统计，
// 每帧给出结果，延迟 = 1 帧（16.7ms @60fps）
//
// 判定规则（RGB565，阈值均可配）：
//   红：R 足够高，G 和 B 都低
//   绿：G 足够高，R 和 B 都低
//   黄：R 和 G 都高，B 低
//
// 输出：
//   o_red_cnt / o_green_cnt / o_yellow_cnt   本帧 ROI 内各类像素数
//   o_total_cnt                              本帧 ROI 内总有效像素数
//   o_light                                  颜色判定结果
//        0 = 无/不确定   1 = 红   2 = 绿   3 = 黄
//   o_frame_done                             帧统计完成脉冲（锁存新结果）
//
// ROI 用参数配置（X/Y 范围）。若要与 ARM 联动动态改 ROI，
// 把 X_MIN/X_MAX/Y_MIN/Y_MAX 从 parameter 改成 input 端口即可。
//
// 适用：紫光同创 Logos2 系列，纯 Verilog-2001，无厂商原语
//=============================================================================
`timescale 1ns / 1ps
`default_nettype none

module color_detect_roi #(
    // ---- ROI 区域（含边界）----
    parameter X_MIN = 0,
    parameter X_MAX = 1279,
    parameter Y_MIN = 0,
    parameter Y_MAX = 719,

    // ---- 颜色判定阈值（RGB565 的 5/6/5 位域）----
    parameter RED_R_MIN    = 5'd20,   // 红：R >= 20 (约 65%)
    parameter RED_G_MAX    = 6'd20,   //      G <= 20 (约 32%)
    parameter RED_B_MAX    = 5'd16,   //      B <= 16 (约 52%)
    parameter GRN_G_MIN    = 6'd36,   // 绿：G >= 36 (约 57%)
    parameter GRN_R_MAX    = 5'd16,   //      R <= 16 (约 52%)
    parameter GRN_B_MAX    = 5'd16,   //      B <= 16 (约 52%)
    parameter YEL_R_MIN    = 5'd20,   // 黄：R >= 20
    parameter YEL_G_MIN    = 6'd36,   //      G >= 36
    parameter YEL_B_MAX    = 5'd10,   //      B <= 10 (约 32%)

    // ---- 判定为「该颜色存在」所需的最小像素数 ----
    parameter CNT_THRESH   = 32'd40
)(
    input  wire        i_clk,
    input  wire        i_rstn,
    input  wire        i_de,          // 像素有效
    input  wire        i_vsync,       // 帧同步
    input  wire        i_frame_end,   // 帧结束脉冲（可由 vsync 上升沿产生）
    input  wire [15:0] i_rgb565,

    output reg  [31:0] o_red_cnt,
    output reg  [31:0] o_green_cnt,
    output reg  [31:0] o_yellow_cnt,
    output reg  [31:0] o_total_cnt,
    output reg  [1:0]  o_light,
    output reg         o_frame_done
);

    //------------------------------------------------------------------
    // 行/列坐标计数
    //------------------------------------------------------------------
    reg [11:0] x;
    reg [11:0] y;

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            x <= 12'd0;
            y <= 12'd0;
        end else if (i_vsync) begin
            x <= 12'd0;
            y <= 12'd0;
        end else if (i_de) begin
            if (x >= X_MAX[11:0]) begin
                x <= 12'd0;
                y <= (y >= Y_MAX[11:0]) ? 12'd0 : (y + 12'd1);
            end else begin
                x <= x + 12'd1;
            end
        end
    end

    wire in_roi = (x >= X_MIN[11:0]) && (x <= X_MAX[11:0]) &&
                  (y >= Y_MIN[11:0]) && (y <= Y_MAX[11:0]);

    //------------------------------------------------------------------
    // 颜色分类
    //------------------------------------------------------------------
    wire [4:0] r5 = i_rgb565[15:11];
    wire [5:0] g6 = i_rgb565[10:5];
    wire [4:0] b5 = i_rgb565[4:0];

    wire is_red = (r5 >= RED_R_MIN) && (g6 <= RED_G_MAX) && (b5 <= RED_B_MAX);
    wire is_grn = (g6 >= GRN_G_MIN) && (r5 <= GRN_R_MAX) && (b5 <= GRN_B_MAX);
    wire is_yel = (r5 >= YEL_R_MIN) && (g6 >= YEL_G_MIN) && (b5 <= YEL_B_MAX);

    wire hit = i_de && in_roi;

    //------------------------------------------------------------------
    // 帧内累加
    //------------------------------------------------------------------
    reg [31:0] acc_red, acc_grn, acc_yel, acc_tot;

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            acc_red <= 32'd0;
            acc_grn <= 32'd0;
            acc_yel <= 32'd0;
            acc_tot <= 32'd0;
        end else if (i_vsync) begin
            // 帧开始，清零（结果已在上一帧结束时锁存）
            acc_red <= 32'd0;
            acc_grn <= 32'd0;
            acc_yel <= 32'd0;
            acc_tot <= 32'd0;
        end else if (hit) begin
            acc_tot <= acc_tot + 32'd1;
            if (is_red)      acc_red <= acc_red + 32'd1;
            if (is_grn)      acc_grn <= acc_grn + 32'd1;
            if (is_yel)      acc_yel <= acc_yel + 32'd1;
        end
    end

    //------------------------------------------------------------------
    // 帧结束锁存 + 颜色判定
    //   优先级：绿 > 红 > 黄（避免黄色误判成红）
    //------------------------------------------------------------------
    always @(posedge i_clk) begin
        if (!i_rstn) begin
            o_red_cnt    <= 32'd0;
            o_green_cnt  <= 32'd0;
            o_yellow_cnt <= 32'd0;
            o_total_cnt  <= 32'd0;
            o_light      <= 2'd0;
            o_frame_done <= 1'b0;
        end else if (i_frame_end) begin
            o_red_cnt    <= acc_red;
            o_green_cnt  <= acc_grn;
            o_yellow_cnt <= acc_yel;
            o_total_cnt  <= acc_tot;

            if      (acc_grn >= CNT_THRESH) o_light <= 2'd2;   // 绿
            else if (acc_red >= CNT_THRESH) o_light <= 2'd1;   // 红
            else if (acc_yel >= CNT_THRESH) o_light <= 2'd3;   // 黄
            else                            o_light <= 2'd0;   // 无

            o_frame_done <= 1'b1;
        end else begin
            o_frame_done <= 1'b0;
        end
    end

endmodule

`default_nettype wire
