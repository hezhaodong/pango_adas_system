//=============================================================================
// line_buffer_3x3.v
//-----------------------------------------------------------------------------
// 功能：把逐像素的流式数据转成 3x3 窗口流（任何卷积/滤波的必备基础设施）
//
// 窗口编号（标准卷积核方向，p00 是左上角）：
//
//      p00 p01 p02      第 0 行 = 两行前（最旧）
//      p10 p11 p12      第 1 行 = 一行前
//      p20 p21 p22      第 2 行 = 当前行（最新）
//       ^   ^   ^
//      列 c-2 c-1 c
//
//-----------------------------------------------------------------------------
// 实现原理（只用 2 个行缓冲 + 6 个移位寄存器）
//
//   行缓冲 lb0：存「上一行」（第 N-1 行）
//   行缓冲 lb1：存「上上行」（第 N-2 行）
//
//   每个像素周期：
//     lb0[col] 写入当前像素（第 N 行）
//     lb1[col] 写入 lb0[col] 的旧值（第 N-1 行）—— 这就是「行移位」
//     异步读 lb0[col] = 第 N-1 行第 col 列
//     异步读 lb1[col] = 第 N-2 行第 col 列
//
//   横向 3 像素：每行用 2 级移位寄存器延迟
//
//   「异步读 + 同步写」→ 推断为分布式 RAM（LUTRAM）
//   1280x8 两个约 400 LUT，占 PG2L50H（50K LUT）的 0.8%，代价可忽略
//   好处：零读延迟，窗口对齐简单，不易出错
//
//-----------------------------------------------------------------------------
// 时序：输出比输入延迟 1 拍（窗口已寄存，利于时序收敛）
//
// 边界处理：最上方 2 行、最左侧 2 列不产生有效窗口（o_valid=0）
//
// 适用：紫光同创 Logos2 系列，纯 Verilog-2001，无厂商原语
//=============================================================================
`timescale 1ns / 1ps
`default_nettype none

module line_buffer_3x3 #(
    parameter LINE_LENGTH = 1280,      // 一行像素数
    parameter DATA_WIDTH  = 8          // 像素位宽
)(
    input  wire                    i_clk,
    input  wire                    i_rstn,       // 同步低有效复位
    input  wire                    i_de,         // 数据有效（行内）
    input  wire                    i_vsync,      // 帧同步
    input  wire [DATA_WIDTH-1:0]   i_data,       // 像素输入

    output reg                     o_valid,      // 窗口有效
    output reg  [DATA_WIDTH-1:0]   o_p00, o_p01, o_p02,   // 第 0 行（最旧）
    output reg  [DATA_WIDTH-1:0]   o_p10, o_p11, o_p12,   // 第 1 行
    output reg  [DATA_WIDTH-1:0]   o_p20, o_p21, o_p22    // 第 2 行（当前）
);

    //------------------------------------------------------------------
    // 常量计算（函数必须先定义后使用）
    //------------------------------------------------------------------
    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam CW       = clog2(LINE_LENGTH);
    localparam COL_LAST = LINE_LENGTH - 1;

    //------------------------------------------------------------------
    // 行缓冲：lb0 = 第 N-1 行，lb1 = 第 N-2 行
    //------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] lb0 [0:LINE_LENGTH-1];
    reg [DATA_WIDTH-1:0] lb1 [0:LINE_LENGTH-1];

    // 异步读（零延迟）
    wire [DATA_WIDTH-1:0] q0 = lb0[col];       // 第 N-1 行第 col 列
    wire [DATA_WIDTH-1:0] q1 = lb1[col];       // 第 N-2 行第 col 列

    //------------------------------------------------------------------
    // 列计数 / 行计数
    //------------------------------------------------------------------
    reg [CW-1:0] col;
    reg [1:0]    row;          // 当前行序号（0,1,2,... 饱和于 3）
    reg          de_d;
    wire         line_end = ~i_de & de_d;      // 行结束（de 下降沿）

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            col  <= {CW{1'b0}};
            row  <= 2'd0;
            de_d <= 1'b0;
        end else begin
            de_d <= i_de;

            // ---- 列计数：de 有效时递增，行结束或帧同步时归零 ----
            if (i_vsync) begin
                col <= {CW{1'b0}};
            end else if (!i_de) begin
                col <= {CW{1'b0}};
            end else if (col == COL_LAST[CW-1:0]) begin
                col <= {CW{1'b0}};
            end else begin
                col <= col + 1'b1;
            end

            // ---- 行计数：一行结束时 +1（饱和，不回绕）----
            if (i_vsync) begin
                row <= 2'd0;
            end else if (line_end && (row != 2'd3)) begin
                row <= row + 1'b1;
            end
        end
    end

    //------------------------------------------------------------------
    // 行缓冲写（同步）+ 行移位
    //   q0 是 lb0[col] 的「写前旧值」，所以 lb1 拿到的是上一行数据
    //------------------------------------------------------------------
    always @(posedge i_clk) begin
        if (i_de) begin
            lb0[col] <= i_data;                // 当前行 → lb0
            lb1[col] <= q0;                    // 第 N-1 行 → lb1
        end
    end

    //------------------------------------------------------------------
    // 横向窗口：每行 2 级移位寄存器
    //------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] r0_d1, r0_d2;         // 第 N-2 行延迟
    reg [DATA_WIDTH-1:0] r1_d1, r1_d2;         // 第 N-1 行延迟
    reg [DATA_WIDTH-1:0] r2_d1, r2_d2;         // 第 N   行延迟

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            r0_d1 <= {DATA_WIDTH{1'b0}};  r0_d2 <= {DATA_WIDTH{1'b0}};
            r1_d1 <= {DATA_WIDTH{1'b0}};  r1_d2 <= {DATA_WIDTH{1'b0}};
            r2_d1 <= {DATA_WIDTH{1'b0}};  r2_d2 <= {DATA_WIDTH{1'b0}};
        end else if (i_de) begin
            r2_d1 <= i_data;   r2_d2 <= r2_d1;
            r1_d1 <= q0;       r1_d2 <= r1_d1;
            r0_d1 <= q1;       r0_d2 <= r0_d1;
        end
    end

    //------------------------------------------------------------------
    // 输出寄存（1 拍），与窗口完全对齐
    //
    //   第 t 拍：col=c  q0=第N-1行[c]  q1=第N-2行[c]
    //            r2_d1=第N行[c-1]  r2_d2=第N行[c-2]  ...
    //   寄存到 t+1 拍输出 = 完整对齐的 3x3 窗口
    //------------------------------------------------------------------
    always @(posedge i_clk) begin
        if (!i_rstn) begin
            o_p00 <= {DATA_WIDTH{1'b0}};  o_p01 <= {DATA_WIDTH{1'b0}};  o_p02 <= {DATA_WIDTH{1'b0}};
            o_p10 <= {DATA_WIDTH{1'b0}};  o_p11 <= {DATA_WIDTH{1'b0}};  o_p12 <= {DATA_WIDTH{1'b0}};
            o_p20 <= {DATA_WIDTH{1'b0}};  o_p21 <= {DATA_WIDTH{1'b0}};  o_p22 <= {DATA_WIDTH{1'b0}};
            o_valid <= 1'b0;
        end else begin
            o_p00 <= r0_d2;   o_p01 <= r0_d1;   o_p02 <= q1;
            o_p10 <= r1_d2;   o_p11 <= r1_d1;   o_p12 <= q0;
            o_p20 <= r2_d2;   o_p21 <= r2_d1;   o_p22 <= i_data;
            // 边界抑制：至少 3 行（row>=2）且至少 3 列（col>=2）
            o_valid <= i_de && (col >= 2) && (row >= 2);
        end
    end

endmodule

`default_nettype wire
