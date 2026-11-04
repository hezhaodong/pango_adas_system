//=============================================================================
// binarize.v
//-----------------------------------------------------------------------------
// 功能：灰度二值化（阈值可配）
//
//   o_bin = (i_data >= THRESH) ? 1 : 0
//
// 赛题要求「二值化」，这是最基础也是最有用的形态学预处理。
// 后续可以接：
//   - 连通域标记
//   - 行/列投影统计（车道线检测）
//   - 形态学腐蚀/膨胀
//
// 流水：1 级
//
// 适用：紫光同创 Logos2 系列，纯 Verilog-2001，无厂商原语
//=============================================================================
`timescale 1ns / 1ps
`default_nettype none

module binarize #(
    parameter DW      = 8,
    parameter THRESH  = 8'd128,     // 二值化阈值
    parameter INVERT  = 0           // 1 = 反相（暗背景取亮目标时用）
)(
    input  wire            i_clk,
    input  wire            i_rstn,
    input  wire            i_valid,
    input  wire [DW-1:0]   i_data,

    output reg             o_valid,
    output reg             o_bin
);

    wire w_bin_raw = (i_data >= THRESH[7:0]);
    wire w_bin     = INVERT ? ~w_bin_raw : w_bin_raw;

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            o_bin   <= 1'b0;
            o_valid <= 1'b0;
        end else begin
            o_bin   <= w_bin;
            o_valid <= i_valid;
        end
    end

endmodule

`default_nettype wire
