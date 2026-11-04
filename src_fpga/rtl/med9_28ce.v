// ============================================================================
// med9_28ce.v : 3x3 中值滤波的 9 选 5 选择网络（28 个 compare-swap，9 级）
// 来源：本工程自研（Batcher 奇偶归并 16 路排序网络 + 剪枝），生成脚本
//       _work/fpga_prep/tools/gen_med9.py；已验证 = sorted(v)[4]（全 0/1、
//       值域 2..5 穷举、30 万组随机）。
// 许可：随本工程，无第三方代码。
// 用法：3 个通道各例化一份，W=5(R/B, RGB565 的 5bit) / W=6(G, 6bit)；
//       也可先用位复制扩展到 8bit 再统一 W=8（多耗约 30% LUT）。
// 注意：本文件未做综合/仿真验证，上板前请先跑 Pango 综合 + 与 Python 黄金模型比对。
// ============================================================================
module med9_28ce #(
    parameter W = 8
) (
    input  wire [W-1:0] p00, p01, p02,
    input  wire [W-1:0] p10, p11, p12,
    input  wire [W-1:0] p20, p21, p22,
    output wire [W-1:0] med
);

    // ---- level 1 (4 个 compare-swap) ----
    wire lt_1_0_1 = (p00 < p01);
    wire [W-1:0] lo_1_0 = lt_1_0_1 ? p00 : p01;
    wire [W-1:0] hi_1_1 = lt_1_0_1 ? p01 : p00;
    wire lt_1_2_3 = (p02 < p10);
    wire [W-1:0] lo_1_2 = lt_1_2_3 ? p02 : p10;
    wire [W-1:0] hi_1_3 = lt_1_2_3 ? p10 : p02;
    wire lt_1_4_5 = (p11 < p12);
    wire [W-1:0] lo_1_4 = lt_1_4_5 ? p11 : p12;
    wire [W-1:0] hi_1_5 = lt_1_4_5 ? p12 : p11;
    wire lt_1_6_7 = (p20 < p21);
    wire [W-1:0] lo_1_6 = lt_1_6_7 ? p20 : p21;
    wire [W-1:0] hi_1_7 = lt_1_6_7 ? p21 : p20;

    // ---- level 2 (4 个 compare-swap) ----
    wire lt_2_0_2 = (lo_1_0 < lo_1_2);
    wire [W-1:0] lo_2_0 = lt_2_0_2 ? lo_1_0 : lo_1_2;
    wire [W-1:0] hi_2_2 = lt_2_0_2 ? lo_1_2 : lo_1_0;
    wire lt_2_1_3 = (hi_1_1 < hi_1_3);
    wire [W-1:0] lo_2_1 = lt_2_1_3 ? hi_1_1 : hi_1_3;
    wire [W-1:0] hi_2_3 = lt_2_1_3 ? hi_1_3 : hi_1_1;
    wire lt_2_4_6 = (lo_1_4 < lo_1_6);
    wire [W-1:0] lo_2_4 = lt_2_4_6 ? lo_1_4 : lo_1_6;
    wire [W-1:0] hi_2_6 = lt_2_4_6 ? lo_1_6 : lo_1_4;
    wire lt_2_5_7 = (hi_1_5 < hi_1_7);
    wire [W-1:0] lo_2_5 = lt_2_5_7 ? hi_1_5 : hi_1_7;
    wire [W-1:0] hi_2_7 = lt_2_5_7 ? hi_1_7 : hi_1_5;

    // ---- level 3 (4 个 compare-swap) ----
    wire lt_3_1_2 = (lo_2_1 < hi_2_2);
    wire [W-1:0] lo_3_1 = lt_3_1_2 ? lo_2_1 : hi_2_2;
    wire [W-1:0] hi_3_2 = lt_3_1_2 ? hi_2_2 : lo_2_1;
    wire lt_3_5_6 = (lo_2_5 < hi_2_6);
    wire [W-1:0] lo_3_5 = lt_3_5_6 ? lo_2_5 : hi_2_6;
    wire [W-1:0] hi_3_6 = lt_3_5_6 ? hi_2_6 : lo_2_5;
    wire lt_3_0_4 = (lo_2_0 < lo_2_4);
    wire [W-1:0] lo_3_0 = lt_3_0_4 ? lo_2_0 : lo_2_4;
    wire [W-1:0] hi_3_4 = lt_3_0_4 ? lo_2_4 : lo_2_0;
    wire lt_3_3_7 = (hi_2_3 < hi_2_7);
    wire [W-1:0] lo_3_3 = lt_3_3_7 ? hi_2_3 : hi_2_7;
    wire [W-1:0] hi_3_7 = lt_3_3_7 ? hi_2_7 : hi_2_3;
    // >>> 建议在此插一级寄存器：把上面 9 根线全部打一拍 <<<

    // ---- level 4 (3 个 compare-swap) ----
    wire lt_4_2_6 = (hi_3_2 < hi_3_6);
    wire [W-1:0] lo_4_2 = lt_4_2_6 ? hi_3_2 : hi_3_6;
    wire [W-1:0] hi_4_6 = lt_4_2_6 ? hi_3_6 : hi_3_2;
    wire lt_4_1_5 = (lo_3_1 < lo_3_5);
    wire [W-1:0] lo_4_1 = lt_4_1_5 ? lo_3_1 : lo_3_5;
    wire [W-1:0] hi_4_5 = lt_4_1_5 ? lo_3_5 : lo_3_1;
    wire lt_4_0_8 = (lo_3_0 < p22);
    wire [W-1:0] lo_4_0 = lt_4_0_8 ? lo_3_0 : p22;
    wire [W-1:0] hi_4_8 = lt_4_0_8 ? p22 : lo_3_0;

    // ---- level 5 (2 个 compare-swap) ----
    wire lt_5_2_4 = (lo_4_2 < hi_3_4);
    wire [W-1:0] lo_5_2 = lt_5_2_4 ? lo_4_2 : hi_3_4;
    wire [W-1:0] hi_5_4 = lt_5_2_4 ? hi_3_4 : lo_4_2;
    wire lt_5_3_5 = (lo_3_3 < hi_4_5);
    wire [W-1:0] lo_5_3 = lt_5_3_5 ? lo_3_3 : hi_4_5;
    wire [W-1:0] hi_5_5 = lt_5_3_5 ? hi_4_5 : lo_3_3;

    // ---- level 6 (3 个 compare-swap) ----
    wire lt_6_1_2 = (lo_4_1 < lo_5_2);
    wire [W-1:0] lo_6_1 = lt_6_1_2 ? lo_4_1 : lo_5_2;
    wire [W-1:0] hi_6_2 = lt_6_1_2 ? lo_5_2 : lo_4_1;
    wire lt_6_3_4 = (lo_5_3 < hi_5_4);
    wire [W-1:0] lo_6_3 = lt_6_3_4 ? lo_5_3 : hi_5_4;
    wire [W-1:0] hi_6_4 = lt_6_3_4 ? hi_5_4 : lo_5_3;
    wire lt_6_5_6 = (hi_5_5 < hi_4_6);
    wire [W-1:0] lo_6_5 = lt_6_5_6 ? hi_5_5 : hi_4_6;
    wire [W-1:0] hi_6_6 = lt_6_5_6 ? hi_4_6 : hi_5_5;
    // >>> 建议在此插一级寄存器：把上面 9 根线全部打一拍 <<<

    // ---- level 7 (2 个 compare-swap) ----
    wire lt_7_4_8 = (hi_6_4 < hi_4_8);
    wire [W-1:0] lo_7_4 = lt_7_4_8 ? hi_6_4 : hi_4_8;
    wire [W-1:0] hi_7_8 = lt_7_4_8 ? hi_4_8 : hi_6_4;
    wire lt_7_3_5 = (lo_6_3 < lo_6_5);
    wire [W-1:0] lo_7_3 = lt_7_3_5 ? lo_6_3 : lo_6_5;
    wire [W-1:0] hi_7_5 = lt_7_3_5 ? lo_6_5 : lo_6_3;

    // ---- level 8 (2 个 compare-swap) ----
    wire lt_8_2_4 = (hi_6_2 < lo_7_4);
    wire [W-1:0] lo_8_2 = lt_8_2_4 ? hi_6_2 : lo_7_4;
    wire [W-1:0] hi_8_4 = lt_8_2_4 ? lo_7_4 : hi_6_2;
    wire lt_8_6_8 = (hi_6_6 < hi_7_8);
    wire [W-1:0] lo_8_6 = lt_8_6_8 ? hi_6_6 : hi_7_8;
    wire [W-1:0] hi_8_8 = lt_8_6_8 ? hi_7_8 : hi_6_6;

    // ---- level 9 (4 个 compare-swap) ----
    wire lt_9_1_2 = (lo_6_1 < lo_8_2);
    wire [W-1:0] lo_9_1 = lt_9_1_2 ? lo_6_1 : lo_8_2;
    wire [W-1:0] hi_9_2 = lt_9_1_2 ? lo_8_2 : lo_6_1;
    wire lt_9_3_4 = (lo_7_3 < hi_8_4);
    wire [W-1:0] lo_9_3 = lt_9_3_4 ? lo_7_3 : hi_8_4;
    wire [W-1:0] hi_9_4 = lt_9_3_4 ? hi_8_4 : lo_7_3;
    wire lt_9_5_6 = (hi_7_5 < lo_8_6);
    wire [W-1:0] lo_9_5 = lt_9_5_6 ? hi_7_5 : lo_8_6;
    wire [W-1:0] hi_9_6 = lt_9_5_6 ? lo_8_6 : hi_7_5;
    wire lt_9_7_8 = (hi_3_7 < hi_8_8);
    wire [W-1:0] lo_9_7 = lt_9_7_8 ? hi_3_7 : hi_8_8;
    wire [W-1:0] hi_9_8 = lt_9_7_8 ? hi_8_8 : hi_3_7;

    assign med = hi_9_4;

endmodule
