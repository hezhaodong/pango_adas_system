//==========================================================================
// dbg_hdmi.v — HDMI 输入通道观测 v2（纯只读测量，不改变任何数据通路）
//--------------------------------------------------------------------------
// 8 个 32 位快照，由 pio_crtl 在 pcie 时钟域读出：
//   0x0D0 = DE 高电平周期数
//   0x0D4 = DE 上升沿数（每行 1 次 → 窗口内行数）
//   0x0D8 = VS 上升沿数（窗口内帧数）
//   0x0DC = HS 上升沿数
//   0x0E0 = 最近一次 DE 上升沿时采到的 {r,g,b} 像素（24 位）
//   0x0E4 = 最近一次 VS 上升沿时采到的 {r,g,b} 像素（24 位）
//   0x0E8 = pixclk 域自由计数器（gray 码，>1 位安全跨域）；两次读之间的
//           差值 × 快照周期数即可算出像素时钟频率（无需参考时钟）
//   0x0EC = 相邻两次 VS 上升沿之间的 pixclk 周期数（= 行数 × 行周期）
// 快照窗口 = 2^WINDOW 个 pixclk_in 周期；74.25MHz 时 WINDOW=22 → 约 56.5ms。
// 设计要点：窗口很长（毫秒级），快照值在被 pcie 时钟域采样时已稳定很久，
//           因此跨域直接采样在本用途下是安全的；gray 计数器低位每秒变化
//           远小于采样间隔，误差可忽略。
//==========================================================================
module dbg_hdmi(
    input               pixclk      ,
    input               rst_n       ,
    input               de_in       ,
    input               vs_in       ,
    input               hs_in       ,
    input       [23:0]  data_in     ,
    output reg  [31:0]  o_de_high   ,
    output reg  [31:0]  o_de_rise   ,
    output reg  [31:0]  o_vs_rise   ,
    output reg  [31:0]  o_hs_rise   ,
    output reg  [31:0]  o_data_de   ,
    output reg  [31:0]  o_data_vs   ,
    output reg  [31:0]  o_pixclk_gry,
    output reg  [31:0]  o_vs_period
);

parameter WINDOW = 22;

reg  [23:0]  r_cnt      ;
reg  [31:0]  r_de_high  ;
reg  [31:0]  r_de_rise  ;
reg  [31:0]  r_vs_rise  ;
reg  [31:0]  r_hs_rise  ;
reg  [23:0]  r_data_de  ;
reg  [23:0]  r_data_vs  ;
reg  [31:0]  r_pixcnt   ;
reg  [31:0]  r_vs_last  ;
reg  [31:0]  r_vs_period;
reg          r_de_d     ;
reg          r_vs_d     ;
reg          r_hs_d     ;

wire [31:0] w_pixcnt_nx = r_pixcnt + 32'd1;
wire [31:0] w_gray_nx   = w_pixcnt_nx ^ (w_pixcnt_nx >> 1);
wire [31:0] w_vs_period_now = r_pixcnt - r_vs_last;

always @(posedge pixclk) begin
    if (!rst_n) begin
        r_cnt       <= 24'd0;
        r_de_high   <= 32'd0;
        r_de_rise   <= 32'd0;
        r_vs_rise   <= 32'd0;
        r_hs_rise   <= 32'd0;
        r_data_de   <= 24'd0;
        r_data_vs   <= 24'd0;
        r_pixcnt    <= 32'd0;
        r_vs_last   <= 32'd0;
        r_vs_period <= 32'd0;
        r_de_d      <= 1'b0 ;
        r_vs_d      <= 1'b0 ;
        r_hs_d      <= 1'b0 ;
        o_de_high   <= 32'd0;
        o_de_rise   <= 32'd0;
        o_vs_rise   <= 32'd0;
        o_hs_rise   <= 32'd0;
        o_data_de   <= 32'd0;
        o_data_vs   <= 32'd0;
        o_pixclk_gry<= 32'd0;
        o_vs_period <= 32'd0;
    end else begin
        r_de_d <= de_in;
        r_vs_d <= vs_in;
        r_hs_d <= hs_in;

        // 自由运行计数器（gray 码输出，跨域安全）
        r_pixcnt <= w_pixcnt_nx;

        if (de_in)
            r_de_high <= r_de_high + 32'd1;

        if (de_in & ~r_de_d) begin
            r_de_rise <= r_de_rise + 32'd1;
            r_data_de <= data_in;
        end

        if (hs_in & ~r_hs_d)
            r_hs_rise <= r_hs_rise + 32'd1;

        if (vs_in & ~r_vs_d) begin
            r_vs_rise   <= r_vs_rise + 32'd1;
            r_data_vs   <= data_in;
            r_vs_period <= w_vs_period_now;
            r_vs_last   <= r_pixcnt;
        end

        if (r_cnt == ((24'd1 << WINDOW) - 24'd1)) begin
            r_cnt        <= 24'd0;
            o_de_high    <= r_de_high;
            o_de_rise    <= r_de_rise;
            o_vs_rise    <= r_vs_rise;
            o_hs_rise    <= r_hs_rise;
            o_data_de    <= {8'd0, r_data_de};
            o_data_vs    <= {8'd0, r_data_vs};
            o_pixclk_gry <= w_gray_nx;
            o_vs_period  <= r_vs_period;
            r_de_high    <= 32'd0;
            r_de_rise    <= 32'd0;
            r_vs_rise    <= 32'd0;
            r_hs_rise    <= 32'd0;
        end else begin
            r_cnt <= r_cnt + 24'd1;
        end
    end
end

endmodule
