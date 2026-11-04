//==========================================================================
// disp_prep.v  v4 — FPGA 端基础图像预处理（纯插入式，不改学长任何模块）
//--------------------------------------------------------------------------
// 赛题评分点："能进行基础图像处理，如灰度化、去噪、二值化、色彩空间转换"
// 位置：v_src_clk 域、video_crtl 之前，只处理 RGB565 像素流 + de/vs。
//
// 模式（3 位，顶层 parameter PREP_MODE 固化；运行时不依赖任何寄存器）：
//   0 原图      —— 顶层硬件旁路，本模块输出不被使用（与原设计 100% 等价）
//   1 灰度化    —— BT.601 Y=(77R+150G+29B)>>8
//   2 边缘      —— 水平梯度 |Y(x)-Y(x-1)| 二值（门限 THRESH_EDGE）
//   3 二值化    —— Y > THRESH_BIN → 白
//   4 去噪      —— 三抽头平滑（当前 + 前两个像素，R/G/B 各自平均）
//   5 色彩空间  —— RGB565 → YCbCr(BT.601)，伪彩显示 R=Cr G=Y B=Cb
//   6 去噪+二值 —— 三抽头平滑后再二值化（在 Y 上）
//   7 去噪+灰度 —— 三抽头平滑后再转灰度
//
// 实现要点：
//   * 全部延迟统一 1 拍（只对像素做 2 级移位寄存器，不依赖行缓存），
//     因此不存在行/帧对齐问题，也不会因存储器推断导致数据错位。
//   * de/vs 与像素同拍延迟，保证帧边界/行边界一致。
//   * 顶层 PREP_MODE 为常数时，综合器会裁掉不用的模式。
//==========================================================================
module disp_prep(
    input               pixclk          ,
    input               rst_n           ,
    input       [2:0]   mode            ,
    input       [15:0]  i_data          ,
    input               i_de            ,
    input               i_vs            ,
    output reg  [15:0]  o_data          ,
    output reg          o_de            ,
    output reg          o_vs            ,
    output reg  [10:0]  o_line_len
);

    parameter  [7:0] THRESH_EDGE = 8'd24    ;   // 水平梯度阈值
    parameter  [7:0] THRESH_BIN  = 8'd110   ;   // 二值化亮度阈值

    localparam [15:0] WHITE = 16'hFFFF ;
    localparam [15:0] BLACK = 16'h0000 ;

    //------------------------------------------------------------------
    // 像素移位寄存器（当前 + 前 2 个）
    //------------------------------------------------------------------
    reg  [15:0] s0, s1 ;
    reg         de_d1  ;
    reg         vs_d1  ;
    reg  [10:0] pix_cnt;

    wire [15:0] cur   = i_data ;
    wire [15:0] prev1 = s0 ;
    wire [15:0] prev2 = s1 ;

    //------------------------------------------------------------------
    // RGB565 → 8bit 分量
    //------------------------------------------------------------------
    function [7:0] f_r8; input [15:0] p; begin f_r8 = {p[15:11], p[15:13]}; end endfunction
    function [7:0] f_g8; input [15:0] p; begin f_g8 = {p[10:5],  p[10:9] }; end endfunction
    function [7:0] f_b8; input [15:0] p; begin f_b8 = {p[4:0],   p[4:2]  }; end endfunction

    // 亮度（BT.601 Q8）
    function [7:0] f_y;
        input [15:0] p;
        reg   [17:0] t;
        begin
            t   = f_r8(p) * 18'd77 + f_g8(p) * 18'd150 + f_b8(p) * 18'd29;
            f_y = t[15:8];
        end
    endfunction

    //------------------------------------------------------------------
    // 三抽头平滑：各通道 (a+b+c) * 21 >> 6（≈ /3）
    //------------------------------------------------------------------
    function [7:0] f_avg3;
        input [7:0] a, b, c;
        reg   [9:0] t;
        begin
            t      = a + b + c;
            f_avg3 = (t * 10'd21) >> 6;
        end
    endfunction

    wire [7:0] avg_r = f_avg3(f_r8(cur), f_r8(prev1), f_r8(prev2));
    wire [7:0] avg_g = f_avg3(f_g8(cur), f_g8(prev1), f_g8(prev2));
    wire [7:0] avg_b = f_avg3(f_b8(cur), f_b8(prev1), f_b8(prev2));

    wire [15:0] smooth_rgb = {avg_r[7:3], avg_g[7:2], avg_b[7:3]};

    wire [7:0] y_smooth = (avg_r * 18'd77 + avg_g * 18'd150 + avg_b * 18'd29) >> 8;

    //------------------------------------------------------------------
    // 单像素处理
    //------------------------------------------------------------------
    wire [7:0] y_cur = f_y(cur);
    wire [7:0] y_p1  = f_y(prev1);
    wire [8:0] dy    = (y_cur > y_p1) ? (y_cur - y_p1) : (y_p1 - y_cur);

    wire [15:0] gray_rgb = {y_cur[7:3], y_cur[7:2], y_cur[7:3]};
    wire [15:0] edge_rgb = (dy > {1'b0, THRESH_EDGE}) ? WHITE : BLACK;
    wire [15:0] bin_rgb  = (y_cur > THRESH_BIN) ? WHITE : BLACK;
    wire [15:0] smt_bin  = (y_smooth > THRESH_BIN) ? WHITE : BLACK;
    wire [15:0] smt_gray = {y_smooth[7:3], y_smooth[7:2], y_smooth[7:3]};

    //------------------------------------------------------------------
    // RGB → YCbCr（BT.601），伪彩显示 R=Cr G=Y B=Cb
    //------------------------------------------------------------------
    wire signed [19:0] ycc_y  = ( $signed({12'd0, f_r8(cur)}) * 20'sd77
                               + $signed({12'd0, f_g8(cur)}) * 20'sd150
                               + $signed({12'd0, f_b8(cur)}) * 20'sd29  ) >>> 8 ;
    wire signed [19:0] ycc_cb = (-$signed({12'd0, f_r8(cur)}) * 20'sd43
                                 -$signed({12'd0, f_g8(cur)}) * 20'sd85
                                 +$signed({12'd0, f_b8(cur)}) * 20'sd128
                                 + 20'sd32768 ) >>> 8 ;
    wire signed [19:0] ycc_cr = ( $signed({12'd0, f_r8(cur)}) * 20'sd128
                                 -$signed({12'd0, f_g8(cur)}) * 20'sd107
                                 -$signed({12'd0, f_b8(cur)}) * 20'sd21
                                 + 20'sd32768 ) >>> 8 ;

    function [7:0] f_clipneg;
        input signed [19:0] v;
        begin
            if (v < 0)             f_clipneg = 8'd0   ;
            else if (v > 20'sd255) f_clipneg = 8'd255 ;
            else                   f_clipneg = v[7:0];
        end
    endfunction

    wire [7:0] ycc_yy  = f_clipneg(ycc_y);
    wire [7:0] ycc_b   = f_clipneg(ycc_cb);
    wire [7:0] ycc_r   = f_clipneg(ycc_cr);
    wire [15:0] ycc_rgb = {ycc_r[7:3], ycc_yy[7:2], ycc_b[7:3]};

    //------------------------------------------------------------------
    // 主时序
    //------------------------------------------------------------------
    always @(posedge pixclk) begin
        if (!rst_n) begin
            s0 <= 16'd0; s1 <= 16'd0;
            de_d1 <= 1'b0; vs_d1 <= 1'b0;
            pix_cnt <= 11'd0;
            o_data <= 16'd0; o_de <= 1'b0; o_vs <= 1'b0; o_line_len <= 11'd0;
        end else begin
            s0    <= cur  ;
            s1    <= s0   ;
            de_d1 <= i_de ;
            vs_d1 <= i_vs ;

            if (i_de) pix_cnt <= pix_cnt + 11'd1 ;
            else begin
                o_line_len <= pix_cnt ;
                pix_cnt    <= 11'd0   ;
            end

            o_de <= i_de  ;   // 与 o_data 同拍：本拍送入的像素 -> 下一拍输出
            o_vs <= i_vs  ;

            if (i_de) begin
                case (mode)
                    3'd1 : o_data <= gray_rgb  ;
                    3'd2 : o_data <= edge_rgb  ;
                    3'd3 : o_data <= bin_rgb   ;
                    3'd4 : o_data <= smooth_rgb;
                    3'd5 : o_data <= ycc_rgb   ;
                    3'd6 : o_data <= smt_bin   ;
                    3'd7 : o_data <= smt_gray  ;
                    default: o_data <= cur     ;
                endcase
            end else begin
                o_data <= 16'd0 ;
            end
        end
    end

endmodule
