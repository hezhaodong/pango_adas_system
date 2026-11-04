`timescale 1ns / 1ps
`define UD #1

module hdmi_loop(
    input wire        sys_clk,     // input system clock 27MHz
    

    //pcie
    input                       button_rst_n    ,
    input                       perst_n         ,

    //ov5640

    //clk and rst
    input                       ref_clk_n       ,      //100 Mhz
    input                       ref_clk_p       ,      //100 Mhz
    //diff signals

    input           [1:0]       rxn             ,
    input           [1:0]       rxp             ,
    output  wire    [1:0]       txn             ,
    output  wire    [1:0]       txp             ,
    //LED signals
    output reg                  ref_led         ,
    output reg                  pclk_led        ,
    output wire                 smlh_link_up    ,
    output wire                 rdlh_link_up    ,


    ///0v5640摄像头输入
    output  [1:0]                        cmos_init_done      /*synthesis PAP_MARK_DEBUG="1"*/ ,//OV5640寄存器初始化完成
    //coms1	
    inout                                cmos1_scl            ,//cmos1 i2c 
    inout                                cmos1_sda            ,//cmos1 i2c 
    input                                cmos1_vsync  /* synthesis syn_keep = 1 */        ,//cmos1 vsync
    input                                cmos1_href     /* synthesis syn_keep = 1 */      ,//cmos1 hsync refrence,data valid
    input                                cmos1_pclk       /* synthesis syn_keep = 1 */    ,//cmos1 pxiel clock
    input   [7:0]                        cmos1_data        /* synthesis syn_keep = 1 */   ,//cmos1 data
    output                               cmos1_reset          ,//


    //hdmi
    output                      rstn_out        ,
    output                      iic_scl         ,
    inout                       iic_sda         , 
    output                      iic_tx_scl      ,
    inout                       iic_tx_sda      , 
    input                       pixclk_in       ,                            
    input                       vs_in           , 
    input                       hs_in           /* synthesis PAP_MARK_DEBUG="true" */, 
    input                       de_in           ,
    input     [7:0]             r_in            , 
    input     [7:0]             g_in            , 
    input     [7:0]             b_in            ,  

    output                      pixclk_out      ,                            
    output reg                  vs_out          , 
    output reg                  hs_out          , 
    output reg                  de_out          ,
    output reg    [7:0]         r_out           , 
    output reg    [7:0]         g_out           , 
    output reg    [7:0]         b_out           ,
    output                      led_int
);

//==========================================================================
//  输入源选择（2026-10-10 新增）
//--------------------------------------------------------------------------
//  VIDEO_SOURCE = 0 : OV5640 摄像头（默认，与原有行为完全一致）
//  VIDEO_SOURCE = 1 : HDMI 接收器（MS7200，走 pixclk_in 域）
//
//  为何用【编译期参数】而不是 0x300 运行时切换：
//    两路输入时钟域不同（摄像头 pclk_in_test1 vs HDMI pixclk_in），
//    运行时切换需要时钟 MUX 原语，且切换时有毛刺风险。
//    等拿到 HDMI 引脚约束、验证基本通路能出帧后，再升级为 0x300 驱动的
//    时钟+数据 MUX（对应学长 high_speed_test_ov_fmc 的多路输入）。
//==========================================================================
parameter VIDEO_SOURCE = 1;


reg [15:0]  rstn_1ms       ;
wire        pix_clk        ;
wire        cfg_clk        ;
wire        locked         ;
wire        pll_rst_n      ;
wire        clk_50m        ;
//产生148.5
hdmi_pll hdmi_pll_inst (
  .clkout0(pix_clk),    // output
  .lock(pll_rst_n),          // output
  .clkin1(sys_clk),      // input
  .rst(1'b0)             // input
);


PLL_50mHZ  U_PLL(
  .clkout0(cfg_clk),    // output
  .clkout1(clk_50m),    // output
  .lock(locked),          // output
  .clkin1(sys_clk)       // input//10MHz
);


ms72xx_ctl ms72xx_ctl(
    .clk         (  cfg_clk    ), //input       clk,
    .rst_n       (  rstn_out   ), //input       rstn,
                            
    .init_over   (  init_over  ), //output      init_over,
    .iic_tx_scl  (  iic_tx_scl ), //output      iic_scl,
    .iic_tx_sda  (  iic_tx_sda ), //inout       iic_sda
    .iic_scl     (  iic_scl    ), //output      iic_scl,
    .iic_sda     (  iic_sda    )  //inout       iic_sda
);

assign    led_int  =  init_over; 

always @(posedge cfg_clk)
begin
	if(!locked)
	    rstn_1ms <= 16'd0;
	else
	begin
		if(rstn_1ms == 16'h2710)
		    rstn_1ms <= rstn_1ms;
		else
		    rstn_1ms <= rstn_1ms + 1'b1;
	end
end

assign rstn_out = (rstn_1ms == 16'h2710);




//-----------------------------------ov5640驱动-----------------------------------



wire    pclk_in_test1  /* synthesis PAP_MARK_DEBUG="true" */    ;
wire    vs_in_test1    /* synthesis PAP_MARK_DEBUG="true" */    ;
wire    data_de        /* synthesis PAP_MARK_DEBUG="true" */    ;
wire    de_in_test1    /* synthesis PAP_MARK_DEBUG="true" */    ;
wire    [15:0]        i_rgb5651    /* synthesis PAP_MARK_DEBUG="true" */    ;
ov5640 ov5640_inst(
    .video_clk    (),
    .clk_50m      (clk_50m ),
    .rst_n        (1),
    
    .ov_rst_n     (cmos_init_done[0]),
    
    .cam_pclk     (cmos1_pclk),
    .cam_vsync    (cmos1_vsync),
    .cam_href     (cmos1_href),
    .cam_data     (cmos1_data),
    .cam_rst_n    (cmos1_reset),
    .cam_pwdn     (),
    .cam_scl      (cmos1_scl),
    .cam_sda      (cmos1_sda),
    .clk_div2     (pclk_in_test1),
    .video_vsync  (vs_in_test1),
    .video_hsync  (de_in_test1),
    .video_valid  (data_de    ),
    .video_data   (i_rgb5651)
);



//==========================================================================
//  图像处理流水线（FPGA 侧道路环境感知）—— 赛题基础任务 3
//--------------------------------------------------------------------------
//  包含：灰度化 / 3x3 窗口 / 高斯去噪 / Sobel 边缘 / 二值化 /
//        红绿灯 ROI 颜色统计 / 帧差运动检测
//
//  时钟域：cmos1_pclk（与 i_rgb5651 同域，由 cmos_capture_data 输出）
//  复位  ：rstn_out 同步 2 拍到像素时钟域
//
//  输出说明（后续可接到 PCIe 寄存器 / LED / 图像叠加）：
//    img_edge_mag    梯度幅值（可当灰度图显示）
//    img_edge_bin    二值边缘
//    img_light       红绿灯判定：0=无 1=红 2=绿 3=黄
//    img_motion_any  本帧是否有运动
//==========================================================================
wire [15:0] img_red_cnt   /* synthesis syn_keep = 1 */ ;
wire [15:0] img_grn_cnt   /* synthesis syn_keep = 1 */ ;
wire [15:0] img_tot_cnt   /* synthesis syn_keep = 1 */ ;
wire [1:0]  img_light     /* synthesis syn_keep = 1 */ ;
wire [7:0]  img_edge_mag  /* synthesis syn_keep = 1 */ ;
wire        img_edge_bin  /* synthesis syn_keep = 1 */ ;
wire [7:0]  img_gray      /* synthesis syn_keep = 1 */ ;
wire [7:0]  img_blur      /* synthesis syn_keep = 1 */ ;
wire        img_motion_any /* synthesis syn_keep = 1 */ ;
wire [15:0] img_motion_cnt /* synthesis syn_keep = 1 */ ;

wire        img_gray_de, img_blur_de, img_edge_de, img_bin_de, img_bin;

// ---- 复位同步到像素时钟域（异步复位、同步释放的简化版）----
reg img_rstn_d1, img_rstn_d2;
always @(posedge cmos1_pclk) begin
    img_rstn_d1 <= rstn_out;
    img_rstn_d2 <= img_rstn_d1;
end

img_proc_pipeline #(
    .IMG_W        (1280),
    .IMG_H        (720),
    .SOBEL_THRESH (13'd256),
    .BIN_THRESH   (8'd128),
    .DIFF_THRESH  (8'd24),
    .BLOCK_LOG2   (4)
) u_img_proc (
    .i_clk    (cmos1_pclk),
    .i_rstn   (img_rstn_d2),
    .i_de     (data_de),
    .i_vsync  (vs_in_test1),
    .i_rgb565 (i_rgb5651),

    .o_gray_de    (img_gray_de),
    .o_gray       (img_gray),

    .o_blur_de    (img_blur_de),
    .o_blur       (img_blur),

    .o_edge_de    (img_edge_de),
    .o_edge_mag   (img_edge_mag),
    .o_edge_bin   (img_edge_bin),

    .o_bin_de     (img_bin_de),
    .o_bin        (img_bin),

    .o_red_cnt    (img_red_cnt),
    .o_green_cnt  (img_grn_cnt),
    .o_total_cnt  (img_tot_cnt),
    .o_light      (img_light),

    .o_motion_any (img_motion_any),
    .o_motion_cnt (img_motion_cnt)
);

//==========================================================================
//  TODO：把 FPGA 感知结果送出 FPGA（三选一或都做）
//
//  方案 A（推荐，改动小）：在 pio_crtl.v 里加只读寄存器，
//          把 img_light / img_red_cnt / img_motion_cnt 暴露给 ARM 读
//  方案 B（演示直观）：把 img_edge_mag 拼成 RGB565 送到 video_crtl，
//          屏幕上直接看到 FPGA 处理的边缘图
//  方案 C（最快验证）：接 LED
//          assign led_int = (img_light == 2'd1);   // 检测到红灯亮灯
//==========================================================================

//==========================================================================
//  显示数据源选择（方案 B：把 FPGA 边缘图送屏幕）
//  默认 mode=0 → 原样输出摄像头视频，完全不影响原有链路；
//  写 pio_crtl 的 0x20C 寄存器 bit0=1 → 输出 FPGA 算出的 Sobel 边缘图
//==========================================================================
wire [1:0]  disp_mode_w    /* synthesis syn_keep = 1 */ ;   // 来自 pio_crtl

// 8 位灰度 → RGB565 灰阶（R5/G6/B5 取灰度的高位）
wire [15:0] img_edge_rgb565 = {img_edge_mag[7:3], img_edge_mag[7:2], img_edge_mag[7:3]};

wire [15:0] video_src_data = disp_mode_w[0] ? img_edge_rgb565 : i_rgb5651;



//-----------------------------------数据打拍-----------------------------------
reg    [23:0]    r_hdmi_data_d0    ;
reg    [23:0]    r_hdmi_data_d1    ;
reg              r_hdmi_de_d0      ;
reg              r_hdmi_de_d1      ;
reg              r_hdmi_vs_d0      ;
reg              r_hdmi_vs_d1      ;

always@(posedge pixclk_in)    begin

    r_hdmi_data_d0    <=    {r_in,g_in,b_in}    ;
    r_hdmi_data_d1    <=    r_hdmi_data_d0      ;
    r_hdmi_de_d0      <=    de_in               ;
    r_hdmi_de_d1      <=    r_hdmi_de_d0        ;
    r_hdmi_vs_d0      <=    vs_in               ;
    r_hdmi_vs_d1      <=    r_hdmi_vs_d0        ;

end


//-----------------------------------生成测试数据-----------------------------------
wire              video_gen_vs      ;
wire              video_gen_de      ;
wire              video_gen_hs      ;
wire    [23:0]    video_gen_data    ;
wire    [15:0]    rgb565_data       ;
video_gen_data#(
    .H_TOTAL       ( 1920 ),
    .V_TOTAL       ( 1080 ),
    .PPC           ( 1 ),
    .BPC           ( 8 )
)u_video_gen_data(
    .video_clk     ( pix_clk      ),
    .rst_n         ( pll_rst_n      ),
    .video_gen_vs  ( video_gen_vs  ),
    .video_gen_hs  ( video_gen_hs  ),
    .video_gen_de  ( video_gen_de  ),
    .video_gen_data  ( video_gen_data  )
);

assign rgb565_data = {video_gen_data[23:19],video_gen_data[15:10],video_gen_data[7:3]};


//-----------------------------------数据打包-----------------------------------
parameter	IMG_WIDTH	=	1920	;
parameter	IMG_HEIGHT	=	1080    	;
parameter	PPC			=	1		;
parameter	PIXCEL_BYTES=	2		;	
parameter	DMA_LEN		=	3840	;	//字节
parameter	IMG_SIZE	=	IMG_WIDTH*IMG_HEIGHT*PIXCEL_BYTES;
parameter	SEND_TIMES	=	IMG_SIZE/DMA_LEN	;

wire              w_video_crtl_de    /* synthesis PAP_MARK_DEBUG="true" */;
wire              w_video_crtl_vs    ;
wire    [23:0]    w_video_crtl_data  /* synthesis PAP_MARK_DEBUG="true" */;
wire    [15:0]    rgb_565_data       ;
wire              w_start_flag       /* synthesis PAP_MARK_DEBUG="true" */;
wire              w_dma_rd_en        /* synthesis PAP_MARK_DEBUG="true" */;
wire    [127:0]   w_dma_rd_data      ;

wire                  video_crtl_vs      ;   
wire                  video_crtl_de      /* synthesis PAP_MARK_DEBUG="true" */; 
wire				  dma_tx_done		 /* synthesis PAP_MARK_DEBUG="true" */;  

//==========================================================================
//  HDMI 输入通路（2026-10-10）：RGB888 -> RGB565 + 输入源选择
//--------------------------------------------------------------------------
//  r_hdmi_data_d1 / r_hdmi_de_d1 / r_hdmi_vs_d1 是 pixclk_in 域同步两拍后的
//  HDMI 数据（见下方"数据打拍"段）。与摄像头时钟域不同，故用 VIDEO_SOURCE
//  编译期选择。RGB565 取 RGB888 的高位：R[7:3] G[7:2] B[7:3]。
//==========================================================================
wire [15:0] hdmi_rgb565 = {r_hdmi_data_d1[23:19], r_hdmi_data_d1[15:10], r_hdmi_data_d1[7:3]};

wire        v_src_clk  = (VIDEO_SOURCE == 0) ? pclk_in_test1 : pixclk_in;
wire [15:0] v_src_data = (VIDEO_SOURCE == 0) ? video_src_data : hdmi_rgb565;
wire        v_src_vs   = (VIDEO_SOURCE == 0) ? vs_in_test1   : r_hdmi_vs_d1;
wire        v_src_de   = (VIDEO_SOURCE == 0) ? de_in_test1   : r_hdmi_de_d1;

video_crtl#(
	.PPC				  ( PPC	),
    .DATA_WIDTH           ( 16  ),
    .IMG_WIDTH            ( IMG_WIDTH ),
    .IMG_HEIGHT           ( IMG_HEIGHT )
)u_video_crtl(
    .i_video_clk          ( v_src_clk           ),
    .i_rst_n              ( pll_rst_n           ),
    .i_start_dma_tx_flag  ( w_start_flag        ),
    .i_video_data         ( v_src_data          ),
    .i_video_vs           ( v_src_vs            ),
    .i_video_de           ( v_src_de            ),
    .o_video_crtl_data    ( w_video_crtl_data   ),
    .o_video_crtl_vs      ( w_video_crtl_vs     ),
    .o_video_crtl_de      ( w_video_crtl_de     )
);

//vs复位fifo 避免错位
reg             r_video_crtl_vs_d0     ;
reg    [7:0]    r_vs_ext_cnt           ;      //扩展
reg             r_vs_rst               ;      //复位
reg             r_vs_en                ;
always@(posedge pixclk_in)    begin

    r_video_crtl_vs_d0    <=    w_video_crtl_vs;
end


always@(posedge pixclk_in)    begin
    if(!rstn_out)    
        r_vs_en    <=    1'd0    ;
    else    if(r_vs_ext_cnt == 20 && r_vs_en)
        r_vs_en    <=    1'd0    ;
    else    if(w_video_crtl_vs && ~r_video_crtl_vs_d0)
        r_vs_en    <=    1'd1    ;
    else
        r_vs_en    <=    r_vs_en ;
end


always@(posedge pixclk_in)    begin
    if(!rstn_out)    begin
        r_vs_ext_cnt    <=    'd0    ;
        r_vs_rst        <=    'd0    ;
    end
    else    if(r_vs_ext_cnt == 20 && r_vs_en)    begin
        r_vs_ext_cnt    <=    'd0    ;
        r_vs_rst        <=    'd0    ;
    end
    else    if(r_vs_en)    begin
        r_vs_ext_cnt    <=    r_vs_ext_cnt + 1'b1    ;
        r_vs_rst        <=    1'd1                   ;
    end
    else    begin
        r_vs_ext_cnt    <=    r_vs_ext_cnt    ;
        r_vs_rst        <=    r_vs_rst    ;
    end
end

assign rgb_565_data = {w_video_crtl_data[23:19],w_video_crtl_data[15:10],w_video_crtl_data[7:3]};

reg [15:0]		wr_cnt			/* synthesis PAP_MARK_DEBUG="true" */;
reg	[15:0]		rd_cnt			/* synthesis PAP_MARK_DEBUG="true" */;
reg             r_line_reg      /* synthesis PAP_MARK_DEBUG="true" */;

//hdmi时钟下
reg				r_video_dma_req_pix_d0	/* synthesis PAP_MARK_DEBUG="true" */;
reg				r_video_dma_req_pix_d1	/* synthesis PAP_MARK_DEBUG="true" */;
reg				r_video_dma_req_pix_d2	/* synthesis PAP_MARK_DEBUG="true" */;

//pcie时钟下
//reg             video_dma_req   /* synthesis PAP_MARK_DEBUG="true" */;
wire            video_dma_req       ;
reg             video_dma_req_d0    ;
reg				r_line_req_d0	/* synthesis PAP_MARK_DEBUG="true" */;
reg				r_line_req_d1	/* synthesis PAP_MARK_DEBUG="true" */;
reg				r_line_req_d2	/* synthesis PAP_MARK_DEBUG="true" */;
reg             r_frame_done_d0    ;
reg             r_frame_done_d1    ;
reg             r_frame_done_d2    ;
reg             frame_done         /* synthesis PAP_MARK_DEBUG="true" */;
reg    [15:0]   frame_cnt          /* synthesis PAP_MARK_DEBUG="true" */;
//reg    [63:0]   video_dma_addr     /* synthesis PAP_MARK_DEBUG="true" */;
wire   [63:0]   video_dma_addr     /* synthesis PAP_MARK_DEBUG="true" */;
reg    [63:0]   video_ch0_base_addr/* synthesis PAP_MARK_DEBUG="true" */;
reg    [1:0]    r_wr_index         /* synthesis PAP_MARK_DEBUG="true" */;
reg    [1:0]    r_wr_index_d0      /* synthesis PAP_MARK_DEBUG="true" */;
wire            dma_cmd_rdy        ;
wire            dma_tx_done        ;
wire   [9:0]    video_dma_len      ; 
wire            set_dma_config_en  ;
wire   [63:0]   ch0_dma_base_addr  ;
wire   [63:0]   ch0_dma_base_addr2 ;
wire   [63:0]   ch0_dma_base_addr3 ;
wire   [63:0]   ch0_dma_base_addr4 ;

//hdmi时钟下
always@(posedge    pixclk_in)    begin
	r_video_dma_req_pix_d0	<=	video_dma_req	;
	r_video_dma_req_pix_d1	<=	r_video_dma_req_pix_d0	;
	r_video_dma_req_pix_d2	<=	r_video_dma_req_pix_d1	;
end


//pcie时钟下
always@(posedge    pclk_div2)    begin

	video_dma_req_d0	    <=	video_dma_req;
	r_line_req_d0		    <=	r_line_reg	;
	r_line_req_d1		    <=	r_line_req_d0	;
	r_line_req_d2		    <=	r_line_req_d1	;

	r_frame_done_d0			<=	frame_done	;
	r_frame_done_d1			<=	r_frame_done_d0	;
	r_frame_done_d2			<=	r_frame_done_d1	;

end


//是否发送一帧
always@(posedge    pclk_div2)    begin
    if(!core_rst_n || !w_start_flag || r_vs_rst)
		frame_cnt	<=	'd0	;
	else	if(w_start_flag==0)
		frame_cnt	<=	'd0;
	else	if(dma_tx_done && frame_cnt == SEND_TIMES-1)	//一帧
		frame_cnt	<=	'd0	;
	else	if(dma_tx_done)	//一次是3840字节  
		frame_cnt	<=	frame_cnt + 1'b1	;
	else
		frame_cnt	<=	frame_cnt;
end

//一帧发送完成 发送标志信号
always@(posedge    pclk_div2)    begin
    if(!core_rst_n)
		frame_done	<=	'd0;
	else	if(w_start_flag==0)
		frame_done	<=	'd0;
	else	if(dma_tx_done && frame_cnt == SEND_TIMES-1)	//一帧
		frame_done	<=	'd1;
	else
		frame_done	<=	'd0;

end


//帧空间切换
always@(posedge    pclk_div2)    begin
    if(!core_rst_n || !w_start_flag)
        r_wr_index    <=    'd0    ;
    else    if(frame_done)
        r_wr_index    <=    r_wr_index + 1'b1;
    else
        r_wr_index    <=    r_wr_index    ;
end

//寄存写完的帧号
always@(posedge    pclk_div2)    begin
    if(!core_rst_n || !w_start_flag)
        r_wr_index_d0    <=    'd0    ;
    else    if(frame_done)
        r_wr_index_d0    <=    r_wr_index    ;
    else
        r_wr_index_d0    <=    r_wr_index_d0    ;       
end





assign video_dma_len = 640    ;    //dw





//----------------------------------------------------------tx fun ----------------------------------------------------------
wire    [31:0]    o_check_data    ;
wire    [1:0]     o_dma_wr_index  ;
wire              o_dma_wr_done   ;
pcie_tx_fun#(
    .IMG_WIDTH        ( 1280 ),
    .IMG_HEIGHT       ( 720  ),
    .DMA_ADDR_WIDTH   ( 64   ),
    .VIDEO_DATA_WIDTH ( 16   ),
    .PCIE_DATA_WIDTH  ( 128  ),
    .PPC              ( 1    ),
    .PIXCEL_BYTES     ( 2    ),
    .DMA_LEN          ( 2560 )
)u_pcie_tx_fun(
    .i_pcie_clk       ( pclk_div2         ),
    .i_pcie_rst_n     ( core_rst_n        ),
    .i_dma_base_addr  ( ch0_dma_base_addr ),
    .i_dma_base_addr2 ( ch0_dma_base_addr2),
    .i_dma_base_addr3 ( ch0_dma_base_addr3),
    .i_dma_base_addr4 ( ch0_dma_base_addr4),
    .i_dma_len        ( 3840              ),
    .i_dma_set_en     ( set_dma_config_en ),
    .o_dma_wr_index   ( o_dma_wr_index    ),
    .o_dma_wr_done    ( o_dma_wr_done     ),
    .i_start_tx_flag  ( w_start_flag      ),
    .o_check_data     ( o_check_data      ),
    .i_dma_cmd_rdy    ( dma_cmd_rdy       ),
    .o_dma_req        ( video_dma_req     ),
    .o_dma_addr       ( video_dma_addr    ),
    .o_dma_len        (         ),        
    .i_dma_tx_done    ( dma_tx_done       ),
    .i_video_clk      ( v_src_clk         ),
    .i_video_rst_n    ( pll_rst_n         ),
    .i_video_data     ( w_video_crtl_data ),
    .i_video_vs       ( w_video_crtl_vs   ),
    .i_video_de       ( w_video_crtl_de   ),
    .o_dma_rd_data    ( w_dma_rd_data     ),
    .i_dma_rd_en      ( w_dma_rd_en       )
);



//----------------------------------------------------------rst debounce ----------------------------------------------------------
localparam  DEVICE_TYPE   = 3'b000  ;//@IPC enum 3'b000,3'b001,3'b100
localparam  AXIS_SLAVE_NUM = 3      ;  //@IPC enum 1 2 3
//TEST UNIT MODE SIGNALS
wire            pcie_cfg_ctrl_en        ;
wire            axis_master_tready_cfg  ;

wire            cfg_axis_slave0_tvalid  ;
wire    [127:0] cfg_axis_slave0_tdata   ;
wire            cfg_axis_slave0_tlast   ;
wire            cfg_axis_slave0_tuser   ;

//for mux
wire            axis_master_tready_mem  ;
wire            axis_master_tvalid_mem  ;
wire    [127:0] axis_master_tdata_mem   ;
wire    [3:0]   axis_master_tkeep_mem   ;
wire            axis_master_tlast_mem   ;
wire    [7:0]   axis_master_tuser_mem   ;

wire            cross_4kb_boundary      ;

wire            dma_axis_slave0_tvalid  ;
wire    [127:0] dma_axis_slave0_tdata   ;
wire            dma_axis_slave0_tlast   ;
wire            dma_axis_slave0_tuser   ;

//RESET DEBOUNCE and SYNC
wire            sync_button_rst_n       ;
wire            s_pclk_rstn             ;
wire            s_pclk_div2_rstn        ;

//********************** internal signal
//clk and rst
wire            pclk_div2               ;
wire            pclk                    ;
wire            ref_clk                 ;
wire            core_rst_n              ;
//AXIS master interface
wire            axis_master_tvalid      ;
wire            axis_master_tready      ;
wire    [127:0] axis_master_tdata       ;
wire    [3:0]   axis_master_tkeep       ;
wire            axis_master_tlast       ;
wire    [7:0]   axis_master_tuser       ;

//axis slave 0 interface
wire            axis_slave0_tready      ;
wire            axis_slave0_tvalid      ;
wire    [127:0] axis_slave0_tdata       ;
wire            axis_slave0_tlast       ;
wire            axis_slave0_tuser       ;

//axis slave 1 interface
wire            axis_slave1_tready      ;
wire            axis_slave1_tvalid      ;
wire    [127:0] axis_slave1_tdata       ;
wire            axis_slave1_tlast       ;
wire            axis_slave1_tuser       ;

//axis slave 2 interface
wire            axis_slave2_tready      ;
wire            axis_slave2_tvalid      ;
wire    [127:0] axis_slave2_tdata       ;
wire            axis_slave2_tlast       ;
wire            axis_slave2_tuser       ;

wire    [7:0]   cfg_pbus_num            ;
wire    [4:0]   cfg_pbus_dev_num        ;
wire    [2:0]   cfg_max_rd_req_size     ;
wire    [2:0]   cfg_max_payload_size    ;
wire            cfg_rcb                 ;

wire            cfg_ido_req_en          ;
wire            cfg_ido_cpl_en          ;
wire    [7:0]   xadm_ph_cdts            ;
wire    [11:0]  xadm_pd_cdts            ;
wire    [7:0]   xadm_nph_cdts           ;
wire    [11:0]  xadm_npd_cdts           ;
wire    [7:0]   xadm_cplh_cdts          ;
wire    [11:0]  xadm_cpld_cdts          ;

//system signal
wire    [4:0]   smlh_ltssm_state        ;

// led lights up
reg     [22:0]  ref_led_cnt             ;
reg     [26:0]  pclk_led_cnt            ;

//uart2apb 32bits
wire            uart_p_sel              ;
wire    [3:0]   uart_p_strb             ;
wire    [15:0]  uart_p_addr             ;
wire    [31:0]  uart_p_wdata            ;
wire            uart_p_ce               ;
wire            uart_p_we               ;
wire            uart_p_rdy              ;
wire    [31:0]  uart_p_rdata            ;

//apb
wire    [3:0]   p_strb                  ;
wire    [15:0]  p_addr                  ;
wire    [31:0]  p_wdata                 ;
wire            p_ce                    ;
wire            p_we                    ;

//apb mux
wire            p_sel_pcie              ;       //0~5:hsstlp 6:Reserved 7:pcie
wire            p_sel_cfg               ;       //8: cfg
wire            p_sel_dma               ;       //9: dma

wire    [31:0]  p_rdata_pcie            ;       //0~5:hsstlp 6:Reserved 7:pcie
wire    [31:0]  p_rdata_cfg             ;       //8: cfg
wire    [31:0]  p_rdata_dma             ;       //9: dma

wire            p_rdy_pcie              ;       //0~5:hsstlp 6:Reserved 7:pcie
wire            p_rdy_cfg               ;       //8: cfg
wire            p_rdy_dma               ;       //9: dma

wire            start_flag              ;       //开始采集标志

assign cfg_ido_req_en   =   1'b0;
assign cfg_ido_cpl_en   =   1'b0;
assign xadm_ph_cdts     =   8'b0;
assign xadm_pd_cdts     =   12'b0;
assign xadm_nph_cdts    =   8'b0;
assign xadm_npd_cdts    =   12'b0;
assign xadm_cplh_cdts   =   8'b0;
assign xadm_cpld_cdts   =   12'b0;
//ASYNC RST  define IPS2L_PCIE_SPEEDUP_SIM when simulation
hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_buttonrstn_debounce(
    .clk                (ref_clk            ),
    .rstn_in            (button_rst_n       ),
    .rstn_out           (sync_button_rst_n  )
);

hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_perstn_debounce(
    .clk                (ref_clk            ),
    .rstn_in            (perst_n            ),
    .rstn_out           (sync_perst_n       )
);

hsst_rst_sync_v1_0  u_ref_core_rstn_sync    (
    .clk                (ref_clk            ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (ref_core_rst_n     )
);

hsst_rst_sync_v1_0  u_pclk_core_rstn_sync   (
    .clk                (pclk               ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (s_pclk_rstn        )
);


always @(posedge ref_clk or negedge sync_perst_n)
begin
    if (!sync_perst_n)
        ref_led_cnt    <= 23'd0;
    else
        ref_led_cnt    <= ref_led_cnt + 23'd1;
end

always @(posedge ref_clk or negedge sync_perst_n)
begin
    if (!sync_perst_n)
        ref_led        <= 1'b1;
    else if(&ref_led_cnt)
        ref_led        <= ~ref_led;
end

always @(posedge pclk or negedge s_pclk_rstn)
begin
    if (!s_pclk_rstn)
        pclk_led_cnt    <= 27'd0;
    else
        pclk_led_cnt    <= pclk_led_cnt + 27'd1;
end

always @(posedge pclk or negedge s_pclk_rstn)
begin
    if (!s_pclk_rstn)
        pclk_led        <= 1'b1;
    else if(&pclk_led_cnt)
        pclk_led        <= ~pclk_led;
end


//----------------------------------------------------------   pcie pio  ----------------------------------------------------------
wire            pio_wr_en      ;
wire    [9:0]   pio_wr_addr    ;
wire    [31:0]  pio_wr_data    ;

wire            pio_rd_en      ;
wire    [9:0]   pio_rd_addr    ;
wire    [31:0]  pio_rd_data    ;            
//==========================================================================
//  图像处理结果 → PCIe 时钟域（准静态信号，3 级同步器）
//  这些值在 cmos1_pclk 域每帧锁存一次，稳定保持约 16ms，
//  远长于同步器建立时间，因此 3 级同步采样是安全的。
//==========================================================================
reg [15:0] img_red_s1, img_red_s2, img_red_s3;
reg [15:0] img_grn_s1, img_grn_s2, img_grn_s3;
reg [15:0] img_tot_s1, img_tot_s2, img_tot_s3;
reg [15:0] img_mot_s1, img_mot_s2, img_mot_s3;
reg [1:0]  img_lit_s1, img_lit_s2, img_lit_s3;
reg        img_mna_s1, img_mna_s2, img_mna_s3;

always @(posedge pclk_div2) begin
    if (!core_rst_n) begin
        img_red_s1 <= 16'd0; img_red_s2 <= 16'd0; img_red_s3 <= 16'd0;
        img_grn_s1 <= 16'd0; img_grn_s2 <= 16'd0; img_grn_s3 <= 16'd0;
        img_tot_s1 <= 16'd0; img_tot_s2 <= 16'd0; img_tot_s3 <= 16'd0;
        img_mot_s1 <= 16'd0; img_mot_s2 <= 16'd0; img_mot_s3 <= 16'd0;
        img_lit_s1 <= 2'd0;  img_lit_s2 <= 2'd0;  img_lit_s3 <= 2'd0;
        img_mna_s1 <= 1'b0;  img_mna_s2 <= 1'b0;  img_mna_s3 <= 1'b0;
    end else begin
        img_red_s1 <= img_red_cnt;    img_red_s2 <= img_red_s1;    img_red_s3 <= img_red_s2;
        img_grn_s1 <= img_grn_cnt;    img_grn_s2 <= img_grn_s1;    img_grn_s3 <= img_grn_s2;
        img_tot_s1 <= img_tot_cnt;    img_tot_s2 <= img_tot_s1;    img_tot_s3 <= img_tot_s2;
        img_mot_s1 <= img_motion_cnt; img_mot_s2 <= img_mot_s1;    img_mot_s3 <= img_mot_s2;
        img_lit_s1 <= img_light;      img_lit_s2 <= img_lit_s1;    img_lit_s3 <= img_lit_s2;
        img_mna_s1 <= img_motion_any; img_mna_s2 <= img_mna_s1;    img_mna_s3 <= img_mna_s2;
    end
end

pio_crtl u_pio_crtl(
    .pcie_clk              ( pclk_div2         ),
    .rst_n                 ( core_rst_n        ),
    .start_flag            ( w_start_flag      ),
    .set_dma_config_en     ( set_dma_config_en ),
    .o_ch0_base_addr       ( ch0_dma_base_addr ),
    .o_ch0_base_addr2      ( ch0_dma_base_addr2),
    .o_ch0_base_addr3      ( ch0_dma_base_addr3),
    .o_ch0_base_addr4      ( ch0_dma_base_addr4),
    .i_wr_frame_done       ( o_dma_wr_done      ),
    .i_wr_index            ( o_dma_wr_index     ),
    .i_img_light           ( img_lit_s3         ),
    .i_img_motion_any      ( img_mna_s3         ),
    .i_img_motion_cnt      ( img_mot_s3         ),
    .i_img_red_cnt         ( img_red_s3         ),
    .i_img_grn_cnt         ( img_grn_s3         ),
    .i_img_tot_cnt         ( img_tot_s3         ),
    .o_disp_mode           ( disp_mode_w        ),
    .pio_wr_en             ( pio_wr_en         ),
    .pio_wr_addr           ( pio_wr_addr       ),
    .pio_wr_data           ( pio_wr_data       ),
    .pio_rd_en             ( pio_rd_en         ),
    .pio_rd_addr           ( pio_rd_addr       ),
    .pio_rd_data           ( pio_rd_data       )
);

//----------------------------------------------------------   pcie dma  ----------------------------------------------------------

// DMA CTRL      BASE ADDR = 0x8000
ips2l_pcie_dma #(
    .DEVICE_TYPE            (DEVICE_TYPE            ),
    .AXIS_SLAVE_NUM         (AXIS_SLAVE_NUM         )
)
u_ips2l_pcie_dma
(
    .clk                    (pclk_div2              ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                  (core_rst_n             ),
    //**********************************************************************
    .i_video_dma_req        (video_dma_req          ),
    .i_video_dma_addr       (video_dma_addr         ),
    .i_video_dma_len        (video_dma_len          ),
    .i_dma_32or64           (0                      ),    //64bit
    .o_dma_cmd_rdy          (dma_cmd_rdy            ),
    .o_dma_tx_done          (dma_tx_done            ),
    .o_ch0_base_addr        (      ),
    //dma
    .o_dma_rd_en            (w_dma_rd_en            ),
    .i_dma_rd_data          (w_dma_rd_data          ),
    .o_dma_wr_en            (dma_wr_en              ),
    .o_dma_wr_data          (dma_wr_data            ),
    //pio
    .o_pio_wr_en            (pio_wr_en              ),
    .o_pio_wr_addr          (pio_wr_addr            ),
    .o_pio_wr_data          (pio_wr_data            ),                            
    .o_pio_rd_en            (pio_rd_en              ),
    .o_pio_rd_addr          (pio_rd_addr            ),
    .o_pio_rd_data          (pio_rd_data            ),
    //**********************************************************************
    //num
    .i_cfg_pbus_num         (cfg_pbus_num           ),  //input [7:0]
    .i_cfg_pbus_dev_num     (cfg_pbus_dev_num       ),  //input [4:0]
    .i_cfg_max_rd_req_size  (cfg_max_rd_req_size    ),  //input [2:0]
    .i_cfg_max_payload_size (cfg_max_payload_size   ),  //input [2:0]
    //**********************************************************************
    //axis master interface
    .i_axis_master_tvld     (axis_master_tvalid_mem ),
    .o_axis_master_trdy     (axis_master_tready_mem ),
    .i_axis_master_tdata    (axis_master_tdata_mem  ),
    .i_axis_master_tkeep    (axis_master_tkeep_mem  ),
    .i_axis_master_tlast    (axis_master_tlast_mem  ),
    .i_axis_master_tuser    (axis_master_tuser_mem  ),

    //**********************************************************************
    //axis_slave0 interface
    .i_axis_slave0_trdy     (axis_slave0_tready     ),
    .o_axis_slave0_tvld     (dma_axis_slave0_tvalid ),
    .o_axis_slave0_tdata    (dma_axis_slave0_tdata  ),
    .o_axis_slave0_tlast    (dma_axis_slave0_tlast  ),
    .o_axis_slave0_tuser    (dma_axis_slave0_tuser  ),
    //axis_slave1 interface
    .i_axis_slave1_trdy     (axis_slave1_tready     ),
    .o_axis_slave1_tvld     (axis_slave1_tvalid     ),
    .o_axis_slave1_tdata    (axis_slave1_tdata      ),
    .o_axis_slave1_tlast    (axis_slave1_tlast      ),
    .o_axis_slave1_tuser    (axis_slave1_tuser      ),
    //axis_slave2 interface
    .i_axis_slave2_trdy     (axis_slave2_tready     ),
    .o_axis_slave2_tvld     (axis_slave2_tvalid     ),
    .o_axis_slave2_tdata    (axis_slave2_tdata      ),
    .o_axis_slave2_tlast    (axis_slave2_tlast      ),
    .o_axis_slave2_tuser    (axis_slave2_tuser      ),
    //from pcie
    .i_cfg_ido_req_en       (cfg_ido_req_en         ),
    .i_cfg_ido_cpl_en       (cfg_ido_cpl_en         ),
    .i_xadm_ph_cdts         (xadm_ph_cdts           ),
    .i_xadm_pd_cdts         (xadm_pd_cdts           ),
    .i_xadm_nph_cdts        (xadm_nph_cdts          ),
    .i_xadm_npd_cdts        (xadm_npd_cdts          ),
    .i_xadm_cplh_cdts       (xadm_cplh_cdts         ),
    .i_xadm_cpld_cdts       (xadm_cpld_cdts         ),
    //**********************************************************************
    //apb interface
    .i_apb_psel             (p_sel_dma              ),
    .i_apb_paddr            (p_addr[8:0]            ),
    .i_apb_pwdata           (p_wdata                ),
    .i_apb_pstrb            (p_strb                 ),
    .i_apb_pwrite           (p_we                   ),
    .i_apb_penable          (p_ce                   ),
    .o_apb_prdy             (p_rdy_dma              ),
    .o_apb_prdata           (p_rdata_dma            ),
    .o_cross_4kb_boundary   (cross_4kb_boundary     )
);

generate
    if (DEVICE_TYPE == 3'd4)
    begin:rc
    //----------------------------------------------------------   cfg ctrl  ----------------------------------------------------------
    //CFG TLP TX RX     BASE ADDR = 0x9000
        pcie_cfg_ctrl   u_pcie_cfg_ctrl(
            //from APB
            .pclk_div2              (pclk_div2              ),
            .apb_rst_n              (core_rst_n             ),
            .p_sel                  (p_sel_cfg              ),
            .p_strb                 (p_strb                 ),
            .p_addr                 (p_addr[7:0]            ),
            .p_wdata                (p_wdata                ),
            .p_ce                   (p_ce                   ),
            .p_we                   (p_we                   ),
            .p_rdy                  (p_rdy_cfg              ),
            .p_rdata                (p_rdata_cfg            ),
            .pcie_cfg_ctrl_en       (pcie_cfg_ctrl_en       ),
            //to PCIE ctrl
            .axis_slave_tready      (axis_slave0_tready     ),
            .axis_slave_tvalid      (cfg_axis_slave0_tvalid ),
            .axis_slave_tlast       (cfg_axis_slave0_tlast  ),
            .axis_slave_tuser       (cfg_axis_slave0_tuser  ),
            .axis_slave_tdata       (cfg_axis_slave0_tdata  ),
            
            .axis_master_tready     (axis_master_tready_cfg ),
            .axis_master_tvalid     (axis_master_tvalid     ),
            .axis_master_tlast      (axis_master_tlast      ),
        //    .axis_master_tuser      (axis_master_tuser      ),
            .axis_master_tkeep      (axis_master_tkeep      ),
            .axis_master_tdata      (axis_master_tdata      )
        );

        //----------------------------------------------------------   logic mux  ----------------------------------------------------------
        assign axis_slave0_tvalid      = pcie_cfg_ctrl_en ? cfg_axis_slave0_tvalid  : dma_axis_slave0_tvalid;
        assign axis_slave0_tlast       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tlast   : dma_axis_slave0_tlast;
        assign axis_slave0_tuser       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tuser   : dma_axis_slave0_tuser;
        assign axis_slave0_tdata       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tdata   : dma_axis_slave0_tdata;

        assign axis_master_tvalid_mem  = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tvalid;
        assign axis_master_tdata_mem   = pcie_cfg_ctrl_en ? 128'b0                  : axis_master_tdata;
        assign axis_master_tkeep_mem   = pcie_cfg_ctrl_en ? 4'b0                    : axis_master_tkeep;
        assign axis_master_tlast_mem   = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tlast;
        assign axis_master_tuser_mem   = pcie_cfg_ctrl_en ? 8'b0                    : axis_master_tuser;
        
        assign axis_master_tready      = pcie_cfg_ctrl_en ? axis_master_tready_cfg  : axis_master_tready_mem;
    end
    else
    begin:ep
        assign p_rdy_cfg               = 1'b0;
        assign p_rdata_cfg             = 32'b0;

        assign axis_slave0_tvalid      = dma_axis_slave0_tvalid;
        assign axis_slave0_tlast       = dma_axis_slave0_tlast;
        assign axis_slave0_tuser       = dma_axis_slave0_tuser;
        assign axis_slave0_tdata       = dma_axis_slave0_tdata;

        assign axis_master_tvalid_mem  = axis_master_tvalid;
        assign axis_master_tdata_mem   = axis_master_tdata;
        assign axis_master_tkeep_mem   = axis_master_tkeep;
        assign axis_master_tlast_mem   = axis_master_tlast;
        assign axis_master_tuser_mem   = axis_master_tuser;
        
        assign axis_master_tready      = axis_master_tready_mem;
    end
endgenerate

//----------------------------------------------------------   pcie wrap  ----------------------------------------------------------
//pcie wrap : HSSTLP : 0x0000~6000 PCIe BASE ADDR : 0x7000
pcie_test
u_ips2l_pcie_wrap
(
    .button_rst_n               (sync_button_rst_n      ),
    .power_up_rst_n             (sync_perst_n           ),
    .perst_n                    (sync_perst_n           ),
    //clk and rst
    .pclk                       (pclk                   ),      //output
    .pclk_div2                  (pclk_div2              ),      //output
    .ref_clk                    (ref_clk                ),      //output
    .ref_clk_n                  (ref_clk_n              ),      //input
    .ref_clk_p                  (ref_clk_p              ),      //input
    .core_rst_n                 (core_rst_n             ),      //output
    
    //APB interface to  DBI cfg
    //.p_clk                      (ref_clk                ),      //input
    .p_sel                      (p_sel_pcie             ),      //input
    .p_strb                     (uart_p_strb            ),      //input  [ 3:0]
    .p_addr                     (uart_p_addr            ),      //input  [15:0]
    .p_wdata                    (uart_p_wdata           ),      //input  [31:0]
    .p_ce                       (uart_p_ce              ),      //input
    .p_we                       (uart_p_we              ),      //input
    .p_rdy                      (p_rdy_pcie             ),      //output
    .p_rdata                    (p_rdata_pcie           ),      //output [31:0]
    
    //PHY diff signals
    .rxn                        (rxn                    ),      //input   [3:0]
    .rxp                        (rxp                    ),      //input   [3:0]
    .txn                        (txn                    ),      //output  [3:0]
    .txp                        (txp                    ),      //output  [3:0]
    
    .pcs_nearend_loop           ({2{1'b0}}              ),      //input
    .pma_nearend_ploop          ({2{1'b0}}              ),      //input
    .pma_nearend_sloop          ({2{1'b0}}              ),      //input
    
    //AXIS master interface
    .axis_master_tvalid         (axis_master_tvalid     ),      //output
    .axis_master_tready         (axis_master_tready     ),      //input
    .axis_master_tdata          (axis_master_tdata      ),      //output [127:0]
    .axis_master_tkeep          (axis_master_tkeep      ),      //output [3:0]
    .axis_master_tlast          (axis_master_tlast      ),      //output
    .axis_master_tuser          (axis_master_tuser      ),      //output [7:0]
    
    //axis slave 0 interface
    .axis_slave0_tready         (axis_slave0_tready     ),      //output
    .axis_slave0_tvalid         (axis_slave0_tvalid     ),      //input
    .axis_slave0_tdata          (axis_slave0_tdata      ),      //input  [127:0]
    .axis_slave0_tlast          (axis_slave0_tlast      ),      //input
    .axis_slave0_tuser          (axis_slave0_tuser      ),      //input
    
    //axis slave 1 interface
    .axis_slave1_tready         (axis_slave1_tready     ),      //output
    .axis_slave1_tvalid         (axis_slave1_tvalid     ),      //input
    .axis_slave1_tdata          (axis_slave1_tdata      ),      //input  [127:0]
    .axis_slave1_tlast          (axis_slave1_tlast      ),      //input
    .axis_slave1_tuser          (axis_slave1_tuser      ),      //input
    //axis slave 2 interface
    .axis_slave2_tready         (axis_slave2_tready     ),      //output
    .axis_slave2_tvalid         (axis_slave2_tvalid     ),      //input
    .axis_slave2_tdata          (axis_slave2_tdata      ),      //input  [127:0]
    .axis_slave2_tlast          (axis_slave2_tlast      ),      //input
    .axis_slave2_tuser          (axis_slave2_tuser      ),      //input
     
    .pm_xtlh_block_tlp          (                       ),      //output
    
    .cfg_send_cor_err_mux       (                       ),      //output
    .cfg_send_nf_err_mux        (                       ),      //output
    .cfg_send_f_err_mux         (                       ),      //output
    .cfg_sys_err_rc             (                       ),      //output
    .cfg_aer_rc_err_mux         (                       ),      //output
    //radm timeout
    .radm_cpl_timeout           (                       ),      //output
    
    //configuration signals
    .cfg_max_rd_req_size        (cfg_max_rd_req_size    ),      //output [2:0]
    .cfg_bus_master_en          (                       ),      //output
    .cfg_max_payload_size       (cfg_max_payload_size   ),      //output [2:0]
    .cfg_ext_tag_en             (                       ),      //output
    .cfg_rcb                    (cfg_rcb                ),      //output
    .cfg_mem_space_en           (                       ),      //output
    .cfg_pm_no_soft_rst         (                       ),      //output
    .cfg_crs_sw_vis_en          (                       ),      //output
    .cfg_no_snoop_en            (                       ),      //output
    .cfg_relax_order_en         (                       ),      //output
    .cfg_tph_req_en             (                       ),      //output [2-1:0]
    .cfg_pf_tph_st_mode         (                       ),      //output [3-1:0]
    .rbar_ctrl_update           (                       ),      //output
    .cfg_atomic_req_en          (                       ),      //output
    
    .cfg_pbus_num               (cfg_pbus_num           ),      //output [7:0]
    .cfg_pbus_dev_num           (cfg_pbus_dev_num       ),      //output [4:0]
    
    //debug signals
    .radm_idle                  (                       ),      //output
    .radm_q_not_empty           (                       ),      //output
    .radm_qoverflow             (                       ),      //output
    .diag_ctrl_bus              (2'b0                   ),      //input   [1:0]
    .cfg_link_auto_bw_mux       (                       ),      //output              merge cfg_link_auto_bw_msi and cfg_link_auto_bw_int
    .cfg_bw_mgt_mux             (                       ),      //output              merge cfg_bw_mgt_int and cfg_bw_mgt_msi
    .cfg_pme_mux                (                       ),      //output              merge cfg_pme_int and cfg_pme_msi
    .app_ras_des_sd_hold_ltssm  (1'b0                   ),      //input
    .app_ras_des_tba_ctrl       (2'b0                   ),      //input   [1:0]
    
    .dyn_debug_info_sel         (4'b0                   ),      //input   [3:0]
    .debug_info_mux             (                       ),      //output  [132:0]
    
    //system signal
    .smlh_link_up               (smlh_link_up           ),      //output
    .rdlh_link_up               (rdlh_link_up           ),      //output
    .smlh_ltssm_state           (smlh_ltssm_state       )       //output  [4:0]
);





endmodule
