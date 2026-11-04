/*
	v1_0:pio控制
*/
module	pio_crtl
(
	input					pcie_clk			/* synthesis PAP_MARK_DEBUG="true" */,
	input					rst_n				,

	//crtl
	output					start_flag			/* synthesis PAP_MARK_DEBUG="true" */,
    output                  set_dma_config_en   /* synthesis PAP_MARK_DEBUG="true" */,
	output	[63:0]			o_ch0_base_addr		/* synthesis PAP_MARK_DEBUG="true" */,
	output	[63:0]			o_ch0_base_addr2    /* synthesis PAP_MARK_DEBUG="true" */,
	output	[63:0]			o_ch0_base_addr3	/* synthesis PAP_MARK_DEBUG="true" */,
	output	[63:0]			o_ch0_base_addr4	/* synthesis PAP_MARK_DEBUG="true" */,
    input                   i_wr_frame_done     /* synthesis PAP_MARK_DEBUG="true" */,
    input   [1:0]           i_wr_index          /* synthesis PAP_MARK_DEBUG="true" */,
	//------ 图像处理结果（已同步到 pcie_clk 域）------
	input	[1:0]			i_img_light					,
	input					i_img_motion_any				,
	input	[15:0]			i_img_motion_cnt			,
	input	[15:0]			i_img_red_cnt				,
	input	[15:0]			i_img_grn_cnt				,
	input	[15:0]			i_img_tot_cnt				,
	//------ 显示模式控制（输出给显示通路）------
	output	[1:0]			o_disp_mode					,
	//
	input					pio_wr_en			/* synthesis PAP_MARK_DEBUG="true" */,
	input	[9:0]			pio_wr_addr			/* synthesis PAP_MARK_DEBUG="true" */,
	input	[31:0]			pio_wr_data			/* synthesis PAP_MARK_DEBUG="true" */,
	
	//
	input					pio_rd_en			/* synthesis PAP_MARK_DEBUG="true" */,
	input	[9:0]			pio_rd_addr			/* synthesis PAP_MARK_DEBUG="true" */,
	output	[31:0]			pio_rd_data		    /* synthesis PAP_MARK_DEBUG="true" */

);
//------------------寄存器----------------------------
// ⚠ 地址必须声明为 10 位：pio_wr_addr / pio_rd_addr 是 [9:0]。
//   若误写成 9 位宽且地址 > 0x1FF，Verilog 会静默截断高位，例如
//     9 位的 h200 -> 0x000（与 START/STOP 冲突）、h20C -> 0x00C、h3FC -> 0x1FC！
//   原设计地址均 <= 0x1FF，所以 9 位恰好没暴露问题；新增地址必须用 10 位。
parameter       WR_FRAME_DONE    =    10'h140    ;
parameter       DMA_ADDR_L  	 =    10'h050    ;
parameter       DMA_ADDR_H	   	 =    10'h054    ;    //
parameter       DMA_ADDR1_L  	 =    10'h020    ;
parameter       DMA_ADDR1_H	   	 =    10'h024    ;    //
parameter       DMA_ADDR2_L  	 =    10'h040    ;
parameter       DMA_ADDR2_H	   	 =    10'h044    ;    //
parameter       DMA_ADDR3_L  	 =    10'h030    ;
parameter       DMA_ADDR3_H	   	 =    10'h034    ;    //
parameter		DMA_SET_EN		 =	  10'h060	;
//------------------ 图像处理寄存器（本项目新增）------------------
parameter       FPGA_STATUS0     =    10'h200    ;   // R: 感知状态（红绿灯/运动）
parameter       FPGA_STATUS1     =    10'h204    ;   // R: 红/绿 像素计数
parameter       FPGA_STATUS2     =    10'h208    ;   // R: ROI 总像素计数
parameter       FPGA_DISP_MODE   =    10'h20C    ;   // W: 显示模式控制
// ⚠ 选址注意（2026-10-10 真机实测）：PIO 区域【最后 8 字节 0x3F8~0x3FF 不承载写数据】。
//   实测：向 0x2E0~0x3F0 写任意值，0x1A0(lastData) 都能正确捕获；
//         向 0x3F8 / 0x3FC 写，0x1A0 捕获到 0（地址能到、数据丢了）。
//   指纹寄存器原先放在 0x3FC，真机读回 0 —— 故改到 0x320。
parameter       FPGA_DESIGN_ID   =    10'h320    ;   // R: 设计指纹（验证烧的是哪版位流）
//==========================================================================
//  学长 ARM 兼容寄存器（本项目新增）
//--------------------------------------------------------------------------
//  来源：板上 /userdata/high_speed_pcie/pcie_device.cpp（学长 ARM 源码）
//        + video_pipeline_multi_input.md（架构文档）
//        2026-10-10 在真机上逐位实测确认过语义。
//
//  为什么必须有：学长的 ARM 程序（camera_capture_v2，完整版 986KB）启动采集时
//  会依赖这些寄存器（依据：mainwindow.cpp 的"开始采集"路径 + pcie_device.cpp）：
//    · startCaptureChecked(150) 写 0x000=0xFFFFFFE5 后轮询 0x2D0 的 bit1 确认 START
//      → bit1 恒 0 就报 "FPGA START command was not confirmed" 并放弃采集【硬阻塞】
//    · rearmFpgaInputSource()/setFpgaInputSource() 依赖 0x300/0x310
//    · programFpgaDmaBases() 末尾读 0x190/0x1A0/0x1B0 —— 但只是打日志，不校验
//      （pcie_device.cpp 自注释："0x1b0 is only a PIO write counter"）
//==========================================================================
parameter       FPGA_BRIGHTNESS  =    10'h070    ;   // W: 亮度等级（采集启动后 ARM 写 0）
parameter       FPGA_LAST_ADDR   =    10'h190    ;   // R: 最后一次 PIO 写的地址
parameter       FPGA_LAST_DATA   =    10'h1A0    ;   // R: 最后一次 PIO 写的数据
parameter       FPGA_WR_COUNT    =    10'h1B0    ;   // R: PIO 写操作总计数（仅日志）
parameter       FPGA_FRAME_CNT   =    10'h2A0    ;   // R: FPGA 帧完成计数 frameDoneDbgCount
parameter       FPGA_DBG_STATUS  =    10'h2D0    ;   // R: [1]=START 已生效
parameter       FPGA_SRC_SWITCH  =    10'h300    ;   // W: 输入源切换请求 0=FMC 1=HDMI 2=FIBER
parameter       FPGA_SRC_STATUS  =    10'h310    ;   // R: 输入源状态
// 本工程只有一路输入，没有 MUX。这两条参数决定"假装"报告哪一路：
//   0 = FMC   1 = HDMI   2 = FIBER
parameter       SRC_INITIAL      =    2'd0       ;   // 上电默认值（真机实测为 0=FMC）
parameter       SRC_LOOPBACK     =    1'b1       ;   // 1 = 0x300 写入即生效（让 ARM 的切换一定成功）
//------------------reg------------------------------
reg				r_start_flag			;
reg				r_set_dma_config_en     ;
reg		[31:0]  r_pio_rd_data           ;
reg             r_wr_frame_done         ;
reg		[63:0]	r_ch0_base_addr			/* synthesis PAP_MARK_DEBUG="true" */;
reg		[63:0]	r_ch0_base_addr2		/* synthesis PAP_MARK_DEBUG="true" */;
reg		[63:0]	r_ch0_base_addr3		/* synthesis PAP_MARK_DEBUG="true" */;
reg		[63:0]	r_ch0_base_addr4		/* synthesis PAP_MARK_DEBUG="true" */;
reg		[1:0]	r_disp_mode				;
// 调试寄存器（0x190/0x1A0/0x1B0）
reg		[9:0]	r_last_addr				;
reg		[31:0]	r_last_data				;
reg		[31:0]	r_wr_count				;
// 输入源（0x300/0x310）—— 本工程单路，仅"上报"用
reg		[1:0]	r_src_req				;
reg		[1:0]	r_src_active			;
reg		[7:0]	r_src_epoch				;
// 亮度（0x070）/ 帧计数（0x2A0）
reg		[31:0]	r_brightness			;
reg		[31:0]	r_frame_done_cnt		;

//用户端
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_start_flag	<=	1'd0	;
	else	if(pio_wr_en && pio_wr_addr == 0 && pio_wr_data == 32'hffffffe5)	//开启
		r_start_flag	<=	1'd1	;
	else	if(pio_wr_en && pio_wr_addr == 0 && pio_wr_data == 32'hffffff00)	//关闭
		r_start_flag	<=	1'd0	;
	else
		r_start_flag	<=	r_start_flag	;
end

//内核
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_set_dma_config_en	<=	1'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_SET_EN && pio_wr_data == 32'h00000001)	//开启
		r_set_dma_config_en	<=	1'd1	;
	else
		r_set_dma_config_en	<=	1'd0	;
end


//低32
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_ch0_base_addr[31:0]	<=	'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_ADDR_L)
		r_ch0_base_addr[31:0]	<=	pio_wr_data	;
	else
		r_ch0_base_addr[31:0]	<=	r_ch0_base_addr[31:0]	;
end

//高32
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_ch0_base_addr[63:32]	<=	'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_ADDR_H)
		r_ch0_base_addr[63:32]	<=	pio_wr_data	;
	else
		r_ch0_base_addr[63:32]	<=	r_ch0_base_addr[63:32]	;
end

//低32
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_ch0_base_addr2[31:0]	<=	'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_ADDR1_L)
		r_ch0_base_addr2[31:0]	<=	pio_wr_data	;
	else
		r_ch0_base_addr2[31:0]	<=	r_ch0_base_addr2[31:0]	;
end

//高32
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_ch0_base_addr2[63:32]	<=	'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_ADDR1_H)
		r_ch0_base_addr2[63:32]	<=	pio_wr_data	;
	else
		r_ch0_base_addr2[63:32]	<=	r_ch0_base_addr2[63:32]	;
end

//低32
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_ch0_base_addr3[31:0]	<=	'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_ADDR2_L)
		r_ch0_base_addr3[31:0]	<=	pio_wr_data	;
	else
		r_ch0_base_addr3[31:0]	<=	r_ch0_base_addr3[31:0]	;
end

//高32
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_ch0_base_addr3[63:32]	<=	'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_ADDR2_H)
		r_ch0_base_addr3[63:32]	<=	pio_wr_data	;
	else
		r_ch0_base_addr3[63:32]	<=	r_ch0_base_addr3[63:32]	;
end

//低32
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_ch0_base_addr4[31:0]	<=	'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_ADDR3_L)
		r_ch0_base_addr4[31:0]	<=	pio_wr_data	;
	else
		r_ch0_base_addr4[31:0]	<=	r_ch0_base_addr4[31:0]	;
end

//高32
always@(posedge pcie_clk)	begin
	if(!rst_n)	
		r_ch0_base_addr4[63:32]	<=	'd0	;
	else	if(pio_wr_en && pio_wr_addr == DMA_ADDR3_H)
		r_ch0_base_addr4[63:32]	<=	pio_wr_data	;
	else
		r_ch0_base_addr4[63:32]	<=	r_ch0_base_addr4[63:32]	;
end


//写完一帧信号
always@(posedge pcie_clk)	begin
	if(!rst_n)	
        r_wr_frame_done    <=    1'd0    ;  
    else    if(pio_rd_en && pio_rd_addr == WR_FRAME_DONE)
        r_wr_frame_done    <=    1'd0    ;
    else    if(i_wr_frame_done)
        r_wr_frame_done    <=    1'd1    ; 
    else
        r_wr_frame_done    <=    r_wr_frame_done    ;
end



//显示模式控制（写 0x20C，bit[1:0]）
always@(posedge pcie_clk)	begin
	if(!rst_n)
		r_disp_mode	<=	2'd0	;
	else	if(pio_wr_en && pio_wr_addr == FPGA_DISP_MODE)
		r_disp_mode	<=	pio_wr_data[1:0]	;
	else
		r_disp_mode	<=	r_disp_mode	;
end

//==========================================================================
// 调试寄存器：记录最后一次写 + 写总次数（0x190 / 0x1A0 / 0x1B0）
//--------------------------------------------------------------------------
// 真机实测语义（2026-10-10）：
//   写 1 次 START 后        0x1A0=0xFFFFFFE5  0x1B0=0x00000001
//   再写 0x060/0x050 两笔后 0x190=0x00000050  0x1A0=0xDEADBEEF  0x1B0=0x00000003
//   → 0x190=最后写的地址，0x1A0=最后写的数据，0x1B0=写计数（每写一次 +1）
// 注意：计数包含对调试寄存器自身的写，与真机行为一致。
//==========================================================================
always@(posedge pcie_clk)	begin
	if(!rst_n)	begin
		r_last_addr	<=	10'd0	;
		r_last_data	<=	32'd0	;
		r_wr_count	<=	32'd0	;
	end else if(pio_wr_en)	begin
		r_last_addr	<=	pio_wr_addr	;
		r_last_data	<=	pio_wr_data	;
		r_wr_count	<=	r_wr_count + 32'd1	;
	end
end

//==========================================================================
// 输入源请求/状态（0x300 写 / 0x310 读）
//--------------------------------------------------------------------------
// ⚠ 本工程（pcie_test_img_50h_5640_mul）只有【一路】输入，没有 MUX。
//   学长的 high_speed_test_ov_fmc 才是双路。所以这里只能"假装"支持切换：
//     SRC_LOOPBACK=1 时，写 0x300 立即生效 → ARM 的 setFpgaInputSource() 一定成功
//     否则 requested 跟随写入、active 保持 SRC_INITIAL → ARM 会超时回滚（更诚实）
//   epoch 每完成一次源变化自增 1，与真机一致。
//==========================================================================
always@(posedge pcie_clk)	begin
	if(!rst_n)	begin
		r_src_req		<=	SRC_INITIAL	;
		r_src_active	<=	SRC_INITIAL	;
		r_src_epoch		<=	8'd0	;
	end else if(pio_wr_en && pio_wr_addr == FPGA_SRC_SWITCH)	begin
		r_src_req	<=	pio_wr_data[1:0]	;
		if(SRC_LOOPBACK)	begin
			if(pio_wr_data[1:0] != r_src_active)	begin
				r_src_active	<=	pio_wr_data[1:0]	;
				r_src_epoch		<=	r_src_epoch + 8'd1	;
			end
		end
	end
end

//==========================================================================
// 亮度等级（0x070，写寄存器）。本工程没有亮度处理通路，仅存值 + 可读回验证。
//==========================================================================
always@(posedge pcie_clk)	begin
	if(!rst_n)
		r_brightness	<=	32'd0	;
	else	if(pio_wr_en && pio_wr_addr == FPGA_BRIGHTNESS)
		r_brightness	<=	pio_wr_data	;
end

//==========================================================================
// 帧完成计数（0x2A0）。每收到一次 i_wr_frame_done（一帧 DMA 发完）自增 1。
// 对应完整版 ARM 的 frameDoneDbgCount，用于 [capture-rate] hw_delta 诊断。
//==========================================================================
always@(posedge pcie_clk)	begin
	if(!rst_n)
		r_frame_done_cnt	<=	32'd0	;
	else	if(i_wr_frame_done)
		r_frame_done_cnt	<=	r_frame_done_cnt + 32'd1	;
end

//pio_read
always@(posedge pcie_clk)	begin
	if(!rst_n)
        r_pio_rd_data    <=    32'd0    ;
    else    if(pio_rd_en)    begin
        case(pio_rd_addr)
            // 0x140 扩展：真机为 { epoch[15:8], source[4:3], buffer_index[2:1], frame_done[0] }
            //   ⚠ source 是【2 位】：完整版 ARM 用 (rd>>3)&0x3 解析，
            //     2=Fiber / 1=HDMI / 0=FMC。曾误放 1 位（Fiber 会被报成 FMC）。
            //   位宽核对：16+8+3+2+2+1 = 32
            WR_FRAME_DONE    :    r_pio_rd_data    <=    {16'd0, r_src_epoch, 3'd0, r_src_active, i_wr_index, r_wr_frame_done};
            //------ 学长 ARM 兼容寄存器 ------
            FPGA_BRIGHTNESS  :    r_pio_rd_data    <=    r_brightness;
            FPGA_LAST_ADDR   :    r_pio_rd_data    <=    {22'd0, r_last_addr};
            FPGA_LAST_DATA   :    r_pio_rd_data    <=    r_last_data;
            FPGA_WR_COUNT    :    r_pio_rd_data    <=    r_wr_count;
            FPGA_FRAME_CNT   :    r_pio_rd_data    <=    r_frame_done_cnt;
            // 0x2D0：真机 bit1 = START 已生效（ARM startCapture 轮询它）
            //   位宽核对：28+2+1+1 = 32（曾误写 29'd0 → 33 位隐式截断）
            FPGA_DBG_STATUS  :    r_pio_rd_data    <=    {28'd0, 2'b00, r_start_flag, 1'b0};
            // 0x310 SOURCE_STATUS
            //   位布局（与学长 pcie_device.cpp 的解析函数一致）：
            //     bit0     = active_source[0]
            //     bit1     = request[0]
            //     bit2     = busy（单路恒 0）
            //     bit3     = active_source[1]
            //     bit4     = request[1]
            //     [15:8]   = epoch
            //   ⚠ 架构文档 video_pipeline_multi_input.md 把 bit3/bit4 写作
            //     fmc_valid / hdmi_valid，与代码冲突；代码是实际运行的解析者，
            //     故此处按代码布局实现（单路输入时 req/active 的高位恒 0）。
            FPGA_SRC_STATUS  :    r_pio_rd_data    <=    {16'd0, r_src_epoch, 3'd0,
                                                          r_src_req[1],
                                                          r_src_active[1],
                                                          1'b0,                 // bit2 busy
                                                          r_src_req[0],
                                                          r_src_active[0]};
            //------ 图像处理状态（本项目新增）------
            FPGA_STATUS0     :    r_pio_rd_data    <=    {i_img_motion_cnt, i_img_motion_any, 2'b00, i_img_light, 11'd0};
            FPGA_STATUS1     :    r_pio_rd_data    <=    {i_img_red_cnt, i_img_grn_cnt};
            FPGA_STATUS2     :    r_pio_rd_data    <=    {i_img_tot_cnt, 16'd0};
            FPGA_DESIGN_ID   :    r_pio_rd_data    <=    32'h494D4750;   // "IMGP" 图像处理版
            default          :    r_pio_rd_data    <=    32'd0;
        endcase
    end
end

//--------------------------------------------------------------------------------------
//开始
assign start_flag = r_start_flag	;
assign set_dma_config_en = r_set_dma_config_en    ;

assign pio_rd_data 	= r_pio_rd_data;
assign o_ch0_base_addr  = r_ch0_base_addr	;
assign o_ch0_base_addr2 = r_ch0_base_addr2	;
assign o_ch0_base_addr3 = r_ch0_base_addr3	;
assign o_ch0_base_addr4 = r_ch0_base_addr4	;
assign o_disp_mode = r_disp_mode	;
//-------------预留---------------



endmodule
