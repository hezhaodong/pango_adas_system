//=============================================================================
// frame_diff_motion.v
//-----------------------------------------------------------------------------
// 功能：帧差运动检测（降采样版，用分布式 RAM 存上一帧）
//
// 原理：
//   把画面按 BLOCK x BLOCK 分块（默认 16x16）
//   每块求灰度平均值，存到「上一帧块值」存储器
//   当前帧块均值与上一帧比较，差值超过 DIFF_THRESH 就判定该块有运动
//
// 为什么要降采样：
//   完整 1280x720 帧缓冲需要 921,600 字节 ≈ 7.4 Mb，
//   PG2L50H 的 BRAM 只有 2.5 Mb，装不下（除非用板载 DDR3）
//   降采样到 16x16 块只要 80x45 = 3,600 字节，用 LUTRAM 即可，代价极小
//
// 每帧输出：
//   o_motion_any   本帧是否存在运动
//   o_motion_cnt   本帧运动块数量
//   o_block_valid / o_block_idx / o_block_motion   块级明细（可选，便于可视化）
//
// 典型用途：行人/车辆闯入检测、摄像头被遮挡检测、路障出现检测
//
// 适用：紫光同创 Logos2 系列，纯 Verilog-2001，无厂商原语
//=============================================================================
`timescale 1ns / 1ps
`default_nettype none

module frame_diff_motion #(
    parameter IMG_W        = 1280,
    parameter IMG_H        = 720,
    parameter BLOCK_LOG2   = 4,        // 块大小 = 2^BLOCK_LOG2（默认 16）
    parameter DIFF_THRESH  = 8'd24     // 灰度差阈值
)(
    input  wire        i_clk,
    input  wire        i_rstn,
    input  wire        i_de,
    input  wire        i_vsync,
    input  wire [7:0]  i_gray,

    output reg         o_motion_any,    // 帧末更新
    output reg  [15:0] o_motion_cnt,    // 帧末更新
    output reg         o_block_valid,   // 块完成脉冲
    output reg  [13:0] o_block_idx,
    output reg         o_block_motion
);

    localparam BLOCK     = 1 << BLOCK_LOG2;              // 16
    localparam BITS      = BLOCK_LOG2;                   // 4
    localparam BLK_W     = (IMG_W + BLOCK - 1) / BLOCK;  // 80
    localparam BLK_H     = (IMG_H + BLOCK - 1) / BLOCK;  // 45
    localparam BLK_TOTAL = BLK_W * BLK_H;                // 3600
    localparam SUM_BITS  = 8 + 2*BLOCK_LOG2;             // 8+8=16

    //------------------------------------------------------------------
    // 行/列坐标
    //------------------------------------------------------------------
    reg [11:0] x, y;

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            x <= 12'd0;
            y <= 12'd0;
        end else if (i_vsync) begin
            x <= 12'd0;
            y <= 12'd0;
        end else if (i_de) begin
            if (x >= IMG_W[11:0] - 12'd1) begin
                x <= 12'd0;
                y <= (y >= IMG_H[11:0] - 12'd1) ? 12'd0 : (y + 12'd1);
            end else begin
                x <= x + 12'd1;
            end
        end
    end

    // 块内最后一个像素
    wire blk_last_x = (x[BITS-1:0] == {BITS{1'b1}});
    wire blk_last_y = (y[BITS-1:0] == {BITS{1'b1}});
    wire blk_done   = i_de && blk_last_x && blk_last_y;

    //------------------------------------------------------------------
    // 块内累加（求和）
    //------------------------------------------------------------------
    reg [SUM_BITS-1:0] blk_sum;

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            blk_sum <= {SUM_BITS{1'b0}};
        end else if (i_vsync) begin
            blk_sum <= {SUM_BITS{1'b0}};
        end else if (i_de) begin
            if (blk_done) blk_sum <= {SUM_BITS{1'b0}};       // 块结束，清零
            else          blk_sum <= blk_sum + {8'd0, i_gray};
        end
    end

    // 块均值：最后一个像素也要算进去
    wire [SUM_BITS-1:0] blk_sum_full = blk_sum + {8'd0, i_gray};
    wire [7:0]          blk_avg      = blk_sum_full[SUM_BITS-1 : 2*BLOCK_LOG2];

    //------------------------------------------------------------------
    // 上一帧块值存储器（异步读，推断为分布式 RAM）
    //------------------------------------------------------------------
    reg [7:0] prev_mem [0:BLK_TOTAL-1];

    reg [13:0] blk_idx;      // 0 ~ BLK_TOTAL-1

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            blk_idx <= 14'd0;
        end else if (i_vsync) begin
            blk_idx <= 14'd0;
        end else if (blk_done) begin
            blk_idx <= (blk_idx >= BLK_TOTAL[13:0] - 14'd1) ? 14'd0 : (blk_idx + 14'd1);
        end
    end

    wire [7:0] prev_val = prev_mem[blk_idx];

    // 写回当前块均值（同步写）
    always @(posedge i_clk) begin
        if (blk_done) begin
            prev_mem[blk_idx] <= blk_avg;
        end
    end

    //------------------------------------------------------------------
    // 差分判定
    //------------------------------------------------------------------
    wire [7:0]  diff   = (blk_avg >= prev_val) ? (blk_avg - prev_val)
                                               : (prev_val - blk_avg);
    wire        motion = (diff > DIFF_THRESH);

    //------------------------------------------------------------------
    // 帧级统计
    //------------------------------------------------------------------
    reg [15:0] frame_cnt;

    always @(posedge i_clk) begin
        if (!i_rstn) begin
            frame_cnt      <= 16'd0;
            o_motion_cnt   <= 16'd0;
            o_motion_any   <= 1'b0;
            o_block_valid  <= 1'b0;
            o_block_idx    <= 14'd0;
            o_block_motion <= 1'b0;
        end else begin
            o_block_valid <= blk_done;

            if (blk_done) begin
                o_block_idx    <= blk_idx;
                o_block_motion <= motion;
                if (motion && (frame_cnt != 16'hFFFF))
                    frame_cnt <= frame_cnt + 16'd1;
            end

            if (i_vsync) begin
                // 帧末：锁存统计结果并清零
                o_motion_cnt <= frame_cnt;
                o_motion_any <= (frame_cnt != 16'd0);
                frame_cnt    <= 16'd0;
            end
        end
    end

endmodule

`default_nettype wire
