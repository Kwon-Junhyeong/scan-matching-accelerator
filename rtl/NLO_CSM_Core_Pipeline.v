`timescale 1ns / 1ps

// One Gauss-Newton pass: scan points in on a stream, the sums H and g out.
// Input is accepted only while the FIFO in front of stage 2 has room.
module NLO_CSM_Core_Pipeline #(
    parameter integer FIFO_DEPTH   = 8,
    parameter integer READ_LATENCY = 1,     // block RAM read latency in clocks
    parameter integer DRAIN        = 10     // clocks from stage 2 output to the sums, with margin
)(
    input  wire clk,
    input  wire rst_n,

    input  wire start,
    input  wire [31:0] init_pose_x, init_pose_y, init_pose_theta,
    input  wire [31:0] tau_threshold,

    input  wire [31:0] i_map_base_addr, i_map_width, i_map_height,

    input  wire [31:0] s_axis_tdata,     // {angle[15:0], distance[15:0]}
    input  wire        s_axis_tvalid,
    input  wire        s_axis_tlast,
    output wire        s_axis_tready,

    output wire [31:0] bram_addr,
    output wire        bram_en,
    output wire [3:0]  bram_we,
    input  wire [31:0] bram_rdata,

    output reg         done_irq,
    output wire signed [63:0] H11, H12, H13, H22, H23, H33,
    output wire signed [63:0] K1,  K2,  K3,
    output wire [31:0] o_point_cnt,
    output wire [1:0]  o_current_state,
    output wire        o_st1_ready,
    output wire        o_st2_ready
);

    localparam STATE_IDLE = 2'd0;
    localparam STATE_RUN  = 2'd1;
    localparam STATE_WAIT = 2'd2;
    localparam STATE_DONE = 2'd3;
    localparam integer AW = (FIFO_DEPTH > 1) ? $clog2(FIFO_DEPTH) : 1;

    reg  [1:0]  current_state;
    reg  [31:0] point_counter;
    reg  [7:0]  latency_cnt;
    reg  [7:0]  in_flight;           // accepted, not yet taken by stage 2

    reg         st1_valid_reg;
    reg  [31:0] st1_data_reg;

    wire signed [31:0] st1_gx, st1_gy, st1_dx, st1_dy; wire st1_v;
    wire [7:0]  st2_p00, st2_p10, st2_p01, st2_p11; wire [31:0] st2_fx, st2_fy;
    wire signed [31:0] st2_dx, st2_dy; wire st2_v;
    wire signed [31:0] st3_gx, st3_gy, st3_sf; wire signed [31:0] st3_dx, st3_dy; wire st3_v, st3_pv;
    wire signed [31:0] st4_j1, st4_j2, st4_j3, st4_sf; wire st4_v, st4_pv;
    wire st1_ready, st2_ready, st2_idle;

    // FIFO between stage 1 and stage 2
    reg  [127:0] fifo_mem [0:FIFO_DEPTH-1];
    reg  [AW-1:0] wr_ptr, rd_ptr;
    reg  [7:0]   fifo_cnt;
    wire         fifo_valid = (fifo_cnt != 0);
    wire [127:0] fifo_out   = fifo_mem[rd_ptr];
    wire         fifo_pop   = fifo_valid && st2_ready;

    wire in_hs = s_axis_tvalid && s_axis_tready;

    assign s_axis_tready   = (current_state == STATE_RUN) && st1_ready && (in_flight < FIFO_DEPTH);
    assign o_current_state = current_state;
    assign o_point_cnt     = point_counter;
    assign o_st1_ready     = st1_ready;
    assign o_st2_ready     = st2_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st1_valid_reg <= 1'b0;
            st1_data_reg  <= 32'b0;
        end else begin
            st1_valid_reg <= in_hs;
            st1_data_reg  <= s_axis_tdata;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0; rd_ptr <= 0; fifo_cnt <= 0; in_flight <= 0;
        end else begin
            if (st1_v) begin
                fifo_mem[wr_ptr] <= {st1_gx, st1_gy, st1_dx, st1_dy};
                wr_ptr <= (wr_ptr == FIFO_DEPTH-1) ? {AW{1'b0}} : wr_ptr + 1'b1;
            end
            if (fifo_pop) rd_ptr <= (rd_ptr == FIFO_DEPTH-1) ? {AW{1'b0}} : rd_ptr + 1'b1;
            fifo_cnt  <= fifo_cnt  + (st1_v ? 8'd1 : 8'd0) - (fifo_pop ? 8'd1 : 8'd0);
            in_flight <= in_flight + (in_hs ? 8'd1 : 8'd0) - (fifo_pop ? 8'd1 : 8'd0);
        end
    end

    wire pipe_empty = (in_flight == 0) && st2_idle;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= STATE_IDLE;
            point_counter <= 0;
            latency_cnt   <= 0;
            done_irq      <= 0;
        end else if (!start && current_state != STATE_IDLE) begin
            current_state <= STATE_IDLE;
        end else begin
            case (current_state)
                STATE_IDLE: begin
                    done_irq <= 0;
                    point_counter <= 0;
                    latency_cnt <= 0;
                    if (start && pipe_empty) current_state <= STATE_RUN;
                end

                STATE_RUN: begin
                    if (in_hs) begin
                        point_counter <= point_counter + 1;
                        if ((point_counter == 359) || s_axis_tlast)
                            current_state <= STATE_WAIT;
                    end
                end

                STATE_WAIT: begin
                    if (!pipe_empty)
                        latency_cnt <= 0;
                    else if (latency_cnt < DRAIN)
                        latency_cnt <= latency_cnt + 1;
                    else begin
                        done_irq <= 1'b1;
                        current_state <= STATE_DONE;
                    end
                end

                STATE_DONE: begin
                    if (!start) current_state <= STATE_IDLE;
                end
            endcase
        end
    end

    Stage1_Coord_Calc u_st1 (
        .clk(clk), .rst_n(rst_n),
        .pose_x(init_pose_x), .pose_y(init_pose_y), .pose_theta(init_pose_theta),
        .lidar_dist(st1_data_reg[15:0]), .lidar_angle(st1_data_reg[31:16]),
        .valid_in(st1_valid_reg), .ready_out(st1_ready),
        .global_x(st1_gx), .global_y(st1_gy),
        .deriv_theta_x(st1_dx), .deriv_theta_y(st1_dy),
        .valid_out(st1_v)
    );

    Stage2_Mem_Read #(.READ_LATENCY(READ_LATENCY)) u_st2 (
        .clk(clk), .rst_n(rst_n), .o_ready(st2_ready), .o_idle(st2_idle),
        .i_map_base_addr(i_map_base_addr), .i_map_width(i_map_width), .i_map_height(i_map_height),
        .st1_global_x(fifo_out[127:96]), .st1_global_y(fifo_out[95:64]),
        .st1_deriv_x(fifo_out[63:32]),   .st1_deriv_y(fifo_out[31:0]),
        .st1_valid(fifo_valid),
        .bram_addr(bram_addr), .bram_en(bram_en), .bram_we(bram_we), .bram_rdata(bram_rdata),
        .m_p00(st2_p00), .m_p10(st2_p10), .m_p01(st2_p01), .m_p11(st2_p11),
        .frac_x(st2_fx), .frac_y(st2_fy),
        .st2_deriv_x(st2_dx), .st2_deriv_y(st2_dy), .st2_valid(st2_v)
    );

    Stage3_Gradient_Calc u_st3 (
        .clk(clk), .rst_n(rst_n),
        .p00(st2_p00), .p10(st2_p10), .p01(st2_p01), .p11(st2_p11),
        .frac_x(st2_fx), .frac_y(st2_fy), .tau_threshold(tau_threshold),
        .st2_deriv_x(st2_dx), .st2_deriv_y(st2_dy), .st2_valid(st2_v),
        .grad_x(st3_gx), .grad_y(st3_gy), .score_f(st3_sf), .point_valid_flag(st3_pv),
        .st3_deriv_x(st3_dx), .st3_deriv_y(st3_dy), .st3_valid(st3_v)
    );

    Stage4_Jacobian_Calc u_st4 (
        .clk(clk), .rst_n(rst_n),
        .grad_x(st3_gx), .grad_y(st3_gy), .score_f(st3_sf), .point_valid_in(st3_pv),
        .synced_deriv_x(st3_dx), .synced_deriv_y(st3_dy), .st3_valid(st3_v),
        .j_i1(st4_j1), .j_i2(st4_j2), .j_i3(st4_j3), .st4_score(st4_sf),
        .point_valid_out(st4_pv), .st4_valid(st4_v)
    );

    Stage5_Matrix_MAC u_st5 (
        .clk(clk), .rst_n(rst_n), .current_state(current_state),
        .j_i1(st4_j1), .j_i2(st4_j2), .j_i3(st4_j3), .f_i(st4_sf),
        .point_valid_in(st4_pv), .st4_valid(st4_v),
        .H11(H11), .H12(H12), .H13(H13), .H22(H22), .H23(H23), .H33(H33),
        .K1(K1), .K2(K2), .K3(K3)
    );

endmodule
