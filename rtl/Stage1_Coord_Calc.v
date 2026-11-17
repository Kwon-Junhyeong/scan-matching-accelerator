`timescale 1ns / 1ps

// Stage 1: scan point to map coordinates (14 fraction bits) and their heading derivatives.
module Stage1_Coord_Calc (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] pose_x, pose_y, pose_theta,
    input  wire [15:0] lidar_dist,
    input  wire [15:0] lidar_angle,
    input  wire        valid_in,
    output wire        ready_out,

    output reg  [31:0] global_x, global_y,
    output reg  [31:0] deriv_theta_x, deriv_theta_y,
    output reg         valid_out
);

    // CORDIC latency in clocks; the pose and the distance travel beside it
    localparam PIPE_DEPTH = 22;

    reg [31:0] px_pipe   [0:PIPE_DEPTH-1];
    reg [31:0] py_pipe   [0:PIPE_DEPTH-1];
    reg [15:0] dist_pipe [0:PIPE_DEPTH-1];

    wire [31:0] s_px   = px_pipe[PIPE_DEPTH-1];
    wire [31:0] s_py   = py_pipe[PIPE_DEPTH-1];
    wire [15:0] s_dist = dist_pipe[PIPE_DEPTH-1];

    // degrees to CORDIC phase (8192 = pi), heading added, wrapped to one turn
    localparam [21:0] DEG_TO_PHASE = 22'd2982616;
    wire [37:0] beam_q16    = lidar_angle * DEG_TO_PHASE + 38'd32768;
    wire [13:0] beam_phase  = beam_q16[29:16];
    wire [13:0] turn_phase  = pose_theta[13:0] + beam_phase;
    wire [15:0] total_angle = {{2{turn_phase[13]}}, turn_phase};

    wire [31:0] cordic_dout;   // {sin, cos}, 14 fraction bits each
    wire        cordic_valid;
    wire        cordic_ready;

    assign ready_out = cordic_ready;

    cordic_sin_cos u_cordic (
        .aclk(clk),
        .aresetn(rst_n),
        .s_axis_phase_tvalid(valid_in),
        .s_axis_phase_tdata(total_angle),
        .s_axis_phase_tready(cordic_ready),
        .m_axis_dout_tvalid(cordic_valid),
        .m_axis_dout_tready(1'b1),
        .m_axis_dout_tdata(cordic_dout)
    );

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < PIPE_DEPTH; i = i + 1) begin
                px_pipe[i] <= 0; py_pipe[i] <= 0; dist_pipe[i] <= 0;
            end
        end else begin
            px_pipe[0]   <= (valid_in && cordic_ready) ? pose_x     : px_pipe[0];
            py_pipe[0]   <= (valid_in && cordic_ready) ? pose_y     : py_pipe[0];
            dist_pipe[0] <= (valid_in && cordic_ready) ? lidar_dist : dist_pipe[0];
            for (i = 1; i < PIPE_DEPTH; i = i + 1) begin
                px_pipe[i]   <= px_pipe[i-1];
                py_pipe[i]   <= py_pipe[i-1];
                dist_pipe[i] <= dist_pipe[i-1];
            end
        end
    end

    wire signed [16:0] dist_s = $signed({1'b0, s_dist});
    wire signed [15:0] cos_s  = $signed(cordic_dout[15:0]);
    wire signed [15:0] sin_s  = $signed(cordic_dout[31:16]);
    wire signed [31:0] dcos   = dist_s * cos_s;
    wire signed [31:0] dsin   = dist_s * sin_s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            global_x <= 0; global_y <= 0;
            deriv_theta_x <= 0; deriv_theta_y <= 0;
            valid_out <= 0;
        end else if (cordic_valid) begin
            global_x      <= (dcos >>> 1) + $signed(s_px);
            global_y      <= (dsin >>> 1) + $signed(s_py);
            deriv_theta_x <= -(dsin >>> 1);
            deriv_theta_y <=  (dcos >>> 1);
            valid_out     <= 1'b1;
        end else begin
            valid_out <= 1'b0;
        end
    end

endmodule
