// Stage 1: scan point to map coordinates (Q14) and their heading derivatives, with a CORDIC core.
`timescale 1ns / 1ps

module Stage1_Coord_Calc (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] pose_x, pose_y, pose_theta,
    input  wire [15:0] lidar_dist,
    input  wire [15:0] lidar_angle,
    input  wire        valid_in,
    output wire        ready_out,  // CORDIC ready, to the top

    output reg  [31:0] global_x, global_y,
    output reg  [31:0] deriv_theta_x, deriv_theta_y,
    output reg         valid_out
);

    // IP latency = 22
    localparam PIPE_DEPTH = 22;

    reg [31:0] px_pipe [0:PIPE_DEPTH-1];
    reg [31:0] py_pipe [0:PIPE_DEPTH-1];
    reg [15:0] dist_pipe [0:PIPE_DEPTH-1];

    wire [31:0] s_px   = px_pipe[PIPE_DEPTH-1];
    wire [31:0] s_py   = py_pipe[PIPE_DEPTH-1];
    wire [15:0] s_dist = dist_pipe[PIPE_DEPTH-1];

    wire [31:0] cordic_dout;
    wire        cordic_valid;
    wire        cordic_ready;
    // beam angle in degrees to 65536 per turn; the heading uses the same unit, and the sum wraps at one turn
    wire signed [31:0] scaled_lidar_val = $signed({1'b0, lidar_angle}) * 16'd182;
    wire signed [15:0] scaled_lidar_angle = scaled_lidar_val[15:0];
    wire [15:0] turn_angle = pose_theta[15:0] + scaled_lidar_angle;
    // CORDIC phase: 8192 is pi, so one turn is 16384
    wire [15:0] total_angle = {{2{turn_angle[15]}}, turn_angle[15:2]};

    assign ready_out = cordic_ready;  // accept input only when the IP is ready

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

    // the pipeline moves only when the IP takes data
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<PIPE_DEPTH; i=i+1) begin
                px_pipe[i] <= 0; py_pipe[i] <= 0; dist_pipe[i] <= 0;
            end
        end else begin
            px_pipe[0] <= (valid_in && cordic_ready) ? pose_x : px_pipe[0];
            py_pipe[0] <= (valid_in && cordic_ready) ? pose_y : py_pipe[0];
            dist_pipe[0] <= (valid_in && cordic_ready) ? lidar_dist : dist_pipe[0];
        for (i=1; i<PIPE_DEPTH; i=i+1) begin
            px_pipe[i] <= px_pipe[i-1];
            py_pipe[i] <= py_pipe[i-1];
            dist_pipe[i] <= dist_pipe[i-1];
        end
      end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            global_x <= 0; global_y <= 0;
        end else if (cordic_valid) begin
            // global_x, y (Q14), signed throughout
            global_x <= (($signed({1'b0, s_dist}) * $signed(cordic_dout[15:0])) >>> 1) + $signed(s_px);
            global_y <= (($signed({1'b0, s_dist}) * $signed(cordic_dout[31:16])) >>> 1) + $signed(s_py);
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            deriv_theta_x <= 0;
            deriv_theta_y <= 0;
            valid_out     <= 0;
        end else if (cordic_valid) begin
            // rotation derivative in Q14 as well
            deriv_theta_x <= -($signed({1'b0, s_dist}) * $signed(cordic_dout[31:16]) >>> 1);
            deriv_theta_y <=  ($signed({1'b0, s_dist}) * $signed(cordic_dout[15:0]) >>> 1);

            valid_out <= 1'b1;
        end else begin
            valid_out <= 1'b0;
        end
    end

endmodule
