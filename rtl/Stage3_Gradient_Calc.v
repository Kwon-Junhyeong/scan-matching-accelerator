`timescale 1ns / 1ps

// Stage 3: bilinear map value M and its slope at the point, residual f = 255 - M,
// and the gate M >= tau.
// Fractions in: 14 bits. Slope out: 14 fraction bits. M, f and tau: 16 fraction bits.
module Stage3_Gradient_Calc #(
    parameter integer Q14_ONE = 16384
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  p00, p10, p01, p11,
    input  wire [31:0] frac_x, frac_y,
    input  wire [31:0] st2_deriv_x, st2_deriv_y,
    input  wire        st2_valid,

    input  wire [31:0] tau_threshold,

    output reg  signed [31:0] grad_x, grad_y,
    output reg  signed [31:0] score_f,
    output reg                point_valid_flag,
    output reg  [31:0]        st3_deriv_x, st3_deriv_y,
    output reg                st3_valid
);

    localparam signed [31:0] MAX_SCORE_Q16 = 32'sh00FF0000;   // 255.0

    reg [7:0]  r1_p00, r1_p10, r1_p01, r1_p11;
    reg [31:0] r1_fx, r1_fy, r1_dx, r1_dy;
    reg        r1_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {r1_p00, r1_p10, r1_p01, r1_p11} <= 0;
            {r1_fx, r1_fy, r1_dx, r1_dy} <= 0;
            r1_v <= 0;
        end else begin
            r1_p00 <= p00; r1_p10 <= p10;
            r1_p01 <= p01; r1_p11 <= p11;
            r1_fx <= frac_x; r1_fy <= frac_y;
            r1_dx <= st2_deriv_x; r1_dy <= st2_deriv_y;
            r1_v  <= st2_valid;
        end
    end

    wire [31:0] inv_fx = Q14_ONE - r1_fx;
    wire [31:0] inv_fy = Q14_ONE - r1_fy;

    // cell value x Q14 x Q14 = 28 fraction bits
    wire signed [39:0] t0 = $signed({1'b0, r1_p00}) * inv_fx * inv_fy;
    wire signed [39:0] t1 = $signed({1'b0, r1_p10}) * r1_fx  * inv_fy;
    wire signed [39:0] t2 = $signed({1'b0, r1_p01}) * inv_fx * r1_fy;
    wire signed [39:0] t3 = $signed({1'b0, r1_p11}) * r1_fx  * r1_fy;
    wire signed [39:0] m_val_q28 = t0 + t1 + t2 + t3;

    // 28 -> 16 fraction bits; 24 bits would overflow the 32-bit word at M = 255
    wire signed [31:0] m_val_q16 = m_val_q28 >>> 12;

    wire signed [15:0] d00_10 = $signed({1'b0, r1_p10}) - $signed({1'b0, r1_p00});
    wire signed [15:0] d01_11 = $signed({1'b0, r1_p11}) - $signed({1'b0, r1_p01});
    wire signed [15:0] d00_01 = $signed({1'b0, r1_p01}) - $signed({1'b0, r1_p00});
    wire signed [15:0] d10_11 = $signed({1'b0, r1_p11}) - $signed({1'b0, r1_p10});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grad_x <= 0; grad_y <= 0;
            score_f <= 0;
            point_valid_flag <= 0;
            st3_deriv_x <= 0; st3_deriv_y <= 0;
            st3_valid <= 0;
        end else begin
            grad_x <= (d00_10 * $signed({1'b0, inv_fy})) + (d01_11 * $signed({1'b0, r1_fy}));
            grad_y <= (d00_01 * $signed({1'b0, inv_fx})) + (d10_11 * $signed({1'b0, r1_fx}));
            score_f <= MAX_SCORE_Q16 - m_val_q16;
            point_valid_flag <= (m_val_q16 >= $signed(tau_threshold));
            st3_deriv_x <= r1_dx;
            st3_deriv_y <= r1_dy;
            st3_valid   <= r1_v;
        end
    end
endmodule
