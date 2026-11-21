// Stage 4: Jacobian row [dM/dx, dM/dy, dM/dtheta].
`timescale 1ns / 1ps

module Stage4_Jacobian_Calc (
    input wire clk,
    input wire rst_n,

    // from stage 3
    input wire signed [31:0] grad_x,  // Q14
    input wire signed [31:0] grad_y,  // Q14
    input wire signed [31:0] score_f,  // Q16
    input wire               point_valid_in,
    input wire               st3_valid,

    // rotation derivative from stage 1 (-y_local, x_local)
    input wire signed [31:0] synced_deriv_x,
    input wire signed [31:0] synced_deriv_y,

    // to stage 5
    output reg signed [31:0] j_i1, j_i2, j_i3,
    output reg signed [31:0] st4_score,
    output reg               point_valid_out,
    output reg               st4_valid
);

    localparam SHIFT = 14;

    // Pipeline Stage 1: Multiplication
    reg signed [63:0] r_mult_x, r_mult_y;
    reg signed [31:0] r_grad_x, r_grad_y, r_score;
    reg               r_pv, r_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_mult_x <= 0; r_mult_y <= 0;
            r_grad_x <= 0; r_grad_y <= 0;
            r_score <= 0;  r_pv <= 0; r_v <= 0;
        end else begin
            // J3: dM/dx * dx/dtheta + dM/dy * dy/dtheta
            r_mult_x <= grad_x * synced_deriv_x;
            r_mult_y <= grad_y * synced_deriv_y;

            r_grad_x <= grad_x;
            r_grad_y <= grad_y;
            r_score  <= score_f;
            r_pv     <= point_valid_in;
            r_v      <= st3_valid;
        end
    end

    // Pipeline Stage 2: Summation
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {j_i1, j_i2, j_i3} <= 0;
            st4_score <= 0;
            point_valid_out <= 0;
            st4_valid <= 0;
        end else begin
            j_i1 <= r_grad_x;  // J1 = dM/dx (Q14)
            j_i2 <= r_grad_y;  // J2 = dM/dy (Q14)

            // J3 (Q14): Q28 to Q14
            j_i3 <= (r_mult_x >>> SHIFT) + (r_mult_y >>> SHIFT);

            st4_score       <= r_score;  // f_i (Q16)
            point_valid_out <= r_pv;
            st4_valid       <= r_v;
        end
    end
endmodule
