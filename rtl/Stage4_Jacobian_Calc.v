`timescale 1ns / 1ps

// Stage 4: Jacobian row J = [dM/dx, dM/dy, dM/dtheta].
module Stage4_Jacobian_Calc (
    input  wire clk,
    input  wire rst_n,

    input  wire signed [31:0] grad_x,
    input  wire signed [31:0] grad_y,
    input  wire signed [31:0] score_f,
    input  wire               point_valid_in,
    input  wire               st3_valid,

    input  wire signed [31:0] synced_deriv_x,
    input  wire signed [31:0] synced_deriv_y,

    output reg  signed [31:0] j_i1, j_i2, j_i3,
    output reg  signed [31:0] st4_score,
    output reg                point_valid_out,
    output reg                st4_valid
);

    localparam SHIFT = 14;

    reg signed [63:0] r_mult_x, r_mult_y;
    reg signed [31:0] r_grad_x, r_grad_y, r_score;
    reg               r_pv, r_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_mult_x <= 0; r_mult_y <= 0;
            r_grad_x <= 0; r_grad_y <= 0;
            r_score <= 0;  r_pv <= 0; r_v <= 0;
        end else begin
            r_mult_x <= grad_x * synced_deriv_x;
            r_mult_y <= grad_y * synced_deriv_y;
            r_grad_x <= grad_x;
            r_grad_y <= grad_y;
            r_score  <= score_f;
            r_pv     <= point_valid_in;
            r_v      <= st3_valid;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {j_i1, j_i2, j_i3} <= 0;
            st4_score <= 0;
            point_valid_out <= 0;
            st4_valid <= 0;
        end else begin
            j_i1 <= r_grad_x;
            j_i2 <= r_grad_y;
            j_i3 <= (r_mult_x >>> SHIFT) + (r_mult_y >>> SHIFT);
            st4_score       <= r_score;
            point_valid_out <= r_pv;
            st4_valid       <= r_v;
        end
    end
endmodule
