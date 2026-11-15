`timescale 1ns / 1ps

// Stage 5: H += J^T J (6 terms) and g += J^T f (3 terms) in 64-bit accumulators.
// H11..H23: 28 fraction bits. H33: 20 fraction bits (J3 x J3 >>> 8), so that 360 points at up to
// 361 cells, the diagonal of a 256 x 256 map, cannot overflow 64 bits. g: 14 + 16 - 10 = 20 fraction bits.
// Cleared while the pass is idle.
module Stage5_Matrix_MAC (
    input  wire clk,
    input  wire rst_n,

    input  wire signed [31:0] j_i1, j_i2, j_i3,
    input  wire signed [31:0] f_i,
    input  wire               point_valid_in,
    input  wire               st4_valid,

    output reg  signed [63:0] H11, H12, H13, H22, H23, H33,
    output reg  signed [63:0] K1, K2, K3,
    input  wire [1:0]         current_state
);

    reg signed [63:0] p_h11, p_h12, p_h13, p_h22, p_h23, p_h33;
    reg signed [63:0] p_k1, p_k2, p_k3;
    reg               v_d1, pv_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {p_h11, p_h12, p_h13, p_h22, p_h23, p_h33} <= 0;
            {p_k1, p_k2, p_k3} <= 0;
            {v_d1, pv_d1} <= 0;
        end else begin
            p_h11 <= $signed(j_i1) * $signed(j_i1);
            p_h12 <= $signed(j_i1) * $signed(j_i2);
            p_h13 <= $signed(j_i1) * $signed(j_i3);
            p_h22 <= $signed(j_i2) * $signed(j_i2);
            p_h23 <= $signed(j_i2) * $signed(j_i3);
            p_h33 <= ($signed(j_i3) * $signed(j_i3)) >>> 8;
            p_k1  <= ($signed(j_i1) * $signed(f_i)) >>> 10;
            p_k2  <= ($signed(j_i2) * $signed(f_i)) >>> 10;
            p_k3  <= ($signed(j_i3) * $signed(f_i)) >>> 10;
            v_d1  <= st4_valid;
            pv_d1 <= point_valid_in;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {H11, H12, H13, H22, H23, H33} <= 0;
            {K1, K2, K3} <= 0;
        end else if (current_state == 2'd0) begin
            {H11, H12, H13, H22, H23, H33} <= 0;
            {K1, K2, K3} <= 0;
        end else if (v_d1 && pv_d1) begin
            H11 <= H11 + p_h11;
            H12 <= H12 + p_h12;
            H13 <= H13 + p_h13;
            H22 <= H22 + p_h22;
            H23 <= H23 + p_h23;
            H33 <= H33 + p_h33;
            K1  <= K1 + p_k1;
            K2  <= K2 + p_k2;
            K3  <= K3 + p_k3;
        end
    end
endmodule
