// Stage 5: H += J^T J and g += J^T f in 64-bit accumulators.
`timescale 1ns / 1ps

module Stage5_Matrix_MAC (
    input wire clk,
    input wire rst_n,
    input wire frame_start,

    // from stage 4
    input wire signed [31:0] j_i1, j_i2, j_i3,  // Q14
    input wire signed [31:0] f_i,  // Q24
    input wire                point_valid_in,
    input wire                st4_valid,

    // outputs
    output reg signed [63:0] H11, H12, H13, H22, H23, H33,
    output reg signed [63:0] K1, K2, K3,
    input wire [1:0] current_state
);

    // Pipeline Stage 1: Multiplication
    reg signed [63:0] p_h11, p_h12, p_h13, p_h22, p_h23, p_h33;
    reg signed [63:0] p_k1, p_k2, p_k3;
    reg                v_d1, pv_d1, fs_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {p_h11, p_h12, p_h13, p_h22, p_h23, p_h33} <= 0;
            {p_k1, p_k2, p_k3} <= 0;
            {v_d1, pv_d1, fs_d1} <= 0;
        end else begin
            // Hessian (Q14 * Q14 = Q28)
            p_h11 <= $signed(j_i1) * $signed(j_i1);
            p_h12 <= $signed(j_i1) * $signed(j_i2);
            p_h13 <= $signed(j_i1) * $signed(j_i3);
            p_h22 <= $signed(j_i2) * $signed(j_i2);
            p_h23 <= $signed(j_i2) * $signed(j_i3);
            p_h33 <= $signed(j_i3) * $signed(j_i3);

            // Gradient (Q14 * Q24 = Q38 -> Q28 normalization)
            p_k1  <= ($signed(j_i1) * $signed(f_i)) >>> 10;
            p_k2  <= ($signed(j_i2) * $signed(f_i)) >>> 10;
            p_k3  <= ($signed(j_i3) * $signed(f_i)) >>> 10;

            v_d1  <= st4_valid;
            pv_d1 <= point_valid_in;
            fs_d1 <= frame_start;
        end
    end

    // Pipeline Stage 2: Accumulation with First-Point Fix
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {H11, H12, H13, H22, H23, H33} <= 0;
            {K1, K2, K3} <= 0;
        end
        else if (current_state == 2'd0) begin  // reset in STATE_IDLE
            {H11, H12, H13, H22, H23, H33} <= 0;
            {K1, K2, K3} <= 0;
        end
        else if (v_d1 && pv_d1) begin  // accumulate valid points in STATE_RUN
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
