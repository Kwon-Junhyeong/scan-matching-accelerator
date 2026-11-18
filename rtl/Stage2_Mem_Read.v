// Stage 2: the four map cells around a point, read from block RAM one after another.
`timescale 1ns / 1ps

module Stage2_Mem_Read (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] i_map_base_addr,
    input  wire [31:0] i_map_width,
    input  wire [31:0] i_map_height,
    output wire        o_ready,

    input  wire [31:0] st1_global_x,
    input  wire [31:0] st1_global_y,
    input  wire [31:0] st1_deriv_x,
    input  wire [31:0] st1_deriv_y,
    input  wire        st1_valid,

    output reg  [31:0] bram_addr,
    output reg         bram_en,
    output wire [3:0]  bram_we,
    input  wire [31:0] bram_rdata,

    output reg  [7:0]  m_p00, m_p10, m_p01, m_p11,
    output reg  [31:0] frac_x, frac_y,
    output reg  [31:0] st2_deriv_x, st2_deriv_y,
    output reg         st2_valid
);

    localparam S_IDLE  = 3'd0, S_REQ_1 = 3'd1, S_REQ_2 = 3'd2,
               S_REQ_3 = 3'd3, S_WAIT  = 3'd4, S_DONE  = 3'd5;
    localparam READ_LATENCY = 3'd5;

    reg [2:0] state;
    reg [31:0] r_base, r_fx, r_fy, r_dx, r_dy;
    reg r_in;
    reg [2:0] wait_cnt;
    assign bram_we = 4'b0000;

    wire signed [31:0] x_idx = $signed(st1_global_x) >>> 14;
    wire signed [31:0] y_idx = $signed(st1_global_y) >>> 14;

    wire inside_w = (x_idx >= 0 && x_idx < ($signed(i_map_width) - 1)) &&
                    (y_idx >= 0 && y_idx < ($signed(i_map_height) - 1));

    wire [31:0] base_idx_w = (y_idx * i_map_width) + x_idx;
    wire [23:0] unused_bram_data = bram_rdata[31:8];

    assign o_ready = (state == S_IDLE);

    always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= S_IDLE;
        wait_cnt <= 0;
        bram_en <= 0;
        st2_valid <= 0;
        bram_addr <= 0;
        {m_p00, m_p10, m_p01, m_p11} <= 0;
        {frac_x, frac_y} <= 0;
        {st2_deriv_x, st2_deriv_y} <= 0;
        {r_base, r_fx, r_fy, r_dx, r_dy} <= 0;
        r_in <= 0;
    end else begin
        st2_valid <= 0;
        bram_en   <= 0;  // Default off

        case (state)
            S_IDLE: begin
                if (st1_valid) begin
                    r_base <= base_idx_w;
                    r_in   <= inside_w;
                    r_fx   <= st1_global_x & 32'h3FFF;
                    r_fy   <= st1_global_y & 32'h3FFF;
                    r_dx   <= st1_deriv_x;
                    r_dy   <= st1_deriv_y;

                    if (inside_w) begin
                        bram_addr <= i_map_base_addr + (base_idx_w << 2);
                        bram_en   <= 1;
                    end
                    wait_cnt <= 0;
                    state <= S_REQ_1;
                end
            end

            S_REQ_1: begin
                // wait for p00
                if (wait_cnt < READ_LATENCY) begin
                    wait_cnt <= wait_cnt + 1;
                end else begin
                    if (r_in) m_p00 <= bram_rdata[7:0];
                    else      m_p00 <= 0;

                    if (r_in) begin
                        bram_addr <= i_map_base_addr + ((r_base + 1) << 2);
                        bram_en   <= 1;
                    end
                    wait_cnt <= 0;
                    state <= S_REQ_2;
                end
            end

            S_REQ_2: begin
                // wait for p10
                if (wait_cnt < READ_LATENCY) begin
                    wait_cnt <= wait_cnt + 1;
                end else begin
                    if (r_in) m_p10 <= bram_rdata[7:0];
                    else      m_p10 <= 0;

                    if (r_in) begin
                        bram_addr <= i_map_base_addr + ((r_base + i_map_width) << 2);
                        bram_en   <= 1;
                    end
                    wait_cnt <= 0;
                    state <= S_REQ_3;
                end
            end

            S_REQ_3: begin
                // wait for p01
                if (wait_cnt < READ_LATENCY) begin
                    wait_cnt <= wait_cnt + 1;
                end else begin
                    if (r_in) m_p01 <= bram_rdata[7:0];
                    else      m_p01 <= 0;

                    if (r_in) begin
                        bram_addr <= i_map_base_addr + ((r_base + i_map_width + 1) << 2);
                        bram_en   <= 1;
                    end
                    wait_cnt <= 0;
                    state <= S_WAIT;
                end
            end

            S_WAIT: begin
                // wait for p11
                if (wait_cnt < READ_LATENCY) begin
                    wait_cnt <= wait_cnt + 1;
                end else begin
                    if (r_in) m_p11 <= bram_rdata[7:0];
                    else      m_p11 <= 0;

                    state <= S_DONE;
                end
            end

            S_DONE: begin
                // output after all four reads
                frac_x <= r_fx;
                frac_y <= r_fy;
                st2_deriv_x <= r_dx;
                st2_deriv_y <= r_dy;
                st2_valid <= 1;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
     end
  end
endmodule
