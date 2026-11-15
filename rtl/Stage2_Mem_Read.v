`timescale 1ns / 1ps

// Stage 2: reads the four map cells around a point through one block RAM port, one read per clock,
// so a new point can start every 4 clocks. The data of a read is taken READ_LATENCY + 1 clocks after
// its address is registered. A point outside the map reads nothing and gets four zero cells.
module Stage2_Mem_Read #(
    parameter integer READ_LATENCY = 1
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] i_map_base_addr,
    input  wire [31:0] i_map_width,
    input  wire [31:0] i_map_height,
    output wire        o_ready,
    output wire        o_idle,

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

    localparam integer SD = READ_LATENCY + 1;

    assign bram_we = 4'b0000;

    // cell index = coordinate >> 14, fraction = low 14 bits
    wire signed [31:0] x_idx = $signed(st1_global_x) >>> 14;
    wire signed [31:0] y_idx = $signed(st1_global_y) >>> 14;
    wire inside_w = (x_idx >= 0 && x_idx < ($signed(i_map_width)  - 1)) &&
                    (y_idx >= 0 && y_idx < ($signed(i_map_height) - 1));
    wire [31:0] base_idx_w = (y_idx * i_map_width) + x_idx;

    reg  [1:0]   ph;              // corner issued next: 0 p00 (and a new point), 1 p10, 2 p01, 3 p11
    reg  [31:0]  r_base;
    reg          r_in;
    reg  [127:0] r_meta;          // fraction x, y and the heading derivatives of the point

    wire         take     = (ph == 2'd0) && st1_valid;
    wire         issue    = take || (ph != 2'd0);
    wire         in_now   = take ? inside_w   : r_in;
    wire [31:0]  base_now = take ? base_idx_w : r_base;
    wire [31:0]  off_now  = (ph == 2'd0) ? 32'd0 :
                            (ph == 2'd1) ? 32'd1 :
                            (ph == 2'd2) ? i_map_width : i_map_width + 32'd1;

    // one entry per issued read, until its data is taken
    reg  [SD-1:0] t_v, t_in;
    reg  [1:0]    t_k    [0:SD-1];
    reg  [127:0]  t_meta [0:SD-1];
    reg  [7:0]    c00, c10, c01;

    wire [7:0] rd_cell = t_in[SD-1] ? bram_rdata[7:0] : 8'd0;

    assign o_ready = (ph == 2'd0);
    assign o_idle  = (ph == 2'd0) && (t_v == {SD{1'b0}});

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ph <= 0; r_base <= 0; r_in <= 0; r_meta <= 0;
            bram_addr <= 0; bram_en <= 0;
            t_v <= 0; t_in <= 0;
            for (i = 0; i < SD; i = i + 1) begin t_k[i] <= 0; t_meta[i] <= 0; end
            {c00, c10, c01} <= 0;
            {m_p00, m_p10, m_p01, m_p11} <= 0;
            {frac_x, frac_y, st2_deriv_x, st2_deriv_y} <= 0;
            st2_valid <= 0;
        end else begin
            bram_en   <= 0;
            st2_valid <= 0;

            if (take) begin
                r_base <= base_idx_w;
                r_in   <= inside_w;
                r_meta <= {st1_global_x & 32'h3FFF, st1_global_y & 32'h3FFF, st1_deriv_x, st1_deriv_y};
            end
            if (issue) begin
                if (in_now) begin
                    bram_addr <= i_map_base_addr + ((base_now + off_now) << 2);
                    bram_en   <= 1;
                end
                ph <= ph + 2'd1;
            end

            t_v[0]    <= issue;
            t_in[0]   <= in_now;
            t_k[0]    <= ph;
            t_meta[0] <= r_meta;
            for (i = 1; i < SD; i = i + 1) begin
                t_v[i]    <= t_v[i-1];
                t_in[i]   <= t_in[i-1];
                t_k[i]    <= t_k[i-1];
                t_meta[i] <= t_meta[i-1];
            end

            if (t_v[SD-1]) begin
                case (t_k[SD-1])
                    2'd0: c00 <= rd_cell;
                    2'd1: c10 <= rd_cell;
                    2'd2: c01 <= rd_cell;
                    2'd3: begin
                        m_p00 <= c00; m_p10 <= c10; m_p01 <= c01; m_p11 <= rd_cell;
                        {frac_x, frac_y, st2_deriv_x, st2_deriv_y} <= t_meta[SD-1];
                        st2_valid <= 1;
                    end
                endcase
            end
        end
    end
endmodule
