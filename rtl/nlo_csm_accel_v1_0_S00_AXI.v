`timescale 1 ns / 1 ps

// AXI4-Lite slave.
// Write: 0x00 control (bit 0 start), 0x04 pose x, 0x08 pose y, 0x0C heading,
//        0x10 tau, 0x14 map base, 0x18 map width, 0x1C map height.
// Read:  0x00 status, 0x04..0x1C the settings above,
//        0x20..0x4C H11 H12 H13 H22 H23 H33, 0x50..0x64 K1 K2 K3 (64 bit each, low word first).
module nlo_csm_accel_v1_0_S00_AXI #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 7
)(
    output wire        o_start,
    output wire [31:0] o_pose_x, o_pose_y, o_pose_theta,
    output wire [31:0] o_tau_threshold, o_map_base_addr, o_map_width, o_map_height,
    input  wire [31:0] i_point_cnt,
    input  wire [63:0] i_H11, i_H12, i_H13, i_H22, i_H23, i_H33,
    input  wire [63:0] i_K1,  i_K2,  i_K3,
    input  wire [1:0]  i_current_state,
    input  wire        i_st1_ready,
    input  wire        i_st2_ready,

    input  wire S_AXI_ACLK,
    input  wire S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
    input  wire [2 : 0] S_AXI_AWPROT,
    input  wire S_AXI_AWVALID,
    output wire S_AXI_AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    input  wire S_AXI_WVALID,
    output wire S_AXI_WREADY,
    output wire [1 : 0] S_AXI_BRESP,
    output wire S_AXI_BVALID,
    input  wire S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
    input  wire [2 : 0] S_AXI_ARPROT,
    input  wire S_AXI_ARVALID,
    output wire S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
    output wire [1 : 0] S_AXI_RRESP,
    output wire S_AXI_RVALID,
    input  wire S_AXI_RREADY
);
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr, axi_araddr;
    reg axi_awready, axi_wready, axi_bvalid, axi_arready, axi_rvalid;
    reg [C_S_AXI_DATA_WIDTH-1 : 0] axi_rdata;

    reg [31:0] slv_reg [0:7];

    wire [4:0] wr_addr_index = axi_awaddr[6:2];
    wire [4:0] rd_addr_index = axi_araddr[6:2];

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_BRESP   = 2'b00;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RVALID  = axi_rvalid;
    assign S_AXI_RRESP   = 2'b00;

    assign o_start         = slv_reg[0][0];
    assign o_pose_x        = slv_reg[1];
    assign o_pose_y        = slv_reg[2];
    assign o_pose_theta    = slv_reg[3];
    assign o_tau_threshold = slv_reg[4];
    assign o_map_base_addr = slv_reg[5];
    assign o_map_width     = slv_reg[6];
    assign o_map_height    = slv_reg[7];

    wire slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

    integer r, b;
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_awready <= 0; axi_wready <= 0; axi_bvalid <= 0;
            axi_awaddr <= 0;
            for (r = 0; r < 8; r = r + 1) slv_reg[r] <= 0;
            slv_reg[4] <= 32'h0000_000A;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) begin
                axi_awready <= 1'b1;
                axi_awaddr  <= S_AXI_AWADDR;
            end else axi_awready <= 1'b0;

            if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID) axi_wready <= 1'b1;
            else axi_wready <= 1'b0;

            if (slv_reg_wren && wr_addr_index < 8)
                for (b = 0; b < 4; b = b + 1)
                    if (S_AXI_WSTRB[b]) slv_reg[wr_addr_index][b*8 +: 8] <= S_AXI_WDATA[b*8 +: 8];

            if (axi_awready && S_AXI_AWVALID && axi_wready && S_AXI_WVALID && ~axi_bvalid)
                axi_bvalid <= 1'b1;
            else if (S_AXI_BREADY && axi_bvalid)
                axi_bvalid <= 1'b0;
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_arready <= 0; axi_rvalid <= 0; axi_rdata <= 0; axi_araddr <= 0;
        end else begin
            if (~axi_arready && S_AXI_ARVALID) begin
                axi_arready <= 1'b1;
                axi_araddr  <= S_AXI_ARADDR;
            end else axi_arready <= 1'b0;

            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                case (rd_addr_index)
                    // status: point count, stage 2 ready, stage 1 ready, pass state, start
                    5'h00: axi_rdata <= {i_point_cnt[15:0], 10'h0, i_st2_ready, i_st1_ready,
                                         i_current_state, 1'b0, slv_reg[0][0]};
                    5'h01: axi_rdata <= slv_reg[1];
                    5'h02: axi_rdata <= slv_reg[2];
                    5'h03: axi_rdata <= slv_reg[3];
                    5'h04: axi_rdata <= slv_reg[4];
                    5'h05: axi_rdata <= slv_reg[5];
                    5'h06: axi_rdata <= slv_reg[6];
                    5'h07: axi_rdata <= slv_reg[7];
                    5'h08: axi_rdata <= i_H11[31:0];
                    5'h09: axi_rdata <= i_H11[63:32];
                    5'h0A: axi_rdata <= i_H12[31:0];
                    5'h0B: axi_rdata <= i_H12[63:32];
                    5'h0C: axi_rdata <= i_H13[31:0];
                    5'h0D: axi_rdata <= i_H13[63:32];
                    5'h0E: axi_rdata <= i_H22[31:0];
                    5'h0F: axi_rdata <= i_H22[63:32];
                    5'h10: axi_rdata <= i_H23[31:0];
                    5'h11: axi_rdata <= i_H23[63:32];
                    5'h12: axi_rdata <= i_H33[31:0];
                    5'h13: axi_rdata <= i_H33[63:32];
                    5'h14: axi_rdata <= i_K1[31:0];
                    5'h15: axi_rdata <= i_K1[63:32];
                    5'h16: axi_rdata <= i_K2[31:0];
                    5'h17: axi_rdata <= i_K2[63:32];
                    5'h18: axi_rdata <= i_K3[31:0];
                    5'h19: axi_rdata <= i_K3[63:32];
                    default: axi_rdata <= 32'd0;
                endcase
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end
endmodule
