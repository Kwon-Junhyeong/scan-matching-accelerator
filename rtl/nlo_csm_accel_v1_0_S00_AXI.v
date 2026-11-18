// AXI4-Lite slave: pose, threshold and map settings in, the sums H and g out.
`timescale 1 ns / 1 ps

module nlo_csm_accel_v1_0_S00_AXI # (
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 7
) (
    // User Ports
    output wire o_start,
    output wire [31:0] o_pose_x, o_pose_y, o_pose_theta,
    output wire [31:0] o_tau_threshold, o_map_base_addr, o_map_width, o_map_height,
    input wire [31:0] i_point_cnt,
    // Result Inputs (64-bit)
    input wire [63:0] i_H11, i_H12, i_H13, i_H22, i_H23, i_H33,
    input wire [63:0] i_K1,  i_K2,  i_K3,
    input  wire [1:0] i_current_state,
    input wire        i_st1_ready,
    input wire        i_st2_ready,
    // AXI-Lite Ports
    input wire  S_AXI_ACLK,
    input wire  S_AXI_ARESETN,
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
    input wire [2 : 0] S_AXI_AWPROT,
    input wire  S_AXI_AWVALID,
    output wire  S_AXI_AWREADY,
    input wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
    input wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    input wire  S_AXI_WVALID,
    output wire  S_AXI_WREADY,
    output wire [1 : 0] S_AXI_BRESP,
    output wire  S_AXI_BVALID,
    input wire  S_AXI_BREADY,
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
    input wire [2 : 0] S_AXI_ARPROT,
    input wire  S_AXI_ARVALID,
    output wire  S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
    output wire [1 : 0] S_AXI_RRESP,
    output wire  S_AXI_RVALID,
    input wire  S_AXI_RREADY
);
    // AXI Signals
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
    reg axi_awready, axi_wready, axi_bvalid, axi_arready, axi_rvalid;
    reg [C_S_AXI_DATA_WIDTH-1 : 0] axi_rdata;

    // Registers
    // 00: Control (Start), 04-0C: Pose, 10: Tau, 14-1C: Map Info
    reg [31:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3, slv_reg4, slv_reg5, slv_reg6, slv_reg7;

    // Address Decoding (Word Aligned, [6:2])
    // 7-bit Address -> Max 128 Bytes -> 32 Registers (Index 0 to 31)
    wire [4:0] wr_addr_index = axi_awaddr[6:2];
    wire [4:0] rd_addr_index = axi_araddr[6:2];

    // I/O Connections
    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_BRESP   = 2'b00;  // OKAY Response
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RVALID  = axi_rvalid;
    assign S_AXI_RRESP   = 2'b00;  // OKAY Response

    assign o_start         = slv_reg0[0];
    assign o_pose_x        = slv_reg1;
    assign o_pose_y        = slv_reg2;
    assign o_pose_theta    = slv_reg3;
    assign o_tau_threshold = slv_reg4;
    assign o_map_base_addr = slv_reg5;
    assign o_map_width     = slv_reg6;
    assign o_map_height    = slv_reg7;

    wire slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

    // 1. Write Logic (Configuration)
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_awready <= 0; axi_wready <= 0; axi_bvalid <= 0;
            axi_awaddr <= 0;
            slv_reg0 <= 0; slv_reg1 <= 0; slv_reg2 <= 0; slv_reg3 <= 0;
            slv_reg4 <= 32'h0000_000A;  // Default Tau
            slv_reg5 <= 0; slv_reg6 <= 0; slv_reg7 <= 0;
        end else begin
            // Write Address Handshake
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) begin
                axi_awready <= 1'b1;
                axi_awaddr  <= S_AXI_AWADDR;
            end else axi_awready <= 1'b0;

            // Write Data Handshake
            if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID) axi_wready <= 1'b1;
            else axi_wready <= 1'b0;

            // Actual Register Write
            if (slv_reg_wren) begin
                if (S_AXI_WSTRB[0]) begin
                    case (wr_addr_index)
                        5'h00: slv_reg0[7:0] <= S_AXI_WDATA[7:0];
                        5'h01: slv_reg1[7:0] <= S_AXI_WDATA[7:0];
                        5'h02: slv_reg2[7:0] <= S_AXI_WDATA[7:0];
                        5'h03: slv_reg3[7:0] <= S_AXI_WDATA[7:0];
                        5'h04: slv_reg4[7:0] <= S_AXI_WDATA[7:0];
                        5'h05: slv_reg5[7:0] <= S_AXI_WDATA[7:0];
                        5'h06: slv_reg6[7:0] <= S_AXI_WDATA[7:0];
                        5'h07: slv_reg7[7:0] <= S_AXI_WDATA[7:0];
                    endcase
                end
                if (S_AXI_WSTRB[1]) begin
                    case (wr_addr_index)
                        5'h00: slv_reg0[15:8] <= S_AXI_WDATA[15:8];
                        5'h01: slv_reg1[15:8] <= S_AXI_WDATA[15:8];
                        5'h02: slv_reg2[15:8] <= S_AXI_WDATA[15:8];
                        5'h03: slv_reg3[15:8] <= S_AXI_WDATA[15:8];
                        5'h04: slv_reg4[15:8] <= S_AXI_WDATA[15:8];
                        5'h05: slv_reg5[15:8] <= S_AXI_WDATA[15:8];
                        5'h06: slv_reg6[15:8] <= S_AXI_WDATA[15:8];
                        5'h07: slv_reg7[15:8] <= S_AXI_WDATA[15:8];
                    endcase
                end
                if (S_AXI_WSTRB[2]) begin
                    case (wr_addr_index)
                        5'h00: slv_reg0[23:16] <= S_AXI_WDATA[23:16];
                        5'h01: slv_reg1[23:16] <= S_AXI_WDATA[23:16];
                        5'h02: slv_reg2[23:16] <= S_AXI_WDATA[23:16];
                        5'h03: slv_reg3[23:16] <= S_AXI_WDATA[23:16];
                        5'h04: slv_reg4[23:16] <= S_AXI_WDATA[23:16];
                        5'h05: slv_reg5[23:16] <= S_AXI_WDATA[23:16];
                        5'h06: slv_reg6[23:16] <= S_AXI_WDATA[23:16];
                        5'h07: slv_reg7[23:16] <= S_AXI_WDATA[23:16];
                    endcase
                end
                if (S_AXI_WSTRB[3]) begin
                    case (wr_addr_index)
                        5'h00: slv_reg0[31:24] <= S_AXI_WDATA[31:24];
                        5'h01: slv_reg1[31:24] <= S_AXI_WDATA[31:24];
                        5'h02: slv_reg2[31:24] <= S_AXI_WDATA[31:24];
                        5'h03: slv_reg3[31:24] <= S_AXI_WDATA[31:24];
                        5'h04: slv_reg4[31:24] <= S_AXI_WDATA[31:24];
                        5'h05: slv_reg5[31:24] <= S_AXI_WDATA[31:24];
                        5'h06: slv_reg6[31:24] <= S_AXI_WDATA[31:24];
                        5'h07: slv_reg7[31:24] <= S_AXI_WDATA[31:24];
                    endcase
                end
            end

            // Write Response
            if (axi_awready && S_AXI_AWVALID && axi_wready && S_AXI_WVALID && ~axi_bvalid)
                axi_bvalid <= 1'b1;
            else if (S_AXI_BREADY && axi_bvalid)
                axi_bvalid <= 1'b0;
        end
    end

    // 2. Read Logic (Status & Results)
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
                    // 1. Settings (0x00 to 0x1C)
                    5'h00: axi_rdata <= {i_point_cnt[15:0], 10'h0, i_st2_ready, i_st1_ready, i_current_state, 1'b0, slv_reg0[0]};
                    5'h01: axi_rdata <= slv_reg1;  // Pose X
                    5'h02: axi_rdata <= slv_reg2;  // Pose Y
                    5'h03: axi_rdata <= slv_reg3;  // Pose Theta
                    5'h04: axi_rdata <= slv_reg4;  // Tau Threshold
                    5'h05: axi_rdata <= slv_reg5;  // Map Base Addr
                    5'h06: axi_rdata <= slv_reg6;  // Map Width
                    5'h07: axi_rdata <= slv_reg7;  // Map Height

                    // 2. Results (0x20 to 0x64)
                    5'h08: axi_rdata <= i_H11[31:0];  // 0x20
                    5'h09: axi_rdata <= i_H11[63:32];  // 0x24
                    5'h0A: axi_rdata <= i_H12[31:0];  // 0x28
                    5'h0B: axi_rdata <= i_H12[63:32];  // 0x2C
                    5'h0C: axi_rdata <= i_H13[31:0];  // 0x30
                    5'h0D: axi_rdata <= i_H13[63:32];  // 0x34
                    5'h0E: axi_rdata <= i_H22[31:0];  // 0x38
                    5'h0F: axi_rdata <= i_H22[63:32];  // 0x3C
                    5'h10: axi_rdata <= i_H23[31:0];  // 0x40
                    5'h11: axi_rdata <= i_H23[63:32];  // 0x44
                    5'h12: axi_rdata <= i_H33[31:0];  // 0x48
                    5'h13: axi_rdata <= i_H33[63:32];  // 0x4C
                    5'h14: axi_rdata <= i_K1[31:0];  // 0x50
                    5'h15: axi_rdata <= i_K1[63:32];  // 0x54
                    5'h16: axi_rdata <= i_K2[31:0];  // 0x58
                    5'h17: axi_rdata <= i_K2[63:32];  // 0x5C
                    5'h18: axi_rdata <= i_K3[31:0];  // 0x60
                    5'h19: axi_rdata <= i_K3[63:32];  // 0x64

                default: axi_rdata <= 32'd0;
              endcase
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end
endmodule
