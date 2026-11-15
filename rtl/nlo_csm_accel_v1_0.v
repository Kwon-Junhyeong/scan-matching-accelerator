`timescale 1 ns / 1 ps

// Top of the accelerator IP: AXI4-Lite registers, AXI4-Stream scan input,
// a read port to the map in block RAM, and an interrupt at the end of a pass.
module nlo_csm_accel_v1_0 #(
    parameter integer C_S00_AXI_DATA_WIDTH   = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH   = 7,
    parameter integer C_S00_AXIS_TDATA_WIDTH = 32
)(
    output wire        irq,
    output wire [31:0] bram_addr,
    output wire        bram_en,
    output wire [3:0]  bram_we,
    input  wire [31:0] bram_rdata,
    output wire        bram_clk,
    output wire        bram_rst,

    input  wire s00_axi_aclk,
    input  wire s00_axi_aresetn,
    input  wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
    input  wire [2 : 0] s00_axi_awprot,
    input  wire s00_axi_awvalid,
    output wire s00_axi_awready,
    input  wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
    input  wire s00_axi_wvalid,
    output wire s00_axi_wready,
    output wire [1:0] s00_axi_bresp,
    output wire s00_axi_bvalid,
    input  wire s00_axi_bready,
    input  wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
    input  wire [2 : 0] s00_axi_arprot,
    input  wire s00_axi_arvalid,
    output wire s00_axi_arready,
    output wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
    output wire [1:0] s00_axi_rresp,
    output wire s00_axi_rvalid,
    input  wire s00_axi_rready,

    input  wire s00_axis_aclk,
    input  wire s00_axis_aresetn,
    output wire s00_axis_tready,
    input  wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata,
    input  wire s00_axis_tlast,
    input  wire s00_axis_tvalid
);
    wire        start_sig;
    wire [31:0] pose_x_sig, pose_y_sig, pose_theta_sig;
    wire [31:0] tau_threshold_sig, map_base_addr_sig, map_width_sig, map_height_sig;
    wire [63:0] H11_sig, H12_sig, H13_sig, H22_sig, H23_sig, H33_sig;
    wire [63:0] K1_sig, K2_sig, K3_sig;
    wire [31:0] stream_data_sig;
    wire        stream_valid_sig, stream_last_sig, stream_ready_sig;
    wire [1:0]  state_monitor;
    wire [31:0] point_cnt_sig;
    wire        st1_ready_sig, st2_ready_sig;

    assign bram_clk = s00_axi_aclk;
    assign bram_rst = ~s00_axi_aresetn;

    nlo_csm_accel_v1_0_S00_AXI inst_axi (
        .S_AXI_ACLK(s00_axi_aclk), .S_AXI_ARESETN(s00_axi_aresetn),
        .S_AXI_AWADDR(s00_axi_awaddr), .S_AXI_AWPROT(s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid), .S_AXI_AWREADY(s00_axi_awready),
        .S_AXI_WDATA(s00_axi_wdata), .S_AXI_WSTRB(s00_axi_wstrb),
        .S_AXI_WVALID(s00_axi_wvalid), .S_AXI_WREADY(s00_axi_wready),
        .S_AXI_BRESP(s00_axi_bresp), .S_AXI_BVALID(s00_axi_bvalid), .S_AXI_BREADY(s00_axi_bready),
        .S_AXI_ARADDR(s00_axi_araddr), .S_AXI_ARPROT(s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid), .S_AXI_ARREADY(s00_axi_arready),
        .S_AXI_RDATA(s00_axi_rdata), .S_AXI_RRESP(s00_axi_rresp),
        .S_AXI_RVALID(s00_axi_rvalid), .S_AXI_RREADY(s00_axi_rready),
        .o_start(start_sig),
        .o_pose_x(pose_x_sig), .o_pose_y(pose_y_sig), .o_pose_theta(pose_theta_sig),
        .o_tau_threshold(tau_threshold_sig), .o_map_base_addr(map_base_addr_sig),
        .o_map_width(map_width_sig), .o_map_height(map_height_sig),
        .i_point_cnt(point_cnt_sig), .i_current_state(state_monitor),
        .i_st1_ready(st1_ready_sig), .i_st2_ready(st2_ready_sig),
        .i_H11(H11_sig), .i_H12(H12_sig), .i_H13(H13_sig),
        .i_H22(H22_sig), .i_H23(H23_sig), .i_H33(H33_sig),
        .i_K1(K1_sig), .i_K2(K2_sig), .i_K3(K3_sig)
    );

    nlo_csm_accel_v1_0_S00_AXIS inst_axis (
        .S_AXIS_ACLK(s00_axis_aclk), .S_AXIS_ARESETN(s00_axis_aresetn),
        .S_AXIS_TREADY(s00_axis_tready), .S_AXIS_TDATA(s00_axis_tdata),
        .S_AXIS_TSTRB(4'hF), .S_AXIS_TLAST(s00_axis_tlast), .S_AXIS_TVALID(s00_axis_tvalid),
        .o_stream_data(stream_data_sig), .o_stream_valid(stream_valid_sig),
        .o_stream_last(stream_last_sig), .i_stream_ready(stream_ready_sig)
    );

    // the stream and the registers share one clock (s00_axi_aclk)
    NLO_CSM_Core_Pipeline u_core_engine (
        .clk(s00_axi_aclk), .rst_n(s00_axi_aresetn),
        .start(start_sig),
        .init_pose_x(pose_x_sig), .init_pose_y(pose_y_sig), .init_pose_theta(pose_theta_sig),
        .tau_threshold(tau_threshold_sig),
        .i_map_base_addr(map_base_addr_sig), .i_map_width(map_width_sig), .i_map_height(map_height_sig),
        .s_axis_tdata(stream_data_sig), .s_axis_tvalid(stream_valid_sig),
        .s_axis_tlast(stream_last_sig), .s_axis_tready(stream_ready_sig),
        .bram_addr(bram_addr), .bram_en(bram_en), .bram_we(bram_we), .bram_rdata(bram_rdata),
        .done_irq(irq),
        .H11(H11_sig), .H12(H12_sig), .H13(H13_sig),
        .H22(H22_sig), .H23(H23_sig), .H33(H33_sig),
        .K1(K1_sig), .K2(K2_sig), .K3(K3_sig),
        .o_point_cnt(point_cnt_sig), .o_current_state(state_monitor),
        .o_st1_ready(st1_ready_sig), .o_st2_ready(st2_ready_sig)
    );
endmodule
