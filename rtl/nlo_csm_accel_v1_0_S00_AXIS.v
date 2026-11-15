`timescale 1 ns / 1 ps

// AXI4-Stream slave: passes the scan points straight to the pipeline.
module nlo_csm_accel_v1_0_S00_AXIS #(
    parameter integer C_S_AXIS_TDATA_WIDTH = 32
)(
    input  wire S_AXIS_ACLK,
    input  wire S_AXIS_ARESETN,
    output wire S_AXIS_TREADY,
    input  wire [C_S_AXIS_TDATA_WIDTH-1 : 0] S_AXIS_TDATA,
    input  wire [(C_S_AXIS_TDATA_WIDTH/8)-1 : 0] S_AXIS_TSTRB,
    input  wire S_AXIS_TLAST,
    input  wire S_AXIS_TVALID,

    output wire [C_S_AXIS_TDATA_WIDTH-1 : 0] o_stream_data,
    output wire o_stream_valid,
    output wire o_stream_last,
    input  wire i_stream_ready
);
    assign o_stream_data  = S_AXIS_TDATA;
    assign o_stream_valid = S_AXIS_TVALID;
    assign o_stream_last  = S_AXIS_TLAST;
    assign S_AXIS_TREADY  = i_stream_ready;
endmodule
