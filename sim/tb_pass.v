`timescale 1ns / 1ps

// One pass from files, for the Python scripts. Run in the build directory.
//   cfg.txt     N  POSE_X  POSE_Y  HEADING  TAU  MAP_W  MAP_H   (decimal; pose in 14 fraction bits,
//               heading in CORDIC phase units, 8192 = pi)
//   points.hex  N words {angle[15:0], distance[15:0]}
//   map.hex     MAP_W * MAP_H words, cell value in bits [7:0]
// The stream holds TVALID high, as an AXI DMA does.
// Writes pass.txt: the sine and cosine of every point (P), clocks (C), the nine sums (S).
// C: start to first handshake, then handshake of the first point to its output from stage 1,
//    the FIFO, stage 2, 3, 4 and the accumulators, then start to interrupt.
module tb_pass;
    reg aclk = 0; always #5 aclk = ~aclk;
    reg aresetn = 0;

    reg  [6:0]  awaddr = 0; reg awvalid = 0; wire awready;
    reg  [31:0] wdata = 0;  reg wvalid = 0;  wire wready;
    wire [1:0]  bresp; wire bvalid; reg bready = 0;
    reg  [6:0]  araddr = 0; reg arvalid = 0; wire arready;
    wire [31:0] rdata; wire [1:0] rresp; wire rvalid; reg rready = 0;
    reg  [31:0] tdata = 0; reg tvalid = 0; reg tlast = 0; wire tready;
    wire [31:0] bram_addr; wire bram_en; wire [3:0] bram_we; reg [31:0] bram_rdata = 0;
    wire irq;

    nlo_csm_accel_v1_0 dut (
        .irq(irq), .bram_addr(bram_addr), .bram_en(bram_en), .bram_we(bram_we), .bram_rdata(bram_rdata),
        .bram_clk(), .bram_rst(),
        .s00_axi_aclk(aclk), .s00_axi_aresetn(aresetn),
        .s00_axi_awaddr(awaddr), .s00_axi_awprot(3'b0), .s00_axi_awvalid(awvalid), .s00_axi_awready(awready),
        .s00_axi_wdata(wdata), .s00_axi_wstrb(4'hF), .s00_axi_wvalid(wvalid), .s00_axi_wready(wready),
        .s00_axi_bresp(bresp), .s00_axi_bvalid(bvalid), .s00_axi_bready(bready),
        .s00_axi_araddr(araddr), .s00_axi_arprot(3'b0), .s00_axi_arvalid(arvalid), .s00_axi_arready(arready),
        .s00_axi_rdata(rdata), .s00_axi_rresp(rresp), .s00_axi_rvalid(rvalid), .s00_axi_rready(rready),
        .s00_axis_aclk(aclk), .s00_axis_aresetn(aresetn),
        .s00_axis_tready(tready), .s00_axis_tdata(tdata), .s00_axis_tlast(tlast), .s00_axis_tvalid(tvalid)
    );

    reg [31:0] mem [0:65535];
    always @(posedge aclk) if (bram_en) bram_rdata <= mem[bram_addr[17:2]];

    reg [31:0] pts [0:359];
    integer N, PX, PY, PT, TAU, MW, MH, fd, r, i, k, fo;
    integer cyc = 0; always @(posedge aclk) cyc <= cyc + 1;
    integer c_start, c_first = -1, c_s1 = -1, c_ff = -1, c_s2 = -1, c_s3 = -1, c_s4 = -1, c_s5 = -1, c_irq;

    always @(posedge aclk) begin
        if (tvalid && tready && c_first < 0) c_first = cyc;
        if (dut.u_core_engine.u_st1.cordic_valid) begin
            $fdisplay(fo, "P %0d %0d", $signed(dut.u_core_engine.u_st1.cordic_dout[31:16]),
                                       $signed(dut.u_core_engine.u_st1.cordic_dout[15:0]));
        end
        if (dut.u_core_engine.st1_v && c_s1 < 0) c_s1 = cyc;
        if (dut.u_core_engine.u_st2.st1_valid && c_ff < 0) c_ff = cyc;
        if (dut.u_core_engine.u_st2.st2_valid && c_s2 < 0) c_s2 = cyc;
        if (dut.u_core_engine.u_st3.st3_valid && c_s3 < 0) c_s3 = cyc;
        if (dut.u_core_engine.u_st4.st4_valid && c_s4 < 0) c_s4 = cyc;
        if (dut.u_core_engine.u_st5.v_d1 && c_s5 < 0) c_s5 = cyc + 1;
    end

    task axi_write(input [6:0] adr, input [31:0] dat);
    begin
        @(posedge aclk); awaddr <= adr; awvalid <= 1; wdata <= dat; wvalid <= 1; bready <= 1;
        @(negedge aclk); while (!(awready && wready)) @(negedge aclk);
        @(posedge aclk); awvalid <= 0; wvalid <= 0;
        @(negedge aclk); while (!bvalid) @(negedge aclk);
        @(posedge aclk); bready <= 0;
    end
    endtask

    task axi_read(input [6:0] adr, output [31:0] dat);
    begin
        @(posedge aclk); araddr <= adr; arvalid <= 1; rready <= 1;
        @(negedge aclk); while (!arready) @(negedge aclk);
        @(posedge aclk); arvalid <= 0;
        @(negedge aclk); while (!rvalid) @(negedge aclk);
        dat = rdata;
        @(posedge aclk); rready <= 0;
    end
    endtask

    reg [31:0] lo, hi;
    initial begin
        fo = $fopen("pass.txt", "w");
        fd = $fopen("cfg.txt", "r");
        r = $fscanf(fd, "%d %d %d %d %d %d %d", N, PX, PY, PT, TAU, MW, MH);
        $fclose(fd);
        for (k = 0; k < 65536; k = k + 1) mem[k] = 0;
        $readmemh("map.hex", mem, 0, MW*MH - 1);
        $readmemh("points.hex", pts, 0, N - 1);

        #100 aresetn = 1; #50;
        axi_write(7'h18, MW); axi_write(7'h1C, MH);
        axi_write(7'h04, PX); axi_write(7'h08, PY); axi_write(7'h0C, PT);
        axi_write(7'h10, TAU); axi_write(7'h14, 0);
        axi_write(7'h00, 1);
        c_start = cyc;
        for (i = 0; i < N; i = i + 1) begin
            tdata <= pts[i]; tvalid <= 1; tlast <= (i == N - 1);
            @(negedge aclk); while (!tready) @(negedge aclk);
            @(posedge aclk);
        end
        tvalid <= 0; tlast <= 0;
        @(negedge aclk); while (!irq) @(negedge aclk);
        c_irq = cyc;
        $fdisplay(fo, "C %0d %0d %0d %0d %0d %0d %0d %0d", c_first - c_start, c_s1 - c_first, c_ff - c_first,
                  c_s2 - c_first, c_s3 - c_first, c_s4 - c_first, c_s5 - c_first, c_irq - c_start);
        $fwrite(fo, "S");
        for (k = 0; k < 9; k = k + 1) begin
            axi_read(7'h20 + 8*k, lo);
            axi_read(7'h24 + 8*k, hi);
            $fwrite(fo, " %0d", $signed({hi, lo}));
        end
        $fwrite(fo, "\n");
        $fclose(fo);
        $finish;
    end

    initial begin #20000000; $display("TIMEOUT"); $finish; end
endmodule
