`timescale 1ns / 1ps

// Self-checking bench: coordinates in all four quadrants, three stream patterns, abort, TLAST,
// points outside the map, and a threshold above every map value.
module tb_checks;
    reg aclk = 0; always #5 aclk = ~aclk;    // 100 MHz
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

    // block RAM, one read port, one clock read latency
    localparam integer MW = 64, MH = 64;
    reg [31:0] mem [0:MW*MH-1];
    always @(posedge aclk) if (bram_en) bram_rdata <= mem[bram_addr[13:2]];

    localparam integer TH0 = -7400;
    localparam real    PI  = 3.14159265358979;
    localparam real    TOL = 0.01;              // cells

    reg  [31:0] pts [0:359];
    integer cyc = 0; always @(posedge aclk) cyc <= cyc + 1;

    // ---------------- monitors ----------------
    integer n_acc = 0, n_st1 = 0, n_st2 = 0, n_ren = 0;
    always @(posedge aclk) if (bram_en) n_ren = n_ren + 1;
    integer q_bad = 0; real q_err_max = 0.0;
    reg     check_st1 = 0;
    real    ex, ey, gx, gy, phi, e;
    integer a, d;
    always @(posedge aclk) begin
        if (tvalid && tready) n_acc = n_acc + 1;
        if (dut.u_core_engine.u_st2.st2_valid) n_st2 = n_st2 + 1;
        if (dut.u_core_engine.u_st1.valid_out) begin
            if (check_st1) begin
                a   = pts[n_st1][31:16];
                d   = pts[n_st1][15:0];
                phi = TH0 * PI / 8192.0 + a * PI / 180.0;
                ex  = 32.0 + d / 2.0 * $cos(phi);
                ey  = 32.0 + d / 2.0 * $sin(phi);
                gx  = $signed(dut.u_core_engine.u_st1.global_x) / 16384.0;
                gy  = $signed(dut.u_core_engine.u_st1.global_y) / 16384.0;
                e   = (gx > ex ? gx - ex : ex - gx);
                if ((gy > ey ? gy - ey : ey - gy) > e) e = (gy > ey ? gy - ey : ey - gy);
                if (e > q_err_max) q_err_max = e;
                if (e > TOL) begin
                    q_bad = q_bad + 1;
                    if (q_bad <= 4)
                        $display("  point %0d, beam at %0.1f deg: stage 1 (%0.3f, %0.3f), expected (%0.3f, %0.3f)",
                                 n_st1, phi * 180.0 / PI, gx, gy, ex, ey);
                end
            end
            n_st1 = n_st1 + 1;
        end
    end

    // phase handed to the CORDIC core against heading + angle x 8192/180, modulo one turn
    integer n_ph = 0; real ph_err, ph_err_max = 0.0;
    always @(posedge aclk) begin
        if (check_st1 && dut.u_core_engine.u_st1.valid_in && dut.u_core_engine.u_st1.cordic_ready) begin
            ph_err = $signed(dut.u_core_engine.u_st1.total_angle)
                     - (TH0 + dut.u_core_engine.u_st1.lidar_angle * 8192.0 / 180.0);
            while (ph_err >= 8192.0) ph_err = ph_err - 16384.0;
            while (ph_err < -8192.0) ph_err = ph_err + 16384.0;
            if (ph_err < 0) ph_err = -ph_err;
            if (ph_err > ph_err_max) ph_err_max = ph_err;
            n_ph = n_ph + 1;
        end
    end

    // ---------------- bus tasks ----------------
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

    reg [63:0] sums [0:8];
    task read_sums;
        integer k; reg [31:0] lo, hi;
    begin
        for (k = 0; k < 9; k = k + 1) begin
            axi_read(7'h20 + 8*k, lo);
            axi_read(7'h24 + 8*k, hi);
            sums[k] = {hi, lo};
        end
    end
    endtask

    // mode 0: one point, then 60 idle clocks. 1: TVALID held high. 2: random gaps.
    reg [15:0] lfsr = 16'hACE1;
    integer c_start, c_irq;
    task run_pass(input integer n, input integer mode);
        integer i, g;
    begin
        n_acc = 0; n_st1 = 0; n_st2 = 0;
        axi_write(7'h00, 0);
        axi_write(7'h00, 1);
        c_start = cyc;
        for (i = 0; i < n; i = i + 1) begin
            if (mode == 2) begin
                lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
                if (lfsr[1:0] != 0) begin
                    tvalid <= 0; tlast <= 0;
                    for (g = 0; g < lfsr[1:0]; g = g + 1) @(posedge aclk);
                end
            end
            tdata <= pts[i]; tvalid <= 1; tlast <= (i == n - 1);
            @(negedge aclk); while (!tready) @(negedge aclk);
            @(posedge aclk);
            if (mode == 0) begin
                tvalid <= 0; tlast <= 0;
                for (g = 0; g < 60; g = g + 1) @(posedge aclk);
            end
        end
        tvalid <= 0; tlast <= 0;
        @(negedge aclk); while (!irq) @(negedge aclk);
        c_irq = cyc;
        repeat (20) @(posedge aclk);
        read_sums;
    end
    endtask

    // ---------------- tests ----------------
    integer i, x, y, k, fail_q, fail_s;
    real r, v;
    reg [63:0] ref_sums [0:8];
    integer acc_p [0:2], st2_p [0:2], cyc_p [0:2];
    reg same [0:2];
    reg [31:0] status;
    reg zero, same_a, same_o;
    integer i2, n_out, r_out;
    initial begin
        for (y = 0; y < MH; y = y + 1)
            for (x = 0; x < MW; x = x + 1) begin
                r = $sqrt((x - 32.0) * (x - 32.0) + (y - 32.0) * (y - 32.0));
                v = 255.0 * $exp(-(r - 20.0) * (r - 20.0) / 8.0);
                mem[y*MW + x] = $rtoi(v + 0.5);
            end

        #100 aresetn = 1; #50;
        axi_write(7'h18, MW); axi_write(7'h1C, MH);
        axi_write(7'h04, 32 << 14); axi_write(7'h08, 32 << 14);
        axi_write(7'h0C, TH0);
        axi_write(7'h10, 0);
        axi_write(7'h14, 0);

        // quadrant test: 360 beams, distance 40 (20 cells, on the wall)
        for (i = 0; i < 360; i = i + 1) pts[i] = {i[15:0], 16'd40};
        check_st1 = 1;
        run_pass(360, 0);
        check_st1 = 0;
        fail_q = (q_bad != 0) || (n_st1 != 360) || (n_ph != 360) || (ph_err_max > 0.5);
        $display("quadrant: %0d of %0d stage-1 coordinates off by more than %0.2f cells, largest difference %0.4f cells",
                 q_bad, n_st1, TOL, q_err_max);
        $display("phase: %0d beams, largest difference from heading + angle x 8192/180 %0.4f phase units (%0.5f deg)",
                 n_ph, ph_err_max, ph_err_max * 180.0 / 8192.0);
        if (fail_q) $display("TEST quadrant FAIL"); else $display("TEST quadrant PASS");

        // stream test: 360 points, distance 30 to 50 half cells
        for (i = 0; i < 360; i = i + 1) pts[i] = (i << 16) | (30 + (i * 7) % 21);
        for (k = 0; k < 3; k = k + 1) begin
            run_pass(360, k);
            acc_p[k] = n_acc; st2_p[k] = n_st2; cyc_p[k] = c_irq - c_start;
            if (k == 0) for (i = 0; i < 9; i = i + 1) ref_sums[i] = sums[i];
            same[k] = 1;
            for (i = 0; i < 9; i = i + 1) if (sums[i] !== ref_sums[i]) same[k] = 0;
            case (k)
                0: $display("stream pass 0, one point at a time:");
                1: $display("stream pass 1, TVALID held high:");
                2: $display("stream pass 2, random gaps:");
            endcase
            $display("  accepted %0d, looked up %0d, start to interrupt %0d clocks", acc_p[k], st2_p[k], cyc_p[k]);
            if (same[k]) $display("  sums equal to pass 0");
            else         $display("  sums DIFFERENT from pass 0");
            $display("  H11 %0d H22 %0d H33 %0d K1 %0d K2 %0d K3 %0d",
                     $signed(sums[0]), $signed(sums[3]), $signed(sums[5]),
                     $signed(sums[6]), $signed(sums[7]), $signed(sums[8]));
        end
        fail_s = 0;
        for (k = 0; k < 3; k = k + 1)
            if (acc_p[k] != 360 || st2_p[k] != 360 || !same[k]) fail_s = 1;
        if (fail_s) $display("TEST stream FAIL"); else $display("TEST stream PASS");

        // abort: start cleared after 100 points, then the full scan again
        axi_write(7'h00, 0);
        axi_write(7'h00, 1);
        n_acc = 0;
        for (i = 0; i < 100; i = i + 1) begin
            tdata <= pts[i]; tvalid <= 1; tlast <= 0;
            @(negedge aclk); while (!tready) @(negedge aclk);
            @(posedge aclk);
        end
        tvalid <= 0;
        axi_write(7'h00, 0);
        repeat (200) @(posedge aclk);
        axi_read(7'h00, status);
        read_sums;
        zero = 1;
        for (i = 0; i < 9; i = i + 1) if (sums[i] != 0) zero = 0;
        $display("abort: start cleared after %0d points; pass state %0d, sums %0s",
                 n_acc, status[3:2], zero ? "all zero" : "NOT zero");
        run_pass(360, 1);
        same_a = 1;
        for (i = 0; i < 9; i = i + 1) if (sums[i] !== ref_sums[i]) same_a = 0;
        $display("  next pass: accepted %0d, looked up %0d, start to interrupt %0d clocks, sums %0s",
                 n_acc, n_st2, c_irq - c_start, same_a ? "equal to stream pass 0" : "DIFFERENT from stream pass 0");
        if (zero && status[3:2] == 0 && same_a && n_acc == 360) $display("TEST abort PASS");
        else $display("TEST abort FAIL");

        // short scan: TLAST on the 100th point
        run_pass(100, 1);
        axi_read(7'h00, status);
        $display("tlast: accepted %0d, looked up %0d, point count register %0d, start to interrupt %0d clocks",
                 n_acc, n_st2, status[31:16], c_irq - c_start);
        if (n_acc == 100 && n_st2 == 100 && status[31:16] == 100)
            $display("TEST tlast PASS");
        else $display("TEST tlast FAIL");

        // outside the map: every second point lands 50 cells from the centre, outside the 64 x 64 map.
        // Sums must equal those of the inside points alone, and the outside points must not read the map.
        for (i = 0; i < 360; i = i + 1) pts[i] = {i[15:0], (i % 2 == 0) ? 16'd40 : 16'd100};
        n_ren = 0;
        run_pass(360, 1);
        for (i = 0; i < 9; i = i + 1) ref_sums[i] = sums[i];
        n_out = n_st2; r_out = n_ren;
        for (i = 0; i < 180; i = i + 1) begin i2 = 2 * i; pts[i] = {i2[15:0], 16'd40}; end
        n_ren = 0;
        run_pass(180, 1);
        same_o = 1;
        for (i = 0; i < 9; i = i + 1) if (sums[i] !== ref_sums[i]) same_o = 0;
        $display("outside: 360 points, 180 outside the map: looked up %0d, map reads %0d; the 180 inside points alone: looked up %0d, map reads %0d; sums %0s",
                 n_out, r_out, n_st2, n_ren, same_o ? "equal" : "DIFFERENT");
        if (same_o && n_out == 360 && r_out == 720 && n_ren == 720) $display("TEST outside PASS");
        else $display("TEST outside FAIL");

        // threshold above every map value: no point may count
        for (i = 0; i < 360; i = i + 1) pts[i] = (i << 16) | (30 + (i * 7) % 21);
        axi_write(7'h10, 256 << 16);
        run_pass(360, 1);
        zero = 1;
        for (i = 0; i < 9; i = i + 1) if (sums[i] != 0) zero = 0;
        $display("threshold 256: looked up %0d, sums %0s", n_st2, zero ? "all zero" : "NOT zero");
        if (zero && n_st2 == 360) $display("TEST threshold PASS"); else $display("TEST threshold FAIL");
        axi_write(7'h10, 0);
        $finish;
    end

    initial begin #20000000; $display("TIMEOUT"); $finish; end
endmodule
