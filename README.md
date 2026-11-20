# Scan-Matching Accelerator

FPGA logic for Gauss-Newton LiDAR scan matching on an occupancy grid map, controlled by the ARM cores of a ZCU104 board (Zynq UltraScale+ MPSoC).

## Division of labor between the ARM cores and the logic

A mobile robot matches each LiDAR scan to its map to know where it is, and every pass of the matcher repeats the same arithmetic for every scan point. That per-point arithmetic runs in logic. The ARM cores keep the work done once per pass: they write the pose guess, read nine sums back, solve a 3 x 3 system and update the pose.

![System](docs/system.png)

## Five pipeline stages

Each scan point goes through five stages, from the beam to the update sums:

1. Coordinates of the point, with sine and cosine from a CORDIC core
2. The four map cells around the point, read from block RAM one after another
3. Map value, slope and residual, with a threshold on the map value
4. Jacobian row
5. Sums in 64-bit accumulators

In RTL simulation a point takes 56 clocks from the stream to the sums (24, 26, 2, 2 and 2 clocks per stage), and a 360-point scan takes 0.18 ms at 100 MHz.

## Build

`ip/cordic_sin_cos.tcl` creates the CORDIC core of stage 1 (`vivado -mode batch -source ip/cordic_sin_cos.tcl -tclargs OUT_DIR`). The top module is `rtl/nlo_csm_accel_v1_0.v`. On the ZCU104 the design met timing at 100 MHz.
