/* ARM side of the scan-matching accelerator: register map, data formats and the Gauss-Newton passes. */
#ifndef NLO_CSM_ACCEL_H
#define NLO_CSM_ACCEL_H

#include <stdint.h>
#include "xaxidma.h"

/* AXI4-Lite registers (rtl/nlo_csm_accel_v1_0_S00_AXI.v) */
#define REG_CONTROL     0x00    /* write: bit 0 start; read: [31:16] points taken, [3:2] state */
#define REG_POSE_X      0x04    /* Q14 map cells */
#define REG_POSE_Y      0x08    /* Q14 map cells */
#define REG_POSE_THETA  0x0C    /* 65536 steps per turn */
#define REG_TAU         0x10    /* a point is used when its map value is at least tau, Q16 */
#define REG_MAP_BASE    0x14    /* byte address of the map on the block RAM port of the logic */
#define REG_MAP_WIDTH   0x18
#define REG_MAP_HEIGHT  0x1C

/* 64-bit sums, low word first: H11 H12 H13 H22 H23 H33 K1 K2 K3 */
#define RES_H11         0x20
#define RES_H12         0x28
#define RES_H13         0x30
#define RES_H22         0x38
#define RES_H23         0x40
#define RES_H33         0x48
#define RES_K1          0x50
#define RES_K2          0x58
#define RES_K3          0x60

#define STATE_DONE      3
#define CSM_MAX_BEAMS   360     /* the logic ends a pass after the 360th point or an earlier TLAST */
#define CSM_TAU_Q16     10      /* reset value of REG_TAU */

typedef struct {
    double x, y;                /* map cells */
    double theta;               /* radians */
} csm_pose;

typedef struct {
    UINTPTR accel;              /* register base of the accelerator */
    UINTPTR map_bram;           /* address of the map block RAM seen by the ARM cores */
    XAxiDma *dma;               /* MM2S channel feeds the scan stream */
    uint32_t width, height;     /* map size in cells */
} csm_dev;

typedef struct {
    double h11, h12, h13, h22, h23, h33;
    double k1, k2, k3;
    uint32_t points;
} csm_sums;

/* One map cell per 32-bit word, value in bits 7:0 (255 = occupied), in row-major order. */
void csm_load_map(csm_dev *dev, const uint8_t *cells, uint32_t width, uint32_t height);

/* Stream word of one beam: angle in degrees in bits 31:16, range in half cells in bits 15:0. */
uint32_t csm_pack_beam(int angle_deg, double range_cells);

/* One pass in the logic: sums of J^T J and J^T f over the scan at the given pose. */
int csm_pass(const csm_dev *dev, const csm_pose *pose, const uint32_t *beams, int n, csm_sums *sums);

/* Gauss-Newton passes until the step is small or max_passes is reached; returns the passes run, or -1 when
 * a pass fails or H is singular (the pose keeps its last update). */
int csm_match(const csm_dev *dev, csm_pose *pose, const uint32_t *beams, int n, int max_passes);

#endif
