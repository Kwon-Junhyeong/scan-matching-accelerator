#include <math.h>
#include "xil_io.h"
#include "xil_cache.h"
#include "nlo_csm_accel.h"

#define Q14             16384.0
#define H_SCALE         268435456.0     /* J is Q14, so J^T J is Q28 */
#define K_SCALE         1048576.0       /* Q14 times the Q16 score, shifted right by 10 */
#define TURN_PER_RAD    (65536.0 / (2.0 * M_PI))
#define DONE_POLLS      1000000
#define STEP_CELLS      0.01
#define STEP_RAD        (0.01 * M_PI / 180.0)

void csm_load_map(csm_dev *dev, const uint8_t *cells, uint32_t width, uint32_t height)
{
    for (uint32_t i = 0; i < width * height; i++)
        Xil_Out32(dev->map_bram + 4u * i, cells[i]);
    dev->width = width;
    dev->height = height;
    Xil_Out32(dev->accel + REG_MAP_BASE, 0);
    Xil_Out32(dev->accel + REG_MAP_WIDTH, width);
    Xil_Out32(dev->accel + REG_MAP_HEIGHT, height);
}

uint32_t csm_pack_beam(int angle_deg, double range_cells)
{
    long half = lround(2.0 * range_cells);
    if (half < 0) half = 0;
    if (half > 0xFFFF) half = 0xFFFF;
    return ((uint32_t)angle_deg & 0xFFFFu) << 16 | (uint32_t)half;
}

static double read_sum(const csm_dev *dev, uint32_t offset)
{
    uint64_t lo = Xil_In32(dev->accel + offset);
    uint64_t hi = Xil_In32(dev->accel + offset + 4);
    return (double)(int64_t)(hi << 32 | lo);
}

int csm_pass(const csm_dev *dev, const csm_pose *pose, const uint32_t *beams, int n, csm_sums *sums)
{
    if (n <= 0 || n > CSM_MAX_BEAMS)
        return -1;

    Xil_Out32(dev->accel + REG_POSE_X, (uint32_t)(int32_t)lround(pose->x * Q14));
    Xil_Out32(dev->accel + REG_POSE_Y, (uint32_t)(int32_t)lround(pose->y * Q14));
    Xil_Out32(dev->accel + REG_POSE_THETA, (uint32_t)(int32_t)lround(pose->theta * TURN_PER_RAD) & 0xFFFFu);
    Xil_Out32(dev->accel + REG_TAU, CSM_TAU_Q16);

    /* start stays high for the whole pass; dropping it returns the logic to idle and clears the sums */
    Xil_DCacheFlushRange((UINTPTR)beams, (u32)n * sizeof(uint32_t));
    Xil_Out32(dev->accel + REG_CONTROL, 1);
    if (XAxiDma_SimpleTransfer(dev->dma, (UINTPTR)beams, (u32)n * sizeof(uint32_t),
                               XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS) {
        Xil_Out32(dev->accel + REG_CONTROL, 0);
        return -1;
    }
    int polls = 0;
    while (XAxiDma_Busy(dev->dma, XAXIDMA_DMA_TO_DEVICE)) {
        if (++polls > DONE_POLLS) {
            Xil_Out32(dev->accel + REG_CONTROL, 0);
            return -1;
        }
    }

    uint32_t status;
    polls = 0;
    while (((status = Xil_In32(dev->accel + REG_CONTROL)) >> 2 & 3u) != STATE_DONE) {
        if (++polls > DONE_POLLS) {
            Xil_Out32(dev->accel + REG_CONTROL, 0);
            return -1;
        }
    }

    sums->points = status >> 16;
    sums->h11 = read_sum(dev, RES_H11) / H_SCALE;
    sums->h12 = read_sum(dev, RES_H12) / H_SCALE;
    sums->h13 = read_sum(dev, RES_H13) / H_SCALE;
    sums->h22 = read_sum(dev, RES_H22) / H_SCALE;
    sums->h23 = read_sum(dev, RES_H23) / H_SCALE;
    sums->h33 = read_sum(dev, RES_H33) / H_SCALE;
    sums->k1 = read_sum(dev, RES_K1) / K_SCALE;
    sums->k2 = read_sum(dev, RES_K2) / K_SCALE;
    sums->k3 = read_sum(dev, RES_K3) / K_SCALE;

    Xil_Out32(dev->accel + REG_CONTROL, 0);
    return 0;
}

/* solve H d = k for the symmetric 3 x 3 H using its adjugate */
static int solve3(const csm_sums *s, double d[3])
{
    double a = s->h11, b = s->h12, c = s->h13, e = s->h22, f = s->h23, i = s->h33;
    double A = e * i - f * f, B = c * f - b * i, C = b * f - c * e;
    double E = a * i - c * c, F = b * c - a * f, I = a * e - b * b;
    double det = a * A + b * B + c * C;

    if (!(det > 1e-9 * a * e * i))
        return -1;
    d[0] = (A * s->k1 + B * s->k2 + C * s->k3) / det;
    d[1] = (B * s->k1 + E * s->k2 + F * s->k3) / det;
    d[2] = (C * s->k1 + F * s->k2 + I * s->k3) / det;
    return 0;
}

int csm_match(const csm_dev *dev, csm_pose *pose, const uint32_t *beams, int n, int max_passes)
{
    for (int pass = 1; pass <= max_passes; pass++) {
        csm_sums s;
        double d[3];

        if (csm_pass(dev, pose, beams, n, &s) != 0 || solve3(&s, d) != 0)
            return -1;
        pose->x += d[0];
        pose->y += d[1];
        pose->theta += d[2];
        if (fabs(d[0]) < STEP_CELLS && fabs(d[1]) < STEP_CELLS && fabs(d[2]) < STEP_RAD)
            return pass;
    }
    return max_passes;
}
