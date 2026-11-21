/* Matches every scan in lidar_dataset.h to the map using the accelerator and prints the pose.
 *
 * lidar_dataset.h is made from the recorded data and is not part of the repository. It defines
 *   MAP_WIDTH, MAP_HEIGHT        map size in cells
 *   map_cells[]                  uint8_t, 255 = occupied, in row-major order
 *   N_FRAMES, frame_range[][360] float, beam range in map cells at each degree, 0 = no return
 *   INIT_X, INIT_Y, INIT_THETA   first pose guess in cells, cells and radians
 */
#include <stdio.h>
#include <stdint.h>
#include "xil_io.h"
#include "xparameters.h"
#include "xaxidma.h"
#include "xtime_l.h"
#include "nlo_csm_accel.h"
#include "lidar_dataset.h"

#define NLO_BASE            XPAR_NLO_CSM_ACCEL_V1_0_0_BASEADDR
#define MAP_PS_BRAM_ADDR    0xA0040000      /* map block RAM through the AXI BRAM controller */
#ifndef SDT
#define DMA_ID              XPAR_AXIDMA_0_DEVICE_ID
#else
#define DMA_ID              XPAR_XAXIDMA_0_BASEADDR
#endif
#ifndef MAX_PASSES
#define MAX_PASSES          6
#endif

static uint32_t beams[CSM_MAX_BEAMS] __attribute__((aligned(64)));

int main(void)
{
    XAxiDma dma;
    XAxiDma_Config *cfg = XAxiDma_LookupConfig(DMA_ID);

    if (cfg == NULL || XAxiDma_CfgInitialize(&dma, cfg) != XST_SUCCESS) {
        printf("DMA init failed\n");
        return 1;
    }
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

    csm_dev dev = { NLO_BASE, MAP_PS_BRAM_ADDR, &dma, 0, 0 };
    csm_load_map(&dev, map_cells, MAP_WIDTH, MAP_HEIGHT);
    csm_pose pose = { INIT_X, INIT_Y, INIT_THETA };

    for (int f = 0; f < N_FRAMES; f++) {
        int n = 0;
        for (int deg = 0; deg < 360; deg++)
            if (frame_range[f][deg] > 0.0f)
                beams[n++] = csm_pack_beam(deg, frame_range[f][deg]);

        XTime t0, t1;
        XTime_GetTime(&t0);
        int passes = csm_match(&dev, &pose, beams, n, MAX_PASSES);
        XTime_GetTime(&t1);

        if (passes < 0) {
            printf("Frame %d | beams %d | match failed, pose kept\n", f, n);
            continue;
        }
        printf("Frame %d | beams %d | passes %d | X %.4f Y %.4f cells | theta %.4f deg | %.3f ms\n",
               f, n, passes, pose.x, pose.y, pose.theta * 57.29577951308232,
               1e3 * (double)(t1 - t0) / COUNTS_PER_SECOND);
    }
    return 0;
}
