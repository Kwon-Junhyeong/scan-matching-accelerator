/* One Gauss-Newton pass in software, timed against the number of points.
 *
 * double_pass(): the per-point work of a pass in double precision, as sim/model.py double_pass().
 * fixed_pass():  the same work in the integer formats of rtl/ (sine and cosine from the C library,
 *                rounded to 14 fraction bits, in place of the CORDIC core).
 * The map is sim/data/map.pgm. The scan is 360 beams from a free cell, distance to the first cell
 * of value 200 or more. Each pass is repeated until at least 0.2 s have passed; the time per pass
 * is the median of 15 such runs.
 *
 * Build and run (MSVC): cl /O2 /nologo experiments\pass_sw.c && pass_sw sim\data\map.pgm
 *            (gcc):     gcc -O2 -o pass_sw experiments/pass_sw.c -lm && ./pass_sw sim/data/map.pgm
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
static double now(void) {
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
}
#else
static double now(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + 1e-9 * t.tv_nsec;
}
#endif

#define PI 3.14159265358979323846
static int W, H;
static uint8_t *map;

static int load_pgm(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    char magic[3] = {0};
    int maxv;
    if (fscanf(f, "%2s %d %d %d", magic, &W, &H, &maxv) != 4) return 0;
    fgetc(f);
    map = malloc((size_t)W * H);
    size_t n = fread(map, 1, (size_t)W * H, f);
    fclose(f);
    return n == (size_t)W * H;
}

typedef struct { int angle, dist; } point_t;   /* degrees, half cells */

/* ---------- double precision ---------- */
static void double_pass(const point_t *p, int n, double x, double y, double th, double Hm[6], double g[3]) {
    memset(Hm, 0, 6 * sizeof(double));
    memset(g, 0, 3 * sizeof(double));
    for (int i = 0; i < n; i++) {
        double phi = th + p[i].angle * (PI / 180.0);
        double c = cos(phi), s = sin(phi), d = 0.5 * p[i].dist;
        double px = x + d * c, py = y + d * s;
        int ix = (int)floor(px), iy = (int)floor(py);
        if (ix < 0 || iy < 0 || ix >= W - 1 || iy >= H - 1) continue;
        double fx = px - ix, fy = py - iy;
        double p00 = map[iy * W + ix], p10 = map[iy * W + ix + 1];
        double p01 = map[(iy + 1) * W + ix], p11 = map[(iy + 1) * W + ix + 1];
        double M = p00 * (1 - fx) * (1 - fy) + p10 * fx * (1 - fy) + p01 * (1 - fx) * fy + p11 * fx * fy;
        double gx = (p10 - p00) * (1 - fy) + (p11 - p01) * fy;
        double gy = (p01 - p00) * (1 - fx) + (p11 - p10) * fx;
        double j3 = gx * (-d * s) + gy * (d * c), f = 255.0 - M;
        Hm[0] += gx * gx; Hm[1] += gx * gy; Hm[2] += gx * j3;
        Hm[3] += gy * gy; Hm[4] += gy * j3; Hm[5] += j3 * j3;
        g[0] += gx * f; g[1] += gy * f; g[2] += j3 * f;
    }
}

/* ---------- the integer formats of rtl/ ---------- */
static void fixed_pass(const point_t *p, int n, int32_t x, int32_t y, int32_t th, int64_t Hm[6], int64_t g[3]) {
    memset(Hm, 0, 6 * sizeof(int64_t));
    memset(g, 0, 3 * sizeof(int64_t));
    for (int i = 0; i < n; i++) {
        int32_t beam = (int32_t)(((int64_t)p[i].angle * 2982616 + 32768) >> 16) & 0x3FFF;
        int32_t ph = ((th & 0x3FFF) + beam) & 0x3FFF;
        if (ph & 0x2000) ph -= 0x4000;
        double phi = ph * (PI / 8192.0);
        int32_t c = (int32_t)lround(cos(phi) * 16384), s = (int32_t)lround(sin(phi) * 16384);
        int32_t dcos = p[i].dist * c, dsin = p[i].dist * s;
        int32_t gxq = (dcos >> 1) + x, gyq = (dsin >> 1) + y, dtx = -(dsin >> 1), dty = dcos >> 1;
        int32_t ix = gxq >> 14, iy = gyq >> 14, fx = gxq & 0x3FFF, fy = gyq & 0x3FFF;
        int32_t p00 = 0, p10 = 0, p01 = 0, p11 = 0;
        if (ix >= 0 && iy >= 0 && ix < W - 1 && iy < H - 1) {
            p00 = map[iy * W + ix]; p10 = map[iy * W + ix + 1];
            p01 = map[(iy + 1) * W + ix]; p11 = map[(iy + 1) * W + ix + 1];
        }
        int64_t ifx = 16384 - fx, ify = 16384 - fy;
        int64_t m28 = p00 * ifx * ify + p10 * fx * ify + p01 * ifx * fy + p11 * fx * fy;
        int32_t m16 = (int32_t)(m28 >> 12);
        int32_t grx = (p10 - p00) * (int32_t)ify + (p11 - p01) * fy;
        int32_t gry = (p01 - p00) * (int32_t)ifx + (p11 - p10) * fx;
        int32_t f = (255 << 16) - m16;
        int32_t j3 = (int32_t)((((int64_t)grx * dtx) >> 14) + (((int64_t)gry * dty) >> 14));
        Hm[0] += (int64_t)grx * grx; Hm[1] += (int64_t)grx * gry; Hm[2] += (int64_t)grx * j3;
        Hm[3] += (int64_t)gry * gry; Hm[4] += (int64_t)gry * j3; Hm[5] += ((int64_t)j3 * j3) >> 8;
        g[0] += ((int64_t)grx * f) >> 10; g[1] += ((int64_t)gry * f) >> 10; g[2] += ((int64_t)j3 * f) >> 10;
    }
}

static int cmp(const void *a, const void *b) {
    double d = *(const double *)a - *(const double *)b;
    return (d > 0) - (d < 0);
}

volatile double sink;

int main(int argc, char **argv) {
    if (argc < 2 || !load_pgm(argv[1])) { fprintf(stderr, "usage: pass_sw map.pgm\n"); return 1; }
    /* sensor at a free cell near the centre, scan to the first cell of value 200 or more */
    double sx = W / 2 + 0.5, sy = H / 2 + 0.5, th = 0.3;
    for (int r = 0; r < W / 2 && map[(int)sy * W + (int)sx] != 0; r++) sx += 1;
    point_t scan[360];
    int n = 0;
    for (int a = 0; a < 360; a++) {
        double phi = th + a * PI / 180.0;
        for (double r = 0.5; r < 120; r += 0.05) {
            int ix = (int)floor(sx + r * cos(phi)), iy = (int)floor(sy + r * sin(phi));
            if (ix < 0 || iy < 0 || ix >= W || iy >= H) break;
            if (map[iy * W + ix] >= 200) { scan[n].angle = a; scan[n].dist = (int)lround(2 * r); n++; break; }
        }
    }
    printf("map %d x %d, sensor (%.1f, %.1f), %d of 360 beams return\n", W, H, sx, sy, n);
    printf("points  double us/pass  fixed us/pass\n");
    int sizes[] = {60, 120, 180, 240, 300, 360};
    for (int k = 0; k < 6; k++) {
        int m = sizes[k];
        point_t pts[360];
        for (int i = 0; i < m; i++) pts[i] = scan[i % n];
        double td[15], tf[15];
        for (int rep = 0; rep < 15; rep++) {
            double Hd[6], gd[3]; int64_t Hf[6], gf[3];
            long it = 0; double t0 = now(), t1;
            do { double_pass(pts, m, sx, sy, th, Hd, gd); sink = Hd[0] + gd[2]; it++; } while ((t1 = now()) - t0 < 0.2);
            td[rep] = (t1 - t0) / it;
            it = 0; t0 = now();
            do {
                fixed_pass(pts, m, (int32_t)(sx * 16384), (int32_t)(sy * 16384), (int32_t)(th / PI * 8192), Hf, gf);
                sink = (double)Hf[0] + (double)gf[2]; it++;
            } while ((t1 = now()) - t0 < 0.2);
            tf[rep] = (t1 - t0) / it;
        }
        qsort(td, 15, sizeof(double), cmp);
        qsort(tf, 15, sizeof(double), cmp);
        printf("%6d  %14.2f  %13.2f\n", m, td[7] * 1e6, tf[7] * 1e6);
    }
    return 0;
}
