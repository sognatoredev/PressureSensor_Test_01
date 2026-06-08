/* KissFFT - Mark Borgerding (BSD-3-Clause)
 * Complete C++ implementation for Arduino/ESP32
 * Uses inline functions instead of macros to avoid name conflicts */

#include "kiss_fft.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- complex arithmetic (inline functions, no macros) ---- */
static inline void cx_mul(kiss_fft_cpx &dst,
                           const kiss_fft_cpx &a, const kiss_fft_cpx &b) {
    dst.r = a.r*b.r - a.i*b.i;
    dst.i = a.r*b.i + a.i*b.r;
}
static inline void cx_add(kiss_fft_cpx &dst,
                           const kiss_fft_cpx &a, const kiss_fft_cpx &b) {
    dst.r = a.r + b.r;  dst.i = a.i + b.i;
}
static inline void cx_sub(kiss_fft_cpx &dst,
                           const kiss_fft_cpx &a, const kiss_fft_cpx &b) {
    dst.r = a.r - b.r;  dst.i = a.i - b.i;
}
static inline void cx_addto(kiss_fft_cpx &dst, const kiss_fft_cpx &a) {
    dst.r += a.r;  dst.i += a.i;
}
static inline void cx_scale(kiss_fft_cpx &dst, float s) {
    dst.r *= s;  dst.i *= s;
}

/* ---- butterfly radix-2 ---- */
static void kf_bfly2(kiss_fft_cpx *Fout, size_t fstride,
                     const kiss_fft_cfg st, int cnt)
{
    kiss_fft_cpx *Fout2 = Fout + cnt;
    kiss_fft_cpx *tw    = st->twiddles;
    kiss_fft_cpx  t;
    do {
        cx_mul(t, *Fout2, *tw);
        tw += fstride;
        cx_sub(*Fout2, *Fout, t);
        cx_addto(*Fout, t);
        ++Fout2;  ++Fout;
    } while (--cnt);
}

/* ---- butterfly radix-4 ---- */
static void kf_bfly4(kiss_fft_cpx *Fout, size_t fstride,
                     const kiss_fft_cfg st, size_t cnt)
{
    kiss_fft_cpx *tw1, *tw2, *tw3;
    kiss_fft_cpx  s[6];
    size_t k = cnt;

    tw3 = tw2 = tw1 = st->twiddles;
    do {
        cx_mul(s[0], Fout[cnt],    *tw1);
        cx_mul(s[1], Fout[2*cnt],  *tw2);
        cx_mul(s[2], Fout[3*cnt],  *tw3);

        cx_sub (s[5], *Fout, s[1]);
        cx_addto(*Fout, s[1]);
        cx_add (s[3], s[0], s[2]);
        cx_sub (s[4], s[0], s[2]);
        cx_sub (Fout[2*cnt], *Fout, s[3]);
        tw1 += fstride;  tw2 += fstride*2;  tw3 += fstride*3;
        cx_addto(*Fout, s[3]);

        if (st->inverse) {
            Fout[cnt].r   = s[5].r - s[4].i;
            Fout[cnt].i   = s[5].i + s[4].r;
            Fout[3*cnt].r = s[5].r + s[4].i;
            Fout[3*cnt].i = s[5].i - s[4].r;
        } else {
            Fout[cnt].r   = s[5].r + s[4].i;
            Fout[cnt].i   = s[5].i - s[4].r;
            Fout[3*cnt].r = s[5].r - s[4].i;
            Fout[3*cnt].i = s[5].i + s[4].r;
        }
        ++Fout;
    } while (--k);
}

/* ---- butterfly radix-3 ---- */
static void kf_bfly3(kiss_fft_cpx *Fout, size_t fstride,
                     const kiss_fft_cfg st, size_t cnt)
{
    size_t k = cnt;
    kiss_fft_cpx *tw1, *tw2;
    kiss_fft_cpx  s[5];
    kiss_fft_cpx  epi3 = st->twiddles[fstride * cnt];

    tw1 = tw2 = st->twiddles;
    do {
        cx_mul(s[1], Fout[cnt],   *tw1);
        cx_mul(s[2], Fout[2*cnt], *tw2);
        cx_add(s[3], s[1], s[2]);
        cx_sub(s[0], s[1], s[2]);
        tw1 += fstride;  tw2 += fstride*2;

        Fout[cnt].r = Fout->r - s[3].r * 0.5f;
        Fout[cnt].i = Fout->i - s[3].i * 0.5f;
        cx_scale(s[0], epi3.i);
        cx_addto(*Fout, s[3]);
        Fout[2*cnt].r = Fout[cnt].r + s[0].i;
        Fout[2*cnt].i = Fout[cnt].i - s[0].r;
        Fout[cnt].r  -= s[0].i;
        Fout[cnt].i  += s[0].r;
        ++Fout;
    } while (--k);
}

/* ---- butterfly radix-5 ---- */
static void kf_bfly5(kiss_fft_cpx *Fout, size_t fstride,
                     const kiss_fft_cfg st, int cnt)
{
    kiss_fft_cpx *F0 = Fout;
    kiss_fft_cpx *F1 = Fout +   cnt;
    kiss_fft_cpx *F2 = Fout + 2*cnt;
    kiss_fft_cpx *F3 = Fout + 3*cnt;
    kiss_fft_cpx *F4 = Fout + 4*cnt;
    kiss_fft_cpx  s[13];
    kiss_fft_cpx  ya = st->twiddles[fstride*cnt];
    kiss_fft_cpx  yb = st->twiddles[fstride*2*cnt];

    for (int u = 0; u < cnt; ++u) {
        s[0] = *F0;
        cx_mul(s[1], *F1, st->twiddles[u*fstride]);
        cx_mul(s[2], *F2, st->twiddles[2*u*fstride]);
        cx_mul(s[3], *F3, st->twiddles[3*u*fstride]);
        cx_mul(s[4], *F4, st->twiddles[4*u*fstride]);

        cx_add(s[7],  s[1], s[4]);
        cx_sub(s[10], s[1], s[4]);
        cx_add(s[8],  s[2], s[3]);
        cx_sub(s[9],  s[2], s[3]);

        F0->r += s[7].r + s[8].r;
        F0->i += s[7].i + s[8].i;

        s[5].r = s[0].r + s[7].r*ya.r + s[8].r*yb.r;
        s[5].i = s[0].i + s[7].i*ya.r + s[8].i*yb.r;
        s[6].r =  s[10].i*ya.i + s[9].i*yb.i;
        s[6].i = -s[10].r*ya.i - s[9].r*yb.i;

        cx_sub(*F1, s[5], s[6]);
        cx_add(*F4, s[5], s[6]);

        s[11].r = s[0].r + s[7].r*yb.r + s[8].r*ya.r;
        s[11].i = s[0].i + s[7].i*yb.r + s[8].i*ya.r;
        s[12].r = -s[10].i*yb.i + s[9].i*ya.i;
        s[12].i =  s[10].r*yb.i - s[9].r*ya.i;

        cx_add(*F2, s[11], s[12]);
        cx_sub(*F3, s[11], s[12]);
        ++F0; ++F1; ++F2; ++F3; ++F4;
    }
}

/* ---- butterfly generic (any radix) ---- */
static void kf_bfly_generic(kiss_fft_cpx *Fout, size_t fstride,
                             const kiss_fft_cfg st, int cnt, int p)
{
    int Norig = st->nfft;
    kiss_fft_cpx *scratch = (kiss_fft_cpx *)malloc((size_t)p * sizeof(kiss_fft_cpx));
    if (!scratch) return;

    for (int u = 0; u < cnt; ++u) {
        int k = u;
        for (int q1 = 0; q1 < p; ++q1) { scratch[q1] = Fout[k]; k += cnt; }
        k = u;
        for (int q1 = 0; q1 < p; ++q1) {
            Fout[k] = scratch[0];
            int twidx = 0;
            for (int q = 1; q < p; ++q) {
                twidx += (int)fstride * k;
                if (twidx >= Norig) twidx -= Norig;
                kiss_fft_cpx t;
                cx_mul(t, scratch[q], st->twiddles[twidx]);
                cx_addto(Fout[k], t);
            }
            k += cnt;
        }
    }
    free(scratch);
}

/* ---- recursive FFT work ---- */
static void kf_work(kiss_fft_cpx *Fout, const kiss_fft_cpx *f,
                    size_t fstride, int in_stride,
                    int *factors, const kiss_fft_cfg st)
{
    kiss_fft_cpx *Fout_beg = Fout;
    const int p   = *factors++;
    const int m   = *factors++;
    const kiss_fft_cpx *Fout_end = Fout + p * m;

    if (m == 1) {
        do { *Fout = *f; f += fstride * in_stride; } while (++Fout != Fout_end);
    } else {
        do {
            kf_work(Fout, f, fstride * p, in_stride, factors, st);
            f += fstride * in_stride;
        } while ((Fout += m) != Fout_end);
    }
    Fout = Fout_beg;

    switch (p) {
        case 2:  kf_bfly2(Fout, fstride, st, m);        break;
        case 3:  kf_bfly3(Fout, fstride, st, m);        break;
        case 4:  kf_bfly4(Fout, fstride, st, m);        break;
        case 5:  kf_bfly5(Fout, fstride, st, m);        break;
        default: kf_bfly_generic(Fout, fstride, st, m, p); break;
    }
}

/* ---- factorize n into small primes ---- */
static void kf_factor(int n, int *facbuf)
{
    int p = 4;
    double floor_sqrt = floor(sqrt((double)n));
    do {
        while (n % p) {
            switch (p) {
                case 4:  p = 2; break;
                case 2:  p = 3; break;
                default: p += 2; break;
            }
            if (p > floor_sqrt) p = n;
        }
        n /= p;
        *facbuf++ = p;
        *facbuf++ = n;
    } while (n > 1);
}

/* ---- public API ---- */

kiss_fft_cfg kiss_fft_alloc(int nfft, int inverse_fft, void *mem, size_t *lenmem)
{
    size_t memneeded = sizeof(struct kiss_fft_state)
                     + sizeof(kiss_fft_cpx) * (size_t)(nfft - 1);
    kiss_fft_cfg st = NULL;

    if (lenmem == NULL) {
        st = (kiss_fft_cfg)malloc(memneeded);
    } else {
        if (mem && *lenmem >= memneeded) st = (kiss_fft_cfg)mem;
        *lenmem = memneeded;
    }
    if (!st) return NULL;

    st->nfft    = nfft;
    st->inverse = inverse_fft;

    for (int i = 0; i < nfft; ++i) {
        double phase = -2.0 * M_PI * i / nfft;
        if (inverse_fft) phase = -phase;
        st->twiddles[i].r = (kiss_fft_scalar)cos(phase);
        st->twiddles[i].i = (kiss_fft_scalar)sin(phase);
    }
    kf_factor(nfft, st->factors);
    return st;
}

void kiss_fft_stride(kiss_fft_cfg st, const kiss_fft_cpx *fin,
                     kiss_fft_cpx *fout, int in_stride)
{
    if (fin == fout) {
        kiss_fft_cpx *tmp = (kiss_fft_cpx *)malloc(sizeof(kiss_fft_cpx) * (size_t)st->nfft);
        if (!tmp) return;
        kf_work(tmp, fin, 1, in_stride, st->factors, st);
        memcpy(fout, tmp, sizeof(kiss_fft_cpx) * (size_t)st->nfft);
        free(tmp);
    } else {
        kf_work(fout, fin, 1, in_stride, st->factors, st);
    }
}

void kiss_fft(kiss_fft_cfg cfg, const kiss_fft_cpx *fin, kiss_fft_cpx *fout)
{
    kiss_fft_stride(cfg, fin, fout, 1);
}

void kiss_fft_cleanup(void) {}

int kiss_fft_next_fast_size(int n)
{
    while (1) {
        int m = n;
        while ((m % 2) == 0) m /= 2;
        while ((m % 3) == 0) m /= 3;
        while ((m % 5) == 0) m /= 5;
        if (m <= 1) break;
        n++;
    }
    return n;
}
