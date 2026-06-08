#ifndef KISS_FFT_H
#define KISS_FFT_H
/* KissFFT - Mark Borgerding (BSD-3-Clause)
 * Self-contained C++ version for Arduino/ESP32 */

#include <stdlib.h>
#include <stddef.h>

typedef float kiss_fft_scalar;

typedef struct {
    kiss_fft_scalar r;
    kiss_fft_scalar i;
} kiss_fft_cpx;

#define MAXFACTORS 32

struct kiss_fft_state {
    int nfft;
    int inverse;
    int factors[2 * MAXFACTORS];
    kiss_fft_cpx twiddles[1];
};
typedef struct kiss_fft_state *kiss_fft_cfg;

kiss_fft_cfg kiss_fft_alloc(int nfft, int inverse_fft, void *mem, size_t *lenmem);
void kiss_fft(kiss_fft_cfg cfg, const kiss_fft_cpx *fin, kiss_fft_cpx *fout);
void kiss_fft_stride(kiss_fft_cfg cfg, const kiss_fft_cpx *fin, kiss_fft_cpx *fout, int fin_stride);
void kiss_fft_cleanup(void);
int  kiss_fft_next_fast_size(int n);

#define kiss_fft_free free

#endif /* KISS_FFT_H */
