#ifndef __FFT_H
#define __FFT_H
#include "arm_math.h"
void calc_window(int number);
void fft_calc(float32_t *src1,float32_t *src2,float32_t *out1,float32_t *out2,int n);
#endif
