#include "bsp.h"
#include "arm_math.h"
#include "arm_const_structs.h"
uint32_t fftSize = 128; 
uint32_t ifftFlag = 0; 
uint32_t doBitReverse = 1;
float32_t hanning_window[128];
void calc_window(int number)
{
	int i=0;
	for(i=0;i<number;i++)
	{
		hanning_window[i]=0.5-0.5*arm_cos_f32((2 * PI*(i - 1))/(number - 1));
	}
}
void fft_calc(float32_t *src1,float32_t *src2,float32_t *out1,float32_t *out2,int n)
{
	float32_t sum1   =0.0f;
	float32_t mean1  =0.0f;
	float32_t sum2   =0.0f;
	float32_t mean2  =0.0f;
	int i=0;
	
	for(i=0;i<n;i++)
	{
		sum1+=src1[i];
		sum2+=src1[i];		
	}
	mean1=sum1/n;
	mean2=sum1/n;
	//AD7606_StopRecord();
	//step 2:remove mean value
	for(i=0;i<ADC_FIFO_SIZE;i++)
	{
		out1[2*i]      =src1[i]-mean1;
		out1[2*i+1]    =0.0f;
		out2[2*i]      =src2[i]-mean2;
		out2[2*i+1]    =0.0f;		
	}
	/* CFFT±ä»» */ 
	// step 3: fft
	arm_cfft_f32(&arm_cfft_sR_f32_len128, out1, ifftFlag, doBitReverse);
	arm_cfft_f32(&arm_cfft_sR_f32_len128, out2, ifftFlag, doBitReverse);
}


