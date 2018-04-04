#ifndef __ENCODE_H__
#define __ENCODE_H__
//#include "sys.h" 
#define MAX_COUNT (0xffff-1)
void TIM4_Init(void);
uint16_t TIM4_Encoder_Read(void);

#endif
