/*
*********************************************************************************************************
*
*	ģ������ : DAC8562 ����ģ��(��ͨ����16λDAC)
*	�ļ����� : bsp_dac8562.c
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_DAC8562_H
#define _BSP_DAC8562_H

void bsp_InitDAC8562(void);
void DAC8562_SetData(uint8_t _ch, uint16_t _dac,uint16_t _dac1);
void DAC8562_WriteCmd(uint32_t _cmd);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/