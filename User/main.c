/*
*********************************************************************************************************
*
*	ģ������ : ������ģ�顣
*	�ļ����� : main.c
*	��    �� : V1.2
*	˵    �� : ������
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2013-02-01 armfly  �׷�
*		v1.0    2013-02-01 armfly  ����BSP,���Ӳ�����ʾ
*		V1.2    2014-02-28 armfly  �����̼��⵽V1.3.0
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"		/* ���Ҫ��ST�Ĺ̼��⣬�����������ļ� */

/* ���������������̷������� */
#define EXAMPLE_NAME	"V5-120_AD7606��8ͨ��16λͬ��ADC������"
#define EXAMPLE_DATE	"2014-02-28"
#define DEMO_VER		"1.2"


/* �������ļ��ڵ��õĺ������� */

static void DispMenu(void);
#include "usbd_cdc_core_loopback.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "arm_math.h"
#include "arm_const_structs.h"
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

 uint8_t Rxbuffer[64]; 
__IO uint32_t receive_count =1;

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: c�������
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
#define DAC_OUT_FREQ	100000		/* DAC �������Ƶ�� */
#define WAVE_SAMPLES	250		    /* ÿ������������ Խ���η���Խϸ�壬����������Ƶ�ʻή�� */
static uint16_t s_WavePos1  = 0;
static uint16_t s_WavePos2  = 0;
static uint16_t s_WaveBuf[WAVE_SAMPLES];
static uint16_t s_CWaveBuf[WAVE_SAMPLES];
uint8_t headBuf[16];
uint8_t sendBuf[64];
int32_t encoder_value=0;
uint16_t encode_high16bit=0;
uint16_t encode_low16bit=0;
int32_t sum1=0;
int32_t sum2=0;
float32_t mean1;
float32_t mean2;
extern u8 Rx_buffer[2048];
extern u32 Rx_length;
extern int change;
extern AD7606_FIFO_T g_tAdcFifo;	/* ����FIFO�ṹ����� */
extern AD7606_FIFO_T g_tAdcFifo1;
void transmit(uint8_t* Buf, uint32_t Len);
/*********************************************************************************************************
*	�� �� ��: MakeSinTable
*	����˵��: ����������Ҳ�����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������ż����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MakeSinTable(uint16_t *_pCBuf,uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t i;
	uint16_t mid;	/* ��ֵ */
	uint16_t att;	/* ���� */

	mid = (_usBottom + _usTop) / 2;	/* 0λ��ֵ */
	att = (_usTop - _usBottom) / 2;  	/* ���Ҳ����ȣ����ֵ����2 */

	for (i = 0; i < _usSamples; i++)
	{
		_pBuf[i] = mid + (int32_t)(att * arm_sin_f32((i * 2 * 3.14159) / _usSamples));
		_pCBuf[i] = mid -1+ (int32_t)(att * arm_cos_f32((i * 2 * 3.14159) / _usSamples));
	}
}
int getOpticalEncoder()
{
	static int last_position=0;
	static int position=0;
	int cur_position;
	int delta=0;
	cur_position =	TIM4_Encoder_Read();
	delta=last_position-cur_position;
	if(delta>MAX_COUNT/2) //zheng zhuan
	{
		position=position+MAX_COUNT;
	
	}
	else if (delta+(MAX_COUNT/2)<0)// opposite running
	{
	
		position=position-MAX_COUNT;
	}
	last_position=cur_position;
	return (position+cur_position);
	



}
/*
*********************************************************************************************************
*	�� �� ��: arm_cfft_f32_app
*	����˵��: ���ú���arm_cfft_f32_app�����Ƶ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint32_t fftSize = 128; 
uint32_t ifftFlag = 0; 
uint32_t doBitReverse = 1;
#define TEST_LENGTH_SAMPLES 2048 
static float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t travlling_wave[2*ADC_FIFO_SIZE];
static float32_t ref_wave[2*ADC_FIFO_SIZE];
static float32_t testOutput[ADC_FIFO_SIZE]; 
static float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
float32_t phase=0;
float32_t phase1=0.0f;
float32_t phase2=0.0f;

void PrintfLogo(void);
int main(void)
{
	int16_t data;
	int i=0;
	int change_old=0;
	
	/*
		����ST�̼���������ļ��Ѿ�ִ����CPUϵͳʱ�ӵĳ�ʼ�������Բ����ٴ��ظ�����ϵͳʱ�ӡ�
		�����ļ�������CPU��ʱ��Ƶ�ʡ��ڲ�Flash�����ٶȺͿ�ѡ���ⲿSRAM FSMC��ʼ����

		ϵͳʱ��ȱʡ����Ϊ72MHz�������Ҫ���ģ������޸ģ�
		\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x\system_stm32f10x.c
		������ϵͳʱ�ӵĺꡣ
	*/

	  bsp_Init();
	 
//	  USBD_Init(&USB_OTG_dev,
//#ifdef USE_USB_OTG_HS 
//            USB_OTG_HS_CORE_ID,
//#else            
//            USB_OTG_FS_CORE_ID,
//#endif  
//            &USR_desc, 
//            &USBD_CDC_cb, 
//            &USR_cb);
	bsp_InitAD7606();	/* ����AD7606���õ�GPIO */
	AD7606_SetOS(AD_OS_NO);		/* �޹����� */
	AD7606_SetInputRange(1);	/* 0��ʾ��������Ϊ����5V, 1��ʾ����10V */
	bsp_StartAutoTimer(0, 10);	    /* ����1��500ms���Զ���װ�Ķ�ʱ�� */
	MakeSinTable(s_CWaveBuf,s_WaveBuf, WAVE_SAMPLES, 12768, 25768);
	bsp_InitDAC8562();	/* ��ʼ������DAC8501E */
	bsp_SetTIMforInt(TIM7, DAC_OUT_FREQ, 0, 0);
	TIM4_Init();
	s_WavePos2 = 0;
	s_WavePos1=0;
	AD7606_StartRecord(10000);

	while (1)
	{	
			bsp_Idle();		/* ���������bsp.c�ļ����û������޸��������ʵ��CPU���ߺ�ι�� */
			encoder_value=getOpticalEncoder();
//			if (bsp_CheckTimer(0))	/* �ж϶�ʱ����ʱʱ�� */
//			{
//				/* ÿ��500ms ����һ�� */
//				
////				//����-1��-1��-1��-1

////				
////				
////				encode_high16bit=encoder_value/20000;
////				encode_low16bit =encoder_value%20000;
////				//encode_low16bit =encoder_value&0xffff;
////				headBuf[0]=0x7f;
////				headBuf[1]=0xff;
////				headBuf[2]=0x7f;
////				headBuf[3]=0xff;
////				headBuf[4]=encode_high16bit>>8;
////				headBuf[5]=encode_high16bit&0xff;
////				headBuf[6]=encode_low16bit>>8;
////				headBuf[7]=encode_low16bit&0xff;


////				sprintf(headBuf,"%d\r\n",encoder_value);
//				VCP_SendData(&USB_OTG_dev, headBuf, sizeof(headBuf));
////				AD7606_StartRecord(10000);
//			}
//			else
			{	
				if(change_old!=change)
				{
					change_old=change;
					if(change==1)
					{
#ifdef ONLINE							
						bsp_LedOff(3);
						//step 1 mean value
						for(i=0;i<ADC_FIFO_SIZE;i++)
						{
							sum1+=g_tAdcFifo1.travelBuf[i];
							sum2+=g_tAdcFifo1.refBuf[i];														
						}
						mean1=sum1/ADC_FIFO_SIZE;
						mean2=sum2/ADC_FIFO_SIZE;
						//AD7606_StopRecord();
						//step 2:remove mean value
						for(i=0;i<ADC_FIFO_SIZE;i++)
						{
//							sendBuf[2*i]=g_tAdcFifo1.sBuf[i]>>8;
//							sendBuf[2*i+1]=g_tAdcFifo1.sBuf[i]&0xff;
							travlling_wave[2*i]      =(float32_t)g_tAdcFifo1.travelBuf[i]-mean1;
							travlling_wave[2*i+1]    =0.0f;
							ref_wave[2*i]            =(float32_t)g_tAdcFifo1.refBuf[i]-mean2;
							ref_wave[2*i+1]          =0.0f;
							//printf("%d\r\n",(int)ref_wave[2*i]);
							//printf("%d\r\n",(int)travlling_wave[2*i]);
							/* CFFT�任 */ 
														
						}
						// step 3: fft
						arm_cfft_f32(&arm_cfft_sR_f32_len128, travlling_wave, ifftFlag, doBitReverse);
						arm_cfft_f32(&arm_cfft_sR_f32_len128, ref_wave, ifftFlag, doBitReverse);
						//arm_cmplx_mag_f32(ref_wave, testOutput, fftSize);
						phase1=atan2(travlling_wave[11],travlling_wave[10]);
						phase2=atan2(ref_wave[11],ref_wave[10]);
						//printf("%.3f\r\n",phase1*180/3.14159);
						//printf("%.3f\r\n",phase2*180/3.14159);
						printf("%.3f\r\n",(phase2-phase1)*180/3.14159);
						phase=atan2(travlling_wave[10]*ref_wave[10]+travlling_wave[11]*ref_wave[11],
						travlling_wave[11]*ref_wave[10]-travlling_wave[10]*ref_wave[11]
						   );
						printf("1111  %.3f\r\n",(phase)*180/3.14159);
#endif
						
					//	printf("////");
						//for(i=0;i<128;i++)
						{
						//	sprintf(headBuf,"sss%d   ",(int)(ref_wave[i]));
						//	VCP_SendData(&USB_OTG_dev, headBuf, sizeof(headBuf));
						//	printf("%.3f\r\n",testOutput[i]);
						}
						//sprintf(headBuf,"*******",ref_wave[i]);						
						bsp_LedOn(3);
					}
					else
					{
						for(i=0;i<ADC_FIFO_SIZE/2;i++)
						{
//							sendBuf[2*i]=g_tAdcFifo.sBuf[i]>>8;
//							sendBuf[2*i+1]=g_tAdcFifo.sBuf[i]&0xff;
					//		travlling_wave[2*i]=(float32_t)g_tAdcFifo1.sBuf[i];
					//		ref_wave[2*i]       =(float32_t)g_tAdcFifo1.sBuf[2*i+1];							
						}									
					}
//					VCP_SendData(&USB_OTG_dev, sendBuf, 64);
//					
//				
				}
  
			}		
	}
}


/*********************************************************************************************************
*	�� �� ��: TIM4_IRQHandler
*	����˵��: TIM4 �жϷ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void TIM7_IRQHandler(void)
{
	uint16_t dac,dac1;

	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

		bsp_LedToggle(1);	/* ��תLED1��ָʾ�ж�Ƶ�� */

		/* ����1 */
		dac = s_WaveBuf[s_WavePos1++];
		if (s_WavePos1 >= WAVE_SAMPLES)
		{
			s_WavePos1 = 0;
		}

		/* ����1 */
		dac1 = s_CWaveBuf[s_WavePos2++];
		if (s_WavePos2 >= WAVE_SAMPLES)
		{
			s_WavePos2 = 0;
		}
		DAC8562_SetData(1,dac,dac1);		/* �ı��1.2ͨ�� DAC�����ѹ */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: PrintfLogo
*	����˵��: ��ӡ�������ƺ����̷�������, ���ϴ����ߺ󣬴�PC���ĳ����ն�������Թ۲���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void PrintfLogo(void)
{
	/* ���CPU ID */
	{
		/* �ο��ֲ᣺
			32.6.1 MCU device ID code
			33.1 Unique device ID register (96 bits)
		*/
		uint32_t CPU_Sn0, CPU_Sn1, CPU_Sn2;

		CPU_Sn0 = *(__IO uint32_t*)(0x1FFF7A10);
		CPU_Sn1 = *(__IO uint32_t*)(0x1FFF7A10 + 4);
		CPU_Sn2 = *(__IO uint32_t*)(0x1FFF7A10 + 8);

		printf("\r\nCPU : STM32F407IGT6, LQFP176, UID = %08X %08X %08X\n\r"
			, CPU_Sn2, CPU_Sn1, CPU_Sn0);
	}

	printf("\n\r");
	printf("*************************************************************\n\r");
	printf("* ��������   : %s\r\n", EXAMPLE_NAME);	/* ��ӡ�������� */
	printf("* ���̰汾   : %s\r\n", DEMO_VER);		/* ��ӡ���̰汾 */
	printf("* ��������   : %s\r\n", EXAMPLE_DATE);	/* ��ӡ�������� */

	/* ��ӡST�̼���汾����3���������stm32f10x.h�ļ��� */
	printf("* �̼���汾 : V%d.%d.%d (STM32F4xx_StdPeriph_Driver)\r\n", __STM32F4XX_STDPERIPH_VERSION_MAIN,
			__STM32F4XX_STDPERIPH_VERSION_SUB1,__STM32F4XX_STDPERIPH_VERSION_SUB2);
	printf("* \r\n");	/* ��ӡһ�пո� */
	printf("* QQ    : 1295744630 \r\n");
	printf("* ����  : armfly\r\n");
	printf("* Email : armfly@qq.com \r\n");
	printf("* �Ա���: armfly.taobao.com\r\n");
	printf("* Copyright www.armfly.com ����������\r\n");
	printf("*************************************************************\n\r");
}

