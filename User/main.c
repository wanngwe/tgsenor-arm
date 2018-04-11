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
int16_t sendBuf[ADC_FIFO_SIZE*2];
float32_t temp1[ADC_FIFO_SIZE];
float32_t temp2[ADC_FIFO_SIZE];
int16_t testBuf[4]={0xffee,0x5511,0x3344,0x7788};
int32_t encoder_value=0;
uint16_t encode_high16bit=0;
uint16_t encode_low16bit=0;
int32_t sum1=0;
int32_t sum2=0;
float32_t mean1;
float32_t mean2;
uint16_t upload_degree=0;
extern u8 Rx_buffer[2048];
extern u32 Rx_length;
extern int change;
extern AD7606_FIFO_T g_tAdcFifo;	/* ����FIFO�ṹ����� */
extern AD7606_FIFO_T g_tAdcFifo1;
extern AD7606_FIFO_T *pFIFO;
extern float32_t hanning_window[128];
void VCP_SendData( USB_OTG_CORE_HANDLE *pdev, uint8_t* pbuf, uint32_t  buf_len);
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
		_pBuf[i] = mid + (int32_t)(att * arm_sin_f32((i * 2 * PI) / _usSamples));
		_pCBuf[i] = mid -1+ (int32_t)(att * arm_cos_f32((i * 2 *PI) / _usSamples));
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

static float32_t travlling_wave[2*ADC_FIFO_SIZE];
static float32_t ref_wave[2*ADC_FIFO_SIZE];
float32_t phase=0;
float32_t phase1=0.0f;
float32_t phase2=0.0f;

void PrintfLogo(void);
int main(void)
{
	int i=0;
	int change_old=0;
	bsp_Init();

	USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
		USB_OTG_HS_CORE_ID,
#else            
		USB_OTG_FS_CORE_ID,
#endif  
		&USR_desc, 
		&USBD_CDC_cb, 
		&USR_cb);
	bsp_InitAD7606();	                                 /* ����AD7606���õ�GPIO */
	AD7606_SetOS(AD_OS_NO);		                         /* �޹����� */
	AD7606_SetInputRange(1);	                         /* 0��ʾ��������Ϊ����5V, 1��ʾ����10V */
	bsp_StartAutoTimer(0, 10);	                         /* ����1��500ms���Զ���װ�Ķ�ʱ�� */
	MakeSinTable(s_CWaveBuf,s_WaveBuf, WAVE_SAMPLES, 12768, 25768);
	calc_window(ADC_FIFO_SIZE);
	bsp_InitDAC8562();	                                 /* ��ʼ������DAC8501E */
	bsp_SetTIMforInt(TIM7, DAC_OUT_FREQ, 0, 0);
	TIM4_Init();
	s_WavePos2 = 0;
	s_WavePos1=0;
	AD7606_StartRecord(10000);

	while (1)
	{	
			bsp_Idle();		                            /* ���������bsp.c�ļ����û������޸��������ʵ��CPU���ߺ�ι�� */
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
//				VCP_SendData(&USB_OTG_dev, headBuf, sizeof(headBuf));
//			}
//			else
//			{	
				if(change_old!=change)
				{
					change_old=change;
					if(change==1)
						pFIFO=&g_tAdcFifo1;
					else if(change==0)
						pFIFO=&g_tAdcFifo;												
#ifdef ONLINE							
					bsp_LedOff(3);
					*pFIFO=g_tAdcFifo1;
					for(i=0;i<ADC_FIFO_SIZE;i++)
					{
							temp1[i]=(float32_t)pFIFO->refBuf[i]    *hanning_window[i];
							temp2[i]=(float32_t)pFIFO->travelBuf[i] *hanning_window[i];
					}
					fft_calc(temp1,temp2,travlling_wave,ref_wave,ADC_FIFO_SIZE);
					phase=atan2(travlling_wave[11]*ref_wave[10]-travlling_wave[10]*ref_wave[11],
								travlling_wave[10]*ref_wave[10]+travlling_wave[11]*ref_wave[11]);
					upload_degree=(uint16_t)((phase+PI)*180000/(PI*166));
					bsp_LedOn(3);
					VCP_SendData(&USB_OTG_dev,(uint8_t*)&upload_degree, 2);
#else						
					for(i=0;i<ADC_FIFO_SIZE;i++)
					{
						sendBuf[2*i]    =pFIFO->refBuf[i];
						sendBuf[2*i+1]  =pFIFO->travelBuf[i];												
					}					
					VCP_SendData(&USB_OTG_dev,(uint8_t*)sendBuf, ADC_FIFO_SIZE*4);
#endif																								
				}
  
			}		
//	}
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

