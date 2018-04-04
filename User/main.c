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
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

 uint8_t Rxbuffer[64]; 
__IO uint32_t receive_count =1;
#include "math.h"

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
uint8_t headBuf[8];
uint8_t sendBuf[64];
int32_t encoder_value=0;
uint16_t encode_high16bit=0;
uint16_t encode_low16bit=0;
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
		_pBuf[i] = mid + (int32_t)(att * sin((i * 2 * 3.14159) / _usSamples));
		_pCBuf[i] = mid -1+ (int32_t)(att * cos((i * 2 * 3.14159) / _usSamples));
		//_pBuf[i] =0;
		//_pCBuf[i]=0;
		
	}
}
int getOpticalEncoder()
{
	static int last_position=0;
	int cur_position;
	int delta=0;
	cur_position =	TIM4_Encoder_Read();
	delta=cur_position-last_position;
	if(delta>MAX_COUNT) //zheng zhuan
	{
		
	
	}
	if (delta<-MAX_COUNT)// opposite running
	{
	
	
	}
	return ;
	



}
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
	  USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif  
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
	bsp_InitAD7606();	/* ����AD7606���õ�GPIO */
	AD7606_SetOS(AD_OS_NO);		/* �޹����� */
	AD7606_SetInputRange(1);	/* 0��ʾ��������Ϊ����5V, 1��ʾ����10V */
	bsp_StartAutoTimer(0, 100);	    /* ����1��500ms���Զ���װ�Ķ�ʱ�� */
	MakeSinTable(s_CWaveBuf,s_WaveBuf, WAVE_SAMPLES, 12768, 25768);
	bsp_InitDAC8562();	/* ��ʼ������DAC8501E */
	bsp_SetTIMforInt(TIM7, DAC_OUT_FREQ, 0, 0);
	TIM4_Init();
	//s_WavePos1 = WAVE_SAMPLES / 4;	/* ����1��ǰ 90�� */
	s_WavePos2 = 0;
	s_WavePos1=0;


	while (1)
	{
			bsp_Idle();		/* ���������bsp.c�ļ����û������޸��������ʵ��CPU���ߺ�ι�� */
			encoder_value=TIM4_Encoder_Read();
			if (bsp_CheckTimer(0))	/* �ж϶�ʱ����ʱʱ�� */
			{
				/* ÿ��500ms ����һ�� */
				bsp_LedToggle(4);	/* ��תLED4��״̬ */
				//����-1��-1��-1��-1

				AD7606_StartRecord(10000);
				
				encode_high16bit=encoder_value/20000;
				encode_low16bit =encoder_value%20000;
				//encode_low16bit =encoder_value&0xffff;
//				headBuf[0]=0x7f;
//				headBuf[1]=0xff;
//				headBuf[2]=0x7f;
//				headBuf[3]=0xff;
//				headBuf[4]=encode_high16bit>>8;
//				headBuf[5]=encode_high16bit&0xff;
//				headBuf[6]=encode_low16bit>>8;
//				headBuf[7]=encode_low16bit&0xff;
				sprintf(headBuf,"%d\r\n",encoder_value);
				VCP_SendData(&USB_OTG_dev, headBuf, sizeof(headBuf));
				//transmit(headBuf,sizeof(headBuf));
			}
//			if(change_old!=change)
//			{
//				change_old=change;
//				if(change==1)
//				{
//					for(i=0;i<32;i++)
//					{
//						sendBuf[2*i]=g_tAdcFifo1.sBuf[i]>>8;
//						sendBuf[2*i+1]=g_tAdcFifo1.sBuf[i]&0xff;					
//					}
//				
//				}
//				else
//				{
//					for(i=0;i<32;i++)
//					{
//						sendBuf[2*i]=g_tAdcFifo.sBuf[i]>>8;
//						sendBuf[2*i+1]=g_tAdcFifo.sBuf[i]&0xff;					
//					}				
//				
//				
//				}
				//VCP_SendData(&USB_OTG_dev, sendBuf, 64);
				
			
			//}
		
			
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

