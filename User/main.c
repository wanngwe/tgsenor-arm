/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块。
*	文件名称 : main.c
*	版    本 : V1.2
*	说    明 : 主程序
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2013-02-01 armfly  首发
*		v1.0    2013-02-01 armfly  升级BSP,增加操作提示
*		V1.2    2014-02-28 armfly  升级固件库到V1.3.0
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"		/* 如果要用ST的固件库，必须包含这个文件 */

/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"V5-120_AD7606（8通道16位同步ADC）例程"
#define EXAMPLE_DATE	"2014-02-28"
#define DEMO_VER		"1.2"


/* 仅允许本文件内调用的函数声明 */

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
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
#define DAC_OUT_FREQ	100000		/* DAC 输出样本频率 */
#define WAVE_SAMPLES	250		    /* 每周期样本数， 越大波形幅度越细腻，但是输出最大频率会降低 */
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
extern AD7606_FIFO_T g_tAdcFifo;	/* 定义FIFO结构体变量 */
extern AD7606_FIFO_T g_tAdcFifo1;
extern AD7606_FIFO_T *pFIFO;
extern float32_t hanning_window[128];
void VCP_SendData( USB_OTG_CORE_HANDLE *pdev, uint8_t* pbuf, uint32_t  buf_len);
/*********************************************************************************************************
*	函 数 名: MakeSinTable
*	功能说明: 计算产生正弦波数组
*	形    参: _pBuf : 目标缓冲区
*			  _usSamples : 每个周期的样本数 （建议大于32，并且是偶数）
*	返 回 值: 无
*********************************************************************************************************
*/
static void MakeSinTable(uint16_t *_pCBuf,uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t i;
	uint16_t mid;	/* 中值 */
	uint16_t att;	/* 幅度 */

	mid = (_usBottom + _usTop) / 2;	/* 0位的值 */
	att = (_usTop - _usBottom) / 2;  	/* 正弦波幅度，峰峰值除以2 */

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
*	函 数 名: arm_cfft_f32_app
*	功能说明: 调用函数arm_cfft_f32_app计算幅频。
*	形    参：无
*	返 回 值: 无
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
	bsp_InitAD7606();	                                 /* 配置AD7606所用的GPIO */
	AD7606_SetOS(AD_OS_NO);		                         /* 无过采样 */
	AD7606_SetInputRange(1);	                         /* 0表示输入量程为正负5V, 1表示正负10V */
	bsp_StartAutoTimer(0, 10);	                         /* 启动1个500ms的自动重装的定时器 */
	MakeSinTable(s_CWaveBuf,s_WaveBuf, WAVE_SAMPLES, 12768, 25768);
	calc_window(ADC_FIFO_SIZE);
	bsp_InitDAC8562();	                                 /* 初始化配置DAC8501E */
	bsp_SetTIMforInt(TIM7, DAC_OUT_FREQ, 0, 0);
	TIM4_Init();
	s_WavePos2 = 0;
	s_WavePos1=0;
	AD7606_StartRecord(10000);

	while (1)
	{	
			bsp_Idle();		                            /* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */
			encoder_value=getOpticalEncoder();
//			if (bsp_CheckTimer(0))	/* 判断定时器超时时间 */
//			{
//				/* 每隔500ms 进来一次 */
//				
////				//发送-1，-1，-1，-1

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
*	函 数 名: TIM4_IRQHandler
*	功能说明: TIM4 中断服务程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void TIM7_IRQHandler(void)
{
	uint16_t dac,dac1;

	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

		bsp_LedToggle(1);	/* 翻转LED1，指示中断频率 */

		/* 波形1 */
		dac = s_WaveBuf[s_WavePos1++];
		if (s_WavePos1 >= WAVE_SAMPLES)
		{
			s_WavePos1 = 0;
		}

		/* 波形1 */
		dac1 = s_CWaveBuf[s_WavePos2++];
		if (s_WavePos2 >= WAVE_SAMPLES)
		{
			s_WavePos2 = 0;
		}
		DAC8562_SetData(1,dac,dac1);		/* 改变第1.2通道 DAC输出电压 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: PrintfLogo
*	功能说明: 打印例程名称和例程发布日期, 接上串口线后，打开PC机的超级终端软件可以观察结果
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/

