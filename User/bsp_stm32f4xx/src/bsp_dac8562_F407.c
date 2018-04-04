/*
*********************************************************************************************************
*
*	模块名称 : DAC8562/8563 驱动模块(单通道带16位DAC)
*	文件名称 : bsp_dac8562.c
*	版    本 : V1.0
*	说    明 : DAC8562/8563模块和CPU之间采用SPI接口。本驱动程序支持硬件SPI接口和软件SPI接口。
*			  通过宏切换。
*
*	修改记录 :
*		版本号  日期         作者     说明
*		V1.0    2014-01-17  armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

//#define SOFT_SPI		/* 定义此行表示使用GPIO模拟SPI接口 */
#define HARD_SPI		/* 定义此行表示使用CPU的硬件SPI接口 */

/*
	DAC8501模块可以直接插到STM32-V5开发板CN19排母(2*4P 2.54mm)接口上

    DAC8562/8563模块    STM32F407开发板
	  GND   ------  GND    
	  VCC   ------  3.3V
	  
	  LDAC  ------  PA4/NRF905_TX_EN/NRF24L01_CE/DAC1_OUT
      SYNC  ------  PF7/NRF24L01_CSN
      	  
      SCLK  ------  PB3/SPI3_SCK
      DIN   ------  PB5/SPI3_MOSI

			------  PB4/SPI3_MISO
	  CLR   ------  PH7/NRF24L01_IRQ
*/

/*
	DAC8562基本特性:
	1、供电2.7 - 5V;  【本例使用3.3V】
	4、参考电压2.5V，使用内部参考

	对SPI的时钟速度要求: 高达50MHz， 速度很快.
	SCLK下降沿读取数据, 每次传送24bit数据， 高位先传
*/

#if !defined(SOFT_SPI) && !defined(HARD_SPI)
 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
#endif

#ifdef SOFT_SPI		/* 软件SPI */
	/* 定义GPIO端口 */
	#define RCC_SCLK 	RCC_AHB1Periph_GPIOB
	#define PORT_SCLK	GPIOB
	#define PIN_SCLK	GPIO_Pin_3
	
	#define RCC_DIN 	RCC_AHB1Periph_GPIOB
	#define PORT_DIN 	GPIOB
	#define PIN_DIN 	GPIO_Pin_5
	
	/* 片选 */
	#define RCC_SYNC 	RCC_AHB1Periph_GPIOF
	#define PORT_SYNC	GPIOF
	#define PIN_SYNC	GPIO_Pin_7

	/* LDAC, 可以不用 */
	#define RCC_LDAC	RCC_AHB1Periph_GPIOA
	#define PORT_LDAC	GPIOA
	#define PIN_LDAC	GPIO_Pin_4
	
	/* CLR, 可以不用 */
	#define RCC_CLR 	RCC_AHB1Periph_GPIOF
	#define PORT_CLR	GPIOF
	#define PIN_CLR 	GPIO_Pin_7

	/* 定义口线置0和置1的宏 */
	#define SYNC_0()	PORT_SYNC->BSRRH = PIN_SYNC
	#define SYNC_1()	PORT_SYNC->BSRRL = PIN_SYNC

	#define SCLK_0()	PORT_SCLK->BSRRH = PIN_SCLK
	#define SCLK_1()	PORT_SCLK->BSRRL = PIN_SCLK

	#define DIN_0()		PORT_DIN->BSRRH = PIN_DIN
	#define DIN_1()		PORT_DIN->BSRRL = PIN_DIN

	#define LDAC_0()	PORT_LDAC->BSRRH = PIN_LDAC
	#define LDAC_1()	PORT_LDAC->BSRRL = PIN_LDAC

	#define CLR_0()		PORT_CLR->BSRRH = PIN_CLR
	#define CLR_1()		PORT_CLR->BSRRL = PIN_CLR	
#endif

#ifdef HARD_SPI		/* 硬件SPI (未做) */

	/* 定义GPIO端口 */
	#define RCC_SCLK 	RCC_AHB1Periph_GPIOB
	#define PORT_SCLK	GPIOB
	#define PIN_SCLK	GPIO_Pin_3
	
	#define RCC_DIN 	RCC_AHB1Periph_GPIOB
	#define PORT_DIN 	GPIOB
	#define PIN_DIN 	GPIO_Pin_5
	
	/* 片选 */
	#define RCC_SYNC 	RCC_AHB1Periph_GPIOF
	#define PORT_SYNC	GPIOF
	#define PIN_SYNC	GPIO_Pin_7

	/* LDAC, 可以不用 */
	#define RCC_LDAC	RCC_AHB1Periph_GPIOA
	#define PORT_LDAC	GPIOA
	#define PIN_LDAC	GPIO_Pin_4
	
	/* CLR, 可以不用 */
	#define RCC_CLR 	RCC_AHB1Periph_GPIOF
	#define PORT_CLR	GPIOF
	#define PIN_CLR 	GPIO_Pin_7

	/* 定义口线置0和置1的宏 */
	#define SYNC_0()	PORT_SYNC->BSRRH = PIN_SYNC
	#define SYNC_1()	PORT_SYNC->BSRRL = PIN_SYNC

	#define SCLK_0()	PORT_SCLK->BSRRH = PIN_SCLK
	#define SCLK_1()	PORT_SCLK->BSRRL = PIN_SCLK

	#define DIN_0()		PORT_DIN->BSRRH = PIN_DIN
	#define DIN_1()		PORT_DIN->BSRRL = PIN_DIN

	#define LDAC_0()	PORT_LDAC->BSRRH = PIN_LDAC
	#define LDAC_1()	PORT_LDAC->BSRRL = PIN_LDAC

	#define CLR_0()		PORT_CLR->BSRRH = PIN_CLR
	#define CLR_1()		PORT_CLR->BSRRL = PIN_CLR	;
#endif
/*
*********************************************************************************************************
*	函 数 名: nRF24L01_CfgSpiHard
*	功能说明: 配置STM32内部SPI硬件的工作模式、速度等参数，用于访问nRF24L01。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void dac8562_CfgSpiHard(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	
	/*
	STM32F4XX 时钟计算.
		HCLK = 168M
		PCLK1 = HCLK / 4 = 42M
		PCLK2 = HCLK / 2 = 84M

		SPI2、SPI3 在 PCLK1, 时钟42M
		SPI1       在 PCLK2, 时钟84M

		STM32F4 支持的最大SPI时钟为 37.5 Mbits/S, 因此需要分频。下面使用的是SPI1。
	*/
	/* 配置SPI硬件参数 */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	/* 数据方向：2线全双工 */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		/* STM32的SPI工作模式 ：主机模式 */
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	/* 数据位长度 ： 8位 */
	/* SPI_CPOL和SPI_CPHA结合使用决定时钟和数据采样点的相位关系、
	   本例配置: 总线空闲是低电平,第1个边沿采样数据。
	*/
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			/* 片选控制方式：软件控制 */

	/* 设置波特率预分频系数 84MHz / 16 = 5.25MHz，24L01支持的最大SPI时钟为10MHz
	   这里选择SPI_BaudRatePrescaler_8容易出错。
	*/
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	/* 数据位传输次序：高位先传 */
	SPI_InitStructure.SPI_CRCPolynomial = 7;			/* CRC多项式寄存器，复位后为7。本例程不用 */
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);				/* 使能SPI  */
}
/*
*********************************************************************************************************
*	函 数 名: nRF24L01_WriteReadByte
*	功能说明: 用于向NRF读/写一字节数据
*	形    参: 写入的数据
*	返 回 值: 读取得的数据		        
*********************************************************************************************************
*/
static uint8_t dac8562_WriteReadByte(uint8_t dat)
{  	
	/* 当SPI发送缓冲器非空时等待 */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 通过SPI发送一字节数据 */
	SPI_I2S_SendData(SPI1, dat);		
	
	/* 当SPI接收缓冲器为空时等待 */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* 通过SPI接收一字节数据 */
	return SPI_I2S_ReceiveData(SPI1);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitDAC8562
*	功能说明: 配置STM32的GPIO和SPI接口，用于连接 DAC8562
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitDAC8562(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef SOFT_SPI
	SYNC_1();	/* SYNC = 1 */

	/* 打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_SCLK | RCC_DIN | RCC_SYNC, ENABLE);

	/* 配置几个推完输出IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* 设为输出口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 上下拉电阻不使能 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

	GPIO_InitStructure.GPIO_Pin = PIN_SCLK;
	GPIO_Init(PORT_SCLK, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DIN;
	GPIO_Init(PORT_DIN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_SYNC;
	GPIO_Init(PORT_SYNC, &GPIO_InitStructure);
	

#endif
#define dac8652_RCC_SPI  RCC_APB2Periph_SPI1
#define nRF24L01_PORT_CE   GPIOA
#define nRF24L01_PIN_CE    GPIO_Pin_4

#define nRF24L01_PORT_CSN  GPIOF
#define nRF24L01_PIN_CSN   GPIO_Pin_7

#define nRF24L01_PORT_IRQ  GPIOH
#define nRF24L01_PIN_IRQ   GPIO_Pin_7

/* SPI接口的SCK MOSI MISO端口定义 */
#define DAC8562_PORT_SPI	GPIOB
#define nRF24L01_PIN_SCK	GPIO_Pin_3
#define nRF24L01_PIN_MISO	GPIO_Pin_4
#define nRF24L01_PIN_MOSI	GPIO_Pin_5

#define DAC8562_PinSource_SCK	 GPIO_PinSource3
#define DAC8562_PinSource_MISO	 GPIO_PinSource4
#define DAC8562_PinSource_MOSI	 GPIO_PinSource5

#define DAC8562_AF_SPI    GPIO_AF_SPI1
	

	/* 打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_SCLK | RCC_DIN | RCC_SYNC, ENABLE);

	/* 配置几个推完输出IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* 设为输出口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 上下拉电阻不使能 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

	GPIO_InitStructure.GPIO_Pin = PIN_SCLK;
	GPIO_Init(PORT_SCLK, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DIN;
	GPIO_Init(PORT_DIN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_SYNC;
	GPIO_Init(PORT_SYNC, &GPIO_InitStructure);
	SYNC_1();	/* SYNC = 1 */
	
	GPIO_InitStructure.GPIO_Pin = PIN_LDAC;
	GPIO_Init(PORT_LDAC, &GPIO_InitStructure);
	LDAC_1();
	//spi configure
	/* 配置 SCK, MISO , MOSI 为复用功能 */
	/* 使能SPI1时钟 */
	RCC_APB2PeriphClockCmd(dac8652_RCC_SPI, ENABLE);
	GPIO_PinAFConfig(DAC8562_PORT_SPI, DAC8562_PinSource_SCK,  DAC8562_AF_SPI);
	GPIO_PinAFConfig(DAC8562_PORT_SPI, DAC8562_PinSource_MISO, DAC8562_AF_SPI);
	GPIO_PinAFConfig(DAC8562_PORT_SPI, DAC8562_PinSource_MOSI, DAC8562_AF_SPI);

	/* 注意GPIO速度最好不好超过25MHz，要不容易出错 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = nRF24L01_PIN_SCK | nRF24L01_PIN_MISO | nRF24L01_PIN_MOSI;
	GPIO_Init(DAC8562_PORT_SPI, &GPIO_InitStructure);
	
	dac8562_CfgSpiHard();
	/* Power up DAC-A and DAC-B */
//	DAC8562_WriteCmd((4 << 19) | (0 << 16) | (3 << 0));
	SYNC_0();
	dac8562_WriteReadByte(0x20);
	dac8562_WriteReadByte(0);
	dac8562_WriteReadByte(0x3);
	SYNC_1();
	
	/* LDAC pin inactive for DAC-B and DAC-A  不使用LDAC引脚更新数据 */
	//DAC8562_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));
	
	SYNC_0();
	dac8562_WriteReadByte(0x30);
	dac8562_WriteReadByte(0);
	dac8562_WriteReadByte(0x0);
	SYNC_1();

	/* 复位2个DAC到中间值, 输出2.5V */
	DAC8562_SetData(0, 32767,32767);
	DAC8562_SetData(1, 32767,32767);

	/* 选择内部参考并复位2个DAC的增益=2 （复位时，内部参考是禁止的) */
	//DAC8562_WriteCmd((7 << 19) | (0 << 16) | (1 << 0));
	SYNC_0();
	dac8562_WriteReadByte(0x38);
	dac8562_WriteReadByte(0);
	dac8562_WriteReadByte(1);
	SYNC_1();	
}


/*********************************************************************************************************
*	函 数 名: DAC8562_WriteCmd
*	功能说明: 向SPI总线发送24个bit数据。
*	形    参: _cmd : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
//void DAC8562_WriteCmd(uint32_t _cmd)
//{
//	uint8_t i;
//	
//	SYNC_0();
//	
//	/*　DAC8562 SCLK时钟高达50M，因此可以不延迟 */
//	for(i = 0; i < 24; i++)
//	{
//		if (_cmd & 0x800000)
//		{
//			DIN_1();
//		}
//		else
//		{
//			DIN_0();
//		}
//		SCLK_1();
//		_cmd <<= 1;
//		SCLK_0();
//	}
//	
//	SYNC_1();
//}

/*
*********************************************************************************************************
*	函 数 名: DAC8562_SetData
*	功能说明: 设置DAC输出，并立即更新。
*	形    参: _ch, 通道, 0 , 1
*		     _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void DAC8562_SetData(uint8_t _ch, uint16_t _dac,uint16_t _dac1)
{
	LDAC_1();

	/* Write to DAC-A input register and update DAC-A; */
	//DAC8562_WriteCmd((3 << 19) | (0 << 16) | (_dac << 0));
	SYNC_0();
	dac8562_WriteReadByte(0x00);
	dac8562_WriteReadByte(_dac>>8);
	dac8562_WriteReadByte(_dac&0xff);
	SYNC_1();

	/* Write to DAC-B input register and update DAC-B; */
	//DAC8562_WriteCmd((3 << 19) | (1 << 16) | (_dac << 0));
	SYNC_0();
	dac8562_WriteReadByte(0x01);
	dac8562_WriteReadByte(_dac1>>8);
	dac8562_WriteReadByte(_dac1&0xff);
	SYNC_1();		
	LDAC_0();
	
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
