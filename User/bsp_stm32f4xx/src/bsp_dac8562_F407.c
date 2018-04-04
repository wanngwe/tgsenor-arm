/*
*********************************************************************************************************
*
*	ģ������ : DAC8562/8563 ����ģ��(��ͨ����16λDAC)
*	�ļ����� : bsp_dac8562.c
*	��    �� : V1.0
*	˵    �� : DAC8562/8563ģ���CPU֮�����SPI�ӿڡ�����������֧��Ӳ��SPI�ӿں����SPI�ӿڡ�
*			  ͨ�����л���
*
*	�޸ļ�¼ :
*		�汾��  ����         ����     ˵��
*		V1.0    2014-01-17  armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

//#define SOFT_SPI		/* ������б�ʾʹ��GPIOģ��SPI�ӿ� */
#define HARD_SPI		/* ������б�ʾʹ��CPU��Ӳ��SPI�ӿ� */

/*
	DAC8501ģ�����ֱ�Ӳ嵽STM32-V5������CN19��ĸ(2*4P 2.54mm)�ӿ���

    DAC8562/8563ģ��    STM32F407������
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
	DAC8562��������:
	1������2.7 - 5V;  ������ʹ��3.3V��
	4���ο���ѹ2.5V��ʹ���ڲ��ο�

	��SPI��ʱ���ٶ�Ҫ��: �ߴ�50MHz�� �ٶȺܿ�.
	SCLK�½��ض�ȡ����, ÿ�δ���24bit���ݣ� ��λ�ȴ�
*/

#if !defined(SOFT_SPI) && !defined(HARD_SPI)
 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
#endif

#ifdef SOFT_SPI		/* ���SPI */
	/* ����GPIO�˿� */
	#define RCC_SCLK 	RCC_AHB1Periph_GPIOB
	#define PORT_SCLK	GPIOB
	#define PIN_SCLK	GPIO_Pin_3
	
	#define RCC_DIN 	RCC_AHB1Periph_GPIOB
	#define PORT_DIN 	GPIOB
	#define PIN_DIN 	GPIO_Pin_5
	
	/* Ƭѡ */
	#define RCC_SYNC 	RCC_AHB1Periph_GPIOF
	#define PORT_SYNC	GPIOF
	#define PIN_SYNC	GPIO_Pin_7

	/* LDAC, ���Բ��� */
	#define RCC_LDAC	RCC_AHB1Periph_GPIOA
	#define PORT_LDAC	GPIOA
	#define PIN_LDAC	GPIO_Pin_4
	
	/* CLR, ���Բ��� */
	#define RCC_CLR 	RCC_AHB1Periph_GPIOF
	#define PORT_CLR	GPIOF
	#define PIN_CLR 	GPIO_Pin_7

	/* ���������0����1�ĺ� */
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

#ifdef HARD_SPI		/* Ӳ��SPI (δ��) */

	/* ����GPIO�˿� */
	#define RCC_SCLK 	RCC_AHB1Periph_GPIOB
	#define PORT_SCLK	GPIOB
	#define PIN_SCLK	GPIO_Pin_3
	
	#define RCC_DIN 	RCC_AHB1Periph_GPIOB
	#define PORT_DIN 	GPIOB
	#define PIN_DIN 	GPIO_Pin_5
	
	/* Ƭѡ */
	#define RCC_SYNC 	RCC_AHB1Periph_GPIOF
	#define PORT_SYNC	GPIOF
	#define PIN_SYNC	GPIO_Pin_7

	/* LDAC, ���Բ��� */
	#define RCC_LDAC	RCC_AHB1Periph_GPIOA
	#define PORT_LDAC	GPIOA
	#define PIN_LDAC	GPIO_Pin_4
	
	/* CLR, ���Բ��� */
	#define RCC_CLR 	RCC_AHB1Periph_GPIOF
	#define PORT_CLR	GPIOF
	#define PIN_CLR 	GPIO_Pin_7

	/* ���������0����1�ĺ� */
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
*	�� �� ��: nRF24L01_CfgSpiHard
*	����˵��: ����STM32�ڲ�SPIӲ���Ĺ���ģʽ���ٶȵȲ��������ڷ���nRF24L01��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void dac8562_CfgSpiHard(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	
	/*
	STM32F4XX ʱ�Ӽ���.
		HCLK = 168M
		PCLK1 = HCLK / 4 = 42M
		PCLK2 = HCLK / 2 = 84M

		SPI2��SPI3 �� PCLK1, ʱ��42M
		SPI1       �� PCLK2, ʱ��84M

		STM32F4 ֧�ֵ����SPIʱ��Ϊ 37.5 Mbits/S, �����Ҫ��Ƶ������ʹ�õ���SPI1��
	*/
	/* ����SPIӲ������ */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	/* ���ݷ���2��ȫ˫�� */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		/* STM32��SPI����ģʽ ������ģʽ */
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	/* ����λ���� �� 8λ */
	/* SPI_CPOL��SPI_CPHA���ʹ�þ���ʱ�Ӻ����ݲ��������λ��ϵ��
	   ��������: ���߿����ǵ͵�ƽ,��1�����ز������ݡ�
	*/
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			/* Ƭѡ���Ʒ�ʽ��������� */

	/* ���ò�����Ԥ��Ƶϵ�� 84MHz / 16 = 5.25MHz��24L01֧�ֵ����SPIʱ��Ϊ10MHz
	   ����ѡ��SPI_BaudRatePrescaler_8���׳���
	*/
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	/* ����λ������򣺸�λ�ȴ� */
	SPI_InitStructure.SPI_CRCPolynomial = 7;			/* CRC����ʽ�Ĵ�������λ��Ϊ7�������̲��� */
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);				/* ʹ��SPI  */
}
/*
*********************************************************************************************************
*	�� �� ��: nRF24L01_WriteReadByte
*	����˵��: ������NRF��/дһ�ֽ�����
*	��    ��: д�������
*	�� �� ֵ: ��ȡ�õ�����		        
*********************************************************************************************************
*/
static uint8_t dac8562_WriteReadByte(uint8_t dat)
{  	
	/* ��SPI���ͻ������ǿ�ʱ�ȴ� */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ͨ��SPI����һ�ֽ����� */
	SPI_I2S_SendData(SPI1, dat);		
	
	/* ��SPI���ջ�����Ϊ��ʱ�ȴ� */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* ͨ��SPI����һ�ֽ����� */
	return SPI_I2S_ReceiveData(SPI1);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitDAC8562
*	����˵��: ����STM32��GPIO��SPI�ӿڣ��������� DAC8562
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitDAC8562(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef SOFT_SPI
	SYNC_1();	/* SYNC = 1 */

	/* ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_SCLK | RCC_DIN | RCC_SYNC, ENABLE);

	/* ���ü����������IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

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

/* SPI�ӿڵ�SCK MOSI MISO�˿ڶ��� */
#define DAC8562_PORT_SPI	GPIOB
#define nRF24L01_PIN_SCK	GPIO_Pin_3
#define nRF24L01_PIN_MISO	GPIO_Pin_4
#define nRF24L01_PIN_MOSI	GPIO_Pin_5

#define DAC8562_PinSource_SCK	 GPIO_PinSource3
#define DAC8562_PinSource_MISO	 GPIO_PinSource4
#define DAC8562_PinSource_MOSI	 GPIO_PinSource5

#define DAC8562_AF_SPI    GPIO_AF_SPI1
	

	/* ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_SCLK | RCC_DIN | RCC_SYNC, ENABLE);

	/* ���ü����������IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

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
	/* ���� SCK, MISO , MOSI Ϊ���ù��� */
	/* ʹ��SPI1ʱ�� */
	RCC_APB2PeriphClockCmd(dac8652_RCC_SPI, ENABLE);
	GPIO_PinAFConfig(DAC8562_PORT_SPI, DAC8562_PinSource_SCK,  DAC8562_AF_SPI);
	GPIO_PinAFConfig(DAC8562_PORT_SPI, DAC8562_PinSource_MISO, DAC8562_AF_SPI);
	GPIO_PinAFConfig(DAC8562_PORT_SPI, DAC8562_PinSource_MOSI, DAC8562_AF_SPI);

	/* ע��GPIO�ٶ���ò��ó���25MHz��Ҫ�����׳��� */
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
	
	/* LDAC pin inactive for DAC-B and DAC-A  ��ʹ��LDAC���Ÿ������� */
	//DAC8562_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));
	
	SYNC_0();
	dac8562_WriteReadByte(0x30);
	dac8562_WriteReadByte(0);
	dac8562_WriteReadByte(0x0);
	SYNC_1();

	/* ��λ2��DAC���м�ֵ, ���2.5V */
	DAC8562_SetData(0, 32767,32767);
	DAC8562_SetData(1, 32767,32767);

	/* ѡ���ڲ��ο�����λ2��DAC������=2 ����λʱ���ڲ��ο��ǽ�ֹ��) */
	//DAC8562_WriteCmd((7 << 19) | (0 << 16) | (1 << 0));
	SYNC_0();
	dac8562_WriteReadByte(0x38);
	dac8562_WriteReadByte(0);
	dac8562_WriteReadByte(1);
	SYNC_1();	
}


/*********************************************************************************************************
*	�� �� ��: DAC8562_WriteCmd
*	����˵��: ��SPI���߷���24��bit���ݡ�
*	��    ��: _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//void DAC8562_WriteCmd(uint32_t _cmd)
//{
//	uint8_t i;
//	
//	SYNC_0();
//	
//	/*��DAC8562 SCLKʱ�Ӹߴ�50M����˿��Բ��ӳ� */
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
*	�� �� ��: DAC8562_SetData
*	����˵��: ����DAC��������������¡�
*	��    ��: _ch, ͨ��, 0 , 1
*		     _data : ����
*	�� �� ֵ: ��
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
