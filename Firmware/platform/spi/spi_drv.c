#include "common.h"
#include "stm32f4xx_gpio.h"
#include "spi_drv.h"
#include "uart_drv.h"

/*----------------------------------------------------------------------------*/

void SPI1_drv_Init(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	GPIO_InitStructure.GPIO_Pin = SPI1_CLK_PIN|SPI1_MISO_PIN|SPI1_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

	GPIO_PinAFConfig(SPI1_GPIO,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_GPIO,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_GPIO,GPIO_PinSource7,GPIO_AF_SPI1);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;	
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);

	SPI_ReadWriteByte(SPI1,0xff);		 
}   


void SPI2_drv_Init(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	GPIO_InitStructure.GPIO_Pin = SPI2_CLK_PIN|SPI2_MISO_PIN|SPI2_MOSI_PIN;;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

	GPIO_PinAFConfig(SPI2_GPIO,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI2_GPIO,GPIO_PinSource15,GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI2_GPIO,GPIO_PinSource13,GPIO_AF_SPI2);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;	
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_Cmd(SPI2, ENABLE);

	SPI_ReadWriteByte(SPI2,0xff);		 
}   

void SPI3_drv_Init(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	GPIO_InitStructure.GPIO_Pin = SPI3_MISO_PIN|SPI3_MOSI_PIN;;;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPI3_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI3_CLK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SPI3_GPIO, &GPIO_InitStructure);

	GPIO_PinAFConfig(SPI3_GPIO,GPIO_PinSource10,GPIO_AF_SPI3);
	GPIO_PinAFConfig(SPI3_GPIO,GPIO_PinSource11,GPIO_AF_SPI3);
	GPIO_PinAFConfig(SPI3_GPIO,GPIO_PinSource12,GPIO_AF_SPI3);

	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, ENABLE);

	SPI_Cmd(SPI3, ENABLE);

	SPI_ReadWriteByte(SPI3,0xff);		 
}   

u8 SPI_ReadWriteByte(SPI_TypeDef* SPIx,u8 TxData)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}
	
	SPI_I2S_SendData(SPIx, TxData);
		
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){}
 
	return SPI_I2S_ReceiveData(SPIx);
 		    
}





