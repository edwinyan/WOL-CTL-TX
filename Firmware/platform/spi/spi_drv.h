#ifndef _SPI_DRV_H_
#define _SPI_DRV_H_

#define SPI1_GPIO	GPIOA
#define SPI1_CLK_PIN	GPIO_Pin_5
#define SPI1_MISO_PIN	GPIO_Pin_6
#define SPI1_MOSI_PIN	GPIO_Pin_7


#define SPI2_GPIO	GPIOB
#define SPI2_CLK_PIN	GPIO_Pin_13
#define SPI2_MISO_PIN	GPIO_Pin_14
#define SPI2_MOSI_PIN	GPIO_Pin_15


#define SPI3_GPIO	GPIOC
#define SPI3_CLK_PIN	GPIO_Pin_10
#define SPI3_MISO_PIN	GPIO_Pin_11
#define SPI3_MOSI_PIN	GPIO_Pin_12





void SPI1_drv_Init(void);
void SPI2_drv_Init(void);
void SPI3_drv_Init(void);


u8 SPI_ReadWriteByte(SPI_TypeDef* SPIx,u8 TxData);

#endif
