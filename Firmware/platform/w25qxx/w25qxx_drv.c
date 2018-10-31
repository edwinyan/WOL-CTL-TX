#include "common.h"
#include "stm32f4xx_gpio.h"
#include "w25qxx_drv.h"
#include "spi_drv.h"
#include "uart_drv.h"
#include "bsp_os.h"

//u16 W25QXX_TYPE=W25Q128;	

/*----------------------------------------------------------------------------*/
void W25QXX_Init(void)
{ 
  GPIO_InitTypeDef  GPIO_InitStructure;
 
	//GPIOB3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	W25QXX_CS_OFF;	
	SPI3_drv_Init();	
	BSP_OS_TimeDly(10);
}  

u8 W25QXX_ReadSR(void)   
{  
	u8 byte=0;   
	W25QXX_CS_ON;                            
	SPI_ReadWriteByte(SPI3,W25X_ReadStatusReg);     
	byte=SPI_ReadWriteByte(SPI3,0Xff);          
	W25QXX_CS_OFF;                          
	return byte;   
} 

void W25QXX_Write_SR(u8 sr)   
{   
	W25QXX_CS_ON;                              
	SPI_ReadWriteByte(SPI3,W25X_WriteStatusReg);   
	SPI_ReadWriteByte(SPI3,sr);               
	W25QXX_CS_OFF;                            	      
}   

void W25QXX_Write_Enable(void)   
{
	W25QXX_CS_ON;                             
    SPI_ReadWriteByte(SPI3,W25X_WriteEnable);     
	W25QXX_CS_OFF;                              	      
} 

void W25QXX_Write_Disable(void)   
{  
	W25QXX_CS_ON;                        
    SPI_ReadWriteByte(SPI3,W25X_WriteDisable);   
	W25QXX_CS_OFF;	      
} 		

u16 W25QXX_ReadID(void)
{
	u16 Temp = 0;	  
	W25QXX_CS_ON;				    
	SPI_ReadWriteByte(SPI3,0x90);  
	SPI_ReadWriteByte(SPI3,0x00); 	    
	SPI_ReadWriteByte(SPI3,0x00); 	    
	SPI_ReadWriteByte(SPI3,0x00); 	 			   
	Temp|=SPI_ReadWriteByte(SPI3,0xFF)<<8;  
	Temp|=SPI_ReadWriteByte(SPI3,0xFF);	 
	W25QXX_CS_OFF;				    
	return Temp;
}   		    

void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;
	W25QXX_CS_ON;                            
    SPI_ReadWriteByte(SPI3,W25X_ReadData);        
    SPI_ReadWriteByte(SPI3,(u8)((ReadAddr)>>16));    
    SPI_ReadWriteByte(SPI3,(u8)((ReadAddr)>>8));   
    SPI_ReadWriteByte(SPI3,(u8)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI_ReadWriteByte(SPI3,0XFF);
    }
	W25QXX_CS_OFF; 
}  

void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
    W25QXX_Write_Enable();                 
	W25QXX_CS_ON;                              
    SPI_ReadWriteByte(SPI3,W25X_PageProgram);      
    SPI_ReadWriteByte(SPI3,(u8)((WriteAddr)>>16));   
    SPI_ReadWriteByte(SPI3,(u8)((WriteAddr)>>8));   
    SPI_ReadWriteByte(SPI3,(u8)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)
	{
		SPI_ReadWriteByte(SPI3,pBuffer[i]);
    }
	W25QXX_CS_OFF;                             
	W25QXX_Wait_Busy();	
} 

void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256;	 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			 
			if(NumByteToWrite>256)pageremain=256; 
			else pageremain=NumByteToWrite;
		}
	} ;
} 

u8 W25QXX_BUFFER[4096];		 
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;  
	secoff=WriteAddr%4096;
	secremain=4096-secoff;
 	//MSG("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);
		for(i=0;i<secremain;i++)
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;  
		}
		if(i<secremain)
		{
			W25QXX_Erase_Sector(secpos);
			for(i=0;i<secremain;i++)	  
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);

		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);				   
		if(NumByteToWrite==secremain)break;
		else
		{
			secpos++;
			secoff=0; 

		   	pBuffer+=secremain; 
			WriteAddr+=secremain;
		   	NumByteToWrite-=secremain;				
			if(NumByteToWrite>4096)secremain=4096;	
			else secremain=NumByteToWrite;
		}	 
	};
}

void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();            
    W25QXX_Wait_Busy();   
  	W25QXX_CS_ON;                       
    SPI_ReadWriteByte(SPI3,W25X_ChipErase);        
	W25QXX_CS_OFF;                             	      
	W25QXX_Wait_Busy();   				
}   

void W25QXX_Erase_Sector(u32 Dst_Addr)   
{   
 	Dst_Addr*=4096;
    W25QXX_Write_Enable(); 
    W25QXX_Wait_Busy(); 
  	W25QXX_CS_ON;                            
    SPI_ReadWriteByte(SPI3,W25X_SectorErase);     
    SPI_ReadWriteByte(SPI3,(u8)((Dst_Addr)>>16));   
    SPI_ReadWriteByte(SPI3,(u8)((Dst_Addr)>>8));   
    SPI_ReadWriteByte(SPI3,(u8)Dst_Addr); 
	W25QXX_CS_OFF;                           	      
    W25QXX_Wait_Busy();
}  

void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);  
}  

void W25QXX_PowerDown(void)   
{ 
  	W25QXX_CS_ON;                          
    SPI_ReadWriteByte(SPI3,W25X_PowerDown);       
	W25QXX_CS_OFF;                           	      
    BSP_OS_TimeDly(1);                           
}   

void W25QXX_WAKEUP(void)   
{  
  	W25QXX_CS_ON;                             
    SPI_ReadWriteByte(SPI3,W25X_ReleasePowerDown); 
	W25QXX_CS_OFF;                           	      
    BSP_OS_TimeDly(1);                            
}   

/*----------------------------------------------------------------------------*/



