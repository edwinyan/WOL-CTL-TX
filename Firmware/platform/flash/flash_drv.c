#include "common.h"
#include "stm32f4xx_flash.h"
#include "flash_drv.h"

/*----------------------------------------------------------------------------*/

u32 FLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
} 

u16 FLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
/*从指定地址开始写入指定长度的数据
  特别注意：因为STM32F4的扇区实在太大，没办法本地保存扇区数据，所以本函数写
  地址如果非0XFF，那么会先擦除整个扇区且不保存扇区数据，所以写非0XFF的地址，
  将导致整个扇区数据丢失，建议写之前确保扇区里的没有重要数据，最好是整个扇区先擦除，
  然后慢慢往后写

  该函数对OTP区域也有效，可以用来写OTP区
  OTP区域地址范围:0X1FFF7800 - 0X1FFF7A0F
  writeaddr:起始地址，必须为4的倍数
  pbuffer:数据指针
  numtowrite:字数
*/
void FLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  	FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;
	FLASH_Unlock();		
  	FLASH_DataCacheCmd(DISABLE);
 		
	addrx=WriteAddr;				
	endaddr=WriteAddr+NumToWrite*4;
	if(addrx<0X1FFF0000)			
	{
		while(addrx<endaddr)		
		{
			if(FLASH_ReadWord(addrx)!=0XFFFFFFFF)
			{   
				status=FLASH_EraseSector(FLASH_GetFlashSector(addrx),VoltageRange_3);
				if(status!=FLASH_COMPLETE)break;
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE){
				break;	
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  	FLASH_DataCacheCmd(ENABLE);	
	FLASH_Lock();
} 

void FLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=FLASH_ReadWord(ReadAddr);
		ReadAddr+=4;
	}
}

