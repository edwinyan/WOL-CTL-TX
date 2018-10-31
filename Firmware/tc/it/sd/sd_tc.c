#include "common.h"
#include "bsp.h"
#include "bsp_os.h"
#include "tc_common.h"
#include "sd_drv.h"
#include "sd_common.h"
#include "sd_rw.h"
#include "sd_tc.h"
#include "led_drv.h"
#include "stm32f4xx_rng.h"
#include "sd_config.h"

#define TEST_HEAD          (0xAA55)
#define TEST_SIZE          (0x14000)   //80k
#define ADDR_COUNT         (1024)
#define INNER_COUNT        (80)
#define EXTERNAL_COUNT     (50)
#define MULTI_SECTOR_SIZE  (40)
//#define FILL_SECTOR_SIZE   (80)
#define LBA_SIZE           (5*1024)    //5k 
#define TEMP_BUFF_SIZE     (10*1024)    //20k 
//#define FILL_DATA_SIZE     (40*1024)    //40k 
uint8_t buffer_block_tx[1024], buffer_block_rx[buffer_words_size];
uint8_t buffer_multiblock_tx[buffer_multiwords_size], buffer_multiblock_rx[buffer_multiwords_size];
uint8_t test_buff[TEMP_BUFF_SIZE],test_data_buf[2048],test_read_buf[2048],temp[TEMP_BUFF_SIZE];
//uint8_t fill_data_buff[FILL_DATA_SIZE];
uint8_t test_data_buff[512];
//uint8_t compare_data_buff[TEST_SIZE];
lba_pattern_t lba_buff[LBA_SIZE];
//-----------------Test functions------------------------------------

/*
 * 函数名：Buffercmp
 * 描述  ：比较两个缓冲区中的数据是否相等
 * 输入  ：-pBuffer1, -pBuffer2 : 要比较的缓冲区的指针
 *         -BufferLength 缓冲区长度
 * 输出  ：-PASSED 相等
 *         -FAILED 不等
 */
uint8_t buffer_cmp(uint8_t* pbuffer1, uint8_t* pbuffer2, uint16_t bufferlength)
{
    while (bufferlength--)
    {
        if (*pbuffer1 != *pbuffer2)
        {
              return 0;
        }

        pbuffer1++;
        pbuffer2++;
    }

    return 1;
}


/*
 * 函数名：Fill_Buffer
 * 描述  ：在缓冲区中填写数据
 * 输入  ：-pBuffer 要填充的缓冲区
 *         -BufferLength 要填充的大小
 *         -Offset 填在缓冲区的第一个值
 * 输出  ：无 
 */
void fill_buffer(uint8_t *pbuffer, uint16_t bufferlenght, uint8_t Offset)
{
    uint16_t index = 0;

    /* Put in global buffer same values */
    for (index = 0; index < bufferlenght; index++ )
    {

        pbuffer[index] = Offset;

    }
}

/*预填充值函数*/
void fill_all_lba(u16 loop_count)
{
    OS_ERR  err;
    u32 sector_addr,count;
    u16 print_count=1;
    u8 test_status;
    for(sector_addr=0x00;sector_addr<MAX_ADDR;sector_addr=sector_addr+MULTI_SECTOR_SIZE)
    {
        fill_buffer(test_buff, TEMP_BUFF_SIZE, loop_count);
        
        test_status=sd_drv_wr(test_buff,sector_addr,MULTI_SECTOR_SIZE);
       
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        }
        
        if ((count+1)%1000==0)
        {
            MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
        count++;
       
    }
}


//-------------------------------------------------------------------

//init/deinit
STATIC void sd_tc_01(void)
{
    DSTATUS Dstatus;
    tc_printf_banner("sd_tc_01 test ");
    Dstatus= sd_drv_initialize();
    if(Dstatus==0)
    {
        MSG(SYS_INFO "sd init ok\r\n");
    }
    else
    {
        MSG(SYS_INFO "sd init failed\r\n");
    }
    
    tc_printf_tail("sd_tc_01 test ");
}

//single sector + sequential write/read/compare
STATIC void sd_tc_02(void)
{
    OS_ERR  err;
    u32 sector_addr=0x0001,print_count=1;
    u8 test_status,data=0x01;
    u16 count=0;
    tc_printf_banner("sd_tc_02 test ");
    LED_R_ON;
    for(sector_addr=0x00;sector_addr<=0x1000;sector_addr++)
    {
        fill_buffer(buffer_block_tx,buffer_words_size,data);
        
        test_status=sd_drv_wr(buffer_block_tx,sector_addr,1);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        }
        if ((count+1)%1000==0)
        {
            MSG(SYS_INFO "sd compare count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
        count++;
        data=data+2;
    }
    count=0;
    print_count=1;
    for(sector_addr=0x00;sector_addr<=0x1000;sector_addr++)
    {
        test_status=sd_drv_rd(buffer_block_rx,sector_addr,1);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        }

        //test_status=buffer_cmp(buffer_block_rx,buffer_block_tx,buffer_words_size);
        //if(test_status!=1)
        //{
        //    MSG(SYS_INFO "sd read/write failed_count--%d\r\n",count); 
        //}

        if ((count+1)%1000==0)
        {
            MSG(SYS_INFO "sd compare count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
        
        count++;
       
    }
    LED_R_OFF;
    tc_printf_tail("sd_tc_02 test ");
}

//multiple sectors + sequential write/read/compare
STATIC void sd_tc_03(void)
{
    OS_ERR  err;
    u32 sector_addr=0x00000001, count=1;
    u8 test_status,data=0xFF;
    u16 print_count=1;
    tc_printf_banner("sd_tc_03 test ");
    LED_R_ON;
    for(sector_addr=0x1500;sector_addr<=0x35000;sector_addr=sector_addr+10)
    {
        fill_buffer(test_buff,TEMP_BUFF_SIZE,data);
        
        test_status=sd_drv_wr(test_buff,sector_addr,10);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        }

        test_status=sd_drv_rd(temp,sector_addr,10);//多块读最大值为189，超出即出错
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        }

        if(test_buff[0] != temp[0])
       // test_status=buffer_cmp(buffer_block_rx,buffer_block_tx,buffer_words_size);
       // if(test_status!=1)
        {
            MSG(SYS_INFO "sd read/write failed_count--%d\r\n",count); 

        }

        if (count%1024==0)
        {
            MSG(SYS_INFO "sd compare count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        } 
        
        data=data+2;
        count++;
        
    }
    LED_R_OFF;
    tc_printf_tail("sd_tc_03 test ");
}

//single sector + random write/read/compare
STATIC void sd_tc_04(void)
{
    OS_ERR  err;
    u8 test_status,data=0x00;
    u32 addr,count,print_count=1;
    tc_printf_banner("sd_tc_04 test ");
    LED_R_ON;
    for(count=0;count<0x2000;count++)
    {        
        srand(count);
        addr=(rand())%MAX_ADDR;    //SD卡最大地址为3AF000
        
        srand(count);
        data=rand();
        
        fill_buffer(test_data_buf,512,data);
        test_status=sd_drv_wr(test_data_buf,addr,1);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        } 
        
        if ((count+1)%1024==0)
        {
            MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
    }
    LED_R_OFF;

    print_count=1;
    LED_R_ON;
    for(count=0;count<0x2000;count++)
    {
        srand(count);
        addr=(rand())%MAX_ADDR;    
        test_status=sd_drv_rd(test_read_buf,addr,1);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        }        
        srand(count);
        data=rand();
        #if 1        
        if(data !=test_read_buf[0])
        {
            MSG(SYS_INFO "sd compare failed--%d\r\n",count); 
        }
       #endif
        if ((count+1)%1024==0)
        {
            MSG(SYS_INFO "sd compare count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }    
    }
    LED_R_OFF;
    
    tc_printf_tail("sd_tc_04 test ");
}



//multiple sectors + random write/read/compare
STATIC void sd_tc_05(void)
{
    OS_ERR  err;
    u8 test_status,data;
    u32 addr,count,print_count=1;
    tc_printf_banner("sd_tc_05 test ");
    LED_R_ON;
    for(count=0x0001;count<0x5000;count++)
    {        
        srand(count);
        addr=(rand())%MAX_ADDR;
        //MSG(SYS_INFO "addr is %d\r\n",addr);
        srand(count);
        data=rand();
        //MSG(SYS_INFO "data is %d\r\n",data);
        fill_buffer(test_data_buf,1024,data);
        //fill_buffer(test_buff,TEMP_BUFF_SIZE,data);
        test_status=sd_drv_wr(test_data_buf,addr,4);
        //test_status=sd_drv_wr(test_buff,addr,MULTI_SECTOR_SIZE);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        } 

        if ((count+1)%1024==0)
        {
            MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
    }
    LED_R_OFF;

    print_count=1;
    LED_R_ON;
    for(count=0x0001;count<0x5000;count++)
    {
        srand(count);
        addr=(rand())%MAX_ADDR;
        //MSG(SYS_INFO "addr is %d\r\n",addr);
        //test_status=sd_drv_rd(temp,addr,MULTI_SECTOR_SIZE);
        test_status=sd_drv_rd(test_read_buf,addr,4);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        } 

        srand(count);
        data=rand();
        #if 1        
        if(data !=test_read_buf[0])
        {
            MSG(SYS_INFO "sd compare failed--%d\r\n",count); 
        }
       #endif
        if ((count+1)%1024==0)
        {
            MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }    
    }
    LED_R_OFF;
    
    tc_printf_tail("sd_tc_05 test ");
}

//write protect
STATIC void sd_tc_06(void)
{
    tc_printf_banner("sd_tc_06 test ");
    tc_printf_tail("sd_tc_06 test ");
}

//erase
STATIC void sd_tc_07(void)
{
    //OS_ERR  err;
    u8 test_status;
    u16 lba_val,i,count;
    u32 lba;
    tc_printf_banner("sd_tc_07 test ");
    for(count=0;count<100;count++)
    {
        srand(OSTimeGet(0));
        lba=rand()%MAX_ADDR;         //lba 为32bit数据,最大地址范围不超过MAX_ADDR
        //lba = addr;    

        srand(OSTimeGet(0));
        lba_val=rand()%0xFF;         //lba_val 为8bit数据

        lba_buff[count].lba= lba;
        lba_buff[count].lba_val= lba_val;
        MSG(SYS_INFO "lba_buffer--%x & %x\r\n",lba_buff[count].lba,lba_buff[count].lba_val);
        
        fill_buffer(test_buff,TEMP_BUFF_SIZE-6,lba_val);
        temp[0]=(lba & 0xff000000)>>24;
        temp[1]=(lba & 0x00ff0000)>>16;
        temp[2]=(lba & 0x0000ff00)>>8;
        temp[3]=(lba & 0x000000ff);

        for(i=0;i<TEMP_BUFF_SIZE-6;i++)
        {
            temp[i+4] = test_buff[i];
        }
        temp[TEMP_BUFF_SIZE-2]=0xff;
        temp[TEMP_BUFF_SIZE-1]=0xff;
        test_status=sd_drv_wr(temp,lba,40);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        }
    }
    for(count=0;count<100;count++)
    {
        lba = lba_buff[count].lba;
        test_status=sd_drv_rd(temp,lba,40);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        }
        if((temp[0]<<24) & (temp[1]<<16) & (temp[2]<<8) & temp[3] != lba)
        MSG(SYS_INFO "sd compare failed--1\r\n");
        if(lba_buff[count].lba_val !=temp[4])
        MSG(SYS_INFO "sd compare failed--2\r\n");
    }
    
    sd_reset();

    for(count=0;count<100;count++)
    {
        lba = lba_buff[count].lba;
        test_status=sd_drv_rd(temp,lba,40);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        }
        if((temp[0]<<24) & (temp[1]<<16) & (temp[2]<<8) & temp[3] != lba)
        MSG(SYS_INFO "sd compare failed--1\r\n");
        if(lba_buff[count].lba_val !=temp[4])
        MSG(SYS_INFO "sd compare failed--2\r\n");
    }

    
    tc_printf_tail("sd_tc_07 test ");
}
#if 0
STATIC void sd_tc_08(void)
{
    OS_ERR  err;
    u16 inner_count,addr_count,i,print_count=1; 
    u8 data=0x01,test_status,external_count=0,count=1;
    u32 addr,temp_size;
    tc_printf_banner("sd_tc_08 test ");
    for(external_count=0;external_count<EXTERNAL_COUNT;external_count++)
    {
        for(inner_count=0;inner_count<INNER_COUNT;inner_count++)
        {
            
            //for(addr_count=0;addr_count<500;addr_count++)
            for(addr_count=0;addr_count<ADDR_COUNT;addr_count=addr_count+2)
            {
                fill_buffer(test_data_buf,buffer_words_size-10,data);
                buffer_block_tx[0]=(TEST_HEAD&0xff00)>>8;
                buffer_block_tx[1]=TEST_HEAD&0x00ff;
                buffer_block_tx[2]=(external_count&0xff00)>>8;
                buffer_block_tx[3]=(external_count&0x00ff);
                buffer_block_tx[4]=(inner_count&0xff00)>>8;
                buffer_block_tx[5]=(inner_count&0x00ff);
                buffer_block_tx[6]=(addr_count&0xff00)>>8;
                buffer_block_tx[7]=(addr_count&0x00ff);
                for(i=0;i<buffer_words_size-10;i++)
                {
                    buffer_block_tx[i+8]=test_data_buf[i];
                }
                buffer_block_tx[buffer_words_size-2]=0xFF;
                buffer_block_tx[buffer_words_size-1]=0xFF;
                
                temp_size=addr_count+ADDR_COUNT*inner_count;
                compare_data_buff[temp_size]=test_data_buf[0];
                //for(i=0;i<10;i++)
                //{
                //    MSG(SYS_INFO "buffer_block_tx is %x\r\n",buffer_block_tx[i]);
                //}
                addr=addr_count+ADDR_COUNT*inner_count+ADDR_COUNT*INNER_COUNT*external_count;
                test_status=sd_drv_wr(buffer_block_tx,addr,2);
                if(test_status!=0)
                {
                    MSG(SYS_INFO "sd write failed\r\n");
                }
                data++;
                if ((addr+1)%100==0)
                {
                    MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
                    print_count++;
                }
            }
        }
        #if 1

        print_count=0;
        for(inner_count=0;inner_count<INNER_COUNT;inner_count++)
        {
            for(addr_count=0;addr_count<ADDR_COUNT;addr_count=addr_count+2)
            {
                temp_size=addr_count+ADDR_COUNT*inner_count;
                addr=addr_count+ADDR_COUNT*inner_count+ADDR_COUNT*INNER_COUNT*external_count;
                test_status=sd_drv_rd(buffer_block_rx,addr,2);
                if(test_status!=0)
                {
                    MSG(SYS_INFO "sd read failed\r\n");
                }
                #if 1
                //data decode
                if(buffer_block_rx[0] != 0xAA && buffer_block_rx[1] != 0x55)
                {
                    MSG(SYS_INFO "sd compare failed--1\r\n");
                }
                if(((buffer_block_rx[2]<<8) + (buffer_block_rx[3])) != external_count)
                {
                    MSG(SYS_INFO "sd compare failed--2\r\n");
                }
                if(((buffer_block_rx[4]<<8) + (buffer_block_rx[5])) != inner_count)
                {
                    MSG(SYS_INFO "sd compare failed--3\r\n");
                }
                if(((buffer_block_rx[6]<<8) + (buffer_block_rx[7])) != addr_count)
                {
                    MSG(SYS_INFO "sd compare failed--4\r\n");
                }
                if(buffer_block_rx[8] != compare_data_buff[temp_size])
                {
                    MSG(SYS_INFO "sd compare failed--5\r\n");
                }
                #endif
                if ((addr+1)%100==0)
                {
                    MSG(SYS_INFO "sd compare count--%d @ %d\r\n",print_count,OSTimeGet(&err));
                    print_count++;
                }            
            }
                            
        }
        #endif
        MSG(SYS_INFO "complete count--%d @ %d\r\n",count,OSTimeGet(&err));
        count++;
        print_count=0;
    }
    tc_printf_tail("sd_tc_08 test ");
}

#endif
STATIC void sd_tc_09(void)
{
    OS_ERR  err;
    u32 sector_addr=0x0001,print_count=1;
    u8 test_status,data=0x01;
    u16 count=0;
    tc_printf_banner("sd_tc_09 test ");
    LED_R_ON;
    fill_buffer(buffer_block_tx,buffer_words_size,data);
    for(sector_addr=0x00;sector_addr<=0x4000;sector_addr++)
    {
        test_status=sd_drv_wr(buffer_block_tx,sector_addr,1);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        }
        if ((count+1)%1000==0)
        {
            MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
        count++;
       
    }
    print_count=0;
    for(sector_addr=0x00;sector_addr<=0x4000;sector_addr++)
    {
        test_status=sd_drv_rd(buffer_block_rx,sector_addr,1);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        }

        if ((count+1)%1000==0)
        {
            MSG(SYS_INFO "sd read count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
        count++;
       
    }

    LED_R_OFF;
    tc_printf_tail("sd_tc_09 test ");
}


STATIC void sd_tc_10(void)
{
    OS_ERR  err;
    u32 sector_addr=0x0001,print_count=1;
    u8 test_status,data=0x01;
    u16 count=0;
    tc_printf_banner("sd_tc_10 test ");
    LED_R_ON;
    fill_buffer(buffer_block_tx,buffer_words_size,data);

    for(sector_addr=0x00;sector_addr<=0x100000;sector_addr=sector_addr+100)
    {
        
        test_status=sd_drv_wr(buffer_block_tx,sector_addr,100);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        }
        if ((count+1)%1000==0)
        {
            MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
        count++;
       
    }
    print_count=0;
    for(sector_addr=0x15000;sector_addr<=0x100000;sector_addr=sector_addr+100)
    {
        test_status=sd_drv_rd(buffer_block_rx,sector_addr,100);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        }

        if ((count+1)%1000==0)
        {
            MSG(SYS_INFO "sd read count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        }
        count++;
       
    }

    LED_R_OFF;
    tc_printf_tail("sd_tc_10 test ");
}

STATIC void sd_tc_11(void)
{
    tc_printf_banner("sd_tc_11 test ");
    sd_erase(0x00*512,MAX_ADDR*512);
    tc_printf_tail("sd_tc_11 test ");
}
#if 1
STATIC void sd_tc_12(void)
{
    OS_ERR  err;
    u16 loop_count,i,print_count=1;
    u8 lba_val,test_status;
    u32 count=0,lba,wr_cnt=0,addr;
    tc_printf_banner("sd_tc_12 test ");
    for(loop_count=0x00;loop_count<0x50;loop_count++)
    {
        //fill_all_lba(loop_count);//fill addr with offset data

        for(addr=0;addr<LBA_SIZE;addr++)
        {
            srand(OSTimeGet(0));
            lba=rand()%MAX_ADDR;         //lba 为32bit数据,最大地址范围不超过MAX_ADDR
            //lba = addr;    
            
            srand(OSTimeGet(0));
            lba_val=rand()%0xFF;         //lba_val 为8bit数据
            
            lba_buff[count].lba = lba;
            lba_buff[count].lba_val = lba_val;
            BSP_OS_TimeDly(1);  //deley 1ms
            count++;
        }
        
        for(i=0;i<20;i++)
        {
            MSG(SYS_INFO "lba_buffer--%x & %x\r\n",lba_buff[i].lba,lba_buff[i].lba_val);
        }
        #if 1
        count=0;
        for(addr=0;addr<LBA_SIZE;addr++)
        {
            //write data
            lba = lba_buff[count].lba;
            lba_val = lba_buff[count].lba_val;

            fill_buffer(test_buff,TEMP_BUFF_SIZE,lba_val);    
            test_status=sd_drv_wr(test_buff,lba,MULTI_SECTOR_SIZE);
            if(test_status!=0)
            {
                MSG(SYS_INFO "sd write failed\r\n");
            }
            if ((wr_cnt+1)%1000==0)
            {
                MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
                print_count++;
            }
            wr_cnt++;
            count++;
        }
        #endif
        wr_cnt=0;
        count=0;
        print_count=1;
        
        for(addr=0;addr<LBA_SIZE;addr++)
        {
            lba = lba_buff[count].lba;
            lba_val = lba_buff[count].lba_val;

            test_status=sd_drv_rd(test_buff,lba,MULTI_SECTOR_SIZE);
            if(test_status!=0)
            {
                MSG(SYS_INFO "sd read failed\r\n");
            }
            if ((wr_cnt+1)%1000==0)
            {
                MSG(SYS_INFO "sd read count--%d @ %d\r\n",print_count,OSTimeGet(&err));
                print_count++;
            }
            wr_cnt++;    
            if(lba_val != test_buff[0])
            {
                MSG(SYS_INFO "sd read failed--1\r\n");
            }
        }
        
    }
    tc_printf_tail("sd_tc_12 test ");
}
#endif
//连续地址多块写
STATIC void sd_tc_13(void)
{
    OS_ERR  err;
    u32 sector_addr=0x01, count=1;
    u8 test_status,data=0x01;
    u16 print_count=1;
    tc_printf_banner("sd_tc_13 test ");
    LED_R_ON;
    //写数据
    MSG(SYS_INFO "start write data\r\n");
    for(sector_addr=0x10000;sector_addr<=0x50000;sector_addr=sector_addr+2)
    {
        fill_buffer(test_buff,TEMP_BUFF_SIZE,data);
        //lba_buff[sector_addr].lba = sector_addr;
        //lba_buff[sector_addr].lba_val = data;;
        test_status=sd_drv_wr(test_buff,sector_addr,2);
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd write failed\r\n");
        }

        if (count%1024==0)
        {
            MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        } 
        data=data+2;
        count++;

    }
    MSG(SYS_INFO "end write data\r\n");
    MSG(SYS_INFO "start read data\r\n");
    //读数据
    count=0;
    print_count=1;
    for(sector_addr=0x10000;sector_addr<=0x50000;sector_addr=sector_addr+2)
    {

        test_status=sd_drv_rd(temp,sector_addr,2);//多块读最大值为189，超出即出错
        if(test_status!=0)
        {
            MSG(SYS_INFO "sd read failed\r\n");
        }
        
        //if(lba_buff[sector_addr].lba_val != temp[0])
        //{
        //    MSG(SYS_INFO "sd read/write failed_count--%d\r\n",count); 

        //}

        if (count%1024==0)
        {
            MSG(SYS_INFO "sd read count--%d @ %d\r\n",print_count,OSTimeGet(&err));
            print_count++;
        } 
        count++;
        
    }
    MSG(SYS_INFO "end read data\r\n");
    LED_R_OFF;
    tc_printf_tail("sd_tc_13 test ");
}
//随机地址多块写
STATIC void sd_tc_14(void)
{
    OS_ERR  err;
    u32 wr_cnt=0x01, count=1,lba;
    u8 test_status,lba_val,i,loop_cnt=0;
    u16 print_count=1,buff_count=0;
    tc_printf_banner("sd_tc_14 test ");
    LED_R_ON;
    while(loop_cnt<250)
    {
        //产生5k个随机地址和数据
        for(wr_cnt=0x00;wr_cnt<LBA_SIZE;wr_cnt++)
        {
            srand(OSTimeGet(0));
            lba = rand()%MAX_ADDR;         //32 bit random addr

            srand(OSTimeGet(0));
            lba_val = rand()%0XFF;         // 8 bit random data

            lba_buff[buff_count].lba = lba;
            lba_buff[buff_count].lba_val = lba_val;
           // MSG(SYS_INFO "lba_buffer--%x & %x\r\n",lba_buff[i].lba,lba_buff[i].lba_val);
            BSP_OS_TimeDly(1);  //deley 1ms
            buff_count++;
        }
        for(i=0;i<20;i++)
        {
            MSG(SYS_INFO "lba_buffer--%x & %x\r\n",lba_buff[i].lba,lba_buff[i].lba_val);
        }
        //多块写数据
        //MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
        buff_count=0;
        for(wr_cnt=0x00;wr_cnt<LBA_SIZE;wr_cnt++)
        {
            lba = lba_buff[buff_count].lba;
            lba_val = lba_buff[buff_count].lba_val;
            fill_buffer(test_buff,TEMP_BUFF_SIZE,lba_val);
            test_status=sd_drv_wr(test_buff,lba,MULTI_SECTOR_SIZE);
            if(test_status!=0)
            {
                MSG(SYS_INFO "sd write failed\r\n");
            }

            if (count%1000==0)
            {
                MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
                print_count++;
            } 
            count++;
            buff_count++;
        }
        //MSG(SYS_INFO "sd write count--%d @ %d\r\n",print_count,OSTimeGet(&err));
        //print_count++;

        //多块读数据
        buff_count=0;
        count=1;
        print_count=1;
        for(wr_cnt=0x00;wr_cnt<LBA_SIZE;wr_cnt++)
        {
            lba = lba_buff[buff_count].lba;
            test_status=sd_drv_rd(temp,lba,MULTI_SECTOR_SIZE);
            if(test_status!=0)
            {
                MSG(SYS_INFO "sd write failed\r\n");
            }

            if (count%1000==0)
            {
                MSG(SYS_INFO "sd read count--%d @ %d\r\n",print_count,OSTimeGet(&err));
                print_count++;
            } 
            count++;
            buff_count++;
        }
        loop_cnt++;
    }
    LED_R_OFF;
    tc_printf_tail("sd_tc_14 test ");

}

void sd_tc_run(void)
{
   sd_tc_01();
   sd_tc_11();
   sd_tc_02();
   sd_tc_03();
   sd_tc_04();
   sd_tc_05();
   sd_tc_06();
    sd_tc_07();
  // sd_tc_08();
    sd_tc_09();
    
    sd_tc_10();
    sd_tc_12();
    sd_tc_13();
    sd_tc_14();

}

