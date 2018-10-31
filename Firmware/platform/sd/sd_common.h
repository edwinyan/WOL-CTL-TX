#ifndef _SD_COMMON_H_
#define _SD_COMMON_H_

#define SDIO_STA_ALLERRBITS             (0x0000023FL)     /*SDIO_STA register all error bit*/

#define SDIO_INIT_CLK_DIV                  (0x76)
#define SDIO_TRANSFER_CLK_DIV              (0x0)    //24M
//#define SDIO_TRANSFER_CLK_DIV              (0x2E)    //1M
//#define SDIO_TRANSFER_CLK_DIV              (0x0A)    //4M
//#define SDIO_TRANSFER_CLK_DIV              (0x04)    //8M
//#define SDIO_TRANSFER_CLK_DIV              (0x02)    //12M
//#define SDIO_TRANSFER_CLK_DIV              (0x01)    //16M

#define SD_DETECT_PORT                    GPIOC
#define SD_DETECT_PIN                     GPIO_Pin_13

#define SD_POWER_PORT                    GPIOC
#define SD_POWER_PIN                     GPIO_Pin_14

#define SD_SECTOR_U8_SIZE		(512)
#define SD_SECTOR_BIT_EXP          (9)
#define SD_SECTOR_BIT_MASK        ((1UL<<SD_SECTOR_BIT_EXP) - 1)

/* Command Class Supported */
#define SD_CCCC_LOCK_UNLOCK             (0x00000080L)
#define SD_CCCC_WRITE_PROT              (0x00000040L)
#define SD_CCCC_ERASE                   (0x00000020L)

/* SDIO Commands  Index */
#define SDIO_GO_IDLE_STATE                       (0)
#define SDIO_SEND_OP_COND                        (1)
#define SDIO_ALL_SEND_CID                        (2)
#define SDIO_SET_REL_ADDR                        (3) /* SDIO_SEND_REL_ADDR for SD Card */
#define SDIO_SET_DSR                             (4)
#define SDIO_SDIO_SEN_OP_COND                    (5)
#define SDIO_HS_SWITCH                           (6)
#define SDIO_SEL_DESEL_CARD                      (7)
#define SDIO_HS_SEND_EXT_CSD                     (8)
#define SDIO_SEND_CSD                            (9)
#define SDIO_SEND_CID                            (10)
#define SDIO_READ_DAT_UNTIL_STOP                 (11) /* SD Card doesn't support it */
#define SDIO_STOP_TRANSMISSION                   (12)
#define SDIO_SEND_STATUS                         (13)
#define SDIO_HS_BUSTEST_READ                     (14)
#define SDIO_GO_INACTIVE_STATE                   (15)
#define SDIO_SET_BLOCKLEN                        (16)
#define SDIO_READ_SINGLE_BLOCK                   (17)
#define SDIO_READ_MULT_BLOCK                     (18)
#define SDIO_HS_BUSTEST_WRITE                    (19)
#define SDIO_WRITE_DAT_UNTIL_STOP                (20) /* SD Card doesn't support it */
#define SDIO_SET_BLOCK_COUNT                     (23) /* SD Card doesn't support it */
#define SDIO_WRITE_SINGLE_BLOCK                  (24)
#define SDIO_WRITE_MULT_BLOCK                    (25)
#define SDIO_PROG_CID                            (26) /* reserved for manufacturers */
#define SDIO_PROG_CSD                            (27)
#define SDIO_SET_WRITE_PROT                      (28)
#define SDIO_CLR_WRITE_PROT                      (29)
#define SDIO_SEND_WRITE_PROT                     (30)
#define SDIO_SD_ERASE_GRP_START                  (32) /* To set the address of the first write
                                                                  block to be erased. (For SD card only) */
#define SDIO_SD_ERASE_GRP_END                    (33) /* To set the address of the last write block of the
                                                                  continuous range to be erased. (For SD card only) */
#define SDIO_ERASE_GRP_START                     (35) /* To set the address of the first write block to be erased.
                                                                  (For MMC card only spec 3.31) */

#define SDIO_ERASE_GRP_END                       (36) /* To set the address of the last write block of the
                                                                  continuous range to be erased. (For MMC card only spec 3.31) */

#define SDIO_ERASE                               (38)
#define SDIO_FAST_IO                             (39) /* SD Card doesn't support it */
#define SDIO_GO_IRQ_STATE                        (40) /* SD Card doesn't support it */
#define SDIO_LOCK_UNLOCK                         (42)
#define SDIO_APP_CMD                             (55)
#define SDIO_GEN_CMD                             (56)
#define SDIO_NO_CMD                              (64)


/* Following commands are SD Card Specific commands.
   SDIO_APP_CMD should be sent before sending these
   commands. */
#define SDIO_APP_SD_SET_BUSWIDTH                 (6)  /* For SD Card only */
#define SDIO_SD_APP_STAUS                        (13) /* For SD Card only */
#define SDIO_SD_APP_SEND_NUM_WRITE_BLOCKS        (22) /* For SD Card only */
#define SDIO_SD_APP_OP_COND                      (41) /* For SD Card only */
#define SDIO_SD_APP_SET_CLR_CARD_DETECT          (42) /* For SD Card only */
#define SDIO_SD_APP_SEND_SCR                     (51) /* For SD Card only */
#define SDIO_SDIO_RW_DIRECT                      (52) /* For SD I/O Card only */
#define SDIO_SDIO_RW_EXTENDED                    (53) /* For SD I/O Card only */

/* Supported Memory Cards */
#define SDIO_STD_CAPACITY_SD_CARD_V1_1     (0x0L)
#define SDIO_STD_CAPACITY_SD_CARD_V2_0     (0x1L)
#define SDIO_HIGH_CAPACITY_SD_CARD         (0x2L)
#define SDIO_MULTIMEDIA_CARD               (0x3L)
#define SDIO_SECURE_DIGITAL_IO_CARD        (0x4L)
#define SDIO_HIGH_SPEED_MULTIMEDIA_CARD    (0x5L)
#define SDIO_SECURE_DIGITAL_IO_COMBO_CARD  (0x6L)
#define SDIO_HIGH_CAPACITY_MMC_CARD        (0x7L)


/** 
  * @brief  SD Card States 
  */   
typedef enum
{
  SD_CARD_READY                  = (0x00000001L),
  SD_CARD_IDENTIFICATION         = (0x00000002L),
  SD_CARD_STANDBY                = (0x00000003L),
  SD_CARD_TRANSFER               = (0x00000004L),
  SD_CARD_SENDING                = (0x00000005L),
  SD_CARD_RECEIVING              = (0x00000006L),
  SD_CARD_PROGRAMMING            = (0x00000007L),
  SD_CARD_DISCONNECTED           = (0x00000008L),
  SD_CARD_ERROR                  = (0x000000FFL)
}sd_cardstate_enum;


typedef enum
{
    /* SDIO specific error defines */
    SD_CMD_CRC_FAIL                    = (1), /* Command response received (but CRC check failed) */
    SD_DATA_CRC_FAIL                   = (2), /* Data bock sent/received (CRC check Failed) */
    SD_CMD_RSP_TIMEOUT                 = (3), /* Command response timeout */
    SD_DATA_TIMEOUT                    = (4), /* Data time out */
    SD_TX_UNDERRUN                     = (5), /* Transmit FIFO under-run */
    SD_RX_OVERRUN                      = (6), /* Receive FIFO over-run */
    SD_START_BIT_ERR                   = (7), /* Start bit not detected on all data signals in widE bus mode */
    SD_CMD_OUT_OF_RANGE                = (8), /* CMD's argument was out of range.*/
    SD_ADDR_MISALIGNED                 = (9), /* Misaligned address */
    SD_BLOCK_LEN_ERR                   = (10), /* Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
    SD_ERASE_SEQ_ERR                   = (11), /* An error in the sequence of erase command occurs.*/
    SD_BAD_ERASE_PARAM                 = (12), /* An Invalid selection for erase groups */
    SD_WRITE_PROT_VIOLATION            = (13), /* Attempt to program a write protect block */
    SD_LOCK_UNLOCK_FAILED              = (14), /* Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card */
    SD_COM_CRC_FAILED                  = (15), /* CRC check of the previous command failed */
    SD_ILLEGAL_CMD                     = (16), /* Command is not legal for the card state */
    SD_CARD_ECC_FAILED                 = (17), /* Card internal ECC was applied but failed to correct the data */
    SD_CC_ERROR                        = (18), /* Internal card controller error */
    SD_GENERAL_UNKNOWN_ERROR           = (19), /* General or Unknown error */
    SD_STREAM_READ_UNDERRUN            = (20), /* The card could not sustain data transfer in stream read operation. */
    SD_STREAM_WRITE_OVERRUN            = (21), /* The card could not sustain data programming in stream mode */
    SD_CID_CSD_OVERWRITE               = (22), /* CID/CSD overwrite error */
    SD_WP_ERASE_SKIP                   = (23), /* only partial address space was erased */
    SD_CARD_ECC_DISABLED               = (24), /* Command has been executed without using internal ECC */
    SD_ERASE_RESET                     = (25), /* Erase sequence was cleared before executing because an out of erase sequence command was received */
    SD_AKE_SEQ_ERROR                   = (26), /* Error in sequence of authentication. */
    SD_INVALID_VOLTRANGE               = (27),
    SD_ADDR_OUT_OF_RANGE               = (28),
    SD_SWITCH_ERROR                    = (29),
    SD_SDIO_DISABLED                   = (30),
    SD_SDIO_FUNCTION_BUSY              = (31),
    SD_SDIO_FUNCTION_FAILED            = (32),
    SD_SDIO_UNKNOWN_FUNCTION           = (33),

    /* Standard error defines */
    SD_INTERNAL_ERROR, 
    SD_NOT_CONFIGURED,
    SD_REQUEST_PENDING, 
    SD_REQUEST_NOT_APPLICABLE, 
    SD_INVALID_PARAMETER,  
    SD_UNSUPPORTED_FEATURE,  
    SD_UNSUPPORTED_HW,  
    SD_ERROR,  
    SD_OK,  
} sd_error_enum;




#endif
