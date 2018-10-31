#ifndef _SD_CONFIG_H_
#define _SD_CONFIG_H_

#include "common.h"
#include "sd_response.h"
#include "sd_command.h"

#define SD_WIDE_BUS_SUPPORT             (0x00040000L)
#define SD_SINGLE_BUS_SUPPORT           (0x00010000L)
#define SD_CARD_LOCKED                  (0x02000000L)
#define SD_DATATIMEOUT                  (0x00FFFFFFL)
#define SD_0TO7BITS                     (0x000000FFL)
#define SD_8TO15BITS                    (0x0000FF00L)
#define SD_16TO23BITS                   (0x00FF0000L)
#define SD_24TO31BITS                   (0xFF000000L)
#define SD_MAX_DATA_LENGTH              (0x01FFFFFFL)

/* @brief  SD detect state */
#define SD_PRESENT                                 (0x01)
#define SD_NOT_PRESENT                             (0x00)


/* Card Specific Data */
typedef struct      
{
    __IO uint8_t  CSDStruct;            /* CSD structure */
    __IO uint8_t  SysSpecVersion;       /* System specification version */
    __IO uint8_t  Reserved1;            /* Reserved */
    __IO uint8_t  TAAC;                 /* Data read access-time 1 */
    __IO uint8_t  NSAC;                 /* Data read access-time 2 in CLK cycles */
    __IO uint8_t  MaxBusClkFrec;        /* Max. bus clock frequency */
    __IO uint16_t CardComdClasses;      /* Card command classes */
    __IO uint8_t  RdBlockLen;           /* Max. read data block length */
    __IO uint8_t  PartBlockRead;        /* Partial blocks for read allowed */
    __IO uint8_t  WrBlockMisalign;      /* Write block misalignment */
    __IO uint8_t  RdBlockMisalign;      /* Read block misalignment */
    __IO uint8_t  DSRImpl;              /* DSR implemented */
    __IO uint8_t  Reserved2;            /* Reserved */
    __IO uint32_t DeviceSize;           /* Device Size */
    __IO uint8_t  MaxRdCurrentVDDMin;   /* Max. read current @ VDD min */
    __IO uint8_t  MaxRdCurrentVDDMax;   /* Max. read current @ VDD max */
    __IO uint8_t  MaxWrCurrentVDDMin;   /* Max. write current @ VDD min */
    __IO uint8_t  MaxWrCurrentVDDMax;   /* Max. write current @ VDD max */
    __IO uint8_t  DeviceSizeMul;        /* Device size multiplier */
    __IO uint8_t  EraseGrSize;          /* Erase group size */
    __IO uint8_t  EraseGrMul;           /* Erase group size multiplier */
    __IO uint8_t  WrProtectGrSize;      /* Write protect group size */
    __IO uint8_t  WrProtectGrEnable;    /* Write protect group enable */
    __IO uint8_t  ManDeflECC;           /* Manufacturer default ECC */
    __IO uint8_t  WrSpeedFact;          /* Write speed factor */
    __IO uint8_t  MaxWrBlockLen;        /* Max. write data block length */
    __IO uint8_t  WriteBlockPaPartial;  /* Partial blocks for write allowed */
    __IO uint8_t  Reserved3;            /* Reserded */
    __IO uint8_t  ContentProtectAppli;  /* Content protection application */
    __IO uint8_t  FileFormatGrouop;     /* File format group */
    __IO uint8_t  CopyFlag;             /* Copy flag (OTP) */
    __IO uint8_t  PermWrProtect;        /* Permanent write protection */
    __IO uint8_t  TempWrProtect;        /* Temporary write protection */
    __IO uint8_t  FileFormat;           /* File Format */
    __IO uint8_t  ECC;                  /* ECC code */
    __IO uint8_t  CSD_CRC;              /* CSD CRC */
    __IO uint8_t  Reserved4;            /* always 1*/
} sd_csd_t;

/*Card Identification Data*/
typedef struct          
{
    __IO uint8_t  ManufacturerID;       /* ManufacturerID */
    __IO uint16_t OEM_AppliID;          /* OEM/Application ID */
    __IO uint32_t ProdName1;            /* Product Name part1 */
    __IO uint8_t  ProdName2;            /* Product Name part2*/
    __IO uint8_t  ProdRev;              /* Product Revision */
    __IO uint32_t ProdSN;               /* Product Serial Number */
    __IO uint8_t  Reserved1;            /* Reserved1 */
    __IO uint16_t ManufactDate;         /* Manufacturing Date */
    __IO uint8_t  CID_CRC;              /* CID CRC */
    __IO uint8_t  Reserved2;            /* always 1 */
} sd_cid_t;

/*Card information Data*/
typedef struct
{
    sd_csd_t SD_csd;
    sd_cid_t SD_cid;
    uint32_t CardCapacity;     /*!< Card Capacity */
    uint32_t CardBlockSize;    /*!< Card Block Size */
    uint16_t RCA;
    uint8_t CardType;
} sd_cardinfo_t;

/** 
  * @brief  SDIO Transfer state 
  */ 
typedef enum 
{
  SD_TRANSFER_OK     = 0,
  SD_TRANSFER_BUSY   = 1,
  SD_TRANSFER_ERROR
} sd_transferstate_enum;


uint8_t sd_detect(void);
void sd_reset(void);
sd_transferstate_enum sd_getstatus(void);
sd_error_enum sd_getcardinfo(sd_cardinfo_t *cardinfo);
sd_error_enum sd_selectdeselect(uint32_t addr);
sd_error_enum sd_enable_widebusoperation(uint32_t WideMode);
sd_error_enum sd_erase(uint32_t startaddr, uint32_t endaddr);
#endif
