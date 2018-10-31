#ifndef _SD_RESPONSE_H_
#define _SD_RESPONSE_H_

#include "sd_common.h"

typedef struct
{
    uint32_t      card_status;
    sd_error_enum error_code;
}card_error_status_t;

#define SDIO_CMD0TIMEOUT                (0x00002710)  //10000
#define SDIO_STATIC_FLAGS               (0x000005FF)
#define ERROR_STATUS_ENUM               (30)
#define R1_COUNT                        (18)
#define R6_COUNT                        (3)
#define R1_CHECK_START                  (0)
#define R1_CHECK_END                     R1_COUNT 
#define R6_CHECK_START                   R1_COUNT
#define R6_CHECK_END                    (R1_COUNT+R6_COUNT)

/* Mask for errors Card Status R1 (OCR Register) */
#define SD_OCR_ADDR_OUT_OF_RANGE        (0x80000000L)
#define SD_OCR_ADDR_MISALIGNED          (0x40000000L)
#define SD_OCR_BLOCK_LEN_ERR            (0x20000000L)
#define SD_OCR_ERASE_SEQ_ERR            (0x10000000L)
#define SD_OCR_BAD_ERASE_PARAM          (0x08000000L)
#define SD_OCR_WRITE_PROT_VIOLATION     (0x04000000L)
#define SD_OCR_LOCK_UNLOCK_FAILED       (0x01000000L)
#define SD_OCR_COM_CRC_FAILED           (0x00800000L)
#define SD_OCR_ILLEGAL_CMD              (0x00400000L)
#define SD_OCR_CARD_ECC_FAILED          (0x00200000L)
#define SD_OCR_CC_ERROR                 (0x00100000L)
#define SD_OCR_GENERAL_UNKNOWN_ERROR    (0x00080000L)
#define SD_OCR_STREAM_READ_UNDERRUN     (0x00040000L)
#define SD_OCR_STREAM_WRITE_OVERRUN     (0x00020000L)
#define SD_OCR_CID_CSD_OVERWRIETE       (0x00010000L)
#define SD_OCR_WP_ERASE_SKIP            (0x00008000L)
#define SD_OCR_CARD_ECC_DISABLED        (0x00004000L)
#define SD_OCR_ERASE_RESET              (0x00002000L)
#define SD_OCR_AKE_SEQ_ERROR            (0x00000008L)
#define SD_OCR_ERRORBITS                (0xFDFFE008L)

/* Masks for R6 Response */
#define SD_R6_GENERAL_UNKNOWN_ERROR     (0x00002000L)
#define SD_R6_ILLEGAL_CMD               (0x00004000L)
#define SD_R6_COM_CRC_FAILED            (0x00008000L)
#define SD_R6_ERRORBITS                 (0x0000E000L)
#define SD_ALLZERO                      (0x00000000L)


sd_error_enum cmd_error(void);
sd_error_enum cmd_resp1_error(uint8_t cmd);
sd_error_enum cmd_resp2_error(void);
sd_error_enum cmd_resp3_error(void);
sd_error_enum cmd_resp6_error(uint8_t cmd, uint16_t *prca);
sd_error_enum cmd_resp7_error(void);
#endif
