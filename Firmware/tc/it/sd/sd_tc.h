#ifndef _SD_TC_H_
#define _SD_TC_H_

#define MAX_ADDR             0x3AF000
#define BLOCKSIZE            512            /* Block Size in Bytes */
#define buffer_words_size    BLOCKSIZE
#define buffer_multiwords_size    BLOCKSIZE*2
//#define rand_test_size       10000          /* 10k */
//#define rand_test_size       0x5000         /* 20k */
//#define rand_test_size       0x6400         /* 25k */
//#define rand_test_size       0x6800         /* 26k */
//#define rand_test_size       0x7800         /* 30k */
//#define rand_test_size       0x16800         /* 90k */
//#define rand_test_size       0x100000         /* 1M */
//#define rand_test_size         0x3AF000         /* 3772k */

typedef struct{
u32 lba;
u8 lba_val;
}lba_pattern_t;

typedef struct{
u8 test_buff[buffer_words_size];

} teststruct_t;
void sd_tc_run(void);

#endif
