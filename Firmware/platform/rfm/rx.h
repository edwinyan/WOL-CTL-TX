#ifndef _RX_H_
#define _RX_H_

typedef struct {
  GPIO_TypeDef *gpio;
  u32 pin;
} pinDefine_t;

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))


void RXsetup(void);
void RXloop(void);


#endif
