#ifndef _DELAY_H
#define _DELAY_H
#include <stm32f4xx.h>

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
typedef struct{
	uint32_t TMStart;
	uint32_t TMInter;
	
}tTimeDelay;


void SetTime(tTimeDelay *TimeType,uint32_t TimeInter);

uint8_t  CompareTime(tTimeDelay *TimeType);



void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
#endif

