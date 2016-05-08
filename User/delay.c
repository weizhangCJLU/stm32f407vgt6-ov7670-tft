#include "delay.h"

#include "stm32f4xx.h"

void delay_us(u32 nus)
{		
	u16 i;
	for(i=0;i<10;i++)	while(nus--);
}

//ÑÓÊ±nms 
//nms:0~65535
void delay_ms(u16 nms)
{	 	 
	u16 i;
	
	for(i=0;i<100;i++) while(nms--);
	
} 