#ifndef __hardware_i2c_H
#define __hardware_i2c_H
#include "stm32f4xx.h"


void hardware_i2c_init();

uint8_t ov7670_ReadReg(uint8_t Addr);
uint8_t ov7670_WriteReg(uint8_t Addr, uint8_t Data);
void ov7670_configuration(void);

#endif