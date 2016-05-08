#include "hardware_i2c.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx.h"
#include "sccb.h"

#define TIMEOUT_MAX2     10000

#define ov7670_DEVICE_WRITE_ADDRESS    0x42
#define ov7670_DEVICE_READ_ADDRESS     0x43


void hardware_i2c_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	I2C_DeInit(I2C2);

	
	I2C_Cmd(I2C2, ENABLE);

	
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 100000;

	
	I2C_Init(I2C2, &I2C_InitStruct);
}

uint8_t ov7670_WriteReg(uint8_t Addr, uint8_t Data)
{
  uint32_t timeout = TIMEOUT_MAX2;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
 //   if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7670_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
 //   if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Addr);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
 //   if ((timeout--) == 0) return 0xFF;
  }

  /* Send Data */
  I2C_SendData(I2C2, Data);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
 //   if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 STOP Condition */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* If operation is OK, return 0 */
  return 0;
}

uint8_t ov7670_ReadReg(uint8_t Addr)
{
  uint32_t timeout = TIMEOUT_MAX2;
  uint8_t Data = 0;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    //if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7670_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    //if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Addr);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    //if ((timeout--) == 0) return 0xFF;
  }

  /* Clear AF flag if arised */
  I2C2->SR1 |= (uint16_t)0x0400;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    //if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7670_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    //if ((timeout--) == 0) return 0xFF;
  }

  /* Prepare an NACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, DISABLE);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX2; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    //if ((timeout--) == 0) return 0xFF;
  }

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Receive the Data */
  Data = I2C_ReceiveData(I2C2);

  /* return the read data */
  return Data;
}

void ov7670_configuration(void)
{

  ov7670_WriteReg(0x12, 0x80);
  sccb_delay(1000000);

  if( (ov7670_ReadReg(0x0b))==(0x73) )
  {
  ov7670_WriteReg(0x3a, 0x0C); 
  
  //ò???3??2?êyó?à'????í???êy?Yê?3???ê??oQVGA RGB565
  ov7670_WriteReg(0x40, 0xd0); 
  ov7670_WriteReg(0x12, 0x14); //COM7 
//  ov7670_WriteReg(0x8c,0x00);//default
  
  //ò???6??2?êyó?à'????HSTART,HSTOP,VSTART,VSTOP
  ov7670_WriteReg(0x32, 0x80); 
  ov7670_WriteReg(0x17, 0x17); 
  ov7670_WriteReg(0x18, 0x05); 
  ov7670_WriteReg(0x19, 0x02); 
  ov7670_WriteReg(0x1a, 0x7b);//0x7a, 
  ov7670_WriteReg(0x03, 0x0a);//0x0a,     
  
  ov7670_WriteReg(0x0c, 0x00); 
  ov7670_WriteReg(0x3e, 0x00);// 
  
  //0x70:0x00 0x71:0x812êì?2aê?
  ov7670_WriteReg(0x70, 0x00); 
  ov7670_WriteReg(0x71, 0x81); 
  
  ov7670_WriteReg(0x72, 0x11); 
  ov7670_WriteReg(0x73, 0x00);// 
  ov7670_WriteReg(0xa2, 0x02); 
  ov7670_WriteReg(0x11, 0x81); //?ú2?ê±?ó·??µ,PCLK=?ú2?ê±?ó/2
  ov7670_WriteReg(0x7a, 0x20); 
  ov7670_WriteReg(0x7b, 0x1c); 
  ov7670_WriteReg(0x7c, 0x28); 
  ov7670_WriteReg(0x7d, 0x3c); 
  ov7670_WriteReg(0x7e, 0x55); 
  ov7670_WriteReg(0x7f, 0x68); 
  ov7670_WriteReg(0x80, 0x76); 
  ov7670_WriteReg(0x81, 0x80); 
  ov7670_WriteReg(0x82, 0x88); 
  ov7670_WriteReg(0x83, 0x8f); 
  ov7670_WriteReg(0x84, 0x96); 
  ov7670_WriteReg(0x85, 0xa3); 
  ov7670_WriteReg(0x86, 0xaf); 
  ov7670_WriteReg(0x87, 0xc4); 
  ov7670_WriteReg(0x88, 0xd7); 
  ov7670_WriteReg(0x89, 0xe8); 
  ov7670_WriteReg(0x13, 0xe0); 
  ov7670_WriteReg(0x00, 0x00);//AGC 
  ov7670_WriteReg(0x10, 0x00); 
  ov7670_WriteReg(0x0d, 0x00); //0x0d:è?è?2?'°?ú
  ov7670_WriteReg(0x14, 0x28);//0x38, limit the max gain 
  ov7670_WriteReg(0xa5, 0x05); 
  ov7670_WriteReg(0xab, 0x07); 
  ov7670_WriteReg(0x24, 0x75); 
  ov7670_WriteReg(0x25, 0x63); 
  ov7670_WriteReg(0x26, 0xA5); 
  ov7670_WriteReg(0x9f, 0x78); 
  ov7670_WriteReg(0xa0, 0x68); 
  ov7670_WriteReg(0xa1, 0x03);//0x0b, 
  ov7670_WriteReg(0xa6, 0xdf);//0xd8, 
  ov7670_WriteReg(0xa7, 0xdf);//0xd8, 
  ov7670_WriteReg(0xa8, 0xf0); 
  ov7670_WriteReg(0xa9, 0x90); 
  ov7670_WriteReg(0xaa, 0x94); 
  ov7670_WriteReg(0x13, 0xe5); 
  ov7670_WriteReg(0x0e, 0x61); 
  ov7670_WriteReg(0x0f, 0x4b); 
  ov7670_WriteReg(0x16, 0x02); 
  ov7670_WriteReg(0x1e, 0x07);//0x07, 
  ov7670_WriteReg(0x21, 0x02); 
  ov7670_WriteReg(0x22, 0x91); 
  ov7670_WriteReg(0x29, 0x07); 
  ov7670_WriteReg(0x33, 0x0b); 
  ov7670_WriteReg(0x35, 0x0b); 
  ov7670_WriteReg(0x37, 0x1d); 
  ov7670_WriteReg(0x38, 0x71); 
  ov7670_WriteReg(0x39, 0x2a); 
  ov7670_WriteReg(0x3c, 0x78); 
  ov7670_WriteReg(0x4d, 0x40); 
  ov7670_WriteReg(0x4e, 0x20); 
  ov7670_WriteReg(0x69, 0x55); 
  ov7670_WriteReg(0x6b, 0x0a);//PLL ??òa2?êy  0X0A:???·PLL  0X40?oPLL=ê?è?ê±?óx4 0x80:PLL=ê?è?ê±?óx6
  ov7670_WriteReg(0x74, 0x19); 
  ov7670_WriteReg(0x8d, 0x4f); 
  ov7670_WriteReg(0x8e, 0x00); 
  ov7670_WriteReg(0x8f, 0x00); 
  ov7670_WriteReg(0x90, 0x00); 
  ov7670_WriteReg(0x91, 0x00); 
  ov7670_WriteReg(0x92, 0x00);//0x19,//0x66 
  ov7670_WriteReg(0x96, 0x00); 
  ov7670_WriteReg(0x9a, 0x80); 
  ov7670_WriteReg(0xb0, 0x84); 
  ov7670_WriteReg(0xb1, 0x0c); 
  ov7670_WriteReg(0xb2, 0x0e); 
  ov7670_WriteReg(0xb3, 0x82); 
  ov7670_WriteReg(0xb8, 0x0a); 
  ov7670_WriteReg(0x43, 0x14); 
  ov7670_WriteReg(0x44, 0xf0); 
  ov7670_WriteReg(0x45, 0x34); 
  ov7670_WriteReg(0x46, 0x58); 
  ov7670_WriteReg(0x47, 0x28); 
  ov7670_WriteReg(0x48, 0x3a); 
  ov7670_WriteReg(0x59, 0x88); 
  ov7670_WriteReg(0x5a, 0x88); 
  ov7670_WriteReg(0x5b, 0x44); 
  ov7670_WriteReg(0x5c, 0x67); 
  ov7670_WriteReg(0x5d, 0x49); 
  ov7670_WriteReg(0x5e, 0x0e); 
  ov7670_WriteReg(0x64, 0x04); 
  ov7670_WriteReg(0x65, 0x20); 
  ov7670_WriteReg(0x66, 0x05); 
  ov7670_WriteReg(0x94, 0x04); 
  ov7670_WriteReg(0x95, 0x08); 
  ov7670_WriteReg(0x6c, 0x0a); 
  ov7670_WriteReg(0x6d, 0x55); 
  ov7670_WriteReg(0x6e, 0x11); 
  ov7670_WriteReg(0x6f, 0x9f);//0x9e for advance AWB 
  ov7670_WriteReg(0x6a, 0x40); 
  ov7670_WriteReg(0x01, 0x40); 
  ov7670_WriteReg(0x02, 0x40); 
  ov7670_WriteReg(0x13, 0xe7); 
  ov7670_WriteReg(0x15, 0x08); //??òa2?êy 
  ov7670_WriteReg(0x4f, 0x80); 
  ov7670_WriteReg(0x50, 0x80); 
  ov7670_WriteReg(0x51, 0x00); 
  ov7670_WriteReg(0x52, 0x22); 
  ov7670_WriteReg(0x53, 0x5e); 
  ov7670_WriteReg(0x54, 0x80); 
  ov7670_WriteReg(0x55, 0x0A);//áá?è 
  ov7670_WriteReg(0x56, 0x4f);//??±è?è 
  ov7670_WriteReg(0x58, 0x9e);  
  ov7670_WriteReg(0x41, 0x08); 
  ov7670_WriteReg(0x3f, 0x05);//±??µ????µ÷?? 
  ov7670_WriteReg(0x75, 0x05); 
  ov7670_WriteReg(0x76, 0xe1); 
  ov7670_WriteReg(0x4c, 0x0F);//??éùò??????è 
  ov7670_WriteReg(0x77, 0x0a); 
  ov7670_WriteReg(0x3d, 0xc2);//0xc0, 
  ov7670_WriteReg(0x4b, 0x09); 
  ov7670_WriteReg(0xc9, 0x60); 
  ov7670_WriteReg(0x41, 0x38); 
  ov7670_WriteReg(0x34, 0x11); 
  ov7670_WriteReg(0x3b, 0x02);//0x00,//0x02, 
  ov7670_WriteReg(0xa4, 0x89);//0x88, 
  ov7670_WriteReg(0x96, 0x00); 
  ov7670_WriteReg(0x97, 0x30); 
  ov7670_WriteReg(0x98, 0x20); 
  ov7670_WriteReg(0x99, 0x30); 
  ov7670_WriteReg(0x9a, 0x84); 
  ov7670_WriteReg(0x9b, 0x29); 
  ov7670_WriteReg(0x9c, 0x03); 
  ov7670_WriteReg(0x9d, 0x4c); 
  ov7670_WriteReg(0x9e, 0x3f); 
  ov7670_WriteReg(0x78, 0x04);  
  ov7670_WriteReg(0x79, 0x01); 
  ov7670_WriteReg(0xc8, 0xf0); 
  ov7670_WriteReg(0x79, 0x0f); 
  ov7670_WriteReg(0xc8, 0x00); 
  ov7670_WriteReg(0x79, 0x10); 
  ov7670_WriteReg(0xc8, 0x7e); 
  ov7670_WriteReg(0x79, 0x0a); 
  ov7670_WriteReg(0xc8, 0x80); 
  ov7670_WriteReg(0x79, 0x0b); 
  ov7670_WriteReg(0xc8, 0x01); 
  ov7670_WriteReg(0x79, 0x0c); 
  ov7670_WriteReg(0xc8, 0x0f); 
  ov7670_WriteReg(0x79, 0x0d); 
  ov7670_WriteReg(0xc8, 0x20); 
  ov7670_WriteReg(0x79, 0x09); 
  ov7670_WriteReg(0xc8, 0x80); 
  ov7670_WriteReg(0x79, 0x02); 
  ov7670_WriteReg(0xc8, 0xc0); 
  ov7670_WriteReg(0x79, 0x03); 
  ov7670_WriteReg(0xc8, 0x40); 
  ov7670_WriteReg(0x79, 0x05); 
  ov7670_WriteReg(0xc8, 0x30); 
  ov7670_WriteReg(0x79, 0x26); 
  ov7670_WriteReg(0x09, 0x02); 
  ov7670_WriteReg(0x3b, 0x42);//0x82,//0xc0,//0xc2, //night mode
  sccb_delay(1000000);
  }
}


