/**
  ******************************************************************************
  * @file    Camera_OV7670.c 
  * @author  Jacky.Cheng
  * @version V1.0.0
  * @date    1-June-2012
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 摄像头配置和DCMI模式配置
  ******************************************************************************
  */
#include "main.h"
#include "Sensor_config.h"
#include "sccb.h"
#include "lcd.h"

static uint16_t line_start=0;
static uint8_t  frame_cnt=0;
static uint8_t  frame_start=0;
__IO uint8_t  frame_done;
uint8_t ov_frame=0;//统计帧率

void DCMI_Interface_Init(void)
{
  DCMI_InitTypeDef DCMI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable DCMI clock */
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
  /* DCMI configuration */ 
  //DCMI捕捉模式的设置，默认是连续模式,会看到连续的移动图像
  //用户在使用的时候可调整为SnapShot(抓拍)，也就是单帧
  DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;//DCMI_CaptureMode_SnapShot;
  DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
  DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Falling;
  DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
  DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
  //抓帧频率改为4帧中抓一帧
  DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;//DCMI_CaptureRate_1of4_Frame;
  DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;//经过4个像素时钟，捕获到32位字，触发1次DMA请求

  /* Configures the DMA2 to transfer Data from DCMI */
  /* Enable DMA2 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DCMI_DMA_CONFIG(0);
  
  //DCMI中断配置，在使用帧中断或者垂直同步中断的时候打开
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //使能DCMI中断
	//全局中断IT_DCMI
  DCMI_ITConfig(DCMI_IT_VSYNC, ENABLE);//使能垂直中断
  //DCMI_ITConfig(DCMI_IT_LINE, ENABLE);//使能水平中断
  //DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);//使能帧中断
  
  /* DCMI configuration */
  DCMI_Init(&DCMI_InitStructure);
          
   
//	DCMI_CaptureCmd(ENABLE);
//	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);

}

/*DCMI中断处理函数，这里使用的是垂直和水平同步中断，用户可根据需要自己调整中断的方式*/

void DCMI_IRQHandler(void)
{
	
	u8 zzz=0;
	zzz++;
	zzz++;
	
  
  //分两部分来处理所需的中断，来索取需要的帧数据
  if( DCMI_GetITStatus(DCMI_IT_VSYNC)!= RESET)
  {
    DCMI_ClearITPendingBit(DCMI_IT_VSYNC);
      
    if(frame_start)//等待帧开始信号
    {  
       DCMI_CaptureCmd(ENABLE);//当capture位置1时，激活DMA接口。DCMI_DR寄存器每收到一个32位数据块时，触发一个DMA请求
       DMA_Cmd(DMA2_Stream1, ENABLE); //2个DMA控制器。每个DMA控制器有8个数据流，每个数据流有8个通道 
			 LCD_SetCursor2(0x00,0x0000,0x013f,0x00ef);
			 LCD_WriteRAM_Prepare();
       while ((DMA_GetFlagStatus(DMA2_Stream1, DMA_FLAG_TCIF1) == RESET))//TCIF1是传输完成中断标志。硬件置1，软件清0
       {
       }      
			 //程序往下执行表示数据流传输完成
       DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_HTIF1 | DMA_FLAG_TCIF1);  //HTIF1是半传输完成中断标志。硬件置1，软件清0                                      
 //      DMA_Cmd(DMA2_Stream1, DISABLE);   
       //判断DMA传输是否完成   
 //      DCMI_ITConfig(DCMI_IT_VSYNC, DISABLE);//不使能垂直中断
 //      DCMI_CaptureCmd(DISABLE);
       frame_done = 1;
			 ov_frame++;
//			 GPIO_ToggleBits(GPIOC, GPIO_Pin_7);
    }
    
    if(frame_cnt<15)//在15帧后开始采集，等待摄像头稳定
    {
      if(frame_cnt==14)
      { 
        //DCMI_CaptureCmd(DISABLE);
        //DMA_Cmd(DMA2_Stream1, ENABLE);
     //   STM_EVAL_LEDToggle(LED2);
        frame_start = 1;//使能帧采集
        //DCMI_CaptureCmd(ENABLE);
        //DMA_Cmd(DMA2_Stream1, ENABLE);
      }
      frame_cnt = frame_cnt + 1;//帧开始标志位  
    }  
    else
      frame_cnt = 0;  
  }
  
 /**********************DCMI行中断处理**************************/ 
  if( DCMI_GetITStatus(DCMI_IT_LINE)!= RESET)//记录每行的数据
  {
    DCMI_ClearITPendingBit(DCMI_IT_LINE);
    if(frame_start)//等待帧开始信号
    {  
       DMA_Cmd(DMA2_Stream1, ENABLE);
       if(line_start<240)
       {
         //上半幅图像
         if(line_start==239)
         {
           DMA_Cmd(DMA2_Stream1, ENABLE); 
            //判断DMA传输是否完成
           while ((DMA_GetFlagStatus(DMA2_Stream1, DMA_FLAG_TCIF1) == RESET))
           {
           }
           
           DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_HTIF1 | DMA_FLAG_TCIF1);                                        
           /* Disable the DMA Stream */
           DMA_Cmd(DMA2_Stream1, DISABLE);  
           DCMI_ITConfig(DCMI_IT_FRAME, DISABLE);//不使能垂直中断
           DCMI_ITConfig(DCMI_IT_LINE, DISABLE);//不使能水平中断
           DCMI_CaptureCmd(DISABLE);  
           frame_done = 1;
         }
         line_start = line_start + 1;
       }                           
       else
        line_start = 0;//用作行计数,来控制dma停止      
    }     
  }  
  
/***********************DCMI帧中断处理函数*****************************/  
  if( DCMI_GetITStatus(DCMI_IT_FRAME)!= RESET)
  { 
    if(frame_start)//等待帧开始信号
    {  
        DCMI_CaptureCmd(ENABLE); 
        DMA_Cmd(DMA2_Stream1, ENABLE); 
        //判断DMA传输是否完成
        while ((DMA_GetFlagStatus(DMA2_Stream1, DMA_FLAG_TCIF1) == RESET))
        {
        }
           
        DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_HTIF1 | DMA_FLAG_TCIF1);        
        DMA_Cmd(DMA2_Stream1, DISABLE);          
        DCMI_ITConfig(DCMI_IT_FRAME, DISABLE);//不使能垂直中断       
        DCMI_CaptureCmd(DISABLE);
     
        frame_done = 1;
    }
    
    if(frame_cnt<15)//在15帧后开始采集，等待摄像头稳定
    {
      if(frame_cnt==14)
      { 
   //     STM_EVAL_LEDToggle(LED2);       
        frame_start = 1;//使能帧采集          
      }    
      frame_cnt = frame_cnt + 1;//帧开始标志位
    } 
    else
      frame_cnt = 0;

    DCMI_ClearITPendingBit(DCMI_IT_FRAME);
  } 
}

void DCMI_DMA_CONFIG(uint32_t addr_offset)
{
    DMA_InitTypeDef  DMA_InitStructure;      
    DMA_DeInit(DMA2_Stream1);

    DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
    DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;//DCMI数据寄存器地址，即buffer的起始地址
    //这里dma传输地址指向sram
    DMA_InitStructure.DMA_Memory0BaseAddr = (FSMC_LCD_ADDRESS+addr_offset);//(uint32_t)picture_buffer;//FSMC_SRAM_ADDRESS;//FSMC_LCD_ADDRESS;
    //传输方向从外设到内存
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    //Buffer大小设置成完整的一副图像大小
    DMA_InitStructure.DMA_BufferSize = 38400;//PICTURE_SIZE;//值变大，帧率减小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//设置DMA的外设递增模式。当DMA的通道连接多个外设时，需要设置DMA_PeripheralInc_Enable
    //这里地址需要增加,使能sram地址增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//设置内存递增模式。当DMA需要访问多个内存参数时，需要设置DMA_MemoryInc_Enable
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;//每次操作的数据长度
    //Memory数据宽度改为16bit，对应sram位宽
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA传输模式：连续循环模式/通用模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//设置DMA的优先级别
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    /* DMA2 IRQ channel Configuration */
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);
}

//摄像头配置函数，包括gpio和dcmi初始化
void Camera_Init(void)
{	 
    u16 i,j;
    u8  temp;
	  u32 cnt;
            
    OV7670_HW_Init();//初始化DCMI GPIO
    //设置sccb 配置gpio   
    SCCB_GPIO_Config();//初始化SDA,SCL
        
    SCCB_WriteByte(0x12, 0x80);  //软件复位摄像头
        
    //软件复位后必须要加延时 500ms
    for(j=0;j<500;j++)
        
       sccb_delay(1000);      
    
    SCCB_WriteByte(0x0B, 0x00);
        
    temp = SCCB_ReadByte(0x0B);  //读摄像头ID
	
	  cnt = sizeof(OV7670_RGB_reg)/sizeof(OV7670_RGB_reg[0]);	
	
    if(temp==0x73)//OV7670
    {  
	      for(i=0;i<OV7670_RGB_REG_NUM;i++)
            SCCB_WriteByte(OV7670_RGB_reg[i][0],OV7670_RGB_reg[i][1]);   
//			OV7670_Window_Set(1,174,240,320);
        //for(i=0;i<OV7670_YUV_REG_NUM;i++)
            //SCCB_WriteByte(OV7670_YUV_reg[i][0],OV7670_YUV_reg[i][1]);  
    }
		
    
    sccb_delay(1000000);//配置完成要延时，等待稳定
   
    //在摄像头配置完成后再打开dcmi采集功能
    DCMI_Interface_Init();
    
    sccb_delay(1000000);
    
} 

void OV7670_Window_Set(u16 sx,u16 sy,u16 width,u16 height)
{
 u16 endx;
 u16 endy;
 u8 temp; 
 endx=sx+width*2; //V*2
  endy=sy+height*2;
 if(endy>784)endy-=784;
 temp=SCCB_ReadByte(0X03);    //??Vref????
 temp&=0XF0;
 temp|=((endx&0X03)<<2)|(sx&0X03);
 SCCB_WriteByte(0X03,temp);    //??Vref?start?end???2?
 SCCB_WriteByte(0X19,sx>>2);   //??Vref?start?8?
 SCCB_WriteByte(0X1A,endx>>2);   //??Vref?end??8?

 temp=SCCB_ReadByte(0X32);    //??Href????
 temp&=0XC0;
 temp|=((endy&0X07)<<3)|(sy&0X07);
 SCCB_WriteByte(0X17,sy>>3);   //??Href?start?8?
 SCCB_WriteByte(0X18,endy>>3);   //??Href?end??8?
}


///////////////////

