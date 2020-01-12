#ifndef _FOURIER_DMA_H
#define _FOURIER_DMA_H

#include "common.h"
#include "LPC546XX_dma.h"
#include "LPC546XX_adc.h"

//所要处理的信号频率,若要修改，则需要同时修改三角函数表
#define Wave_Freq 		(20000)										
//设置一个周期采样点数目，一般取5或10个，数目大会导致数据溢出，如果会卡住则降低采样点数目，若要修改，则需要同时修改三角函数表
#define ADC_Samp_num  (10)				
//采样深度，必须为ADC_Samp_num的倍数，一般取两个周期，只处理第二个周期，以达到滤波目的
#define ADC_Samp_SIZE (2*ADC_Samp_num)
//ADC采样频率，必须是信号频率的ADC_Samp_num倍
#define ADC_SampFreq  (Wave_Freq*ADC_Samp_num)	

ALIGN(512) extern dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX];
extern uint8 START_FLAG;

extern struct Fourier_Data_t
{
	uint8 reserve0[100];			//保留位，防止附近寄存器溢出导致的数据异常

	DMACH_enum DMA_CH[14];	//保存每个ADC通道的DMA传输通道
	
	uint8 reserve1[100];			//保留位，防止附近寄存器溢出导致的数据异常
	
	ADCCH_enum ADCCH_Save;	//保存此时正在转换的通道
	
	uint8 reserve2[100];			//保留位，防止附近寄存器溢出导致的数据异常
	
	uint8 Busy_Flag;				//全局忙标志
	
	uint8 reserve3[100];			//保留位，防止附近寄存器溢出导致的数据异常
}Fourier_Data;

extern struct kalman_Data_t
{
		uint8 reserve5[100];			//保留位，防止附近寄存器溢出导致的数据异常
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001，值只要不为0都可以，都会很快收敛
    float R;//观测噪声协方差 初始化值为0.2,值越大数据跟随性越差，会有相位滞后，值越小会使滤波效果变差
		uint8 reserve4[100];			//保留位，防止附近寄存器溢出导致的数据异常
}kalman_Data[14];


/* ----------------------------------------------------------------------------
   -- 用户调用的函数
   ---------------------------------------------------------------------------- */
void Fourier_Init(DMACH_enum dmach,ADCCH_enum ch);	//初始化函数，详细使用详见C文件，只需执行一次
uint16 Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution);//提取一次基波幅值，详细使用详见C文件，可重复执行
__STATIC_INLINE void Fourier_interrupt_Func(void)//中断服务函数，需要将它在DMA中断服务函数void DMA0_DriverIRQHandler(void)内调用
{
    if(READ_DMA_FLAG(Fourier_Data.DMA_CH[Fourier_Data.ADCCH_Save]))
    {
      CLEAR_DMA_FLAG(Fourier_Data.DMA_CH[Fourier_Data.ADCCH_Save]);
    }
		else
		{
			CLEAR_DMA_FLAG(Fourier_Data.DMA_CH[Fourier_Data.ADCCH_Save]);
		}
		START_FLAG=0;
}


#endif
