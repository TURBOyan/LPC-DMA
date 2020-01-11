#ifndef _FOURIER_DMA_H
#define _FOURIER_DMA_H

#include "common.h"
#include "LPC546XX_dma.h"
#include "LPC546XX_adc.h"

#define Wave_Freq 		(20000)										//所要处理的信号频率,若要修改，则需要同时修改三角函数表
#define ADC_Samp_num  (10)											//设置一个周期采样点数目，一般取5或10个，数目大会导致数据溢出，如果会卡住则降低采样点数目，若要修改，则需要同时修改三角函数表
#define ADC_Samp_SIZE (2*ADC_Samp_num)					//采样深度，必须为ADC_Samp_num的倍数，一般取两个周期，只处理第二个周期，以达到滤波目的
#define ADC_SampFreq  (Wave_Freq*ADC_Samp_num)	//ADC采样频率，必须是信号频率的ADC_Samp_num倍

ALIGN(512) extern dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX];

extern struct Fourier_Data_t
{
	struct
	{
		uint32 Result;			//电压数据滤波后结果
		uint8 Reserve0[10];		//保留区，数据隔离，保护标志位，防止数据溢出导致的标志位错误
		DMACH_enum DMA_CH;
		uint8  START_Flag;		//通道转换起始标志位,这个标志位非常重要，如果他存储区附近有任何寄存器溢出，会改变此标志位，导致程序卡住
		uint8 Reserve1[10];  //保留区，数据隔离，保护标志位，防止数据溢出导致的标志位错误
		
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001，值只要不为0都可以，都会很快收敛
    float R;//观测噪声协方差 初始化值为0.2,值越大数据跟随性越差，会有相位滞后，值越小会使滤波效果变差

	}ADCCH_Data[14];
	
	ADCCH_enum ADCCH_Save;	//保存此时正在转换的通道
	uint8 Busy_Flag;		//全局忙标志
}Fourier_Data;

void Fourier_Init(DMACH_enum dmach,ADCCH_enum ch);
uint16 Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution);

__STATIC_INLINE void Fourier_interrupt_Func(void)
{
    if(READ_DMA_FLAG(Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].DMA_CH))
    {
      CLEAR_DMA_FLAG(Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].DMA_CH);
    }
		else
		{
			CLEAR_DMA_FLAG(Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].DMA_CH);
		}
		Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].START_Flag = 0;		//清除起始标志位，以便下次转换
}


#endif
