#ifndef _FOURIER_DMA_H
#define _FOURIER_DMA_H
#include "common.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_pint.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_sct.h"
#include "LPC546XX_dma.h"
#include "LPC546XX_pll.h"
#include "LPC546XX_systick.h"
#include "LPC546XX_adc.h"

#define Fourier_DMACH DMA_CH1									//设置DMA传输的通道
#define Wave_Freq 		(20000)										//所要处理的信号频率,若要修改，则需要同时修改三角函数表
#define ADC_Samp_num  (5)											//设置一个周期采样点数目，一般取10个
#define ADC_Samp_SIZE (2*ADC_Samp_num)					//采样深度，必须为ADC_Samp_num的倍数，一般取两个周期，只处理第二个周期，以达到滤波目的
#define ADC_SampFreq  (Wave_Freq*ADC_Samp_num)	//ADC采样频率，必须是信号频率的ADC_Samp_num倍

#define Ratio 	15.0f		//滤波系数
#define FILTER_NUM 3		// 滤波深度，深度越大，数据跟随性越差，酌情设置，一般设置为3

extern struct Fourier_Data_t
{
		__IO  uint8 Reserve_1[100];
		__O 	uint16 Buff[ADC_Samp_SIZE];		//电压数据缓冲区
		__IO  uint8 Reserve_2[100];
	struct
	{
		__IO  uint8 Reserve_3[5];
		__IO  uint16 Result_WithoutFilt;			//电压数据结果(未滑动滤波)
		__IO	int32 Filt_BUF[FILTER_NUM];			//滑动滤波缓冲区
		__IO	int64 Filt_BUF_SUM;							//滑动滤波累加器
		__IO	uint8 Filt_point;								//数据指针
		
		__IO uint16 Result;			//电压数据滤波后结果
		
		__IO uint8  START_Flag;		//通道转换起始标志位
		__IO ADCRES_enum resolution;	//保存精度
	}ADCCH_Data[6];
	
	uint8 ADCCH_Save;	//保存此时正在转换的通道
	uint8 Busy_Flag;		//全局忙标志

}Fourier_Data;

void Fourier_Init(ADCCH_enum ch);
void Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution);
void Fourier_interrupt_Func(void);
#endif
