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
#include "Selfbuild_oled.h"

#define Fourier_DMACH DMA_CH1		//设置DMA传输的通道
#define ADC_Samp_SIZE 20				//采样深度，必须为10的倍数
#define ADC_SampFreq  200000		//ADC采样频率


extern struct Fourier_Data_t
{
		__O uint16 Buff[ADC_Samp_SIZE];		//电压数据缓冲区
	
	struct
	{
		__IO uint16 Result;			//电压数据结果
		__IO uint8  START_Flag;		//通道转换起始标志位
		__IO ADCRES_enum resolution;	//保存精度
	}ADCCH_Data[6];
	
	 uint8 ADCCH_Save;	//保存此时正在转换的通道
	 uint8 Busy_Flag;		//全局忙标志
}Fourier_Data;

void Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution);
void Fourier_interrupt_Func(void);
#endif
