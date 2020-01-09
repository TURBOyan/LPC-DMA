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

#define Fourier_DMACH DMA_CH0									//设置DMA传输的通道
#define Wave_Freq 		(20000)										//所要处理的信号频率,若要修改，则需要同时修改三角函数表
#define ADC_Samp_num  (10)											//设置一个周期采样点数目，一般取5或10个，数目大会导致数据溢出，如果会卡住则降低采样点数目，若要修改，则需要同时修改三角函数表
#define ADC_Samp_SIZE (2*ADC_Samp_num)					//采样深度，必须为ADC_Samp_num的倍数，一般取两个周期，只处理第二个周期，以达到滤波目的
#define ADC_SampFreq  (Wave_Freq*ADC_Samp_num)	//ADC采样频率，必须是信号频率的ADC_Samp_num倍

#define Ratio 	5.0f		//滤波系数
#define FILTER_NUM 5		// 滤波深度，深度越大，数据跟随性越差，酌情设置，一般设置为3

ALIGN(512) extern dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX];

extern struct Fourier_Data_t
{
		__IO  uint8 Reserve_1[100];
		__O 	uint16 Buff[ADC_Samp_SIZE+3];		//电压数据缓冲区
		__IO  uint8 Reserve_2[100];
	struct
	{
		__IO  uint8 Reserve_3[10];
		__IO  uint16 Result_WithoutFilt;			//电压数据结果(未滑动滤波)
		__IO  uint8 Reserve_4[10];
		__IO	int32 Filt_BUF[FILTER_NUM+3];			//滑动滤波缓冲区
		__IO  uint8 Reserve_5[10];
		__IO	int32 Filt_BUF_SUM;							//滑动滤波累加器
		__IO  uint8 Reserve_6[10];
		__IO	uint8 Filt_point;								//数据指针
  	__IO  uint8 Reserve_7[10];
		
		__IO uint16 Result;			//电压数据滤波后结果
		__IO  uint8 Reserve_8[10];
		__IO uint8  START_Flag;		//通道转换起始标志位
		__IO  uint8 Reserve_9[10];
		__IO ADCRES_enum resolution;	//保存精度
		__IO  uint8 Reserve_10[10];
	}ADCCH_Data[7];
	
	uint8 ADCCH_Save;	//保存此时正在转换的通道
	uint8 Busy_Flag;		//全局忙标志

}Fourier_Data;

void Fourier_Init(ADCCH_enum ch);
void Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution);

__STATIC_INLINE void Fourier_interrupt_Func(void)
{
    if(READ_DMA_FLAG(Fourier_DMACH))
    {
      CLEAR_DMA_FLAG(Fourier_DMACH);
    }
		else
		{
			CLEAR_DMA_FLAG(Fourier_DMACH);
		}
		Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].START_Flag = 0;		//清除起始标志位，以便下次转换
}

//使用__STATIC_INLINE为了将这段函数内嵌到使用该函数的地方，这样可以减少函数调用的时间
__STATIC_INLINE void Fourier_dma_reload(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)//DMA参数重载  重新设置参数无需调用初始化
{
    DMA0->COMMON[0].ENABLECLR = 1<<dmach;	//清除某个信道的DMA允许
    DMA0->COMMON[0].ABORT = 1<<dmach;		//终止某个信道的DMA操作
    FourierDMA_ChannelDescriptors[dmach].xfercfg = ( 0
																						 | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //参数自动重载
																						 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																						 | DMA_CHANNEL_XFERCFG_SETINTA_MASK      
																						 | DMA_CHANNEL_XFERCFG_WIDTH(1)           //宽度16位
																						 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //源地址不自增
																						 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //目的地址自增一个数据宽度
																						 | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA次数
                                            );
    FourierDMA_ChannelDescriptors[dmach].srcEndAddr = (uint32*)SADDR;		//设置源地址
		FourierDMA_ChannelDescriptors[dmach].linkToNextDesc   = 0;
    FourierDMA_ChannelDescriptors[dmach].dstEndAddr = (uint16*)DADDR+count-1;	//设置目的地址
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;
}
#endif
