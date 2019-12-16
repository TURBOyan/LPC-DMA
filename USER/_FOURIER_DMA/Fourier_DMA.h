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

#define Fourier_DMACH DMA_CH1		//����DMA�����ͨ��
#define ADC_Samp_SIZE 20				//������ȣ�����Ϊ10�ı���
#define ADC_SampFreq  200000		//ADC����Ƶ��


extern struct Fourier_Data_t
{
		__O uint16 Buff[ADC_Samp_SIZE];		//��ѹ���ݻ�����
	
	struct
	{
		__IO uint16 Result;			//��ѹ���ݽ��
		__IO uint8  START_Flag;		//ͨ��ת����ʼ��־λ
		__IO ADCRES_enum resolution;	//���澫��
	}ADCCH_Data[6];
	
	 uint8 ADCCH_Save;	//�����ʱ����ת����ͨ��
	 uint8 Busy_Flag;		//ȫ��æ��־
}Fourier_Data;

void Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution);
void Fourier_interrupt_Func(void);
#endif
