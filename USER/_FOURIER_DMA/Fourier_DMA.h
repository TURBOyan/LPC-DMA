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

#define Fourier_DMACH DMA_CH1									//����DMA�����ͨ��
#define Wave_Freq 		20000										//��Ҫ������ź�Ƶ��
#define ADC_Samp_num  10 											//����һ�����ڲ�������Ŀ��һ��ȡ10��
#define ADC_Samp_SIZE 2*ADC_Samp_num					//������ȣ�����ΪADC_Samp_num�ı�����һ��ȡ�������ڣ�ֻ����ڶ������ڣ��Դﵽ�˲�Ŀ��
#define ADC_SampFreq  Wave_Freq*ADC_Samp_num	//ADC����Ƶ�ʣ��������ź�Ƶ�ʵ�ADC_Samp_num��

#define Ratio 	15.0f		//�˲�ϵ��
#define FILTER_NUM 3		// �˲���ȣ����Խ�����ݸ�����Խ��������ã�һ������Ϊ3

extern struct Fourier_Data_t
{
		__O uint16 Buff[ADC_Samp_SIZE];		//��ѹ���ݻ�����
	
	struct
	{
		__IO  uint16 Result_WithoutFilt;			//��ѹ���ݽ��(δ�����˲�)
		__IO	int32 Filt_BUF[FILTER_NUM];			//�����˲�������
		__IO	int64 Filt_BUF_SUM;							//�����˲��ۼ���
		__IO	uint8 Filt_point;								//����ָ��
		
		__IO uint16 Result;			//��ѹ�����˲�����
		
		__IO uint8  START_Flag;		//ͨ��ת����ʼ��־λ
		__IO ADCRES_enum resolution;	//���澫��
	}ADCCH_Data[6];
	
	uint8 ADCCH_Save;	//�����ʱ����ת����ͨ��
	uint8 Busy_Flag;		//ȫ��æ��־
	
}Fourier_Data;

void Fourier_Init(ADCCH_enum ch);
void Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution);
void Fourier_interrupt_Func(void);
#endif
