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
#define Wave_Freq 		(20000)										//��Ҫ������ź�Ƶ��,��Ҫ�޸ģ�����Ҫͬʱ�޸����Ǻ�����
#define ADC_Samp_num  (10)											//����һ�����ڲ�������Ŀ��һ��ȡ5��10������Ŀ��ᵼ���������������Ῠס�򽵵Ͳ�������Ŀ����Ҫ�޸ģ�����Ҫͬʱ�޸����Ǻ�����
#define ADC_Samp_SIZE (2*ADC_Samp_num)					//������ȣ�����ΪADC_Samp_num�ı�����һ��ȡ�������ڣ�ֻ����ڶ������ڣ��Դﵽ�˲�Ŀ��
#define ADC_SampFreq  (Wave_Freq*ADC_Samp_num)	//ADC����Ƶ�ʣ��������ź�Ƶ�ʵ�ADC_Samp_num��

#define Ratio 	5.0f		//�˲�ϵ��
#define FILTER_NUM 5		// �˲���ȣ����Խ�����ݸ�����Խ��������ã�һ������Ϊ3

extern struct Fourier_Data_t
{
		__IO  uint8 Reserve_1[100];
		__O 	uint16 Buff[ADC_Samp_SIZE+3];		//��ѹ���ݻ�����
		__IO  uint8 Reserve_2[100];
	struct
	{
		__IO  uint8 Reserve_3[10];
		__IO  uint16 Result_WithoutFilt;			//��ѹ���ݽ��(δ�����˲�)
		__IO  uint8 Reserve_4[10];
		__IO	int32 Filt_BUF[FILTER_NUM+3];			//�����˲�������
		__IO  uint8 Reserve_5[10];
		__IO	int32 Filt_BUF_SUM;							//�����˲��ۼ���
		__IO  uint8 Reserve_6[10];
		__IO	uint8 Filt_point;								//����ָ��
  	__IO  uint8 Reserve_7[10];
		
		__IO uint16 Result;			//��ѹ�����˲�����
		__IO  uint8 Reserve_8[10];
		__IO uint8  START_Flag;		//ͨ��ת����ʼ��־λ
		__IO  uint8 Reserve_9[10];
		__IO ADCRES_enum resolution;	//���澫��
		__IO  uint8 Reserve_10[10];
	}ADCCH_Data[7];
	
	uint8 ADCCH_Save;	//�����ʱ����ת����ͨ��
	uint8 Busy_Flag;		//ȫ��æ��־

}Fourier_Data;

void Fourier_Init(ADCCH_enum ch);
void Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution);
void Fourier_interrupt_Func(void);
#endif
