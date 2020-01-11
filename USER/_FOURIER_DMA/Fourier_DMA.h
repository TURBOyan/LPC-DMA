#ifndef _FOURIER_DMA_H
#define _FOURIER_DMA_H

#include "common.h"
#include "LPC546XX_dma.h"
#include "LPC546XX_adc.h"

#define Wave_Freq 		(20000)										//��Ҫ������ź�Ƶ��,��Ҫ�޸ģ�����Ҫͬʱ�޸����Ǻ�����
#define ADC_Samp_num  (10)											//����һ�����ڲ�������Ŀ��һ��ȡ5��10������Ŀ��ᵼ���������������Ῠס�򽵵Ͳ�������Ŀ����Ҫ�޸ģ�����Ҫͬʱ�޸����Ǻ�����
#define ADC_Samp_SIZE (2*ADC_Samp_num)					//������ȣ�����ΪADC_Samp_num�ı�����һ��ȡ�������ڣ�ֻ����ڶ������ڣ��Դﵽ�˲�Ŀ��
#define ADC_SampFreq  (Wave_Freq*ADC_Samp_num)	//ADC����Ƶ�ʣ��������ź�Ƶ�ʵ�ADC_Samp_num��

ALIGN(512) extern dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX];

extern struct Fourier_Data_t
{
	struct
	{
		uint32 Result;			//��ѹ�����˲�����
		uint8 Reserve0[10];		//�����������ݸ��룬������־λ����ֹ����������µı�־λ����
		DMACH_enum DMA_CH;
		uint8  START_Flag;		//ͨ��ת����ʼ��־λ,�����־λ�ǳ���Ҫ��������洢���������κμĴ����������ı�˱�־λ�����³���ס
		uint8 Reserve1[10];  //�����������ݸ��룬������־λ����ֹ����������µı�־λ����
		
    float LastP;//�ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P;//��ǰ����Э���� ��ʼ��ֵΪ0
    float out;//�������˲������ ��ʼ��ֵΪ0
    float Kg;//���������� ��ʼ��ֵΪ0
    float Q;//��������Э���� ��ʼ��ֵΪ0.001��ֵֻҪ��Ϊ0�����ԣ�����ܿ�����
    float R;//�۲�����Э���� ��ʼ��ֵΪ0.2,ֵԽ�����ݸ�����Խ�������λ�ͺ�ֵԽС��ʹ�˲�Ч�����

	}ADCCH_Data[14];
	
	ADCCH_enum ADCCH_Save;	//�����ʱ����ת����ͨ��
	uint8 Busy_Flag;		//ȫ��æ��־
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
		Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].START_Flag = 0;		//�����ʼ��־λ���Ա��´�ת��
}


#endif
