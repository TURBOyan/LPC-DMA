#include "headfile.h"

#define num 0xff

uint16 data[30];

uint32 data1[10]={num,num,num,num,num,num,num,num,num,num}; 

int main(void)
{
	uint32 i,j;
	get_clk();
 	Init_ALL();
	Fourier_Init(ADC_CH6_B0);
	Fourier_Init(ADC_CH11_A23);
	while(1)
	{
		Fourier_Once(ADC_CH6_B0,ADC_8BIT);
		OLED_P6x8Int(0, 0, (Fourier_Data.Buff[0]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-Fourier_Data.ADCCH_Data[ADC_CH11_A23].resolution)*2), 5);
		Fourier_Once(ADC_CH11_A23,ADC_8BIT);
		OLED_P6x8Int(0, 1, (Fourier_Data.Buff[0]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-Fourier_Data.ADCCH_Data[ADC_CH11_A23].resolution)*2), 5);
	}
}