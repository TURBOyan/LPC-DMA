#include "headfile.h"

int main(void)
{
	uint32 i,j;
	get_clk();
 	Init_ALL();
	
	Fourier_Init(ADC_CH6_B0);
	Fourier_Init(ADC_CH11_A23);
	while(1)
	{
		Fourier_Once(ADC_CH6_B0  ,ADC_8BIT);
		OLED_P6x8Int(0, 0,Fourier_Data.ADCCH_Data[ADC_CH6_B0].Result, 5);
		Fourier_Once(ADC_CH11_A23,ADC_8BIT);
		OLED_P6x8Int(0, 1,Fourier_Data.ADCCH_Data[ADC_CH11_A23].Result, 5);
	}
}