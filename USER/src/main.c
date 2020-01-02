#include "headfile.h"
#include "math.h"

extern int16 cos_sin_search[11][2];

int main(void)
{
	get_clk();
 	Init_ALL();
	
//	Fourier_Init(ADC_CH6_B0);
//	Fourier_Init(ADC_CH11_A23);
//	Fourier_Init(ADC_CH0_A10);
//	Fourier_Init(ADC_CH3_A15);
//	Fourier_Init(ADC_CH4_A16);
//	Fourier_Init(ADC_CH5_A31);
	adc_init(ADC_CH6_B0);
	adc_init(ADC_CH11_A23);
	adc_init(ADC_CH0_A10);
	adc_init(ADC_CH3_A15);
	adc_init(ADC_CH4_A16);
	adc_init(ADC_CH5_A31);
	while(1)
	{
//		Fourier_Once(ADC_CH6_B0   ,ADC_8BIT);
//		Fourier_Once(ADC_CH11_A23 ,ADC_8BIT);
//		Fourier_Once(ADC_CH0_A10  ,ADC_8BIT);
//		Fourier_Once(ADC_CH3_A15  ,ADC_8BIT);
//		Fourier_Once(ADC_CH4_A16  ,ADC_8BIT);
//		Fourier_Once(ADC_CH5_A31  ,ADC_8BIT);
//		OLED_P6x8Int(0, 0,Fourier_Data.ADCCH_Data[ADC_CH6_B0].Result, 5);
//		OLED_P6x8Int(0, 1,Fourier_Data.ADCCH_Data[ADC_CH11_A23].Result, 5);
//		OLED_P6x8Int(0, 2,Fourier_Data.ADCCH_Data[ADC_CH0_A10].Result, 5);
//		OLED_P6x8Int(0, 3,Fourier_Data.ADCCH_Data[ADC_CH3_A15].Result, 5);
//		OLED_P6x8Int(0, 4,Fourier_Data.ADCCH_Data[ADC_CH4_A16].Result, 5);
//		OLED_P6x8Int(0, 5,Fourier_Data.ADCCH_Data[ADC_CH5_A31].Result, 5);
		OLED_P6x8Int(0, 0,adc_convert(ADC_CH6_B0, ADC_8BIT), 5);
		OLED_P6x8Int(0, 1,adc_convert(ADC_CH11_A23, ADC_8BIT), 5);
		OLED_P6x8Int(0, 2,adc_convert(ADC_CH0_A10, ADC_8BIT), 5);
		OLED_P6x8Int(0, 3,adc_convert(ADC_CH3_A15, ADC_8BIT), 5);
		OLED_P6x8Int(0, 4,adc_convert(ADC_CH4_A16, ADC_8BIT), 5);
		OLED_P6x8Int(0, 5,adc_convert(ADC_CH5_A31, ADC_8BIT), 5);
	}
}

