#include "headfile.h"
#include "math.h"

extern int16 cos_sin_search[11][2];

int main(void)
{
	uint32 i,j;
	get_clk();
 	Init_ALL();
	
	Fourier_Init(ADC_CH6_B0);
	Fourier_Init(ADC_CH11_A23);
	Fourier_Init(ADC_CH0_A10);
	Fourier_Init(ADC_CH3_A15);
	Fourier_Init(ADC_CH4_A16);
	Fourier_Init(ADC_CH5_A31);
	
//	for(uint8 num=5;num<11;num++)
//	{
//		OLED_P6x8Int(0, num-5,cos_sin_search[num][0], -3);
//		OLED_P6x8Int(50, num-5,cos_sin_search[num][1], -3);
//	}
//	while(1);
	while(1)
	{
		
//		gpio_set(Beep,1);
		Fourier_Once(ADC_CH6_B0  ,ADC_12BIT);
//		gpio_set(Beep,0);
		Fourier_Once(ADC_CH11_A23  ,ADC_12BIT);
		Fourier_Once(ADC_CH0_A10  ,ADC_12BIT);
		Fourier_Once(ADC_CH3_A15  ,ADC_12BIT);
		Fourier_Once(ADC_CH4_A16  ,ADC_12BIT);
		Fourier_Once(ADC_CH5_A31  ,ADC_12BIT);
//		if(Fourier_Data.ADCCH_Data[ADC_CH6_B0].Result_WithoutFilt > 1000)
//			gpio_set(Beep,1);
		OLED_P6x8Int(0, 0,Fourier_Data.ADCCH_Data[ADC_CH6_B0].Result_WithoutFilt, 5);
		OLED_P6x8Int(0, 1,Fourier_Data.ADCCH_Data[ADC_CH11_A23].Result_WithoutFilt, 5);
		OLED_P6x8Int(0, 2,Fourier_Data.ADCCH_Data[ADC_CH0_A10].Result_WithoutFilt, 5);
		OLED_P6x8Int(0, 3,Fourier_Data.ADCCH_Data[ADC_CH3_A15].Result_WithoutFilt, 5);
		OLED_P6x8Int(0, 4,Fourier_Data.ADCCH_Data[ADC_CH4_A16].Result_WithoutFilt, 5);
		OLED_P6x8Int(0, 5,Fourier_Data.ADCCH_Data[ADC_CH5_A31].Result_WithoutFilt, 5);
		
	//	OLED_P6x8Flo(0, 1,cos_sin_search[1][0], -3);
	//	gpio_toggle(Beep);
	}
}

