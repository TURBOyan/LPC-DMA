#include "headfile.h"
#include "math.h"

extern int16 cos_sin_search[11][2];

int main(void)
{
	get_clk();
 	Init_ALL();
	gpio_init(Beep,GPO,0,PULLUP);
	gpio_init(Button_Up,GPI,0,PULLUP);
	Fourier_Init(DMA_CH0,ADC_CH6_B0);
	Fourier_Init(DMA_CH1,ADC_CH11_A23);
	Fourier_Init(DMA_CH2,ADC_CH0_A10);
	Fourier_Init(DMA_CH3,ADC_CH3_A15);
	Fourier_Init(DMA_CH4,ADC_CH4_A16);
	Fourier_Init(DMA_CH5,ADC_CH5_A31);
	while(1)
	{
//		gpio_set(Beep,1);
//		Fourier_Once(ADC_CH6_B0   ,ADC_12BIT);
//		gpio_set(Beep,0);
		OLED_P6x8Int(0, 0,Fourier_Once(ADC_CH6_B0    ,ADC_12BIT), 5);
		OLED_P6x8Int(0, 1,Fourier_Once(ADC_CH0_A10   ,ADC_12BIT), 5);
		OLED_P6x8Int(0, 2,Fourier_Once(ADC_CH3_A15   ,ADC_12BIT), 5);
		OLED_P6x8Int(0, 3,Fourier_Once(ADC_CH4_A16   ,ADC_12BIT), 5);
		OLED_P6x8Int(0, 4,Fourier_Once(ADC_CH5_A31   ,ADC_12BIT), 5);
		OLED_P6x8Int(0, 5,Fourier_Once(ADC_CH11_A23  ,ADC_12BIT), 5);
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
//		
//		OLED_P6x8Int(50, 0,Fourier_Data.ADCCH_Data[ADC_CH6_B0].Result_WithoutFilt, 5);
//		OLED_P6x8Int(50, 1,Fourier_Data.ADCCH_Data[ADC_CH11_A23].Result_WithoutFilt, 5);
//		OLED_P6x8Int(50, 2,Fourier_Data.ADCCH_Data[ADC_CH0_A10].Result_WithoutFilt, 5);
//		OLED_P6x8Int(50, 3,Fourier_Data.ADCCH_Data[ADC_CH3_A15].Result_WithoutFilt, 5);
//		OLED_P6x8Int(50, 4,Fourier_Data.ADCCH_Data[ADC_CH4_A16].Result_WithoutFilt, 5);
//		OLED_P6x8Int(50, 5,Fourier_Data.ADCCH_Data[ADC_CH5_A31].Result_WithoutFilt, 5);
	}
}

