#include "headfile.h"

#define num 0xff

uint32 data[10];

uint32 data1[10]={num,num,num,num,num,num,num,num,num,num}; 

int main(void)
{
	uint8 i,j;
	get_clk();
 	Init_ALL();
	
	DMA_Init_ADC(ADC_CH6_B0 ,DMA_CH0,200000, (void*)&data1[0],(void*)&data[0], 1);
	while(1)
	{
//		Beep_On;
		OLED_P6x8Int(0, 0, data[0], 5);
		OLED_P6x8Int(0, 1, data[1], 5);
		OLED_P6x8Int(0, 2, data[2], 5);
		OLED_P6x8Int(0, 3, data[3], 5);
		OLED_P6x8Int(0, 4, data[4], 5);
		OLED_P6x8Int(0, 5, data[5], 5);
		OLED_P6x8Int(0, 6, data[6], 5);
//    OLED_P6x8Int(0, 0, ((data[0]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2)), -5);
//		OLED_P6x8Int(0, 1, ((data[1]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2)), 5);
//		OLED_P6x8Int(0, 2, ((data[2]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2)), 5);
//		OLED_P6x8Int(0, 3, ((data[3]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2)), 5);
//		OLED_P6x8Int(0, 4, ((data[4]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2)), 5);
//		OLED_P6x8Int(0, 5, ((data[5]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2)), 5);
//		OLED_P6x8Int(0, 6, ((data[6]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2)), 5);

		//OLED_P6x8Int(0, 1, ADC0->SEQ_GDAT[0], -5);
	//	adc_convert(ADC_CH6_B0, ADC_12BIT);
//		Beep_Off;
//		adc_convert(ADC_CH6_B0, ADC_12BIT);
	}
}