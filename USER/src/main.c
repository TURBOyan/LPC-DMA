#include "headfile.h"

#define num 0xff

uint16 data[32];

uint32 data1[10]={num,num,num,num,num,num,num,num,num,num}; 

int main(void)
{
	uint32 i,j;
	get_clk();
 	Init_ALL();
	
	DMA_Init_ADC(ADC_CH6_B0 ,DMA_CH0,200000, (void*)ADC0->SEQ_GDAT[0],(void*)&data[0], 5);
	while(1)
	{
//		Beep_On;
		OLED_P6x8Int(0, 0, (data[0]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2), 5);
		OLED_P6x8Int(0, 1, (data[4]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2), 5);
		OLED_P6x8Int(0, 2, (data[7]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2), 5);
		OLED_P6x8Int(0, 3, (data[9]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2), 5);
		OLED_P6x8Int(0, 4, (data[14]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2), 5);
		OLED_P6x8Int(0, 5, (data[16]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2), 5);
		OLED_P6x8Int(0, 6, (data[19]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2), 5);

		OLED_P6x8Int(50, 0, ADC0->SEQ_GDAT[0]>>31, -5);
		j=(ADC0->SEQ_GDAT[0]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-ADC_12BIT)*2);
		OLED_P6x8Int(50, 1, ADC0->SEQ_GDAT[0], -5);
		OLED_P6x8Int(50, 2, j, -5);

//	  adc_convert(ADC_CH6_B0, ADC_12BIT);
//		Beep_Off;
//		adc_convert(ADC_CH6_B0, ADC_12BIT);
	}
}