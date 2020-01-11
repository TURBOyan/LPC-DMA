#include "Fourier_DMA.h"
#include "string.h"
#include "Selfbuild_oled.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_sct.h"
#include "LPC546XX_pll.h"
#include "LPC546XX_systick.h"

/** Fourier_Data - ���屣�汾�ײ������û����ݵĽṹ�� */
struct Fourier_Data_t Fourier_Data;	
/** FourierDMA_ChannelDescriptors - ���帵��ҶDMA�Ĵ���������,ALIGN(512)->�����ֽڶ��� */
ALIGN(512) dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX]={0};

/*!
 * @addtogroup ������ֵ��ȡ��ʼ��
 * @{
 */
void Fourier_Init(DMACH_enum dmach,ADCCH_enum ch)
{
	/* ----------------------------------------------------------------------------
		 -- �������˲�������ʼ��
		 ---------------------------------------------------------------------------- */
		Fourier_Data.ADCCH_Data[ch].LastP=0.02;
		Fourier_Data.ADCCH_Data[ch].Now_P=0;
		Fourier_Data.ADCCH_Data[ch].out=0;
		Fourier_Data.ADCCH_Data[ch].Kg=0;
		Fourier_Data.ADCCH_Data[ch].Q=0.001;
		Fourier_Data.ADCCH_Data[ch].R=0.2;
	/* ----------------------------------------------------------------------------
		 -- SCT�����źų�ʼ��
		 ---------------------------------------------------------------------------- */
		sct_pwm_init(SCT0_OUT9_A30, ADC_SampFreq, SCT0_OUTPUT_CH9_DUTY_MAX*0.5);
	/* ----------------------------------------------------------------------------
		 -- ADC��ʼ��
		 ---------------------------------------------------------------------------- */
		uint16 temp_div;
		switch(ch)			//�˿ڸ���
		{
				case ADC_CH0_A10:   iocon_init(A10,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
				case ADC_CH1_A11:   iocon_init(A11,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
				case ADC_CH2_A12:   iocon_init(A12,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
				case ADC_CH3_A15:   iocon_init(A15,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
				case ADC_CH4_A16:   iocon_init(A16,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
				case ADC_CH5_A31:   iocon_init(A31,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
				case ADC_CH6_B0 :   iocon_init(B0 ,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
				case ADC_CH11_A23:  iocon_init(A23,ALT0 | NOPULL | ANALOG | FILTEROFF); break;

				default:        ASSERT(0);//ͨ������ �������ʧ��
		}
		
		SYSCON->PDRUNCFGCLR[0] = ( 0
														 | SYSCON_PDRUNCFGCLR_PDEN_ADC0_MASK 
														 | SYSCON_PDRUNCFGCLR_PDEN_VD2_ANA_MASK 
														 | SYSCON_PDRUNCFGCLR_PDEN_VDDA_MASK
														 | SYSCON_PDRUNCFGCLR_PDEN_VREFP_MASK
														 ); //��ADC��Դ
		systick_delay_us(20);                                       //��Ҫ��ʱ
	 
		SYSCON->ADCCLKSEL = SYSCON_ADCCLKSEL_SEL(0x01);             //ѡ��pll_clkΪADCʱ��Դ

		SYSCON->ADCCLKDIV = 0;  
		SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_HALT_MASK;							//����ʱ�ӷ�Ƶǰ����ֹͣ��Ƶ����������ֹ����
		SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_DIV(0);								//����ʱ�ӷ�ƵΪ1��Ƶ
		
		SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_ADC0_MASK;     //��ADCʱ��
		SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK; //�����λADCʱ��
		temp_div = (main_clk_mhz*100/80 + 99)/100;
		ADC0->CTRL = ( 0
								 | ADC_CTRL_CLKDIV(temp_div-1)      //��Ƶ��󲻳���80M
								 | ADC_CTRL_RESOL(0x0)              //Ĭ��12λ�ֱ���
								 | ADC_CTRL_BYPASSCAL_MASK        //����У׼  0:����У׼����    1���ر�У׼   ����Ϊ0
								 | ADC_CTRL_TSAMP(0)                //�������ڣ�����ԼΪ2.5��ADCʱ��
								 );
		ADC0->STARTUP = ADC_STARTUP_ADC_ENA_MASK;           //����ADC
		systick_delay_us(10);                               //��Ҫ��ʱ
		if (!(ADC0->STARTUP & ADC_STARTUP_ADC_ENA_MASK))
		{
				ASSERT(0);//ADCû���ϵ� �������ʧ��
		}
		ADC0->CALIB = ADC_CALIB_CALIB_MASK;                 //ADCУ׼
		while(ADC_CALIB_CALIB_MASK == (ADC0->CALIB & ADC_CALIB_CALIB_MASK));
		
		ADC0->STARTUP |= ADC_STARTUP_ADC_INIT_MASK;         //ADC��ʼ��
		while(ADC_STARTUP_ADC_INIT_MASK == (ADC0->STARTUP & ADC_STARTUP_ADC_INIT_MASK)){};
		
		systick_delay_us(100);
	/* ----------------------------------------------------------------------------
		 -- DMA��ʼ��
		 ---------------------------------------------------------------------------- */
		Fourier_Data.ADCCH_Data[ch].DMA_CH=dmach;	//����DMA����ͨ��
		SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //��DMAʱ��
		SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //���DMA��λʱ��
		
		SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�򿪶�·����ʱ�ӣ�����DMA��������ͨ�������ȴ򿪶�·����ʱ��
		INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(0); //����DMA��������ͨ�� 0ΪADC0 Sequence A interrupt 1ΪADC0 Sequence B interrupt
		SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�رն�·����ʱ��
			
		FourierDMA_ChannelDescriptors[dmach].srcEndAddr = 0;		//����Դ��ַ
		FourierDMA_ChannelDescriptors[dmach].dstEndAddr = 0;	//����Ŀ�ĵ�ַ
		FourierDMA_ChannelDescriptors[dmach].linkToNextDesc= 0;
		FourierDMA_ChannelDescriptors[dmach].xfercfg=0;
		DMA0->SRAMBASE = (uint32_t)FourierDMA_ChannelDescriptors;		//���ṹDMA_ChannelDescriptorsӳ����SRAMBASE
		
		DMA0->CTRL = DMA_CTRL_ENABLE_MASK;		//ʹ��DMA

		DMA0->CHANNEL[dmach].CFG = ( 0
															 | DMA_CHANNEL_CFG_HWTRIGEN_MASK			
															 | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 ������
															 | DMA_CHANNEL_CFG_TRIGTYPE_MASK   		//0 :���ش���
															 | DMA_CHANNEL_CFG_TRIGBURST_MASK     //����burst����
															 | DMA_CHANNEL_CFG_BURSTPOWER(0)      
															 | DMA_CHANNEL_CFG_CHPRIORITY(0)      //���ȼ�����   0Ϊ���
															 );

		DMA0->COMMON[0].SETVALID = 1<<dmach;
		DMA0->COMMON[0].INTENSET = 1<<dmach;
		
		
		DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;	//��������

}

//ʹ��__STATIC_INLINEΪ�˽���κ�����Ƕ��ʹ�øú����ĵط����������Լ��ٺ������õ�ʱ��
__STATIC_INLINE void Fourier_dma_reload(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)//DMA��������  �������ò���������ó�ʼ��
{
    DMA0->COMMON[0].ENABLECLR = 1<<dmach;	//���ĳ���ŵ���DMA����
    DMA0->COMMON[0].ABORT = 1<<dmach;		//��ֹĳ���ŵ���DMA����
    FourierDMA_ChannelDescriptors[dmach].xfercfg = ( 0
																						 //| DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
																						 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																						 | DMA_CHANNEL_XFERCFG_SETINTA_MASK      
																						 | DMA_CHANNEL_XFERCFG_CLRTRIG_MASK				//���������رմ���
																						 | DMA_CHANNEL_XFERCFG_WIDTH(2)           //���32λ
																						 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
																						 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
																						 | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA����
                                            );
    FourierDMA_ChannelDescriptors[dmach].srcEndAddr = (uint32*)SADDR;		//����Դ��ַ
		FourierDMA_ChannelDescriptors[dmach].linkToNextDesc   = 0;
    FourierDMA_ChannelDescriptors[dmach].dstEndAddr = (uint32*)DADDR+count-1;	//����Ŀ�ĵ�ַ
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;
	
		if((DMA_CHANNEL_XFERCFG_CFGVALID_MASK & DMA0->CHANNEL[dmach].XFERCFG) == 0)	//�������ʧ�������
		{
			ASSERT(0);//�������������Ϣ�����Ҫʹ������Ҫ��LPC546XX_config.h�ļ��ڶ���ʹ��
		}
}

__STATIC_INLINE void Fourier_adc_reload(ADCCH_enum ch,ADCRES_enum resolution)
{
	  ADC0->CTRL &= ~ADC_CTRL_RESOL_MASK;
    ADC0->CTRL |= ADC_CTRL_RESOL(resolution);   //���÷ֱ���	
		ADC0->SEQ_CTRL[0]= 0;		//��������SEQB_ENA�Ĵ�������ֹ���������λ
		ADC0->SEQ_CTRL[0]= (0
											|ADC_SEQ_CTRL_CHANNELS(1<<ch)  //����ͨ��
											|ADC_SEQ_CTRL_TRIGGER(0x05) //0x05-��SCT0��Output9��Ϊ�����ź�����
											|ADC_SEQ_CTRL_TRIGPOL_MASK	//ѡ��λ�����ش���
											|ADC_SEQ_CTRL_SYNCBYPASS_MASK
											|ADC_SEQ_CTRL_MODE_MASK
			                |ADC_SEQ_CTRL_SINGLESTEP_MASK
											|ADC_SEQ_CTRL_SEQ_ENA_MASK				//ʹ��ADCת��
										 );
		ADC0->SEQ_CTRL[0] |= ADC_SEQ_CTRL_START_MASK; 	//����ADCת��
}

__STATIC_INLINE void IRQ_Ctrl_ADC_DMA(uint8 ONorOFF)
{
		if(ONorOFF == 0)
		{
			disable_irq(DMA0_IRQn);		//ʧ��DMA0�ж�
			ADC0->INTEN =0;						//ʧ��ADC�ж�
		}
		else
		{
			enable_irq(DMA0_IRQn);		//ʹ��DMA0�ж�
			ADC0->INTEN= (0						//ʹ��ADC�ж�
							 |ADC_INTEN_SEQA_INTEN_MASK
								);
		}
}

void Fourier_DMAReadBuff(ADCCH_enum ch,ADCRES_enum resolution,uint16 *Buff_Out)
{
		static vuint32 Buff[ADC_Samp_SIZE+10];
	
		while(Fourier_Data.Busy_Flag);							//�ȴ�����ͨ��������ɣ���֤���ۺ�ʱCPUֻ��������һ��ͨ��
	
		Fourier_Data.ADCCH_Data[ch].START_Flag = 1;	//��λת����־λ
		Fourier_Data.Busy_Flag = 1;									//��λȫ��æ��־����ֹ��������ͨ��ͬʱ��ʼת�������
		Fourier_Data.ADCCH_Save = ch;								//�����ʱ����ת����ͨ������ֹ��������ͨ��ͬʱ��ʼת�������

		Fourier_adc_reload(ch,resolution);	//ADCת����������
		Fourier_dma_reload(Fourier_Data.ADCCH_Data[ch].DMA_CH, (void*)&ADC0->SEQ_GDAT[0],(void*)&Buff[0],ADC_Samp_SIZE);//DMAת����������
	
		IRQ_Ctrl_ADC_DMA(1);	//���ж�
	
		while(DMA0->CHANNEL[Fourier_Data.ADCCH_Data[ch].DMA_CH].CTLSTAT== 0x00){}	//�ȴ��������
			OLED_P6x8Int(0, 0,Buff[0], 5);
			OLED_P6x8Int(0, 1,Buff[1], 5);
			OLED_P6x8Int(0, 2,Buff[2], 5);
			OLED_P6x8Int(0, 3,Buff[3], 5);
			OLED_P6x8Int(0, 4,Buff[4], 5);
			OLED_P6x8Int(0, 5,Buff[5], 5);
			OLED_P6x8Int(0, 6,Buff[6], 5);
			OLED_P6x8Int(50, 0,Buff[8], 5);
			OLED_P6x8Int(50, 1,Buff[9], 5);
			OLED_P6x8Int(50, 2,Buff[10], 5);
			OLED_P6x8Int(50, 3,Buff[12], 5);
			OLED_P6x8Int(50, 4,Buff[14], 5);
			OLED_P6x8Int(50, 5,Buff[17], 5);
			OLED_P6x8Int(50, 6,Buff[19], 5);
			OLED_P6x8Int(95, 0,Buff[20], 5);
			OLED_P6x8Int(95, 1,Buff[21], 5);
			OLED_P6x8Int(95, 6,DMA0->CHANNEL[Fourier_Data.ADCCH_Data[ch].DMA_CH].CTLSTAT, 2);
//		while(Fourier_Data.ADCCH_Data[ch].START_Flag)		//�ȴ��������
//		{
//			if((DMA0->CHANNEL[Fourier_Data.ADCCH_Data[ch].DMA_CH].CTLSTAT & DMA_CHANNEL_CTLSTAT_TRIG_MASK) ==0)
//			{
//				Fourier_Data.ADCCH_Data[ch].START_Flag=0;
//			}
//				
//		}
	
		IRQ_Ctrl_ADC_DMA(0);//�ر��ж�
	
		for(uint8 num=0;num<ADC_Samp_SIZE;num++)	//�������
		{
				*(Buff_Out+num)=(uint16)((Buff[num]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-resolution)*2));;
		}
		Fourier_Data.Busy_Flag = 0;	//��ȫ��æ��־		
}


/********************************FFT����ʵ�ּ���ر�������ʼ��*****************************************/
//cos_sin_search[n][0]=(2/��������n)*cos(2*Pi*�ź�Ƶ��f*n*��������T)
//cos_sin_search[n][1]=(2/��������n)*sin(2*Pi*�ź�Ƶ��f*n*��������T)
float cos_sin_search[11][2]=		//�����Ҳ��������10����ԭ�ź�Ƶ��20khz
{														//k=
  {  0.2000,  0.0000},			//0
  {  0.1618,  0.1175},			//1
  {  0.0618,  0.1902},			//2
  { -0.0618,  0.1902},			//3
  { -0.1618,  0.1175},		  //4
  {	-0.2000,  0.0000},			//5
  { -0.1618, -0.1175},			//6
  { -0.0618, -0.1902},			//7
  {  0.0618, -0.1902},			//8
  {  0.1618, -0.1175},			//9
  {  0.2000,  0.0000}				//10
};
double FFT(uint16 *adc_data)
{
	int16 An=0,Bn=0;
	uint16 Begin_Point=ADC_Samp_SIZE-ADC_Samp_num;
	for(uint8 k=1;k<=ADC_Samp_num;k++)
	{
	  An+=(int16)adc_data[k+Begin_Point-1] * cos_sin_search[k][0];
	  Bn+=(int16)adc_data[k+Begin_Point-1] * cos_sin_search[k][1];
	}
	return ((An*An+Bn*Bn)==0 ? 0 : sqrt(An*An+Bn*Bn));	//���ػ�����ֵ
}

/**
 *�������˲���
 *@param KFP *kfp �������ṹ�����
 *   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
*/
float kalmanFilter(struct Fourier_Data_t* KALMAN,ADCCH_enum ch,float input)
{
	 //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
	 KALMAN->ADCCH_Data[ch].Now_P = KALMAN->ADCCH_Data[ch].LastP + KALMAN->ADCCH_Data[ch].Q;
	 //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
	 KALMAN->ADCCH_Data[ch].Kg = KALMAN->ADCCH_Data[ch].Now_P / (KALMAN->ADCCH_Data[ch].Now_P + KALMAN->ADCCH_Data[ch].R);
	 //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
	 KALMAN->ADCCH_Data[ch].out = KALMAN->ADCCH_Data[ch].out + KALMAN->ADCCH_Data[ch].Kg * (input -KALMAN->ADCCH_Data[ch].out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
	 //����Э�����: ���ε�ϵͳЭ���ֵ�� kfp->LastP Ϊ��һ������׼����
	 KALMAN->ADCCH_Data[ch].LastP = (1-KALMAN->ADCCH_Data[ch].Kg) * KALMAN->ADCCH_Data[ch].Now_P;
	 return KALMAN->ADCCH_Data[ch].out;
}

uint16 Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution)
{
		uint16 Data_Buff[ADC_Samp_SIZE]={0};
		float Result_WithoutFilt;
		uint32 Result_WithFilt;
		//��2�����ڵĲ����ź������Data_Buff
		Fourier_DMAReadBuff(ch,resolution,Data_Buff);		
		//���������ֵ���������˲�

		Result_WithoutFilt=(float)FFT(Data_Buff);
		Result_WithoutFilt=Result_WithoutFilt<=0?0:Result_WithoutFilt;
		
		//Result_WithFilt = (uint32)kalmanFilter(&Fourier_Data,ch,Result_WithoutFilt);	
		return Result_WithoutFilt;
}