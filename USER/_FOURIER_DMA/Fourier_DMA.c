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
struct kalman_Data_t kalman_Data[14];
/** FourierDMA_ChannelDescriptors - ���帵��ҶDMA�Ĵ���������,ALIGN(512)->�����ֽڶ��� */
ALIGN(512) dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX]={0};

/**
 *������ֵ��ȡ��ʼ��
 *@param DMACH_enum dmach ���ò�ͬADC�˿����ݴ����DMAͨ����,��ΧDMA_CH0~DMA_CH29
 *   		 ADCCH_enum ch 		����ADת���˿�
 *@return ��
 *@example Fourier_Init(DMA_CH0,ADC_CH6_B0);  ����B0�ڵĻ�����ֵ��ȡ��ʽ�����ݴ�����DMA_CH0ͨ��
*/
void Fourier_Init(DMACH_enum dmach,ADCCH_enum ch)
{
	/* ----------------------------------------------------------------------------
		 -- �������˲�������ʼ��
		 ---------------------------------------------------------------------------- */
		kalman_Data[ch].LastP=0.02;
		kalman_Data[ch].Now_P=0;
		kalman_Data[ch].out=0;
		kalman_Data[ch].Kg=0;
		kalman_Data[ch].Q=0.001;
		kalman_Data[ch].R=0.2;
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
		Fourier_Data.DMA_CH[ch]=dmach;	//����DMA����ͨ��
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
		
		enable_irq(DMA0_IRQn);		//ʹ��DMA0�ж�
		ADC0->INTEN= (0						//ʹ��ADC�ж�
						 |ADC_INTEN_SEQA_INTEN_MASK
							);

}

/**
 *DMA�������أ��ڲ�������������ã�
 *ʹ��__STATIC_INLINEΪ�˽���κ�����Ƕ��ʹ�øú����ĵط����������Լ��ٺ������õ�ʱ��
 *@param DMACH_enum dmach ���ô����DMAͨ����,��ΧDMA_CH0~DMA_CH29
 *   		 void *SADDR 			DMA����Դ��ַ
				 void *DADDR			DMA����Ŀ�ĵ�ַ
				 uint16 count			�������
 *@return ��
 *@example Fourier_dma_reload(DMA_CH0, (void*)&ADC0->SEQ_GDAT[0],(void*)&Buff[0],20); ��ADC����Ĵ����ڵ����ݴ�����Buff�����ڣ�ѭ��20��
*/
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

/**
 *ADC�������أ��ڲ�������������ã�
 *ʹ��__STATIC_INLINEΪ�˽���κ�����Ƕ��ʹ�øú����ĵط����������Լ��ٺ������õ�ʱ��
 *@param ADCCH_enum ch 		����ADת���˿�
 *   		 ADCRES_enum resolution ����ADת��λ������Χ6λ��8λ��10λ��12λ
 *@return ��
 *@example Fourier_adc_reload(ADC_CH6_B0,ADC_12BIT);����B0�ڵ�ADת��������������Ϊ12λADC
*/
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

/**
 *�жϿ��ƣ��ڲ�������������ã�
 *ʹ��__STATIC_INLINEΪ�˽���κ�����Ƕ��ʹ�øú����ĵط����������Լ��ٺ������õ�ʱ��
 *@param uint8 ONorOFF 	�жϿ��أ�1Ϊ����0Ϊ�أ�����DMA��ADC�ж�
 *@return ��
 *@example IRQ_Ctrl_ADC_DMA(1); ��ADC��DMA�ж�
*/
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

/**
 *��ȡN�˵�ѹ������ָ��������
*@param ADCCH_enum ch 		����ADת���˿�	
				ADCRES_enum resolution ����ADת��λ������Χ6λ��8λ��10λ��12λ
				uint16 *Buff_Out  ����N���������ѹ���������ָ��
 *@return ��
 *@example Fourier_DMAReadBuff(ADC_CH6_B0,ADC_12BIT,Data_Buff); ��B0�ڵ�N���������12λ��ѹ���ݴ���Data_Buff������
*/
uint8 START_FLAG=0;		//��ʼת����־
void Fourier_DMAReadBuff(ADCCH_enum ch,ADCRES_enum resolution,uint16 *Buff_Out)
{
		static vuint32 Buff[ADC_Samp_SIZE+10];
		uint32 Buff2[ADC_Samp_SIZE+10];
	
		while(Fourier_Data.Busy_Flag);							//�ȴ�����ͨ��������ɣ���֤���ۺ�ʱCPUֻ��������һ��ͨ��
		START_FLAG=1;
		Fourier_Data.Busy_Flag = 1;									//��λȫ��æ��־����ֹ��������ͨ��ͬʱ��ʼת�������
		Fourier_Data.ADCCH_Save = ch;								//�����ʱ����ת����ͨ������ֹ��������ͨ��ͬʱ��ʼת�������

		Fourier_adc_reload(ch,resolution);	//ADC��������
		Fourier_dma_reload(Fourier_Data.DMA_CH[ch], (void*)&ADC0->SEQ_GDAT[0],(void*)&Buff[0],ADC_Samp_SIZE);//DMA��������
	
		IRQ_Ctrl_ADC_DMA(1);	//���ж�
		while(START_FLAG){}		//�ȴ�ת�����
		IRQ_Ctrl_ADC_DMA(0);	//�ر��ж�
		
		uint8 Carry=(ADC_SEQ_GDAT_RESULT_SHIFT+(3-resolution)*2);
		for(uint8 num=0;num<ADC_Samp_SIZE;num++)	//�������
		{
				Buff2[num]=Buff[num];
				*(Buff_Out+num)=(uint16)((Buff2[num]&ADC_SEQ_GDAT_RESULT_MASK)>>Carry);
		}
		Fourier_Data.Busy_Flag = 0;	//��ȫ��æ��־		
}

/* ------------------------------------------------------------------------------------------------------------------------
   -- ���»�����ֵ���㺯����оƬƽ̨�޹أ���ֱ����ֲ��ֻ�轫N��������ĵ�ѹ���ݴ��뼴��
   -------------------------------------------------------------------------------------------------------------------------- */
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

/**
 *FFT������ֵ��ȡ����
*@param uint16 *adc_data	���뱣����N��������ĵ�ѹ����
 *@return ������ֵ���������ĵ�ѹ����Ϊ6λ�� �����ػ�����ֵ��ΧΪ0-31
 *								 �������ĵ�ѹ����Ϊ8λ�� �����ػ�����ֵ��ΧΪ0-127
 *								 �������ĵ�ѹ����Ϊ10λ�ģ����ػ�����ֵ��ΧΪ0-512
 *								 �������ĵ�ѹ����Ϊ12λ�ģ����ػ�����ֵ��ΧΪ0-2048
 *@example FFT(Data_Buff) ����Data_Buff�����ڵĻ�����ֵ
*/
double FFT(uint16 *adc_data)
{
	int16 An=0,Bn=0;
	uint16 Begin_Point=ADC_Samp_SIZE-ADC_Samp_num-5;
	for(uint8 k=1;k<=ADC_Samp_num;k++)
	{
	  An+=(int16)adc_data[k+Begin_Point-1] * cos_sin_search[k][0];
	  Bn+=(int16)adc_data[k+Begin_Point-1] * cos_sin_search[k][1];
	}
	return ((An*An+Bn*Bn)==0 ? 0 : sqrt(An*An+Bn*Bn));	//���ػ�����ֵ
}

/**
 *�������˲���
 *@param struct kalman_Data_t* KALMAN �������ṹ�����
 *   		 float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
*/
float kalmanFilter(struct kalman_Data_t* KALMAN,float input)
{
	 //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
	 KALMAN->Now_P = KALMAN->LastP + KALMAN->Q;
	 //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
	 KALMAN->Kg = KALMAN->Now_P / (KALMAN->Now_P + KALMAN->R);
	 //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
	 KALMAN->out = KALMAN->out + KALMAN->Kg * (input -KALMAN->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
	 //����Э�����: ���ε�ϵͳЭ���ֵ�� kfp->LastP Ϊ��һ������׼����
	 KALMAN->LastP = (1-KALMAN->Kg) * KALMAN->Now_P;
	 return KALMAN->out;
}
/**
 *����һ��ͨ����һ�λ�����ֵ
*@param ADCCH_enum ch 		����ADת���˿�	
				ADCRES_enum resolution ����ADת��λ������Χ6λ��8λ��10λ��12λ
 *@return �������˲��Ժ�Ļ�����ֵ
 *@example FFT(Data_Buff) ����Data_Buff�����ڵĻ�����ֵ
*/
uint16 Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution)
{
		uint16 Data_Buff[ADC_Samp_SIZE]={0};
		float Result_WithoutFilt;
		uint16 Result_WithFilt;
		//��2�����ڵĲ����ź������Data_Buff
		Fourier_DMAReadBuff(ch,resolution,Data_Buff);		
		//���������ֵ���������˲�
		
		Result_WithoutFilt=(float)FFT(Data_Buff);		//��ȡ������ֵԭʼֵ
		Result_WithoutFilt=Result_WithoutFilt<=0?0:Result_WithoutFilt;	//�޷�
		
		Result_WithFilt = (uint16)kalmanFilter(&kalman_Data[ch],Result_WithoutFilt);			//�������˲��󷵻ػ�����ֵ
		return Result_WithFilt;
}