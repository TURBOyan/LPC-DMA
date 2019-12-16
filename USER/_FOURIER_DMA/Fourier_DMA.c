#include "Fourier_DMA.h"
#include "string.h"

struct Fourier_Data_t Fourier_Data;		//���屣�渵��Ҷ�任���ݵĽṹ��

//ALIGN(512)�����������ֽڶ���
//���帵��ҶDMA�Ĵ���������
ALIGN(512) dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX]={0};

//ʹ��__STATIC_INLINEΪ�˽���κ�����Ƕ��ʹ�øú����ĵط����������Լ��ٺ������õ�ʱ��
__STATIC_INLINE void Fourier_dma_reload(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)//DMA��������  �������ò���������ó�ʼ��
{
    DMA0->COMMON[0].ENABLECLR = 1<<dmach;	//���ĳ���ŵ���DMA����
    DMA0->COMMON[0].ABORT = 1<<dmach;		//��ֹĳ���ŵ���DMA����
    FourierDMA_ChannelDescriptors[dmach].xfercfg = ( 0
																						 | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
																						 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																						 | DMA_CHANNEL_XFERCFG_SETINTA_MASK      
																						 | DMA_CHANNEL_XFERCFG_WIDTH(1)           //���16λ
																						 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
																						 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
																						 | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA����
                                            );
    FourierDMA_ChannelDescriptors[dmach].srcEndAddr = (uint32*)SADDR;		//����Դ��ַ
		FourierDMA_ChannelDescriptors[dmach].linkToNextDesc   = 0;
    FourierDMA_ChannelDescriptors[dmach].dstEndAddr = (uint16*)DADDR+count-1;	//����Ŀ�ĵ�ַ
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;
}

void Fourier_DMA_Init(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)
{
	  SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //��DMAʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //���DMA��λʱ��
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�򿪶�·����ʱ�ӣ�����DMA��������ͨ�������ȴ򿪶�·����ʱ��
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(0); //����DMA��������ͨ�� 0ΪADC0 Sequence A interrupt 1ΪADC0 Sequence B interrupt
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�رն�·����ʱ��
			
    FourierDMA_ChannelDescriptors[dmach].srcEndAddr = (uint32*)SADDR;		//����Դ��ַ
    FourierDMA_ChannelDescriptors[dmach].dstEndAddr = (uint16*)DADDR+count-1;	//����Ŀ�ĵ�ַ
	
    FourierDMA_ChannelDescriptors[dmach].linkToNextDesc   = 0;
		FourierDMA_ChannelDescriptors[dmach].xfercfg=  ( 0
																			 | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
																			 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																			 | DMA_CHANNEL_XFERCFG_SETINTA_MASK      
																			 | DMA_CHANNEL_XFERCFG_WIDTH(1)           //���16λ
																			 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
																			 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
																			 | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA����
																			 );

    DMA0->SRAMBASE = (uint32_t)FourierDMA_ChannelDescriptors;		//���ṹDMA_ChannelDescriptorsӳ����SRAMBASE
    DMA0->CTRL = DMA_CTRL_ENABLE_MASK;		//ʹ��DMA
    DMA0->COMMON[0].ENABLESET = 1<<dmach;	//ʹ��DMAͨ��
		
    
    DMA0->CHANNEL[dmach].CFG = ( 0
                               | DMA_CHANNEL_CFG_HWTRIGEN_MASK			
                               | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 ������
                               | DMA_CHANNEL_CFG_TRIGTYPE_MASK   		//0 :���ش���
                               | DMA_CHANNEL_CFG_TRIGBURST_MASK     //����burst����
                               | DMA_CHANNEL_CFG_BURSTPOWER(0)      //burst����Ϊ4���ֽ�
                               | DMA_CHANNEL_CFG_CHPRIORITY(0)      //���ȼ�����   0Ϊ���
                               );

    DMA0->COMMON[0].SETVALID = 1<<dmach;
    DMA0->COMMON[0].INTENSET = 1<<dmach;

    DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;	//��������
		enable_irq(DMA0_IRQn);		//ʹ��DMA0�ж�
		set_irq_priority(DMA0_IRQn,0);//����DMA�ж�Ϊ������ȼ�
		
		if((DMA_CHANNEL_XFERCFG_CFGVALID_MASK & DMA0->CHANNEL[dmach].XFERCFG) == 0)	//�������ʧ�������
		{
			ASSERT(0);//�������������Ϣ�����Ҫʹ������Ҫ��LPC546XX_config.h�ļ��ڶ���ʹ��
		}
}

void Fourier_ADC_Init(ADCCH_enum ch)
{
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
               //  | ADC_CTRL_BYPASSCAL_MASK        //����У׼  0:����У׼����    1���ر�У׼   ����Ϊ0
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
		
		ADC0->INTEN= (0
								 |ADC_INTEN_SEQA_INTEN_MASK
									);
}

void Fourier_Init(ADCCH_enum ch)
{
	sct_pwm_init(SCT0_OUT9_A30, ADC_SampFreq, SCT0_OUTPUT_CH9_DUTY_MAX*0.5);
	Fourier_ADC_Init(ch);
	Fourier_DMA_Init(Fourier_DMACH,(void*)&ADC0->SEQ_GDAT[0],(void*)&Fourier_Data.Buff[0],ADC_Samp_SIZE);
}

void Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution)
{
		while(Fourier_Data.Busy_Flag);//�ȴ�����ͨ���������
		Fourier_Data.ADCCH_Data[ch].START_Flag = 1;	//��λת����־λ
		Fourier_Data.Busy_Flag = 1;		//��λȫ��æ��־����ֹ��������ͨ��ͬʱ��ʼת�������
		Fourier_Data.ADCCH_Data[ch].resolution = resolution;	//����ֱ���
		Fourier_Data.ADCCH_Save = ch;			//�����ʱ����ת����ͨ������ֹ��������ͨ��ͬʱ��ʼת�������
		memset((void*)&Fourier_Data.Buff,0,sizeof(Fourier_Data.Buff));		//�������
		
	  ADC0->CTRL &= ~ADC_CTRL_RESOL_MASK;
    ADC0->CTRL |= ADC_CTRL_RESOL(resolution);   //���÷ֱ���
		
		ADC0->SEQ_CTRL[0]= 0;		//��������SEQB_ENA�Ĵ�������ֹ������ٴ���
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

		Fourier_dma_reload(Fourier_DMACH, (void*)&ADC0->SEQ_GDAT[0],(void*)&Fourier_Data.Buff[0],ADC_Samp_SIZE);
		
		while(Fourier_Data.ADCCH_Data[ch].START_Flag);		//�ȴ��������
		Fourier_Data.Busy_Flag = 0;	//��ȫ��æ��־
}


		
void Fourier_interrupt_Func(void)
{
		uint16 Data_Buff[ADC_Samp_SIZE]={0};
    if(READ_DMA_FLAG(Fourier_DMACH))
    {
        CLEAR_DMA_FLAG(Fourier_DMACH);
				
				for(uint8 num=0;num < ADC_Samp_SIZE;num++)		//��buff������ת���󻺴�
				{
					Data_Buff[num] = (Fourier_Data.Buff[num]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].resolution)*2);
				}
				
				Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].START_Flag = 0;		//�����ʼ��־λ���Ա��´�ת��
    }
}
