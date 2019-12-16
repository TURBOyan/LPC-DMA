/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		DMA
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/


#include "common.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_pint.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_sct.h"
#include "LPC546XX_dma.h"
#include "LPC546XX_pll.h"
#include "LPC546XX_systick.h"
#include "LPC546XX_adc.h"


ALIGN(512) dma_descriptor_t s_dma_descriptor_table[DMA_CHMAX] = {0};//DMAͨ��������

//-------------------------------------------------------------------------------------------------------------------
//  @brief      DMA��ʼ��
//  @param      dmach       DMAͨ��
//  @param      *SADDR      Դ��ַ
//  @param      *DADDR      Ŀ�ĵ�ַ
//  @param      count       DMA�������
//  @return     void
//  Sample usage:           dam_init(DMA_CH0, (void *)&GPIO_PIN(0,0), (void *)&image[0][0], 188);     //��ʼ��DMA  ͨ��0   Դ��ַΪA0-A7  Ŀ�ĵ�ַΪimage������׵�ַ   �����ֽ���Ϊ188��
//-------------------------------------------------------------------------------------------------------------------
void dam_init(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)
{
    uint8  n;
    uint32 temp_pin;

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //��DMAʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //���DMA��λʱ��
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�򿪶�·����ʱ��
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(2); //����DMA��������ͨ�� ΪSCT request0
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�رն�·����ʱ��
    
    temp_pin = ((uint32)SADDR - (uint32)&GPIO_PIN(0,0)) * 8;
    n = 8;
    while(n--)
    {
        gpio_init((PIN_enum)(temp_pin+n),GPI,0,PULLUP | FILTEROFF);
    }

    DMA0->SRAMBASE = (uint32_t)s_dma_descriptor_table;
    DMA0->CTRL = DMA_CTRL_ENABLE_MASK;
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    
    DMA0->CHANNEL[dmach].CFG = ( 0
                               | DMA_CHANNEL_CFG_HWTRIGEN_MASK
                               | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 ������
                                //| DMA_CHANNEL_CFG_TRIGTYPE_MASK   //0 :���ش���
                               | DMA_CHANNEL_CFG_TRIGBURST_MASK     //����burst����
                               | DMA_CHANNEL_CFG_BURSTPOWER(0)      //burst����Ϊһ���ֽ�
                               | DMA_CHANNEL_CFG_CHPRIORITY(0)      //���ȼ�����   0Ϊ���
                               );
    
    DMA0->COMMON[0].SETVALID = 1<<dmach;
    DMA0->COMMON[0].INTENSET = 1<<dmach;
    
    s_dma_descriptor_table[dmach].xfercfg = ( 0
                                   //| DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
                                   | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                   | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                   | DMA_CHANNEL_XFERCFG_WIDTH(0)           //���8λ
                                   | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
                                   | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
                                   | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA����
                                   );
    
    s_dma_descriptor_table[dmach].srcEndAddr = SADDR;
    s_dma_descriptor_table[dmach].dstEndAddr = (void *)((uint32)DADDR + count - 1);
    s_dma_descriptor_table[dmach].linkToNextDesc = 0;
    
    DMA0->CHANNEL[dmach].XFERCFG = s_dma_descriptor_table[dmach].xfercfg;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      DMA ���Ӵ��� ��ʼ��
//  @param      dmach       DMAͨ��
//  @param      *SADDR      Դ��ַ
//  @param      *DADDR      Ŀ�ĵ�ַ
//  @param      count       DMA�������
//  @return     void
//  Sample usage:           dam_init(DMA_CH0, (void *)&GPIO_PIN(0,0), (void *)&image[0][0], 188);     //��ʼ��DMA  ͨ��0   Դ��ַΪA0-A7  Ŀ�ĵ�ַΪimage������׵�ַ   �����ֽ���Ϊ188��
//  @note                   ʹ�����Ӵ������ʵ�� һ���Դ��䳬��1024 ����ռ������DMAͨ���� ͨ��������
//-------------------------------------------------------------------------------------------------------------------
void dam_init_linked(DMACH_enum dmach, void *SADDR, void *DADDR, uint32 count)
{
    uint8  n;
    uint32 temp_pin;

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //��DMAʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //���DMA��λʱ��
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�򿪶�·����ʱ��
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(2); //����DMA��������ͨ�� ΪSCT request0
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�رն�·����ʱ��
    
    temp_pin = ((uint32)SADDR - (uint32)&GPIO_PIN(0,0)) * 8;
    n = 8;
    while(n--)
    {
        gpio_init((PIN_enum)(temp_pin+n),GPI,0,PULLUP | FILTEROFF);
    }

    DMA0->SRAMBASE = (uint32_t)s_dma_descriptor_table;
    DMA0->CTRL = DMA_CTRL_ENABLE_MASK;
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    
    DMA0->CHANNEL[dmach].CFG = ( 0
                               | DMA_CHANNEL_CFG_HWTRIGEN_MASK
                               | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 ������
                                //| DMA_CHANNEL_CFG_TRIGTYPE_MASK   //0 :���ش���
                               | DMA_CHANNEL_CFG_TRIGBURST_MASK     //����burst����
                               | DMA_CHANNEL_CFG_BURSTPOWER(0)      //burst����Ϊһ���ֽ�
                               | DMA_CHANNEL_CFG_CHPRIORITY(0)      //���ȼ�����   0Ϊ���
                               );
    
    DMA0->COMMON[0].SETVALID = 1<<dmach;
    DMA0->COMMON[0].INTENSET = 1<<dmach;
    
    n = 0;
    while((n+1)<<10 < count)//ʣ����������1024
    {
        s_dma_descriptor_table[n].xfercfg = ( 0
                                            | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
                                            | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                            | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                            | DMA_CHANNEL_XFERCFG_WIDTH(0)           //���8λ
                                            | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
                                            | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
                                            | DMA_CHANNEL_XFERCFG_XFERCOUNT(1024-1)  //DMA����
                                            );
        s_dma_descriptor_table[n].srcEndAddr = SADDR;
        s_dma_descriptor_table[n].dstEndAddr = (void *)((uint32)DADDR + ((n+1)<<10) - 1);
        s_dma_descriptor_table[n].linkToNextDesc = (void *)(&s_dma_descriptor_table[n+1]);
        n++;
    }
    
    //ʣ������������ߵ���1024  
    s_dma_descriptor_table[n].xfercfg = ( 0
                                        //| DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
                                        | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                        | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                        | DMA_CHANNEL_XFERCFG_WIDTH(0)           //���8λ
                                        | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
                                        | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
                                        | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - (n<<10) - 1)  //DMA����
                                        );
    s_dma_descriptor_table[n].srcEndAddr = SADDR;
    s_dma_descriptor_table[n].dstEndAddr = (void *)((uint32)DADDR + count - 1);
    s_dma_descriptor_table[n].linkToNextDesc = 0;
        
    DMA0->CHANNEL[dmach].XFERCFG = s_dma_descriptor_table[dmach].xfercfg;
}



















ALIGN(512) dma_pingpong_descriptor_t ADC_TransferDescriptors[4]={0};//ADC-DMA��Ping-Pong����������
ALIGN(512) dma_pingpong_descriptor_t DMA_ChannelDescriptors[DMA_CHMAX]={0};//ADC-DMA��Ping-Pong����������
extern uint16 data[32];
void DMA_Init_ADC(ADCCH_enum ch ,DMACH_enum dmach,uint32 freq, void *SADDR, void *DADDR, uint16 count)
{
    uint8  n;
	  uint16 temp_div;

/*! @name ����ΪADCʱ�����ò���***************************************************************/
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

/*! @name ����ΪADC���ò��� *********************************************************************/

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
		iocon_init(B0 ,ALT0 | NOPULL | ANALOG | FILTEROFF);
		iocon_init(A23,ALT0 | NOPULL | ANALOG | FILTEROFF);
		
    temp_div = (main_clk_mhz*100/80 + 99)/100;
    ADC0->CTRL = ( 0
                 | ADC_CTRL_CLKDIV(temp_div-1)      //��Ƶ��󲻳���80M
                 | ADC_CTRL_RESOL(0x0)              //Ĭ��12λ�ֱ���
                 //| ADC_CTRL_BYPASSCAL_MASK        //����У׼  0:����У׼����    1���ر�У׼   ����Ϊ0
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
			
/**/sct_pwm_init(SCT0_OUT9_A30, freq, SCT0_OUTPUT_CH9_DUTY_MAX*0.5);	//����SCT0��OUT9���200kHz�ķ��������ڴ���ADC
		
		ADC0->SEQ_CTRL[0]= 0;		//��������SEQB_ENA�Ĵ�������ֹ������ٴ���
		ADC0->SEQ_CTRL[0]= (0
											|ADC_SEQ_CTRL_CHANNELS(1<<ch)  //����ͨ��
											|ADC_SEQ_CTRL_TRIGGER(0x03) //0x03-��SCT0��Output4��Ϊ�����ź�����
											|ADC_SEQ_CTRL_TRIGPOL_MASK	//ѡ��λ�����ش���
											|ADC_SEQ_CTRL_BURST_MASK
											|ADC_SEQ_CTRL_SYNCBYPASS_MASK
											|ADC_SEQ_CTRL_MODE_MASK
											|ADC_SEQ_CTRL_SEQ_ENA_MASK				//ʹ��ADCת��
										 );
		ADC0->INTEN= (0
								 |ADC_INTEN_SEQA_INTEN_MASK
									);
			
/*******************************************************************************************************/
/*! @name ����ΪDMA���ò��� ******************************************************************/
			
	  SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //��DMAʱ�� enable the clock to the DMA
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //���DMA��λʱ��Clear the DMA peripheral reset 
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�򿪶�·����ʱ��
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(0); //����DMA��������ͨ�� 0ΪADC0 Sequence A interrupt 1ΪADC0 Sequence B interrupt
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�رն�·����ʱ��

		ADC_TransferDescriptors[0].source = (uint32_t)&ADC0->SEQ_GDAT[0];
		ADC_TransferDescriptors[1].source = (uint32_t)&ADC0->SEQ_GDAT[0];
		ADC_TransferDescriptors[2].source = (uint32_t)&ADC0->SEQ_GDAT[0];
		ADC_TransferDescriptors[3].source = (uint32_t)&ADC0->SEQ_GDAT[0];

		ADC_TransferDescriptors[0].dest = (uint32_t)&data[4];
		ADC_TransferDescriptors[1].dest = (uint32_t)&data[9];
		ADC_TransferDescriptors[2].dest = (uint32_t)&data[14];
		ADC_TransferDescriptors[3].dest = (uint32_t)&data[19];

		ADC_TransferDescriptors[0].next = (uint32_t)&ADC_TransferDescriptors[1];
		ADC_TransferDescriptors[1].next = (uint32_t)&ADC_TransferDescriptors[2];
		ADC_TransferDescriptors[2].next = (uint32_t)&ADC_TransferDescriptors[3];
		ADC_TransferDescriptors[3].next = (uint32_t)&ADC_TransferDescriptors[0];


		ADC_TransferDescriptors[0].xfercfg = ( 0
																			 | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
																			 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																			 | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
																			 | DMA_CHANNEL_XFERCFG_WIDTH(1)           //���16λ
																			 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
																			 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
																			 | DMA_CHANNEL_XFERCFG_XFERCOUNT(5) //DMA����
																			 );
		ADC_TransferDescriptors[1].xfercfg = ADC_TransferDescriptors[0].xfercfg;
		ADC_TransferDescriptors[2].xfercfg = ADC_TransferDescriptors[0].xfercfg;
		ADC_TransferDescriptors[3].xfercfg = ADC_TransferDescriptors[0].xfercfg;
			
    DMA_ChannelDescriptors[dmach].source = (uint32_t)ADC_TransferDescriptors[0].source;
    DMA_ChannelDescriptors[dmach].dest   = (uint32_t)ADC_TransferDescriptors[0].dest;
    DMA_ChannelDescriptors[dmach].next   = (uint32_t)&ADC_TransferDescriptors[1];
		DMA_ChannelDescriptors[dmach].xfercfg= (uint32_t)ADC_TransferDescriptors[0].xfercfg;
		
    DMA0->SRAMBASE = (uint32_t)DMA_ChannelDescriptors;		//���ṹDMA_ChannelDescriptors��RAMBASE
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

    DMA0->CHANNEL[dmach].XFERCFG = DMA_ChannelDescriptors[dmach].xfercfg;
}

void DMA_Init_ADC_once(ADCCH_enum ch ,DMACH_enum dmach, ADCRES_enum resolution,uint32 freq, void *SADDR, void *DADDR, uint16 count)
{
	
	  ADC0->CTRL &= ~ADC_CTRL_RESOL_MASK;
    ADC0->CTRL |= ADC_CTRL_RESOL(resolution);   //�ֱ���
	
/****/sct_pwm_init(SCT0_OUT9_A30, freq, SCT0_OUTPUT_CH9_DUTY_MAX*0.5);	//����SCT0��OUT9���200kHz�ķ��������ڴ���ADC
		
		ADC0->INTEN = (0
//									|ADC_INTEN_SEQA_INTEN_MASK
									);		//����ADCϵ��A�ж�
	
	
		ADC0->SEQ_CTRL[0]= 0;		//��������SEQA_ENA�Ĵ�������ֹ������ٴ���
		ADC0->SEQ_CTRL[0]= (0
											|ADC_SEQ_CTRL_CHANNELS(1<<ch)  //����ͨ��
											|ADC_SEQ_CTRL_TRIGGER(0x03) //0x03-��SCT0��Output4��Ϊ�����ź�����
											|ADC_SEQ_CTRL_TRIGPOL_MASK	//ѡ��λ�����ش���
											|ADC_SEQ_CTRL_BURST_MASK
											|ADC_SEQ_CTRL_SEQ_ENA_MASK				//ʹ��ADCת��
										 );				
		
}
