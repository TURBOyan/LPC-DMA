/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		DMA
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看LPC546XX_config.h文件内版本宏定义
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


ALIGN(512) dma_descriptor_t s_dma_descriptor_table[DMA_CHMAX] = {0};//DMA通道描述符

//-------------------------------------------------------------------------------------------------------------------
//  @brief      DMA初始化
//  @param      dmach       DMA通道
//  @param      *SADDR      源地址
//  @param      *DADDR      目的地址
//  @param      count       DMA传输次数
//  @return     void
//  Sample usage:           dam_init(DMA_CH0, (void *)&GPIO_PIN(0,0), (void *)&image[0][0], 188);     //初始化DMA  通道0   源地址为A0-A7  目的地址为image数组的首地址   传输字节数为188次
//-------------------------------------------------------------------------------------------------------------------
void dam_init(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)
{
    uint8  n;
    uint32 temp_pin;

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //打开DMA时钟
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //清除DMA复位时钟
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //打开多路复用时钟
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(2); //设置DMA触发复用通道 为SCT request0
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //关闭多路复用时钟
    
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
                               | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 上升沿
                                //| DMA_CHANNEL_CFG_TRIGTYPE_MASK   //0 :边沿触发
                               | DMA_CHANNEL_CFG_TRIGBURST_MASK     //启用burst传输
                               | DMA_CHANNEL_CFG_BURSTPOWER(0)      //burst传输为一个字节
                               | DMA_CHANNEL_CFG_CHPRIORITY(0)      //优先级设置   0为最高
                               );
    
    DMA0->COMMON[0].SETVALID = 1<<dmach;
    DMA0->COMMON[0].INTENSET = 1<<dmach;
    
    s_dma_descriptor_table[dmach].xfercfg = ( 0
                                   //| DMA_CHANNEL_XFERCFG_RELOAD_MASK        //参数自动重载
                                   | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                   | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                   | DMA_CHANNEL_XFERCFG_WIDTH(0)           //宽度8位
                                   | DMA_CHANNEL_XFERCFG_SRCINC(0)          //源地址不自增
                                   | DMA_CHANNEL_XFERCFG_DSTINC(1)          //目的地址自增一个数据宽度
                                   | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA次数
                                   );
    
    s_dma_descriptor_table[dmach].srcEndAddr = SADDR;
    s_dma_descriptor_table[dmach].dstEndAddr = (void *)((uint32)DADDR + count - 1);
    s_dma_descriptor_table[dmach].linkToNextDesc = 0;
    
    DMA0->CHANNEL[dmach].XFERCFG = s_dma_descriptor_table[dmach].xfercfg;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      DMA 链接传输 初始化
//  @param      dmach       DMA通道
//  @param      *SADDR      源地址
//  @param      *DADDR      目的地址
//  @param      count       DMA传输次数
//  @return     void
//  Sample usage:           dam_init(DMA_CH0, (void *)&GPIO_PIN(0,0), (void *)&image[0][0], 188);     //初始化DMA  通道0   源地址为A0-A7  目的地址为image数组的首地址   传输字节数为188次
//  @note                   使用链接传输可以实现 一次性传输超过1024 但会占用其他DMA通道的 通道描述符
//-------------------------------------------------------------------------------------------------------------------
void dam_init_linked(DMACH_enum dmach, void *SADDR, void *DADDR, uint32 count)
{
    uint8  n;
    uint32 temp_pin;

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //打开DMA时钟
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //清除DMA复位时钟
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //打开多路复用时钟
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(2); //设置DMA触发复用通道 为SCT request0
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //关闭多路复用时钟
    
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
                               | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 上升沿
                                //| DMA_CHANNEL_CFG_TRIGTYPE_MASK   //0 :边沿触发
                               | DMA_CHANNEL_CFG_TRIGBURST_MASK     //启用burst传输
                               | DMA_CHANNEL_CFG_BURSTPOWER(0)      //burst传输为一个字节
                               | DMA_CHANNEL_CFG_CHPRIORITY(0)      //优先级设置   0为最高
                               );
    
    DMA0->COMMON[0].SETVALID = 1<<dmach;
    DMA0->COMMON[0].INTENSET = 1<<dmach;
    
    n = 0;
    while((n+1)<<10 < count)//剩下数量大于1024
    {
        s_dma_descriptor_table[n].xfercfg = ( 0
                                            | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //参数自动重载
                                            | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                            | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                            | DMA_CHANNEL_XFERCFG_WIDTH(0)           //宽度8位
                                            | DMA_CHANNEL_XFERCFG_SRCINC(0)          //源地址不自增
                                            | DMA_CHANNEL_XFERCFG_DSTINC(1)          //目的地址自增一个数据宽度
                                            | DMA_CHANNEL_XFERCFG_XFERCOUNT(1024-1)  //DMA次数
                                            );
        s_dma_descriptor_table[n].srcEndAddr = SADDR;
        s_dma_descriptor_table[n].dstEndAddr = (void *)((uint32)DADDR + ((n+1)<<10) - 1);
        s_dma_descriptor_table[n].linkToNextDesc = (void *)(&s_dma_descriptor_table[n+1]);
        n++;
    }
    
    //剩下数量不足或者等于1024  
    s_dma_descriptor_table[n].xfercfg = ( 0
                                        //| DMA_CHANNEL_XFERCFG_RELOAD_MASK        //参数自动重载
                                        | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                        | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                        | DMA_CHANNEL_XFERCFG_WIDTH(0)           //宽度8位
                                        | DMA_CHANNEL_XFERCFG_SRCINC(0)          //源地址不自增
                                        | DMA_CHANNEL_XFERCFG_DSTINC(1)          //目的地址自增一个数据宽度
                                        | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - (n<<10) - 1)  //DMA次数
                                        );
    s_dma_descriptor_table[n].srcEndAddr = SADDR;
    s_dma_descriptor_table[n].dstEndAddr = (void *)((uint32)DADDR + count - 1);
    s_dma_descriptor_table[n].linkToNextDesc = 0;
        
    DMA0->CHANNEL[dmach].XFERCFG = s_dma_descriptor_table[dmach].xfercfg;
}



















ALIGN(512) dma_pingpong_descriptor_t ADC_TransferDescriptors[4]={0};//ADC-DMA的Ping-Pong传输描述符
ALIGN(512) dma_pingpong_descriptor_t DMA_ChannelDescriptors[DMA_CHMAX]={0};//ADC-DMA的Ping-Pong传输描述符
extern uint16 data[32];
void DMA_Init_ADC(ADCCH_enum ch ,DMACH_enum dmach,uint32 freq, void *SADDR, void *DADDR, uint16 count)
{
    uint8  n;
	  uint16 temp_div;

/*! @name 以下为ADC时钟配置部分***************************************************************/
    SYSCON->PDRUNCFGCLR[0] = ( 0
                             | SYSCON_PDRUNCFGCLR_PDEN_ADC0_MASK 
                             | SYSCON_PDRUNCFGCLR_PDEN_VD2_ANA_MASK 
                             | SYSCON_PDRUNCFGCLR_PDEN_VDDA_MASK
                             | SYSCON_PDRUNCFGCLR_PDEN_VREFP_MASK
                             ); //打开ADC电源
    systick_delay_us(20);                                       //必要延时
   
    SYSCON->ADCCLKSEL = SYSCON_ADCCLKSEL_SEL(0x01);             //选择pll_clk为ADC时钟源
	
    SYSCON->ADCCLKDIV = 0;  
    SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_HALT_MASK;							//设置时钟分频前必须停止分频计数器，防止出错
    SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_DIV(0);								//设置时钟分频为1分频
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_ADC0_MASK;     //打开ADC时钟
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK; //清除复位ADC时钟

/*! @name 以下为ADC配置部分 *********************************************************************/

    switch(ch)			//端口复用
    {
        case ADC_CH0_A10:   iocon_init(A10,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH1_A11:   iocon_init(A11,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH2_A12:   iocon_init(A12,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH3_A15:   iocon_init(A15,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH4_A16:   iocon_init(A16,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH5_A31:   iocon_init(A31,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH6_B0 :   iocon_init(B0 ,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH11_A23:  iocon_init(A23,ALT0 | NOPULL | ANALOG | FILTEROFF); break;

        default:        ASSERT(0);//通道错误 进入断言失败
    }
		iocon_init(B0 ,ALT0 | NOPULL | ANALOG | FILTEROFF);
		iocon_init(A23,ALT0 | NOPULL | ANALOG | FILTEROFF);
		
    temp_div = (main_clk_mhz*100/80 + 99)/100;
    ADC0->CTRL = ( 0
                 | ADC_CTRL_CLKDIV(temp_div-1)      //分频最大不超过80M
                 | ADC_CTRL_RESOL(0x0)              //默认12位分辨率
                 //| ADC_CTRL_BYPASSCAL_MASK        //采样校准  0:启用校准功能    1：关闭校准   屏蔽为0
                 | ADC_CTRL_TSAMP(0)                //采样周期，设置约为2.5个ADC时钟
                 );
    ADC0->STARTUP = ADC_STARTUP_ADC_ENA_MASK;           //开启ADC
    systick_delay_us(10);                               //必要延时
    if (!(ADC0->STARTUP & ADC_STARTUP_ADC_ENA_MASK))
    {
        ASSERT(0);//ADC没有上电 进入断言失败
    }
    
    
    ADC0->CALIB = ADC_CALIB_CALIB_MASK;                 //ADC校准
    while(ADC_CALIB_CALIB_MASK == (ADC0->CALIB & ADC_CALIB_CALIB_MASK));
    
    ADC0->STARTUP |= ADC_STARTUP_ADC_INIT_MASK;         //ADC初始化
    while(ADC_STARTUP_ADC_INIT_MASK == (ADC0->STARTUP & ADC_STARTUP_ADC_INIT_MASK)){};
			
/**/sct_pwm_init(SCT0_OUT9_A30, freq, SCT0_OUTPUT_CH9_DUTY_MAX*0.5);	//设置SCT0的OUT9输出200kHz的方波，用于触发ADC
		
		ADC0->SEQ_CTRL[0]= 0;		//首先清零SEQB_ENA寄存器，防止生成虚假触发
		ADC0->SEQ_CTRL[0]= (0
											|ADC_SEQ_CTRL_CHANNELS(1<<ch)  //设置通道
											|ADC_SEQ_CTRL_TRIGGER(0x03) //0x03-以SCT0的Output4作为触发信号输入
											|ADC_SEQ_CTRL_TRIGPOL_MASK	//选择位上升沿触发
											|ADC_SEQ_CTRL_BURST_MASK
											|ADC_SEQ_CTRL_SYNCBYPASS_MASK
											|ADC_SEQ_CTRL_MODE_MASK
											|ADC_SEQ_CTRL_SEQ_ENA_MASK				//使能ADC转换
										 );
		ADC0->INTEN= (0
								 |ADC_INTEN_SEQA_INTEN_MASK
									);
			
/*******************************************************************************************************/
/*! @name 以下为DMA配置部分 ******************************************************************/
			
	  SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //打开DMA时钟 enable the clock to the DMA
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //清除DMA复位时钟Clear the DMA peripheral reset 
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //打开多路复用时钟
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(0); //设置DMA触发复用通道 0为ADC0 Sequence A interrupt 1为ADC0 Sequence B interrupt
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //关闭多路复用时钟

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
																			 | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //参数自动重载
																			 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																			 | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
																			 | DMA_CHANNEL_XFERCFG_WIDTH(1)           //宽度16位
																			 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //源地址不自增
																			 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //目的地址自增一个数据宽度
																			 | DMA_CHANNEL_XFERCFG_XFERCOUNT(5) //DMA次数
																			 );
		ADC_TransferDescriptors[1].xfercfg = ADC_TransferDescriptors[0].xfercfg;
		ADC_TransferDescriptors[2].xfercfg = ADC_TransferDescriptors[0].xfercfg;
		ADC_TransferDescriptors[3].xfercfg = ADC_TransferDescriptors[0].xfercfg;
			
    DMA_ChannelDescriptors[dmach].source = (uint32_t)ADC_TransferDescriptors[0].source;
    DMA_ChannelDescriptors[dmach].dest   = (uint32_t)ADC_TransferDescriptors[0].dest;
    DMA_ChannelDescriptors[dmach].next   = (uint32_t)&ADC_TransferDescriptors[1];
		DMA_ChannelDescriptors[dmach].xfercfg= (uint32_t)ADC_TransferDescriptors[0].xfercfg;
		
    DMA0->SRAMBASE = (uint32_t)DMA_ChannelDescriptors;		//将结构DMA_ChannelDescriptors至RAMBASE
    DMA0->CTRL = DMA_CTRL_ENABLE_MASK;		//使能DMA
    DMA0->COMMON[0].ENABLESET = 1<<dmach;	//使能DMA通道
    
    DMA0->CHANNEL[dmach].CFG = ( 0
                               | DMA_CHANNEL_CFG_HWTRIGEN_MASK
                               | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 上升沿
                               | DMA_CHANNEL_CFG_TRIGTYPE_MASK   		//0 :边沿触发
                               | DMA_CHANNEL_CFG_TRIGBURST_MASK     //启用burst传输
                               | DMA_CHANNEL_CFG_BURSTPOWER(0)      //burst传输为4个字节
                               | DMA_CHANNEL_CFG_CHPRIORITY(0)      //优先级设置   0为最高
                               );
    
    DMA0->COMMON[0].SETVALID = 1<<dmach;
    DMA0->COMMON[0].INTENSET = 1<<dmach;

    DMA0->CHANNEL[dmach].XFERCFG = DMA_ChannelDescriptors[dmach].xfercfg;
}

void DMA_Init_ADC_once(ADCCH_enum ch ,DMACH_enum dmach, ADCRES_enum resolution,uint32 freq, void *SADDR, void *DADDR, uint16 count)
{
	
	  ADC0->CTRL &= ~ADC_CTRL_RESOL_MASK;
    ADC0->CTRL |= ADC_CTRL_RESOL(resolution);   //分辨率
	
/****/sct_pwm_init(SCT0_OUT9_A30, freq, SCT0_OUTPUT_CH9_DUTY_MAX*0.5);	//设置SCT0的OUT9输出200kHz的方波，用于触发ADC
		
		ADC0->INTEN = (0
//									|ADC_INTEN_SEQA_INTEN_MASK
									);		//禁用ADC系列A中断
	
	
		ADC0->SEQ_CTRL[0]= 0;		//首先清零SEQA_ENA寄存器，防止生成虚假触发
		ADC0->SEQ_CTRL[0]= (0
											|ADC_SEQ_CTRL_CHANNELS(1<<ch)  //设置通道
											|ADC_SEQ_CTRL_TRIGGER(0x03) //0x03-以SCT0的Output4作为触发信号输入
											|ADC_SEQ_CTRL_TRIGPOL_MASK	//选择位上升沿触发
											|ADC_SEQ_CTRL_BURST_MASK
											|ADC_SEQ_CTRL_SEQ_ENA_MASK				//使能ADC转换
										 );				
		
}
