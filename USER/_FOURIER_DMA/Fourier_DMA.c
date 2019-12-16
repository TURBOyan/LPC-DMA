#include "Fourier_DMA.h"
#include "string.h"

struct Fourier_Data_t Fourier_Data;		//定义保存傅里叶变换数据的结构体

//ALIGN(512)作用是设置字节对齐
//定义傅里叶DMA的传输描述符
ALIGN(512) dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX]={0};

//使用__STATIC_INLINE为了将这段函数内嵌到使用该函数的地方，这样可以减少函数调用的时间
__STATIC_INLINE void Fourier_dma_reload(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)//DMA参数重载  重新设置参数无需调用初始化
{
    DMA0->COMMON[0].ENABLECLR = 1<<dmach;	//清除某个信道的DMA允许
    DMA0->COMMON[0].ABORT = 1<<dmach;		//终止某个信道的DMA操作
    FourierDMA_ChannelDescriptors[dmach].xfercfg = ( 0
																						 | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //参数自动重载
																						 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																						 | DMA_CHANNEL_XFERCFG_SETINTA_MASK      
																						 | DMA_CHANNEL_XFERCFG_WIDTH(1)           //宽度16位
																						 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //源地址不自增
																						 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //目的地址自增一个数据宽度
																						 | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA次数
                                            );
    FourierDMA_ChannelDescriptors[dmach].srcEndAddr = (uint32*)SADDR;		//设置源地址
		FourierDMA_ChannelDescriptors[dmach].linkToNextDesc   = 0;
    FourierDMA_ChannelDescriptors[dmach].dstEndAddr = (uint16*)DADDR+count-1;	//设置目的地址
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;
}

void Fourier_DMA_Init(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)
{
	  SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //打开DMA时钟
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //清除DMA复位时钟
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //打开多路复用时钟，设置DMA触发复用通道必须先打开多路复用时钟
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(0); //设置DMA触发复用通道 0为ADC0 Sequence A interrupt 1为ADC0 Sequence B interrupt
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //关闭多路复用时钟
			
    FourierDMA_ChannelDescriptors[dmach].srcEndAddr = (uint32*)SADDR;		//设置源地址
    FourierDMA_ChannelDescriptors[dmach].dstEndAddr = (uint16*)DADDR+count-1;	//设置目的地址
	
    FourierDMA_ChannelDescriptors[dmach].linkToNextDesc   = 0;
		FourierDMA_ChannelDescriptors[dmach].xfercfg=  ( 0
																			 | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //参数自动重载
																			 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																			 | DMA_CHANNEL_XFERCFG_SETINTA_MASK      
																			 | DMA_CHANNEL_XFERCFG_WIDTH(1)           //宽度16位
																			 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //源地址不自增
																			 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //目的地址自增一个数据宽度
																			 | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA次数
																			 );

    DMA0->SRAMBASE = (uint32_t)FourierDMA_ChannelDescriptors;		//将结构DMA_ChannelDescriptors映射至SRAMBASE
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

    DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;	//保存配置
		enable_irq(DMA0_IRQn);		//使能DMA0中断
		set_irq_priority(DMA0_IRQn,0);//设置DMA中断为最高优先级
		
		if((DMA_CHANNEL_XFERCFG_CFGVALID_MASK & DMA0->CHANNEL[dmach].XFERCFG) == 0)	//如果配置失败则断言
		{
			ASSERT(0);//串口输出错误信息，如果要使用则需要将LPC546XX_config.h文件内断言使能
		}
}

void Fourier_ADC_Init(ADCCH_enum ch)
{
		uint16 temp_div;
	
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
    temp_div = (main_clk_mhz*100/80 + 99)/100;
    ADC0->CTRL = ( 0
                 | ADC_CTRL_CLKDIV(temp_div-1)      //分频最大不超过80M
                 | ADC_CTRL_RESOL(0x0)              //默认12位分辨率
               //  | ADC_CTRL_BYPASSCAL_MASK        //采样校准  0:启用校准功能    1：关闭校准   屏蔽为0
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
		while(Fourier_Data.Busy_Flag);//等待其他通道计算完成
		Fourier_Data.ADCCH_Data[ch].START_Flag = 1;	//置位转换标志位
		Fourier_Data.Busy_Flag = 1;		//置位全局忙标志，防止出现两个通道同时开始转换的情况
		Fourier_Data.ADCCH_Data[ch].resolution = resolution;	//保存分辨率
		Fourier_Data.ADCCH_Save = ch;			//保存此时正在转换的通道，防止出现两个通道同时开始转换的情况
		memset((void*)&Fourier_Data.Buff,0,sizeof(Fourier_Data.Buff));		//清空数组
		
	  ADC0->CTRL &= ~ADC_CTRL_RESOL_MASK;
    ADC0->CTRL |= ADC_CTRL_RESOL(resolution);   //设置分辨率
		
		ADC0->SEQ_CTRL[0]= 0;		//首先清零SEQB_ENA寄存器，防止生成虚假触发
		ADC0->SEQ_CTRL[0]= (0
											|ADC_SEQ_CTRL_CHANNELS(1<<ch)  //设置通道
											|ADC_SEQ_CTRL_TRIGGER(0x05) //0x05-以SCT0的Output9作为触发信号输入
											|ADC_SEQ_CTRL_TRIGPOL_MASK	//选择位上升沿触发
											|ADC_SEQ_CTRL_SYNCBYPASS_MASK
											|ADC_SEQ_CTRL_MODE_MASK
			                |ADC_SEQ_CTRL_SINGLESTEP_MASK
											|ADC_SEQ_CTRL_SEQ_ENA_MASK				//使能ADC转换
										 );
		ADC0->SEQ_CTRL[0] |= ADC_SEQ_CTRL_START_MASK; 	//开启ADC转换

		Fourier_dma_reload(Fourier_DMACH, (void*)&ADC0->SEQ_GDAT[0],(void*)&Fourier_Data.Buff[0],ADC_Samp_SIZE);
		
		while(Fourier_Data.ADCCH_Data[ch].START_Flag);		//等待计算完成
		Fourier_Data.Busy_Flag = 0;	//清全局忙标志
}


		
void Fourier_interrupt_Func(void)
{
		uint16 Data_Buff[ADC_Samp_SIZE]={0};
    if(READ_DMA_FLAG(Fourier_DMACH))
    {
        CLEAR_DMA_FLAG(Fourier_DMACH);
				
				for(uint8 num=0;num < ADC_Samp_SIZE;num++)		//将buff内数据转换后缓存
				{
					Data_Buff[num] = (Fourier_Data.Buff[num]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].resolution)*2);
				}
				
				Fourier_Data.ADCCH_Data[Fourier_Data.ADCCH_Save].START_Flag = 0;		//清除起始标志位，以便下次转换
    }
}
