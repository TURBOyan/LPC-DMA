#include "Fourier_DMA.h"
#include "string.h"
#include "Selfbuild_oled.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_sct.h"
#include "LPC546XX_pll.h"
#include "LPC546XX_systick.h"

/** Fourier_Data - 定义保存本底层所有用户数据的结构体 */
struct Fourier_Data_t Fourier_Data;	
/** FourierDMA_ChannelDescriptors - 定义傅里叶DMA的传输描述符,ALIGN(512)->设置字节对齐 */
ALIGN(512) dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX]={0};

/*!
 * @addtogroup 基波幅值提取初始化
 * @{
 */
void Fourier_Init(DMACH_enum dmach,ADCCH_enum ch)
{
	/* ----------------------------------------------------------------------------
		 -- 卡尔曼滤波参数初始化
		 ---------------------------------------------------------------------------- */
		Fourier_Data.ADCCH_Data[ch].LastP=0.02;
		Fourier_Data.ADCCH_Data[ch].Now_P=0;
		Fourier_Data.ADCCH_Data[ch].out=0;
		Fourier_Data.ADCCH_Data[ch].Kg=0;
		Fourier_Data.ADCCH_Data[ch].Q=0.001;
		Fourier_Data.ADCCH_Data[ch].R=0.2;
	/* ----------------------------------------------------------------------------
		 -- SCT触发信号初始化
		 ---------------------------------------------------------------------------- */
		sct_pwm_init(SCT0_OUT9_A30, ADC_SampFreq, SCT0_OUTPUT_CH9_DUTY_MAX*0.5);
	/* ----------------------------------------------------------------------------
		 -- ADC初始化
		 ---------------------------------------------------------------------------- */
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
								 | ADC_CTRL_BYPASSCAL_MASK        //采样校准  0:启用校准功能    1：关闭校准   屏蔽为0
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
		
		systick_delay_us(100);
	/* ----------------------------------------------------------------------------
		 -- DMA初始化
		 ---------------------------------------------------------------------------- */
		Fourier_Data.ADCCH_Data[ch].DMA_CH=dmach;	//保存DMA传输通道
		SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //打开DMA时钟
		SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //清除DMA复位时钟
		
		SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //打开多路复用时钟，设置DMA触发复用通道必须先打开多路复用时钟
		INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(0); //设置DMA触发复用通道 0为ADC0 Sequence A interrupt 1为ADC0 Sequence B interrupt
		SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //关闭多路复用时钟
			
		FourierDMA_ChannelDescriptors[dmach].srcEndAddr = 0;		//设置源地址
		FourierDMA_ChannelDescriptors[dmach].dstEndAddr = 0;	//设置目的地址
		FourierDMA_ChannelDescriptors[dmach].linkToNextDesc= 0;
		FourierDMA_ChannelDescriptors[dmach].xfercfg=0;
		DMA0->SRAMBASE = (uint32_t)FourierDMA_ChannelDescriptors;		//将结构DMA_ChannelDescriptors映射至SRAMBASE
		
		DMA0->CTRL = DMA_CTRL_ENABLE_MASK;		//使能DMA

		DMA0->CHANNEL[dmach].CFG = ( 0
															 | DMA_CHANNEL_CFG_HWTRIGEN_MASK			
															 | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 上升沿
															 | DMA_CHANNEL_CFG_TRIGTYPE_MASK   		//0 :边沿触发
															 | DMA_CHANNEL_CFG_TRIGBURST_MASK     //启用burst传输
															 | DMA_CHANNEL_CFG_BURSTPOWER(0)      
															 | DMA_CHANNEL_CFG_CHPRIORITY(0)      //优先级设置   0为最高
															 );

		DMA0->COMMON[0].SETVALID = 1<<dmach;
		DMA0->COMMON[0].INTENSET = 1<<dmach;
		
		
		DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;	//保存配置

}

//使用__STATIC_INLINE为了将这段函数内嵌到使用该函数的地方，这样可以减少函数调用的时间
__STATIC_INLINE void Fourier_dma_reload(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)//DMA参数重载  重新设置参数无需调用初始化
{
    DMA0->COMMON[0].ENABLECLR = 1<<dmach;	//清除某个信道的DMA允许
    DMA0->COMMON[0].ABORT = 1<<dmach;		//终止某个信道的DMA操作
    FourierDMA_ChannelDescriptors[dmach].xfercfg = ( 0
																						 //| DMA_CHANNEL_XFERCFG_RELOAD_MASK        //参数自动重载
																						 | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
																						 | DMA_CHANNEL_XFERCFG_SETINTA_MASK      
																						 | DMA_CHANNEL_XFERCFG_CLRTRIG_MASK				//传输结束后关闭触发
																						 | DMA_CHANNEL_XFERCFG_WIDTH(2)           //宽度32位
																						 | DMA_CHANNEL_XFERCFG_SRCINC(0)          //源地址不自增
																						 | DMA_CHANNEL_XFERCFG_DSTINC(1)          //目的地址自增一个数据宽度
																						 | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA次数
                                            );
    FourierDMA_ChannelDescriptors[dmach].srcEndAddr = (uint32*)SADDR;		//设置源地址
		FourierDMA_ChannelDescriptors[dmach].linkToNextDesc   = 0;
    FourierDMA_ChannelDescriptors[dmach].dstEndAddr = (uint32*)DADDR+count-1;	//设置目的地址
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    DMA0->CHANNEL[dmach].XFERCFG = FourierDMA_ChannelDescriptors[dmach].xfercfg;
	
		if((DMA_CHANNEL_XFERCFG_CFGVALID_MASK & DMA0->CHANNEL[dmach].XFERCFG) == 0)	//如果配置失败则断言
		{
			ASSERT(0);//串口输出错误信息，如果要使用则需要将LPC546XX_config.h文件内断言使能
		}
}

__STATIC_INLINE void Fourier_adc_reload(ADCCH_enum ch,ADCRES_enum resolution)
{
	  ADC0->CTRL &= ~ADC_CTRL_RESOL_MASK;
    ADC0->CTRL |= ADC_CTRL_RESOL(resolution);   //设置分辨率	
		ADC0->SEQ_CTRL[0]= 0;		//首先清零SEQB_ENA寄存器，防止生成虚假置位
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
}

__STATIC_INLINE void IRQ_Ctrl_ADC_DMA(uint8 ONorOFF)
{
		if(ONorOFF == 0)
		{
			disable_irq(DMA0_IRQn);		//失能DMA0中断
			ADC0->INTEN =0;						//失能ADC中断
		}
		else
		{
			enable_irq(DMA0_IRQn);		//使能DMA0中断
			ADC0->INTEN= (0						//使能ADC中断
							 |ADC_INTEN_SEQA_INTEN_MASK
								);
		}
}

void Fourier_DMAReadBuff(ADCCH_enum ch,ADCRES_enum resolution,uint16 *Buff_Out)
{
		static vuint32 Buff[ADC_Samp_SIZE+10];
	
		while(Fourier_Data.Busy_Flag);							//等待其他通道计算完成，保证无论何时CPU只完整处理一个通道
	
		Fourier_Data.ADCCH_Data[ch].START_Flag = 1;	//置位转换标志位
		Fourier_Data.Busy_Flag = 1;									//置位全局忙标志，防止出现两个通道同时开始转换的情况
		Fourier_Data.ADCCH_Save = ch;								//保存此时正在转换的通道，防止出现两个通道同时开始转换的情况

		Fourier_adc_reload(ch,resolution);	//ADC转换参数重载
		Fourier_dma_reload(Fourier_Data.ADCCH_Data[ch].DMA_CH, (void*)&ADC0->SEQ_GDAT[0],(void*)&Buff[0],ADC_Samp_SIZE);//DMA转换参数重载
	
		IRQ_Ctrl_ADC_DMA(1);	//打开中断
	
		while(DMA0->CHANNEL[Fourier_Data.ADCCH_Data[ch].DMA_CH].CTLSTAT== 0x00){}	//等待传输完成
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
//		while(Fourier_Data.ADCCH_Data[ch].START_Flag)		//等待计算完成
//		{
//			if((DMA0->CHANNEL[Fourier_Data.ADCCH_Data[ch].DMA_CH].CTLSTAT & DMA_CHANNEL_CTLSTAT_TRIG_MASK) ==0)
//			{
//				Fourier_Data.ADCCH_Data[ch].START_Flag=0;
//			}
//				
//		}
	
		IRQ_Ctrl_ADC_DMA(0);//关闭中断
	
		for(uint8 num=0;num<ADC_Samp_SIZE;num++)	//输出数据
		{
				*(Buff_Out+num)=(uint16)((Buff[num]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-resolution)*2));;
		}
		Fourier_Data.Busy_Flag = 0;	//清全局忙标志		
}


/********************************FFT功能实现及相关变量及初始化*****************************************/
//cos_sin_search[n][0]=(2/采样点数n)*cos(2*Pi*信号频率f*n*采样周期T)
//cos_sin_search[n][1]=(2/采样点数n)*sin(2*Pi*信号频率f*n*采样周期T)
float cos_sin_search[11][2]=		//正余弦查表，采样点10个，原信号频率20khz
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
	return ((An*An+Bn*Bn)==0 ? 0 : sqrt(An*An+Bn*Bn));	//返回基波幅值
}

/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
*/
float kalmanFilter(struct Fourier_Data_t* KALMAN,ADCCH_enum ch,float input)
{
	 //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
	 KALMAN->ADCCH_Data[ch].Now_P = KALMAN->ADCCH_Data[ch].LastP + KALMAN->ADCCH_Data[ch].Q;
	 //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
	 KALMAN->ADCCH_Data[ch].Kg = KALMAN->ADCCH_Data[ch].Now_P / (KALMAN->ADCCH_Data[ch].Now_P + KALMAN->ADCCH_Data[ch].R);
	 //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
	 KALMAN->ADCCH_Data[ch].out = KALMAN->ADCCH_Data[ch].out + KALMAN->ADCCH_Data[ch].Kg * (input -KALMAN->ADCCH_Data[ch].out);//因为这一次的预测值就是上一次的输出值
	 //更新协方差方程: 本次的系统协方差赋值给 kfp->LastP 为下一次运算准备。
	 KALMAN->ADCCH_Data[ch].LastP = (1-KALMAN->ADCCH_Data[ch].Kg) * KALMAN->ADCCH_Data[ch].Now_P;
	 return KALMAN->ADCCH_Data[ch].out;
}

uint16 Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution)
{
		uint16 Data_Buff[ADC_Samp_SIZE]={0};
		float Result_WithoutFilt;
		uint32 Result_WithFilt;
		//将2个周期的采样信号输出至Data_Buff
		Fourier_DMAReadBuff(ch,resolution,Data_Buff);		
		//计算基波幅值并卡尔曼滤波

		Result_WithoutFilt=(float)FFT(Data_Buff);
		Result_WithoutFilt=Result_WithoutFilt<=0?0:Result_WithoutFilt;
		
		//Result_WithFilt = (uint32)kalmanFilter(&Fourier_Data,ch,Result_WithoutFilt);	
		return Result_WithoutFilt;
}