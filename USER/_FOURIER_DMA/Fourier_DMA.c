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
struct kalman_Data_t kalman_Data[14];
/** FourierDMA_ChannelDescriptors - 定义傅里叶DMA的传输描述符,ALIGN(512)->设置字节对齐 */
ALIGN(512) dma_descriptor_t FourierDMA_ChannelDescriptors[DMA_CHMAX]={0};

/**
 *基波幅值提取初始化
 *@param DMACH_enum dmach 设置不同ADC端口数据传输的DMA通道号,范围DMA_CH0~DMA_CH29
 *   		 ADCCH_enum ch 		设置AD转换端口
 *@return 无
 *@example Fourier_Init(DMA_CH0,ADC_CH6_B0);  设置B0口的基波幅值提取方式，数据传输用DMA_CH0通道
*/
void Fourier_Init(DMACH_enum dmach,ADCCH_enum ch)
{
	/* ----------------------------------------------------------------------------
		 -- 卡尔曼滤波参数初始化
		 ---------------------------------------------------------------------------- */
		kalman_Data[ch].LastP=0.02;
		kalman_Data[ch].Now_P=0;
		kalman_Data[ch].out=0;
		kalman_Data[ch].Kg=0;
		kalman_Data[ch].Q=0.001;
		kalman_Data[ch].R=0.2;
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
		Fourier_Data.DMA_CH[ch]=dmach;	//保存DMA传输通道
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
		
		enable_irq(DMA0_IRQn);		//使能DMA0中断
		ADC0->INTEN= (0						//使能ADC中断
						 |ADC_INTEN_SEQA_INTEN_MASK
							);

}

/**
 *DMA参数重载（内部函数，无需调用）
 *使用__STATIC_INLINE为了将这段函数内嵌到使用该函数的地方，这样可以减少函数调用的时间
 *@param DMACH_enum dmach 设置传输的DMA通道号,范围DMA_CH0~DMA_CH29
 *   		 void *SADDR 			DMA传输源地址
				 void *DADDR			DMA传输目的地址
				 uint16 count			传输次数
 *@return 无
 *@example Fourier_dma_reload(DMA_CH0, (void*)&ADC0->SEQ_GDAT[0],(void*)&Buff[0],20); 将ADC结果寄存器内的数据传输至Buff数组内，循环20次
*/
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

/**
 *ADC参数重载（内部函数，无需调用）
 *使用__STATIC_INLINE为了将这段函数内嵌到使用该函数的地方，这样可以减少函数调用的时间
 *@param ADCCH_enum ch 		设置AD转换端口
 *   		 ADCRES_enum resolution 设置AD转换位数，范围6位，8位，10位，12位
 *@return 无
 *@example Fourier_adc_reload(ADC_CH6_B0,ADC_12BIT);设置B0口的AD转换参数，并设置为12位ADC
*/
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

/**
 *中断控制（内部函数，无需调用）
 *使用__STATIC_INLINE为了将这段函数内嵌到使用该函数的地方，这样可以减少函数调用的时间
 *@param uint8 ONorOFF 	中断开关，1为开，0为关，控制DMA和ADC中断
 *@return 无
 *@example IRQ_Ctrl_ADC_DMA(1); 打开ADC和DMA中断
*/
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

/**
 *获取N此电压数据至指定数组内
*@param ADCCH_enum ch 		设置AD转换端口	
				ADCRES_enum resolution 设置AD转换位数，范围6位，8位，10位，12位
				uint16 *Buff_Out  传入N个采样点电压数据数组的指针
 *@return 无
 *@example Fourier_DMAReadBuff(ADC_CH6_B0,ADC_12BIT,Data_Buff); 将B0口的N个采样点的12位电压数据传入Data_Buff数组内
*/
uint8 START_FLAG=0;		//开始转换标志
void Fourier_DMAReadBuff(ADCCH_enum ch,ADCRES_enum resolution,uint16 *Buff_Out)
{
		static vuint32 Buff[ADC_Samp_SIZE+10];
		uint32 Buff2[ADC_Samp_SIZE+10];
	
		while(Fourier_Data.Busy_Flag);							//等待其他通道计算完成，保证无论何时CPU只完整处理一个通道
		START_FLAG=1;
		Fourier_Data.Busy_Flag = 1;									//置位全局忙标志，防止出现两个通道同时开始转换的情况
		Fourier_Data.ADCCH_Save = ch;								//保存此时正在转换的通道，防止出现两个通道同时开始转换的情况

		Fourier_adc_reload(ch,resolution);	//ADC参数重载
		Fourier_dma_reload(Fourier_Data.DMA_CH[ch], (void*)&ADC0->SEQ_GDAT[0],(void*)&Buff[0],ADC_Samp_SIZE);//DMA参数重载
	
		IRQ_Ctrl_ADC_DMA(1);	//打开中断
		while(START_FLAG){}		//等待转换完成
		IRQ_Ctrl_ADC_DMA(0);	//关闭中断
		
		uint8 Carry=(ADC_SEQ_GDAT_RESULT_SHIFT+(3-resolution)*2);
		for(uint8 num=0;num<ADC_Samp_SIZE;num++)	//输出数据
		{
				Buff2[num]=Buff[num];
				*(Buff_Out+num)=(uint16)((Buff2[num]&ADC_SEQ_GDAT_RESULT_MASK)>>Carry);
		}
		Fourier_Data.Busy_Flag = 0;	//清全局忙标志		
}

/* ------------------------------------------------------------------------------------------------------------------------
   -- 以下基波幅值计算函数与芯片平台无关，可直接移植，只需将N个采样点的电压数据传入即可
   -------------------------------------------------------------------------------------------------------------------------- */
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

/**
 *FFT基波幅值提取函数
*@param uint16 *adc_data	传入保存了N个采样点的电压数据
 *@return 基波幅值，如果传入的电压数据为6位的 ，返回基波幅值范围为0-31
 *								 如果传入的电压数据为8位的 ，返回基波幅值范围为0-127
 *								 如果传入的电压数据为10位的，返回基波幅值范围为0-512
 *								 如果传入的电压数据为12位的，返回基波幅值范围为0-2048
 *@example FFT(Data_Buff) 计算Data_Buff数组内的基波幅值
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
	return ((An*An+Bn*Bn)==0 ? 0 : sqrt(An*An+Bn*Bn));	//返回基波幅值
}

/**
 *卡尔曼滤波器
 *@param struct kalman_Data_t* KALMAN 卡尔曼结构体参数
 *   		 float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
*/
float kalmanFilter(struct kalman_Data_t* KALMAN,float input)
{
	 //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
	 KALMAN->Now_P = KALMAN->LastP + KALMAN->Q;
	 //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
	 KALMAN->Kg = KALMAN->Now_P / (KALMAN->Now_P + KALMAN->R);
	 //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
	 KALMAN->out = KALMAN->out + KALMAN->Kg * (input -KALMAN->out);//因为这一次的预测值就是上一次的输出值
	 //更新协方差方程: 本次的系统协方差赋值给 kfp->LastP 为下一次运算准备。
	 KALMAN->LastP = (1-KALMAN->Kg) * KALMAN->Now_P;
	 return KALMAN->out;
}
/**
 *计算一个通道的一次基波幅值
*@param ADCCH_enum ch 		设置AD转换端口	
				ADCRES_enum resolution 设置AD转换位数，范围6位，8位，10位，12位
 *@return 卡尔曼滤波以后的基波幅值
 *@example FFT(Data_Buff) 计算Data_Buff数组内的基波幅值
*/
uint16 Fourier_Once(ADCCH_enum ch,ADCRES_enum resolution)
{
		uint16 Data_Buff[ADC_Samp_SIZE]={0};
		float Result_WithoutFilt;
		uint16 Result_WithFilt;
		//将2个周期的采样信号输出至Data_Buff
		Fourier_DMAReadBuff(ch,resolution,Data_Buff);		
		//计算基波幅值并卡尔曼滤波
		
		Result_WithoutFilt=(float)FFT(Data_Buff);		//提取基波幅值原始值
		Result_WithoutFilt=Result_WithoutFilt<=0?0:Result_WithoutFilt;	//限幅
		
		Result_WithFilt = (uint16)kalmanFilter(&kalman_Data[ch],Result_WithoutFilt);			//卡尔曼滤波后返回基波幅值
		return Result_WithFilt;
}