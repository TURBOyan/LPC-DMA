/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看LPC546XX_config.h文件内版本宏定义
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/
 
#ifndef _isr_h
#define _isr_h

#include "common.h"

#define SUCCESS 1
#define FAIL    0

extern uint8 FLAG_2MS,FLAG_5MS,FLAG_10MS,FLAG_50MS,FLAG_100MS,FLAG_200MS;

void RIT_DriverIRQHandler(void);																									

void PIN_INT7_DriverIRQHandler(void);
void DMA0_DriverIRQHandler(void);
void FLEXCOMM5_DriverIRQHandler(void);

#endif
