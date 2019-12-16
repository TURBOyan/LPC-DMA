/*
 * @brief State Configurable Timer (SCT) PWM example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"

#define SCT_PWM            LPC_SCT

#define NUM_BUFFERS 4
#define DMA_TRANSFER_SIZE 8
#define ADC_INPUT_CHANNEL 1

#define SCT_PWM_RATE   10000		/* PWM frequency 10 KHz */
#define SCT_PWM_PIN_OUT    7		/* COUT7 Generate square wave */
#define SCT_PWM_OUT        1		/* Index of OUT PWM */

uint16_t adcOut;

ALIGN(512) DMA_CHDESC_T ADC_TransferDescriptors[NUM_BUFFERS];

//uint16_t CapturedData[NUM_BUFFERS * DMA_TRANSFER_SIZE];
uint16_t CapturedData[32];

uint16_t DMA_Sum=0;

/**
 *
 * ADC IRQ not Used right now... Only for testing
 */
void ADC_SEQA_IRQHandler(void)
{
            /* If SeqA flags is set i.e. data in global register is valid then read it */
//    if(Chip_ADC_GetFlags(LPC_ADC) & ADC_FLAGS_SEQA_INT_MASK) {
//        adcOut = (Chip_ADC_GetGlobalDataReg(LPC_ADC, ADC_SEQA_IDX) & ADC_SEQ_GDAT_RESULT_MASK) >> ADC_SEQ_GDAT_RESULT_BITPOS;       
//    }
        Chip_GPIO_SetPinState(LPC_GPIO, 0, 6, true);
        //DEBUGOUT("ADC Output = %d\r\n", adcOut);
        Chip_GPIO_SetPinState(LPC_GPIO, 0, 6, false);
        Chip_ADC_ClearFlags(LPC_ADC,0xFFFFFFFF);
}


void DMA_IRQHandler(void)
{
        static uint16_t DMA_Sum=0;
        
        DMA_Sum++;
        
         if(DMA_Sum ==8)
         {
           DMA_Sum=4;
         }

       
	Chip_GPIO_SetPinState(LPC_GPIO, 0, 7,true);

	/* Rrror interrupt on channel 0? */
	if ((Chip_DMA_GetIntStatus(LPC_DMA) & DMA_INTSTAT_ACTIVEERRINT) != 0)
	{
		/* This shouldn't happen for this simple DMA example, so set the LED
		   to indicate an error occurred. This is the correct method to clear
		   an abort. */
		Chip_DMA_DisableChannel(LPC_DMA, DMA_CH0);
		while ((Chip_DMA_GetBusyChannels(LPC_DMA) & (1 << DMA_CH0)) != 0) {}
		Chip_DMA_AbortChannel(LPC_DMA, DMA_CH0);
		Chip_DMA_ClearErrorIntChannel(LPC_DMA, DMA_CH0);
		Chip_DMA_EnableChannel(LPC_DMA, DMA_CH0);
		Board_LED_Set(0, true);
	}

	Chip_GPIO_SetPinState(LPC_GPIO, 0,7,false);

	/* Clear DMA interrupt for the channel */
	LPC_DMA->DMACOMMON[0].INTA = 1;
}





	/***
	 *      ____  __  __    _
	 *     |  _ \|  \/  |  / \
	 *     | | | | |\/| | / _ \
	 *     | |_| | |  | |/ ___ \
	 *     |____/|_|  |_/_/   \_\
	 *     / ___|  ___| |_ _   _ _ __
	 *     \___ \ / _ \ __| | | | '_ \
	 *      ___) |  __/ |_| |_| | |_) |
	 *     |____/ \___|\__|\__,_| .__/
	 *                          |_|
	 */
void DMA_Steup(void)
{
  DMA_CHDESC_T Initial_DMA_Descriptor;
        
  ADC_TransferDescriptors[0].source = (uint32_t)&LPC_ADC->SEQ_GDAT[0];
	ADC_TransferDescriptors[1].source = (uint32_t)&LPC_ADC->SEQ_GDAT[0];
	ADC_TransferDescriptors[2].source = (uint32_t)&LPC_ADC->SEQ_GDAT[0];
	ADC_TransferDescriptors[3].source = (uint32_t)&LPC_ADC->SEQ_GDAT[0];


	ADC_TransferDescriptors[0].dest = (uint32_t)&CapturedData[(0+1)*DMA_TRANSFER_SIZE-1];
	ADC_TransferDescriptors[1].dest = (uint32_t)&CapturedData[(1+1)*DMA_TRANSFER_SIZE-1];
	ADC_TransferDescriptors[2].dest = (uint32_t)&CapturedData[(2+1)*DMA_TRANSFER_SIZE-1];
	ADC_TransferDescriptors[3].dest = (uint32_t)&CapturedData[(3+1)*DMA_TRANSFER_SIZE-1];

	//The initial DMA desciptor is the same as the 1st transfer descriptor.   It
	//Will link into the 2nd of the main descriptors.

	ADC_TransferDescriptors[0].next = (uint32_t)&ADC_TransferDescriptors[1];
	ADC_TransferDescriptors[1].next = (uint32_t)&ADC_TransferDescriptors[2];
	ADC_TransferDescriptors[2].next = (uint32_t)&ADC_TransferDescriptors[3];

	//Link back to the 1st descriptor
	ADC_TransferDescriptors[3].next = (uint32_t)&ADC_TransferDescriptors[0];

	//For a test,  stop the transfers here.   The sine wave will look fine.
	//ADC_TransferDescriptors[3].next = 0;

	ADC_TransferDescriptors[0].xfercfg = (DMA_XFERCFG_CFGVALID |
					      DMA_XFERCFG_RELOAD  |
					      DMA_XFERCFG_SETINTA |
					      DMA_XFERCFG_WIDTH_16 |
					      DMA_XFERCFG_SRCINC_0 |
					      DMA_XFERCFG_DSTINC_1 |
					      DMA_XFERCFG_XFERCOUNT(DMA_TRANSFER_SIZE));

	ADC_TransferDescriptors[1].xfercfg = ADC_TransferDescriptors[0].xfercfg;
	ADC_TransferDescriptors[2].xfercfg = ADC_TransferDescriptors[0].xfercfg;

	ADC_TransferDescriptors[3].xfercfg = (DMA_XFERCFG_CFGVALID |
					      DMA_XFERCFG_RELOAD  |
					      DMA_XFERCFG_SETINTA |
						DMA_XFERCFG_WIDTH_16 |
						DMA_XFERCFG_SRCINC_0 |
						DMA_XFERCFG_DSTINC_1 |
						DMA_XFERCFG_XFERCOUNT(DMA_TRANSFER_SIZE));

	Initial_DMA_Descriptor.source = ADC_TransferDescriptors[0].source;
	Initial_DMA_Descriptor.dest =   ADC_TransferDescriptors[0].dest;
	Initial_DMA_Descriptor.next =  (uint32_t)&ADC_TransferDescriptors[1];
	Initial_DMA_Descriptor.xfercfg = ADC_TransferDescriptors[0].xfercfg;

	/* DMA initialization - enable DMA clocking and reset DMA if needed */
	Chip_DMA_Init(LPC_DMA);

	/* Enable DMA controller and use driver provided DMA table for current descriptors */
	Chip_DMA_Enable(LPC_DMA);
	Chip_DMA_SetSRAMBase(LPC_DMA, DMA_ADDR(Chip_DMA_Table));

	/* Setup channel 0 for the following configuration:
	   - High channel priority
	   - Interrupt A fires on descriptor completion */
	Chip_DMA_EnableChannel(LPC_DMA, DMA_CH0);
	Chip_DMA_EnableIntChannel(LPC_DMA, DMA_CH0);
	Chip_DMA_SetupChannelConfig(LPC_DMA, DMA_CH0,	//(DMA_CFG_PERIPHREQEN     |
															(DMA_CFG_HWTRIGEN        |
															 DMA_CFG_TRIGBURST_BURST |
															 DMA_CFG_TRIGTYPE_EDGE   |
															 DMA_CFG_TRIGPOL_LOW     |    //DMA_CFG_TRIGPOL_HIGH
															 DMA_CFG_BURSTPOWER_1    |
															 DMA_CFG_CHPRIORITY(0)
																 )
							    );


	//make sure ADC Sequence A interrupts is selected for for a DMA trigger
	LPC_INMUX->DMA_ITRIG_INMUX[0] = 0;

	/* Enable DMA interrupt */
	NVIC_EnableIRQ(DMA_IRQn);

	// The 1st descriptor is set up through the registers.

	/* Setup transfer descriptor and validate it */
	Chip_DMA_SetupTranChannel(LPC_DMA, DMA_CH0, &Initial_DMA_Descriptor);

	//Use the transfer configuration for our 4 main descriptors
	Chip_DMA_SetupChannelTransfer(LPC_DMA, DMA_CH0,	ADC_TransferDescriptors[0].xfercfg);
	Chip_DMA_SetValidChannel(LPC_DMA, DMA_CH0);      
}

void SCT_PWM_Generate(void)
{
    	/* Initialize the SCT as PWM and set frequency */
	Chip_SCTPWM_Init(SCT_PWM);
	Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);

	/* Setup Board specific output pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 14, IOCON_FUNC3 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
	/* Use SCT0_OUT7 pin */
	Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_OUT, SCT_PWM_PIN_OUT);

        
	/* Start with 50% duty cycle */
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT, Chip_SCTPWM_PercentageToTicks(SCT_PWM, 10));
	Chip_SCTPWM_Start(SCT_PWM);    
}


	/***
		 *         _    ____   ____
		 *        / \  |  _ \ / ___|
		 *       / _ \ | | | | |
		 *      / ___ \| |_| | |___
		 *     /_/__ \_\____/ \____|
		 *     / ___|  ___| |_ _   _ _ __
		 *     \___ \ / _ \ __| | | | '_ \
		 *      ___) |  __/ |_| |_| | |_) |
		 *     |____/ \___|\__|\__,_| .__/
		 *                          |_|
		 */
void ADC_Steup(void)
{
    /*Set Asynch Clock to the Main clock*/
    LPC_SYSCON->ADCCLKSEL = 0;
    //Set the divider to 1 and enable.  note,  the HALT bit (30) and RESET (29) are not in the manual
    LPC_SYSCON->ADCCLKDIV = 0;
     /* Initialization ADC to 12 bit and set clock divide to 1 to operate synchronously at System clock */
    Chip_ADC_Init(LPC_ADC, ADC_CR_RESOL(3) | ADC_CR_CLKDIV(0)| ADC_CR_ASYNC_MODE);   
    //select ADC Channel 1 as input
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 30, IOCON_FUNC0 | IOCON_ANALOG_EN| IOCON_INPFILT_OFF);   
    LPC_ADC->INSEL = 0x01;
    Chip_ADC_SetupSequencer(LPC_ADC,ADC_SEQA_IDX,		    						
						ADC_SEQ_CTRL_SEQ_ENA |
						ADC_SEQ_CTRL_CHANNEL_EN(ADC_INPUT_CHANNEL) |
                                                ADC_SEQ_CTRL_TRIGGER(2) |
						ADC_SEQ_CTRL_HWTRIG_POLPOS |
                                                ADC_SEQ_CTRL_HWTRIG_SYNCBYPASS |
						ADC_SEQ_CTRL_MODE_EOS |
                                                ADC_SEQ_CTRL_SEQ_ENA);
    /* Enable Sequence A interrupt */
    Chip_ADC_EnableInt(LPC_ADC, ADC_INTEN_SEQA_ENABLE);
    
    /* Calibrate ADC */
    if(Chip_ADC_Calibration(LPC_ADC) == LPC_OK) {
        /* Enable ADC SeqA Interrupt */
        NVIC_EnableIRQ(ADC_SEQA_IRQn);
    }
    else {
        DEBUGSTR("ADC Calibration Failed \r\n");
        return ;
    }
}

int main(void)
{
  
    SystemCoreClockUpdate();
    Board_Init();
    
    DMA_Steup();
    ADC_Steup();
    SCT_PWM_Generate();

    
    while(1)
    {}
    
}






