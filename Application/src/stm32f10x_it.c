/**
  ******************************************************************************
  * @file    EXTI/EXTI_Example/stm32l1xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include "nRF24L01p.h"
#include "uart.h"
/** @addtogroup EXTI_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
//	GPIOA->ODR ^= NSS_Pin;	
}

/******************************************************************************/
/*            STM32L1xx Peripherals Interrupt Handlers                        */
/******************************************************************************/
 


/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
extern uint8_t RxFIFO[2][768];
extern uint8_t pntFIFO;

extern volatile uint8_t sts_reg;

extern void SetSysClock(void);

void DMA1_Channel4_IRQHandler(void){
	
	if (DMA_GetFlagStatus(DMA1_FLAG_TC4) == SET){
		DMA_ClearFlag(DMA1_FLAG_TC4);
		NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
}


void EXTI15_10_IRQHandler(void){
	uint8_t reBuf, wrBuf, i;
	
  if(EXTI_GetITStatus(EXTI_Line12) != RESET){	/* IRQ is worked	*/
    EXTI_ClearITPendingBit(EXTI_Line12);
		
		nRF24L01p_Send_CMD( STATUS_REG, 0, 0, 0);
		
		if (sts_reg & STATUS_MAX_RT){
			wrBuf = STATUS_MAX_RT;
			nRF24L01p_Send_CMD( CMD_W_REGISTER | STATUS_REG, &reBuf, &wrBuf , 1);
			debugger("STATUS_MAX_RT SET \r\n");
		}
		
		if (sts_reg & STATUS_RX_DR){
			nRF24L01p_Send_CMD(CMD_R_RX_PAYLOAD, (uint8_t*)&RxFIFO[pntFIFO][0], 0, 32);
			
			usart_dma_tx_start((uint32_t*)&RxFIFO[pntFIFO][0], 32);
			if (pntFIFO == 0)
				pntFIFO = 1;
			else
				pntFIFO = 0;
			wrBuf = STATUS_RX_DR;
			nRF24L01p_Send_CMD( CMD_W_REGISTER | STATUS_REG, &reBuf, &wrBuf , 1);
			
			nRF24L01p_Send_CMD( FIFO_STATUS_REG, &reBuf, 0x00, 1);
		}
		if (sts_reg & STATUS_TX_DS){
			wrBuf = STATUS_TX_DS;
			nRF24L01p_Send_CMD( CMD_W_REGISTER | STATUS_REG, &reBuf, &wrBuf , 1);	
			wrBuf = STATUS_TX_DS;
			nRF24L01p_Send_CMD( CMD_W_REGISTER | STATUS_REG, &reBuf, &wrBuf , 1);
		}
  }
}


/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
