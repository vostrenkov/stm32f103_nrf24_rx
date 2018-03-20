
#ifndef __USER_UART_H
#define __USER_UART_H

#include "defines.h"

void uart_config(void);

#define SLEEP_Pin	GPIO_Pin_10

extern volatile DMA_InitTypeDef  DMA_USARTStructure;

__forceinline static void usart_dma_tx_start(uint32_t* buf, uint32_t buflen){
	
	NVIC_DisableIRQ(EXTI15_10_IRQn);
	DMA_ClearFlag(DMA1_FLAG_TC4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA1_Channel4->CMAR = (uint32_t) buf;
	DMA1_Channel4->CNDTR = buflen;
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

__forceinline static void usart_dma_tx_wait(void){
	
	while(DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
	DMA_DeInit(DMA1_Channel4);
}

#endif
