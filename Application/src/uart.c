

#include "uart.h"

#define bufSize 768

uint8_t txBuffer[bufSize]="Hello World";

extern GPIO_InitTypeDef 	GPIO_InitStructure;

volatile DMA_InitTypeDef  DMA_USARTStructure;

void uart_config(void){
  USART_InitTypeDef USART_InitStructure;
	
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
  /* Enable USART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
  /* Enable the DMA periph */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 2000000 baud  
  - Word Length = 8 Bits
  - one Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;	//2000000
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode =  USART_Mode_Tx;
	USART_OverSampling8Cmd(USART1, ENABLE);
  USART_Init(USART1, &USART_InitStructure);
  
	/* DMA Configuration -------------------------------------------------------*/
  DMA_USARTStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(USART1->DR));
  DMA_USARTStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_USARTStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_USARTStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_USARTStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_USARTStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_USARTStructure.DMA_M2M = DMA_M2M_Disable;
  
	/* DMA channel Tx of USART Configuration */
	DMA_DeInit(DMA1_Channel4);
	DMA_USARTStructure.DMA_BufferSize = (uint16_t) bufSize;
	DMA_USARTStructure.DMA_MemoryBaseAddr = (uint32_t) txBuffer;
	DMA_USARTStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_USARTStructure.DMA_Priority = DMA_Priority_High;
			
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);
	
	DMA_Init(DMA1_Channel4,(DMA_InitTypeDef*) &DMA_USARTStructure);
		
	DMA_ClearFlag(DMA1_FLAG_TC4);
	/* Enable DMA Stream Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}

