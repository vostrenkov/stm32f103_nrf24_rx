

#include "nRF24L01p.h"
#include <stdlib.h>

#include <stdio.h>


#define RF_CH_VALUE	100 /* F0= 2400 + RF_CH_VALUE [MHz] */

#define AUTO_ACK 0


static uint8_t nRF24L01p_SPI_RW(uint8_t byte);

extern GPIO_InitTypeDef GPIO_InitStructure;
extern volatile uint8_t sts_reg;

DMA_InitTypeDef  DMA_SPIStructure;
volatile SPI_InitTypeDef  SPI_InitStructure;	

void nRF24L01p_Config(void){
		
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* IRQ port */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* Enable the SPI peripheral */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
  /* Enable the DMA peripheral */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable SCK, MOSI, MISO, NSS, IRQ, CE GPIO clocks */
	
	/* CE config */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = CE_Pin;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	CE_RESET;
	

	
	/* IRQ config */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = IRQ_Pin;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		/* EXT IRQ Interrupt */
	
		/* Connect EXTI12 Line to IRQ pin  pin */
		
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);
	
		/* Configure EXTI12 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  /* SPI */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  /* SPI NSS pin configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);		
	NSS_RESET;
		
	/* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;

	 /* Initializes the SPI communication */
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(SPI1, (SPI_InitTypeDef*) &SPI_InitStructure);
	SPI_CalculateCRC(SPI1, DISABLE);
	SPI_I2S_ClearFlag(SPI1,SPI_I2S_FLAG_RXNE);
	SPI_Cmd(SPI1, ENABLE);
	
		/* DMA Configuration -------------------------------------------------------*/
	DMA_DeInit(DMA1_Channel2);
	DMA_SPIStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_SPIStructure.DMA_MemoryDataSize =  DMA_MemoryDataSize_Byte;
	DMA_SPIStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_SPIStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_SPIStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_SPIStructure.DMA_M2M = DMA_M2M_Disable;
	
	DMA_SPIStructure.DMA_BufferSize = (uint32_t) 32;
	DMA_SPIStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(SPI1->DR));
	DMA_SPIStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_SPIStructure.DMA_Priority = DMA_Priority_VeryHigh;

	DMA_Init(DMA1_Channel2, (DMA_InitTypeDef*)&DMA_SPIStructure);
}



/************************************************RX MODE*****************************************************/
void nRF24L01pConfRxMode(void){
	uint8_t addr[5] = "0epip";
	uint8_t tmp_buf[6] = "";	
	
	uint8_t reBuf, wrBuf;
	uint32_t i = 0;
	
	debugger("Rx Mode\r\n");
	
	/* CONFIG */
	wrBuf =  CONFIG_PWR_UP | CONFIG_EN_CRC | CONFIG_CRCO |CONFIG_PRIM_RX;
	nRF24L01p_Send_CMD( CMD_W_REGISTER | CONFIG_REG, NULL, &wrBuf, 1);
	i=0; while(++i!=10000);	
	nRF24L01p_Send_CMD(CMD_R_REGISTER | CONFIG_REG, &reBuf, NULL, 1);	
	debugger("CONFIG %02x\r\n",reBuf);

	/* EN_AA */
#if AUTO_ACK == 1
	wrBuf = 0x01;
#else
	wrBuf = 0x00;
#endif
	nRF24L01p_Send_CMD( CMD_W_REGISTER | EN_AA_REG , NULL, &wrBuf, 1);
	i=0; while(++i!=10000);	
	nRF24L01p_Send_CMD(CMD_R_REGISTER | EN_AA_REG, &reBuf, NULL, 1);	
	debugger("EN_AA %02x\r\n",reBuf);

	/* EN_RXADDR */	
	wrBuf = EN_RXADDR_ERX_P0;
	nRF24L01p_Send_CMD( CMD_W_REGISTER | EN_RXADDR_REG , NULL, &wrBuf, 1);
	nRF24L01p_Send_CMD( CMD_R_REGISTER | EN_RXADDR_REG, &reBuf, NULL, 1);
	debugger("EN_RXADDR %02x\r\n", reBuf);	
	
	/* SETUP_AW */
	wrBuf = SETUP_AW_AW_0 | SETUP_AW_AW_1;
	nRF24L01p_Send_CMD( CMD_W_REGISTER | SETUP_AW_REG , NULL, &wrBuf, 1);
	nRF24L01p_Send_CMD( CMD_R_REGISTER | SETUP_AW_REG, &reBuf, NULL, 1);
	debugger("SETUP_AW %02x\r\n", reBuf);
	
	/* RF_CH*/
	wrBuf = 100;
	nRF24L01p_Send_CMD( CMD_W_REGISTER | RF_CH_REG , NULL, &wrBuf, 1);
	nRF24L01p_Send_CMD( CMD_R_REGISTER | RF_CH_REG, &reBuf, NULL, 1);
	debugger("RF_CH %02x\r\n", reBuf);	
	
	/* RF_SETUP */
	wrBuf = RF_SETUP_RF_PWR_0 | RF_SETUP_RF_PWR_1 | RF_SETUP_RF_DR_HIGH;
  nRF24L01p_Send_CMD( CMD_W_REGISTER | RF_SETUP_REG, NULL, &wrBuf, 1);
	nRF24L01p_Send_CMD( CMD_R_REGISTER | RF_SETUP_REG, &reBuf, NULL, 1);
	debugger("RF_SETUP %02x\r\n", reBuf);

	/* STATUS */
	wrBuf = STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT;
  nRF24L01p_Send_CMD( CMD_W_REGISTER | STATUS_REG, NULL, &wrBuf, 1);
	nRF24L01p_Send_CMD( CMD_R_REGISTER | STATUS_REG, &reBuf, NULL, 1);
	debugger("STATUS %02x\r\n", reBuf);
	
	/* RX_ADDR_P0 */
	nRF24L01p_Send_CMD( CMD_W_REGISTER | RX_ADDR_P0_REG, NULL, addr, 5);
	nRF24L01p_Send_CMD( CMD_R_REGISTER | RX_ADDR_P0_REG, tmp_buf, NULL, 5);
	debugger("RX_ADDR_P0 %s\r\n", tmp_buf);	
	
	/* RX_PW_P0 */
	wrBuf = 32;
	nRF24L01p_Send_CMD( CMD_W_REGISTER | RX_PW_P0_REG, NULL, &wrBuf, 1);
	nRF24L01p_Send_CMD( CMD_R_REGISTER | RX_PW_P0_REG, &reBuf, NULL, 1); 
	debugger("RX_PW_P0 %02x\r\n", 	reBuf);
	
	/* RX FIFO */
	nRF24L01p_Send_CMD( CMD_FLUSH_RX, NULL, NULL, 0);
	nRF24L01p_Send_CMD( FIFO_STATUS_REG, &reBuf, NULL, 1);
	debugger("FIFO_STATUS_REG %02x\r\n", reBuf);
}


void nRF24L01p_Send_CMD(uint8_t nRF_cmd, uint8_t* read_buf, uint8_t* write_buf, unsigned char rw_buf_len ){
	uint32_t i = 0;
		
	NSS_SET;
	sts_reg = nRF24L01p_SPI_RW(nRF_cmd);
	
	switch(nRF_cmd){
		case CMD_R_RX_PAYLOAD:{
			spi_dma_rx_start((uint32_t*) read_buf, rw_buf_len );
			spi_dma_rx_wait();
			break;
		}
		default:{
			if (rw_buf_len!=0){
				/* After Read/Write operations*/ 
				for (i=0; i!=rw_buf_len; i++){
					if (write_buf == NULL){
						if (read_buf != NULL) 
							*(read_buf+i) = nRF24L01p_SPI_RW(NULL);
					}
					else
						nRF24L01p_SPI_RW(*(write_buf+i));
				}
			}			
			break;
		}
	}
	NSS_RESET;
}

static uint8_t nRF24L01p_SPI_RW(uint8_t byte){
	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
   SPI_I2S_SendData(SPI1, byte);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
   return SPI_I2S_ReceiveData(SPI1);
}
