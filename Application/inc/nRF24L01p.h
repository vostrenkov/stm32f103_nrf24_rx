

#ifndef __NRF24L01P_H
#define __NRF24L01P_H

/*

Pins	Name			In/Out		Default Value
PA3		CE				Out				Low
PA12	SPI_NSS		Out				High
PA5		SPI_SCK		Out				Low
PA6		SPI_MISO	In				~
PA7		SPI_MOSI	Out				~
PA8		IRQ				In				Float

P.S. SPI -> SPI 1

*/	
#include "defines.h"

#define NSS_Pin	GPIO_Pin_2
#define CE_Pin GPIO_Pin_3
#define IRQ_Pin GPIO_Pin_12

#define DEBUG_EN 0

//extern volatile DMA_InitTypeDef  DMA_SPIStructure;
//extern volatile SPI_InitTypeDef  SPI_InitStructure;

#if DEBUG_EN == 1
#define debugger(...) printf(__VA_ARGS__);
#else
#define debugger(...)
#endif
	
#include <stdint.h>

/* NRF24L01P commands */

#define CMD_R_REGISTER					(uint8_t) (0x00)
#define CMD_W_REGISTER					(uint8_t) (0x20)	
#define CMD_R_RX_PAYLOAD 				(uint8_t) (0x61)
#define CMD_W_TX_PAYLOAD 				(uint8_t) (0xA0)	
#define CMD_FLUSH_TX						(uint8_t) (0xE1)	
#define CMD_FLUSH_RX						(uint8_t) (0xE2)	
#define CMD_REUSE_TX_PL					(uint8_t) (0xE3)	
#define CMD_R_RX_PL_WID					(uint8_t) (0x60)	
#define CMD_W_ACK_PAYLOAD				(uint8_t) (0xA8)	
#define CMD_W_TX_PAYLOAD_NO_ACK	(uint8_t) (0xB0)	
#define CMD_NOP									(uint8_t) (0xFF)

/************************************************ Registers of NRF24L01P ***************************************/

	/* CONFIG REGISTER */ 
#define CONFIG_REG				(uint8_t)	(0x00)

#define CONFIG_MASK_RX_DR (uint8_t) (1<<6)
#define CONFIG_MASK_TX_DS (uint8_t) (1<<5)
#define CONFIG_MASK_MAX_RT (uint8_t) (1<<4)
#define CONFIG_EN_CRC			(uint8_t) (1<<3)	
#define CONFIG_CRCO				(uint8_t) (1<<2)
#define CONFIG_PWR_UP			(uint8_t) (1<<1)
#define CONFIG_PRIM_RX		(uint8_t) (1<<0)

	/* EN_AA REGISTER */	// Enable ‘Auto Acknowledgment’ Function Disable
#define EN_AA_REG 				(uint8_t)	(0x01)

#define EN_AA_REG_ENAA_P5 (uint8_t) (1<<5)
#define EN_AA_REG_ENAA_P4 (uint8_t) (1<<4)
#define EN_AA_REG_ENAA_P3 (uint8_t) (1<<3)
#define EN_AA_REG_ENAA_P2 (uint8_t) (1<<2)
#define EN_AA_REG_ENAA_P1 (uint8_t) (1<<1)
#define EN_AA_REG_ENAA_P0 (uint8_t) (1<<0)

	/* EN_RXADDR REGISTER */
#define EN_RXADDR_REG			(uint8_t)	(0x02)

#define EN_RXADDR_ERX_P5	(uint8_t) (1<<5)
#define EN_RXADDR_ERX_P4	(uint8_t) (1<<4)
#define EN_RXADDR_ERX_P3	(uint8_t) (1<<3)
#define EN_RXADDR_ERX_P2	(uint8_t) (1<<2)
#define EN_RXADDR_ERX_P1	(uint8_t) (1<<1)
#define EN_RXADDR_ERX_P0	(uint8_t) (1<<0)

	/* SETUP_AW REGISTER */
#define SETUP_AW_REG			(uint8_t)	(0x03)

#define SETUP_AW_AW_1			(uint8_t) (1<<1)	
#define SETUP_AW_AW_0			(uint8_t) (1<<0)

	/* SETUP_RETR REGISTER */
#define SETUP_RETR_REG		(uint8_t)	(0x04)

#define SETUP_RETR_ARD_3	(uint8_t) (1<<7)
#define SETUP_RETR_ARD_2	(uint8_t) (1<<6)
#define SETUP_RETR_ARD_1	(uint8_t) (1<<5)
#define SETUP_RETR_ARD_0	(uint8_t) (1<<4)
#define SETUP_RETR_ARC_3	(uint8_t) (1<<3)
#define SETUP_RETR_ARC_2	(uint8_t) (1<<2)
#define SETUP_RETR_ARC_1	(uint8_t) (1<<1)
#define SETUP_RETR_ARC_0	(uint8_t) (1<<0)

	/* RF_CH REGISTER */
#define	RF_CH_REG					(uint8_t)	(0x05)

	/* RF_SETUP REGISTER */
#define RF_SETUP_REG			(uint8_t)	(0x06)

#define RF_SETUP_CONT_WAVE	(uint8_t) (1<<7)
#define RF_SETUP_RF_DR_LOW	(uint8_t) (1<<5)
#define RF_SETUP_PLL_LOCK		(uint8_t) (1<<4)
#define RF_SETUP_RF_DR_HIGH	(uint8_t) (1<<3)
#define RF_SETUP_RF_PWR_1		(uint8_t) (1<<2)
#define RF_SETUP_RF_PWR_0		(uint8_t) (1<<1)

	/* STATUS REGISTER */
#define STATUS_REG				(uint8_t)	(0x07)

#define STATUS_RX_DR			(uint8_t) (1<<6)
#define STATUS_TX_DS			(uint8_t) (1<<5)
#define STATUS_MAX_RT			(uint8_t) (1<<4)
#define STATUS_TX_FULL 		(uint8_t) (1<<0)

	/* OBSERVE_TX REGISTER */
#define OBSERVE_TX_REG		(uint8_t)	(0x08)

	/* RPD REGISTER */
#define RPD_REG						(uint8_t)	(0x09)
	
#define RPD_RPD						(uint8_t) (1<<0)

	/* RX_ADDR_P REGISTERS */
#define RX_ADDR_P0_REG		(uint8_t)	(0x0A)
#define RX_ADDR_P1_REG		(uint8_t)	(0x0B)
#define RX_ADDR_P2_REG		(uint8_t)	(0x0C)
#define RX_ADDR_P3_REG		(uint8_t)	(0x0D)
#define RX_ADDR_P4_REG		(uint8_t)	(0x0E)
#define RX_ADDR_P5_REG		(uint8_t)	(0x0F)

	/* TX_ADDR REGISTER */
#define TX_ADDR_REG				(uint8_t)	(0x10)

	/* RX_PW_P REGISTERS */
#define RX_PW_P0_REG			(uint8_t)	(0x11)
#define RX_PW_P1_REG			(uint8_t)	(0x12)
#define RX_PW_P2_REG			(uint8_t)	(0x13)
#define RX_PW_P3_REG			(uint8_t)	(0x14)
#define RX_PW_P4_REG			(uint8_t)	(0x15)
#define RX_PW_P5_REG			(uint8_t)	(0x16)

	/* FIFO_STATUS REGISTER */
#define	FIFO_STATUS_REG		(uint8_t)	(0x17)

#define	FIFO_STATUS_TX_REUSE	(uint8_t) (1<<6)
#define	FIFO_STATUS_TX_FULL		(uint8_t) (1<<5)
#define	FIFO_STATUS_TX_EMPTY	(uint8_t) (1<<4)
#define	FIFO_STATUS_RX_FULL		(uint8_t) (1<<1)
#define	FIFO_STATUS_RX_EMPTY	(uint8_t) (1<<0)

	/* DYNPD REGISTER*/
#define	DYNPD_REG			(uint8_t)	(0x1C)

#define DYNPD_DPL_P5	(uint8_t) (1<<5)
#define DYNPD_DPL_P4	(uint8_t) (1<<4)
#define DYNPD_DPL_P3	(uint8_t) (1<<3)
#define DYNPD_DPL_P2	(uint8_t) (1<<2)
#define DYNPD_DPL_P1	(uint8_t) (1<<1)
#define DYNPD_DPL_P0	(uint8_t) (1<<0)

	/* FEATURE REGISTER */
#define	FEATURE_REG		(uint8_t) (0x1D)

#define FEATURE_EN_DPL			(uint8_t) (1<<2)
#define FEATURE_EN_ACK_PAY	(uint8_t) (1<<1)
#define FEATURE_EN_DYN_ACK	(uint8_t) (1<<0)

#define NSS_SET 		GPIO_ResetBits(GPIOA, NSS_Pin)
#define NSS_RESET 	GPIO_SetBits(GPIOA, NSS_Pin)

#define CE_SET			GPIO_SetBits(GPIOA, CE_Pin)
#define CE_RESET		GPIO_ResetBits(GPIOA, CE_Pin)

/**********************************************FUNCTIONS***********************************************/

void nRF24L01p_Config(void);
void nRF24L01pConfRxMode(void);	
void nRF24L01p_Send_CMD(uint8_t nRF_cmd, uint8_t* read_buf, uint8_t* write_buf, unsigned char rw_buf_len );


__forceinline static void spi_dma_rx_start(uint32_t* buf, uint32_t buflen){
		
		DMA1_Channel2->CMAR = (uint32_t) buf;
		DMA1_Channel2->CNDTR = (uint32_t) buflen;
		SPI_Cmd(SPI1, DISABLE);
		SPI1->CR1 |= (1 << 10);
		SPI_I2S_ClearFlag(SPI1,SPI_I2S_FLAG_RXNE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
		DMA_Cmd(DMA1_Channel2, ENABLE);
		SPI_Cmd(SPI1, ENABLE);
}

__forceinline static void spi_dma_rx_wait(void){

	while (!(DMA_GetFlagStatus(DMA1_FLAG_TC2)));
	SPI_Cmd(SPI1, DISABLE);
	DMA_ClearFlag(DMA1_FLAG_TC2);
	SPI_I2S_ClearFlag(SPI1,SPI_I2S_FLAG_RXNE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	SPI1->CR1 &= ~(1 << 10);
	SPI_Cmd(SPI1, ENABLE);
}

#endif
