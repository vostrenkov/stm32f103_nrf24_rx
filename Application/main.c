
/*

Прошивка содержит только код приемника
	SYSCLK_Frequency = 32000000	Hz
	HCLK_Frequency   = 32000000 Hz
	PCLK1_Frequency  = 16000000 Hz
	PCLK2_Frequency  = 16000000 Hz
*/

#include "defines.h"

GPIO_InitTypeDef 	GPIO_InitStructure;
RCC_ClocksTypeDef RCC_Clock;

volatile uint8_t sts_reg = 0;

uint8_t RxFIFO[2][32];
volatile uint8_t pntFIFO = 0;

int sendchar(int ch);
struct __FILE {int handle;};
FILE __stdout;
static void NVIC_config(void);

int main(void)
{
	
	RCC_GetClocksFreq(&RCC_Clock);
	
	SysTick_Config(RCC_Clock.SYSCLK_Frequency/1000);
	
	uart_config();
	NVIC_config();
	
	nRF24L01p_Config();
	nRF24L01pConfRxMode();
	
	CE_SET;

	for(;;)
	{
		
	}
}

static void NVIC_config(void)
{
	uint32_t nvic_status = 0;
	
	NVIC_DisableIRQ(EXTI15_10_IRQn);
	NVIC_DisableIRQ(DMA1_Channel4_IRQn);
	
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
	nvic_status = NVIC_GetPriorityGrouping();
	
	NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(nvic_status,1,0));
	NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(nvic_status,2,0));
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}


int fputc(int ch, FILE *f) 
{
	return (sendchar(ch));
}

int sendchar(int ch)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, ch);
	return 0;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
