#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_exti.h"

/* user inc */
#include "nRF24L01p.h"
#include "uart.h"

#include <stdio.h>
#include <stdlib.h>

#define RxMode 0
#define TxMode 1

#define nRF_Mode TxMode


#endif
