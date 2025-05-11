/*
 * usart_debuger.c
 *
 *  Created on: May 7, 2025
 *      Author: Mohamad
 */


#include "usart_debuger.h"
#include "string.h"

char debugStr[30];
void debugTransmit(void){
	HAL_UART_Transmit(&huart3, (uint8_t *)debugStr, strlen(debugStr), 100);
}
