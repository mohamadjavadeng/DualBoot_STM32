/*
 * bootloader.c
 *
 *  Created on: May 7, 2025
 *      Author: Mohamad
 */


#include "bootloader.h"


typedef void (*pFunction)(void);

void jumpToApp(uint32_t AppAddress){
	uint32_t resetHandlerAddr = *(volatile uint32_t*)(AppAddress + 4); 	//Reset Handler Address
	pFunction appResetHandler = (pFunction)resetHandlerAddr;			//Reset Handler Function

	HAL_RCC_DeInit();
	HAL_DeInit();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	SCB->VTOR = AppAddress;
	uint32_t msp_value = *((volatile uint32_t *)(AppAddress));
	__set_MSP(msp_value);
	appResetHandler();

}

// two Examples function for two different Addresses
void jumpToFirmware1(void) {
    jumpToApp(FIRMWARE1_BASE_ADDR);
}

void jumpToFirmware2(void) {
    jumpToApp(FIRMWARE2_BASE_ADDR);
}
