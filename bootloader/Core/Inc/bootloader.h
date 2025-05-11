/*
 * bootloader.h
 *
 *  Created on: May 7, 2025
 *      Author: Mohamad
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_
#include "main.h"
#include <stdint.h>

#define FIRMWARE1_BASE_ADDR 0x08008000
#define FIRMWARE2_BASE_ADDR 0x08040000
#define FIRMWARE_BASE_ADDR 0x08008000

void jumpToApp(uint32_t AppAddress);
void jumpToFirmware1(void);
void jumpToFirmware2(void);



#endif /* INC_BOOTLOADER_H_ */
