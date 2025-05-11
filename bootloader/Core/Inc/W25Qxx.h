/*
 * W25Qxx.h
 *
 *  Created on: Apr 14, 2025
 *      Author: Mohamad
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

// -----------general Functions
void W25Q_Delay(uint32_t time);
void csLOW (void);
void csHIGH (void);
void SPI_Write (uint8_t *data, uint8_t len);
void SPI_Read (uint8_t *data, uint32_t len);
void W25Q_Reset(void);
uint32_t W25Q_ReadID (void);
//------------




void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void write_enable (void);
void write_disable(void);
uint32_t bytestowrite (uint32_t size, uint16_t offset);
uint32_t bytestomodify (uint32_t size, uint16_t offset);
void W25Q_Erase_Sector (uint16_t numsector);
void W25Q_Write_Clean (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
void W25Q_Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
uint8_t W25Q_Read_Byte (uint32_t Addr);
void W25Q_Write_Byte (uint32_t Addr, uint8_t data);
void W25Q_Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);


#endif /* INC_W25QXX_H_ */
