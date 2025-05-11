/*
 * W25Qxx.c
 *
 *  Created on: Apr 14, 2025
 *      Author: Mohamad
 */


#include "main.h"
#include "W25Qxx.h"

extern SPI_HandleTypeDef hspi2;
#define W25Q_SPI hspi2

/*#define csLOW() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)
#define csHIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)*/

#define numBLOCK 32 // number of totoal 16MB flash


void W25Q_Delay(uint32_t time)
{
	HAL_Delay(time);
}

void csLOW (void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

void csHIGH (void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

void SPI_Write (uint8_t *data, uint8_t len)
{
	HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
}

void SPI_Read (uint8_t *data, uint32_t len)
{
	HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
}

/**************************************************************************************************/

void W25Q_Reset (void)
{
	uint8_t tData[2];
	tData[0] = 0x66;  // enable Reset
	tData[1] = 0x99;  // Reset
	csLOW();
	SPI_Write(tData, 2);
	csHIGH();
	W25Q_Delay(100);
}

uint32_t W25Q_ReadID (void)
{
	uint8_t tData = 0x9F;  // Read JEDEC ID
	uint8_t rData[3];
	csLOW();
	SPI_Write(&tData, 1);
	SPI_Read(rData, 3);
	csHIGH();
	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x03;  // enable Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x13;  // Read Data with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI_Write(tData, 4);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI_Write(tData, 5);  // send read instruction along with the 32 bit memory address
	}

	SPI_Read(rData, size);  // Read the data
	csHIGH();  // pull the CS High
}

void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x0B;  // enable Fast Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
		tData[4] = 0;  // Dummy clock
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x0C;  // Fast Read with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
		tData[5] = 0;  // Dummy clock
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI_Write(tData, 5);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI_Write(tData, 6);  // send read instruction along with the 32 bit memory address
	}

	SPI_Read(rData, size);  // Read the data
	csHIGH();  // pull the CS High
}

void write_enable (void)
{
	uint8_t tData = 0x06;  // enable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_Delay(5);  // 5ms delay
}

void write_disable(void)
{
	uint8_t tData = 0x04;  // disable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_Delay(5);  // 5ms delay
}

uint32_t bytestowrite (uint32_t size, uint16_t offset)
{
	if ((size+offset)<256) return size;
	else return 256-offset;
}

uint32_t bytestomodify (uint32_t size, uint16_t offset)
{
	if ((size+offset)<4096) return size;
	else return 4096-offset;
}

void W25Q_Erase_Sector (uint16_t numsector)
{
	uint8_t tData[6];
	uint32_t memAddr = numsector*16*256;   // Each sector contains 16 pages * 256 bytes

	write_enable();

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x20;  // Erase sector
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address

		csLOW();
		SPI_Write(tData, 4);
		csHIGH();
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x21;  // ERASE Sector with 32bit address
		tData[1] = (memAddr>>24)&0xFF;
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = memAddr&0xFF;

		csLOW();  // pull the CS LOW
		SPI_Write(tData, 5);
		csHIGH();  // pull the HIGH
	}

	W25Q_Delay(100);  // 450ms delay for sector erase

	write_disable();

}


void W25Q_Write_Clean (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage-startPage+1;

	uint16_t startSector  = startPage/16;
	uint16_t endSector  = endPage/16;
	uint16_t numSectors = endSector-startSector+1;
	for (uint16_t i=0; i<numSectors; i++)
	{
		W25Q_Erase_Sector(startSector++);
	}

	uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i=0; i<numPages; i++)
	{
		uint32_t memAddr = (startPage*256)+offset;
		uint16_t bytesremaining  = bytestowrite(size, offset);
		uint32_t indx = 0;

		write_enable();

		if (numBLOCK<512)   // Chip Size<256Mb
		{
			tData[0] = 0x02;  // page program
			tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>8)&0xFF;
			tData[3] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 4;
		}

		else // we use 32bit memory address for chips >= 256Mb
		{
			tData[0] = 0x12;  // page program with 4-Byte Address
			tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>16)&0xFF;
			tData[3] = (memAddr>>8)&0xFF;
			tData[4] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 5;
		}

		uint16_t bytestosend  = bytesremaining + indx;

		for (uint16_t i=0; i<bytesremaining; i++)
		{
			tData[indx++] = data[i+dataPosition];
		}

		if (bytestosend > 250)
		{
			csLOW();
			SPI_Write(tData, 100);
			SPI_Write(tData+100, bytestosend-100);
			csHIGH();

		}

		else
		{
			csLOW();
			SPI_Write(tData, bytestosend);
			csHIGH();
		}


		startPage++;
		offset = 0;
		size = size-bytesremaining;
		dataPosition = dataPosition+bytesremaining;

		W25Q_Delay(5);
		write_disable();

	}
}

void W25Q_Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint16_t startSector  = page/16;
	uint16_t endSector  = (page + ((size+offset-1)/256))/16;
	uint16_t numSectors = endSector-startSector+1;

	uint8_t previousData[4096];
	uint32_t sectorOffset = ((page%16)*256)+offset;
	uint32_t dataindx = 0;

	for (uint16_t i=0; i<numSectors; i++)
	{
		uint32_t startPage = startSector*16;
		W25Q_FastRead(startPage, 0, 4096, previousData);

		uint16_t bytesRemaining = bytestomodify(size, sectorOffset);
		for (uint16_t i=0; i<bytesRemaining; i++)
		{
			previousData[i+sectorOffset] = data[i+dataindx];
		}

		W25Q_Write_Clean(startPage, 0, 4096, previousData);

		startSector++;
		sectorOffset = 0;
		dataindx = dataindx+bytesRemaining;
		size = size-bytesRemaining;
	}
}

uint8_t W25Q_Read_Byte (uint32_t Addr)
{
	uint8_t tData[5];
	uint8_t rData;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x03;  // enable Read
		tData[1] = (Addr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr)&0xFF; // LSB of the memory Address
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x13;  // Read Data with 4-Byte Address
		tData[1] = (Addr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>16)&0xFF;
		tData[3] = (Addr>>8)&0xFF;
		tData[4] = (Addr)&0xFF; // LSB of the memory Address
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI_Write(tData, 4);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI_Write(tData, 5);  // send read instruction along with the 32 bit memory address
	}

	SPI_Read(&rData, 1);  // Read the data
	csHIGH();  // pull the CS High

	return rData;
}

void W25Q_Write_Byte (uint32_t Addr, uint8_t data)
{
	uint8_t tData[6];
	uint8_t indx;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x02;  // page program
		tData[1] = (Addr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr)&0xFF; // LSB of the memory Address
		tData[4] = data;
		indx = 5;
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x12;  // Write Data with 4-Byte Address
		tData[1] = (Addr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>16)&0xFF;
		tData[3] = (Addr>>8)&0xFF;
		tData[4] = (Addr)&0xFF; // LSB of the memory Address
		tData[5] = data;
		indx = 6;
	}


	if (W25Q_Read_Byte(Addr) == 0xFF)
	{
		write_enable();
		csLOW();
		SPI_Write(tData, indx);
		csHIGH();

		W25Q_Delay(5);
		write_disable();
	}
}


void W25Q_Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage-startPage+1;

//	uint16_t startSector  = startPage/16;
//	uint16_t endSector  = endPage/16;
//	uint16_t numSectors = endSector-startSector+1;
	/*for (uint16_t i=0; i<numSectors; i++)
	{
		W25Q_Erase_Sector(startSector++);
	}
*/
	uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i=0; i<numPages; i++)
	{
		uint32_t memAddr = (startPage*256)+offset;
		uint16_t bytesremaining  = bytestowrite(size, offset);
		uint32_t indx = 0;

		write_enable();

		if (numBLOCK<512)   // Chip Size<256Mb
		{
			tData[0] = 0x02;  // page program
			tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>8)&0xFF;
			tData[3] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 4;
		}

		else // we use 32bit memory address for chips >= 256Mb
		{
			tData[0] = 0x12;  // page program with 4-Byte Address
			tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>16)&0xFF;
			tData[3] = (memAddr>>8)&0xFF;
			tData[4] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 5;
		}

		uint16_t bytestosend  = bytesremaining + indx;

		for (uint16_t i=0; i<bytesremaining; i++)
		{
			tData[indx++] = data[i+dataPosition];
		}

		csLOW();
		if (bytestosend > 200)
		{
			SPI_Write(tData, 100);
			SPI_Write(tData+100, bytestosend-100);
		}

		else
		{
			SPI_Write(tData, bytestosend);
		}
		csHIGH();
		startPage++;
		offset = 0;
		size = size-bytesremaining;
		dataPosition = dataPosition+bytesremaining;

		W25Q_Delay(5);
		write_disable();

	}
}
