#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "main.h"
#include "system.h"

#define SPI2_NSS                        PBout(12)
#define sFLASH_ID                       0XEF4018
#define SPI_FLASH_PageSize              256
#define SPI_FLASH_PerWritePageSize      256
/* FLASH 常用命令 */
#define W25X_WriteEnable                0x06
#define W25X_WriteDisable               0x04
#define W25X_ReadStatusReg              0x05
#define W25X_WriteStatusReg             0x01
#define W25X_ReadData                   0x03
#define W25X_FastReadData               0x0B
#define W25X_FastReadDual               0x3B
#define W25X_PageProgram                0x02
#define W25X_BlockErase                 0xD8
#define W25X_SectorErase                0x20
#define W25X_ChipErase                  0xC7
#define W25X_PowerDown                  0xB9
#define W25X_ReleasePowerDown           0xAB
#define W25X_DeviceID                   0xAB
#define W25X_ManufactDeviceID           0x90
#define W25X_JedecDeviceID              0x9F
/* 其它 */
#define Dummy_Byte                      0xFF
#define WIP_Flag                        0x01

uint8_t SPI2_ReadWriteByte(uint8_t byte);
uint32_t SPI_FLASH_ReadID(void);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_BufferRead(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_PageWrite(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void SPI_FLASH_SectorErase(uint32_t SectorAddr);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);
void SPI_FLASH_Test(void);

#endif /* __SPI_FLASH_H */