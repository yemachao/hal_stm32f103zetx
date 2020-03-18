#include "spi_flash.h"
#include "spi.h"
#include <stdio.h>

 /**
  * @brief  SPI写入读取数据
  * @param	byte 写入的数据
  * @retval 读取的数据
  */
uint8_t SPI2_ReadWriteByte(uint8_t byte)
{
  uint8_t res=0x00;
  HAL_SPI_TransmitReceive(&hspi2,&byte,&res,1,HAL_MAX_DELAY);
  return res;
}
 /**
  * @brief  FLASH读取ID
  * @param	无
  * @retval FLASH ID
  */
uint32_t SPI_FLASH_ReadID(void)
{
  uint32_t temp=0x00;
  SPI2_NSS=RESET;
  SPI2_ReadWriteByte(W25X_JedecDeviceID);
  temp|=(SPI2_ReadWriteByte(Dummy_Byte)<<16);
  temp|=(SPI2_ReadWriteByte(Dummy_Byte)<<8);
  temp|=SPI2_ReadWriteByte(Dummy_Byte);
  SPI2_NSS=SET;
  return temp;
}
 /**
  * @brief  FLASH等待写入完成
  * @param	无
  * @retval 无
  */
void SPI_FLASH_WaitForWriteEnd(void)
{
  uint8_t temp00=0x00;
  SPI2_NSS=RESET;
  SPI2_ReadWriteByte(W25X_ReadStatusReg);
  do{
    temp00=SPI2_ReadWriteByte(Dummy_Byte);
  }while((temp00&WIP_Flag)==SET);
  SPI2_NSS=SET;
}
 /**
  * @brief  FLASH写使能
  * @param	无
  * @retval 无
  */
void SPI_FLASH_WriteEnable(void)
{
  SPI2_NSS=RESET;
  SPI2_ReadWriteByte(W25X_WriteEnable);
  SPI2_NSS=SET;
}
 /**
  * @brief  FLASH擦除扇区数据
  * @param	SectorAddr 擦除扇区的地址
  * @retval 无
  */
void SPI_FLASH_SectorErase(uint32_t SectorAddr)
{
  SPI_FLASH_WriteEnable();
  SPI_FLASH_WaitForWriteEnd();
  SPI2_NSS=RESET;
  SPI2_ReadWriteByte(W25X_SectorErase);
  SPI2_ReadWriteByte((SectorAddr&0xFF0000)>>16);
  SPI2_ReadWriteByte((SectorAddr&0xFF00)>>8);
  SPI2_ReadWriteByte(SectorAddr&0xFF);
  SPI2_NSS=SET;
  SPI_FLASH_WaitForWriteEnd();
}
 /**
  * @brief  FLASH页写入函数
  * @param	pBuffer 写入数据的指针
  * @param  WriteAddr 写入地址
  * @param  NumByteToWrite 写入数据长度（页写入长度不能大于256）
  * @retval 无
  */
void SPI_FLASH_PageWrite(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
  SPI_FLASH_WriteEnable();
  SPI2_NSS=RESET;
  SPI2_ReadWriteByte(W25X_PageProgram);
  SPI2_ReadWriteByte((WriteAddr&0xFF0000)>>16);
  SPI2_ReadWriteByte((WriteAddr&0xFF00)>>8);
  SPI2_ReadWriteByte(WriteAddr&0xFF);
  if(NumByteToWrite>256){
    NumByteToWrite=256;
    printf("Error:FLASH Data long len max!\n");
  }
  while(NumByteToWrite--){
    SPI2_ReadWriteByte(*pBuffer);
    pBuffer++;
  }
  SPI2_NSS=SET;
  SPI_FLASH_WaitForWriteEnd();
}
 /**
  * @brief  从FLASH写入不定长数据
  * @param	pBuffer 写入数据的指针
  * @param  WriteAddr 写入地址
  * @param  NumByteToWrite 写入数据长度
  * @retval 无
  */
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage=0,NumOfSingle=0,Addr=0,count=0,temp=0;
  Addr=WriteAddr%SPI_FLASH_PageSize;
  count=SPI_FLASH_PageSize-Addr;
  NumOfPage=NumByteToWrite/SPI_FLASH_PageSize;
  NumOfSingle=NumByteToWrite%SPI_FLASH_PageSize;
  if(Addr==0){
    if(NumOfPage==0){
      SPI_FLASH_PageWrite(pBuffer,WriteAddr,NumByteToWrite);
    }else{
      while(NumOfPage--){
        SPI_FLASH_PageWrite(pBuffer,WriteAddr,SPI_FLASH_PageSize);
        WriteAddr+=SPI_FLASH_PageSize;
        pBuffer+=SPI_FLASH_PageSize;
      }
      SPI_FLASH_PageWrite(pBuffer,WriteAddr,NumOfSingle);
    }
  }else{
    if(NumOfPage==0){
      if(NumOfSingle>count){
        temp=NumOfSingle-count;
        SPI_FLASH_PageWrite(pBuffer,WriteAddr,count);
        WriteAddr++;
        pBuffer++;
        SPI_FLASH_PageWrite(pBuffer,WriteAddr,temp);
      }else{
        SPI_FLASH_PageWrite(pBuffer,WriteAddr,NumByteToWrite);
      }
    }else{
      NumByteToWrite-=count;
      NumOfPage=NumByteToWrite/SPI_FLASH_PageSize;
      NumOfSingle=NumByteToWrite%SPI_FLASH_PageSize;
      SPI_FLASH_PageWrite(pBuffer,WriteAddr,count);
      WriteAddr+=count;
      pBuffer+=count;
      while (NumOfPage--){
        SPI_FLASH_PageWrite(pBuffer, WriteAddr,
        SPI_FLASH_PageSize);
        WriteAddr += SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
      if (NumOfSingle != 0){
        SPI_FLASH_PageWrite(pBuffer, WriteAddr,
        NumOfSingle);
      }
    }
  }
}
 /**
  * @brief  从FLASH读取不定长数据
  * @param	pBuffer 存放读取数据的指针
  * @param  WriteAddr 读取地址
  * @param  NumByteToRead 读取数据长度
  * @retval 无
  */
void SPI_FLASH_BufferRead(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)
{
  SPI2_NSS=RESET;
  SPI2_ReadWriteByte(W25X_ReadData);
  SPI2_ReadWriteByte((ReadAddr&0xFF0000)>>16);
  SPI2_ReadWriteByte((ReadAddr&0xFF00)>>8);
  SPI2_ReadWriteByte(ReadAddr&0xFF);
  while(NumByteToRead--){
    *pBuffer=SPI2_ReadWriteByte(Dummy_Byte);
    pBuffer++;
  }
  SPI2_NSS=SET;
}

 /**
  * @brief  擦除FLASH扇区，整片擦除
  * @param  无
  * @retval 无
  */
void SPI_FLASH_BulkErase(void)
{
  SPI_FLASH_WriteEnable();
  SPI2_NSS=RESET;
  SPI2_ReadWriteByte(W25X_ChipErase);
  SPI2_NSS=SET;
  SPI_FLASH_WaitForWriteEnd();
}