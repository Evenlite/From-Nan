/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : Flash_EEPROM.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  *2022 NAN LIU 
  *
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Flash_EEPROM.h"
#include "stm32f1xx_hal_flash.h"

/* Base address of the Emulated Flash sectors (1K per page) for STM32F103R8T6 64KB ROM 0x08000000-0x08010000*/ 

#define ADDR_FLASH_SECTOR_56     ((uint32_t)0x0800E000) /* Base address of Sector 56, 1 Kbytes  */
#define ADDR_FLASH_SECTOR_57     ((uint32_t)0x0800E400) /* Base address of Sector 57, 1 Kbytes  */
#define ADDR_FLASH_SECTOR_58     ((uint32_t)0x0800E800) /* Base address of Sector 58, 1 Kbytes  */
#define ADDR_FLASH_SECTOR_59     ((uint32_t)0x0800EC00) /* Base address of Sector 59, 1 Kbytes  */
#define ADDR_FLASH_SECTOR_60     ((uint32_t)0x0800F000) /* Base address of Sector 60, 1 Kbytes  */
#define ADDR_FLASH_SECTOR_61     ((uint32_t)0x0800F400) /* Base address of Sector 61, 1 Kbytes  */
#define ADDR_FLASH_SECTOR_62     ((uint32_t)0x0800F800) /* Base address of Sector 62, 1 Kbytes  */
#define ADDR_FLASH_SECTOR_63     ((uint32_t)0x0800FC00) /* Base address of Sector 63, 1 Kbytes  */

#define FLASH_MEMORY_ADDR   ADDR_FLASH_SECTOR_63




extern NV_Profile_TypeDef NV_Profile_Runtime;


void Flash_Erase(void)
{
	uint32_t  PageError ;                                 //erase error flag
	
	FLASH_EraseInitTypeDef FLASH_EraseInit;               //erease struct define
	FLASH_EraseInit.Banks = FLASH_BANK_1;                 //memory bank
	FLASH_EraseInit.NbPages = 1;                          //num of page to be erased
	FLASH_EraseInit.PageAddress = FLASH_MEMORY_ADDR;      //erase address
	FLASH_EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;    //erase type

	HAL_FLASH_Unlock();                                   //unlock flash
	if( HAL_FLASHEx_Erase(&FLASH_EraseInit,&PageError)!= HAL_OK)
	{
//		LED_CMD (RED_LED, LED_BLINK);     // error display
		return;
	}
}


void Flash_write(void)
{
  uint8_t i;                                       
  Flash_Erase();                                   //erase flash memory

	/* write data with the size of "double bytes"  ********************************/
 
	for(i=0;i<sizeof(NV_Profile_Runtime)/4;i++)
	{

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD , FLASH_MEMORY_ADDR+i*4, *((__IO uint32_t*)&NV_Profile_Runtime+i) );
	}
 
/* lock the falash after wrriten*/
	HAL_FLASH_Lock(); 
}


void Flash_read(uint32_t *data_buf_addr)             // read the flash profile from flash to data buffer
{
 

	for(uint8_t i = 0;i<sizeof(data_buf_addr)/4;i++)
	{
		*((uint32_t *)& data_buf_addr+i) = *((__IO uint32_t*)FLASH_MEMORY_ADDR + i);
		
	}

}









