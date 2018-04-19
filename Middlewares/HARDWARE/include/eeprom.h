#ifndef __EEPROM_H
#define __EEPROM_H 	

#include "sys.h"

#define EEPROM_BASE_ADDR	0x08080000	
#define EEPROM_BYTE_SIZE	0x0FFF      //4K字节

#define PEKEY1	0x89ABCDEF		//FLASH_PEKEYR
#define PEKEY2	0x02030405		//FLASH_PEKEYR

#define EN_INT      	__enable_irq();		//系统开全局中断
#define DIS_INT     	__disable_irq();	//系统关全局中断

void EEPROM_ReadBytes(uint16_t Addr,uint8_t *Buffer,uint16_t Length);
void EEPROM_ReadWords(uint16_t Addr,uint16_t *Buffer,uint16_t Length);
void EEPROM_WriteBytes(uint16_t Addr,uint8_t *Buffer,uint16_t Length);
void EEPROM_WriteWords(uint16_t Addr,uint16_t *Buffer,uint16_t Length);

#endif
