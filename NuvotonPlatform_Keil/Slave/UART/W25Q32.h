
#ifndef _W25Q32_H__
#define _W25Q32_H__
#include <stdint.h>

#define SPI_FLASH_4KB_ERASE  0x20
#define SPI_FLASH_32KB_ERASE 0x52
#define SPI_FLASH_64KB_ERASE 0xD8
#define SPI_FLASH_CHIP_ERASE 0xC7

extern int32_t SpiFlash_NormalRead(uint32_t addr);
extern uint16_t SpiFlash_ReadMidDid(void);
extern void SpiFlash_NormalPageProgram(uint32_t addr, uint32_t data) ;
extern void spi_flash_erase(uint8_t cmd, uint32_t addr);

#endif  /* __UART_TRANS_H__ */


