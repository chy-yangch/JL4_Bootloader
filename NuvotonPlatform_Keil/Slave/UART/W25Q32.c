#include <stdio.h>
#include "Nano1X2Series.h"
#include "W25Q32.h"
#include "spi.h"

#define SPI_FLASH_PORT 	SPI1

#define SPI_WRITE(n) 		SPI_WRITE_TX1(SPI_FLASH_PORT, n)
#define SPI_READ     		SPI_READ_RX1(SPI_FLASH_PORT)
#define SPI_SS_HIGH  		SPI_SET_SS0_HIGH(SPI_FLASH_PORT)
#define SPI_SS_LOW   		SPI_SET_SS0_LOW(SPI_FLASH_PORT)

/**
  * @brief Clear Rx FIFO buffer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    spi->FFCTL |= SPI_FFCTL_RX_CLR_Msk;
}

uint16_t SpiFlash_ReadMidDid(void)
{
	uint8_t u8RxData[6], u8IDCnt = 0;
	
	// /CS: active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);

	// send Command: 0x90, Read Manufacturer/Device ID
	SPI_WRITE(0x90);

	// send 24-bit '0', dummy
	SPI_WRITE(0x00);
	SPI_WRITE(0x00);
	SPI_WRITE(0x00);

	// receive 16-bit
	SPI_WRITE(0x00);
	SPI_WRITE(0x00);

	// wait tx finish
	while (SPI_IS_BUSY(SPI_FLASH_PORT));

	// /CS: de-active
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

	while (!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
		u8RxData[u8IDCnt ++] = SPI_READ;

	return ((u8RxData[4] << 8) | u8RxData[5]);
}

int32_t SpiFlash_NormalRead(uint32_t addr)
{
	uint8_t *flash_buf;
	uint32_t data = 0;

	addr = addr * 4;
	flash_buf = (uint8_t *)(&addr);

	/* /CS: active */
	SPI_SS_LOW;

	/* send Command: 0x03, Read data */
	SPI_WRITE(0x03);

	/* send 24-bit start address */
	SPI_WRITE(flash_buf[2]);
	SPI_WRITE(flash_buf[1]);
	SPI_WRITE(flash_buf[0]);
	while(SPI_IS_BUSY(SPI_FLASH_PORT))
		continue;

	/* clear RX buffer  */
	//SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
	SPI_ClearRxFIFO(SPI_FLASH_PORT);

	flash_buf = (uint8_t *)(&data);

	/* read data */
	SPI_WRITE(0x00);
	while(SPI_IS_BUSY(SPI_FLASH_PORT))
		continue;

	flash_buf[3] = SPI_READ;

	SPI_WRITE(0x00);
	while(SPI_IS_BUSY(SPI_FLASH_PORT))
		continue;

	flash_buf[2] = SPI_READ;

	SPI_WRITE(0x00);
	while(SPI_IS_BUSY(SPI_FLASH_PORT))
		continue;

	flash_buf[1] = SPI_READ;

	SPI_WRITE(0x00);
	while(SPI_IS_BUSY(SPI_FLASH_PORT))
		continue;

	flash_buf[0] = SPI_READ;

	while(SPI_IS_BUSY(SPI_FLASH_PORT))
		continue;

	/* /CS: de-active */
	SPI_SS_HIGH;

	return data;
}

uint8_t SpiFlash_ReadStatusReg(void)
{
	/* /CS: active */
	SPI_SS_LOW;

	/* send Command: 0x05, Read status register */
	SPI_WRITE(0x05);

	/* read status */
	SPI_WRITE(0x00);

	/* wait tx finish */
	while(SPI_IS_BUSY(SPI_FLASH_PORT));

	/* /CS: de-active */
	SPI_SS_HIGH;

	/* skip first rx data */
	SPI_READ;

	return (SPI_READ & 0xff);
}

void SpiFlash_WaitReady_1(void)
{
	uint8_t ReturnValue = 0;

	do {
		ReturnValue = SpiFlash_ReadStatusReg();
		ReturnValue = ReturnValue & 1;
	} while(ReturnValue != 0);   /* check the BUSY bit */
}

void spi_flash_erase(uint8_t cmd, uint32_t addr)
{
	addr *= 4;

	/* /CS: active */
	//SPI_SET_SS_LOW(SPI_FLASH_PORT);
	SPI_SS_LOW;

	/* send Command: 0x06, Write enable */
	SPI_WRITE(0x06);
	while(SPI_IS_BUSY(SPI_FLASH_PORT))
		continue;

	SPI_SS_HIGH; /* /CS: de-active */

	SPI_SS_LOW; /* /CS: active */
	SPI_WRITE(cmd);
	SPI_WRITE((addr >> 16) & 0xFF);
	SPI_WRITE((addr >> 8) & 0xFF);
	SPI_WRITE(addr & 0xFF);
	while(SPI_IS_BUSY(SPI_FLASH_PORT)) /* wait tx finish */
		continue;

	SPI_SS_HIGH; /* CS: de-active */
	//SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
	SPI_ClearRxFIFO(SPI_FLASH_PORT);
	SpiFlash_WaitReady_1();
}

void SpiFlash_NormalPageProgram(uint32_t addr, uint32_t data) 
{
	uint8_t *flash_buf;

	addr = addr * 4;
	flash_buf = (uint8_t *)(&addr);

	SPI_SS_LOW;         /* /CS: active */
	SPI_WRITE(0x06);    /* send Command: 0x06, Write enable */
	while(SPI_IS_BUSY(SPI_FLASH_PORT));   /* wait tx finish */

	SPI_SS_HIGH;        /* /CS: de-active */
	
	SPI_SS_LOW;         /* /CS: active */
	SPI_WRITE(0x02);    /* send Command: 0x02, Page program */
	SPI_WRITE(flash_buf[2]);   /* send 24-bit start address */
	SPI_WRITE(flash_buf[1]);
	SPI_WRITE(flash_buf[0]);

	flash_buf = (uint8_t *)(&data);
	SPI_WRITE(flash_buf[3]);
	SPI_WRITE(flash_buf[2]);
	SPI_WRITE(flash_buf[1]);
	SPI_WRITE(flash_buf[0]);
	while(SPI_IS_BUSY(SPI_FLASH_PORT));   /* wait tx finish */
	
	SPI_SS_HIGH;        /* /CS: de-active */
	SPI_ClearRxFIFO(SPI_FLASH_PORT);
	SpiFlash_WaitReady_1();
}

void SpiFlash_ChipErase_logger(uint8_t sets)
{

//  0 sets:  0x000000 ~ 0x1FFFFF
//  1 sets:  0x200000 ~ 0x3FFFFF

//set 0:  0x000000  ~  0x0FFFFF
//set 1:  0x100000  ~  0x1FFFFF
//set 2:  0x200000  ~  0x2FFFFF
//set 3:  0x300000  ~  0x3FFFFF
// 資料分別 4個 sets

	uint32_t addre;
	uint32_t addre1;
	uint8_t i;

	sets = sets * 16;
	for (i = 0; i < 16; i++) {
		addre1 = sets + i;
		addre = addre1 << 16;
		SPI_SET_SS0_LOW(SPI_FLASH_PORT);       // /CS: active
		SPI_WRITE(0x06);    // send Command: 0x06, Write enable
		while (SPI_IS_BUSY(SPI_FLASH_PORT));   // wait tx finish
		SPI_SET_SS0_HIGH(SPI_FLASH_PORT);      // /CS: de-active
		SPI_SET_SS0_LOW(SPI_FLASH_PORT);        // /CS: active
		SPI_WRITE(0xD8);     // send Command: 0xC7, Chip Erase

		SPI_WRITE((addre >> 16) & 0xFF);
		SPI_WRITE((addre >> 8)  & 0xFF);
		SPI_WRITE( addre  & 0xFF);

		while (SPI_IS_BUSY(SPI_FLASH_PORT));    // wait tx finish
		SPI_SET_SS0_HIGH(SPI_FLASH_PORT);       //CS: de-active
		SPI_ClearRxFIFO(SPI1);
		SpiFlash_WaitReady_1();
	}

}
