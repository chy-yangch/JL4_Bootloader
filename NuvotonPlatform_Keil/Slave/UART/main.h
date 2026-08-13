/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include "Nano1X2Series.h"
#include "spi.h"

/* SPI傳送資料更新用的變收定義 */
#define SPI_FLASH_PORT  SPI1
#define SPI_WRITE_TX    SPI_WRITE_TX1
#define SPI_READ_RX     SPI_READ_RX1
#define SKIP_MODE_NAME 16
#define FW_Ud_Addr 1572864	//0   //

void SpiFlash_WaitReady(void);
volatile uint16_t 	FW_Upd_size = 0;	/* FW更新的大小(2byte一個單位算) */
volatile uint8_t 	FW_Upd_start_flag = 0 , Upd_complete = 0;
volatile uint16_t 	FW_Upd_count = 0;	/* FW更新時的迴圈計數器 */
volatile uint16_t 	DL_FW_cont = 0;	/* 計算是否為16的整數 , 若不是 FW_Upd_count要再加1 */
volatile uint16_t   	dl1 =0;		/* FW更新迴圈變數,總數為(byte/16) */
volatile uint8_t   	dl2=0;		/* FW傳送變數,一次16byte */
volatile uint8_t 	BT_Upd_flag = 0;	/* 更新模式;藍芽或SPI */






static __inline void SPI1Init(void)
{
	uint32_t u32Div = 0;//u32ClkSrc, 

	/* PA.12 ~ PA.15   */
	SYS->PA_H_MFP = ((SYS->PA_H_MFP & ~0xFFFF0000) | 0x66660000);
	CLK->APBCLK |= CLK_APBCLK_SPI1_EN_Msk;    /* enable SPI1 clock */
	/* SPI clock source from HCLK  */
	CLK->CLKSEL1 |= CLK_CLKSEL2_SPI1_S_Pos;
	//SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 8, 2000000);
	SPI_FLASH_PORT->CTL = SPI_MASTER | (8 << SPI_CTL_TX_BIT_LEN_Pos) | (SPI_MODE_0);
	SPI_FLASH_PORT->CLKDIV = (SPI_FLASH_PORT->CLKDIV & ~SPI_CLKDIV_DIVIDER1_Msk) | u32Div;
	//SPI_EnableAutoSS(SPI1, SPI_SS0, SPI_SS0_ACTIVE_LOW);
	SPI_FLASH_PORT->SSR |= (SPI_SS0 | SPI_SS0_ACTIVE_LOW) | SPI_SSR_AUTOSS_Msk;
	//SPI_EnableFIFO(SPI1, 4, 4);
	SPI_FLASH_PORT->FFCTL = (SPI_FLASH_PORT->FFCTL & ~(SPI_FFCTL_TX_THRESHOLD_Msk | SPI_FFCTL_RX_THRESHOLD_Msk) |
                  (4 << SPI_FFCTL_TX_THRESHOLD_Pos) | (4 << SPI_FFCTL_RX_THRESHOLD_Pos));

	SPI_FLASH_PORT->CTL |= SPI_CTL_FIFOM_Msk;
	SpiFlash_WaitReady();
}
uint8_t SpiFlash_ReadStatusReg(void)
{
	// /CS: active
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);
	// send Command: 0x05, Read status register
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x05);
	// read status
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
	// wait tx finish
	while(SPI_IS_BUSY(SPI_FLASH_PORT));
	// /CS: de-active
	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);
	// skip first rx data
	SPI_READ_RX(SPI_FLASH_PORT);
	return (SPI_READ_RX(SPI_FLASH_PORT) & 0xff);
}

void SpiFlash_WaitReady(void)
{
	uint8_t ReturnValue;

	do {
		ReturnValue = SpiFlash_ReadStatusReg();
		ReturnValue = ReturnValue & 1;
	} while(ReturnValue!=0);   // check the BUSY bit
}


int16_t SpiFlash_NormalRead(uint32_t Address)
{
	uint8_t i;
	uint16_t j;

	Address=Address*2;
	SPI_SET_SS0_LOW(SPI_FLASH_PORT);   // /CS: active
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x03);  //0x03 send Command: 0x03, Read data
	SPI_WRITE_TX(SPI_FLASH_PORT, (Address>>16) & 0xFF);  // send 24-bit start address
	SPI_WRITE_TX(SPI_FLASH_PORT, (Address>>8)  & 0xFF);
	SPI_WRITE_TX(SPI_FLASH_PORT, Address       & 0xFF);
	while(SPI_IS_BUSY(SPI_FLASH_PORT));

	//SPI_ClearRxFIFO(SPI_FLASH_PORT);    // clear RX buffer 
	SPI_FLASH_PORT->FFCTL |= SPI_FFCTL_RX_CLR_Msk;
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);     // read data
	while(SPI_IS_BUSY(SPI_FLASH_PORT)); 

	i = SPI_READ_RX(SPI_FLASH_PORT);
	SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
	while(SPI_IS_BUSY(SPI_FLASH_PORT));

	j = SPI_READ_RX(SPI_FLASH_PORT);
	j=j<<8;
	j |= i;
	while(SPI_IS_BUSY(SPI_FLASH_PORT));     // wait tx finish  

	SPI_SET_SS0_HIGH(SPI_FLASH_PORT);       // /CS: de-active 
	return j;
		
}  
