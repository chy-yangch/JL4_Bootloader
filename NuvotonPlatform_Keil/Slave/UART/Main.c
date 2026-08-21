/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include <string.h>
#include "Nano1X2Series.h"
#include "fmc.h"
#include "sys.h"
#include "clk.h"
#include "timer.h"
#include "W25Q32.h"
#include "gpio.h"

#define JL4RH
//#define JL4PR
//#define JL4PC

#define FW_BIN_ADDR	0x1000
//#define FW_BIN_ADDR	0x0000 //test

#define USING_AUTODETECT

#define PACKET_SIZE		64

#define CMD_UPDATE_APROM	0x000000A0
#define CMD_UPDATE_CONFIG	0x000000A1
#define CMD_READ_CONFIG	0x000000A2
#define CMD_ERASE_ALL		0x000000A3
#define CMD_SYNC_PACKNO	0x000000A4
#define CMD_GET_FWVER		0x000000A6
#define CMD_RUN_APROM		0x000000AB
#define CMD_RUN_LDROM		0x000000AC
#define CMD_RESET			0x000000AD
#define CMD_CONNECT		0x000000AE
#define CMD_DISCONNECT		0x000000AF

#define CMD_GET_DEVICEID	0x000000B1

#define CMD_UPDATE_DATAFLASH	0x000000C3
#define CMD_WRITE_CHECKSUM	0x000000C9
#define CMD_GET_FLASHMODE		0x000000CA

#define CMD_RESEND_PACKET		0x000000FF

#define	V6M_AIRCR_VECTKEY_DATA	0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ		0x00000004UL

#define DISCONNECTED	0
#define CONNECTING		1
#define CONNECTED		2

// BLE
#define BLE_EN                  	PC4
#define BLE_STATUS         	PC5
#define BLE_WAKEUP       	PC6
// LED
#define SYNC_LED			PC1

#define APO_PIN			PC14
#define TSL_PIN			PA5
#define FCS_PIN			PA6

#define POW_KEY			PA3
#define BLE_LED			PD9

//#define RTC_SPR_BASE        0x40008000UL
//#define RTC_RWEN        	(*(volatile uint32_t *)(RTC_SPR_BASE + 0x20))
//#define RTC_SPR(n)      		(*(volatile uint32_t *)(RTC_SPR_BASE + 0x40 + ((n) * 4)))

static uint8_t volatile	bufhead;
static uint8_t volatile	g_connStatus;
__align(4) static uint8_t uart_rcvbuf[64];
__align(4) static uint8_t uart_sendbuf[64];
__align(4) static uint8_t aprom_buf[PAGE_SIZE];
BOOL bUsbDataReady, bUartDataReady;
BOOL bUsbInReady, bUpdateApromCmd;
uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;
volatile uint32_t g_pdid, g_timecnt;
volatile uint16_t timer0_cnt = 0, timer0_start = 0, timer0_cnt1 = 0;
volatile uint8_t  start_chk = 0;

__IO uint8_t ble_ota_status;	//0:init IO,
						//1:send ble command 'M'
						//2:wait command 'M' ack
						//3:wait meter tyep check 

#ifdef SUPPORT_WRITECKSUM
static uint32_t g_ckbase = (0x20000 - 8);
static void CheckCksumBase(void);
#endif

#ifdef UART0_TEST
static UART_T  *g_pUART = UART0;
static IRQn_Type	g_UARTIRQ = UART0_IRQn;
#else
static UART_T  *g_pUART = UART1;
static IRQn_Type	g_UARTIRQ = UART1_IRQn;
#endif

void Delay(uint32_t delayCnt);
void my_memcpy(void *dest, void *src, int32_t size);
extern void WordsCpy(void *dest, void *src, int32_t size);
void SysTimerDelay(uint32_t us);

volatile uint32_t lcmd_reg;
volatile uint16_t led_tt = 0;
volatile uint16_t led_mode = 0;

volatile uint32_t fmc_data;

__IO uint8_t ble_ota_step = 0;
__IO uint8_t ble_ota_cmd_retry_cnt;

uint32_t calculate_aprom_checksum(void);
void  bin_to_approm (void);
void default_flash_rom(void);
void SYS_Init (void);
void ble_ota_io_init(void);
void ble_mode_cmd_m(void);
void ble_ota_step_function (void);

/* 從Flash讀出序號, 20170407 */
int FMC_Read1(unsigned int address)
{
	unsigned int Reg;

//	outp32(ISPCMD, ISP_Read);
//	outp32(ISPADR, address);
//	outp32(ISPDAT, 0x00000000);
//	outp32(ISPTRG, ISPGO);

//	__ISB();

//	Reg = inp32(FISPCON);
//	if (Reg & ISPFF) {
//		outp32(FISPCON, Reg);
//		return -1;
//	}

//	fmc_data  = inp32(ISPDAT);

	return 0;
}

//-------------------------------------------------------------------------------------------
// Wake up BLE Module, 20170104, chc add
//-------------------------------------------------------------------------------------------
void BLE_wakeup(void)
{
	PC->DOUT |= BIT6;
	CLK_SysTickDelay(5); // 5 us
	PC->DOUT &= ~BIT6;
	CLK_SysTickDelay(7000); // 7 ms
}
//-------------------------------------------------------------------------------------------
// BLE Module Sleep, 20170104, chc add
//-------------------------------------------------------------------------------------------
void BLE_sleep(void)
{
	CLK_SysTickDelay(5000); // 5 ms
	PC->DOUT |= BIT6;
}
/* send data for UART, 20170104, chc add */
void PutString(uint8_t len)//static __inline void PutString()
{
	int i;

	for (i = 0; i < len; i++) {
		while (g_pUART->FSR & UART_FSR_TX_FULL_F_Msk);
		g_pUART->THR = uart_sendbuf[i];
	}
	bufhead = 0;
}

/* UART0/UART1 interrupt handler. handle receive only */
#ifdef UART0_TEST
void UART0_IRQHandler(void)//UART0_IRQHandler
#else
void UART1_IRQHandler(void)//UART1_IRQHandler
#endif
{
	/* RDA FIFO interrupt & RDA timeout interrupt  */
	if (g_pUART->ISR & (UART_ISR_RTO_IS_Msk | UART_ISR_RDA_IS_Msk)) {
		while (((g_pUART->FSR & UART_FSR_RX_EMPTY_F_Msk) == 0) && (bufhead < 64))
			uart_rcvbuf[bufhead++] = g_pUART->RBR;
	}
	if (bufhead < 64) {
		if (uart_rcvbuf[0] == '>' && uart_rcvbuf[bufhead - 2] == 0x0D &&
		    uart_rcvbuf[bufhead - 1] == 0x0A) {
			if (start_chk < 7) {
				bUartDataReady = TRUE;
				bufhead = 0;
			}
		}
	} else if (bufhead == 64) {
		bUartDataReady = TRUE;
		bufhead = 0;
		led_mode = 2;
	}
}

static __inline void UartInit(void)
{

	/* PB.13(RX) PB.14(TX)   */
//	SYS->PC_H_MFP = ((SYS->PC_H_MFP & ~0x0000000F) | 0x00000007);
	/* PB.13(RX) PB.14(TX)   */
//	SYS->PC_L_MFP = ((SYS->PC_L_MFP & ~0xF0000000) | 0x70000000);

#ifdef UART0_TEST
	CLK->APBCLK |=
		CLK_APBCLK_UART0_EN_Msk;          /* enable UART0 clock */
#else
	CLK->APBCLK |= CLK_APBCLK_UART1_EN_Msk;    /* enable UART1 clock */
#endif
	/* UART clock source from HIRC 12 MHz  */
	CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | 0x3;

	g_pUART->BAUD = 0x66;      /* Baud Rate:115200  OSC:12 MHz        */
	g_pUART->TLCTL = 0x3;      /* character len is 8 bits             */

	NVIC_SetPriority(g_UARTIRQ, 2);
	NVIC_EnableIRQ(g_UARTIRQ);
	g_pUART->IER = UART_IER_RTO_IE_Msk | UART_IER_RDA_IE_Msk;
}


static uint16_t Checksum(unsigned char *buf, int len)
{
	int i;
	uint16_t c;

	for (c = 0, i = 0; i < len; i++)
		c += buf[i];
	return (c);
}

static uint16_t CalCheckSum(uint32_t start, uint32_t len)
{
	int i;
	uint16_t lcksum = 0;

	for (i = 0; i < len; i += PAGE_SIZE) {
		ReadData(start + i, start + i + PAGE_SIZE, (uint32_t *)aprom_buf);
		if (len - i >= PAGE_SIZE)
			lcksum += Checksum(aprom_buf, PAGE_SIZE);
		else
			lcksum += Checksum(aprom_buf, len - i);
	}
	return lcksum;
}

static int ParseCmd(unsigned char *buffer, uint8_t len, BOOL bUSB)
{
	static uint32_t StartAddress, StartAddress_bak, TotalLen, TotalLen_bak,
	       LastDataLen, g_packno = 1;
	uint8_t *response;
	uint16_t cksum, lcksum;
	uint32_t	lcmd, srclen, i, regcnf0, security;
	unsigned char *pSrc;
	static uint32_t	gcmd;

	response = uart_sendbuf;

	pSrc = buffer;
	srclen = len;

	lcmd = inpw(pSrc);
	inpw(pSrc + 4);
	outpw(response + 4, 0);

	pSrc += 8;
	srclen -= 8;

	ReadData(Config0, Config0 + 8, (uint32_t *)(response + 8));
	regcnf0 = *(uint32_t *)(response + 8);
	security = regcnf0 & 0x2;

	if (lcmd == CMD_SYNC_PACKNO) {
		start_chk = 7;
		g_packno = inpw(pSrc);
	}

	if ((lcmd) && (lcmd != CMD_RESEND_PACKET))
		gcmd = lcmd;

	if (lcmd == CMD_GET_FWVER)
		response[8] = FW_VERSION;
	else if (lcmd == CMD_GET_DEVICEID) {
		outpw(response + 8, SYS->PDID);
		goto out;
	} else if (lcmd == CMD_RUN_APROM || lcmd == CMD_RUN_LDROM ||
		   lcmd == CMD_RESET) {
		/* Set BS */
		if (lcmd == CMD_RUN_APROM) {
			lcmd_reg = lcmd;
			TIMER0->PRECNT = 0x00000000;
			/* go into interrupt, BASE_TIME/10 counts */
			TIMER0->CMPR = 1200000 - 1;
			TIMER0->CTL = TIMER_CTL_TMR_EN_Msk | TIMER_PERIODIC_MODE;
		} else if (lcmd == CMD_RUN_LDROM) {
			SYS->RST_SRC = 3;
			i = (inpw(&FMC->ISPCON) & 0xFFFFFFFC);
			i |= 0x00000002;
			FMC->ISPCON = i;
			SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);
			/* Trap the CPU */
			while (1);
		} else {
			SYS->RST_SRC = 3;
			i = (inpw(&FMC->ISPCON) & 0xFFFFFFFE);/* ISP disable */
			FMC->ISPCON = i;
			SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);
			/* Trap the CPU */
			while (1);
		}
	} else if (lcmd == CMD_CONNECT) {
		g_packno = 1;
		goto out;
	} else if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_ERASE_ALL)) {
		if (lcmd == CMD_ERASE_ALL)  /* erase APROM + data flash */
			EraseAP(FALSE, 0, (g_apromSize < 0x20000) ? 0x20000 :
				g_apromSize);
		else
			EraseAP(TRUE, 0, 0);  /* don't erase data flash */

		if (lcmd == CMD_ERASE_ALL) {
			*(uint32_t *)(response + 8) = regcnf0 | 0x02;
			UpdateConfig((uint32_t *)(response + 8), NULL);
		}

		bUpdateApromCmd = TRUE;
	}


#ifdef SUPPORT_WRITECKSUM
	else if (lcmd == CMD_WRITE_CHECKSUM) { //write cksum to aprom last
		cktotallen = inpw(pSrc);
		lcksum = inpw(pSrc + 4);
		CheckCksumBase();
		ReadData(g_ckbase & 0xFFE00, g_ckbase, (uint32_t *)aprom_buf);
		outpw(aprom_buf + PAGE_SIZE - 8, cktotallen);
		outpw(aprom_buf + PAGE_SIZE - 4, lcksum);
		FMC_Erase(g_ckbase & 0xFFE00);
		WriteData(g_ckbase & 0xFFE00, g_ckbase + 8, (uint32_t *)aprom_buf);
	}
#endif
	else if (lcmd == CMD_GET_FLASHMODE) {
		outpw(response + 8, (inpw(&FMC->ISPCON) & 0x2) ? 2 : 1);
	}


	if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH)) {
		if (lcmd == CMD_UPDATE_DATAFLASH) {
			StartAddress = g_dataFlashAddr;
			if (g_dataFlashSize)
				EraseAP(FALSE, g_dataFlashAddr, g_dataFlashAddr + g_dataFlashSize);
			else
				goto out;
		} else
			StartAddress = 0;

		TotalLen = inpw(pSrc + 4);
		pSrc += 8;
		srclen -= 8;
		StartAddress_bak = StartAddress;
		TotalLen_bak = TotalLen;

	} else if (lcmd == CMD_UPDATE_CONFIG) {
		if ((security == 0) && (!bUpdateApromCmd)) //security lock
			goto out;

		UpdateConfig((uint32_t *)(pSrc), (uint32_t *)(response + 8));
		GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

		goto out;
	} else if (lcmd == CMD_RESEND_PACKET) {
		StartAddress -= LastDataLen;
		TotalLen += LastDataLen;
		if ((StartAddress & 0xFFE00) >= Config0)
			goto out;
		ReadData(StartAddress & 0xFFE00, StartAddress,
			 (uint32_t *)aprom_buf);
		FMC_Erase(StartAddress & 0xFFE00);
		WriteData(StartAddress & 0xFFE00, StartAddress,
			  (uint32_t *)aprom_buf);
		if ((StartAddress % PAGE_SIZE) >= (PAGE_SIZE - LastDataLen))
			FMC_Erase((StartAddress & 0xFFE00) + PAGE_SIZE);
		goto out;
	}

	if ((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH)) {
		if (TotalLen >= srclen)
			TotalLen -= srclen;
		else {
			srclen = TotalLen;//prevent last package from over writing
			TotalLen = 0;
		}

		WriteData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
		{
			uint32_t *p = (uint32_t *)pSrc;
			for (i = 0; i < srclen / 4; i++)
				p[i] = 0;
		}
		ReadData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
		StartAddress += srclen;
		LastDataLen = srclen;
		if (TotalLen == 0) {
			lcksum = CalCheckSum(StartAddress_bak, TotalLen_bak);
			outps(response + 8, lcksum);
		}
	}
out:
	cksum = Checksum(buffer, len);
	outps(response, cksum);
	++g_packno;
	outpw(response + 4, g_packno);
	g_packno++;
	return 0;
}

//unit is 0.5us
void SysTimerDelay(uint32_t us)
{

	SysTick->LOAD = us * 16; /* using 32MHz cpu clock*/
	SysTick->VAL   = (0x00);
	SysTick->CTRL = SysTick->CTRL | (1 << SysTick_CTRL_CLKSOURCE_Pos) |
			(1 << SysTick_CTRL_ENABLE_Pos);
	/* Waiting for down-count to zero */
	while ((SysTick->CTRL & (1 << 16)) == 0);
}

#ifdef SUPPORT_WRITECKSUM
static void CheckCksumBase()
{
	unsigned int aprom_end = g_apromSize, data;
	int result;

	result = FMC_Read(0x1F000 - 8, &data);
	if (result == 0) {	//128K flash
		FMC_Read(Config0, &data);
		if ((data & 0x01) == 0) //DFEN enable
			FMC_Read(Config1, &aprom_end);
		g_ckbase = aprom_end - 8;
		return;
	} else {	// less than 128K
		aprom_end = 0x10000;//64K
		do {
			result = FMC_Read(aprom_end - 8, &data);
			if (result == 0)
				break;
			aprom_end = aprom_end / 2;
		} while (aprom_end > 4096);
		g_ckbase = aprom_end - 8;
	}
}
#endif

//the smallest of APROM size is 2K
static __inline uint32_t GetApromSize()
{
	uint32_t size = 0x800, data;
	int result;

	do {
		result = FMC_Read(size, &data);
		if (result < 0)
			return size;
		else
			size *= 2;
	} while (1);
}

void CLK_SysTickDelay(uint32_t us)
{
	SysTick->LOAD = us * CyclesPerUs;
	SysTick->VAL  = (0x00);
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	/* Waiting for down-count to zero */
	while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
}
__IO uint32_t view,test_key = 1;
__IO uint8_t approm_update = 0;
//__IO uint32_t dd = 0x12345678;
//__IO uint8_t reg[10] ={0};
__IO uint32_t aprom_sum;

__IO uint32_t flash_checksum,calculate_approm,get_approm_checksum,ota_upate;

int32_t main()
{
	extern uint32_t SystemCoreClock;
	uint8_t volatile bufhead_bak = 0;
	uint8_t   i,*ptr;
	//uint32_t flash_checksum,calculate_approm,get_approm_checksum;

	
//	ptr = (uint8_t *)&dd;
//	
//	reg[0] = *(ptr + 3);
//	reg[1] = *(ptr + 2);
//	reg[2] = *(ptr + 1);
//	reg[3] = *(ptr + 0);
	
	SYS_Init();

	/* Write 80-byte spare RAM */
	//RTC_RWEN = 0x0000A965;
	//RTC_SPR(0) = 0x12345678;
	//RTC_SPR(1) = 0xABCDEF01;
	//RTC_SPR(2) = 0x00000055;
	
	// Enable Write RTC RAM
	//RTC->CAR = 0x0000A965;
	//Write RTC RAM
	//RTC->SPR0 = 0x12345678;
	//RTC->SPR1 = 0xABCDEF01;
	//Read RTC RAM	
	//approm_update = RTC->SPR0;

	PD->DOUT |= BIT10;
	
	//ReadData( 0x7DFC,  0X7E00, (uint32_t *)&get_approm_checksum); // 取得儲存於APROM最後4 Bytes的chekcsum
	
	ota_upate = SpiFlash_NormalRead(0);			

	if (ota_upate == 0)
		approm_update = 1;
	
	
	if (approm_update) {
		PD->DOUT |= BIT10;
		PD->DOUT |= BIT13;
		
		bin_to_approm();
		
		spi_flash_erase(SPI_FLASH_4KB_ERASE,0);
		//RTC->CAR = 0x0000A965;
		//RTC->SPR0 = 0;
	}
	
	
	
	//PD->DOUT &= ~BIT10;
	
	// Flash Test
	//view = SpiFlash_ReadMidDid();
	calculate_approm = calculate_aprom_checksum();	//計算31.5K - 4 Bytes的APROM checksum,***在這之前作ReadData動作會造成APROM數值異動,原因不明
	get_approm_checksum = 0;
	ReadData( 0x7DFC,  0X7E00, (uint32_t *)&get_approm_checksum); // 取得儲存於APROM最後4 Bytes的chekcsum
	//由APROM讀出的checksum需再次反轉
	get_approm_checksum=__REV(get_approm_checksum);
	
	//default_flash_rom();
	//spi_flash_erase(SPI_FLASH_4KB_ERASE,0);
	//SpiFlash_NormalPageProgram(0,0x11223344);	    
	//flash_checksum = SpiFlash_NormalRead(0);

	//while(1);

	SysTimerDelay(5000000);
//	goto _APROM;
	
	if (calculate_approm != get_approm_checksum) {
		
		__NOP();
//		__NOP();		
		
		goto _CHECK_ERR;
	}
	
	
	if((POW_KEY) && (test_key == 1)) {

_CHECK_ERR:
		
		while (1) {
			ble_ota_step_function();
			if (ble_ota_step == 4)
				goto _ISP;
		}
	} else {
		
		goto _APROM;
	
	}

_APROM:

		SYS->RST_SRC = 3;
		FMC->ISPCON &= 0xFFFFFFFC;
		SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);
		/* Trap the CPU */
		while (1);
	
_RST:
	SYS->RST_SRC = 3;
	i = (inpw(&FMC->ISPCON) & 0xFFFFFFFC);
	i |= 0x00000002;
	FMC->ISPCON = i;
	SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);
	while (1);
_ISP:
	while (1) {
		if (bUartDataReady == TRUE) {
			bUartDataReady = FALSE;
			if ((uart_rcvbuf[1] == 'B') &&
			    (uart_rcvbuf[2] == 'L') &&
			    (uart_rcvbuf[3] == 'E')) {
				FMC_Write(0X7E00, 0X424C4500);
				goto _APROM;
			} else {	/* 硬體序號 + 機種碼 */
				if (uart_rcvbuf[2] == 'E') {
					for (i = 0; i < 12; i++)
						uart_sendbuf[i] = 0;
					uart_sendbuf[0] = '#';
					uart_sendbuf[1] = 10;
					uart_sendbuf[2] = 'E';
					uart_sendbuf[3] = 'A';
					uart_sendbuf[4] = 'A';


					#if defined(JL4PR)
						uart_sendbuf[5] = 'B';
						uart_sendbuf[6] = 'G';
					#elif defined(JL4RH)
						uart_sendbuf[5] = 'B';
						uart_sendbuf[6] = 'H';
					#elif defined(JL4PC)
						uart_sendbuf[5] = 'B';
						uart_sendbuf[6] = 'F';
					#else
					 __NOP();
					 __NOP();
					#endif
					

					for (i = 0; i < uart_sendbuf[1] - 3; i++)
						uart_sendbuf[uart_sendbuf[1] - 3] += uart_sendbuf[i];
					uart_sendbuf[uart_sendbuf[1] - 2] = 0x0D;
					uart_sendbuf[uart_sendbuf[1] - 1] = 0x0A;
					PutString(uart_sendbuf[1]);
				} else { /* UPD packet handshake */
					g_timecnt = 0;
					ParseCmd(uart_rcvbuf, PACKET_SIZE, FALSE);
					PutString(16);
				}
			}
		}

//		if (PA->PIN & 0x00000008) {
//			timer0_start = 1;
//			if (timer0_cnt1 >= 5)
//				goto _RST;
//		} else {
//			timer0_start = 0;
//			timer0_cnt1 = 0;
//		}
		
//		if (lcmd_reg == CMD_RUN_APROM) {
//			if (led_tt >= 30) {
//				goto _RST;
//			} else
//				__NOP();
//		}
	/* timeout happen; but byte is less than 64 bytes; host goes wrong */
//		if (bufhead > 0) {
//			if (g_timecnt == 0)
//				bufhead_bak = bufhead;
//			SysTimerDelay(1);
//			g_timecnt++;
//			if (g_timecnt > 2000) {
//				g_timecnt = 0;
//				if (bufhead_bak == bufhead)
//					bufhead = 0;
//			}
//		}
	}
}

void TMR0_IRQHandler(void)
{
	TIMER0->ISR = TIMER_ISR_TMR_IS_Msk;
	if (lcmd_reg == CMD_RUN_APROM) {
		if (led_mode == 2) {
			led_mode = 1;
			TIMER1->CMPR = 600000;
			TIMER1->DR = 0;
		}
		led_tt++;
	} else {
		timer0_cnt++;
		if (timer0_start == 1)
			timer0_cnt1++;
	}
	
	ble_ota_cmd_retry_cnt++;
}

void TMR1_IRQHandler(void)
{
	TIMER1->ISR = TIMER_ISR_TMR_IS_Msk;
	switch (led_mode) {
	case 1:
		
		PD->DOUT ^= BIT9;
		break;
	case 2:

		PD->DOUT ^= BIT9;
		break;
	}
}
__IO uint32_t k = 0;
uint32_t calculate_aprom_checksum(void)			//約50ms完成
{
	uint16_t i,j;
	uint32_t check_sum;
	uint8_t upd_data[512];

	check_sum = 0;
	
	//31.5K的APROM最後4 Bytes為checksum不列入計算
	for (i = 0; i < 63; i++) {

		ReadData( i * 0x200,  i * 0x200 + 0x200, (uint32_t *)upd_data);

		if (i < 62) {
			
			for (j = 0; j < 512;j++) {
				check_sum+= upd_data[j];
				k++;
			}
		} else {
			//最後4Bytes為checksum不進行加總運算
			for (j = 0; j < 508;j++) {
				check_sum+= upd_data[j];
				k++;
			}
		}
	}

	return check_sum;
}

//__IO uint32_t data_upd_data[128];
void  bin_to_approm (void)
{
	
	uint32_t data_upd_data[128],addr,u32_data,i,j;
	//uint16_t i,j;

	EraseAP(FALSE, 0, 0x00007E00);	
	
	//更新31.5K,其中0.5K儲存設定值
	for (i = 0; i < 63; i++) {

		for (j = 0; j < 128; j++) {
			
			addr = FW_BIN_ADDR + (j * 4) + ( i * 512);
			
			u32_data = SpiFlash_NormalRead(addr);
			
			//__REV() 就是 32-bit Byte Reverse,因寫入MCU的函數是由小排到大,所以需經過反轉
			data_upd_data[j] = __REV(u32_data);
			//data_upd_data[j] = SpiFlash_NormalRead(FW_BIN_ADDR + j + (i * 128));
			
		}

		WriteData(i * 0x200, (i * 0x200 )+ 0x200, (uint32_t *)data_upd_data);
		
//		if (i >= 56) {
//			
//			__NOP();
//			__NOP();
//		}

	}
	
//	uint32_t data_upd_data[128],addr;
//	uint16_t i,j;

//	//更新31.5K,其中0.5K儲存設定值
//	for (i = 0; i < 63; i++) {

//		for (j = 0; j < 128; j++) {
//			
//			addr = FW_BIN_ADDR + (j * 4) + ( i * 512);
//			data_upd_data[j] = SpiFlash_NormalRead(addr);
//			//data_upd_data[j] = SpiFlash_NormalRead(FW_BIN_ADDR + j + (i * 128));
//			
//		}

//		WriteData(i * 0x200, (i * 0x200 )+ 0x200, (uint32_t *)data_upd_data);

//	}

}
/*
 * 寫Flash內定值進行測試
*/
void default_flash_rom(void)
{
	uint16_t i = 0;
	
	SpiFlash_ChipErase_logger(0);
	
	for (i = 0; i < 32768; i++)
		SpiFlash_NormalPageProgram(i,i);
	
}
void SYS_Init (void)
{
	int32_t i32TimeOutCnt;
	
	/* Init System, peripheral clock and multi-function I/O */

#ifdef SUPPORT_WRITECKSUM
	uint32_t totallen, cksum;
#endif

	UNLOCKREG();

	CLK->PWRCTL  |= (CLK_PWRCTL_HIRC_EN | CLK_PWRCTL_LIRC_EN) ;
	CLK->PWRCTL  &=  ~(CLK_PWRCTL_LXT_EN | CLK_PWRCTL_HXT_EN) ;
	/* Waiting for 12M/HIRC:12M Xtal stalble */
	i32TimeOutCnt = __HSI / 200;
	while ((CLK->CLKSTATUS & CLK_CLKSTATUS_HIRC_STB_Msk) !=
	       CLK_CLKSTATUS_HIRC_STB_Msk) {
		if (i32TimeOutCnt-- <= 0)
			__NOP();
	}
	i32TimeOutCnt = __HSI / 200;
	while ((CLK->CLKSTATUS & CLK_CLKSTATUS_LIRC_STB_Msk) !=
	       CLK_CLKSTATUS_LIRC_STB_Msk) {
		if (i32TimeOutCnt-- <= 0)
			__NOP();
	}
	/* reseaved bit defined in main : enable 10kHz */
	CLK->CLKSEL1  |= CLK_CLKSEL1_LCD_S_LIRC + CLK_CLKSEL1_TMR0_S_HIRC;
	CLK->APBCLK   |= CLK_APBCLK_LCD_EN + CLK_APBCLK_TMR0_EN + CLK_APBCLK_TMR1_EN|CLK_APBCLK_SPI1_EN | CLK_APBCLK_RTC_EN;

    #if defined(JL4PR)
	PA->DOUT = 0x00000000;
	SYS->PA_H_MFP =	0x00000000;
	SYS->PA_L_MFP =	0x00001200;
	PA->PMD = 0x00000005;
	PA->OFFD = 0x00070000;
	PA->PUEN = 0x00000000;
	PA->DBEN = 0x00000008;
	
	PC->DOUT = 0x00004000;//0x00000050;
	SYS->PC_H_MFP = 0x00000000;//0x00000007;
	SYS->PC_L_MFP = 0x00000000;//0x70000000;
	PC->PMD  = 0x10015100;//0x10015100;//0x10001500;0x15511151
	PC->OFFD = 0x00000000;
	PC->PUEN = 0x00000200;	
	
    #elif defined(JL4RH)
	PA->DOUT = 0x00000000;
	SYS->PA_H_MFP = 0x00000000;
	SYS->PA_L_MFP = 0x00001200;
	PA->PMD =  0x00000005;
	PA->OFFD = 0x00070000;
	PA->PUEN = 0x00000000;
	PA->DBEN = 0x00000008;
	
	PC->DOUT = 0x00004000;//0x00000050;
	SYS->PC_H_MFP = 0x00000000;//0x00000007;
	SYS->PC_L_MFP = 0x00000000;//0x70000000;
	PC->PMD  = 0x15555505;//0x15540005;//0x15541105;//0x15551105;//0x10001500; 
	PC->OFFD = 0x00000000;
	PC->PUEN = 0x00000000;
		
    #elif defined(JL4PC)
	PA->DOUT = 0x00000000;
	SYS->PA_H_MFP =	0x00000000;
	SYS->PA_L_MFP =	0x00001222;
	PA->PMD = 0x00000005;
	PA->OFFD = 0x00070000;
	PA->PUEN = 0x00000000;
	PA->DBEN = 0x00000008;
	
	PC->DOUT = 0x00004000;//0x00000050;
	SYS->PC_H_MFP = 0x00000000;//0x00000007;
	SYS->PC_L_MFP = 0x00000003;//0x70000003;
	PC->PMD  = 0x10015501;//0x10001501;
	PC->OFFD = 0x00000000;
	PC->PUEN = 0x00000000;
		
    #else
	__NOP();
	__NOP();
    #endif
	
	PB->OFFD = 0x00000000;
	PB->PUEN = 0x00000000;
	
	PB->PMD |= 0x01000000;

	PD->DOUT = 0x00000000;
	PD->PMD =  0x04140000;
	SYS->PD_H_MFP =	0x00000000;
	SYS->PD_L_MFP = 0x00000000;
	PD->PUEN = 0x00000000;
	PD->OFFD = 0x00000000;

	SYS->PF_L_MFP = 0x00FF0020;
	PF->PMD   = 0XFFFFF055;
	PF->OFFD = 0x00000000;
	PF->PUEN = 0x00000000;
	PF->DOUT = 0x00000000;

	/* Lock protected registers */
	/* Give a dummy target frequency here. */
	/* Will over write capture resolution with macro */
	TIMER0->PRECNT = 0x00000000;
	TIMER0->CMPR   = 12000000 - 1; /* go into interrupt, BASE_TIME counts */
	TIMER0->CTL    = TIMER_CTL_TMR_EN_Msk | TIMER_PERIODIC_MODE;
	/* Enable timer interrupt */
	TIMER0->IER   |= TIMER_IER_TMR_IE_Msk;
	NVIC_SetPriority(TMR0_IRQn, 2);
	NVIC_EnableIRQ(TMR0_IRQn);

	TIMER1->PRECNT = 0x00000000;
	TIMER1->CMPR   = 2000000  ;
	TIMER1->CTL = TIMER_CTL_TMR_EN_Msk | TIMER_PERIODIC_MODE;
	TIMER1->IER |= TIMER_IER_TMR_IE_Msk;
	NVIC_SetPriority(TMR1_IRQn, 2);
	NVIC_EnableIRQ(TMR1_IRQn);

	//while (1);
	
	/* Enable VCC power */
	FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;

	g_apromSize = GetApromSize();
	GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

	g_pdid = SYS->PDID;
#ifdef UART0_TEST
	g_pUART = UART0;
	g_UARTIRQ = UART0_IRQn;
#else
	g_pUART = UART1;
	g_UARTIRQ = UART1_IRQn;
#endif
	UartInit();

//#if defined(USING_AUTODETECT)
	//timeout 30ms
	SysTick->LOAD = 30000 * 48; // using 48MHz cpu clock
	SysTick->VAL  = 0x00;
	SysTick->CTRL = SysTick->CTRL | (1 << SysTick_CTRL_CLKSOURCE_Pos) |
			(1 << SysTick_CTRL_ENABLE_Pos);


    APO_PIN = 0;
    TSL_PIN = 0;
    FCS_PIN = 0;   
 
    /* Setup SPI1 multi-function pins */
    SYS->PA_H_MFP = 0x66660000;    
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI1 as a low level active device. */
    /* Default setting: slave selection signal is low level active. */    
    SPI1->SSR = 0x00000005;
    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */    
    SPI1->CTL = 0x00200044;
    /* Set DIVIDER = 0 */
    SPI1->CLKDIV = 0U;    
    SPI1->FFCTL = 0x44000000;	

}

void ble_ota_io_init(void)
{
	#if defined(JL4PR)
		
		PC->DOUT = 0x00000010;
		PC->PMD  = 0x10001100;
		CLK_SysTickDelay(500000); // 5 us

		SYS->PC_H_MFP = 0x00000007;
		SYS->PC_L_MFP = 0x70000000;
		PC->DOUT |= BIT6;
		PC->OFFD = 0x00000000;
		PC->PUEN = 0x00000200;					
		
	#elif defined(JL4RH)
		
		PC->DOUT = 0x00000050;
		PC->PMD  = 0x10001100;
		CLK_SysTickDelay(500000); // 5 us

		SYS->PC_H_MFP = 0x00000007;
		SYS->PC_L_MFP = 0x70000000;
		PC->DOUT |= BIT6;
			
	#elif defined(JL4PC)

		PC->DOUT = 0x00000050;
		PC->PMD  = 0x10001101;
		CLK_SysTickDelay(500000); // 5 us
		SYS->PC_H_MFP = 0x00000007;
		SYS->PC_L_MFP = 0x70000003;
		PC->DOUT |= BIT6;
		PC->OFFD = 0x00000000;
		PC->PUEN = 0x00000000;					
	#else
	 __NOP();
	 __NOP();
	#endif
	
	start_chk = 2;
	timer0_cnt = 0;
	//PD->DOUT |= BIT9;
	BLE_LED = 1;
	

}

void ble_mode_cmd_m(void)
{
	uint8_t i;
	
	BLE_wakeup();
	for (i = 0; i < 10; i++)
		uart_sendbuf[i] = 0;

	FMC_Read1(0x7A00);
	if (fmc_data == 0xFFFFFFFF) {
		FMC_Read1(0x79FC);

		uart_sendbuf[3] =
			(uint8_t)(fmc_data & 0x000000FF);
		uart_sendbuf[4] =
			(int8_t)((fmc_data & 0x0000FF00) >> 8);
		uart_sendbuf[5] =
			(int8_t)((fmc_data & 0x00FF0000) >> 16);
		uart_sendbuf[6] =
			(int8_t)((fmc_data & 0xFF000000) >> 24);
	} else {
		for (i = 0; i < 6; i++) {
			FMC_Read1(0x7A00 + i * 4);
			uart_sendbuf[3 + i] = (uint8_t)fmc_data;
		}
	}
	uart_sendbuf[0] = '#';
	uart_sendbuf[1] = 14;
	uart_sendbuf[2] = 'M';
	uart_sendbuf[7] = 'A';
	uart_sendbuf[8] = 'A';

	#if defined(JL4PR)
		uart_sendbuf[9] = 'B';
		uart_sendbuf[10] = 'G';
	#elif defined(JL4RH)
		uart_sendbuf[9] = 'B';
		uart_sendbuf[10] = 'H';
	#elif defined(JL4PC)
		uart_sendbuf[9] = 'B';
		uart_sendbuf[10] = 'F'			
	#else
	 __NOP();
	 __NOP();
	#endif

	for (i = 0; i < uart_sendbuf[1] - 3; i++)
		uart_sendbuf[uart_sendbuf[1] - 3] +=
		uart_sendbuf[i];

	uart_sendbuf[uart_sendbuf[1] - 2] = 0x0D;
	uart_sendbuf[uart_sendbuf[1] - 1] = 0x0A;
	bufhead = 0;
	PutString(uart_sendbuf[1]);
	BLE_sleep();		

}

uint8_t wait_cmd_m_ack(void)
{
	uint8_t ack;
	
	if (bUartDataReady == TRUE && uart_rcvbuf[2] == 'M') {
		bUartDataReady = FALSE;
		ack = 1;
		BLE_WAKEUP = 1;
	} else {
		ack = 0;
	}
	
	return ack;
}

uint8_t wait_meter_tyep_check(void)
{
	uint8_t i,ack;

	if (bUartDataReady == TRUE) {
		bUartDataReady = FALSE;
		if (uart_rcvbuf[2] == 'E') {
			for (i = 0; i < 12; i++)
				uart_sendbuf[i] = 0;
		}
		uart_sendbuf[0] = '#';
		uart_sendbuf[1] = 10;
		uart_sendbuf[2] = 'E';
		uart_sendbuf[3] = 'A';
		uart_sendbuf[4] = 'A';
		
		#if defined(JL4PR)
			uart_sendbuf[5] = 'B';
			uart_sendbuf[6] = 'G';
		#elif defined(JL4RH)
			uart_sendbuf[5] = 'B';
			uart_sendbuf[6] = 'H';
		#elif defined(JL4PC)
			uart_sendbuf[5] = 'B';
			uart_sendbuf[6] = 'F';
		#else
		 __NOP();
		 __NOP();
		#endif

		for (i = 0; i < uart_sendbuf[1] - 3; i++)
			uart_sendbuf[uart_sendbuf[1] - 3] +=
				uart_sendbuf[i];
		uart_sendbuf[uart_sendbuf[1] - 2] = 0x0D;
		uart_sendbuf[uart_sendbuf[1] - 1] = 0x0A;
		PutString(uart_sendbuf[1]);

		ack = 1;
	} else {
		ack = 0;
	}
	
	return ack;
}

void ble_ota_step_function (void)
{
	switch(ble_ota_step) {
	
		case 0:
			ble_ota_io_init();
			ble_ota_step = 1;
			break;
		
		case 1:
			ble_mode_cmd_m();
			ble_ota_step = 2;
			ble_ota_cmd_retry_cnt = 0;
			break;
		
		case 2:
				
			if (ble_ota_cmd_retry_cnt > 3) {
				
				ble_ota_step = 2;

			} else  {
		
				if (wait_cmd_m_ack())
					ble_ota_step = 3;
			}
			
			break;
		
		case 3 :
			
			if(wait_meter_tyep_check()) 
				ble_ota_step = 4;

			break;
		
		case 4 :
			
			__NOP();
			
			break;
	}

}
