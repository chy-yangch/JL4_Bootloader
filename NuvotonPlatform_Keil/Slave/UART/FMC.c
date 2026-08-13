#include <stdio.h>
#include "Nano1X2Series.h"
#include "fmc.h"

extern uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;


int FMC_Write(unsigned int address, unsigned int data)
{
	unsigned int Reg;

	outp32(ISPCMD, ISP_Program);
	outp32(ISPADR, address);
	outp32(ISPDAT, data);
	outp32(ISPTRG, ISPGO);

	__ISB();

	Reg = inp32(FISPCON);
	if (Reg & ISPFF) {
		outp32(FISPCON, Reg);
		return -1;
	}
	return 0;
}

int FMC_Read(unsigned int address, unsigned int *data)
{
	unsigned int Reg;

	outp32(ISPCMD, ISP_Read);
	outp32(ISPADR, address);
	outp32(ISPDAT, 0x00000000);
	outp32(ISPTRG, ISPGO);

	__ISB();

	Reg = inp32(FISPCON);
	if (Reg & ISPFF) {
		outp32(FISPCON, Reg);
		return -1;
	}

	*data = inp32(ISPDAT);

	return 0;
}


int FMC_Erase(unsigned int address)
{
	unsigned int Reg;

	outp32(ISPCMD, ISP_PageErase);
	outp32(ISPADR, address);
	outp32(ISPTRG, ISPGO);

	__ISB();

	Reg = inp32(FISPCON);
	if (Reg & ISPFF) {
		outp32(FISPCON, Reg);
		return -1;
	}
	return 0;
}


void ReadData(unsigned int addr_start, unsigned int addr_end,
	      unsigned int *data)    // Read data from flash
{
	unsigned int rLoop;

	for (rLoop = addr_start; rLoop < addr_end; rLoop += 4) {
		FMC_Read(rLoop, data);
		data++;
	}
	return;
}

void WriteData(unsigned int addr_start, unsigned int addr_end,
	       unsigned int *data)  // Write data into flash
{
	unsigned int wLoop;

	for (wLoop = addr_start; wLoop < addr_end; wLoop += 4) {
		FMC_Write(wLoop, *data);
		data++;
	}
}



//bAprom == TRUE erase all aprom besides data flash
void EraseAP(BOOL bAprom, unsigned int addr_start, unsigned int addr_end)
{
	unsigned int eraseLoop;
	unsigned int erase_end;

	if (bAprom == TRUE) {
		//GetDataFlashInfo(&erase_end, &eraseLoop);
		erase_end = g_dataFlashAddr;
		eraseLoop = APROM_BASE;
	} else {
		eraseLoop = addr_start;
		erase_end = addr_end;
	}
	for (; eraseLoop < erase_end; eraseLoop += PAGE_SIZE) {
		FMC_Erase(eraseLoop);
	}
	return;
}


void UpdateConfig(unsigned int *data, unsigned int *res)
{

	FMC->ISPCON |=
		FMC_ISPCON_CFGUEN_Msk;          /* Enable config update               */
	FMC_Erase(Config0);
	//FMC_Write(Config0, 0xFFFFFF7F);              /* Disable Lock bit                   */
	//FMC_Write(Config1, 0x0001F000);

	FMC_Write(Config0, *data);
	FMC_Write(Config1, *(data + 1));

	if (res) {
		FMC_Read(Config0, res);
		FMC_Read(Config1, res + 1);
	}

	FMC->ISPCON &=
		~FMC_ISPCON_CFGUEN_Msk;         /* Disable config update              */
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
	uint32_t uData;

	*size = 0;

	{
		FMC_Read(Config0, &uData);
		if ((uData & 0x01) == 0) { //DFEN enable
			FMC_Read(Config1, &uData);
			if (uData > g_apromSize || uData < 0x200) //avoid config1 value from error
				uData = g_apromSize;

			*addr = uData;
			*size = g_apromSize - uData;
		} else {
			*addr = g_apromSize;
			*size = 0;
		}
	}
}
