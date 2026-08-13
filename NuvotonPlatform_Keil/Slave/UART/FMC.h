
#ifndef FMC_H
#define FMC_H

#define FW_VERSION          0x24

//#define FLASH64K
#undef outp32
#undef inp32

#define BOOL  int

#define UNLOCKREG(x)        *((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x59;*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x16;*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x88
#define LOCKREG(x)          *((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x00;

#define outp32(port,value)	(*((volatile unsigned int *)(port))=(value))
#define inp32(port)			(*((volatile unsigned int *)(port)))


#define FISPCON         (FMC_BASE+0x000)
#define ISPADR         (FMC_BASE+0x004)
#define ISPDAT         (FMC_BASE+0x008)
#define ISPCMD         (FMC_BASE+0x00C)
#define ISPTRG         (FMC_BASE+0x010)
#define DFBADDR        (FMC_BASE+0x014)
#define FATCON         (FMC_BASE+0x018)
#define ICPCON         (FMC_BASE+0x01C)
#define RMPCON         (FMC_BASE+0x020)

#define Config0         0x00300000
#define Config1         0x00300004
#define APROM_BASE      0x00000000
// For LD ROM
#define LDROM_BASE      0x00100000
#define LDROM_SIZE      0x00001000
#define LDROM_END       0x00101000

#define ISPGO           0x01
#define ISPFF           0x00000040


#define ISP_Read        0x00
#define ISP_Program     0x21
#define ISP_PageErase   0x22


/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define PAGE_SIZE                      0x00000200     /* Page size                                         */
//#define PAGE_NUM                       32             /* Total page number                                 */


extern void ReadID(void);
extern void EraseAP(BOOL bAprom, unsigned int addr_start,
		    unsigned int addr_end);
extern void ReadData(unsigned int addr_start, unsigned int addr_end,
		     unsigned int *data);
extern void WriteData(unsigned int addr_start, unsigned int addr_end,
		      unsigned int *data);
extern void UpdateConfig(unsigned int *data, unsigned int *res);
extern int FMC_Read(unsigned int address, unsigned int *data);
extern int FMC_Write(unsigned int address, unsigned int data);
extern int FMC_Erase(unsigned int address);
extern void GetDataFlashInfo(uint32_t *addr, uint32_t *size);

#endif

