#ifndef __FRAM_CFG__
#define __FRAM_CFG__

#define FRAM_WIDTH   (320)
#define FRAM_HIGH    (240)
#define PIX_BYTE       (2)

#define DMA_BUF_MAX_SIZE (4095)
#define FRAM_BUF_LINE ((DMA_BUF_MAX_SIZE/FRAM_WIDTH/PIX_BYTE)&0xfe)
#define FRAM_BUF_SIZE (FRAM_WIDTH*PIX_BYTE*FRAM_BUF_LINE)

#endif