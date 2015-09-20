#ifndef _BDMCFMACROS_H_
#define _BDMCFMACROS_H_

//=================================================================================
/* BDM commands */
#define BDMCF_RETRY         40   /* how many times to retry before giving up */
#define BDMCF_CMD_NOP       0x0000
#define BDMCF_CMD_GO        0x0C00
#define BDMCF_CMD_RDMREG    0x2D80
#define BDMCF_CMD_WDMREG    0x2C80
#define BDMCF_CMD_RCREG     0x2980
#define BDMCF_CMD_WCREG     0x2880
#define BDMCF_CMD_RAREG     0x2180
#define BDMCF_CMD_WAREG     0x2080
#define BDMCF_CMD_READ8     0x1900
#define BDMCF_CMD_READ16    0x1940
#define BDMCF_CMD_READ32    0x1980
#define BDMCF_CMD_WRITE8    0x1800
#define BDMCF_CMD_WRITE16   0x1840
#define BDMCF_CMD_WRITE32   0x1880
#define BDMCF_CMD_DUMP8     0x1D00
#define BDMCF_CMD_DUMP16    0x1D40
#define BDMCF_CMD_DUMP32    0x1D80
#define BDMCF_CMD_FILL8     0x1C00
#define BDMCF_CMD_FILL16    0x1C40
#define BDMCF_CMD_FILL32    0x1C80

//===============================================================================
// Response from BDM communication
// Status-bit value - 1st bit of Rx value
#define BDMCF_STATUS_OK          0

// LSB of data value when S==BDMCF_STATUS_OK
#define BDMCF_RES_OK          0xFFFF
// LSB of data value when S!=BDMCF_STATUS_OK
#define BDMCF_RES_NOT_READY   0x0000
#define BDMCF_RES_BUS_ERROR   0x0001
#define BDMCF_RES_ILLEGAL     0xFFFF

#endif // _BDMCFMACROS_H_
