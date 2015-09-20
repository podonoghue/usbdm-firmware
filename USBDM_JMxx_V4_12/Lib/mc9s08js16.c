/* Based on CPU DB MC9S08JS16_24, version 3.00.017 (RegistersPrg V2.29) */
/* DataSheet : MC9S08JS16RM Rev. 4 4/2009 */

#include <mc9s08js16.h>

/*lint -save -esym(765, *) */


/* * * * *  8-BIT REGISTERS  * * * * * * * * * * * * * * * */
volatile PTADSTR _PTAD;                                    /* Port A Data Register; 0x00000000 */
volatile PTADDSTR _PTADD;                                  /* Port A Data Direction Register; 0x00000001 */
volatile PTBDSTR _PTBD;                                    /* Port B Data Register; 0x00000002 */
volatile PTBDDSTR _PTBDD;                                  /* Port B Data Direction Register; 0x00000003 */
volatile MTIMSCSTR _MTIMSC;                                /* MTIM Clock Configuration Register; 0x00000008 */
volatile MTIMCLKSTR _MTIMCLK;                              /* MTIM Clock Configuration Register; 0x00000009 */
volatile MTIMCNTSTR _MTIMCNT;                              /* MTIM Counter Register; 0x0000000A */
volatile MTIMMODSTR _MTIMMOD;                              /* MTIM Modulo Register; 0x0000000B */
volatile CRCHSTR _CRCH;                                    /* CRC High Register; 0x0000000C */
volatile CRCLSTR _CRCL;                                    /* CRC Low Register; 0x0000000D */
volatile TPMSCSTR _TPMSC;                                  /* TPM Status and Control Register; 0x00000010 */
volatile TPMC0SCSTR _TPMC0SC;                              /* TPM Timer Channel 0 Status and Control Register; 0x00000015 */
volatile TPMC1SCSTR _TPMC1SC;                              /* TPM Timer Channel 1 Status and Control Register; 0x00000018 */
volatile IRQSCSTR _IRQSC;                                  /* Interrupt request status and control register; 0x0000001B */
volatile KBISCSTR _KBISC;                                  /* KBI Status and Control Register; 0x0000001C */
volatile KBIPESTR _KBIPE;                                  /* KBI Pin Enable Register; 0x0000001D */
volatile KBIESSTR _KBIES;                                  /* KBI Edge Select Register; 0x0000001E */
volatile SCIC1STR _SCIC1;                                  /* SCI Control Register 1; 0x00000022 */
volatile SCIC2STR _SCIC2;                                  /* SCI Control Register 2; 0x00000023 */
volatile SCIS1STR _SCIS1;                                  /* SCI Status Register 1; 0x00000024 */
volatile SCIS2STR _SCIS2;                                  /* SCI Status Register 2; 0x00000025 */
volatile SCIC3STR _SCIC3;                                  /* SCI Control Register 3; 0x00000026 */
volatile SCIDSTR _SCID;                                    /* SCI Data Register; 0x00000027 */
volatile SPIC1STR _SPIC1;                                  /* SPI Control Register 1; 0x00000030 */
volatile SPIC2STR _SPIC2;                                  /* SPI Control Register 2; 0x00000031 */
volatile SPIBRSTR _SPIBR;                                  /* SPI Baud Rate Register; 0x00000032 */
volatile SPISSTR _SPIS;                                    /* SPI Status Register; 0x00000033 */
volatile MCGC1STR _MCGC1;                                  /* MCG Control Register 1; 0x00000040 */
volatile MCGC2STR _MCGC2;                                  /* MCG Control Register 2; 0x00000041 */
volatile MCGTRMSTR _MCGTRM;                                /* MCG Trim Register; 0x00000042 */
volatile MCGSCSTR _MCGSC;                                  /* MCG Status and Control Register; 0x00000043 */
volatile MCGC3STR _MCGC3;                                  /* MCG Control Register 3; 0x00000044 */
volatile RTCSCSTR _RTCSC;                                  /* RTC Status and Control Register; 0x00000048 */
volatile RTCCNTSTR _RTCCNT;                                /* RTC Counter Register; 0x00000049 */
volatile RTCMODSTR _RTCMOD;                                /* RTC Modulo Register; 0x0000004A */
volatile USBCTL0STR _USBCTL0;                              /* USB Control Register 0; 0x00000050 */
volatile PERIDSTR _PERID;                                  /* Peripheral ID Register; 0x00000058 */
volatile IDCOMPSTR _IDCOMP;                                /* Peripheral ID Complement Register; 0x00000059 */
volatile REVSTR _REV;                                      /* Peripheral Revision Register; 0x0000005A */
volatile INTSTATSTR _INTSTAT;                              /* Interrupt Status Register; 0x00000060 */
volatile INTENBSTR _INTENB;                                /* Interrupt Enable Register; 0x00000061 */
volatile ERRSTATSTR _ERRSTAT;                              /* Error Interrupt Status Register; 0x00000062 */
volatile ERRENBSTR _ERRENB;                                /* Error Interrupt Enable Register; 0x00000063 */
volatile STATSTR _STAT;                                    /* Status Register; 0x00000064 */
volatile CTLSTR _CTL;                                      /* Control Register; 0x00000065 */
volatile ADDRSTR _ADDR;                                    /* Address Register; 0x00000066 */
volatile FRMNUMLSTR _FRMNUML;                              /* Frame Number Register Low; 0x00000067 */
volatile FRMNUMHSTR _FRMNUMH;                              /* Frame Number Register High; 0x00000068 */
volatile EPCTL0STR _EPCTL0;                                /* Endpoint Control Register 0; 0x0000006D */
volatile EPCTL1STR _EPCTL1;                                /* Endpoint Control Register 1; 0x0000006E */
volatile EPCTL2STR _EPCTL2;                                /* Endpoint Control Register 2; 0x0000006F */
volatile EPCTL3STR _EPCTL3;                                /* Endpoint Control Register 3; 0x00000070 */
volatile EPCTL4STR _EPCTL4;                                /* Endpoint Control Register 4; 0x00000071 */
volatile EPCTL5STR _EPCTL5;                                /* Endpoint Control Register 5; 0x00000072 */
volatile EPCTL6STR _EPCTL6;                                /* Endpoint Control Register 6; 0x00000073 */
volatile SRSSTR _SRS;                                      /* System Reset Status Register; 0x00001800 */
volatile SBDFRSTR _SBDFR;                                  /* System Background Debug Force Reset Register; 0x00001801 */
volatile SOPT1STR _SOPT1;                                  /* System Options Register 1; 0x00001802 */
volatile SOPT2STR _SOPT2;                                  /* System Options Register 2; 0x00001803 */
volatile SPMSC1STR _SPMSC1;                                /* System Power Management Status and Control 1 Register; 0x00001809 */
volatile SPMSC2STR _SPMSC2;                                /* System Power Management Status and Control 2 Register; 0x0000180A */
volatile FTRIMSTR _FTRIM;                                  /* Reserved for storage of FTRIM; 0x0000180C */
volatile MCGTRIMSTR _MCGTRIM;                              /* Reserved for storage of MCGTRIM; 0x0000180D */
volatile FPROTDSTR _FPROTD;                                /* Flash Protection Defeat Register; 0x0000180E */
volatile SIGNATURESTR _SIGNATURE;                          /* SIGNATURE Register; 0x0000180F */
volatile DBGCSTR _DBGC;                                    /* Debug Control Register; 0x00001816 */
volatile DBGTSTR _DBGT;                                    /* Debug Trigger Register; 0x00001817 */
volatile DBGSSTR _DBGS;                                    /* Debug Status Register; 0x00001818 */
volatile FCDIVSTR _FCDIV;                                  /* FLASH Clock Divider Register; 0x00001820 */
volatile FOPTSTR _FOPT;                                    /* FLASH Options Register; 0x00001821 */
volatile FCNFGSTR _FCNFG;                                  /* FLASH Configuration Register; 0x00001823 */
volatile FPROTSTR _FPROT;                                  /* FLASH Protection Register; 0x00001824 */
volatile FSTATSTR _FSTAT;                                  /* Flash Status Register; 0x00001825 */
volatile FCMDSTR _FCMD;                                    /* FLASH Command Register; 0x00001826 */
volatile PTAPESTR _PTAPE;                                  /* Port A Pull Enable Register; 0x00001840 */
volatile PTASESTR _PTASE;                                  /* Port A Slew Rate Enable Register; 0x00001841 */
volatile PTADSSTR _PTADS;                                  /* Port A Drive Strength Selection Register; 0x00001842 */
volatile PTBPESTR _PTBPE;                                  /* Port B Pull Enable Register; 0x00001844 */
volatile PTBSESTR _PTBSE;                                  /* Port B Slew Rate Enable Register; 0x00001845 */
volatile PTBDSSTR _PTBDS;                                  /* Port B Drive Strength Selection Register; 0x00001846 */
/* NVFTRIM - macro for reading non volatile register       Nonvolatile MCG Fine Trim; 0x0000FFAE */
/* Tip for register initialization in the user code:  const byte NVFTRIM_INIT @0x0000FFAE = <NVFTRIM_INITVAL>; */
/* NVMCGTRM - macro for reading non volatile register      Nonvolatile MCG Trim Register; 0x0000FFAF */
/* Tip for register initialization in the user code:  const byte NVMCGTRM_INIT @0x0000FFAF = <NVMCGTRM_INITVAL>; */
/* NVBACKKEY0 - macro for reading non volatile register    Backdoor Comparison Key 0; 0x0000FFB0 */
/* Tip for register initialization in the user code:  const byte NVBACKKEY0_INIT @0x0000FFB0 = <NVBACKKEY0_INITVAL>; */
/* NVBACKKEY1 - macro for reading non volatile register    Backdoor Comparison Key 1; 0x0000FFB1 */
/* Tip for register initialization in the user code:  const byte NVBACKKEY1_INIT @0x0000FFB1 = <NVBACKKEY1_INITVAL>; */
/* NVBACKKEY2 - macro for reading non volatile register    Backdoor Comparison Key 2; 0x0000FFB2 */
/* Tip for register initialization in the user code:  const byte NVBACKKEY2_INIT @0x0000FFB2 = <NVBACKKEY2_INITVAL>; */
/* NVBACKKEY3 - macro for reading non volatile register    Backdoor Comparison Key 3; 0x0000FFB3 */
/* Tip for register initialization in the user code:  const byte NVBACKKEY3_INIT @0x0000FFB3 = <NVBACKKEY3_INITVAL>; */
/* NVBACKKEY4 - macro for reading non volatile register    Backdoor Comparison Key 4; 0x0000FFB4 */
/* Tip for register initialization in the user code:  const byte NVBACKKEY4_INIT @0x0000FFB4 = <NVBACKKEY4_INITVAL>; */
/* NVBACKKEY5 - macro for reading non volatile register    Backdoor Comparison Key 5; 0x0000FFB5 */
/* Tip for register initialization in the user code:  const byte NVBACKKEY5_INIT @0x0000FFB5 = <NVBACKKEY5_INITVAL>; */
/* NVBACKKEY6 - macro for reading non volatile register    Backdoor Comparison Key 6; 0x0000FFB6 */
/* Tip for register initialization in the user code:  const byte NVBACKKEY6_INIT @0x0000FFB6 = <NVBACKKEY6_INITVAL>; */
/* NVBACKKEY7 - macro for reading non volatile register    Backdoor Comparison Key 7; 0x0000FFB7 */
/* Tip for register initialization in the user code:  const byte NVBACKKEY7_INIT @0x0000FFB7 = <NVBACKKEY7_INITVAL>; */
/* CHECKSUMBYPASS - macro for reading non volatile register Checksum bypass; 0x0000FFBA */
/* Tip for register initialization in the user code:  const byte CHECKSUMBYPASS_INIT @0x0000FFBA = <CHECKSUMBYPASS_INITVAL>; */
/* NVPROT - macro for reading non volatile register        Nonvolatile FLASH Protection Register; 0x0000FFBD */
/* Tip for register initialization in the user code:  const byte NVPROT_INIT @0x0000FFBD = <NVPROT_INITVAL>; */
/* NVOPT - macro for reading non volatile register         Nonvolatile Flash Options Register; 0x0000FFBF */
/* Tip for register initialization in the user code:  const byte NVOPT_INIT @0x0000FFBF = <NVOPT_INITVAL>; */


/* * * * *  16-BIT REGISTERS  * * * * * * * * * * * * * * * */
volatile TPMCNTSTR _TPMCNT;                                /* TPM Timer Counter Register; 0x00000011 */
volatile TPMMODSTR _TPMMOD;                                /* TPM Timer Counter Modulo Register; 0x00000013 */
volatile TPMC0VSTR _TPMC0V;                                /* TPM Timer Channel 0 Value Register; 0x00000016 */
volatile TPMC1VSTR _TPMC1V;                                /* TPM Timer Channel 1 Value Register; 0x00000019 */
volatile SCIBDSTR _SCIBD;                                  /* SCI Baud Rate Register; 0x00000020 */
volatile SPID16STR _SPID16;                                /* SPI Data Register; 0x00000034 */
volatile SPIMSTR _SPIM;                                    /* SPI Match Register; 0x00000036 */
volatile SDIDSTR _SDID;                                    /* System Device Identification Register; 0x00001806 */
volatile DBGCASTR _DBGCA;                                  /* Debug Comparator A Register; 0x00001810 */
volatile DBGCBSTR _DBGCB;                                  /* Debug Comparator B Register; 0x00001812 */
volatile DBGFSTR _DBGF;                                    /* Debug FIFO Register; 0x00001814 */

/*lint -restore */

/* EOF */
