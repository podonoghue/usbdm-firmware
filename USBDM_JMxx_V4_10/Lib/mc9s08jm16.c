/* Based on CPU DB MC9S08JM16_48, version 3.00.045 (RegistersPrg V2.28) */
/* DataSheet : MC9S08JM16 Rev. 2 5/2008 */

#include <mc9s08jm16.h>

/*lint -save -esym(765, *) */


/* * * * *  8-BIT REGISTERS  * * * * * * * * * * * * * * * */
volatile PTADSTR _PTAD;                                    /* Port A Data Register; 0x00000000 */
volatile PTADDSTR _PTADD;                                  /* Port A Data Direction Register; 0x00000001 */
volatile PTBDSTR _PTBD;                                    /* Port B Data Register; 0x00000002 */
volatile PTBDDSTR _PTBDD;                                  /* Port B Data Direction Register; 0x00000003 */
volatile PTCDSTR _PTCD;                                    /* Port C Data Register; 0x00000004 */
volatile PTCDDSTR _PTCDD;                                  /* Port C Data Direction Register; 0x00000005 */
volatile PTDDSTR _PTDD;                                    /* Port D Data Register; 0x00000006 */
volatile PTDDDSTR _PTDDD;                                  /* Port D Data Direction Register; 0x00000007 */
volatile PTEDSTR _PTED;                                    /* Port E Data Register; 0x00000008 */
volatile PTEDDSTR _PTEDD;                                  /* Port E Data Direction Register; 0x00000009 */
volatile PTFDSTR _PTFD;                                    /* Port F Data Register; 0x0000000A */
volatile PTFDDSTR _PTFDD;                                  /* Port F Data Direction Register; 0x0000000B */
volatile PTGDSTR _PTGD;                                    /* Port G Data Register; 0x0000000C */
volatile PTGDDSTR _PTGDD;                                  /* Port G Data Direction Register; 0x0000000D */
volatile ACMPSCSTR _ACMPSC;                                /* ACMP Status and Control Register; 0x0000000E */
volatile ADCSC1STR _ADCSC1;                                /* Status and Control Register 1; 0x00000010 */
volatile ADCSC2STR _ADCSC2;                                /* Status and Control Register 2; 0x00000011 */
volatile ADCCFGSTR _ADCCFG;                                /* Configuration Register; 0x00000016 */
volatile APCTL1STR _APCTL1;                                /* Pin Control 1 Register; 0x00000017 */
volatile APCTL2STR _APCTL2;                                /* Pin Control 2 Register; 0x00000018 */
volatile IRQSCSTR _IRQSC;                                  /* Interrupt request status and control register; 0x0000001B */
volatile KBISCSTR _KBISC;                                  /* KBI Status and Control Register; 0x0000001C */
volatile KBIPESTR _KBIPE;                                  /* KBI Pin Enable Register; 0x0000001D */
volatile KBIESSTR _KBIES;                                  /* KBI Edge Select Register; 0x0000001E */
volatile TPM1SCSTR _TPM1SC;                                /* TPM1 Status and Control Register; 0x00000020 */
volatile TPM1C0SCSTR _TPM1C0SC;                            /* TPM1 Timer Channel 0 Status and Control Register; 0x00000025 */
volatile TPM1C1SCSTR _TPM1C1SC;                            /* TPM1 Timer Channel 1 Status and Control Register; 0x00000028 */
volatile TPM1C2SCSTR _TPM1C2SC;                            /* TPM1 Timer Channel 2 Status and Control Register; 0x0000002B */
volatile TPM1C3SCSTR _TPM1C3SC;                            /* TPM1 Timer Channel 3 Status and Control Register; 0x0000002E */
volatile SCI1C1STR _SCI1C1;                                /* SCI1 Control Register 1; 0x0000003A */
volatile SCI1C2STR _SCI1C2;                                /* SCI1 Control Register 2; 0x0000003B */
volatile SCI1S1STR _SCI1S1;                                /* SCI1 Status Register 1; 0x0000003C */
volatile SCI1S2STR _SCI1S2;                                /* SCI1 Status Register 2; 0x0000003D */
volatile SCI1C3STR _SCI1C3;                                /* SCI1 Control Register 3; 0x0000003E */
volatile SCI1DSTR _SCI1D;                                  /* SCI1 Data Register; 0x0000003F */
volatile SCI2C1STR _SCI2C1;                                /* SCI2 Control Register 1; 0x00000042 */
volatile SCI2C2STR _SCI2C2;                                /* SCI2 Control Register 2; 0x00000043 */
volatile SCI2S1STR _SCI2S1;                                /* SCI2 Status Register 1; 0x00000044 */
volatile SCI2S2STR _SCI2S2;                                /* SCI2 Status Register 2; 0x00000045 */
volatile SCI2C3STR _SCI2C3;                                /* SCI2 Control Register 3; 0x00000046 */
volatile SCI2DSTR _SCI2D;                                  /* SCI2 Data Register; 0x00000047 */
volatile MCGC1STR _MCGC1;                                  /* MCG Control Register 1; 0x00000048 */
volatile MCGC2STR _MCGC2;                                  /* MCG Control Register 2; 0x00000049 */
volatile MCGTRMSTR _MCGTRM;                                /* MCG Trim Register; 0x0000004A */
volatile MCGSCSTR _MCGSC;                                  /* MCG Status and Control Register; 0x0000004B */
volatile MCGC3STR _MCGC3;                                  /* MCG Control Register 3; 0x0000004C */
volatile SPI1C1STR _SPI1C1;                                /* SPI1 Control Register 1; 0x00000050 */
volatile SPI1C2STR _SPI1C2;                                /* SPI1 Control Register 2; 0x00000051 */
volatile SPI1BRSTR _SPI1BR;                                /* SPI1 Baud Rate Register; 0x00000052 */
volatile SPI1SSTR _SPI1S;                                  /* SPI1 Status Register; 0x00000053 */
volatile IICASTR _IICA;                                    /* IIC Address Register; 0x00000058 */
volatile IICFSTR _IICF;                                    /* IIC Frequency Divider Register; 0x00000059 */
volatile IICC1STR _IICC1;                                  /* IIC Control Register 1; 0x0000005A */
volatile IICSSTR _IICS;                                    /* IIC Status Register; 0x0000005B */
volatile IICDSTR _IICD;                                    /* IIC Data I/O Register; 0x0000005C */
volatile IICC2STR _IICC2;                                  /* IIC Control Register 2; 0x0000005D */
volatile TPM2SCSTR _TPM2SC;                                /* TPM2 Status and Control Register; 0x00000060 */
volatile TPM2C0SCSTR _TPM2C0SC;                            /* TPM2 Timer Channel 0 Status and Control Register; 0x00000065 */
volatile TPM2C1SCSTR _TPM2C1SC;                            /* TPM2 Timer Channel 1 Status and Control Register; 0x00000068 */
volatile RTCSCSTR _RTCSC;                                  /* RTC Status and Control Register; 0x0000006C */
volatile RTCCNTSTR _RTCCNT;                                /* RTC Counter Register; 0x0000006D */
volatile RTCMODSTR _RTCMOD;                                /* RTC Modulo Register; 0x0000006E */
volatile SPI2C1STR _SPI2C1;                                /* SPI2 Control Register 1; 0x00000070 */
volatile SPI2C2STR _SPI2C2;                                /* SPI2 Control Register 2; 0x00000071 */
volatile SPI2BRSTR _SPI2BR;                                /* SPI2 Baud Rate Register; 0x00000072 */
volatile SPI2SSTR _SPI2S;                                  /* SPI2 Status Register; 0x00000073 */
volatile USBCTL0STR _USBCTL0;                              /* USB Control Register 0; 0x00000080 */
volatile PERIDSTR _PERID;                                  /* Peripheral ID Register; 0x00000088 */
volatile IDCOMPSTR _IDCOMP;                                /* Peripheral ID Complement Register; 0x00000089 */
volatile REVSTR _REV;                                      /* Peripheral Revision Register; 0x0000008A */
volatile INTSTATSTR _INTSTAT;                              /* Interrupt Status Register; 0x00000090 */
volatile INTENBSTR _INTENB;                                /* Interrupt Enable Register; 0x00000091 */
volatile ERRSTATSTR _ERRSTAT;                              /* Error Interrupt Status Register; 0x00000092 */
volatile ERRENBSTR _ERRENB;                                /* Error Interrupt Enable Register; 0x00000093 */
volatile STATSTR _STAT;                                    /* Status Register; 0x00000094 */
volatile CTLSTR _CTL;                                      /* Control Register; 0x00000095 */
volatile ADDRSTR _ADDR;                                    /* Address Register; 0x00000096 */
volatile FRMNUMLSTR _FRMNUML;                              /* Frame Number Register Low; 0x00000097 */
volatile FRMNUMHSTR _FRMNUMH;                              /* Frame Number Register High; 0x00000098 */
volatile EPCTL0STR _EPCTL0;                                /* Endpoint Control Register 0; 0x0000009D */
volatile EPCTL1STR _EPCTL1;                                /* Endpoint Control Register 1; 0x0000009E */
volatile EPCTL2STR _EPCTL2;                                /* Endpoint Control Register 2; 0x0000009F */
volatile EPCTL3STR _EPCTL3;                                /* Endpoint Control Register 3; 0x000000A0 */
volatile EPCTL4STR _EPCTL4;                                /* Endpoint Control Register 4; 0x000000A1 */
volatile EPCTL5STR _EPCTL5;                                /* Endpoint Control Register 5; 0x000000A2 */
volatile EPCTL6STR _EPCTL6;                                /* Endpoint Control Register 6; 0x000000A3 */
volatile SRSSTR _SRS;                                      /* System Reset Status Register; 0x00001800 */
volatile SBDFRSTR _SBDFR;                                  /* System Background Debug Force Reset Register; 0x00001801 */
volatile SOPT1STR _SOPT1;                                  /* System Options Register 1; 0x00001802 */
volatile SOPT2STR _SOPT2;                                  /* System Options Register 2; 0x00001803 */
volatile SPMSC1STR _SPMSC1;                                /* System Power Management Status and Control 1 Register; 0x00001809 */
volatile SPMSC2STR _SPMSC2;                                /* System Power Management Status and Control 2 Register; 0x0000180A */
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
volatile PTCPESTR _PTCPE;                                  /* Port C Pull Enable Register; 0x00001848 */
volatile PTCSESTR _PTCSE;                                  /* Port C Slew Rate Enable Register; 0x00001849 */
volatile PTCDSSTR _PTCDS;                                  /* Port C Drive Strength Selection Register; 0x0000184A */
volatile PTDPESTR _PTDPE;                                  /* Port D Pull Enable Register; 0x0000184C */
volatile PTDSESTR _PTDSE;                                  /* Port D Slew Rate Enable Register; 0x0000184D */
volatile PTDDSSTR _PTDDS;                                  /* Port D Drive Strength Selection Register; 0x0000184E */
volatile PTEPESTR _PTEPE;                                  /* Port E Pull Enable Register; 0x00001850 */
volatile PTESESTR _PTESE;                                  /* Port E Slew Rate Enable Register; 0x00001851 */
volatile PTEDSSTR _PTEDS;                                  /* Port E Drive Strength Selection Register; 0x00001852 */
volatile PTFPESTR _PTFPE;                                  /* Port F Pull Enable Register; 0x00001854 */
volatile PTFSESTR _PTFSE;                                  /* Port F Slew Rate Enable Register; 0x00001855 */
volatile PTFDSSTR _PTFDS;                                  /* Port F Drive Strength Selection Register; 0x00001856 */
volatile PTGPESTR _PTGPE;                                  /* Port G Pull Enable Register; 0x00001858 */
volatile PTGSESTR _PTGSE;                                  /* Port G Slew Rate Enable Register; 0x00001859 */
volatile PTGDSSTR _PTGDS;                                  /* Port G Drive Strength Selection Register; 0x0000185A */
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
/* NVPROT - macro for reading non volatile register        Nonvolatile FLASH Protection Register; 0x0000FFBD */
/* Tip for register initialization in the user code:  const byte NVPROT_INIT @0x0000FFBD = <NVPROT_INITVAL>; */
/* NVOPT - macro for reading non volatile register         Nonvolatile Flash Options Register; 0x0000FFBF */
/* Tip for register initialization in the user code:  const byte NVOPT_INIT @0x0000FFBF = <NVOPT_INITVAL>; */


/* * * * *  16-BIT REGISTERS  * * * * * * * * * * * * * * * */
volatile ADCRSTR _ADCR;                                    /* Data Result Register; 0x00000012 */
volatile ADCCVSTR _ADCCV;                                  /* Compare Value Register; 0x00000014 */
volatile TPM1CNTSTR _TPM1CNT;                              /* TPM1 Timer Counter Register; 0x00000021 */
volatile TPM1MODSTR _TPM1MOD;                              /* TPM1 Timer Counter Modulo Register; 0x00000023 */
volatile TPM1C0VSTR _TPM1C0V;                              /* TPM1 Timer Channel 0 Value Register; 0x00000026 */
volatile TPM1C1VSTR _TPM1C1V;                              /* TPM1 Timer Channel 1 Value Register; 0x00000029 */
volatile TPM1C2VSTR _TPM1C2V;                              /* TPM1 Timer Channel 2 Value Register; 0x0000002C */
volatile TPM1C3VSTR _TPM1C3V;                              /* TPM1 Timer Channel 3 Value Register; 0x0000002F */
volatile SCI1BDSTR _SCI1BD;                                /* SCI1 Baud Rate Register; 0x00000038 */
volatile SCI2BDSTR _SCI2BD;                                /* SCI2 Baud Rate Register; 0x00000040 */
volatile SPI1D16STR _SPI1D16;                              /* SPI1 Data Register; 0x00000054 */
volatile SPI1MSTR _SPI1M;                                  /* SPI1 Match Register; 0x00000056 */
volatile TPM2CNTSTR _TPM2CNT;                              /* TPM2 Timer Counter Register; 0x00000061 */
volatile TPM2MODSTR _TPM2MOD;                              /* TPM2 Timer Counter Modulo Register; 0x00000063 */
volatile TPM2C0VSTR _TPM2C0V;                              /* TPM2 Timer Channel 0 Value Register; 0x00000066 */
volatile TPM2C1VSTR _TPM2C1V;                              /* TPM2 Timer Channel 1 Value Register; 0x00000069 */
volatile SPI2D16STR _SPI2D16;                              /* SPI2 Data Register; 0x00000074 */
volatile SPI2MSTR _SPI2M;                                  /* SPI2 Match Register; 0x00000076 */
volatile SDIDSTR _SDID;                                    /* System Device Identification Register; 0x00001806 */
volatile DBGCASTR _DBGCA;                                  /* Debug Comparator A Register; 0x00001810 */
volatile DBGCBSTR _DBGCB;                                  /* Debug Comparator B Register; 0x00001812 */
volatile DBGFSTR _DBGF;                                    /* Debug FIFO Register; 0x00001814 */

/*lint -restore */

/* EOF */
