//======================================================================
// Register definitions and utility code for STM32F$x
// Created by Anton Rieckert 2012 - anton@riecktron.com
//======================================================================
unit stm32f0xx;

{$goto on}
{$DEFINE stm32f0xx}

//======================================================================
interface

//======================================================================
type
 TBitvector32 = bitpacked array[0..31] of 0..1;

//======================================================================
{$PACKRECORDS 2}
const
  FLASH_BASE            = $08000000;     // FLASH base address in the alias region 
  SRAM_BASE             = $20000000;     // SRAM base address in the alias region 
  PERIPH_BASE           = $40000000;     // Peripheral base address in the alias region 
  SCS_BASE              = $E000E000;     // System Control Space Base Address 
  CoreDebug_BASE        = $E000EDF0;     // Core Debug Base Address           

  // Peripheral memory map 
  APBPERIPH_BASE        = PERIPH_BASE;
  AHBPERIPH_BASE        = (PERIPH_BASE + $00020000);
  AHB2PERIPH_BASE       = (PERIPH_BASE + $08000000);

const
  // Cortex-M0 Processor Exceptions Numbers
  NonMaskableInt_IRQn         = -14;     // Non Maskable Interrupt        
  HardFault_IRQn              = -13;     // Cortex-M0 Hard Fault Interrupt
  SVC_IRQn                    = -5;      // Cortex-M0 SV Call Interrupt   
  PendSV_IRQn                 = -2;      // Cortex-M0 Pend SV Interrupt   
  SysTick_IRQn                = -1;      // Cortex-M0 System Tick Interrupt

  // STM32F-0 specific Interrupt Numbers
  WWDG_IRQn                   = 0;       // Window WatchDog Interrupt                              
  PVD_IRQn                    = 1;       // PVD through EXTI Line detect Interrupt                 
  RTC_IRQn                    = 2;       // RTC through EXTI Line Interrupt                        
  FLASH_IRQn                  = 3;       // FLASH Interrupt                                        
  RCC_IRQn                    = 4;       // RCC Interrupt                                          
  EXTI0_1_IRQn                = 5;       // EXTI Line 0 and 1 Interrupts                           
  EXTI2_3_IRQn                = 6;       // EXTI Line 2 and 3 Interrupts                           
  EXTI4_15_IRQn               = 7;       // EXTI Line 4 to 15 Interrupts                           
  TS_IRQn                     = 8;       // TS Interrupt                                           
  DMA1_Channel1_IRQn          = 9;       // DMA1 Channel 1 Interrupt                               
  DMA1_Channel2_3_IRQn        = 10;      // DMA1 Channel 2 and Channel 3 Interrupts                
  DMA1_Channel4_5_IRQn        = 11;      // DMA1 Channel 4 and Channel 5 Interrupts                
  ADC1_COMP_IRQn              = 12;      // ADC1, COMP1 and COMP2 Interrupts                       
  TIM1_BRK_UP_TRG_COM_IRQn    = 13;      // TIM1 Break, Update, Trigger and Commutation Interrupts 
  TIM1_CC_IRQn                = 14;      // TIM1 Capture Compare Interrupt                         
  TIM2_IRQn                   = 15;      // TIM2 Interrupt                                         
  TIM3_IRQn                   = 16;      // TIM3 Interrupt                                         
  TIM6_DAC_IRQn               = 17;      // TIM6 and DAC Interrupts                                
  TIM14_IRQn                  = 19;      // TIM14 Interrupt                                        
  TIM15_IRQn                  = 20;      // TIM15 Interrupt                                        
  TIM16_IRQn                  = 21;      // TIM16 Interrupt                                        
  TIM17_IRQn                  = 22;      // TIM17 Interrupt                                        
  I2C1_IRQn                   = 23;      // I2C1 Interrupt                                         
  I2C2_IRQn                   = 24;      // I2C2 Interrupt                                         
  SPI1_IRQn                   = 25;      // SPI1 Interrupt                                         
  SPI2_IRQn                   = 26;      // SPI2 Interrupt                                         
  USART1_IRQn                 = 27;      // USART1 Interrupt                                       
  USART2_IRQn                 = 28;      // USART2 Interrupt                                       
  CEC_IRQn                    = 30;      // CEC Interrupt                                          

//======================================================================
type
  TTimerRegisters = record
    CR1, res1   : word;
    CR2, res2   : word;
    SMCR, res3  : word;
    DIER, res4  : word;
    SR, res5    : word;
    EGR, res    : word;
    CCMR1, res6 : word;
    CCMR2, res7 : word;
    CCER, res8  : word;
    CNT         : dword;
    PSC, res10  : word;
    ARR         : dword;
    RCR, res12  : word;
    CCR1        : dword;
    CCR2        : dword;
    CCR3        : dword;
    CCR4        : dword;
    BDTR, res17 : word;
    DCR, res18  : word;
    DMAR, res19 : word;  
    DOR, res20  : word;
  end;

//======================================================================
type
  TRTCRegisters = record
     TR        : dword;      
     DR        : dword;      
     CR        : dword;      
     ISR       : dword;      
     PRER      : dword;      
     RESERVED0 : dword;  
     RESERVED1 : dword;  
     ALRMAR    : dword;  
     RESERVED2 : dword;  
     WPR       : dword;  
     SSR       : dword;  
     SHIFTR    : dword;  
     TSTR      : dword;  
     TSDR      : dword;  
     TSSSR     : dword;  
     CAL       : dword;  
     TAFCR     : dword;  
     ALRMASSR  : dword;  
     RESERVED3 : dword;  
     RESERVED4 : dword;  
     BKP0R     : dword;  
     BKP1R     : dword;  
     BKP2R     : dword;  
     BKP3R     : dword;  
     BKP4R     : dword;  
  end;

//======================================================================
type
  TIWDGRegisters = record
    KR   : dword;
    PR   : dword;
    RLR  : dword;
    SR   : dword;
    WINR : dword;
  end;

//======================================================================
type
  TWWDGRegisters = record
    CR  : dword;
    CFR : dword;
    SR  : dword;
  end;

//======================================================================
type
  TSPIRegisters = record
    CR1, res1     : word;
    CR2, res2     : word;
    SR, res3      : word;
    DR, res4      : word;
    CRCPR, res5   : word;
    RXCRCR, res6  : word;
    TXCRCR, res7  : word;
    I2SCFGR, res8 : word;
    I2SPR, res9   : word;
  end;

//======================================================================
type
  TUSARTRegisters = record
    CR1       : dword;   
    CR2       : dword;   
    CR3       : dword;   
    BRR       : word;    
    RESERVED1 : word;  
    GTPR      : word;   
    RESERVED2 : word;  
    RTOR      : dword;  
    RQR       : word;    
    RESERVED3 : word;  
    ISR       : dword;   
    ICR       : dword;   
    RDR       : word;    
    RESERVED4 : word;  
    TDR       : word;    
    RESERVED5 : word;  
  end;

//======================================================================
type
  TI2CRegisters = record
    CR1      : dword;      
    CR2      : dword;      
    OAR1     : dword;     
    OAR2     : dword;     
    TIMINGR  : dword;  
    TIMEOUTR : dword; 
    ISR      : dword;      
    ICR      : dword;      
    PECR     : dword;     
    RXDR     : dword;     
    TXDR     : dword;     
  end;

//======================================================================
type
  TPwrRegisters = record
    CR  : dword;
    CSR : dword;
  end;
  
//======================================================================
type
  TCECRegisters = record
    CR   : dword;
    CFGR : dword;     
    TXDR : dword;     
    RXDR : dword;     
    ISR  : dword;     
    IER  : dword;     
  end;    

//======================================================================
type
  TSYSCFGRegisters = record
    CFGR1  : dword;
    RESERVED : dword;
    EXTICR : array[0..3] of dword;
    CFGR2 : dword;
  end;

//======================================================================
type
  TCOMPRegisters = record
    CSR : dword;
  end;  

//======================================================================
type
  TADCRegisters = record
    ISR : dword;         
    IER : dword;         
    CR : dword;          
    CFGR1 : dword;       
    CFGR2 : dword;       
    SMPR : dword;        
    RESERVED1 : dword;   
    RESERVED2 : dword;   
    TR : dword;          
    RESERVED3 : dword;   
    CHSELR : dword;      
    RESERVED4 : array[0..4] of dword;    
    DR : dword;         
  end;

//======================================================================
type
  TADCCommonRegisters = record
    CCR : dword; 
  end;

//======================================================================
type
  TDBGMCURegisters = record
    IDCODE : dword;    
    CR : dword;        
    APB1FZ : dword;    
    APB2FZ : dword;    
  end;
  
//======================================================================
type
  TEXTIRegisters = record
    IMR : dword;
    EMR : dword;
    RTSR : dword;
    FTSR : dword;
    SWIER : dword;
    PR : dword;
  end;

//======================================================================
type
  TPortRegisters = record
    MODER : dword;       
    OTYPER : word;       
    RESERVED0 : word;    
    OSPEEDR : dword;     
    PUPDR : dword;       
    IDR : word;          
    RESERVED1 : word;    
    ODR : word;          
    RESERVED2 : word;    
    BSRR : dword;        
    LCKR : dword;        
    AFR : array[0..1] of dword; 
    BRR : word;          
    RESERVED3 : word;    
  end;

//======================================================================
type
  TRCCRegisters = record
    CR : dword;
    CFGR : dword;
    CIR : dword;
    APB2RSTR : dword;
    APB1RSTR : dword;
    AHBENR : dword;
    APB2ENR : dword;
    APB1ENR : dword;
    BDCR : dword;
    CSR  : dword;
    AHBRSTR : dword;
    CFGR2 : dword;
    CFGR3 : dword;
    CR2 : dword;
  end;

//======================================================================
type
  TFlashRegisters = record
    ACR : dword;
    KEYR : dword;
    OPTKEYR : dword;
    SR : dword;
    CR : dword;
    AR : dword;
    RESERVED : dword;
    OBR : dword;
    WRPR : dword;
  end;

//======================================================================
type
  TOBRegisters = record
    RDP : word;
    USER : word;
    RESERVED0 : word;
    RESERVED1 : word;
    WRP0 : word;
    WRP1 : word;
  end;

//======================================================================
type
  TNVICRegisters = record
    ISER: array[0..0] of dword;
     reserved0: array[0..30] of dword;
    ICER: array[0..0] of dword;
     reserved1: array[0..30] of dword;
    ISPR: array[0..0] of dword;
     reserved2: array[0..30] of dword;
    ICPR: array[0..0] of dword;
     reserved3: array[0..30] of dword;
     reserved4: array[0..63] of dword;
    IP: array[0..7] of dword;
  end;

//======================================================================
type
  TSCBRegisters = record
    CPUID : dword;
    ICSR : dword;
    RESERVED0 : dword;
    AIRCR : dword;
    SCR : dword;
    CCR : dword;
    RESERVED1 : dword;
    SHP : array[0..1] of dword;
    SHCSR : dword;
  end;

//======================================================================
type
  TSysTickRegisters = record
    CTRL : dword;
    LOAD : dword;
    VAL : dword;
    CALIB : dword;
  end;


  
 TDACRegisters = record
  CR,
  SWTRIGR: DWord;

  DHR12R1, res2,
  DHR12L1, res3,
  DHR8R1, res4,
  DHR12R2, res5,
  DHR12L2, res6,
  DHR8R2, res7: word;

  DHR12RD,
  DHR12LD: DWord;

  DHR8RD, res8,

  DOR1, res9,
  DOR2, res10: Word;
 end;

 TDMAChannel = record
  CCR, res1,
  CNDTR, res2: word;
  CPAR,
  CMAR,
  res: DWord;
 end;

 TDMARegisters = record
  ISR,
  IFCR: DWord;
  Channel: array[0..7] of TDMAChannel;
 end;


 TCRCRegisters = record
  DR: DWord;
  IDR: byte; res1: word; res2: byte;
  CR: byte;
 end;

{$ALIGN 2}
var
  // GPIO 
  PortA: TPortRegisters        absolute (AHB2PERIPH_BASE + $00000000);
  PortB: TPortRegisters        absolute (AHB2PERIPH_BASE + $00000400);
  PortC: TPortRegisters        absolute (AHB2PERIPH_BASE + $00000800);
  PortD: TPortRegisters        absolute (AHB2PERIPH_BASE + $00000C00);
  PortF: TPortRegisters        absolute (AHB2PERIPH_BASE + $00001400);

  // Timers
  Timer1: TTimerRegisters      absolute (APBPERIPH_BASE + $00012C00);
  Timer2: TTimerRegisters      absolute (APBPERIPH_BASE + $00000000);
  Timer3: TTimerRegisters      absolute (APBPERIPH_BASE + $00000400);
  Timer6: TTimerRegisters      absolute (APBPERIPH_BASE + $00001000);
  Timer14: TTimerRegisters     absolute (APBPERIPH_BASE + $00002000);
  Timer15: TTimerRegisters     absolute (APBPERIPH_BASE + $00014000);
  Timer16: TTimerRegisters     absolute (APBPERIPH_BASE + $00014400);
  Timer17: TTimerRegisters     absolute (APBPERIPH_BASE + $00014800);

  // RTC 
  RTC: TRTCRegisters           absolute (APBPERIPH_BASE + $00002800);

  // WDG 
  WWDG: TWWDGRegisters         absolute (APBPERIPH_BASE + $00002C00);
  IWDG: TIWDGRegisters         absolute (APBPERIPH_BASE + $00003000);

  // SPI 
  SPI1: TSPIRegisters          absolute (APBPERIPH_BASE + $00013000);
  SPI2: TSPIRegisters          absolute (APBPERIPH_BASE + $00003800);

  // USART/UART 
  USART1: TUSARTRegisters      absolute (APBPERIPH_BASE + $00013800);
  USART2: TUSARTRegisters      absolute (APBPERIPH_BASE + $00004400);

  // I2C 
  I2C1: TI2CRegisters          absolute (APBPERIPH_BASE + $00005400);
  I2C2: TI2CRegisters          absolute (APBPERIPH_BASE + $00005800);

  // PWR
  PWR: TPwrRegisters           absolute (APBPERIPH_BASE + $00007000);

  // DAC 
  DAC: TDACRegisters           absolute (APBPERIPH_BASE + $00007400);

  // SCB 
  SCB: TSCBRegisters           absolute (SCS_BASE + $00000D00);
  
  // SysTick 
  SysTick: TSysTickRegisters   absolute (SCS_BASE + $00000010);
  
  // NVIC 
  NVIC: TNVICRegisters         absolute (SCS_BASE +$00000100);

  // RCC 
  RCC: TRCCRegisters           absolute (AHBPERIPH_BASE + $00001000);

  // Flash 
  Flash: TFlashRegisters       absolute (AHBPERIPH_BASE + $00002000);

  //////
  CEC: TCECRegisters           absolute (APBPERIPH_BASE + $00007800);
  SYSCFG: TSYSCFGRegisters     absolute (APBPERIPH_BASE + $00010000);
  COMP: TCOMPRegisters         absolute (APBPERIPH_BASE + $0001001C);
  EXTI: TEXTIRegisters         absolute (APBPERIPH_BASE + $00010400); 
  ADC1: TADCRegisters          absolute (APBPERIPH_BASE + $00012400);
  ADC: TADCCommonRegisters     absolute (APBPERIPH_BASE + $00012708);
  DBGMCU: TDBGMCURegisters     absolute (APBPERIPH_BASE + $00015800);

{  DMA1                         absolute (AHBPERIPH_BASE + $00000000); //((DMA_TypeDef *) DMA1_BASE)
  DMA1_Channel1                absolute (DMA1_BASE + $00000008); //((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
  DMA1_Channel2                absolute (DMA1_BASE + $0000001C); //((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
  DMA1_Channel3                absolute (DMA1_BASE + $00000030); //((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
  DMA1_Channel4                absolute (DMA1_BASE + $00000044); //((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
  DMA1_Channel5                absolute (DMA1_BASE + $00000058); //((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
  OB                           absolute ($1FFFF800); //((OB_TypeDef *) OB_BASE) 
  CRC                          absolute (AHBPERIPH_BASE + $00003000); //((CRC_TypeDef *) CRC_BASE)
  TSC                          absolute (AHBPERIPH_BASE + $00004000); //((TSC_TypeDef *) TSC_BASE)}
 
implementation

var
  _data: record end; external name '_data';
  _edata: record end; external name '_edata';
  _etext: record end; external name '_etext';
  _bss_start: record end; external name '_bss_start';
  _bss_end: record end; external name '_bss_end';
  _stack_top: record end; external name '_stack_top';

procedure PASCALMAIN; external name 'PASCALMAIN';

procedure DefaultHandler; public name 'DefaultHandler';
begin

end;

procedure _FPC_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhalt:
  b .Lhalt
end;

procedure _FPC_start; assembler; nostackframe; interrupt 0;
label _start;
asm
  .globl _start
_start:
  
  // Copy initialized data to ram
  ldr r1,.L_etext
  ldr r2,.L_data
  ldr r3,.L_edata
.Lcopyloop:
  cmp r2,r3
  ittt ls
  ldrls r0,[r1],#4
  strls r0,[r2],#4
  bls .Lcopyloop

  // clear onboard ram
  ldr r1,.L_bss_start
  ldr r2,.L_bss_end
  mov r0,#0
.Lzeroloop:
  cmp r1,r2
  itt ls
  strls r0,[r1],#4
  bls .Lzeroloop

  bl PASCALMAIN
  b _FPC_haltproc

.L_bss_start:
  .long _bss_start
.L_bss_end:
  .long _bss_end
.L_etext:
  .long _etext
.L_data:
  .long _data
.L_edata:
  .long _edata
end;

end.

