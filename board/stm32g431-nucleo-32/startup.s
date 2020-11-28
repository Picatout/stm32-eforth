/* 
****************************************************
*  Adapted to STM32G431-NUCLEO-32 board by Picatout
*  date: 2020-11-19
*  IMPLEMENTATION NOTES:

*     Use USART2 instead of USART1
*     port config: 115200 8N1 
*     TX on  PA2,  RX on PA3 fed to on board STLINK V3 
*     STLINK V3 has a virtual com port /dev/ttyACM* 
*
*     return stack pointer use R13 instead of R2 
*
*     eForth is executed from flash, not copied to RAM
******************************************************

*****************************************************************************
*	STM32eForth version 7.20
*	Chen-Hanson Ting,  July 2014

*	Subroutine Threaded Forth Model
*	Adapted to STM32F407-Discovery Board
*	Assembled by Keil uVision 5.10

*	Version 4.03
*	Direct Threaded Forth Model
*	Derived from 80386 eForth versin 4.02
*	and Chien-ja Wu's ARM7 eForth version 1.01

*	Subroutine thread (Branch-Link) model
*	  Register assignments
*	IP	 	R0 	*scratch
*	SP	 	R1  * data stack pointer
*	RP	 	R2 * return stack pointer 
*	UP	 	R3 
*	WP	 	R4	*scratch 
*	TOS	 	R5  * to of data stack
*	XP	 	R6 	*scratch
*	YP	 	R7	*scratch
*	  All Forth words are called by 
*	BL.W	addr
*	  All low level code words are terminaled by
*	BX	LR 	(_NEXT)
*	  All high level Forth words start with
*	STRFD	RP!,{LR}	(_NEST)
*	  All high level Forth words end with
*	LDRFD	RP!,{PC}	(_UNNEST)
*	  Top of data stack is cached in R5
*	  USART1 at 115200 baud, 8 data bits, 1 stop bit, no parity
*	TX on PB6 and RX on PB7.

*	Version 5.02, 09oct04cht
*	fOR ADuC702x from Analog Devices
*	Version 6.01, 10apr08cht a
*	.align to at91sam7x256
*	Tested on Olimax SAM7-EX256 Board with LCD display
*	Running under uVision3 RealView from Keil
*	Version 7.01, 29jun14cht
*	Ported to STM32F407-Discovery Board, under uVision 5.10
*	.aligned to eForth 2 Model
*	Assembled to flash memory and executed therefrom.
*	Version 7.10, 30jun14cht
*	Flash memory mapped to Page 0 where codes are executed
*	Version 7.20, 02jul14cht
*	Irreducible Complexity
*	Code copied from flash to RAM, RAM mapped to Page 0.
*	TURNKEY saves current application from RAM to flash.
*********************************************************/

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  isr_vectors
.global  Default_Handler
.global  Reset_Handler

/* start address for the initialization values of the .data section. 
defined in linker script */
.word   _sidata
/* start address for the .data section. defined in linker script */  
.word   _sdata
/* end address for the .data section. defined in linker script */
.word   _edata
/* start address for the .bss section. defined in linker script */
.word   _sbss
/* end address for the .bss section. defined in linker script */
.word   _ebss
/* stack used for SystemInit_ExtMemCtl* always internal RAM used */

/******************************************************************************
*   interrupt service vectors table 
*******************************************************************************/
   .section  .isr_vector,"a",%progbits
  .type  isr_vectors, %object
  .size  isr_vectors, .-isr_vectors
 
isr_vectors:
  .word   _rstack          /* return stack address */
  .word   Reset_Handler    /* startup address */
/* core interrupts || exceptions */
  .word   Default_Handler  /*  NMI */
  .word   Default_Handler  /*  HardFault */
  .word   Default_Handler  /*  Memory Management */
  .word   Default_Handler  /* Bus fault */
  .word   Default_Handler  /* Usage fault */
  .word   0
  .word   0
  .word   0
  .word   0
  .word   Default_Handler  /* SWI instruction */
  .word   Default_Handler  /* Debug monitor */
  .word   0
  .word   Default_Handler  /* PendSV */
  .word   Default_Handler  /* Systick */
  
  /* External Interrupts */
  .word      Default_Handler /* IRQ0, Window WatchDog              */                                        
  .word      Default_Handler /* IRQ1, PVD_VDM */                        
  .word      Default_Handler /* IRQ2, RTC/TAMP/CSS_SLE */            
  .word      Default_Handler /* IRQ3, RTC Wakeup */                      
  .word      Default_Handler /* IRQ4, FLASH */                                          
  .word      Default_Handler /* IRQ5, RCC */                                            
  .word      Default_Handler /* IRQ6, EXTI Line0 */                        
  .word      Default_Handler /* IRQ7, EXTI Line1  */                          
  .word      Default_Handler /* IRQ8, EXTI Line2 */                          
  .word      Default_Handler /* IRQ9, EXTI Line3 */                          
  .word      Default_Handler /* IRQ10, EXTI Line4 */                          
  .word      Default_Handler /* IRQ11, DMA1 CH1 */                  
  .word      Default_Handler /* IRQ12, DMA1 CH2 */                   
  .word      Default_Handler /* IRQ13, DMA1 CH3 */                   
  .word      Default_Handler /* IRQ14, DMA1 CH4  */                   
  .word      Default_Handler /* IRQ15, DMA1 CH5 */                   
  .word      Default_Handler /* IRQ16, DMA1 CH6 */                   
  .word      Default_Handler /* IRQ17, DMA1 CH7 */                   
  .word      Default_Handler /* IRQ18, ADC1, ADC2 global interrupt */                   
  .word      Default_Handler /* IRQ19, USB High priority */                         
  .word      Default_Handler /* IRQ20, USB low priority */                          
  .word      Default_Handler /* IRQ21, FDCAN1 int 0 */                          
  .word      Default_Handler /* IRQ22, FDCAN1 int 1 */                          
  .word      Default_Handler /* IRQ23, External Line[9:5]s */                          
  .word      Default_Handler /* IRQ24, TIM1 Break and TIM15 global */         
  .word      Default_Handler /* IRQ25, TIM1 Update and TIM16 global */         
  .word      Default_Handler /* IRQ26, TIM1 Trigger and Commutation and TIM17 */
  .word      Default_Handler /* IRQ27, TIM1 Capture Compare */                          
  .word      Default_Handler /* IRQ28, TIM2 */                   
  .word      Default_Handler /* IRQ29, TIM3 */                   
  .word      Default_Handler /* IRQ30, TIM4 */                   
  .word      Default_Handler /* IRQ31, I2C1 Event and exti line 23 */                          
  .word      Default_Handler /* IRQ32, I2C1 Error */                          
  .word      Default_Handler /* IRQ33, I2C2 Event and exti line 24 */                          
  .word      Default_Handler /* IRQ34, I2C2 Error */                            
  .word      Default_Handler /* IRQ35, SPI1 */                   
  .word      Default_Handler /* IRQ36, SPI2 */                   
  .word      Default_Handler /* IRQ37, USART1 and exti line 25 */                   
  .word      Default_Handler /* IRQ38, USART2 and exti line 26 */                   
  .word      Default_Handler /* IRQ39, USART3 and exti line 38 */                   
  .word      Default_Handler /* IRQ40, External Line[15:10]s */                          
  .word      Default_Handler /* IRQ41, RTC Alarm */                 
  .word      Default_Handler /* IRQ42, USB OTG FS Wakeup*/                       
  .word      Default_Handler /* IRQ43, TIM8 Break */         
  .word      Default_Handler /* IRQ44, TIM8 Update*/         
  .word      Default_Handler /* IRQ45, TIM8 Trigger and Commutation */
  .word      Default_Handler /* IRQ46, TIM8 Capture Compare */                          
  .word      Default_Handler /* IRQ47, ADC3 global */                          
  .word      Default_Handler /* IRQ48, FSMC */                   
  .word      Default_Handler /* IRQ49, LPTIM1 */                   
  .word      Default_Handler /* IRQ50, TIM5 */                   
  .word      Default_Handler /* IRQ51, SPI3 */                   
  .word      Default_Handler /* IRQ52, UART4 and exti line 34 */                   
  .word      Default_Handler /* IRQ53, UART5 and exti line 35 */                   
  .word      Default_Handler /* IRQ54, TIM6 and DAC1&3 underrun errors */                   
  .word      Default_Handler /* IRQ55, TIM7 and DAC2&4 underrun */
  .word      Default_Handler /* IRQ56, DMA2 CH1 */                   
  .word      Default_Handler /* IRQ57, DMA2 CH2 */                   
  .word      Default_Handler /* IRQ58, DMA2 CH3 */                   
  .word      Default_Handler /* IRQ59, DMA2 CH4 */                   
  .word      Default_Handler /* IRQ60, DMA2 CH5 */                   
  .word      Default_Handler /* IRQ61, ADC4 */                   
  .word      Default_Handler /* IRQ62, ADC5 */                     
  .word      Default_Handler /* IRQ63, UCPD1 and exti line 43 */                          
  .word      Default_Handler /* IRQ64, COMP1/2/3 and exti 21/22/29 */                          
  .word      Default_Handler /* IRQ65, COMP4/5/6 and exti 30/31/32 */                          
  .word      Default_Handler /* IRQ66, COMP7 and exti 33 */                          
  .word      Default_Handler /* IRQ67, HRTIM master int 1 */                   
  .word      Default_Handler /* IRQ68, HRTIM A int 2 */                   
  .word      Default_Handler /* IRQ69, HRTIM B int 3*/                   
  .word      Default_Handler /* IRQ70, HRTIM C int 4 */                   
  .word      Default_Handler /* IRQ71, HRTIM D int 5 */                    
  .word      Default_Handler /* IRQ72, HRTIM E int 6 */                          
  .word      Default_Handler /* IRQ73, HRTIM fault int 8 */                          
  .word      Default_Handler /* IRQ74, HRTIM F int 7 */                   
  .word      Default_Handler /* IRQ75, CRS */                   
  .word      Default_Handler /* IRQ76, SAI */                         
  .word      Default_Handler /* IRQ77, TIM20_BREAK/TERR/IERR */                   
  .word      Default_Handler /* IRQ78, TIM20 update */                   
  .word      Default_Handler /* IRQ79, TIM20_TRG_COM/DIR/IDX */                   
  .word      Default_Handler /* IRQ80, TIM20_CC */
  .word      Default_Handler /* IRQ81, FPU */
  .word      Default_Handler /* IRQ82, I2C4_EV */      
  .word      Default_Handler /* IRQ83, I2C4_ERR */                        
  .word      Default_Handler /* IRQ84, SPI4 */
  .word      Default_Handler /* IRQ85, AES */
  .word      Default_Handler /* IRQ86, FDCAN2 INT0 */
  .word      Default_Handler /* IRQ87, FDCAN2 INT1 */
  .word      Default_Handler /* IRQ88, FDCAN3 INT0 */
  .word      Default_Handler /* IRQ89, FDCAN3 INT1 */
  .word      Default_Handler /* IRQ90, RNG */
  .word      Default_Handler /* IRQ91, LPUART */ 
  .word      Default_Handler /* IRQ92, I2C3_EV */
  .word      Default_Handler /* IRQ93, I2C3_ERR */
  .word      Default_Handler /* IRQ94, DMAMUX_OVR */
  .word      Default_Handler /* IRQ95, QUADSPI */
  .word      Default_Handler /* IRQ96, DMA1 CH8 */
  .word      Default_Handler /* IRQ97, DMA2 CH6 */
  .word      Default_Handler /* IRQ98, DMA2 CH7 */
  .word      Default_Handler /* IRQ99, DMA2 CH8 */
  .word      Default_Handler /* IRQ100, CORDIC */
  .word      Default_Handler /* IRQ101, FMAC */                   

/**************************************
  Reset_Handler execute at MCU reset
***************************************/
    .section  .text.Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:  
	BL	InitDevices	 	/* RCC, GPIOs, USART */
	BL	UNLOCK			/* unlock flash memory */
	LDR	R0,=COLD-MAPOFFSET	/* start Forth */
	BX	R0

	.align 4
 
/***********************************
* Here are devices used by eForth
***********************************/
  .equ RCC	,	0x40021000
  .equ GPIOA , 0x48000000
  .equ GPIOB , 0x48000400
  .equ USART2	,0x40004400
  .equ FLASH_ACR , 0x40022000

  .type InitDevices, %function 
InitDevices:

/* init clock to HSE 150 Mhz */
/* set 4 wait states in FLASH_ACR_LATENCY */
	ldr r0,=FLASH_ACR
  mov r2,#0x704
  str r2,[r0]
/* configure clock for HSE */
  ldr	r0,=RCC 		/* RCC */
  ldr r1,[r0]
  orr r1,r1,#(1<<16) /* HSEON bit */
  str r1,[r0] /* enable HSE */
/* wait HSERDY loop */
wait_hserdy:
  ldr r1,[r0]
  tst r1,#(1<<17)
  bne wait_hserdy
/* configure PLL N/M factors and source 
*  SYSCLOCK=150 Mhz
*/
  mov r1,#(3<<4)+(50<<8)+(3) /* M=4,N=50,PLLR=2 */
  movt r1,#(1<<8) /* PLLREN=1 */
  str r1,[r0,#0xc] /* RCC_PLLCFGR */
/* enable PLL */
  ldr r1,[r0]
  orr r1,r1,#(1<<24)
  str r1,[r0]
/* wait for PLLRDY */
wait_pllrdy:
  ldr r1,[r0]
  tst r1,#(1<<25)
  bne wait_pllrdy 
/* select PLL as sysclock and disable HSI */
  ldr r1,[r0,#8]
  mov r2,#0xfffc
  movt r2,#0xffff
  and r1,r1,r2 
  mov r2,#3
  orr r1,r1,r2
  str r1,[r0,#8] /* PLL selected as sysclock */
/* wait for SWS==3 */
wait_sws:
  ldr r1,[r0,#8]
  tst r1,#(3<<2)
  bne wait_sws
  ldr r1,[r0]
  bic r1,r1,#(1<<8)
  str r1,[r0] /* HSI disabled */
/* now sysclock is 150 Mhz */

/* enable peripheral GPIOA and GPIOB */
	ldr	r1, [r0, #0x4c]	/* RCC_AHB2ENR */
	orr	r1, #3		/* GPIOAEN */
	str	r1, [r0, #0x4c]

/* enable USART2 */ 
	ldr	r1, [r0, #0x58]	/* RCC_APB1ENR1 */
	mov r2,#1
  lsl r2,r2,#17
  orr	r1,r1,r2		/* USART2EN (1 << 17) */
	str	r1, [r0, #0x44]

/* init GPIOB:8 as output for user LED  */
	ldr	r0, =GPIOB /* GPIOB */
	ldr	r1, [r0, #0x00]	/* GPIOx_MODER */
	mov r2,#1
  bic	r1,r1,r2, lsl #16   	/* =output Mode */
	str	r1, [r0, #0x00]
/* turn on LED */
  mov r2,#(1<<8) /* LED on bit 8 */
  str r2,[r0,#0x14] /* GPIO_ODR */
  b .
/* init USART2 */
	ldr	r0, =USART2 	/* USART */
	movw	r1, #0x0200C	/* enable USART */ 
	strh	r1, [r0, #12]	/* +12 USART_CR1 = 0x2000 */
	movs	r1, #139		/* 16MHz/8.6875 (139, 0x8B) == 115200 */
	strh	r1, [r0, #8]	/*  +8 USART_BR */
/* GREEN LED, Configure PA8 as output with push-pull */
	ldr	r0, =GPIOA 	/* GPIOA */
	mov	r1, #0x55000000	/* output */
	str	r1, [r0, #0x00]
	mov	r1, #0xF000 	/* set PA8, turn on LEDs */
	str	r1, [r0, #0x14]
	bx	lr

	.align 4
	.org .
 
/********************
* Version control
*******************/
.equ VER ,	0x07	/*major release version */
.equ EXT ,	0x20	/*minor extension */

/* Constants */

/*.equ RAMOFFSET  ,	0x00000000 */	/*absolute */
/*.equ MAPOFFSET  ,	0x00000000 */	/*absolute */
.equ RAMOFFSET  ,	0x20000000	/*remap */
.equ MAPOFFSET  ,	0x08000000	/*remap */

.equ COMPO ,	0x040	/*lexicon compile only */ 
.equ IMEDD ,	0x080	/*lexicon immediate bit */
.equ MASKK ,	0x0FFFFFF1F	/*lexicon bit mask, allowed for Chineze character */

.equ CELLL ,	4	/*size of a cell */
.equ BASEE ,	16	/*default radix */
.equ VOCSS ,	8	/*depth of vocabulary stack */

.equ BKSPP ,	8	/*backspace */
.equ LF ,	10	/*line feed */
.equ CRR ,	13	/*carriage return */
.equ ERR ,	27	/*error escape */
.equ TIC ,	39	/*tick */

/********************************************************
* Memory allocation	0//code>--//--<sp//tib>--rp//user//
*	  0000	RAM memory mapped to Page 0, Reset vector
* 	0008	init devices
* 	00C0	initial system variables
* 	0100	Forth dictionary
* 	2150	top of dictionary, HERE
* 	2154	WORD buffer
* 	FE00	top of data stack
* 	FE00	TIB terminal input buffer
* 	FF00	top of return stack
* 	FF00	system variables
* 	8000000	flash, code image
* 	1000400	top of hardware stack for interrupts
* 	20000000	RAM
********************************************************/

.equ SPP 	,	0x2000FE00-RAMOFFSET	/*top of data stack (SP0) */
.equ TIBB ,	0x2000FE00-RAMOFFSET	/*terminal input buffer (TIB) */
.equ RPP 	,	0x2000FF00-RAMOFFSET	/*top of return stack (RP0) */
.equ UPP 	,	0x2000FF00-RAMOFFSET	/*start of user area (UP0) */
.equ DTOP ,	0x2000FC00-RAMOFFSET	/*start of usable RAM area (HERE) */

/***********************************************
* MACROS
*	Assemble inline direct threaded code ending.
***********************************************/
 	.macro	_NEXT /*end low level word */
	BX	LR
	.endm

 	.macro	_NEST /*start high level word */
	STMFD	R2!,{LR}
	.endm

 	.macro	_UNNEST	/*end high level word */
	LDMFD	R2!,{PC}
	.endm

 	.macro	_DOLIT /*long literals */
	BL	DOLIT
	.endm

 	.macro	_PUSH	/*push R5 on data stack*/
	STR	R5,[R1,#-4]!
	.endm

 	.macro	_POP /*pop data stack to R5 */
	LDR	R5,[R1],#4
	.endm

/******************************************************
*  COLD start moves the following to USER variables.
*  MUST BE IN SAME ORDER AS USER VARIABLES.
******************************************************/
	.align 6  	/* .align to page boundary */

UZERO:
	.word 0  			/*Reserved
	.word HI-MAPOFFSET  	/*'BOOT
	.word BASEE  		/*BASE
	.word 0			/*tmp
	.word 0			/*SPAN
	.word 0			/*>IN
	.word 0			/*#TIB
	.word TIBB			/*TIB
	.word INTER-MAPOFFSET	/*'EVAL
	.word 0			/*HLD
	.word LASTN-MAPOFFSET	/*CONTEXT
	.word CTOP-MAPOFFSET	/*FLASH
	.word CTOP-MAPOFFSET	/*RAM
	.word LASTN-MAPOFFSET	/*LAST
	.word 0,0			/*reserved
ULAST:

/*****************************************************
* default isr handler called on unexpected interrupt
*****************************************************/
    .section  .text.Default_Handler,"ax",%progbits

  .type Default_Handler, %function
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler
