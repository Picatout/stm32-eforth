/* 
****************************************************
*  STM32eForth version 7.20
*  Adapted to blue pill board by Picatout
*  date: 2020-11-22
*  IMPLEMENTATION NOTES:
* 
*	This version use indirect threaded model. This model enable 
*	leaving the core Forth system in FLASH memory while the users 
*	definitions reside in RAM. 
*	R0 is used as IP , inner interpreter address pointer 
*	R4 is used as WP , working register 
*
*     Use USART1 for console I/O
*     port config: 115200 8N1 
*     TX on  PA9,  RX on PA10  
*
*     eForth is executed from flash, not copied to RAM
*     eForth use main stack R13 as return stack (thread stack not used) 
*
*     Forth return stack is at end of RAM (addr=0x200005000) and reserve 512 bytes
*     a 128 bytes flwr_buffer is reserved below rstack for flash row writing
*     a 128 bytes tib is reserved below flwr_buffer 
*     Forth dstack is below tib and reserve 512 bytes 
*   
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
*	TOS	 	R5  * top of data stack
*	XP	 	R6 	*scratch
*	YP	 	R7	*scratch
*	  All Forth words are called by 
*	BL.W	addr
*	  All low level code words are terminaled by
*	BX	LR 	(_RET)
*	  All high level Forth words start with
*	STRFD	RP!,{LR}	(_NEST)
*	  All high level Forth words end with
*	LDRFD	RP!,{PC}	(_UNNEST)
*	  Top of data stack is cached in R5
*	  USART1 at 115200 baud, 8 data bits, 1 stop bit, no parity
*	TX on PA9 and RX on PA10.

*	Version 5.02, 09oct04cht
*	fOR ADuC702x from Analog Devices
*	Version 6.01, 10apr08cht a
*	.p2align 2 to at91sam7x256
*	Tested on Olimax SAM7-EX256 Board with LCD display
*	Running under uVision3 RealView from Keil
*	Version 7.01, 29jun14cht
*	Ported to STM32F407-Discovery Board, under uVision 5.10
*	.p2aligned to eForth 2 Model
*	Assembled to flash memory and executed therefrom.
*	Version 7.10, 30jun14cht
*	Flash memory mapped to Page 0 where codes are executed
*	Version 7.20, 02jul14cht
*	Irreducible Complexity
*	Code copied from flash to RAM, RAM mapped to Page 0.
*	TURNKEY saves current application from RAM to flash.
*********************************************************/

  .syntax unified
  .cpu cortex-m3
  .fpu softvfp
  .thumb

  .include "board/blue-pill/stm32f103.inc"

/* blue pill specific constants */ 
  .equ LED_GPIO, GPIOC_BASE_ADR
  .equ LED_PIN, 13
  .equ UART, USART1_BASE_ADR 

/* eForth specific constants */
.equ SPP ,	0x20004E80	/*top of data stack (SP0) */
.equ TIBB ,	0x20004E80	/*terminal input buffer (TIB) */
.equ RPP ,	0x20004F80	/*top of return stack (RP0) */
.equ UPP ,	0x20000000	/*start of user area (UP0) */
.equ DTOP ,	0x20000100	/*start of usable RAM area (HERE) */
.equ DEND , 0x20004E00  /*usable RAM end */
 .equ RAMOFFSET ,	0x20000000	// remap
 .equ RAMEND, 0x20005000 // 20Ko

/*************************************
   system variables offset from UPP
*************************************/
  .equ FTRACE_OFS,4 // tracing flag 
  .equ SEED_OFS, FTRACE_OFS+4    // prng seed 
  .equ TICKS_OFS, SEED_OFS+4  // millseconds counter
  .equ TIMER_OFS, TICKS_OFS+4  // count down timer
  .equ TORAM_OFS, TIMER_OFS+4  // compile to RAM 
  .equ IMG_SIGN_OFS, TORAM_OFS+4  // image signature  
  .equ BOOT_OFS, IMG_SIGN_OFS+4  // boot program address
  .equ BASE_OFS, BOOT_OFS+4  // numeric conversion base 
  .equ TMP_OFS, BASE_OFS+4   // temporary variable
  .equ SPAN_OFS, TMP_OFS+4  // character count received by expect  
  .equ TOIN_OFS, SPAN_OFS+4  // >IN  parse pointer in TIB
  .equ NTIB_OFS, TOIN_OFS+4  // #TIB  characters in TIB 
  .equ TIB_OFS, NTIB_OFS+4   // TIB buffer address 
  .equ EVAL_OFS, TIB_OFS+4  // eval|compile vector 
  .equ HLD_OFS, EVAL_OFS+4   // hold pointer 
  .equ CTXT_OFS, HLD_OFS+4  // context pointer 
  .equ FORTH_CTOP_OFS, CTXT_OFS+4  // flash free dictionary address 
  .equ RAM_CTOP_OFS, FORTH_CTOP_OFS+4  // ram free dictionary address
  .equ LASTN_OFS, RAM_CTOP_OFS+4     // last word in dictionary link nfa 
  .equ VARS_END_OFS, LASTN_OFS+4 // end of system variables  

/***********************************************
* MACROS
*	Assemble inline direct threaded code ending.
***********************************************/
	.macro _CALL fn /* low level routine call */ 
 	PUSH {LR}
	BL \fn  
	POP {LR}
	.endm
	
	.macro	_RET /*exit low level call */
	BX LR 
	.endm

	.macro _NEXT /* direct jump INEXT */ 
	B INEXT 
	.endm 

 	.macro	_NEST /*start high level word */
	BL NEST 
	.endm

	.macro _ADR label 
	.word 1+\label  
	.endm 

 	.macro	_UNNEST	/*end high level word */
	_ADR UNNEST 
	.endm

 	.macro	_DOLIT u /*long literals */
	_ADR	DOLIT
	.word	\u 
	.endm

	.macro _DOTQP len, text 
	_ADR DOTQP 
	.byte \len 
	.ascii "\text" 
	.p2align 2 
	.endm

	.macro _COMPI name 
	_ADR COMPI 
	.word \name 
	.endm 

	.macro _ABORQ len,name 
	_ADR ABORQ 
	.byte \len 
	.ascii "\name"
	.p2align 2 
	.endm

	.macro _BRAN label 
	_ADR BRAN 
	.word \label 
	.endm 

	.macro _QBRAN label 
	_ADR QBRAN 
	.word \label
	.endm 

	.macro _DONXT label 
	_ADR DONXT
	.word \label 
	.endm 

 	.macro	_PUSH	/*push R5 on data stack*/
	STR	R5,[R1,#-4]!
	.endm

 	.macro	_POP /*pop data stack to R5 */
	LDR	R5,[R1],#4
	.endm

/*************************************
*   interrupt service vectors table 
**************************************/
   .section  .isr_vector,"a",%progbits
  .type  isr_vectors, %object

isr_vectors:
  .word   _mstack          /* main return stack address */
  .word   reset_handler    /* startup address */
/* core interrupts || exceptions */
  .word   default_handler  /*  NMI */
  .word   default_handler  /*  HardFault */
  .word   default_handler  /*  Memory Management */
  .word   default_handler  /* Bus fault */
  .word   default_handler  /* Usage fault */
  .word   0
  .word   0
  .word   0
  .word   0
  .word   default_handler  /* SWI instruction */
  .word   default_handler  /* Debug monitor */
  .word   0
  .word   default_handler  /* PendSV */
  .word   systick_handler  /* Systick */
  
  /* External Interrupts */
  .word      default_handler /* IRQ0, Window WatchDog              */                                        
  .word      default_handler /* IRQ1, PVD_VDM */                        
  .word      default_handler /* IRQ2, TAMPER */            
  .word      default_handler /* IRQ3, RTC  */                      
  .word      default_handler /* IRQ4, FLASH */                                          
  .word      default_handler /* IRQ5, RCC */                                            
  .word      default_handler /* IRQ6, EXTI Line0 */                        
  .word      default_handler /* IRQ7, EXTI Line1  */                          
  .word      default_handler /* IRQ8, EXTI Line2 */                          
  .word      default_handler /* IRQ9, EXTI Line3 */                          
  .word      default_handler /* IRQ10, EXTI Line4 */                          
  .word      default_handler /* IRQ11, DMA1 CH1 */                  
  .word      default_handler /* IRQ12, DMA1 CH2 */                   
  .word      default_handler /* IRQ13, DMA1 CH3 */                   
  .word      default_handler /* IRQ14, DMA1 CH4  */                   
  .word      default_handler /* IRQ15, DMA1 CH5 */                   
  .word      default_handler /* IRQ16, DMA1 CH6 */                   
  .word      default_handler /* IRQ17, DMA1 CH7 */                   
  .word      default_handler /* IRQ18, ADC1, ADC2 global interrupt */                   
  .word      default_handler /* IRQ19, USB High priority */                         
  .word      default_handler /* IRQ20, USB low priority */                          
  .word      default_handler /* IRQ21, CAN_RX1 */                          
  .word      default_handler /* IRQ22, CAN1_SCE */                          
  .word      default_handler /* IRQ23, External Line[9:5]s */                          
  .word      default_handler /* IRQ24, TIM1 Break and TIM15 global */         
  .word      default_handler /* IRQ25, TIM1 Update and TIM16 global */         
  .word      default_handler /* IRQ26, TIM1 Trigger and Commutation and TIM17 */
  .word      default_handler /* IRQ27, TIM1 Capture Compare */                          
  .word      default_handler /* IRQ28, TIM2 */                   
  .word      default_handler /* IRQ29, TIM3 */                   
  .word      default_handler /* IRQ30, TIM4 */                   
  .word      default_handler /* IRQ31, I2C1 Event and exti line 23 */                          
  .word      default_handler /* IRQ32, I2C1 Error */                          
  .word      default_handler /* IRQ33, I2C2 Event and exti line 24 */                          
  .word      default_handler /* IRQ34, I2C2 Error */                            
  .word      default_handler /* IRQ35, SPI1 */                   
  .word      default_handler /* IRQ36, SPI2 */                   
  .word      default_handler /* IRQ37, USART1 */                   
  .word      default_handler /* IRQ38, USART2 */                   
  .word      default_handler /* IRQ39, USART3 */                   
  .word      default_handler /* IRQ40, External Line[15:10]s */                          
  .word      default_handler /* IRQ41, RTC Alarm */                 
  .word      default_handler /* IRQ42, USB Wakeup*/                       
  .word      default_handler /* IRQ43, TIM8 Break */         
  .word      default_handler /* IRQ44, TIM8 Update*/         
  .word      default_handler /* IRQ45, TIM8 Trigger and Commutation */
  .word      default_handler /* IRQ46, TIM8 Capture Compare */                          
  .word      default_handler /* IRQ47, ADC3 global */                          
  .word      default_handler /* IRQ48, FSMC */                   
  .word      default_handler /* IRQ49, SDIO */                   
  .word      default_handler /* IRQ50, TIM5 */                   
  .word      default_handler /* IRQ51, SPI3 */                   
  .word      default_handler /* IRQ52, UART4 */                   
  .word      default_handler /* IRQ53, UART5 */                   
  .word      default_handler /* IRQ54, TIM6 */                   
  .word      default_handler /* IRQ55, TIM7 */
  .word      default_handler /* IRQ56, DMA2 CH1 */                   
  .word      default_handler /* IRQ57, DMA2 CH2 */                   
  .word      default_handler /* IRQ58, DMA2 CH3 */                   
  .word      default_handler /* IRQ59, DMA2 CH4 & CH5 */                   
  .size  isr_vectors, .-isr_vectors

	.p2align 8

UZERO:
	.word 0  			/*Reserved */
	.word 0		/* FTRACE */ 
	.word 0xaa55 /* SEED  */ 
	.word 0      /* MSEC */
    .word 0     /* TIMER */
	.word -1    /* >RAM */ 
	.ascii "IMAG" /* signature */ 
	.word HI  /*'BOOT */
	.word BASEE 	/*BASE */
	.word 0			/*tmp */
	.word 0			/*SPAN */
	.word 0			/*>IN */
	.word 0			/*#TIB */
	.word TIBB	/*TIB */
	.word INTER	/*'EVAL */
	.word 0			/*HLD */
	.word LASTN	/*CONTEXT */
	.word CTOP	/* FCP end of system dictionnary */
	.word DTOP	/* CP end of RAM dictionary RAM */
	.word LASTN	/*LAST word in dictionary */
	.word 0,0			/*reserved */
ULAST:

/*****************************************************
* default isr handler called on unexpected interrupt
*****************************************************/
    .section  .text,"ax",%progbits
  .type default_handler, %function
  .p2align 2 
  .global default_handler
default_handler:
	ldr r7,exception_msg 
	ldrb r0,[r7],#1 
1:	_PUSH 
	ldrb r5,[r7],#1
	bl EMIT 
	subs r0,r0,#1 
	bne 1b 	
	b REBOOT   
  .size  default_handler, .-default_handler
exception_msg:
	.word .+4 
	.byte 18
	.ascii "\n\rexeption reboot!"
	.p2align 2
REBOOT:
	ldr r0,scb_adr 
	ldr r1,[r0,#SCB_AIRCR]
	orr r1,#(1<<2)
	movt r1,#SCB_VECTKEY
	str r1,[r0,#SCB_AIRCR]
	b . 
	.p2align 2
scb_adr:
	.word SCB_BASE_ADR 

/*********************************
	system milliseconds counter
*********************************/	
  .type systick_handler, %function
  .p2align 2 
  .global systick_handler
systick_handler:
  mov r3,#UPP&0xffff
  movt r3,#UPP>>16  	
  ldr r0,[r3,#TICKS_OFS]  
  add r0,#1
  str r0,[r3,#TICKS_OFS]
  ldr r0,[r3,#TIMER_OFS]
  cbz r0, systick_exit
  sub r0,#1
  str r0,[r3,#TIMER_OFS]
systick_exit:
  bx lr


/**************************************
  reset_handler execute at MCU reset
***************************************/
//    .section  .text.reset_handler
  .type  reset_handler, %function
  .p2align 2 
  .global reset_handler
reset_handler:
	bl	remap 
	bl	init_devices	 	/* RCC, GPIOs, USART */
	bl	unlock			/* unlock flash memory */
	bl	COLD

  .type init_devices, %function
  .p2align 2 
init_devices:
/* init clock to HSE 72 Mhz */
/* set 2 wait states in FLASH_ACR_LATENCY */
	mov r0,#FLASH_BASE_ADR&0xffff
	movt r0,#FLASH_BASE_ADR>>16 
  mov r2,#0x12
  str r2,[r0,#FLASH_ACR]
/* configure clock for HSE, 8 Mhz crystal */
/* enable HSE in RCC_CR */
  mov r0,#RCC_BASE_ADR&0xFFFF
  movt r0,#RCC_BASE_ADR>>16
  ldr r1,[r0,#RCC_CR]
  orr r1,r1,#(1<<16) /* HSEON bit */
  str r1,[r0,#RCC_CR] /* enable HSE */
/* wait HSERDY loop */
wait_hserdy:
  ldr r1,[r0,#RCC_CR]
  tst r1,#(1<<17)
  beq wait_hserdy

/************************************************* 
   configure PLL mul factor and source 
   SYSCLOCK=72 Mhz
   select HSE as  PLL source clock
   multiply frequency by 9 
   APB1 clock is limited to 36 Mhz so divide by 2 
****************************************************/
  mov r1,#(4<<8) /* PLLMUL=7|PLLSCR=HSE|PPRE1=HCLK/2| */
  movt r1,#(7<<2)|1
  str r1,[r0,#RCC_CFGR]
  /* enable PLL */
  ldr r1,[r0,#RCC_CR]
  orr r1, #1<<24 
  str r1,[r0,#RCC_CR]
/* wait for PLLRDY */
wait_pllrdy:
  ldr r1,[r0,#RCC_CR]
  tst r1,#(1<<25)
  beq wait_pllrdy 
/* select PLL as sysclock */
  ldr r1,[r0,#RCC_CFGR]
  mov r2,#0xfffc
  movt r2,#0xffff
  and r1,r1,r2 
  mov r2,#2
  orr r1,r1,r2
  str r1,[r0,#RCC_CFGR] /* PLL selected as sysclock */
/* wait for SWS==2 */
wait_sws:
  ldr r1,[r0,#RCC_CFGR]
  tst r1,#(2<<2)
  beq wait_sws
/* now sysclock is 72 Mhz */

/* enable peripheral clock for GPIOA, GPIOC and USART1 */
  mov r0,#RCC_BASE_ADR&0xFFFF
  movt r0,#RCC_BASE_ADR>>16
  mov	r1, #(1<<2)|(1<<4)|(1<<14)		/* GPIOAEN|GPIOCEN|USART1EN */
  str	r1, [r0, #RCC_APB2ENR]

/* configure GPIOC:13 as output for user LED */
  mov r0,#GPIOC_BASE_ADR&0xffff
  movt r0,#GPIOC_BASE_ADR>>16
  ldr r1,[r0,#GPIO_CRH]
  mvn r2,#(15<<20)
  and r1,r1,r2
  mov r2,#(6<<20)
  orr r1,r1,r2
  str r1,[r0,#GPIO_CRH]

/* configure USART1 */
/* set GPIOA PIN 9, uart TX  */
  mov r0,#GPIOA_BASE_ADR&0XFFFF
  movt r0,#GPIOA_BASE_ADR>>16	
  ldr r1,[r0,#GPIO_CRH]
  mvn r2,#(15<<4)
  and r1,r1,r2
  mov r2,#(0xA<<4)
  orr r1,r1,r2 
  str r1,[r0,#GPIO_CRH]

  mov r0,#UART&0xFFFF
  movt r0,#UART>>16	
/* BAUD rate */
  mov r1,#(39<<4)+1  /* (72Mhz/16)/115200=39,0625, quotient=39, reste=0,0625*16=1 */
  str r1,[r0,#USART_BRR]
  mov r1,#(3<<2)+(1<<13)
  str r1,[r0,#USART_CR1] /*enable usart*/
/* configure systicks for 1msec ticks */
  mov r0,#STK_BASE_ADR&0xFFFF
  movt r0,#STK_BASE_ADR>>16	
  mov r1,#9000 /* reload value for 1msec */
  str r1,[r0,#STK_LOAD]
  mov r1,#3
  str r1,[r0,STK_CTL]
  bx lr  

/* copy system to RAM */ 
//	.type remap, %function 
remap:
	ldr r0,remap_dest  
	ldr r1,remap_src 
	mov r2,#ULAST-UZERO 
1:	ldr r3,[r1],#4 
	str r3,[r0],#4 
	subs R2,#4 
	bne 1b
// zero end of RAM 
	mov r2,#0x5000
	movt r2,#0x2000
	eor r3,r3,r3 
2:  str r3,[r0],#4
	cmp r0,r2 
	blt 2b 
	bx lr  
	.p2align 2
remap_src:
	.word UZERO 
remap_dest:
	.word RAM_ADR  

/********************
* Version control
*******************/
.equ VER ,	0x01	/*major release version */
.equ EXT ,	0x00	/*minor extension */

/* Constants */

.equ COMPO ,	0x040	/*lexicon compile only */ 
.equ IMEDD ,	0x080	/*lexicon immediate bit */
.equ MASKK ,	0x0FFFFFF1F	/*lexicon bit mask, allowed for Chineze character */

.equ CELLL ,	4	/*size of a cell */
.equ BASEE ,	10	/*default radix */
.equ VOCSS ,	8	/*depth of vocabulary stack */

.equ BKSPP ,	8	/*backspace */
.equ LF ,	10	  /*line feed */
.equ CRR ,	13	/*carriage return */
.equ ERR ,	27	/*error escape */
.equ TIC ,	39	/*tick */

/********************************************************
* RAM memory mapping
* 	0x20000000	RAM base address
*	0x20000000  system variables	
* 	0x20000100	Forth dictionary
* 	0x2000????	top of dictionary, HERE
* 	0x2000????	WORD buffer, HERE+16
*   0x20004E00  end of user space
* 	0x20004E80	top of data stack  R2
* 	0x20004E80	TIB terminal input buffer
* 	0x20004F80	top of return stack  R1
* 	0x20005000	top of hardware stack for interrupts R14
********************************************************/


/******************************************************
*  COLD start moves the following to USER variables.
*  MUST BE IN SAME ORDER AS USER VARIABLES.
******************************************************/
	.p2align 10


 

/***********************************
//  Start of eForth system 
***********************************/

	.p2align 2 

// PUSH R5, to be used in colon definition 
TPUSH:
	_PUSH
	_NEXT

// POP R5, to be used in colon defintion  
TPOP:
	_POP 
	_NEXT


// hi level word enter 
NEST: 
	STMFD	R2!,{R0}
	ADD R0,R4,#3
// inner interprer
INEXT: 
/*
	ADD R6,R3,#FTRACE_OFS
	LDR R6,[R6]
	CBZ r6, 1f
	_PUSH 
	LDR R5,[R0]
	SUB R5,#1
	B DBG_PRT
*/
1:	LDR R4,[R0],#4 
	BLX R4 
UNNEST:
	LDMFD R2!,{R0}
	B INEXT 

	.p2align 2 

// RANDOM ( n1 -- {0..n1-1} )
// return pseudo random number 
// REF: https://en.wikipedia.org/wiki/Xorshift

	.word 0
_RAND: .byte 6
	.ascii "RANDOM"
	.p2align 2 
RAND:
	_NEST 
	_ADR SEED 
	_ADR AT 
	_ADR DUPP 
	_DOLIT 13
	_ADR LSHIFT 
	_ADR XORR  
	_ADR DUPP 
	_DOLIT 17 
	_ADR RSHIFT 
	_ADR XORR 
	_ADR DUPP
	_DOLIT 5 
	_ADR LSHIFT 
	_ADR XORR  
	_ADR DUPP 
	_ADR SEED 
	_ADR STORE 
	_ADR ABSS
	_ADR SWAP 
	_ADR MODD 
	_UNNEST 


// PAUSE ( u -- ) 
// suspend execution for u milliseconds
	.word _RAND
_PAUSE: .byte 5
	.ascii "PAUSE"
	.p2align 2
PAUSE:
	_NEST 
	_ADR TIMER 
	_ADR STORE 
PAUSE_LOOP:
	_ADR TIMER 
	_ADR AT 
	_QBRAN PAUSE_EXIT 
	_BRAN PAUSE_LOOP 
PAUSE_EXIT: 		
	_UNNEST 

//  ULED ( T|F -- )
// control user LED, -1 ON, 0 OFF  
	.word _PAUSE
_ULED: .byte 4
	.ascii "ULED"
	.p2align 2
	.type ULED, %function 
ULED:
	mov r6,#(1<<LED_PIN)
	mov r4,#LED_GPIO&0xffff
	movt r4,#LED_GPIO>>16
	movs r5,r5 
	_POP
	beq ULED_OFF 
	str r6,[r4,#GPIO_BRR]
	_NEXT 
ULED_OFF:
	str r6,[r4,#GPIO_BSRR]
	_NEXT    

//    ?RX	 ( -- c T | F )
// 	Return input character and true, or a false if no input.
	.word	_ULED
_QRX:	.byte   4
	.ascii "?KEY"
	.p2align 2 
QKEY:
QRX: 
	_PUSH
	mov r4,#UART&0xFFFF
	movt r4,#UART>>16
	ldrh	r6, [r4, #USART_SR]
	ands	r6, #0x20		//  RXE
	BEQ	QRX1
 	LDR	R5, [R4, #USART_DR]
	_PUSH
    IT NE 
	MVNNE	R5,#0
QRX1:
	IT EQ 
    MOVEQ	R5,#0
	_NEXT 

//    TX!	 ( c -- )
// 	Send character c to the output device.

	.word	_QRX
_TXSTO:	.byte 4
	.ascii "EMIT"
	.p2align 2 	
TXSTO:
EMIT:
TECHO:
	mov r4,#UART&0xFFFF
	movt r4,#UART>>16
TX1:
	ldrh	r6, [r4, #USART_SR]	
	ands	r6, #0x80		//  TXE bit 
	beq	TX1
	strh	r5, [r4, #USART_DR]	
	_POP
	_NEXT 
	
/***************
//  The kernel
***************/

//    NOP	( -- )
// 	do nothing.

	.word	_TXSTO
_NOP:	.byte   3
	.ascii "NOP"
	.p2align 2 	
NOP:
	_NEXT 
 

//    doLIT	( -- w )
// 	Push an inline literal.

// 	.word	_NOP
// _LIT	.byte   COMPO+5
// 	.ascii "doLIT"
 	.p2align 2 	
DOLIT:
	_PUSH				//  store R5 on data stack
	LDR	R5,[R0],#4		//  get literal at word boundary
	_NEXT 

//    EXECUTE	( ca -- )
// 	Execute the word at ca.

	.word	_NOP
_EXECU:	.byte   7
	.ascii "EXECUTE"
	.p2align 2 	
EXECU: 
	ORR	R4,R5,#1 
	_POP
	BX R4 
	_NEXT 

//    next	( -- ) counter on R:
// 	Run time code for the single index loop.
// 	: next ( -- ) \ hilevel model
// 	 r> r> dup if 1 - >r @ >r exit then drop cell+ >r // 

// 	.word	_EXECU
// _DONXT	.byte   COMPO+4
// 	.ascii "next"
// 	.p2align 2 	
DONXT:
	LDR	R4,[R2]   // ( -- u )  
	CBNZ R4,NEXT1 
	/* loop done */
	ADD	R2,R2,#4 // drop counter 
	ADD	R0,R0,#4 // skip after loop address 
	_NEXT
NEXT1:
	/* decrement loop counter */
	SUB	R4,R4,#1
	STR	R4,[R2]
	LDR	R0,[R0]	// gbegining of loop 
	_NEXT 

//    ?branch	( f -- )
// 	Branch if flag is zero.

// 	.word	_DONXT
// _QBRAN	.byte   COMPO+7
// 	.ascii "?branch"
// 	.p2align 2 	
QBRAN:
	MOVS	R5,R5
	_POP
	BNE	QBRAN1
	LDR	R0,[R0]
	_NEXT
QBRAN1:
 	ADD	R0,R0,#4
	_NEXT

//    branch	( -- )
// 	Branch to an inline address.

// 	.word	_QBRAN
// _BRAN	.byte   COMPO+6
// 	.ascii "branch"
// 	.p2align 2 	
BRAN:
	LDR	R0,[R0]
	_NEXT

//    EXIT	(  -- )
// 	Exit the currently executing command.

	.word	_EXECU
_EXIT:	.byte   4
	.ascii "EXIT"
	.p2align 2 	
EXIT:
	_UNNEST

//    !	   ( w a -- )
// 	Pop the data stack to memory.

	.word	_EXIT
_STORE:	.byte   1
	.ascii "!"
	.p2align 2 	
STORE:
	LDR	R4,[R1],#4
	STR	R4,[R5]
	_POP
	_NEXT 

//    @	   ( a -- w )
// 	Push memory location to the data stack.

	.word	_STORE
_AT:	.byte   1
	.ascii "@"
	.p2align 2 	
AT:
	LDR	R5,[R5]
	_NEXT 

//    C!	  ( c b -- )
// 	Pop the data stack to byte memory.

	.word	_AT
_CSTOR:	.byte   2
	.ascii "C!"
	.p2align 2 	
CSTOR:
	LDR	R4,[R1],#4
	STRB R4,[R5]
	_POP
	_NEXT

//    C@	  ( b -- c )
// 	Push byte memory location to the data stack.

	.word	_CSTOR
_CAT:	.byte   2
	.ascii "C@"
	.p2align 2 	
CAT:
	LDRB	R5,[R5]
	_NEXT 

//    R>	  ( -- w )
// 	Pop the return stack to the data stack.

	.word	_CAT
_RFROM:	.byte   2
	.ascii "R>"
	.p2align 2 	
RFROM:
	_PUSH
	LDR	R5,[R2],#4
	_NEXT 

//    R@	  ( -- w )
// 	Copy top of return stack to the data stack.

	.word	_RFROM
_RAT:	.byte   2
	.ascii "R@"
	.p2align 2 	
RAT:
	_PUSH
	LDR	R5,[R2]
	_NEXT 

//    >R	  ( w -- )
// 	Push the data stack to the return stack.

	.word	_RAT
_TOR:	.byte   COMPO+2
	.ascii ">R"
	.p2align 2 	
TOR:
	STR	R5,[R2,#-4]!
	_POP
	_NEXT

//	RP! ( u -- )
// initialize RPP with u 
	.word _TOR 
_RPSTOR: .byte 3 
	.ascii "RP!" 
	.p2align 2 
RPSTOR:
	MOV R2,R5 
	_POP  
	_NEXT 


//	SP! ( u -- )
// initialize SPP with u 
	.word _RPSTOR  
_SPSTOR: .byte 3 
	.ascii "SP!" 
	.p2align 2 
SPSTOR:
	MOV R1,R5 
	EOR R5,R5,R5 
	_NEXT 

//    SP@	 ( -- a )
// 	Push the current data stack pointer.

	.word	_SPSTOR
_SPAT:	.byte   3
	.ascii "SP@"
	.p2align 2 	
SPAT:
	_PUSH
	MOV	R5,R1
	_NEXT

//    DROP	( w -- )
// 	Discard top stack item.

	.word	_SPAT
_DROP:	.byte   4
	.ascii "DROP"
	.p2align 2 	
DROP:
	_POP
	_NEXT 

//    DUP	 ( w -- w w )
// 	Duplicate the top stack item.

	.word	_DROP
_DUPP:	.byte   3
	.ascii "DUP"
	.p2align 2 	
DUPP:
	_PUSH
	_NEXT 

//    SWAP	( w1 w2 -- w2 w1 )
// 	Exchange top two stack items.

	.word	_DUPP
_SWAP:	.byte   4
	.ascii "SWAP"
	.p2align 2 	
SWAP:
	LDR	R4,[R1]
	STR	R5,[R1]
	MOV	R5,R4
	_NEXT 

//    OVER	( w1 w2 -- w1 w2 w1 )
// 	Copy second stack item to top.

	.word	_SWAP
_OVER:	.byte   4
	.ascii "OVER"
	.p2align 2 	
OVER:
	_PUSH
	LDR	R5,[R1,#4]
	_NEXT 

//    0<	  ( n -- t )
// 	Return true if n is negative.

	.word	_OVER
_ZLESS:	.byte   2
	.ascii "0<"
	.p2align 2 	
ZLESS:
//	MOV	R4,#0
//	ADD	R5,R4,R5,ASR #32
	ASR R5,#31
	_NEXT 

//    AND	 ( w w -- w )
// 	Bitwise AND.

	.word	_ZLESS
_ANDD:	.byte   3
	.ascii "AND"
	.p2align 2 	
ANDD:
	LDR	R4,[R1],#4
	AND	R5,R5,R4
	_NEXT 

//    OR	  ( w w -- w )
// 	Bitwise inclusive OR.

	.word	_ANDD
_ORR:	.byte   2
	.ascii "OR"
	.p2align 2 	
ORR:
	LDR	R4,[R1],#4
	ORR	R5,R5,R4
	_NEXT 

//    XOR	 ( w w -- w )
// 	Bitwise exclusive OR.

	.word	_ORR
_XORR:	.byte   3
	.ascii "XOR"
	.p2align 2 	
XORR:
	LDR	R4,[R1],#4
	EOR	R5,R5,R4
	_NEXT 

//    UM+	 ( w w -- w cy )
// 	Add two numbers, return the sum and carry flag.

	.word	_XORR
_UPLUS:	.byte   3
	.ascii "UM+"
	.p2align 2 	
UPLUS:
	LDR	R4,[R1]
	ADDS	R4,R4,R5
	MOV	R5,#0
	ADC	R5,R5,#0
	STR	R4,[R1]
	_NEXT 

//    RSHIFT	 ( w # -- w )
// 	arithmetic Right shift # bits.

	.word	_UPLUS
_RSHIFT:	.byte   6
	.ascii "RSHIFT"
	.p2align 2 	
RSHIFT:
	LDR	R4,[R1],#4
	MOV	R5,R4,ASR R5
	_NEXT 

//    LSHIFT	 ( w # -- w )
// 	Right shift # bits.

	.word	_RSHIFT
_LSHIFT:	.byte   6
	.ascii "LSHIFT"
	.p2align 2 	
LSHIFT:
	LDR	R4,[R1],#4
	MOV	R5,R4,LSL R5
	_NEXT

//    +	 ( w w -- w )
// 	Add.

	.word	_LSHIFT
_PLUS:	.byte   1
	.ascii "+"
	.p2align 2 	
PLUS:
	LDR	R4,[R1],#4
	ADD	R5,R5,R4
	_NEXT 

//    -	 ( w w -- w )
// 	Subtract.

	.word	_PLUS
_SUBB:	.byte   1
	.ascii "-"
	.p2align 2 	
SUBB:
	LDR	R4,[R1],#4
	RSB	R5,R5,R4
	_NEXT 

//    *	 ( w w -- w )
// 	Multiply.

	.word	_SUBB
_STAR:	.byte   1
	.ascii "*"
	.p2align 2 	
STAR:
	LDR	R4,[R1],#4
	MUL	R5,R4,R5
	_NEXT 

//    UM*	 ( w w -- ud )
// 	Unsigned multiply.

	.word	_STAR
_UMSTA:	.byte   3
	.ascii "UM*"
	.p2align 2 	
UMSTA:
	LDR	R4,[R1]
	UMULL	R6,R7,R5,R4
	STR	R6,[R1]
	MOV	R5,R7
	_NEXT 

//    M*	 ( w w -- d )
// 	signed multiply.

	.word	_UMSTA
_MSTAR:	.byte   2
	.ascii "M*"
	.p2align 2 	
MSTAR:
	LDR	R4,[R1]
	SMULL	R6,R7,R5,R4
	STR	R6,[R1]
	MOV	R5,R7
	_NEXT 

//    1+	 ( w -- w+1 )
// 	Add 1.

	.word	_MSTAR
_ONEP:	.byte   2
	.ascii "1+"
	.p2align 2 	
ONEP:
	ADD	R5,R5,#1
	_NEXT 

//    1-	 ( w -- w-1 )
// 	Subtract 1.

	.word	_ONEP
_ONEM:	.byte   2
	.ascii "1-"
	.p2align 2 	
ONEM:
	SUB	R5,R5,#1
	_NEXT 

//    2+	 ( w -- w+2 )
// 	Add 1.

	.word	_ONEM
_TWOP:	.byte   2
	.ascii "2+"
	.p2align 2 	
TWOP:
	ADD	R5,R5,#2
	_NEXT

//    2-	 ( w -- w-2 )
// 	Subtract 2.

	.word	_TWOP
_TWOM:	.byte   2
	.ascii "2-"
	.p2align 2 	
TWOM:
	SUB	R5,R5,#2
	_NEXT

//    CELL+	( w -- w+4 )
// 	Add CELLL.

	.word	_TWOM
_CELLP:	.byte   5
	.ascii "CELL+"
	.p2align 2 	
CELLP:
	ADD	R5,R5,#CELLL
	_NEXT

//    CELL-	( w -- w-4 )
// 	Subtract CELLL.

	.word	_CELLP
_CELLM:	.byte   5
	.ascii "CELL-"
	.p2align 2 	
CELLM:
	SUB	R5,R5,#CELLL
	_NEXT
 
//    BL	( -- 32 )
// 	Blank (ASCII space).

	.word	_CELLM
_BLANK:	.byte   2
	.ascii "BL"
	.p2align 2 	
BLANK:
	_PUSH
	MOV	R5,#32
	_NEXT 

//    CELLS	( w -- w*4 )
// 	Multiply 4.

	.word	_BLANK
_CELLS:	.byte   5
	.ascii "CELLS"
	.p2align 2 	
CELLS:
	MOV	R5,R5,LSL#2
	_NEXT

//    CELL/	( w -- w/4 )
// 	Divide by 4.

	.word	_CELLS
_CELLSL:	.byte   5
	.ascii "CELL/"
	.p2align 2 	
CELLSL:
	MOV	R5,R5,ASR#2
	_NEXT

//    2*	( w -- w*2 )
// 	Multiply 2.

	.word	_CELLSL
_TWOST:	.byte   2
	.ascii "2*"
	.p2align 2 	
TWOST:
	MOV	R5,R5,LSL#1
	_NEXT

//    2/	( w -- w/2 )
// 	Divide by 2.

	.word	_TWOST
_TWOSL:	.byte   2
	.ascii "2/"
	.p2align 2 	
TWOSL:
	MOV	R5,R5,ASR#1
	_NEXT

//    ?DUP	( w -- w w | 0 )
// 	Conditional duplicate.

	.word	_TWOSL
_QDUP:	.byte   4
	.ascii "?DUP"
	.p2align 2 	
QDUP:
	MOVS	R4,R5
	IT NE 
    STRNE	R5,[R1,#-4]!
	_NEXT

//    ROT	( w1 w2 w3 -- w2 w3 w1 )
// 	Rotate top 3 items.

	.word	_QDUP
_ROT:	.byte   3
	.ascii "ROT"
	.p2align 2 	
ROT:
	LDR	R4,[R1]  // r4=w2 
	STR	R5,[R1]  // w3 replace w2 
	LDR	R5,[R1,#4] // w1 replace w3 
	STR	R4,[R1,#4] // w2 rpelace w1 
	_NEXT

//    2DROP	( w1 w2 -- )
// 	Drop top 2 items.

	.word	_ROT
_DDROP:	.byte   5
	.ascii "2DROP"
	.p2align 2 	
DDROP:
	_POP
	_POP
	_NEXT 

//    2DUP	( w1 w2 -- w1 w2 w1 w2 )
// 	Duplicate top 2 items.

	.word	_DDROP
_DDUP:	.byte   4
	.ascii "2DUP"
	.p2align 2 	
DDUP:
	LDR	R4,[R1] // r4=w1
	STR	R5,[R1,#-4]! // push w2  
	STR	R4,[R1,#-4]! // push w1 
	_NEXT

//    D+	( d1 d2 -- d3 )
// 	Add top 2 double numbers.

	.word	_DDUP
_DPLUS:	.byte   2
	.ascii "D+"
	.p2align 2 	
DPLUS:
	LDR	R4,[R1],#4
	LDR	R6,[R1],#4
	LDR	R7,[R1]
	ADDS	R4,R4,R7
	STR	R4,[R1]
	ADC	R5,R5,R6
	_NEXT

//    NOT	 ( w -- !w )
// 	1"s complement.

	.word	_DPLUS
_INVER:	.byte   3
	.ascii "NOT"
	.p2align 2 	
INVER:
	MVN	R5,R5
	_NEXT

//    NEGATE	( w -- -w )
// 	2's complement.

	.word	_INVER
_NEGAT:	.byte   6
	.ascii "NEGATE"
	.p2align 2 	
NEGAT:
	RSB	R5,R5,#0
	_NEXT

//    ABS	 ( w -- |w| )
// 	Absolute.

	.word	_NEGAT
_ABSS:	.byte   3
	.ascii "ABS"
	.p2align 2 	
ABSS:
	TST	R5,#0x80000000
	IT NE
    RSBNE   R5,R5,#0
	_NEXT

//  0= ( w -- f )
// TOS==0?

	.word _ABSS
_ZEQUAL: .byte 2
	.ascii "0="
	.p2align 2
ZEQUAL:
	cbnz r5,1f
	mov r5,#-1
	_NEXT 
1:  eor r5,r5,r5  
	_NEXT 	

//    =	 ( w w -- t )
// 	Equal?

	.word	_ZEQUAL
_EQUAL:	.byte   1
	.ascii "="
	.p2align 2 	
EQUAL:
	LDR	R4,[R1],#4
	CMP	R5,R4
	ITE EQ 
    MVNEQ	R5,#0
	MOVNE	R5,#0
	_NEXT

//    U<	 ( w w -- t )
// 	Unsigned equal?

	.word	_EQUAL
_ULESS:	.byte   2
	.ascii "U<"
	.p2align 2 	
ULESS:
	LDR	R4,[R1],#4
	CMP	R4,R5
	ITE CC 
	MVNCC	R5,#0
	MOVCS	R5,#0
	_NEXT

//    <	( w w -- t )
// 	Less?

	.word	_ULESS
_LESS:	.byte   1
	.ascii "<"
	.p2align 2 	
LESS:
	LDR	R4,[R1],#4
	CMP	R4,R5
    ITE LT
	MVNLT	R5,#0
	MOVGE	R5,#0
	_NEXT 

//    >	( w w -- t )
// 	greater?

	.word	_LESS
_GREAT:	.byte   1
	.ascii ">"
	.p2align 2 	
GREAT:
	LDR	R4,[R1],#4
	CMP	R4,R5
	ITE GT
    MVNGT	R5,#0
	MOVLE	R5,#0
	_NEXT

//    MAX	 ( w w -- max )
// 	Leave maximum.

	.word	_GREAT
_MAX:	.byte   3
	.ascii "MAX"
	.p2align 2 	
MAX:
	LDR	R4,[R1],#4
	CMP	R4,R5
	IT GT 
	MOVGT	R5,R4
	_NEXT 

//    MIN	 ( w w -- min )
// 	Leave minimum.

	.word	_MAX
_MIN:	.byte   3
	.ascii "MIN"
	.p2align 2 	
MIN:
	LDR	R4,[R1],#4
	CMP	R4,R5
	IT LT
	MOVLT	R5,R4
	_NEXT

//    +!	 ( w a -- )
// 	Add to memory.

	.word	_MIN
_PSTOR:	.byte   2
	.ascii "+!"
	.p2align 2 	
PSTOR:
	LDR	R4,[R1],#4
	LDR	R6,[R5]
	ADD	R6,R6,R4
	STR	R6,[R5]
	_POP
	_NEXT

//    2!	 ( d a -- )
// 	Store double number.

	.word	_PSTOR
_DSTOR:	.byte   2
	.ascii "2!"
	.p2align 2 	
DSTOR:
	LDR	R4,[R1],#4
	LDR	R6,[R1],#4
	STR	R4,[R5],#4
	STR	R6,[R5]
	_POP
	_NEXT

//    2@	 ( a -- d )
// 	Fetch double number.

	.word	_DSTOR
_DAT:	.byte   2
	.ascii "2@"
	.p2align 2 	
DAT:
	LDR	R4,[R5,#4]
	STR	R4,[R1,#-4]!
	LDR	R5,[R5]
	_NEXT

//    COUNT	( b -- b+1 c )
// 	Fetch length of string.

	.word	_DAT
_COUNT:	.byte   5
	.ascii "COUNT"
	.p2align 2 	
COUNT:
	LDRB	R4,[R5],#1
	_PUSH
	MOV	R5,R4
	_NEXT

//    DNEGATE	( d -- -d )
// 	Negate double number.

	.word	_COUNT
_DNEGA:	.byte   7
	.ascii "DNEGATE"
	.p2align 2 	
DNEGA:
	LDR	R4,[R1]
	SUB	R6,R6,R6
	SUBS R4,R6,R4
	SBC	R5,R6,R5
	STR	R4,[R1]
	_NEXT

// **************************************************************************
//  System and user variables

//    doVAR	( -- a )
// 	Run time routine for VARIABLE and CREATE.

// 	.word	_DNEGA
// _DOVAR	.byte  COMPO+5
// 	.ascii "doVAR"
// 	.p2align 2 	
DOVAR:
	_PUSH
	LDR R5,[R0],#4 
	_NEXT 

//    doCON	( -- a ) 
// 	Run time routine for CONSTANT.

// 	.word	_DOVAR
// _DOCON	.byte  COMPO+5
// 	.ascii "doCON"
// 	.p2align 2 	
DOCON:
	_PUSH
	LDR R5,[R0],#4

/***********************
  system variables 
***********************/

 // SEED ( -- a)
 // return PRNG seed address 

	.word _DNEGA
_SEED: .byte 4
	.ascii "SEED"
	.p2align 2
SEED:
	_PUSH 
	ADD R5,R3,#SEED_OFS
	_NEXT 	

//  MSEC ( -- a)
// return address of milliseconds counter
  .word _SEED 
_MSEC: .byte 4
  .ascii "MSEC"
  .p2align 2 
MSEC:
  _PUSH
  ADD R5,R3,#TICKS_OFS
  _NEXT 

// TIMER ( -- a )
// count down timer 
  .word _MSEC
_TIMER:  .byte 5
  .ascii "TIMER"
  .p2align 2 
TIMER:
  _PUSH 
  ADD R5,R3,#TIMER_OFS
  _NEXT

// >RAM ( -- a )
// compilation target 
// -1 compile to RAM, 0 compile to FLASH 
	.word _TIMER
_TORAM: .byte 4 
	.ascii ">RAM" 
	.p2align 2 
TORAM:
	_PUSH 
	ADD R5,R3,#TORAM_OFS
	_NEXT 

//    'BOOT	 ( -- a )
// 	Application.

	.word	_TORAM
_TBOOT:	.byte   5
	.ascii "'BOOT"
	.p2align 2 	
TBOOT:
	_PUSH
	ADD	R5,R3,#BOOT_OFS 
	_NEXT
	
//    BASE	( -- a )
// 	Storage of the radix base for numeric I/O.

	.word	_TBOOT
_BASE:	.byte   4
	.ascii "BASE"
	.p2align 2 	
BASE:
	_PUSH
	ADD	R5,R3,#BASE_OFS
	_NEXT

//    tmp	 ( -- a )
// 	A temporary storage location used in parse and find.

// 	.word	_BASE
// _TEMP	.byte   COMPO+3
// 	.ascii "tmp"
// 	.p2align 2 	
TEMP:
	_PUSH
	ADD	R5,R3,#TMP_OFS
	_NEXT

//    SPAN	( -- a )
// 	Hold character count received by EXPECT.

	.word	_BASE
_SPAN:	.byte   4
	.ascii "SPAN"
	.p2align 2 	
SPAN:
	_PUSH
	ADD	R5,R3,#SPAN_OFS
	_NEXT

//    >IN	 ( -- a )
// 	Hold the character pointer while parsing input stream.

	.word	_SPAN
_INN:	.byte   3
	.ascii ">IN"
	.p2align 2 	
INN:
	_PUSH
	ADD	R5,R3,#TOIN_OFS
	_NEXT

//    #TIB	( -- a )
// 	Hold the current count and address of the terminal input buffer.

	.word	_INN
_NTIB:	.byte   4
	.ascii "#TIB"
	.p2align 2 	
NTIB:
	_PUSH
	ADD	R5,R3,#NTIB_OFS
	_NEXT

//    'EVAL	( -- a )
// 	Execution vector of EVAL.

	.word	_NTIB
_TEVAL:	.byte   5
	.ascii "'EVAL"
	.p2align 2 	
TEVAL:
	_PUSH
	ADD	R5,R3,#EVAL_OFS
	_NEXT

//    HLD	 ( -- a )
// 	Hold a pointer in building a numeric output string.

	.word	_TEVAL
_HLD:	.byte   3
	.ascii "HLD"
	.p2align 2 	
HLD:
	_PUSH
	ADD	R5,R3,#HLD_OFS
	_NEXT

//    CONTEXT	( -- a )
// 	A area to specify vocabulary search order.

	.word	_HLD
_CNTXT:	.byte   7
	.ascii "CONTEXT"
	.p2align 2 	
CNTXT:
CRRNT:
	_PUSH
	ADD	R5,R3,#CTXT_OFS
	_NEXT

//    CP	( -- a )
// 	Point to top name in RAM vocabulary.

	.word	_CNTXT
_CP:	.byte   2
	.ascii "CP"
	.p2align 2 	
CPP:
	_PUSH
	ADD	R5,R3,#RAM_CTOP_OFS
	_NEXT

//   FCP ( -- a )
//  Point ot top of Forth system dictionary
	.word _CP
_FCP: .byte 3            
	.ascii "FCP"
	.p2align 2 
FCP: 
	_PUSH 
	ADD R5,R3,#FORTH_CTOP_OFS 
	_NEXT 

//    LAST	( -- a )
// 	Point to the last name in the name dictionary.

	.word	_FCP
_LAST:	.byte   4
	.ascii "LAST"
	.p2align 2 	
LAST:
	_PUSH
	ADD	R5,R3,#LASTN_OFS
	_NEXT

//	FTRACE ( -- a )
// return trace flag address 
	.word _LAST  
_FTRACE: .byte 6
	.ascii "FTRACE"
	.p2align 2 
FTRACE:	
	_PUSH 
	ADD R5,R3,#FTRACE_OFS 
	_NEXT 

/***********************
	system constants 
***********************/

//	USER_BEGIN ( -- a )
//  where user area begin in RAM
	.word _FTRACE
_USER_BGN: .byte 10
	.ascii "USER_BEGIN"
	.p2align 2
USER_BEGIN:
	_PUSH 
	ldr r5,USR_BGN_ADR 
	_NEXT 
USR_BGN_ADR:
.word  DTOP 

//  USER_END ( -- a )
//  where user area end in RAM 
	.word _USER_BGN
_USER_END: .byte 8 
	.ascii "USER_END" 
	.p2align 2 
USER_END:
	_PUSH 
	ldr r5,USER_END_ADR 
	_NEXT 
USER_END_ADR:
	.word DEND 

//  IMAGE0 ( -- a )
//  where user image is saved in FLASH
	.word _USER_END
_IMAGE0: .byte 6
	.ascii "IMAGE0"
	.p2align 2 
IMAGE0:
	_PUSH
	ldr r5,USR_IMG_ADR   
	_NEXT 
USR_IMG_ADR:
	.word USER_SPACE 

// IMG_SIGN ( -- a )
// image signature 
	.word _IMAGE0 
_IMG_SIGN: .byte 8
	.ascii "IMG_SIGN"
	.p2align 2
IMG_SIGN: 
	_PUSH 
	ADD r5,r3,#IMG_SIGN_OFS 
	LDR R5,[R5]
	_NEXT 

/* *********************
  Common functions
***********************/

//    WITHIN	( u ul uh -- t )
// 	Return true if u is within the range of ul and uh.

	.word	_IMG_SIGN
_WITHI:	.byte   6
	.ascii "WITHIN"
	.p2align 2 	
WITHI:
	_NEST
	_ADR	OVER
	_ADR	SUBB
	_ADR	TOR
	_ADR	SUBB
	_ADR	RFROM
	_ADR	ULESS
	_UNNEST

//  Divide

//    UM/MOD	( udl udh u -- ur uq )
// 	Unsigned divide of a double by a single. Return mod and quotient.

	.word	_WITHI
_UMMOD:	.byte   6
	.ascii "UM/MOD"
	.p2align 2 	
UMMOD:
	MOV	R7,#1
	LDR	R4,[R1],#4
	LDR	R6,[R1]
UMMOD0:
	ADDS	R6,R6,R6
	ADCS	R4,R4,R4
	BCC	UMMOD1
	SUB	R4,R4,R5
	ADD	R6,R6,#1
	B UMMOD2
UMMOD1:
	SUBS	R4,R4,R5 
	IT CS 
	ADDCS	R6,R6,#1
	BCS	UMMOD2
	ADD	R4,R4,R5
UMMOD2:
	ADDS	R7,R7,R7
	BCC	UMMOD0
	MOV	R5,R6
	STR	R4,[R1]
	_NEXT

//    M/MOD	( d n -- r q )
// 	Signed floored divide of double by single. Return mod and quotient.

	.word	_UMMOD
_MSMOD:	.byte  5
	.ascii "M/MOD"
	.p2align 2 	
MSMOD:	
	_NEST
	_ADR	DUPP
	_ADR	ZLESS
	_ADR	DUPP
	_ADR	TOR
	_QBRAN MMOD1
	_ADR	NEGAT
	_ADR	TOR
	_ADR	DNEGA
	_ADR	RFROM
MMOD1:
	_ADR	TOR
	_ADR	DUPP
	_ADR	ZLESS
	_QBRAN MMOD2
	_ADR	RAT
	_ADR	PLUS
MMOD2:
	_ADR	RFROM
	_ADR	UMMOD
	_ADR	RFROM
	_QBRAN	MMOD3
	_ADR	SWAP
	_ADR	NEGAT
	_ADR	SWAP
MMOD3:   
	_UNNEST

//    /MOD	( n n -- r q )
// 	Signed divide. Return mod and quotient.

	.word	_MSMOD
_SLMOD:	.byte   4
	.ascii "/MOD"
	.p2align 2 	
SLMOD:
	_NEST
	_ADR	OVER
	_ADR	ZLESS
	_ADR	SWAP
	_ADR	MSMOD
	_UNNEST

//    MOD	 ( n n -- r )
// 	Signed divide. Return mod only.

	.word	_SLMOD
_MODD:	.byte  3
	.ascii "MOD"
	.p2align 2 	
MODD:
	_NEST
	_ADR	SLMOD
	_ADR	DROP
	_UNNEST

//    /	   ( n n -- q )
// 	Signed divide. Return quotient only.

	.word	_MODD
_SLASH:	.byte  1
	.ascii "/"
	.p2align 2 	
SLASH:
	_NEST
	_ADR	SLMOD
	_ADR	SWAP
	_ADR	DROP
	_UNNEST

//    */MOD	( n1 n2 n3 -- r q )
// 	Multiply n1 and n2, then divide by n3. Return mod and quotient.

	.word	_SLASH
_SSMOD:	.byte  5
	.ascii "*/MOD"
	.p2align 2 	
SSMOD:
	_NEST
	_ADR	TOR
	_ADR	MSTAR
	_ADR	RFROM
	_ADR	MSMOD
	_UNNEST

//    */	  ( n1 n2 n3 -- q )
// 	Multiply n1 by n2, then divide by n3. Return quotient only.

	.word	_SSMOD
_STASL:	.byte  2
	.ascii "*/"
	.p2align 2 	
STASL:
	_NEST
	_ADR	SSMOD
	_ADR	SWAP
	_ADR	DROP
	_UNNEST

// **************************************************************************
//  Miscellaneous

//    ALIGNED	( b -- a )
// 	Align address to the cell boundary.

	.word	_STASL
_ALGND:	.byte   7
	.ascii "ALIGNED"
	.p2align 2 	
ALGND:
	ADD	R5,R5,#3
	MVN	R4,#3
	AND	R5,R5,R4
	_NEXT

//    >CHAR	( c -- c )
// 	Filter non-printing characters.

	.word	_ALGND
_TCHAR:	.byte  5
	.ascii ">CHAR"
	.p2align 2 	
TCHAR:
	_NEST
	_DOLIT  0x7F
	_ADR	ANDD
	_ADR	DUPP	// mask msb
	_ADR	BLANK
	_DOLIT 	127
	_ADR	WITHI	// check for printable
	_ADR	INVER
	_QBRAN	TCHA1
	_ADR	DROP
	_DOLIT 	'_'	// replace non-printables
TCHA1:
	  _UNNEST

//    DEPTH	( -- n )
// 	Return the depth of the data stack.

	.word	_TCHAR
_DEPTH:	.byte  5
	.ascii "DEPTH"
	.p2align 2 	
DEPTH:
	_PUSH
	MOVW	R5,#SPP&0xffff+4 
 	MOVT	R5,#SPP>>16 
	SUB	R5,R5,R1
	ASR	R5,R5,#2
	SUB	R5,R5,#1
	_NEXT

//    PICK	( ... +n -- ... w )
// 	Copy the nth stack item to tos.

	.word	_DEPTH
_PICK:	.byte  4
	.ascii "PICK"
	.p2align 2 	
PICK:
	_NEST
	_ADR	ONEP
	_ADR	CELLS
	_ADR	SPAT
	_ADR	PLUS
	_ADR	AT
	_UNNEST

// **************************************************************************
//  Memory access

//    HERE	( -- a )
// 	Return the top of the code dictionary.

	.word	_PICK
_HERE:	.byte  4
	.ascii "HERE"
	.p2align 2 	
HERE:
	_NEST
	_ADR	CPP
	_ADR	AT
	_UNNEST
	
//    PAD	 ( -- a )
// 	Return the address of a temporary buffer.

	.word	_HERE
_PAD:	.byte  3
	.ascii "PAD"
	.p2align 2 	
PAD:
	_NEST
	_ADR	HERE
	_DOLIT 80
	_ADR PLUS 
	_UNNEST

//    TIB	 ( -- a )
// 	Return the address of the terminal input buffer.

	.word	_PAD
_TIB:	.byte  3
	.ascii "TIB"
	.p2align 2 	
TIB:
	_PUSH
	ldr r5,[r3,#TIB_OFS]
	_NEXT

//    @EXECUTE	( a -- )
// 	Execute vector stored in address a.

	.word	_TIB
_ATEXE:	.byte   8
	.ascii "@EXECUTE"
	.p2align 2 	
ATEXE: 
	MOVS	R4,R5
	_POP
	LDR	R4,[R4]
	ORR	R4,R4,#1
    IT NE 
	BXNE	R4
	_NEXT

//    CMOVE	( b1 b2 u -- )
// 	Copy u bytes from b1 to b2.

	.word	_ATEXE
_CMOVE:	.byte   5
	.ascii "CMOVE"
	.p2align 2 	
CMOVE:
	LDR	R6,[R1],#4
	LDR	R7,[R1],#4
	B CMOV1
CMOV0:
	LDRB	R4,[R7],#1
	STRB	R4,[R6],#1
CMOV1:
	MOVS	R5,R5
	BEQ	CMOV2
	SUB	R5,R5,#1
	B CMOV0
CMOV2:
	_POP
	_NEXT

//    MOVE	( a1 a2 u -- )
// 	Copy u words from a1 to a2.

	.word	_CMOVE
_MOVE:	.byte   4
	.ascii "MOVE"
	.p2align 2 	
MOVE:
	AND	R5,R5,#-4
	LDR	R6,[R1],#4
	LDR	R7,[R1],#4
	B MOVE1
MOVE0:
	LDR	R4,[R7],#4
	STR	R4,[R6],#4
MOVE1:
	MOVS	R5,R5
	BEQ	MOVE2
	SUB	R5,R5,#4
	B MOVE0
MOVE2:
	_POP
	_NEXT

//    FILL	( b u c -- )
// 	Fill u bytes of character c to area beginning at b.

	.word	_MOVE
_FILL:	.byte   4
	.ascii "FILL"
	.p2align 2 	
FILL:
	LDR	R6,[R1],#4
	LDR	R7,[R1],#4
FILL0:
	B FILL1
	MOV	R5,R5
FILL1:
	STRB	R5,[R7],#1
	MOVS	R6,R6
	BEQ	FILL2
	SUB	R6,R6,#1
	B FILL0
FILL2:
	_POP
	_NEXT

//    PACK$	( b u a -- a )
// 	Build a counted word with u characters from b. Null fill.

	.word	_FILL
_PACKS:	.byte  5
	.ascii "PACK$$"
	.p2align 2 	
PACKS:
	_NEST
	_ADR	ALGND
	_ADR	DUPP
	_ADR	TOR			// strings only on cell boundary
	_ADR	OVER
	_ADR	PLUS
	_ADR	ONEP
	_DOLIT 	0xFFFFFFFC
	_ADR	ANDD			// count mod cell
	_DOLIT 	0
	_ADR	SWAP
	_ADR	STORE			// null fill cell
	_ADR	RAT
	_ADR	DDUP
	_ADR	CSTOR
	_ADR	ONEP			// save count
	_ADR	SWAP
	_ADR	CMOVE
	_ADR	RFROM
	_UNNEST   			// move string

// **************************************************************************
//  Numeric output, single precision

//    DIGIT	( u -- c )
// 	Convert digit u to a character.

	.word	_PACKS
_DIGIT:	.byte  5
	.ascii "DIGIT"
	.p2align 2 	
DIGIT:
	_NEST
	_DOLIT 9
	_ADR	OVER
	_ADR	LESS
	_DOLIT	7
	_ADR	ANDD
	_ADR	PLUS
	_DOLIT	'0'
	_ADR	PLUS 
	_UNNEST

//    EXTRACT	( n base -- n c )
// 	Extract the least significant digit from n.

	.word	_DIGIT
_EXTRC:	.byte  7
	.ascii "EXTRACT"
	.p2align 2 	
EXTRC:
	_NEST
	_DOLIT 0
	_ADR	SWAP
	_ADR	UMMOD
	_ADR	SWAP
	_ADR	DIGIT
	_UNNEST

//    <#	  ( -- )
// 	Initiate the numeric output process.

	.word	_EXTRC
_BDIGS:	.byte  2
	.ascii "<#"
	.p2align 2 	
BDIGS:
	_NEST
	_ADR	PAD
	_ADR	HLD
	_ADR	STORE
	_UNNEST

//    HOLD	( c -- )
// 	Insert a character into the numeric output string.

	.word	_BDIGS
_HOLD:	.byte  4
	.ascii "HOLD"
	.p2align 2 	
HOLD:
	_NEST
	_ADR	HLD
	_ADR	AT
	_ADR	ONEM
	_ADR	DUPP
	_ADR	HLD
	_ADR	STORE
	_ADR	CSTOR
	_UNNEST

//    #	   ( u -- u )
// 	Extract one digit from u and append the digit to output string.

	.word	_HOLD
_DIG:	.byte  1
	.ascii "#"
	.p2align 2 	
DIG:
	_NEST
	_ADR	BASE
	_ADR	AT
	_ADR	EXTRC
	_ADR	HOLD
	_UNNEST

//    #S	  ( u -- 0 )
// 	Convert u until all digits are added to the output string.

	.word	_DIG
_DIGS:	.byte  2
	.ascii "#S"
	.p2align 2 	
DIGS:
	_NEST
DIGS1:
    _ADR	DIG
	_ADR	DUPP
	_QBRAN 	DIGS2
	_BRAN	DIGS1
DIGS2:
	  _UNNEST

//    SIGN	( n -- )
// 	Add a minus sign to the numeric output string.

	.word	_DIGS
_SIGN:	.byte  4
	.ascii "SIGN"
	.p2align 2 	
SIGN:
	_NEST
	_ADR	ZLESS
	_QBRAN	SIGN1
	_DOLIT '-'
	_ADR	HOLD
SIGN1:
	  _UNNEST

//    #>	  ( w -- b u )
// 	Prepare the output word to be TYPE'd.

	.word	_SIGN
_EDIGS:	.byte  2
	.ascii "#>"
	.p2align 2 	
EDIGS:
	_NEST
	_ADR	DROP
	_ADR	HLD
	_ADR	AT
	_ADR	PAD
	_ADR	OVER
	_ADR	SUBB
	_UNNEST

//    str	 ( n -- b u )
// 	Convert a signed integer to a numeric string.

// 	.word	_EDIGS
// _STRR	.byte  3
// 	.ascii "str"
// 	.p2align 2 	
STRR:
	_NEST
	_ADR	DUPP
	_ADR	TOR
	_ADR	ABSS
	_ADR	BDIGS
	_ADR	DIGS
	_ADR	RFROM
	_ADR	SIGN
	_ADR	EDIGS
	_UNNEST

//    HEX	 ( -- )
// 	Use radix 16 as base for numeric conversions.

	.word	_EDIGS
_HEX:	.byte  3
	.ascii "HEX"
	.p2align 2 	
HEX:
	_NEST
	_DOLIT 16
	_ADR	BASE
	_ADR	STORE
	_UNNEST

//    DECIMAL	( -- )
// 	Use radix 10 as base for numeric conversions.

	.word	_HEX
_DECIM:	.byte  7
	.ascii "DECIMAL"
	.p2align 2 	
DECIM:
	_NEST
	_DOLIT 10
	_ADR	BASE
	_ADR	STORE
	_UNNEST

// **************************************************************************
//  Numeric input, single precision

//    DIGIT?	( c base -- u t )
// 	Convert a character to its numeric value. A flag indicates success.

	.word	_DECIM
_DIGTQ:	.byte  6
	.ascii "DIGIT?"
	.p2align 2 	
DIGTQ:
	_NEST
	_ADR	TOR
	_DOLIT 	'0'
	_ADR	SUBB
	_DOLIT 9
	_ADR	OVER
	_ADR	LESS
	_QBRAN	DGTQ1
	_DOLIT 7
	_ADR	SUBB
	_ADR	DUPP
	_DOLIT	10
	_ADR	LESS
	_ADR	ORR
DGTQ1:
	_ADR	DUPP
	_ADR	RFROM
	_ADR	ULESS
	_UNNEST

//    NUMBER?	( a -- n T | a F )
// 	Convert a number word to integer. Push a flag on tos.

	.word	_DIGTQ
_NUMBQ:	.byte  7
	.ascii "NUMBER?"
	.p2align 2 	
NUMBQ:
	_NEST
	_ADR	BASE
	_ADR	AT
	_ADR	TOR
	_DOLIT	0
	_ADR	OVER
	_ADR	COUNT
	_ADR	OVER
	_ADR	CAT
	_DOLIT '$'
	_ADR	EQUAL
	_QBRAN	NUMQ1
	_ADR	HEX
	_ADR	SWAP
	_ADR	ONEP
	_ADR	SWAP
	_ADR	ONEM
NUMQ1:
	_ADR	OVER
	_ADR	CAT
	_DOLIT	'-'
	_ADR	EQUAL
	_ADR	TOR
	_ADR	SWAP
	_ADR	RAT
	_ADR	SUBB
	_ADR	SWAP
	_ADR	RAT
	_ADR	PLUS
	_ADR	QDUP
	_QBRAN	NUMQ6
	_ADR	ONEM
	_ADR	TOR
NUMQ2:
	_ADR	DUPP
	_ADR	TOR
	_ADR	CAT
	_ADR	BASE
	_ADR	AT
	_ADR	DIGTQ
	_QBRAN	NUMQ4
	_ADR	SWAP
	_ADR	BASE
	_ADR	AT
	_ADR	STAR
	_ADR	PLUS
	_ADR	RFROM
	_ADR	ONEP
	_DONXT	NUMQ2
	_ADR	RAT
	_ADR	SWAP
	_ADR	DROP
	_QBRAN	NUMQ3
	_ADR	NEGAT
NUMQ3:
	_ADR	SWAP
	_BRAN	NUMQ5
NUMQ4:
	_ADR	RFROM
	_ADR	RFROM
	_ADR	DDROP
	_ADR	DDROP
	_DOLIT	0
NUMQ5:
	_ADR	DUPP
NUMQ6:
	_ADR	RFROM
	_ADR	DDROP
	_ADR	RFROM
	_ADR	BASE
	_ADR	STORE
	_UNNEST

// **************************************************************************
//  Basic I/O

//    KEY	 ( -- c )
// 	Wait for and return an input character.

	.word	_NUMBQ
_KEY:	.byte  3
	.ascii "KEY"
	.p2align 2 	
KEY:
	_NEST
KEY1:
	_ADR	QRX
	_QBRAN	KEY1
// CTRL-C reboot
	_ADR DUPP 
	_DOLIT	3 
	_ADR XORR
	_QBRAN	GO_REBOOT 
	_UNNEST
GO_REBOOT: 
	_ADR REBOOT 

//    SPACE	( -- )
// 	Send the blank character to the output device.

	.word	_KEY
_SPACE:	.byte  5
	.ascii "SPACE"
	.p2align 2 	
SPACE:
	_NEST
	_ADR	BLANK
	_ADR	EMIT
	_UNNEST

//    SPACES	( +n -- )
// 	Send n spaces to the output device.

	.word	_SPACE
_SPACS:	.byte  6
	.ascii "SPACES"
	.p2align 2 	
SPACS:
	_NEST
	_DOLIT	0
	_ADR	MAX
	_ADR	TOR
	_BRAN	CHAR2
CHAR1:
	_ADR	SPACE
CHAR2:
	_DONXT	CHAR1
	_UNNEST

//    TYPE	( b u -- )
// 	Output u characters from b.

	.word	_SPACS
_TYPEE:	.byte	4
	.ascii "TYPE"
	.p2align 2 	
TYPEE:
	_NEST
	_ADR  TOR   // ( a+1 -- R: u )
	_BRAN	TYPE2
TYPE1:  
	_ADR  COUNT
	_ADR TCHAR
	_ADR EMIT
TYPE2:  
	_DONXT	TYPE1
	_ADR	DROP
	_UNNEST

//    CR	  ( -- )
// 	Output a carriage return and a line feed.

	.word	_TYPEE
_CR:	.byte  2
	.ascii "CR"
	.p2align 2 	
CR:
	_NEST
	_DOLIT	CRR
	_ADR	EMIT
	_DOLIT	LF
	_ADR	EMIT
	_UNNEST

//    do_$	( -- a )
// 	Return the address of a compiled string.
//  adjust return address to skip over it.

// 	.word	_CR
// _DOSTR	.byte  COMPO+3
// 	.ascii "do$$"
// 	.p2align 2 	
DOSTR:
	_NEST     
/* compiled string address is 2 levels deep */
	_ADR	RFROM	// { -- a1 }
	_ADR	RFROM	//  {a1 -- a1 a2 } 
	_ADR	DUPP	// {a1 a2 -- a1 a2 a2 }
	_ADR	COUNT	//  get addr+1 count  { a1 a2 -- a1 a2 a2+1 c }
	_ADR	PLUS	// { -- a1 a2 a2+1+c }
	_ADR	ALGND	//  end of string
//	_ADR	ONEP	//  restore b0, this result in return address 2 level deep.
	_ADR	TOR		//  address after string { -- a1 a2 }
	_ADR	SWAP	//  count tugged
	_ADR	TOR     //  ( -- a2) is string address
	_UNNEST

//    $"|	( -- a )
// 	Run time routine compiled by _". Return address of a compiled string.

// 	.word	_DOSTR
// _STRQP	.byte  COMPO+3
// 	.ascii "$\"|"
// 	.p2align 2 	
STRQP:
	_NEST
	_ADR	DOSTR
	_UNNEST			// force a call to dostr

//    .$	( a -- )
// 	Run time routine of ." . Output a compiled string.

// 	.word	_STRQP
// _DOTST	.byte  COMPO+2
// 	.ascii ".$$"
// 	.p2align 2 	
DOTST:
	_NEST
	_ADR	COUNT // ( -- a+1 c )
	_ADR	TYPEE
	_UNNEST

//    ."|	( -- )
// 	Run time routine of ." . Output a compiled string.

// 	.word	_DOTST
// _DOTQP	.byte  COMPO+3
// 	.ascii ".""|"
// 	.p2align 2 	
DOTQP:
	_NEST
	_ADR	DOSTR
	_ADR	DOTST
	_UNNEST

//    .R	  ( n +n -- )
// 	Display an integer in a field of n columns, right justified.

	.word	_CR
_DOTR:	.byte  2
	.ascii ".R"
	.p2align 2 	
DOTR:
	_NEST
	_ADR	TOR
	_ADR	STRR
	_ADR	RFROM
	_ADR	OVER
	_ADR	SUBB
	_ADR	SPACS
	_ADR	TYPEE
	_UNNEST

//    U.R	 ( u +n -- )
// 	Display an unsigned integer in n column, right justified.

	.word	_DOTR
_UDOTR:	.byte  3
	.ascii "U.R"
	.p2align 2 	
UDOTR:
	_NEST
	_ADR	TOR
	_ADR	BDIGS
	_ADR	DIGS
	_ADR	EDIGS
	_ADR	RFROM
	_ADR	OVER
	_ADR	SUBB
	_ADR	SPACS
	_ADR	TYPEE
	_UNNEST

//    U.	  ( u -- )
// 	Display an unsigned integer in free format.

	.word	_UDOTR
_UDOT:	.byte  2
	.ascii "U."
	.p2align 2 	
UDOT:
	_NEST
	_ADR	BDIGS
	_ADR	DIGS
	_ADR	EDIGS
	_ADR	SPACE
	_ADR	TYPEE
	_UNNEST

//    .	   ( w -- )
// 	Display an integer in free format, preceeded by a space.

	.word	_UDOT
_DOT:	.byte  1
	.ascii "."
	.p2align 2 	
DOT:
	_NEST
	_ADR	BASE
	_ADR	AT
	_DOLIT 10
	_ADR	XORR			// ?decimal
	_QBRAN	DOT1
	_ADR	UDOT
	_UNNEST			// no,display unsigned
DOT1:
    _ADR	STRR
	_ADR	SPACE
	_ADR	TYPEE
	_UNNEST			// yes, display signed

//    ?	   ( a -- )
// 	Display the contents in a memory cell.

	.word	_DOT
_QUEST:	.byte  1
	.ascii "?"
	.p2align 2 	
QUEST:
	_NEST
	_ADR	AT
	_ADR	DOT
	_UNNEST

// **************************************************************************
//  Parsing

//    parse	( b u c -- b u delta //  string> )
// 	Scan word delimited by c. Return found string and its offset.

// 	.word	_QUEST
// _PARS	.byte  5
// 	.ascii "parse"
// 	.p2align 2 	
PARS:
	_NEST
	_ADR	TEMP
	_ADR	STORE
	_ADR	OVER
	_ADR	TOR
	_ADR	DUPP
	_QBRAN	PARS8
	_ADR	ONEM
	_ADR	TEMP
	_ADR	AT
	_ADR	BLANK
	_ADR	EQUAL
	_QBRAN	PARS3
	_ADR	TOR
PARS1:
	_ADR	BLANK
	_ADR	OVER
	_ADR	CAT			// skip leading blanks 
	_ADR	SUBB
	_ADR	ZLESS
	_ADR	INVER
	_QBRAN	PARS2
	_ADR	ONEP
	_DONXT	PARS1
	_ADR	RFROM
	_ADR	DROP
	_DOLIT	0
	_ADR	DUPP
	_UNNEST
PARS2:
	_ADR	RFROM
PARS3:
	_ADR	OVER
	_ADR	SWAP
	_ADR	TOR
PARS4:
	_ADR	TEMP
	_ADR	AT
	_ADR	OVER
	_ADR	CAT
	_ADR	SUBB			// scan for delimiter
	_ADR	TEMP
	_ADR	AT
	_ADR	BLANK
	_ADR	EQUAL
	_QBRAN	PARS5
	_ADR	ZLESS
PARS5:
	_QBRAN	PARS6
	_ADR	ONEP
	_DONXT	PARS4
	_ADR	DUPP
	_ADR	TOR
	_BRAN	PARS7
PARS6:
	_ADR	RFROM
	_ADR	DROP
	_ADR	DUPP
	_ADR	ONEP
	_ADR	TOR
PARS7:
	_ADR	OVER
	_ADR	SUBB
	_ADR	RFROM
	_ADR	RFROM
	_ADR	SUBB
	_UNNEST
PARS8:
	_ADR	OVER
	_ADR	RFROM
	_ADR	SUBB
	_UNNEST

//    PARSE	( c -- b u //  string> )
// 	Scan input stream and return counted string delimited by c.

	.word	_QUEST
_PARSE:	.byte  5
	.ascii "PARSE"
	.p2align 2 	
PARSE:
	_NEST
	_ADR	TOR
	_ADR	TIB
	_ADR	INN
	_ADR	AT
	_ADR	PLUS			// current input buffer pointer
	_ADR	NTIB
	_ADR	AT
	_ADR	INN
	_ADR	AT
	_ADR	SUBB			// remaining count
	_ADR	RFROM
	_ADR	PARS
	_ADR	INN
	_ADR	PSTOR
	_UNNEST

//    .(	  ( -- )
// 	Output following string up to next ) .

	.word	_PARSE
_DOTPR:	.byte  IMEDD+2
	.ascii ".("
	.p2align 2 	
DOTPR:
	_NEST
	_DOLIT	')'
	_ADR	PARSE
	_ADR	TYPEE
	_UNNEST

//    (	   ( -- )
// 	Ignore following string up to next ) . A comment.

	.word	_DOTPR
_PAREN:	.byte  IMEDD+1
	.ascii "("
	.p2align 2 	
PAREN:
	_NEST
	_DOLIT	')'
	_ADR	PARSE
	_ADR	DDROP
	_UNNEST

//    \	   ( -- )
// 	Ignore following text till the end of line.

	.word	_PAREN
_BKSLA:	.byte  IMEDD+1
	.byte	'\\'
	.p2align 2 	
BKSLA:
	_NEST
	_ADR	NTIB
	_ADR	AT
	_ADR	INN
	_ADR	STORE
	_UNNEST

//    CHAR	( -- c )
// 	Parse next word and return its first character.

	.word	_BKSLA
_CHAR:	.byte  4
	.ascii "CHAR"
	.p2align 2 	
CHAR:
	_NEST
	_ADR	BLANK
	_ADR	PARSE
	_ADR	DROP
	_ADR	CAT
	_UNNEST

//    WORD	( c -- a //  string> )
// 	Parse a word from input stream and copy it to code dictionary.

	.word	_CHAR
_WORDD:	.byte  4
	.ascii "WORD"
	.p2align 2 	
WORDD:
	_NEST
	_ADR	PARSE
	_ADR	HERE
	_ADR	CELLP
	_ADR	PACKS
	_UNNEST

//    TOKEN	( -- a //  string> )
// 	Parse a word from input stream and copy it to name dictionary.

	.word	_WORDD
_TOKEN:	.byte  5
	.ascii "TOKEN"
	.p2align 2 	
TOKEN:
	_NEST
	_ADR	BLANK
	_ADR	WORDD
	_UNNEST

// **************************************************************************
//  Dictionary search

//    NAME>	( na -- ca )
// 	Return a code address given a name address.

	.word	_TOKEN
_NAMET:	.byte  5
	.ascii "NAME>"
	.p2align 2 	
NAMET:
	_NEST
	_ADR	COUNT
	_DOLIT	0x1F
	_ADR	ANDD
	_ADR	PLUS
	_ADR	ALGND
	_UNNEST

//    SAME?	( a1 a2 u -- a1 a2 f | -0+ )
// 	Compare u bytes in two strings. Return 0 if identical.
//
//  Picatout 2020-12-01, 
//      Because of problem with .align directive that
// 		doesn't fill with zero's I had to change the "SAME?" and "FIND" 
// 		words  to do a byte by byte comparison. 
//
	.word	_NAMET
_SAMEQ:	.byte  5
	.ascii "SAME?"
	.p2align 2	
SAMEQ:
	_NEST
	_ADR	TOR
	_BRAN	SAME2
SAME1:
	_ADR	OVER  // ( a1 a2 -- a1 a2 a1 )
	_ADR	RAT   // a1 a2 a1 u 
	_ADR	PLUS  // a1 a2 a1+u 
	_ADR	CAT	   // a1 a2 c1    		
	_ADR	OVER  // a1 a2 c1 a2 
	_ADR	RAT    
	_ADR	PLUS    
	_ADR	CAT	  // a1 a2 c1 c2
	_ADR	SUBB  
	_ADR	QDUP
	_QBRAN	SAME2
	_ADR	RFROM
	_ADR	DROP
	_UNNEST	// strings not equal
SAME2:
	_DONXT	SAME1
	_DOLIT	0
	_UNNEST	// strings equal

//    find	( a na -- ca na | a F )
// 	Search a vocabulary for a string. Return ca and na if succeeded.

//  Picatout 2020-12-01,  
//		Modified from original. See comment for word "SAME?" 

// 	.word	_SAMEQ
// _FIND	.byte  4
// 	.ascii "find"
// 	.p2align 2 	
FIND:
	_NEST
	_ADR	SWAP			// na a	
	_ADR	COUNT			// na a+1 count
	_ADR	DUPP 
	_ADR	TEMP
	_ADR	STORE			// na a+1 count 
	_ADR  TOR		// na a+1  R: count  
	_ADR	SWAP			// a+1 na
FIND1:
	_ADR	DUPP			// a+1 na na
	_QBRAN	FIND6	// end of vocabulary
	_ADR	DUPP			// a+1 na na
	_ADR	CAT			// a+1 na name1
	_DOLIT	MASKK
	_ADR	ANDD
	_ADR	RAT			// a+1 na name1 count 
	_ADR	XORR			// a+1 na,  same length?
	_QBRAN	FIND2
	_ADR	CELLM			// a+1 la
	_ADR	AT			// a+1 next_na
	_BRAN	FIND1			// try next word
FIND2:   
	_ADR	ONEP			// a+1 na+1
	_ADR	TEMP
	_ADR	AT			// a+1 na+1 count
	_ADR	SAMEQ		// a+1 na+1 ? 
FIND3:	
	_BRAN	FIND4
FIND6:	
	_ADR	RFROM			// a+1 0 name1 -- , no match
	_ADR	DROP			// a+1 0
	_ADR	SWAP			// 0 a+1
	_ADR	ONEM			// 0 a
	_ADR	SWAP			// a 0 
	_UNNEST			// return without a match
FIND4:	
	_QBRAN	FIND5			// a+1 na+1
	_ADR	ONEM			// a+1 na
	_ADR	CELLM			// a+4 la
	_ADR	AT			// a+1 next_na
	_BRAN	FIND1			// compare next name
FIND5:	
	_ADR	RFROM			// a+1 na+1 count
	_ADR	DROP			// a+1 na+1
	_ADR	SWAP			// na+1 a+1
	_ADR	DROP			// na+1
	_ADR	ONEM			// na
	_ADR	DUPP			// na na
	_ADR	NAMET			// na ca
	_ADR	SWAP			// ca na
	_UNNEST			//  return with a match

//    NAME?	( a -- ca na | a F )
// 	Search all context vocabularies for a string.

	.word	_SAMEQ
_NAMEQ:	.byte  5
	.ascii "NAME?"
	.p2align 2 	
NAMEQ:
	_NEST
	_ADR	CNTXT
	_ADR	AT
	_ADR	FIND
	_UNNEST

// **************************************************************************
//  Terminal input

//    	  ( bot eot cur -- bot eot cur )
// 	Backup the cursor by one character.

// 	.word	_NAMEQ
// _BKSP	.byte  2
// 	.ascii "^H"
// 	.p2align 2 	
BKSP:
	_NEST
	_ADR	TOR
	_ADR	OVER
	_ADR	RFROM
	_ADR	SWAP
	_ADR	OVER
	_ADR	XORR
	_QBRAN	BACK1
	_DOLIT	BKSPP
	_ADR	TECHO
// 	_ADR	ATEXE
	_ADR	ONEM
	_ADR	BLANK
	_ADR	TECHO
// 	_ADR	ATEXE
	_DOLIT	BKSPP
	_ADR	TECHO
// 	_ADR	ATEXE
BACK1:
	  _UNNEST

//    TAP	 ( bot eot cur c -- bot eot cur )
// 	Accept and echo the key stroke and bump the cursor.

// 	.word	_BKSP
// _TAP	.byte  3
// 	.ascii "TAP"
// 	.p2align 2 	
TAP:
	_NEST
	_ADR	DUPP
	_ADR	TECHO
// 	_ADR	ATEXE
	_ADR	OVER
	_ADR	CSTOR
	_ADR	ONEP
	_UNNEST

//    kTAP	( bot eot cur c -- bot eot cur )
// 	Process a key stroke, CR or backspace.

// 	.word	_TAP
// _KTAP	.byte  4
// 	.ascii "kTAP"
// 	.p2align 2 	
KTAP:
TTAP:
	_NEST
	_ADR	DUPP
	_DOLIT	CRR
	_ADR	XORR
	_QBRAN  KTAP2
	_DOLIT	BKSPP
	_ADR	XORR
	_QBRAN	KTAP1
	_ADR	BLANK
	_ADR	TAP
	_UNNEST
//	.word	0			// patch
KTAP1:
	_ADR	BKSP
	_UNNEST
KTAP2:
	_ADR	DROP
	_ADR	SWAP
	_ADR	DROP
	_ADR	DUPP
	_UNNEST

//    ACCEPT	( b u -- b u )
// 	Accept characters to input buffer. Return with actual count.

	.word	_NAMEQ
_ACCEP:	.byte  6
	.ascii "ACCEPT"
	.p2align 2 	
ACCEP:
	_NEST
	_ADR	OVER
	_ADR	PLUS
	_ADR	OVER
ACCP1:
	_ADR	DDUP
	_ADR	XORR
	_QBRAN	ACCP4
	_ADR	KEY
	_ADR	DUPP
	_ADR	BLANK
	_DOLIT 127
	_ADR	WITHI
	_QBRAN	ACCP2
	_ADR	TAP
	_BRAN	ACCP3
ACCP2:
	_ADR	KTAP
// 	_ADR	ATEXE
ACCP3:	  
	_BRAN	ACCP1
ACCP4:
	_ADR	DROP
	_ADR	OVER
	_ADR	SUBB
	_UNNEST

//    QUERY	( -- )
// 	Accept input stream to terminal input buffer.

	.word	_ACCEP
_QUERY:	.byte  5
	.ascii "QUERY"
	.p2align 2 	
QUERY:
	_NEST
	_ADR	TIB
	_DOLIT 80
	_ADR	ACCEP
	_ADR	NTIB
	_ADR	STORE
	_ADR	DROP
	_DOLIT	0
	_ADR	INN
	_ADR	STORE
	_UNNEST

// **************************************************************************
//  Error handling

//    ABORT	( a -- )
// 	Reset data stack and jump to QUIT.

	.word	_QUERY
_ABORT:	.byte  5
	.ascii "ABORT"
	.p2align 2 	
ABORT:
	_NEST
	_ADR	SPACE
	_ADR	COUNT
	_ADR	TYPEE
	_DOLIT	0X3F
	_ADR	EMIT
	_ADR	CR
	_ADR	PRESE
	_BRAN	QUIT

//    _abort"	( f -- )
// 	Run time routine of ABORT" . Abort with a message.

// 	.word	_ABORT
// _ABORQ	.byte  COMPO+6
// 	.ascii "abort\""
// 	.p2align 2 	
ABORQ:
	_NEST
	_QBRAN	ABOR1	// text flag
	_ADR	DOSTR
	_ADR	COUNT
	_ADR	TYPEE
	_ADR	CR
	_BRAN	QUIT
ABOR1:
	_ADR	DOSTR
	_ADR	DROP
	_UNNEST			// drop error

// **************************************************************************
//  The text interpreter

//    $INTERPRET  ( a -- )
// 	Interpret a word. If failed, try to convert it to an integer.

	.word	_ABORT
_INTER:	.byte  10
	.ascii "$$INTERPRET"
	.p2align 2 	
INTER:
	_NEST
	_ADR	NAMEQ
	_ADR	QDUP	// ?defined
	_QBRAN	INTE1
	_ADR	AT
	_DOLIT	COMPO
	_ADR	ANDD	// ?compile only lexicon bits
	_ABORQ	13," compile only"
	_ADR	EXECU
	_UNNEST			// execute defined word
INTE1:
	_ADR	NUMBQ
	_QBRAN	INTE2
	_UNNEST
INTE2:
	_BRAN	ABORT	// error

//    [	   ( -- )
// 	Start the text interpreter.

	.word	_INTER
_LBRAC:	.byte  IMEDD+1
	.ascii "["
	.p2align 2 	
LBRAC:
	_NEST
	_DOLIT	INTER
	_ADR	TEVAL
	_ADR	STORE
	_UNNEST

//    .OK	 ( -- )
// 	Display "ok" only while interpreting.

	.word	_LBRAC
_DOTOK:	.byte  3
	.ascii ".OK"
	.p2align 2 	
DOTOK:
	_NEST
	_DOLIT	INTER
	_ADR	TEVAL
	_ADR	AT
	_ADR	EQUAL
	_QBRAN	DOTO1
	_DOTQP	3," ok"
DOTO1:
	_ADR	CR
	_UNNEST

//    ?STACK	( -- )
// 	Abort if the data stack underflows.

	.word	_DOTOK
_QSTAC:	.byte  6
	.ascii "?STACK"
	.p2align 2 	
QSTAC:
	_NEST
	_ADR	DEPTH
	_ADR	ZLESS	// check only for underflow
	_ABORQ	10," underflow"
	_UNNEST

//    EVAL	( -- )
// 	Interpret the input stream.

	.word	_QSTAC
_EVAL:	.byte  4
	.ascii "EVAL"
	.p2align 2 	
EVAL:
	_NEST
EVAL1:
    _ADR	TOKEN
	_ADR	DUPP
	_ADR	CAT	// ?input stream empty
	_QBRAN	EVAL2
	_ADR	TEVAL
	_ADR	ATEXE
	_ADR	QSTAC	// evaluate input, check stack
	_BRAN	EVAL1
EVAL2:
	_ADR	DROP
	_ADR	DOTOK
	_UNNEST	// prompt

//    PRESET	( -- )
// 	Reset data stack pointer and the terminal input buffer.

	.word	_EVAL
_PRESE:	.byte  6
	.ascii "PRESET"
	.p2align 2 	
PRESE:
	_NEST 
	_DOLIT SPP 
	_ADR SPSTOR 
	_UNNEST 

//    QUIT	( -- )
// 	Reset return stack pointer and start text interpreter.

	.word	_PRESE
_QUIT:	.byte  4
	.ascii "QUIT"
	.p2align 2 	
QUIT:
	_DOLIT RPP 
	_ADR RPSTOR 
QUIT1:
	_ADR	LBRAC			// start interpretation
QUIT2:
	_ADR	QUERY			// get input
	_ADR	EVAL
	_BRAN	QUIT2	// continue till error

/***************************
//  Flash memory interface
***************************/

unlock:	//  unlock flash memory	
	ldr	r0, flash_regs 
//	mov r4,#(0xD<<2) // clear EOP|WRPRTERR|PGERR bits 
//	str r4,[r0,#FLASH_SR]
//	ldr r4,[r0,#FLASH_CR]
//	tst r4,#(1<<7)
//	beq 1f 
	ldr	r4, flash_regs+4 // key1
	str	r4, [r0, #FLASH_KEYR]
	ldr	r4, flash_regs+8 // key2 
	str	r4, [r0, #FLASH_KEYR]
	/* unlock option registers */
/*	
	ldr	r4, flash_regs+4 
	str	r4, [r0, #FLASH_OPTKEYR]
	ldr	r4, flash_regs+8
	str	r4, [r0, #FLASH_OPTKEYR]
*/
	bx lr 

WAIT_BSY:
	ldr	r0,flash_regs
WAIT1:
	ldr	r4, [r0, #FLASH_SR]	//  FLASH_SR
	ands	r4, #0x1	//  BSY
	bne	WAIT1
	_RET 

//    ERASE_PAGE	   ( adr -- )
// 	  Erase one page of flash memory.
//    stm32f103 page size is 1024 bytes 
//    adr is any address inside page to erase 

	.word _QUIT  
_EPAGE:	.byte  10
	.ascii "ERASE_PAGE"
	.p2align 2 	

EPAGE: 	//  page --
	_CALL	WAIT_BSY
	ldr r0,flash_regs 	 
	mov r4,#2 // set PER bit 
	str r4,[r0,#FLASH_CR]
	str r5,[r0,#FLASH_AR] // page to erase address 
	ldr	r4,[r0, #FLASH_CR]	
	orr	R4,#0x40	//  set STRT bit   
	str	r4,[r0, #FLASH_CR]	//  start erasing
 	_CALL	WAIT_BSY // wait until done
	ldr r5,[r0,#FLASH_SR] // check for errors 
	and r5,r5,#(5<<2)
	_NEST 
	_ABORQ 	13," erase error!"
	_UNNEST

// store 16 bit word
// expect flash unlocked  
HWORD_WRITE: // ( hword address -- )
	ldr	r4, [r0, #FLASH_CR]	//  FLASH_CR
	mov r4,#1 // set PG 
	str r4,[r0,#FLASH_CR]
	mov r6,r5 
	_POP 
	strh r5,[r6] 
	_CALL WAIT_BSY 
	ldr r5,[r0,#FLASH_SR]
	and r5,r5,#(5<<2) 
	_NEST 
	_QBRAN	1f 
	_ABORQ	13," write error!"
	.p2align 2
1:	 
	_UNNEST 


//    I!	   ( data address -- )
// 	   Write one word into flash memory
//	   address must even 

	.word	_EPAGE
_ISTOR:	.byte  2
	.ascii "I!"
	.p2align 2 	
ISTOR:	//  data address --
	_CALL	WAIT_BSY
	_NEST 
	_ADR DDUP 
	_ADR TOR 
	_ADR TOR 
	_ADR HWORD_WRITE
	_ADR RFROM
	_DOLIT 65536 
	_ADR  SLASH
	_ADR RFROM 
	_DOLIT 2 
	_ADR PLUS 
	_ADR HWORD_WRITE 
	_UNNEST

// IMG_SIZE ( -- u )
// return flash pages required to save 
// user ram  
	.word _ISTOR 
_IMG_SIZE: .byte 8
	.ascii "IMG_SIZE" 
	.p2align 2
IMG_SIZE: 
	_NEST
	_DOLIT VARS_END_OFS-IMG_SIGN_OFS 
	_ADR USER_END 
	_ADR USER_BEGIN 
	_ADR SUBB 
	_ADR PLUS 
	_DOLIT PAGE_SIZE 
	_ADR SLMOD 
	_ADR SWAP 
	_QBRAN 1f 
	_ADR ONEP
1:
	_UNNEST  

// IMG? ( n -- T|F )
// check if an image has been saved in slot n 
	.word _IMG_SIZE 
_IMGQ: .byte 4
	.ascii "IMG?"
	.p2align 2
IMGQ:
	_NEST 
	_ADR IMG_ADR 
	_ADR AT 
	_ADR IMG_SIGN  
	_ADR XORR  
	_ADR ZEQUAL
	_UNNEST

// IMG_ADR ( n -- a )
// return image address from its number
// IMG_ADR=USER_SPACE+IMG_SIZE*1024*n  
	.word _IMGQ
_IMG_ADR: .byte 7 
	.ascii "IMG_ADR"
	.p2align 2 
IMG_ADR:
	_NEST 
	_ADR IMG_SIZE // number of pages per image. 
	_DOLIT 10
	_ADR LSHIFT
	_ADR STAR     // * n 
	_ADR IMAGE0 
	_ADR PLUS    // + USER_SPACE  
	_UNNEST 

// LOAD_IMG ( n -- )
// Load image in slot n in RAM. 
	.word _IMG_ADR
_LOAD_IMG: .byte 8 
	.ascii "LOAD_IMG" 
	.p2align 2 
LOAD_IMG:
	_NEST 
	_ADR DUPP 
	_ADR IMGQ 
	_QBRAN 1f
/* copy system variables to RAM */
	_ADR IMG_ADR 
	_ADR DUPP 
	_ADR TOR   // save source address 
	_ADR IMG_SIGN 
	_DOLIT (VARS_END_OFS-IMG_SIGN_OFS) 
	_ADR DUPP 
	_ADR TOR 
	_ADR MOVE // ( src dest count -- ) R: src count 
/* copy user definitions */
	_ADR RFROM 
	_ADR RFROM  
	_ADR PLUS // source address  
	_ADR USER_BEGIN // destination address
	_ADR HERE  
	_ADR OVER 
	_ADR SUBB  // byte count 
	_ADR MOVE
	_UNNEST  
1:	_ADR DROP 
	_UNNEST 


// ERASE_MPG ( u1 u2 -- )
// erase many pages 
// u1 first page number 
// u2 how many pages  
	.word _LOAD_IMG
_ERASE_MPG: .byte 9 
	.ascii "ERASE_MPG"	
	.p2align 2 
ERASE_MPG:
	_NEST 
	_ADR TOR 
	_ADR PG_TO_ADR 
	_BRAN 2f 
1:
	_ADR DUPP 
	_ADR TOR 
	_ADR EPAGE 
	_ADR RFROM
	_DOLIT PAGE_SIZE 
	_ADR PLUS 
2:
	_DONXT 1b 
	_ADR TPOP 
	_UNNEST 

// FLSH_WR ( src dest u -- dest+u )
// write u words to flash memory 
	.word _ERASE_MPG
_FLSH_WR: .byte 7 
	.ascii "FLSH_WR"
	.p2align  
FLSH_WR: 
	_NEST 
	_ADR TOR
	_BRAN 3f  
/* write system variables to FLASH */
2:  _ADR TOR  // destination address 
	_ADR DUPP 
	_ADR AT   // get data 
	_ADR RAT  // get destination address 
	_ADR ISTOR
	_ADR CELLP  // increment source address 
	_ADR RFROM 
	_ADR CELLP  // increment dest address 
3:	_DONXT 2b
	_ADR TOR 
	_ADR DROP 
	_ADR RFROM 
	_UNNEST 

// ADR>PG ( a -- n )
// convert address to page number, {0..127} 
	.word _FLSH_WR
_ADR_TO_PG: .byte 6 
	.ascii "ADR>PG" 
	.p2align 2 
ADR_TO_PG: 
	lsr r5,#10 
	and r5,#127 
	_NEXT  

// PG>ADR ( n -- a )
// convert page# to address 
	.word _ADR_TO_PG
_PG_TO_ADR: .byte 6 
	.ascii "PG>ADR" 
	.p2align 2 
PG_TO_ADR:
	movt r5,#2
	lsl r5,#10 
	_NEXT 

// ERASE_IMG ( n -- )
// erase image in slot n  
	.word _PG_TO_ADR 
_ERASE_IMG: .byte 9
	.ascii "ERASE_IMG"
	.p2align 2
ERASE_IMG:
	_NEST
	_ADR IMG_ADR 
	_ADR IMG_SIZE 
	_ADR TOR 
	_BRAN 2f 
1:	_ADR DUPP 
	_ADR EPAGE
	_DOLIT PAGE_SIZE
	_ADR PLUS  
2:	_DONXT 1b 
	_ADR DROP 
	_UNNEST 

// SAVE_IMG ( n -- )
// copy in flash RAM system variables and user defintitions.
// n is image slot number 
	.word _ERASE_IMG	
_SAVE_IMG: .byte 8 
	.ascii "SAVE_IMG"
	.p2align 2
SAVE_IMG:
	_NEST 
	_ADR HERE 
	_ADR USER_BEGIN
	_ADR EQUAL 
	_QBRAN 1f 
	_ADR DROP 
	_UNNEST  // nothing to save 
1:	_ADR DUPP 
	_ADR IMGQ 
	_QBRAN 2f
/* delete saved image */
	_ADR DUPP 
	_ADR ERASE_IMG 
/* save system variables */
2:	_ADR IMG_ADR // where to save
	_ADR IMG_SIGN 
	_ADR SWAP  //  ( src dest --  
	_DOLIT (VARS_END_OFS-IMG_SIGN_OFS) 
	_ADR CELLSL  // word count 
	_ADR FLSH_WR  // ( src dest count -- dest+u )
/* write user definitions */
	_ADR USER_BEGIN
	_ADR SWAP  // ( src dest+ -- )
	_ADR HERE   
	_ADR USER_BEGIN 
	_ADR SUBB 
	_ADR CELLSL  // src dest+ count -- 
	_ADR FLSH_WR  
	_UNNEST 

// TURNKEY ( -- "WORD") 
// set autorun program in 'BOOT variable 
// and save image in slot 0.
	.word _SAVE_IMG
_TURNKEY: .byte 7
	.ascii "TURNKEY"
	.p2align 2 
TURNKEY:
	_NEST 
	_ADR TICK 
	_ADR TBOOT 
	_ADR STORE 
	_DOLIT  0 
	_ADR SAVE_IMG 
	_UNNEST

	.word _TURNKEY
_FORGET: .byte 6 
	.ascii "FORGET"
	.p2align 2
FORGET:
	_NEST 
	_ADR TOKEN 
	_ADR DUPP 
	_QBRAN 9f 
	_ADR NAMEQ // ( a -- ca na | a 0 )
	_ADR QDUP 
	_QBRAN 8f
	_ADR CELLM // ( ca la )
	_ADR DUPP 
	_ADR CPP   
	_ADR STORE
	_ADR AT 
	_ADR LAST 
	_ADR STORE
	_ADR OVERT 
8:  _ADR DROP 
9:	_UNNEST 

	.p2align 2 
flash_regs:
	.word FLASH_BASE_ADR // 0 
	.word FLASH_KEY1   // 4 
	.word FLASH_KEY2   // 8

// **************************************************************************
//  The compiler

//    '	   ( -- ca )
// 	Search context vocabularies for the next word in input stream.

	.word	_FORGET
_TICK:	.byte  1
	.ascii "'"
	.p2align 2 	
TICK:
	_NEST
	_ADR	TOKEN
	_ADR	NAMEQ	// ?defined
	_QBRAN	TICK1
	_UNNEST	// yes, push code address
TICK1:	
	_BRAN	ABORT	// no, error

//    ALLOT	( n -- )
// 	Allocate n bytes to the ram area.

	.word	_TICK
_ALLOT:	.byte  5
	.ascii "ALLOT"
	.p2align 2 	
ALLOT:
	_NEST
	_ADR	CPP
	_ADR	PSTOR
	_UNNEST			// adjust code pointer

//    ,	   ( w -- )
// 	Compile an integer into the code dictionary.

	.word	_ALLOT
_COMMA:	.byte  1,','
	.p2align 2 	
COMMA:
	_NEST
	_ADR	HERE
	_ADR	DUPP
	_ADR	CELLP	// cell boundary
	_ADR	CPP
	_ADR	STORE
	_ADR	STORE
	_UNNEST	// adjust code pointer, compile
	.p2align 2 
//    [COMPILE]   ( -- //  string> )
// 	Compile the next immediate word into code dictionary.

	.word	_COMMA
_BCOMP:	.byte  IMEDD+9
	.ascii "[COMPILE]"
	.p2align 2 	
BCOMP:
	_NEST
	_ADR	TICK
	_ADR	COMMA
	_UNNEST

//    COMPILE	( -- )
// 	Compile the next address in colon list to code dictionary.

	.word	_BCOMP
_COMPI:	.byte  COMPO+7
	.ascii "COMPILE"
	.p2align 2 	
COMPI:
	_NEST
	_ADR	RFROM
	_ADR	DUPP 
	_ADR	AT
	_DOLIT 1 
	_ADR	ORR 
	_ADR	COMMA 
	_ADR	CELLP 
	_ADR	TOR 
	_UNNEST			// adjust return address

//    LITERAL	( w -- )
// 	Compile tos to code dictionary as an integer literal.

	.word	_COMPI
_LITER:	.byte  IMEDD+7
	.ascii "LITERAL"
	.p2align 2 	
LITER:
	_NEST
	_COMPI	DOLIT
	_ADR	COMMA
	_UNNEST

//    $,"	( -- )
// 	Compile a literal string up to next " .

// 	.word	_LITER
// _STRCQ	.byte  3
// 	.ascii "$$,"""
// 	.p2align 2 	
STRCQ:
	_NEST
	_DOLIT -4
	_ADR	CPP
	_ADR	PSTOR
	_DOLIT	'\"'
	_ADR	WORDD			// move word to code dictionary
	_ADR	COUNT
	_ADR	PLUS
	_ADR	ALGND			// calculate aligned end of string
	_ADR	CPP
	_ADR	STORE
	_UNNEST 			// adjust the code pointer

// **************************************************************************
//  Structures

//    FOR	 ( -- a )
// 	Start a FOR-NEXT loop structure in a colon definition.

	.word	_LITER
_FOR:	.byte  COMPO+IMEDD+3
	.ascii "FOR"
	.p2align 2 	
FOR:
	_NEST
	_COMPI	TOR
	_ADR	HERE
	_UNNEST

//    BEGIN	( -- a )
// 	Start an infinite or indefinite loop structure.

	.word	_FOR
_BEGIN:	.byte  COMPO+IMEDD+5
	.ascii "BEGIN"
	.p2align 2 	
BEGIN:
	_NEST
	_ADR	HERE
	_UNNEST
	.p2align 2 
//    NEXT	( a -- )
// 	Terminate a FOR-NEXT loop structure.

	.word	_BEGIN
_FNEXT:	.byte  COMPO+IMEDD+4
	.ascii "NEXT"
	.p2align 2 	
FNEXT:
	_NEST
	_COMPI	DONXT
	_ADR	COMMA
	_UNNEST

//    UNTIL	( a -- )
// 	Terminate a BEGIN-UNTIL indefinite loop structure.

	.word	_FNEXT
_UNTIL:	.byte  COMPO+IMEDD+5
	.ascii "UNTIL"
	.p2align 2 	
UNTIL:
	_NEST
	_COMPI	QBRAN
	_ADR	COMMA
	_UNNEST

//    AGAIN	( a -- )
// 	Terminate a BEGIN-AGAIN infinite loop structure.

	.word	_UNTIL
_AGAIN:	.byte  COMPO+IMEDD+5
	.ascii "AGAIN"
	.p2align 2 	
AGAIN:
	_NEST
	_COMPI	BRAN
	_ADR	COMMA
	_UNNEST

//    IF	  ( -- A )
// 	Begin a conditional branch structure.

	.word	_AGAIN
_IFF:	.byte  COMPO+IMEDD+2
	.ascii "IF"
	.p2align 2 	
IFF:
	_NEST
	_COMPI	QBRAN
	_ADR	HERE
	_DOLIT	4
	_ADR	CPP
	_ADR	PSTOR
	_UNNEST

//    AHEAD	( -- A )
// 	Compile a forward branch instruction.

	.word	_IFF
_AHEAD:	.byte  COMPO+IMEDD+5
	.ascii "AHEAD"
	.p2align 2 	
AHEAD:
	_NEST
	_COMPI	BRAN
	_ADR	HERE
	_DOLIT	4
	_ADR	CPP
	_ADR	PSTOR
	_UNNEST

//    REPEAT	( A a -- )
// 	Terminate a BEGIN-WHILE-REPEAT indefinite loop.

	.word	_AHEAD
_REPEA:	.byte  COMPO+IMEDD+6
	.ascii "REPEAT"
	.p2align 2 	
REPEA:
	_NEST
	_ADR	AGAIN
	_ADR	HERE
	_ADR	SWAP
	_ADR	STORE
	_UNNEST

//    THEN	( A -- )
// 	Terminate a conditional branch structure.

	.word	_REPEA
_THENN:	.byte  COMPO+IMEDD+4
	.ascii "THEN"
	.p2align 2 	
THENN:
	_NEST
	_ADR	HERE
	_ADR	SWAP
	_ADR	STORE
	_UNNEST

//    AFT	 ( a -- a A )
// 	Jump to THEN in a FOR-AFT-THEN-NEXT loop the first time through.

	.word	_THENN
_AFT:	.byte  COMPO+IMEDD+3
	.ascii "AFT"
	.p2align 2 	
AFT:
	_NEST
	_ADR	DROP
	_ADR	AHEAD
	_ADR	BEGIN
	_ADR	SWAP
	_UNNEST

//    ELSE	( A -- A )
// 	Start the false clause in an IF-ELSE-THEN structure.

	.word	_AFT
_ELSEE:	.byte  COMPO+IMEDD+4
	.ascii "ELSE"
	.p2align 2 	
ELSEE:
	_NEST
	_ADR	AHEAD
	_ADR	SWAP
	_ADR	THENN
	_UNNEST

//    WHILE	( a -- A a )
// 	Conditional branch out of a BEGIN-WHILE-REPEAT loop.

	.word	_ELSEE
_WHILE:	.byte  COMPO+IMEDD+5
	.ascii "WHILE"
	.p2align 2 	
WHILE:
	_NEST
	_ADR	IFF
	_ADR	SWAP
	_UNNEST

//    ABORT"	( -- //  string> )
// 	Conditional abort with an error message.

	.word	_WHILE
_ABRTQ:	.byte  IMEDD+6
	.ascii "ABORT\""
	.p2align 2 	
ABRTQ:
	_NEST
	_COMPI	ABORQ
	_ADR	STRCQ
	_UNNEST

//    $"	( -- //  string> )
// 	Compile an inlineDCB literal.

	.word	_ABRTQ
_STRQ:	.byte  IMEDD+2
	.byte	'$','"'
	.p2align 2 	
STRQ:
	_NEST
	_COMPI	STRQP
	_ADR	STRCQ
	_UNNEST

//    ."	( -- //  string> )
// 	Compile an inline word  literal to be typed out at run time.

	.word	_STRQ
_DOTQ:	.byte  IMEDD+COMPO+2
	.byte	'.','"'
	.p2align 2 	
DOTQ:
	_NEST
	_COMPI	DOTQP
	_ADR	STRCQ
	_UNNEST

// **************************************************************************
//  Name compiler

//    ?UNIQUE	( a -- a )
// 	Display a warning message if the word already exists.

	.word	_DOTQ
_UNIQU:	.byte  7
	.ascii "?UNIQUE"
	.p2align 2 	
UNIQU:
	_NEST
	_ADR	DUPP
	_ADR	NAMEQ			// ?name exists
	_QBRAN	UNIQ1	// redefinitions are OK
	_DOTQP	7," reDef "		// but warn the user
	_ADR	OVER
	_ADR	COUNT
	_ADR	TYPEE			// just in case its not planned
UNIQ1:
	_ADR	DROP
	_UNNEST

//    $,n	 ( na -- )
// 	Build a new dictionary name using the data at na.

// 	.word	_UNIQU
// _SNAME	.byte  3
// 	.ascii "$$,n"
// 	.p2align 2 	
SNAME:
	_NEST
	_ADR	DUPP			//  na na
	_ADR	CAT			//  ?null input
	_QBRAN	SNAM1
	_ADR	UNIQU			//  na
	_ADR	LAST			//  na last
	_ADR	AT			//  na la
	_ADR	COMMA			//  na
	_ADR	DUPP			//  na na
	_ADR	LAST			//  na na last
	_ADR	STORE			//  na , save na for vocabulary link
	_ADR	COUNT			//  na+1 count
	_ADR	PLUS			//  na+1+count
	_ADR	ALGND			//  word boundary
	_ADR	CPP
	_ADR	STORE			//  top of dictionary now
	_UNNEST
SNAM1:
	_ADR	STRQP
	.byte	7
	.ascii " name? "
	_BRAN	ABORT

//    $COMPILE	( a -- )
// 	Compile next word to code dictionary as a token or literal.

	.word	_UNIQU
_SCOMP:	.byte  8
	.ascii "$$COMPILE"
	.p2align 2 	
SCOMP:
	_NEST
	_ADR	NAMEQ
	_ADR	QDUP	// defined?
	_QBRAN	SCOM2
	_ADR	AT
	_DOLIT	IMEDD
	_ADR	ANDD	// immediate?
	_QBRAN	SCOM1
	_ADR	EXECU
	_UNNEST			// it's immediate, execute
SCOM1:
	_ADR	CALLC			// it's not immediate, compile
	_UNNEST	
SCOM2:
	_ADR	NUMBQ
	_QBRAN	SCOM3
	_ADR	LITER
	_UNNEST			// compile number as integer
SCOM3: // compilation abort 
	_ADR COLON_ABORT 
	_BRAN	ABORT			// error

// before aborting a compilation 
// reset HERE and LAST
// to previous values. 
COLON_ABORT:
	_NEST 
	_ADR LAST 
	_ADR AT 
	_ADR CELLM 
	_ADR DUPP 
	_ADR CPP  
	_ADR STORE 
	_ADR AT 
	_ADR LAST 
	_ADR STORE 
	_UNNEST 

//    OVERT	( -- )
// 	Link a new word into the current vocabulary.

	.word	_SCOMP
_OVERT:	.byte  5
	.ascii "OVERT"
	.p2align 2 	
OVERT:
	_NEST
	_ADR	LAST
	_ADR	AT
	_ADR	CNTXT
	_ADR	STORE
	_UNNEST

//    ; 	   ( -- )
// 	Terminate a colon definition.

	.word	_OVERT
_SEMIS:	.byte  IMEDD+COMPO+1
	.ascii ";"
	.p2align 2 	
SEMIS:
	_NEST
//	_DOLIT	_UNNEST
	_ADR	COMMA
	_ADR	LBRAC
	_ADR	OVERT
	_UNNEST

//    ]	   ( -- )
// 	Start compiling the words in the input stream.

	.word	_SEMIS
_RBRAC:	.byte  1
	.ascii "]"
	.p2align 2 	
RBRAC:
	_NEST
	_DOLIT	SCOMP
	_ADR	TEVAL
	_ADR	STORE
	_UNNEST

//    BL.W	( ca -- )
// 	Assemble a branch-link long instruction to ca.
// 	BL.W is split into 2 16 bit instructions with 11 bit address fields.

// 	.word	_RBRAC
// _CALLC	.byte  5
// 	.ascii "call,"
// 	.p2align 2 	
CALLC:
	_NEST 
	_UNNEST 


// 	:	( -- //  string> )
// 	Start a new colon definition using next word as its name.

	.word	_RBRAC
_COLON:	.byte  1
	.ascii ":"
	.p2align 2 	
COLON:
	_NEST
	_ADR	TOKEN
	_ADR	SNAME
	_DOLIT	NEST
	_ADR	COMMA
	_ADR	RBRAC
	_UNNEST

//    IMMEDIATE   ( -- )
// 	Make the last compiled word an immediate word.

	.word	_COLON
_IMMED:	.byte  9
	.ascii "IMMEDIATE"
	.p2align 2 	
IMMED:
	_NEST
	_DOLIT	IMEDD
	_ADR	LAST
	_ADR	AT
	_ADR	AT
	_ADR	ORR
	_ADR	LAST
	_ADR	AT
	_ADR	STORE
	_UNNEST

// **************************************************************************
//  Defining words

//    CONSTANT	( u -- //  string> )
// 	Compile a new constant.

	.word	_IMMED
_CONST:	.byte  8
	.ascii "CONSTANT"
	.p2align 2 	
CONST:
	_NEST 
	_ADR	TOKEN
	_ADR	SNAME
	_ADR	OVERT
	_DOLIT	NEST
	_ADR	COMMA
	_DOLIT	DOCON
	_ADR	CALLC
	_ADR	COMMA
	_UNNEST

//    CREATE	( -- //  string> )
// 	Compile a new array entry without allocating code space.

	.word	_CONST
_CREAT:	.byte  6
	.ascii "CREATE"
	.p2align 2 	
CREAT:
	_NEST 
	_ADR	TOKEN
	_ADR	SNAME
	_ADR	OVERT
	_DOLIT	NEST
	_ADR	COMMA
	_DOLIT	DOVAR
	_ADR	CALLC
	_UNNEST

//    VARIABLE	( -- //  string> )
// 	Compile a new variable initialized to 0.

	.word	_CREAT
_VARIA:	.byte  8
	.ascii "VARIABLE"
	.p2align 2 	
VARIA:
	_NEST
	_ADR	CREAT
	_DOLIT	0
	_ADR	COMMA
	_UNNEST

// **************************************************************************
//  Tools

//  TRACE ( f -- )
// enable or disable tracing 
	.word _VARIA 
_TRACE: .byte 5
	.ascii "TRACE"
	.p2align 2
TRACE: 
	_NEST 
	_ADR FTRACE 
	_ADR STORE 
	_UNNEST 

// TR_PRINT ( ca -- )
// print name from ca  
	.word _TRACE 
_DBG_PRT: .byte 7 
	.ascii "DBG_PRT"
DBG_PRT:
	_NEST 
	_ADR TNAME 
	_ADR QDUP 
	_QBRAN 1f 
	_ADR COUNT 
	_ADR TYPEE 
	_ADR CR
1:  	
	_UNNEST 

//    dm+	 ( a u -- a )
// 	Dump u bytes from , leaving a+u on the stack.

// 	.word	_TRACE
// _DMP	.byte  3
// 	.ascii "dm+"
// 	.p2align 2 	
DMP:
	_NEST
	_ADR	OVER
	_DOLIT	4
	_ADR	UDOTR			// display address
	_ADR	SPACE
	_ADR	TOR			// start count down loop
	_BRAN	PDUM2			// skip first pass
PDUM1:
  _ADR	DUPP
	_ADR	CAT
	_DOLIT	3
	_ADR	UDOTR			// display numeric data
	_ADR	ONEP			// increment address
PDUM2:
  _ADR	DONXT
	.word	PDUM1	// loop till done
	_UNNEST
	.p2align 2 
//    DUMP	( a u -- )
// 	Dump u bytes from a, in a formatted manner.

	.word	_VARIA
_DUMP:	.byte  4
	.ascii "DUMP"
	.p2align 2 	
DUMP:
	_NEST
	_ADR	BASE
	_ADR	AT
	_ADR	TOR
	_ADR	HEX			// save radix,set hex
	_DOLIT	16
	_ADR	SLASH			// change count to lines
	_ADR	TOR
	_BRAN	DUMP4			// start count down loop
DUMP1:
  _ADR	CR
	_DOLIT	16
	_ADR	DDUP
	_ADR	DMP			// display numeric
	_ADR	ROT
	_ADR	ROT
	_ADR	SPACE
	_ADR	SPACE
	_ADR	TYPEE			// display printable characters
DUMP4:
  _DONXT	DUMP1	// loop till done
DUMP3:
	_ADR	DROP
	_ADR	RFROM
	_ADR	BASE
	_ADR	STORE			// restore radix
	_UNNEST

//    .S	  ( ... -- ... )
// 	Display the contents of the data stack.

	.word	_DUMP
_DOTS:
	.byte  2
	.ascii ".S"
	.p2align 2 	
DOTS:
	_NEST
	_ADR	SPACE
	_ADR	DEPTH			// stack depth
	_ADR	TOR			// start count down loop
	_BRAN	DOTS2			// skip first pass
DOTS1:
	_ADR	RAT
	_ADR	PICK
	_ADR	DOT			// index stack, display contents
DOTS2:
	_DONXT	DOTS1	// loop till done
	_ADR	SPACE
	_UNNEST

//    >NAME	( ca -- na | F )
// 	Convert code address to a name address.

	.word	_DOTS
_TNAME:	.byte  5
	.ascii ">NAME"
	.p2align 2 	
TNAME:
	_NEST
	_ADR	TOR			//  
	_ADR	CNTXT			//  va
	_ADR	AT			//  na
TNAM1:
	_ADR	DUPP			//  na na
	_QBRAN	TNAM2	//  vocabulary end, no match
	_ADR	DUPP			//  na na
	_ADR	NAMET			//  na ca
	_ADR	RAT			//  na ca code
	_ADR	XORR			//  na f --
	_QBRAN	TNAM2
	_ADR	CELLM			//  la 
	_ADR	AT			//  next_na
	_BRAN	TNAM1
TNAM2:	
	_ADR	RFROM
	_ADR	DROP			//  0|na --
	_UNNEST			// 0

//    .ID	 ( na -- )
// 	Display the name at address.

	.word	_TNAME
_DOTID:	.byte  3
	.ascii ".ID"
	.p2align 2 	
DOTID:
	_NEST
	_ADR	QDUP			// if zero no name
	_QBRAN	DOTI1
	_ADR	COUNT
	_DOLIT	0x1F
	_ADR	ANDD			// mask lexicon bits
	_ADR	TYPEE
	_UNNEST			// display name string
DOTI1:
	_DOTQP	9," {noName}"
	_UNNEST

	.equ WANT_SEE, 0  // set to 1 if you want SEE 
.if WANT_SEE 
//    SEE	 ( -- //  string> )
// 	A simple decompiler.

	.word	_DOTID
_SEE:	.byte  3
	.ascii "SEE"
	.p2align 2 	
SEE:
	_NEST
	_ADR	TICK	//  ca --, starting address
	_ADR	CR	
	_DOLIT	20
	_ADR	TOR
SEE1:
	_ADR	CELLP			//  a
	_ADR	DUPP			//  a a
	_ADR	DECOMP		//  a
	_DONXT	SEE1
	_ADR	DROP
	_UNNEST

// 	DECOMPILE ( a -- )
// 	Convert code in a.  Display name of command or as data.

	.word	_SEE
_DECOM:	.byte  9
	.ascii "DECOMPILE"
	.p2align 2 
	
DECOMP:	
	_NEST
	_ADR	DUPP			//  a a
// 	_ADR	TOR			//  a
	_ADR	AT			//  a code
	_ADR	DUPP			//  a code code
	_DOLIT	0xF800D000 //0xF800F800
	_ADR	ANDD
	_DOLIT	0xF000D000 //0xF800F000
	_ADR	EQUAL			//  a code ?
	_ADR	INVER 
	_QBRAN	DECOM2	//  not a command
	//  a valid_code --, extract address and display name
	MOVW	R0,#0xFFE
	MOV	R4,R5
	LSL	R5,R5,#21		//  get bits 22-12
	ASR	R5,R5,#9		//  with sign extension
	LSR	R4,R4,#15		//  get bits 11-1
	AND	R4,R4,R0		//  retain only bits 11-1
	ORR	R5,R5,R4		//  get bits 22-1
	NOP
	_ADR	OVER			//  a offset a
	_ADR	PLUS			//  a target-4
	_ADR	CELLP			//  a target
	_ADR	TNAME			//  a na/0 --, is it a name?
	_ADR	QDUP			//  name address or zero
	_QBRAN	DECOM1
	_ADR	SPACE			//  a na
	_ADR	DOTID			//  a --, display name
// 	_ADR	RFROM			//  a
	_ADR	DROP
	_UNNEST
DECOM1:	// _ADR	RFROM		//  a
	_ADR	AT			//  data
	_ADR	UDOT			//  display data
	_UNNEST
DECOM2:
	_ADR	UDOT
// 	_ADR	RFROM
	_ADR	DROP
	_UNNEST

//    WORDS	( -- )
// 	Display the names in the context vocabulary.

	.word	_DECOM
.else 
	.word _DOTID 
.endif 
_WORDS:	.byte  5
	.ascii "WORDS"
	.p2align 2 	
WORDS:
	_NEST
	_ADR	CR
	_ADR	CNTXT
	_ADR	AT			// only in context
WORS1:
	_ADR	QDUP			// ?at end of list
	_QBRAN	WORS2
	_ADR	DUPP
	_ADR	SPACE
	_ADR	DOTID			// display a name
	_ADR	CELLM
	_ADR	AT
	_BRAN	WORS1
WORS2:
	_UNNEST

// **************************************************************************
//  cold start

//    VER	 ( -- n )
// 	Return the version number of this implementation.

// 	.word	_WORDS
// _VERSN	.byte  3
// 	.ascii "VER"
// 	.p2align 2 	
VERSN:
	_NEST
	_DOLIT	VER*256+EXT
	_UNNEST

//    hi	  ( -- )
// 	Display the sign-on message of eForth.

	.word	_WORDS
_HI:	.byte  2
	.ascii "HI"
	.p2align 2 	
HI:
	_NEST
	_ADR	CR	// initialize I/O
	_DOTQP	23, "blue pill stm32eForth v" 
	_ADR	BASE
	_ADR	AT
	_ADR	HEX	// save radix
	_ADR	VERSN
	_ADR	BDIGS
	_ADR	DIG
	_ADR	DIG
	_DOLIT	'.'
	_ADR	HOLD
	_ADR	DIGS
	_ADR	EDIGS
	_ADR	TYPEE	// format version number
	_ADR	BASE
	_ADR	STORE
	_ADR	CR
	_UNNEST			// restore radix

//    COLD	( -- )
// 	The high level cold start sequence.

	.word	_HI
LASTN:	.byte  4
	.ascii "COLD"
	.p2align 2,0	
COLD:
//  Initiate Forth registers
	mov r3,#UPP&0xffff
	movt r3,#UPP>>16 
	add R2,R3,#RPP&0xffff	// Forth return stack
	add R1,R3,#SPP&0xffff // Forth data stack
	eor R5,R5,R5			//  tos=0
	ldr R0,=COLD1 
	_NEXT
COLD1:
	_DOLIT  0 
	_ADR ULED // turn off user LED 
	_DOLIT	UZERO
	_DOLIT	UPP
	_DOLIT	ULAST-UZERO
	_ADR	MOVE 			// initialize user area
	_ADR	PRESE			// initialize stack and TIB
//	_DOLIT 0		// check if user image saved in slot 0 
//	_ADR IMGQ 
//	_QBRAN 1f
//	_DOLIT 0
//	_ADR	LOAD_IMG 
1:
//_DOLIT 1
//_ADR TRACE 
	_ADR	TBOOT
	_ADR	ATEXE			// application boot
	_ADR	OVERT
	_BRAN	QUIT			// start interpretation
COLD2:
	.p2align 2 	
CTOP:
	.word	0XFFFFFFFF		//  keep CTOP even

	.section .text.user, "a", %progbits 
	.p2align 10 
USER_SPACE: // save user image here.  
	.word 0XFFFFFFFF

  .end 
