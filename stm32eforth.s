/* 
****************************************************
*  STM32eForth version 7.20
*  Adapted to blue pill board by Picatout
*  date: 2020-11-22
*  IMPLEMENTATION NOTES:

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
*	BX	LR 	(_NEXT)
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
*	.align 2 to at91sam7x256
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
  .cpu cortex-m3
  .fpu softvfp
  .thumb

  .include "board/blue-pill/stm32f103.inc"

/* blue pill specific constants */ 
  .equ LED_GPIO, GPIOC_BASE_ADR
  .equ LED_PIN, 13
  .equ UART, USART1_BASE_ADR 

/* stm32f103c8t6 specific constants */
.equ SPP 	,	0x20004E80	/*top of data stack (SP0) */
.equ TIBB ,	0x20004E80	/*terminal input buffer (TIB) */
.equ RPP 	,	0x20004F80	/*top of return stack (RP0) */
.equ UPP 	,	0x20000000	/*start of user area (UP0) */
.equ DTOP ,	0x20000100	/*start of usable RAM area (HERE) */
.equ DEND , 0x20004E00  /*usable RAM end */

/*************************************
   system variables offset from UPP
*************************************/
  .equ TICKS_OFS, 4  // millseconds counter
  .equ TIMER_OFS, 8  // count down timer
  .equ BOOT_OFS, 12  // boot program address
  .equ BASE_OFS, 16  // numeric conversion base 
  .equ TMP_OFS, 20   // temporary variable
  .equ SPAN_OFS, 24  // character count received by expect  
  .equ TOIN_OFS, 28  // >IN  parse pointer in TIB
  .equ NTIB_OFS, 32  // #TIB  characters in TIB 
  .equ TIB_OFS, 36   // TIB buffer address 
  .equ EVAL_OFS, 40  // eval|compile vector 
  .equ HLD_OFS, 44   // hold pointer 
  .equ CTXT_OFS, 48  // context pointer 
  .equ FLSH_CTOP_OFS, 52  // flash free dictionary address 
  .equ RAM_CTOP_OFS, 56  // ram free dictionary address
  .equ LASTN_OFS, 60     // last word in dictionary link nfa 

/*************************************
    reserve space for system variables 
*************************************/
  .section .bss 
system_vars: .space 0x100



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

/*************************************
*   interrupt service vectors table 
**************************************/
   .section  .isr_vector,"a",%progbits
  .type  isr_vectors, %object

isr_vectors:
  .word   _rstack          /* return stack address */
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

/*****************************************************
* default isr handler called on unexpected interrupt
*****************************************************/
    .section  .text.default_handler,"ax",%progbits

  .type default_handler, %function
  .align 2 
  .global default_handler
default_handler:
	_DOLIT 
	.word -1
	BL ULED 
	BL	CR	// new line
	BL	DOTQP
	.byte	16
	.ascii " exception hatl!"	
	.align 2 
Infinite_Loop:
  b  Infinite_Loop
  .size  default_handler, .-default_handler



/*********************************
	system milliseconds counter
*********************************/	
  .type systick_handler, %function
  .align 2 
  .global systick_handler
systick_handler:
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
    .section  .text.reset_handler
  .type  reset_handler, %function
  .align 2 
  .global reset_handler
reset_handler:
/* zero RAM */
	mov r0,#0
	movt r0,#0x2000
	mov r1,#0
	mov r2,#0x5000 // RAM size 
zero_loop:
	str r1,[r0],#4
	subs r2,#4
	bne zero_loop		
	bl	init_devices	 	/* RCC, GPIOs, USART */
	bl	UNLOCK			/* unlock flash memory */
	b COLD 

  .type init_devices, %function
  .align 2 
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


/********************
* Version control
*******************/
.equ VER ,	0x01	/*major release version */
.equ EXT ,	0x01	/*minor extension */

/* Constants */

// .equ RAMOFFSET ,	0x20000000	;remap
// .equ MAPOFFSET ,	0x08000000	;remap
.equ RAMOFFSET  ,	0x00000000	/* absolute */
.equ MAPOFFSET  ,	0x00000000	/* absolute */

.equ COMPO ,	0x040	/*lexicon compile only */ 
.equ IMEDD ,	0x080	/*lexicon immediate bit */
.equ MASKK ,	0x0FFFFFF1F	/*lexicon bit mask, allowed for Chineze character */

.equ CELLL ,	4	/*size of a cell */
.equ BASEE ,	16	/*default radix */
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
	.align 2   	
  

UZERO:
	.word 0  			/*Reserved */
	.word 0      /* system Ticks */
    .word 0     /* delay timer */
	.word HI  	/*'BOOT */
	.word BASEE 	/*BASE */
	.word 0			/*tmp */
	.word 0			/*SPAN */
	.word 0			/*>IN */
	.word 0			/*#TIB */
	.word TIBB	/*TIB */
	.word INTER	/*'EVAL */
	.word 0			/*HLD */
	.word LASTN	/*CONTEXT */
	.word CTOP	/*FLASH */
	.word DTOP	/* start adress of RAM dictionary RAM */
	.word LASTN	/*LAST */
  .word 0,0			/*reserved */
ULAST:

 

/***********************************
//  Start of Forth dictionary
***********************************/

	.align 2 

// PAUSE ( u -- ) 
// suspend execution for u milliseconds
	.word 0
_PAUSE: .byte 5
	.ascii "PAUSE"
	.align 2
PAUSE:
	_NEST 
	BL TIMER 
	BL STORE 
PAUSE_LOOP:
	BL TIMER 
	BL AT 
	BL QBRAN 
	.word PAUSE_EXIT 
	BL BRAN 
	.word PAUSE_LOOP 
PAUSE_EXIT: 		
	_UNNEST 

//  ULED ( T|F -- )
// control user LED, -1 ON, 0 OFF  
	.word _PAUSE 
_ULED: .byte 4
	.ascii "ULED"
	.align 2
	.type ULED, %function 
ULED:
	mov r6,#(1<<LED_PIN)
	mov r4,#LED_GPIO&0xffff
	movt r4,#LED_GPIO>>16
	movs r0,r5 
	_POP
	beq ULED_OFF 
	str r6,[r4,#GPIO_BRR]
	_NEXT 
ULED_OFF:
	str r6,[r4,#GPIO_BSRR]
	_NEXT 
	
//    ?RX	 ( -- c T | F )
// 	Return input character and true, or a false if no input.
	.word	_ULED-MAPOFFSET
_QRX:	.byte   4
	.ascii "?KEY"
	.align 2 
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
	.align 2 

//    TX!	 ( c -- )
// 	Send character c to the output device.

	.word	_QRX-MAPOFFSET
_TXSTO:	.byte 4
	.ascii "EMIT"
	.align 2 	
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


	
// **************************************************************************
//  The kernel

//    NOP	( -- )
// 	do nothing.

	.word	_TXSTO-MAPOFFSET
_NOP:	.byte   3
	.ascii "NOP"
	.align 2 	
NOP:
	_NEXT
 

//    doLIT	( -- w )
// 	Push an inline literal.

// 	.word	_NOP-MAPOFFSET
// _LIT	.byte   COMPO+5
// 	.ascii "doLIT"
// 	.align 2 	
DOLIT:
	_PUSH				//  store R5 on data stack
	BIC	LR,LR,#1		//  clear b0 in LR
	LDR	R5,[LR],#4		//  get literal at word boundary
	ORR	LR,LR,#1		//  aet b0 in LR
	_NEXT

//    EXECUTE	( ca -- )
// 	Execute the word at ca.

	.word	_NOP-MAPOFFSET
_EXECU:	.byte   7
	.ascii "EXECUTE"
	.align 2 	
EXECU:
	ORR	R4,R5,#1		//  b0=1 
	_POP
	BX	R4

//    next	( -- ) counter on R:
// 	Run time code for the single index loop.
// 	: next ( -- ) \ hilevel model
// 	 r> r> dup if 1 - >r @ >r exit then drop cell+ >r // 

// 	.word	_EXECU-MAPOFFSET
// _DONXT	.byte   COMPO+4
// 	.ascii "next"
// 	.align 2 	
DONXT:
	LDR	R4,[R2]
	MOVS	R4,R4
	BNE	NEXT1
	/* loop done */
	ADD	R2,R2,#4 // drop counter 
	ADD	LR,LR,#4 // skip after loop 
	_NEXT
NEXT1:
	/* decrement loop counter */
	SUB	R4,R4,#1
	STR	R4,[R2]
	LDR	LR,[LR,#-1]	//  handle b0 in LR 
	ORR	LR,LR,#1 // begining of loop
	_NEXT

//    ?branch	( f -- )
// 	Branch if flag is zero.

// 	.word	_DONXT-MAPOFFSET
// _QBRAN	.byte   COMPO+7
// 	.ascii "?branch"
// 	.align 2 	
QBRAN:
	MOVS	R4,R5
	_POP
	BNE	QBRAN1
	LDR	LR,[LR,#-1]
	ORR LR,LR,#1
	_NEXT
QBRAN1:
 	ADD	LR,LR,#4
	_NEXT

//    branch	( -- )
// 	Branch to an inline address.

// 	.word	_QBRAN-MAPOFFSET
// _BRAN	.byte   COMPO+6
// 	.ascii "branch"
// 	.align 2 	
BRAN:
	LDR	LR,[LR,#-1]
	ORR	LR,LR,#1
	_NEXT

//    EXIT	(  -- )
// 	Exit the currently executing command.

	.word	_EXECU-MAPOFFSET
_EXIT:	.byte   4
	.ascii "EXIT"
	.align 2 	
EXIT:
	_UNNEST

//    !	   ( w a -- )
// 	Pop the data stack to memory.

	.word	_EXIT-MAPOFFSET
_STORE:	.byte   1
	.ascii "!"
	.align 2 	
STORE:
	LDR	R4,[R1],#4
	STR	R4,[R5]
	_POP
	_NEXT

//    @	   ( a -- w )
// 	Push memory location to the data stack.

	.word	_STORE-MAPOFFSET
_AT:	.byte   1
	.ascii "@"
	.align 2 	
AT:
	LDR	R5,[R5]
	_NEXT

//    C!	  ( c b -- )
// 	Pop the data stack to byte memory.

	.word	_AT-MAPOFFSET
_CSTOR:	.byte   2
	.ascii "C!"
	.align 2 	
CSTOR:
	LDR	R4,[R1],#4
	STRB	R4,[R5]
	_POP
	_NEXT

//    C@	  ( b -- c )
// 	Push byte memory location to the data stack.

	.word	_CSTOR-MAPOFFSET
_CAT:	.byte   2
	.ascii "C@"
	.align 2 	
CAT:
	LDRB	R5,[R5]
	_NEXT

//    R>	  ( -- w )
// 	Pop the return stack to the data stack.

	.word	_CAT-MAPOFFSET
_RFROM:	.byte   2
	.ascii "R>"
	.align 2 	
RFROM:
	_PUSH
	LDR	R5,[R2],#4
	_NEXT

//    R@	  ( -- w )
// 	Copy top of return stack to the data stack.

	.word	_RFROM-MAPOFFSET
_RAT:	.byte   2
	.ascii "R@"
	.align 2 	
RAT:
	_PUSH
	LDR	R5,[R2]
	_NEXT

//    >R	  ( w -- )
// 	Push the data stack to the return stack.

	.word	_RAT-MAPOFFSET
_TOR:	.byte   COMPO+2
	.ascii ">R"
	.align 2 	
TOR:
	STR	R5,[R2,#-4]!
	_POP
	_NEXT

//    SP@	 ( -- a )
// 	Push the current data stack pointer.

	.word	_TOR-MAPOFFSET
_SPAT:	.byte   3
	.ascii "SP@"
	.align 2 	
SPAT:
	_PUSH
	MOV	R5,R1
	_NEXT

//    DROP	( w -- )
// 	Discard top stack item.

	.word	_SPAT-MAPOFFSET
_DROP:	.byte   4
	.ascii "DROP"
	.align 2 	
DROP:
	_POP
	_NEXT

//    DUP	 ( w -- w w )
// 	Duplicate the top stack item.

	.word	_DROP-MAPOFFSET
_DUPP:	.byte   3
	.ascii "DUP"
	.align 2 	
DUPP:
	_PUSH
	_NEXT

//    SWAP	( w1 w2 -- w2 w1 )
// 	Exchange top two stack items.

	.word	_DUPP-MAPOFFSET
_SWAP:	.byte   4
	.ascii "SWAP"
	.align 2 	
SWAP:
	LDR	R4,[R1]
	STR	R5,[R1]
	MOV	R5,R4
	_NEXT

//    OVER	( w1 w2 -- w1 w2 w1 )
// 	Copy second stack item to top.

	.word	_SWAP-MAPOFFSET
_OVER:	.byte   4
	.ascii "OVER"
	.align 2 	
OVER:
	_PUSH
	LDR	R5,[R1,#4]
	_NEXT

//    0<	  ( n -- t )
// 	Return true if n is negative.

	.word	_OVER-MAPOFFSET
_ZLESS:	.byte   2
	.ascii "0<"
	.align 2 	
ZLESS:
	MOV	R4,#0
	ADD	R5,R4,R5,ASR #32
	_NEXT

//    AND	 ( w w -- w )
// 	Bitwise AND.

	.word	_ZLESS-MAPOFFSET
_ANDD:	.byte   3
	.ascii "AND"
	.align 2 	
ANDD:
	LDR	R4,[R1],#4
	AND	R5,R5,R4
	_NEXT

//    OR	  ( w w -- w )
// 	Bitwise inclusive OR.

	.word	_ANDD-MAPOFFSET
_ORR:	.byte   2
	.ascii "OR"
	.align 2 	
ORR:
	LDR	R4,[R1],#4
	ORR	R5,R5,R4
	_NEXT

//    XOR	 ( w w -- w )
// 	Bitwise exclusive OR.

	.word	_ORR-MAPOFFSET
_XORR:	.byte   3
	.ascii "XOR"
	.align 2 	
XORR:
	LDR	R4,[R1],#4
	EOR	R5,R5,R4
	_NEXT

//    UM+	 ( w w -- w cy )
// 	Add two numbers, return the sum and carry flag.

	.word	_XORR-MAPOFFSET
_UPLUS:	.byte   3
	.ascii "UM+"
	.align 2 	
UPLUS:
	LDR	R4,[R1]
	ADDS	R4,R4,R5
	MOV	R5,#0
	ADC	R5,R5,#0
	STR	R4,[R1]
	_NEXT

//    RSHIFT	 ( w # -- w )
// 	arithmetic Right shift # bits.

	.word	_UPLUS-MAPOFFSET
_RSHIFT:	.byte   6
	.ascii "RSHIFT"
	.align 2 	
RSHIFT:
	LDR	R4,[R1],#4
	MOV	R5,R4,ASR R5
	_NEXT

//    LSHIFT	 ( w # -- w )
// 	Right shift # bits.

	.word	_RSHIFT-MAPOFFSET
_LSHIFT:	.byte   6
	.ascii "LSHIFT"
	.align 2 	
LSHIFT:
	LDR	R4,[R1],#4
	MOV	R5,R4,LSL R5
	_NEXT

//    +	 ( w w -- w )
// 	Add.

	.word	_LSHIFT-MAPOFFSET
_PLUS:	.byte   1
	.ascii "+"
	.align 2 	
PLUS:
	LDR	R4,[R1],#4
	ADD	R5,R5,R4
	_NEXT

//    -	 ( w w -- w )
// 	Subtract.

	.word	_PLUS-MAPOFFSET
_SUBB:	.byte   1
	.ascii "-"
	.align 2 	
SUBB:
	LDR	R4,[R1],#4
	RSB	R5,R5,R4
	_NEXT

//    *	 ( w w -- w )
// 	Multiply.

	.word	_SUBB-MAPOFFSET
_STAR:	.byte   1
	.ascii "*"
	.align 2 	
STAR:
	LDR	R4,[R1],#4
	MUL	R5,R4,R5
	_NEXT

//    UM*	 ( w w -- ud )
// 	Unsigned multiply.

	.word	_STAR-MAPOFFSET
_UMSTA:	.byte   3
	.ascii "UM*"
	.align 2 	
UMSTA:
	LDR	R4,[R1]
	UMULL	R6,R7,R5,R4
	STR	R6,[R1]
	MOV	R5,R7
	_NEXT

//    M*	 ( w w -- d )
// 	signed multiply.

	.word	_UMSTA-MAPOFFSET
_MSTAR:	.byte   2
	.ascii "M*"
	.align 2 	
MSTAR:
	LDR	R4,[R1]
	SMULL	R6,R7,R5,R4
	STR	R6,[R1]
	MOV	R5,R7
	_NEXT

//    1+	 ( w -- w+1 )
// 	Add 1.

	.word	_MSTAR-MAPOFFSET
_ONEP:	.byte   2
	.ascii "1+"
	.align 2 	
ONEP:
	ADD	R5,R5,#1
	_NEXT

//    1-	 ( w -- w-1 )
// 	Subtract 1.

	.word	_ONEP-MAPOFFSET
_ONEM:	.byte   2
	.ascii "1-"
	.align 2 	
ONEM:
	SUB	R5,R5,#1
	_NEXT

//    2+	 ( w -- w+2 )
// 	Add 1.

	.word	_ONEM-MAPOFFSET
_TWOP:	.byte   2
	.ascii "2+"
	.align 2 	
TWOP:
	ADD	R5,R5,#2
	_NEXT

//    2-	 ( w -- w-2 )
// 	Subtract 2.

	.word	_TWOP-MAPOFFSET
_TWOM:	.byte   2
	.ascii "2-"
	.align 2 	
TWOM:
	SUB	R5,R5,#2
	_NEXT

//    CELL+	( w -- w+4 )
// 	Add CELLL.

	.word	_TWOM-MAPOFFSET
_CELLP:	.byte   5
	.ascii "CELL+"
	.align 2 	
CELLP:
	ADD	R5,R5,#CELLL
	_NEXT

//    CELL-	( w -- w-4 )
// 	Subtract CELLL.

	.word	_CELLP-MAPOFFSET
_CELLM:	.byte   5
	.ascii "CELL-"
	.align 2 	
CELLM:
	SUB	R5,R5,#CELLL
	_NEXT
 
//    BL	( -- 32 )
// 	Blank (ASCII space).

	.word	_CELLM-MAPOFFSET
_BLANK:	.byte   2
	.ascii "BL"
	.align 2 	
BLANK:
	_PUSH
	MOV	R5,#32
	_NEXT

//    CELLS	( w -- w*4 )
// 	Multiply 4.

	.word	_BLANK-MAPOFFSET
_CELLS:	.byte   5
	.ascii "CELLS"
	.align 2 	
CELLS:
	MOV	R5,R5,LSL#2
	_NEXT

//    CELL/	( w -- w/4 )
// 	Divide by 4.

	.word	_CELLS-MAPOFFSET
_CELLSL:	.byte   5
	.ascii "CELL/"
	.align 2 	
CELLSL:
	MOV	R5,R5,ASR#2
	_NEXT

//    2*	( w -- w*2 )
// 	Multiply 2.

	.word	_CELLSL-MAPOFFSET
_TWOST:	.byte   2
	.ascii "2*"
	.align 2 	
TWOST:
	MOV	R5,R5,LSL#1
	_NEXT

//    2/	( w -- w/2 )
// 	Divide by 2.

	.word	_TWOST-MAPOFFSET
_TWOSL:	.byte   2
	.ascii "2/"
	.align 2 	
TWOSL:
	MOV	R5,R5,ASR#1
	_NEXT

//    ?DUP	( w -- w w | 0 )
// 	Conditional duplicate.

	.word	_TWOSL-MAPOFFSET
_QDUP:	.byte   4
	.ascii "?DUP"
	.align 2 	
QDUP:
	MOVS	R4,R5
	IT NE 
    STRNE	R5,[R1,#-4]!
	_NEXT

//    ROT	( w1 w2 w3 -- w2 w3 w1 )
// 	Rotate top 3 items.

	.word	_QDUP-MAPOFFSET
_ROT:	.byte   3
	.ascii "ROT"
	.align 2 	
ROT:
	LDR	R4,[R1]  // r4=w2 
	STR	R5,[R1]  // w3 replace w2 
	LDR	R5,[R1,#4] // w1 replace w3 
	STR	R4,[R1,#4] // w2 rpelace w1 
	_NEXT

//    2DROP	( w1 w2 -- )
// 	Drop top 2 items.

	.word	_ROT-MAPOFFSET
_DDROP:	.byte   5
	.ascii "2DROP"
	.align 2 	
DDROP:
	_POP
	_POP
	_NEXT

//    2DUP	( w1 w2 -- w1 w2 w1 w2 )
// 	Duplicate top 2 items.

	.word	_DDROP-MAPOFFSET
_DDUP:	.byte   4
	.ascii "2DUP"
	.align 2 	
DDUP:
	LDR	R4,[R1] // r4=w1
	STR	R5,[R1,#-4]! // push w2  
	STR	R4,[R1,#-4]! // push w1 
	_NEXT

//    D+	( d1 d2 -- d3 )
// 	Add top 2 double numbers.

	.word	_DDUP-MAPOFFSET
_DPLUS:	.byte   2
	.ascii "D+"
	.align 2 	
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

	.word	_DPLUS-MAPOFFSET
_INVER:	.byte   3
	.ascii "NOT"
	.align 2 	
INVER:
	MVN	R5,R5
	_NEXT

//    NEGATE	( w -- -w )
// 	2's complement.

	.word	_INVER-MAPOFFSET
_NEGAT:	.byte   6
	.ascii "NEGATE"
	.align 2 	
NEGAT:
	RSB	R5,R5,#0
	_NEXT

//    ABS	 ( w -- |w| )
// 	Absolute.

	.word	_NEGAT-MAPOFFSET
_ABSS:	.byte   3
	.ascii "ABS"
	.align 2 	
ABSS:
	TST	R5,#0x80000000
	IT NE
    RSBNE   R5,R5,#0
	_NEXT

//    =	 ( w w -- t )
// 	Equal?

	.word	_ABSS-MAPOFFSET
_EQUAL:	.byte   1
	.ascii "="
	.align 2 	
EQUAL:
	LDR	R4,[R1],#4
	CMP	R5,R4
	ITE EQ 
    MVNEQ	R5,#0
	MOVNE	R5,#0
	_NEXT

//    U<	 ( w w -- t )
// 	Unsigned equal?

	.word	_EQUAL-MAPOFFSET
_ULESS:	.byte   2
	.ascii "U<"
	.align 2 	
ULESS:
	LDR	R4,[R1],#4
	CMP	R4,R5
	ITE CC 
	MVNCC	R5,#0
	MOVCS	R5,#0
	_NEXT

//    <	( w w -- t )
// 	Less?

	.word	_ULESS-MAPOFFSET
_LESS:	.byte   1
	.ascii "<"
	.align 2 	
LESS:
	LDR	R4,[R1],#4
	CMP	R4,R5
    ITE LT
	MVNLT	R5,#0
	MOVGE	R5,#0
	_NEXT

//    >	( w w -- t )
// 	greater?

	.word	_LESS-MAPOFFSET
_GREAT:	.byte   1
	.ascii ">"
	.align 2 	
GREAT:
	LDR	R4,[R1],#4
	CMP	R4,R5
	ITE GT
    MVNGT	R5,#0
	MOVLE	R5,#0
	_NEXT

//    MAX	 ( w w -- max )
// 	Leave maximum.

	.word	_GREAT-MAPOFFSET
_MAX:	.byte   3
	.ascii "MAX"
	.align 2 	
MAX:
	LDR	R4,[R1],#4
	CMP	R4,R5
	IT GT 
	MOVGT	R5,R4
	_NEXT

//    MIN	 ( w w -- min )
// 	Leave minimum.

	.word	_MAX-MAPOFFSET
_MIN:	.byte   3
	.ascii "MIN"
	.align 2 	
MIN:
	LDR	R4,[R1],#4
	CMP	R4,R5
	IT LT
	MOVLT	R5,R4
	_NEXT

//    +!	 ( w a -- )
// 	Add to memory.

	.word	_MIN-MAPOFFSET
_PSTOR:	.byte   2
	.ascii "+!"
	.align 2 	
PSTOR:
	LDR	R4,[R1],#4
	LDR	R6,[R5]
	ADD	R6,R6,R4
	STR	R6,[R5]
	_POP
	_NEXT

//    2!	 ( d a -- )
// 	Store double number.

	.word	_PSTOR-MAPOFFSET
_DSTOR:	.byte   2
	.ascii "2!"
	.align 2 	
DSTOR:
	LDR	R4,[R1],#4
	LDR	R6,[R1],#4
	STR	R4,[R5],#4
	STR	R6,[R5]
	_POP
	_NEXT

//    2@	 ( a -- d )
// 	Fetch double number.

	.word	_DSTOR-MAPOFFSET
_DAT:	.byte   2
	.ascii "2@"
	.align 2 	
DAT:
	LDR	R4,[R5,#4]
	STR	R4,[R1,#-4]!
	LDR	R5,[R5]
	_NEXT

//    COUNT	( b -- b+1 c )
// 	Fetch length of string.

	.word	_DAT-MAPOFFSET
_COUNT:	.byte   5
	.ascii "COUNT"
	.align 2 	
COUNT:
	LDRB	R4,[R5],#1
	_PUSH
	MOV	R5,R4
	_NEXT

//    DNEGATE	( d -- -d )
// 	Negate double number.

	.word	_COUNT-MAPOFFSET
_DNEGA:	.byte   7
	.ascii "DNEGATE"
	.align 2 	
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

// 	.word	_DNEGA-MAPOFFSET
// _DOVAR	.byte  COMPO+5
// 	.ascii "doVAR"
// 	.align 2 	
DOVAR:
	_PUSH
	SUB	R5,LR,#1		//  CLEAR B0
	_UNNEST

//    doCON	( -- a ) 
// 	Run time routine for CONSTANT.

// 	.word	_DOVAR-MAPOFFSET
// _DOCON	.byte  COMPO+5
// 	.ascii "doCON"
// 	.align 2 	
DOCON:
	_PUSH
	LDR	R5,[LR,#-1]	//  clear b0
	_UNNEST

/***********************
  system variables 
***********************/
  
//  MSEC ( -- a)
// return address of milliseconds counter
  .word _DNEGA-MAPOFFSET 
_MSEC: .byte 4
  .ascii "MSEC"
  .align 2 
MSEC:
  _PUSH
  ADD R5,R3,#TICKS_OFS
  _NEXT 

// TIMER ( -- a )
  .word _MSEC-MAPOFFSET
_TIMER:  .byte 5
  .ascii "TIMER"
  .align 2 
TIMER:
  _PUSH 
  ADD R5,53,#TIMER_OFS
  _NEXT

//    'BOOT	 ( -- a )
// 	Application.

	.word	_TIMER-MAPOFFSET
_TBOOT:	.byte   5
	.ascii "'BOOT"
	.align 2 	
TBOOT:
	_PUSH
	ADD	R5,R3,#BOOT_OFS 
	_NEXT
	
//    BASE	( -- a )
// 	Storage of the radix base for numeric I/O.

	.word	_TBOOT-MAPOFFSET
_BASE:	.byte   4
	.ascii "BASE"
	.align 2 	
BASE:
	_PUSH
	ADD	R5,R3,#BASE_OFS
	_NEXT

//    tmp	 ( -- a )
// 	A temporary storage location used in parse and find.

// 	.word	_BASE-MAPOFFSET
// _TEMP	.byte   COMPO+3
// 	.ascii "tmp"
// 	.align 2 	
TEMP:
	_PUSH
	ADD	R5,R3,#TMP_OFS
	_NEXT

//    SPAN	( -- a )
// 	Hold character count received by EXPECT.

	.word	_BASE-MAPOFFSET
_SPAN:	.byte   4
	.ascii "SPAN"
	.align 2 	
SPAN:
	_PUSH
	ADD	R5,R3,#SPAN_OFS
	_NEXT

//    >IN	 ( -- a )
// 	Hold the character pointer while parsing input stream.

	.word	_SPAN-MAPOFFSET
_INN:	.byte   3
	.ascii ">IN"
	.align 2 	
INN:
	_PUSH
	ADD	R5,R3,#TOIN_OFS
	_NEXT

//    #TIB	( -- a )
// 	Hold the current count and address of the terminal input buffer.

	.word	_INN-MAPOFFSET
_NTIB:	.byte   4
	.ascii "#TIB"
	.align 2 	
NTIB:
	_PUSH
	ADD	R5,R3,#NTIB_OFS
	_NEXT

//    'EVAL	( -- a )
// 	Execution vector of EVAL.

	.word	_NTIB-MAPOFFSET
_TEVAL:	.byte   5
	.ascii "'EVAL"
	.align 2 	
TEVAL:
	_PUSH
	ADD	R5,R3,#EVAL_OFS
	_NEXT

//    HLD	 ( -- a )
// 	Hold a pointer in building a numeric output string.

	.word	_TEVAL-MAPOFFSET
_HLD:	.byte   3
	.ascii "HLD"
	.align 2 	
HLD:
	_PUSH
	ADD	R5,R3,#HLD_OFS
	_NEXT

//    CONTEXT	( -- a )
// 	A area to specify vocabulary search order.

	.word	_HLD-MAPOFFSET
_CNTXT:	.byte   7
	.ascii "CONTEXT"
	.align 2 	
CNTXT:
CRRNT:
	_PUSH
	ADD	R5,R3,#CTXT_OFS
	_NEXT

//    CP	( -- a )
// 	Point to top name in RAM vocabulary.

	.word	_CNTXT-MAPOFFSET
_CP:	.byte   2
	.ascii "CP"
	.align 2 	
CPP:
	_PUSH
	ADD	R5,R3,#RAM_CTOP_OFS
	_NEXT

//   FCP ( -- a )
//  Point ot top of Flash dictionary
	.word _CP-MAPOFFSET
_FCPP: .byte 4 
	.ascii "FCPP"
	.align 2 
FCPP: 
	_PUSH 
	ADD R5,R3,#FLSH_CTOP_OFS 
	_NEXT 

//    LAST	( -- a )
// 	Point to the last name in the name dictionary.

	.word	_FCPP-MAPOFFSET
_LAST:	.byte   4
	.ascii "LAST"
	.align 2 	
LAST:
	_PUSH
	ADD	R5,R3,#LASTN_OFS
	_NEXT

// **************************************************************************
//  Common functions

//    WITHIN	( u ul uh -- t )
// 	Return true if u is within the range of ul and uh.

	.word	_LAST-MAPOFFSET
_WITHI:	.byte   6
	.ascii "WITHIN"
	.align 2 	
WITHI:
	_NEST
	BL	OVER
	BL	SUBB
	BL	TOR
	BL	SUBB
	BL	RFROM
	BL	ULESS
	_UNNEST

//  Divide

//    UM/MOD	( udl udh u -- ur uq )
// 	Unsigned divide of a double by a single. Return mod and quotient.

	.word	_WITHI-MAPOFFSET
_UMMOD:	.byte   6
	.ascii "UM/MOD"
	.align 2 	
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
	ADCS	R6,R6,#1
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

	.word	_UMMOD-MAPOFFSET
_MSMOD:	.byte  5
	.ascii "M/MOD"
	.align 2 	
MSMOD:	
	_NEST
	BL	DUPP
	BL	ZLESS
	BL	DUPP
	BL	TOR
	BL	QBRAN
	.word	MMOD1-MAPOFFSET
	BL	NEGAT
	BL	TOR
	BL	DNEGA
	BL	RFROM
MMOD1:
  BL	TOR
	BL	DUPP
	BL	ZLESS
	BL	QBRAN
	.word	MMOD2-MAPOFFSET
	BL	RAT
	BL	PLUS
MMOD2:
  BL	RFROM
	BL	UMMOD
	BL	RFROM
	BL	QBRAN
	.word	MMOD3-MAPOFFSET
	BL	SWAP
	BL	NEGAT
	BL	SWAP
MMOD3:   
	_UNNEST

//    /MOD	( n n -- r q )
// 	Signed divide. Return mod and quotient.

	.word	_MSMOD-MAPOFFSET
_SLMOD:	.byte   4
	.ascii "/MOD"
	.align 2 	
SLMOD:
	_NEST
	BL	OVER
	BL	ZLESS
	BL	SWAP
	BL	MSMOD
	_UNNEST

//    MOD	 ( n n -- r )
// 	Signed divide. Return mod only.

	.word	_SLMOD-MAPOFFSET
_MODD:	.byte  3
	.ascii "MOD"
	.align 2 	
MODD:
	_NEST
	BL	SLMOD
	BL	DROP
	_UNNEST

//    /	   ( n n -- q )
// 	Signed divide. Return quotient only.

	.word	_MODD-MAPOFFSET
_SLASH:	.byte  1
	.ascii "/"
	.align 2 	
SLASH:
	_NEST
	BL	SLMOD
	BL	SWAP
	BL	DROP
	_UNNEST

//    */MOD	( n1 n2 n3 -- r q )
// 	Multiply n1 and n2, then divide by n3. Return mod and quotient.

	.word	_SLASH-MAPOFFSET
_SSMOD:	.byte  5
	.ascii "*/MOD"
	.align 2 	
SSMOD:
	_NEST
	BL	TOR
	BL	MSTAR
	BL	RFROM
	BL	MSMOD
	_UNNEST

//    */	  ( n1 n2 n3 -- q )
// 	Multiply n1 by n2, then divide by n3. Return quotient only.

	.word	_SSMOD-MAPOFFSET
_STASL:	.byte  2
	.ascii "*/"
	.align 2 	
STASL:
	_NEST
	BL	SSMOD
	BL	SWAP
	BL	DROP
	_UNNEST

// **************************************************************************
//  Miscellaneous

//    ALIGNED	( b -- a )
// 	Align address to the cell boundary.

	.word	_STASL-MAPOFFSET
_ALGND:	.byte   7
	.ascii "ALIGNED"
	.align 2 	
ALGND:
	ADD	R5,R5,#3
	MVN	R4,#3
	AND	R5,R5,R4
	_NEXT

//    >CHAR	( c -- c )
// 	Filter non-printing characters.

	.word	_ALGND-MAPOFFSET
_TCHAR:	.byte  5
	.ascii ">CHAR"
	.align 2 	
TCHAR:
	_NEST
	_DOLIT
	.word	0x7F
	BL	ANDD
	BL	DUPP	// mask msb
	BL	BLANK
	_DOLIT
	.word	127
	BL	WITHI	// check for printable
	BL	INVER
	BL	QBRAN
	.word	TCHA1-MAPOFFSET
	BL	DROP
	_DOLIT
	.word	'_'	// replace non-printables
TCHA1:
	  _UNNEST

//    DEPTH	( -- n )
// 	Return the depth of the data stack.

	.word	_TCHAR-MAPOFFSET
_DEPTH:	.byte  5
	.ascii "DEPTH"
	.align 2 	
DEPTH:
	_PUSH
	MOVW	R5,#0X4f04
 	MOVT	R5,#0X2000
	SUB	R5,R5,R1
	ASR	R5,R5,#2
	SUB	R5,R5,#1
	_NEXT

//    PICK	( ... +n -- ... w )
// 	Copy the nth stack item to tos.

	.word	_DEPTH-MAPOFFSET
_PICK:	.byte  4
	.ascii "PICK"
	.align 2 	
PICK:
	_NEST
	BL	ONEP
	BL	CELLS
	BL	SPAT
	BL	PLUS
	BL	AT
	_UNNEST

// **************************************************************************
//  Memory access

//    HERE	( -- a )
// 	Return the top of the code dictionary.

	.word	_PICK-MAPOFFSET
_HERE:	.byte  4
	.ascii "HERE"
	.align 2 	
HERE:
	_NEST
	BL	CPP
	BL	AT
	_UNNEST
	
//    PAD	 ( -- a )
// 	Return the address of a temporary buffer.

	.word	_HERE-MAPOFFSET
_PAD:	.byte  3
	.ascii "PAD"
	.align 2 	
PAD:
	_NEST
	BL	HERE
	ADD	R5,R5,#80
	_UNNEST

//    TIB	 ( -- a )
// 	Return the address of the terminal input buffer.

	.word	_PAD-MAPOFFSET
_TIB:	.byte  3
	.ascii "TIB"
	.align 2 	
TIB:
	_PUSH
	ldr r5,[r3,#TIB_OFS]
	_NEXT

//    @EXECUTE	( a -- )
// 	Execute vector stored in address a.

	.word	_TIB-MAPOFFSET
_ATEXE:	.byte   8
	.ascii "@EXECUTE"
	.align 2 	
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

	.word	_ATEXE-MAPOFFSET
_CMOVE:	.byte   5
	.ascii "CMOVE"
	.align 2 	
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

	.word	_CMOVE-MAPOFFSET
_MOVE:	.byte   4
	.ascii "MOVE"
	.align 2 	
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

	.word	_MOVE-MAPOFFSET
_FILL:	.byte   4
	.ascii "FILL"
	.align 2 	
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

	.word	_FILL-MAPOFFSET
_PACKS:	.byte  5
	.ascii "PACK$$"
	.align 2 	
PACKS:
	_NEST
	BL	ALGND
	BL	DUPP
	BL	TOR			// strings only on cell boundary
	BL	OVER
	BL	PLUS
	BL	ONEP
	_DOLIT
	.word	0xFFFFFFFC
	BL	ANDD			// count mod cell
	_DOLIT
	.word	0
	BL	SWAP
	BL	STORE			// null fill cell
	BL	RAT
	BL	DDUP
	BL	CSTOR
	BL	ONEP			// save count
	BL	SWAP
	BL	CMOVE
	BL	RFROM
	_UNNEST   			// move string

// **************************************************************************
//  Numeric output, single precision

//    DIGIT	( u -- c )
// 	Convert digit u to a character.

	.word	_PACKS-MAPOFFSET
_DIGIT:	.byte  5
	.ascii "DIGIT"
	.align 2 	
DIGIT:
	_NEST
	_DOLIT
	.word	9
	BL	OVER
	BL	LESS
	AND	R5,R5,#7
	BL	PLUS
	ADD	R5,R5,#'0'
	_UNNEST

//    EXTRACT	( n base -- n c )
// 	Extract the least significant digit from n.

	.word	_DIGIT-MAPOFFSET
_EXTRC:	.byte  7
	.ascii "EXTRACT"
	.align 2 	
EXTRC:
	_NEST
	_DOLIT
	.word	0
	BL	SWAP
	BL	UMMOD
	BL	SWAP
	BL	DIGIT
	_UNNEST

//    <#	  ( -- )
// 	Initiate the numeric output process.

	.word	_EXTRC-MAPOFFSET
_BDIGS:	.byte  2
	.ascii "<#"
	.align 2 	
BDIGS:
	_NEST
	BL	PAD
	BL	HLD
	BL	STORE
	_UNNEST

//    HOLD	( c -- )
// 	Insert a character into the numeric output string.

	.word	_BDIGS-MAPOFFSET
_HOLD:	.byte  4
	.ascii "HOLD"
	.align 2 	
HOLD:
	_NEST
	BL	HLD
	BL	AT
	BL	ONEM
	BL	DUPP
	BL	HLD
	BL	STORE
	BL	CSTOR
	_UNNEST

//    #	   ( u -- u )
// 	Extract one digit from u and append the digit to output string.

	.word	_HOLD-MAPOFFSET
_DIG:	.byte  1
	.ascii "#"
	.align 2 	
DIG:
	_NEST
	BL	BASE
	BL	AT
	BL	EXTRC
	BL	HOLD
	_UNNEST

//    #S	  ( u -- 0 )
// 	Convert u until all digits are added to the output string.

	.word	_DIG-MAPOFFSET
_DIGS:	.byte  2
	.ascii "#S"
	.align 2 	
DIGS:
	_NEST
DIGS1:
    BL	DIG
	BL	DUPP
	BL	QBRAN
	.word	DIGS2-MAPOFFSET
	B	DIGS1
DIGS2:
	  _UNNEST

//    SIGN	( n -- )
// 	Add a minus sign to the numeric output string.

	.word	_DIGS-MAPOFFSET
_SIGN:	.byte  4
	.ascii "SIGN"
	.align 2 	
SIGN:
	_NEST
	BL	ZLESS
	BL	QBRAN
	.word	SIGN1-MAPOFFSET
	_DOLIT
	.word	'-'
	BL	HOLD
SIGN1:
	  _UNNEST

//    #>	  ( w -- b u )
// 	Prepare the outputDCB to be TYPE'd.

	.word	_SIGN-MAPOFFSET
_EDIGS:	.byte  2
	.ascii "#>"
	.align 2 	
EDIGS:
	_NEST
	BL	DROP
	BL	HLD
	BL	AT
	BL	PAD
	BL	OVER
	BL	SUBB
	_UNNEST

//    str	 ( n -- b u )
// 	Convert a signed integer to a numeric string.

// 	.word	_EDIGS-MAPOFFSET
// _STRR	.byte  3
// 	.ascii "str"
// 	.align 2 	
STRR:
	_NEST
	BL	DUPP
	BL	TOR
	BL	ABSS
	BL	BDIGS
	BL	DIGS
	BL	RFROM
	BL	SIGN
	BL	EDIGS
	_UNNEST

//    HEX	 ( -- )
// 	Use radix 16 as base for numeric conversions.

	.word	_EDIGS-MAPOFFSET
_HEX:	.byte  3
	.ascii "HEX"
	.align 2 	
HEX:
	_NEST
	_DOLIT
	.word	16
	BL	BASE
	BL	STORE
	_UNNEST

//    DECIMAL	( -- )
// 	Use radix 10 as base for numeric conversions.

	.word	_HEX-MAPOFFSET
_DECIM:	.byte  7
	.ascii "DECIMAL"
	.align 2 	
DECIM:
	_NEST
	_DOLIT
	.word	10
	BL	BASE
	BL	STORE
	_UNNEST

// **************************************************************************
//  Numeric input, single precision

//    DIGIT?	( c base -- u t )
// 	Convert a character to its numeric value. A flag indicates success.

	.word	_DECIM-MAPOFFSET
_DIGTQ:	.byte  6
	.ascii "DIGIT?"
	.align 2 	
DIGTQ:
	_NEST
	BL	TOR
	_DOLIT
	.word	'0'
	BL	SUBB
	_DOLIT
	.word	9
	BL	OVER
	BL	LESS
	BL	QBRAN
	.word	DGTQ1-MAPOFFSET
	_DOLIT
	.word	7
	BL	SUBB
	BL	DUPP
	_DOLIT
	.word	10
	BL	LESS
	BL	ORR
DGTQ1:
  BL	DUPP
	BL	RFROM
	BL	ULESS
	_UNNEST

//    NUMBER?	( a -- n T | a F )
// 	Convert a numberDCB to integer. Push a flag on tos.

	.word	_DIGTQ-MAPOFFSET
_NUMBQ:	.byte  7
	.ascii "NUMBER?"
	.align 2 	
NUMBQ:
	_NEST
	BL	BASE
	BL	AT
	BL	TOR
	_DOLIT
	.word	0
	BL	OVER
	BL	COUNT
	BL	OVER
	BL	CAT
	_DOLIT
	.word	'_'
	BL	EQUAL
	BL	QBRAN
	.word	NUMQ1-MAPOFFSET
	BL	HEX
	BL	SWAP
	BL	ONEP
	BL	SWAP
	BL	ONEM
NUMQ1:
  BL	OVER
	BL	CAT
	_DOLIT
	.word	'-'
	BL	EQUAL
	BL	TOR
	BL	SWAP
	BL	RAT
	BL	SUBB
	BL	SWAP
	BL	RAT
	BL	PLUS
	BL	QDUP
	BL	QBRAN
	.word	NUMQ6-MAPOFFSET
	BL	ONEM
	BL	TOR
NUMQ2:
  BL	DUPP
	BL	TOR
	BL	CAT
	BL	BASE
	BL	AT
	BL	DIGTQ
	BL	QBRAN
	.word	NUMQ4-MAPOFFSET
	BL	SWAP
	BL	BASE
	BL	AT
	BL	STAR
	BL	PLUS
	BL	RFROM
	BL	ONEP
	BL	DONXT
	.word	NUMQ2-MAPOFFSET
	BL	RAT
	BL	SWAP
	BL	DROP
	BL	QBRAN
	.word	NUMQ3-MAPOFFSET
	BL	NEGAT
NUMQ3:
  BL	SWAP
	B.W	NUMQ5
NUMQ4:
  BL	RFROM
	BL	RFROM
	BL	DDROP
	BL	DDROP
	_DOLIT
	.word	0
NUMQ5:
  BL	DUPP
NUMQ6:
  BL	RFROM
	BL	DDROP
	BL	RFROM
	BL	BASE
	BL	STORE
	_UNNEST

// **************************************************************************
//  Basic I/O

//    KEY	 ( -- c )
// 	Wait for and return an input character.

	.word	_NUMBQ-MAPOFFSET
_KEY:	.byte  3
	.ascii "KEY"
	.align 2 	
KEY:
	_NEST
KEY1:
	BL	QRX
	BL	QBRAN
	.word	KEY1-MAPOFFSET
	_UNNEST

//    SPACE	( -- )
// 	Send the blank character to the output device.

	.word	_KEY-MAPOFFSET
_SPACE:	.byte  5
	.ascii "SPACE"
	.align 2 	
SPACE:
	_NEST
	BL	BLANK
	BL	EMIT
	_UNNEST

//    SPACES	( +n -- )
// 	Send n spaces to the output device.

	.word	_SPACE-MAPOFFSET
_SPACS:	.byte  6
	.ascii "SPACES"
	.align 2 	
SPACS:
	_NEST
	_DOLIT
	.word	0
	BL	MAX
	BL	TOR
	B.W	CHAR2
CHAR1:
	BL	SPACE
CHAR2:
	BL	DONXT
	.word	CHAR1-MAPOFFSET
	_UNNEST

//    TYPE	( b u -- )
// 	Output u characters from b.

	.word	_SPACS-MAPOFFSET
_TYPEE:	.byte	4
	.ascii "TYPE"
	.align 2 	
TYPEE:
	_NEST
	BL  TOR
	B.W	TYPE2
TYPE1:  
	BL  COUNT
	BL	TCHAR
	BL	EMIT
TYPE2:  
	BL  DONXT
	.word	TYPE1-MAPOFFSET
	BL	DROP
	_UNNEST

//    CR	  ( -- )
// 	Output a carriage return and a line feed.

	.word	_TYPEE-MAPOFFSET
_CR:	.byte  2
	.ascii "CR"
	.align 2 	
CR:
	_NEST
	_DOLIT
	.word	CRR
	BL	EMIT
	_DOLIT
	.word	LF
	BL	EMIT
	_UNNEST

//    do_$	( -- a )
// 	Return the address of a compiled string.
//  adjust return address to skip over it.

// 	.word	_CR-MAPOFFSET
// _DOSTR	.byte  COMPO+3
// 	.ascii "do$$"
// 	.align 2 	
DOSTR:
	_NEST
	BL	RFROM
	BL	RFROM			//  b0 set
	BL	ONEM			//  clear b0
	BL	DUPP
	BL	COUNT			//  get addr+1 count
	BL	PLUS
	BL	ALGND			//  end of string
	BL	ONEP			//  restore b0
	BL	TOR				//  address after string
	BL	SWAP			//  count tugged
	BL	TOR
	_UNNEST

//    $"|	( -- a )
// 	Run time routine compiled by _". Return address of a compiled string.

// 	.word	_DOSTR-MAPOFFSET
// _STRQP	.byte  COMPO+3
// 	.ascii "$$""|"
// 	.align 2 	
STRQP:
	_NEST
	BL	DOSTR
	_UNNEST			// force a call to dostr

//    .$	( a -- )
// 	Run time routine of ." . Output a compiled string.

// 	.word	_STRQP-MAPOFFSET
// _DOTST	.byte  COMPO+2
// 	.ascii ".$$"
// 	.align 2 	
DOTST:
	_NEST
	BL	COUNT
	BL	TYPEE
	_UNNEST

//    ."|	( -- )
// 	Run time routine of ." . Output a compiled string.

// 	.word	_DOTST-MAPOFFSET
// _DOTQP	.byte  COMPO+3
// 	.ascii ".""|"
// 	.align 2 	
DOTQP:
	_NEST
	BL	DOSTR
	BL	DOTST
	_UNNEST

//    .R	  ( n +n -- )
// 	Display an integer in a field of n columns, right justified.

	.word	_CR-MAPOFFSET
_DOTR:	.byte  2
	.ascii ".R"
	.align 2 	
DOTR:
	_NEST
	BL	TOR
	BL	STRR
	BL	RFROM
	BL	OVER
	BL	SUBB
	BL	SPACS
	BL	TYPEE
	_UNNEST

//    U.R	 ( u +n -- )
// 	Display an unsigned integer in n column, right justified.

	.word	_DOTR-MAPOFFSET
_UDOTR:	.byte  3
	.ascii "U.R"
	.align 2 	
UDOTR:
	_NEST
	BL	TOR
	BL	BDIGS
	BL	DIGS
	BL	EDIGS
	BL	RFROM
	BL	OVER
	BL	SUBB
	BL	SPACS
	BL	TYPEE
	_UNNEST

//    U.	  ( u -- )
// 	Display an unsigned integer in free format.

	.word	_UDOTR-MAPOFFSET
_UDOT:	.byte  2
	.ascii "U."
	.align 2 	
UDOT:
	_NEST
	BL	BDIGS
	BL	DIGS
	BL	EDIGS
	BL	SPACE
	BL	TYPEE
	_UNNEST

//    .	   ( w -- )
// 	Display an integer in free format, preceeded by a space.

	.word	_UDOT-MAPOFFSET
_DOT:	.byte  1
	.ascii "."
	.align 2 	
DOT:
	_NEST
	BL	BASE
	BL	AT
	_DOLIT
	.word	10
	BL	XORR			// ?decimal
	BL	QBRAN
	.word	DOT1-MAPOFFSET
	BL	UDOT
	_UNNEST			// no,display unsigned
DOT1:
    BL	STRR
	BL	SPACE
	BL	TYPEE
	_UNNEST			// yes, display signed

//    ?	   ( a -- )
// 	Display the contents in a memory cell.

	.word	_DOT-MAPOFFSET
_QUEST:	.byte  1
	.ascii "?"
	.align 2 	
QUEST:
	_NEST
	BL	AT
	BL	DOT
	_UNNEST

// **************************************************************************
//  Parsing

//    parse	( b u c -- b u delta //  string> )
// 	ScanDCB delimited by c. Return found string and its offset.

// 	.word	_QUEST-MAPOFFSET
// _PARS	.byte  5
// 	.ascii "parse"
// 	.align 2 	
PARS:
	_NEST
	BL	TEMP
	BL	STORE
	BL	OVER
	BL	TOR
	BL	DUPP
	BL	QBRAN
	.word	PARS8-MAPOFFSET
	BL	ONEM
	BL	TEMP
	BL	AT
	BL	BLANK
	BL	EQUAL
	BL	QBRAN
	.word	PARS3-MAPOFFSET
	BL	TOR
PARS1:
  BL	BLANK
	BL	OVER
	BL	CAT			// skip leading blanks 
	BL	SUBB
	BL	ZLESS
	BL	INVER
	BL	QBRAN
	.word	PARS2-MAPOFFSET
	BL	ONEP
	BL	DONXT
	.word	PARS1-MAPOFFSET
	BL	RFROM
	BL	DROP
	_DOLIT
	.word	0
	BL	DUPP
	_UNNEST
PARS2:
  BL	RFROM
PARS3:
  BL	OVER
	BL	SWAP
	BL	TOR
PARS4:
  BL	TEMP
	BL	AT
	BL	OVER
	BL	CAT
	BL	SUBB			// scan for delimiter
	BL	TEMP
	BL	AT
	BL	BLANK
	BL	EQUAL
	BL	QBRAN
	.word	PARS5-MAPOFFSET
	BL	ZLESS
PARS5:
  BL	QBRAN
	.word	PARS6-MAPOFFSET
	BL	ONEP
	BL	DONXT
	.word	PARS4-MAPOFFSET
	BL	DUPP
	BL	TOR
	B	PARS7
PARS6:
  BL	RFROM
	BL	DROP
	BL	DUPP
	BL	ONEP
	BL	TOR
PARS7:
  BL	OVER
	BL	SUBB
	BL	RFROM
	BL	RFROM
	BL	SUBB
	_UNNEST
PARS8:
  BL	OVER
	BL	RFROM
	BL	SUBB
	_UNNEST

//    PARSE	( c -- b u //  string> )
// 	Scan input stream and return counted string delimited by c.

	.word	_QUEST-MAPOFFSET
_PARSE:	.byte  5
	.ascii "PARSE"
	.align 2 	
PARSE:
	_NEST
	BL	TOR
	BL	TIB
	BL	INN
	BL	AT
	BL	PLUS			// current input buffer pointer
	BL	NTIB
	BL	AT
	BL	INN
	BL	AT
	BL	SUBB			// remaining count
	BL	RFROM
	BL	PARS
	BL	INN
	BL	PSTOR
	_UNNEST

//    .(	  ( -- )
// 	Output following string up to next ) .

	.word	_PARSE-MAPOFFSET
_DOTPR:	.byte  IMEDD+2
	.ascii ".("
	.align 2 	
DOTPR:
	_NEST
	_DOLIT
	.word	')'
	BL	PARSE
	BL	TYPEE
	_UNNEST

//    (	   ( -- )
// 	Ignore following string up to next ) . A comment.

	.word	_DOTPR-MAPOFFSET
_PAREN:	.byte  IMEDD+1
	.ascii "("
	.align 2 	
PAREN:
	_NEST
	_DOLIT
	.word	')'
	BL	PARSE
	BL	DDROP
	_UNNEST

//    \	   ( -- )
// 	Ignore following text till the end of line.

	.word	_PAREN-MAPOFFSET
_BKSLA:	.byte  IMEDD+1
	.byte	'\'
	.align 2 	
BKSLA:
	_NEST
	BL	NTIB
	BL	AT
	BL	INN
	BL	STORE
	_UNNEST

//    CHAR	( -- c )
// 	Parse next word and return its first character.

	.word	_BKSLA-MAPOFFSET
_CHAR:	.byte  4
	.ascii "CHAR"
	.align 2 	
CHAR:
	_NEST
	BL	BLANK
	BL	PARSE
	BL	DROP
	BL	CAT
	_UNNEST

//    WORD	( c -- a //  string> )
// 	Parse a word from input stream and copy it to code dictionary.

	.word	_CHAR-MAPOFFSET
_WORDD:	.byte  4
	.ascii "WORD"
	.align 2 	
WORDD:
	_NEST
	BL	PARSE
	BL	HERE
	BL	CELLP
	BL	PACKS
	_UNNEST

//    TOKEN	( -- a //  string> )
// 	Parse a word from input stream and copy it to name dictionary.

	.word	_WORDD-MAPOFFSET
_TOKEN:	.byte  5
	.ascii "TOKEN"
	.align 2 	
TOKEN:
	_NEST
	BL	BLANK
	BL	WORDD
	_UNNEST

// **************************************************************************
//  Dictionary search

//    NAME>	( na -- ca )
// 	Return a code address given a name address.

	.word	_TOKEN-MAPOFFSET
_NAMET:	.byte  5
	.ascii "NAME>"
	.align 2 	
NAMET:
	_NEST
	BL	COUNT
	_DOLIT
	.word	0x1F
	BL	ANDD
	BL	PLUS
	BL	ALGND
	_UNNEST

//    SAME?	( a a u -- a a f \ -0+ )
// 	Compare u cells in two strings. Return 0 if identical.

	.word	_NAMET-MAPOFFSET
_SAMEQ:	.byte  5
	.ascii "SAME?"
	.align 2 	
SAMEQ:
	_NEST
	BL	TOR
	B.W	SAME2
SAME1:
  BL	OVER
	BL	RAT
	BL	CELLS
	BL	PLUS
	BL	AT			// 32/16 mix-up
	BL	OVER
	BL	RAT
	BL	CELLS
	BL	PLUS
	BL	AT			// 32/16 mix-up
	BL	SUBB
	BL	QDUP
	BL	QBRAN
	.word	SAME2-MAPOFFSET
	BL	RFROM
	BL	DROP
	_UNNEST			// strings not equal
SAME2:
  BL	DONXT
	.word	SAME1-MAPOFFSET
	_DOLIT
	.word	0
	_UNNEST			// strings equal

//    find	( a na -- ca na | a F )
// 	Search a vocabulary for a string. Return ca and na if succeeded.

// 	.word	_SAMEQ-MAPOFFSET
// _FIND	.byte  4
// 	.ascii "find"
// 	.align 2 	
FIND:
	_NEST
	BL	SWAP			//  na a	
	BL	DUPP			//  na a a
	BL	CAT			//  na a count
	BL	CELLSL		//  na a count/4
	BL	TEMP
	BL	STORE			//  na a
	BL	DUPP			//  na a a
	BL	AT			//  na a word1
	BL	TOR			//  na a
	BL	CELLP			//  na a+4
	BL	SWAP			//  a+4 na
FIND1:
	BL	DUPP			//  a+4 na na
	BL	QBRAN
	.word	FIND6-MAPOFFSET	//  end of vocabulary
	BL	DUPP			//  a+4 na na
	BL	AT			//  a+4 na name1
	_DOLIT
	.word	MASKK
	BL	ANDD
	BL	RAT			//  a+4 na name1 word1
	BL	XORR			//  a+4 na ?
	BL	QBRAN
	.word	FIND2-MAPOFFSET
	BL	CELLM			//  a+4 la
	BL	AT			//  a+4 next_na
	B.w	FIND1			//  try next word
FIND2:   
	BL	CELLP			//  a+4 na+4
	BL	TEMP
	BL	AT			//  a+4 na+4 count/4
	BL	SAMEQ			//  a+4 na+4 ? 
FIND3:	
	B.w	FIND4
FIND6:	
	BL	RFROM			//  a+4 0 name1 -- , no match
	BL	DROP			//  a+4 0
	BL	SWAP			//  0 a+4
	BL	CELLM			//  0 a
	BL	SWAP			//  a 0 
	_UNNEST			//  return without a match
FIND4:	
	BL	QBRAN			//  a+4 na+4
	.word	FIND5-MAPOFFSET	//  found a match
	BL	CELLM			//  a+4 na
	BL	CELLM			//  a+4 la
	BL	AT			//  a+4 next_na
	B.w	FIND1			//  compare next name
FIND5:	
	BL	RFROM			//  a+4 na+4 count/4
	BL	DROP			//  a+4 na+4
	BL	SWAP			//  na+4 a+4
	BL	DROP			//  na+4
	BL	CELLM			//  na
	BL	DUPP			//  na na
	BL	NAMET			//  na ca
	BL	SWAP			//  ca na
	_UNNEST			//  return with a match

//    NAME?	( a -- ca na | a F )
// 	Search all context vocabularies for a string.

	.word	_SAMEQ-MAPOFFSET
_NAMEQ:	.byte  5
	.ascii "NAME?"
	.align 2 	
NAMEQ:
	_NEST
	BL	CNTXT
	BL	AT
	BL	FIND
	_UNNEST

// **************************************************************************
//  Terminal input

//    	  ( bot eot cur -- bot eot cur )
// 	Backup the cursor by one character.

// 	.word	_NAMEQ-MAPOFFSET
// _BKSP	.byte  2
// 	.ascii "^H"
// 	.align 2 	
BKSP:
	_NEST
	BL	TOR
	BL	OVER
	BL	RFROM
	BL	SWAP
	BL	OVER
	BL	XORR
	BL	QBRAN
	.word	BACK1-MAPOFFSET
	_DOLIT
	.word	BKSPP
	BL	TECHO
// 	BL	ATEXE
	BL	ONEM
	BL	BLANK
	BL	TECHO
// 	BL	ATEXE
	_DOLIT
	.word	BKSPP
	BL	TECHO
// 	BL	ATEXE
BACK1:
	  _UNNEST

//    TAP	 ( bot eot cur c -- bot eot cur )
// 	Accept and echo the key stroke and bump the cursor.

// 	.word	_BKSP-MAPOFFSET
// _TAP	.byte  3
// 	.ascii "TAP"
// 	.align 2 	
TAP:
	_NEST
	BL	DUPP
	BL	TECHO
// 	BL	ATEXE
	BL	OVER
	BL	CSTOR
	BL	ONEP
	_UNNEST

//    kTAP	( bot eot cur c -- bot eot cur )
// 	Process a key stroke, CR or backspace.

// 	.word	_TAP-MAPOFFSET
// _KTAP	.byte  4
// 	.ascii "kTAP"
// 	.align 2 	
KTAP:
TTAP:
	_NEST
	BL	DUPP
	_DOLIT
	.word	CRR
	BL	XORR
	BL	QBRAN
	.word	KTAP2-MAPOFFSET
	_DOLIT
	.word	BKSPP
	BL	XORR
	BL	QBRAN
	.word	KTAP1-MAPOFFSET
	BL	BLANK
	BL	TAP
	_UNNEST
	.word	0			// patch
KTAP1:
  BL	BKSP
	_UNNEST
KTAP2:
  BL	DROP
	BL	SWAP
	BL	DROP
	BL	DUPP
	_UNNEST

//    ACCEPT	( b u -- b u )
// 	Accept characters to input buffer. Return with actual count.

	.word	_NAMEQ-MAPOFFSET
_ACCEP:	.byte  6
	.ascii "ACCEPT"
	.align 2 	
ACCEP:
	_NEST
	BL	OVER
	BL	PLUS
	BL	OVER
ACCP1:
  BL	DDUP
	BL	XORR
	BL	QBRAN
	.word	ACCP4-MAPOFFSET
	BL	KEY
	BL	DUPP
	BL	BLANK
	_DOLIT
	.word	127
	BL	WITHI
	BL	QBRAN
	.word	ACCP2-MAPOFFSET
	BL	TAP
	B	ACCP3
ACCP2:
  BL	KTAP
// 	BL	ATEXE
ACCP3:	  
	B	ACCP1
ACCP4:
  BL	DROP
	BL	OVER
	BL	SUBB
	_UNNEST

//    QUERY	( -- )
// 	Accept input stream to terminal input buffer.

	.word	_ACCEP-MAPOFFSET
_QUERY:	.byte  5
	.ascii "QUERY"
	.align 2 	
QUERY:
	_NEST
	BL	TIB
	_DOLIT
	.word	80
	BL	ACCEP
	BL	NTIB
	BL	STORE
	BL	DROP
	_DOLIT
	.word	0
	BL	INN
	BL	STORE
	_UNNEST

// **************************************************************************
//  Error handling

//    ABORT	( a -- )
// 	Reset data stack and jump to QUIT.

	.word	_QUERY-MAPOFFSET
_ABORT:	.byte  5
	.ascii "ABORT"
	.align 2 	
ABORT:
	_NEST
	BL	SPACE
	BL	COUNT
	BL	TYPEE
	_DOLIT
	.word	0X3F
	BL	EMIT
	BL	CR
	BL	PRESE
	B.W	QUIT

//    _abort"	( f -- )
// 	Run time routine of ABORT" . Abort with a message.

// 	.word	_ABORT-MAPOFFSET
// _ABORQ	.byte  COMPO+6
// 	.ascii "abort\""
// 	.align 2 	
ABORQ:
	_NEST
	BL	QBRAN
	.word	ABOR1-MAPOFFSET	// text flag
	BL	DOSTR
	BL	COUNT
	BL	TYPEE
	BL	CR
	B.W	QUIT
ABOR1:
  BL	DOSTR
	BL	DROP
	_UNNEST			// drop error

// **************************************************************************
//  The text interpreter

//    $INTERPRET  ( a -- )
// 	Interpret a word. If failed, try to convert it to an integer.

	.word	_ABORT-MAPOFFSET
_INTER:	.byte  10
	.ascii "$$INTERPRET"
	.align 2 	
INTER:
	_NEST
	BL	NAMEQ
	BL	QDUP	// ?defined
	BL	QBRAN
	.word	INTE1-MAPOFFSET
	BL	AT
	_DOLIT
	.word	COMPO
	BL	ANDD	// ?compile only lexicon bits
	BL	ABORQ
	.byte	13
	.ascii " compile only"
	.align 2 	
	BL	EXECU
	_UNNEST			// execute defined word
INTE1:
  BL	NUMBQ
	BL	QBRAN
	.word	INTE2-MAPOFFSET
	_UNNEST
INTE2:
  B.W	ABORT	// error

//    [	   ( -- )
// 	Start the text interpreter.

	.word	_INTER-MAPOFFSET
_LBRAC:	.byte  IMEDD+1
	.ascii "["
	.align 2 	
LBRAC:
	_NEST
	_DOLIT
	.word	INTER-MAPOFFSET
	BL	TEVAL
	BL	STORE
	_UNNEST

//    .OK	 ( -- )
// 	Display "ok" only while interpreting.

	.word	_LBRAC-MAPOFFSET
_DOTOK:	.byte  3
	.ascii ".OK"
	.align 2 	
DOTOK:
	_NEST
	_DOLIT
	.word	INTER-MAPOFFSET
	BL	TEVAL
	BL	AT
	BL	EQUAL
	BL	QBRAN
	.word	DOTO1-MAPOFFSET
	BL	DOTQP
	.byte	3
	.ascii " ok"
DOTO1:
	BL	CR
	_UNNEST

//    ?STACK	( -- )
// 	Abort if the data stack underflows.

	.word	_DOTOK-MAPOFFSET
_QSTAC:	.byte  6
	.ascii "?STACK"
	.align 2 	
QSTAC:
	_NEST
	BL	DEPTH
	BL	ZLESS	// check only for underflow
	BL	ABORQ
	.byte	10
	.ascii " underflow"
	.align 2 	
	_UNNEST

//    EVAL	( -- )
// 	Interpret the input stream.

	.word	_QSTAC-MAPOFFSET
_EVAL:	.byte  4
	.ascii "EVAL"
	.align 2 	
EVAL:
	_NEST
EVAL1:
    BL	TOKEN
	BL	DUPP
	BL	CAT	// ?input stream empty
	BL	QBRAN
	.word	EVAL2-MAPOFFSET
	BL	TEVAL
	BL	ATEXE
	BL	QSTAC	// evaluate input, check stack
	B.W	EVAL1
EVAL2:
  BL	DROP
	BL	DOTOK
	_UNNEST	// prompt

//    PRESET	( -- )
// 	Reset data stack pointer and the terminal input buffer.

	.word	_EVAL-MAPOFFSET
_PRESE:	.byte  6
	.ascii "PRESET"
	.align 2 	
PRESE:
	_NEST
	MOVW	R1,#0x4F00		//  init SP
 	MOVT	R1,#0X2000
	MOVW	R0,#0			//  init TOS ???
	_UNNEST

//    QUIT	( -- )
// 	Reset return stack pointer and start text interpreter.

	.word	_PRESE-MAPOFFSET
_QUIT:	.byte  4
	.ascii "QUIT"
	.align 2 	
QUIT:
	_NEST
	MOVW	R2,#0x4F80  /* RESET RSTACK */
 	MOVT	R2,#0x2000
QUIT1:
  BL	LBRAC			// start interpretation
QUIT2:
  BL	QUERY			// get input
	BL	EVAL
	BL	BRAN
	.word	QUIT2-MAPOFFSET	// continue till error

/***************************
//  Flash memory interface
***************************/
UNLOCK:	//  unlock flash memory	
	ldr	r0, =FLASH_BASE_ADR
	ldr	r4, =FLASH_KEY1
	str	r4, [r0, #FLASH_KEYR]
	ldr	r4, =FLASH_KEY2
	str	r4, [r0, #FLASH_KEYR]
	/* unlock option registers */
	ldr	r4, =FLASH_KEY1
	str	r4, [r0, #FLASH_OPTKEYR]
	ldr	r4, =FLASH_KEY2
	str	r4, [r0, #FLASH_OPTKEYR]
	_NEXT
WAIT_BSY:
	ldr	r0,=FLASH_BASE_ADR
WAIT1:
	ldr	r4, [r0, #FLASH_SR]	//  FLASH_SR
	ands	r4, #0x1	//  BSY
	bne	WAIT1
	_NEXT

//    ERASE_SECTOR	   ( sector -- )
// 	  Erase one sector of flash memory.  Sector=0 to 11

	.word	_QUIT-MAPOFFSET
_ESECT:	.byte  12
	.ascii "ERASE_SECTOR"
	.align 2 	

ESECT: 	//  sector --
	_NEST
	bl	WAIT_BSY
	ldr	r4,[r0, #FLASH_CR]	
	bic	r4,r4,#0x78	//  clear SNB
	lsl	R5,R5,#3		//  align sector #
	orr	r4,r4,r5		//  put in sector #
	orr	R4,R4,#0x10000	//  set STRT bit
	orr	R4,R4,#0x200	//  PSIZE=32
	orr	R4,R4,#2		//  set SER bit, enable erase
	str	r4,[r0, #0x10]	//  start erasing
// 	bl	WAIT_BSY
	_POP
	_UNNEST

//    I!	   ( data address -- )
// 	   Write one word into flash memory

	.word	_ESECT-MAPOFFSET
_ISTOR:	.byte  2
	.ascii "I!"
	.align 2 	

ISTOR:	//  data address --
	_NEST
	bl	WAIT_BSY
	ldr	r4, [r0, #0x10]	//  FLASH_CR
	orr	r4,R4,#0x1		//  PG
	str	r4, [r0, #0x10]	//  enable programming
	bl	STORE
	bl	WAIT_BSY
	ldr	r4, [r0, #0x10]	//  FLASH_CR
	bic	r4,R4,#0x1		//  PG
	str	r4, [r0, #0x10]	//  disable programming
	_UNNEST

//    TURNKEY	( -- )
// 	Copy dictionary from RAM to flash.

	.word	_ISTOR-MAPOFFSET
_TURN:	.byte   7
	.ascii "TURNKEY"
	.align 2 
TURN:	_NEST
	_DOLIT			//  save user area
	.word	0XFF00
	_DOLIT
	.word	0xC0			//  to boot array
	_DOLIT
	.word	0x40
	BL	MOVE
	_DOLIT
	.word	0
	_DOLIT
	.word	0x8000000
	BL	CPP
	BL	AT
	BL	CELLSL
	BL	TOR
TURN1:
	BL	OVER
	BL	AT
	BL	OVER
	BL	ISTOR
	BL	SWAP
	BL	CELLP
	BL	SWAP
	BL	CELLP
	BL	DONXT
	.word	TURN1-MAPOFFSET
	BL	DDROP
	_UNNEST

// **************************************************************************
//  The compiler

//    '	   ( -- ca )
// 	Search context vocabularies for the next word in input stream.

	.word	_TURN-MAPOFFSET
_TICK:	.byte  1
	.ascii "'"
	.align 2 	
TICK:
	_NEST
	BL	TOKEN
	BL	NAMEQ	// ?defined
	BL	QBRAN
	.word	TICK1-MAPOFFSET
	_UNNEST	// yes, push code address
TICK1:	B.W	ABORT	// no, error

//    ALLOT	( n -- )
// 	Allocate n bytes to the ram area.

	.word	_TICK-MAPOFFSET
_ALLOT:	.byte  5
	.ascii "ALLOT"
	.align 2 	
ALLOT:
	_NEST
	BL	CPP
	BL	PSTOR
	_UNNEST			// adjust code pointer

//    ,	   ( w -- )
// 	Compile an integer into the code dictionary.

	.word	_ALLOT-MAPOFFSET
_COMMA:	.byte  1,','
	.align 2 	
COMMA:
	_NEST
	BL	HERE
	BL	DUPP
	BL	CELLP	// cell boundary
	BL	CPP
	BL	STORE
	BL	STORE
	_UNNEST	// adjust code pointer, compile
	.align 2 
//    [COMPILE]   ( -- //  string> )
// 	Compile the next immediate word into code dictionary.

	.word	_COMMA-MAPOFFSET
_BCOMP:	.byte  IMEDD+9
	.ascii "[COMPILE]"
	.align 2 	
BCOMP:
	_NEST
	BL	TICK
	BL	COMMA
	_UNNEST

//    COMPILE	( -- )
// 	Compile the next address in colon list to code dictionary.

	.word	_BCOMP-MAPOFFSET
_COMPI:	.byte  COMPO+7
	.ascii "COMPILE"
	.align 2 	
COMPI:
	_NEST
	BL	RFROM
	BIC	R5,R5,#1
	BL	DUPP
	BL	AT
	BL	CALLC			// compile BL instruction
	BL	CELLP
	ORR	R5,R5,#1
	BL	TOR
	_UNNEST			// adjust return address

//    LITERAL	( w -- )
// 	Compile tos to code dictionary as an integer literal.

	.word	_COMPI-MAPOFFSET
_LITER:	.byte  IMEDD+7
	.ascii "LITERAL"
	.align 2 	
LITER:
	_NEST
	BL	COMPI
	.word	DOLIT-MAPOFFSET
	BL	COMMA
	_UNNEST

//    $,"	( -- )
// 	Compile a literal string up to next " .

// 	.word	_LITER-MAPOFFSET
// _STRCQ	.byte  3
// 	.ascii "$$,"""
// 	.align 2 	
STRCQ:
	_NEST
	_DOLIT
	.word	-4
	BL	CPP
	BL	PSTOR
	_DOLIT
	.word	'\"'
	BL	WORDD			// moveDCB to code dictionary
	BL	COUNT
	BL	PLUS
	BL	ALGND			// calculate aligned end of string
	BL	CPP
	BL	STORE
	_UNNEST 			// adjust the code pointer

// **************************************************************************
//  Structures

//    FOR	 ( -- a )
// 	Start a FOR-NEXT loop structure in a colon definition.

	.word	_LITER-MAPOFFSET
_FOR:	.byte  IMEDD+3
	.ascii "FOR"
	.align 2 	
FOR:
	_NEST
	BL	COMPI
	.word	TOR-MAPOFFSET
	BL	HERE
	_UNNEST

//    BEGIN	( -- a )
// 	Start an infinite or indefinite loop structure.

	.word	_FOR-MAPOFFSET
_BEGIN:	.byte  IMEDD+5
	.ascii "BEGIN"
	.align 2 	
BEGIN:
	_NEST
	BL	HERE
	_UNNEST
	.align 2 
//    NEXT	( a -- )
// 	Terminate a FOR-NEXT loop structure.

	.word	_BEGIN-MAPOFFSET
_NEXT:	.byte  IMEDD+4
	.ascii "NEXT"
	.align 2 	
NEXT:
	_NEST
	BL	COMPI
	.word	DONXT-MAPOFFSET
	BL	COMMA
	_UNNEST

//    UNTIL	( a -- )
// 	Terminate a BEGIN-UNTIL indefinite loop structure.

	.word	_NEXT-MAPOFFSET
_UNTIL:	.byte  IMEDD+5
	.ascii "UNTIL"
	.align 2 	
UNTIL:
	_NEST
	BL	COMPI
	.word	QBRAN-MAPOFFSET
	BL	COMMA
	_UNNEST

//    AGAIN	( a -- )
// 	Terminate a BEGIN-AGAIN infinite loop structure.

	.word	_UNTIL-MAPOFFSET
_AGAIN:	.byte  IMEDD+5
	.ascii "AGAIN"
	.align 2 	
AGAIN:
	_NEST
	BL	COMPI
	.word	BRAN-MAPOFFSET
	BL	COMMA
	_UNNEST

//    IF	  ( -- A )
// 	Begin a conditional branch structure.

	.word	_AGAIN-MAPOFFSET
_IFF:	.byte  IMEDD+2
	.ascii "IF"
	.align 2 	
IFF:
	_NEST
	BL	COMPI
	.word	QBRAN-MAPOFFSET
	BL	HERE
	_DOLIT
	.word	4
	BL	CPP
	BL	PSTOR
	_UNNEST

//    AHEAD	( -- A )
// 	Compile a forward branch instruction.

	.word	_IFF-MAPOFFSET
_AHEAD:	.byte  IMEDD+5
	.ascii "AHEAD"
	.align 2 	
AHEAD:
	_NEST
	BL	COMPI
	.word	BRAN-MAPOFFSET
	BL	HERE
	_DOLIT
	.word	4
	BL	CPP
	BL	PSTOR
	_UNNEST

//    REPEAT	( A a -- )
// 	Terminate a BEGIN-WHILE-REPEAT indefinite loop.

	.word	_AHEAD-MAPOFFSET
_REPEA:	.byte  IMEDD+6
	.ascii "REPEAT"
	.align 2 	
REPEA:
	_NEST
	BL	AGAIN
	BL	HERE
	BL	SWAP
	BL	STORE
	_UNNEST

//    THEN	( A -- )
// 	Terminate a conditional branch structure.

	.word	_REPEA-MAPOFFSET
_THENN:	.byte  IMEDD+4
	.ascii "THEN"
	.align 2 	
THENN:
	_NEST
	BL	HERE
	BL	SWAP
	BL	STORE
	_UNNEST

//    AFT	 ( a -- a A )
// 	Jump to THEN in a FOR-AFT-THEN-NEXT loop the first time through.

	.word	_THENN-MAPOFFSET
_AFT:	.byte  IMEDD+3
	.ascii "AFT"
	.align 2 	
AFT:
	_NEST
	BL	DROP
	BL	AHEAD
	BL	BEGIN
	BL	SWAP
	_UNNEST

//    ELSE	( A -- A )
// 	Start the false clause in an IF-ELSE-THEN structure.

	.word	_AFT-MAPOFFSET
_ELSEE:	.byte  IMEDD+4
	.ascii "ELSE"
	.align 2 	
ELSEE:
	_NEST
	BL	AHEAD
	BL	SWAP
	BL	THENN
	_UNNEST

//    WHILE	( a -- A a )
// 	Conditional branch out of a BEGIN-WHILE-REPEAT loop.

	.word	_ELSEE-MAPOFFSET
_WHILE:	.byte  IMEDD+5
	.ascii "WHILE"
	.align 2 	
WHILE:
	_NEST
	BL	IFF
	BL	SWAP
	_UNNEST

//    ABORT"	( -- //  string> )
// 	Conditional abort with an error message.

	.word	_WHILE-MAPOFFSET
_ABRTQ:	.byte  IMEDD+6
	.ascii "ABORT\""
	.align 2 	
ABRTQ:
	_NEST
	BL	COMPI
	.word	ABORQ-MAPOFFSET
	BL	STRCQ
	_UNNEST

//    $"	( -- //  string> )
// 	Compile an inlineDCB literal.

	.word	_ABRTQ-MAPOFFSET
_STRQ:	.byte  IMEDD+2
	.byte	'$','"'
	.align 2 	
STRQ:
	_NEST
	BL	COMPI
	.word	STRQP-MAPOFFSET
	BL	STRCQ
	_UNNEST

//    ."	( -- //  string> )
// 	Compile an inlineDCB literal to be typed out at run time.

	.word	_STRQ-MAPOFFSET
_DOTQ:	.byte  IMEDD+2
	.byte	'.','"'
	.align 2 	
DOTQ:
	_NEST
	BL	COMPI
	.word	DOTQP-MAPOFFSET
	BL	STRCQ
	_UNNEST

// **************************************************************************
//  Name compiler

//    ?UNIQUE	( a -- a )
// 	Display a warning message if the word already exists.

	.word	_DOTQ-MAPOFFSET
_UNIQU:	.byte  7
	.ascii "?UNIQUE"
	.align 2 	
UNIQU:
	_NEST
	BL	DUPP
	BL	NAMEQ			// ?name exists
	BL	QBRAN
	.word	UNIQ1-MAPOFFSET	// redefinitions are OK
	BL	DOTQP
	.byte	7
	.ascii " reDef "		// but warn the user
	.align 2 	
	BL	OVER
	BL	COUNT
	BL	TYPEE			// just in case its not planned
UNIQ1:
	BL	DROP
	_UNNEST

//    $,n	 ( na -- )
// 	Build a new dictionary name using the data at na.

// 	.word	_UNIQU-MAPOFFSET
// _SNAME	.byte  3
// 	.ascii "$$,n"
// 	.align 2 	
SNAME:
	_NEST
	BL	DUPP			//  na na
	BL	CAT			//  ?null input
	BL	QBRAN
	.word	SNAM1-MAPOFFSET
	BL	UNIQU			//  na
	BL	LAST			//  na last
	BL	AT			//  na la
	BL	COMMA			//  na
	BL	DUPP			//  na na
	BL	LAST			//  na na last
	BL	STORE			//  na , save na for vocabulary link
	BL	COUNT			//  na+1 count
	BL	PLUS			//  na+1+count
	BL	ALGND			//  word boundary
	BL	CPP
	BL	STORE			//  top of dictionary now
	_UNNEST
SNAM1:
	BL	STRQP
	.byte	7
	.ascii " name? "
	B.W	ABORT

//    $COMPILE	( a -- )
// 	Compile next word to code dictionary as a token or literal.

	.word	_UNIQU-MAPOFFSET
_SCOMP:	.byte  8
	.ascii "$$COMPILE"
	.align 2 	
SCOMP:
	_NEST
	BL	NAMEQ
	BL	QDUP	// defined?
	BL	QBRAN
	.word	SCOM2-MAPOFFSET
	BL	AT
	_DOLIT
	.word	IMEDD
	BL	ANDD	// immediate?
	BL	QBRAN
	.word	SCOM1-MAPOFFSET
	BL	EXECU
	_UNNEST			// it's immediate, execute
SCOM1:
	BL	CALLC			// it's not immediate, compile
	_UNNEST	
SCOM2:
	BL	NUMBQ
	BL	QBRAN
	.word	SCOM3-MAPOFFSET
	BL	LITER
	_UNNEST			// compile number as integer
SCOM3:
	B.W	ABORT			// error

//    OVERT	( -- )
// 	Link a new word into the current vocabulary.

	.word	_SCOMP-MAPOFFSET
_OVERT:	.byte  5
	.ascii "OVERT"
	.align 2 	
OVERT:
	_NEST
	BL	LAST
	BL	AT
	BL	CNTXT
	BL	STORE
	_UNNEST

//    ; 	   ( -- )
// 	Terminate a colon definition.

	.word	_OVERT-MAPOFFSET
_SEMIS:	.byte  IMEDD+COMPO+1
	.ascii "// "
	.align 2 	
SEMIS:
	_NEST
	_DOLIT
	_UNNEST
	BL	COMMA
	BL	LBRAC
	BL	OVERT
	_UNNEST

//    ]	   ( -- )
// 	Start compiling the words in the input stream.

	.word	_SEMIS-MAPOFFSET
_RBRAC:	.byte  1
	.ascii "]"
	.align 2 	
RBRAC:
	_NEST
	_DOLIT
	.word	SCOMP-MAPOFFSET
	BL	TEVAL
	BL	STORE
	_UNNEST

//    BL.W	( ca -- )
// 	Assemble a branch-link long instruction to ca.
// 	BL.W is split into 2 16 bit instructions with 11 bit address fields.

// 	.word	_RBRAC-MAPOFFSET
// _CALLC	.byte  5
// 	.ascii "call,"
// 	.align 2 	
CALLC:
	_NEST
	BIC	R5,R5,#1		//  clear b0 of address from R>
	BL	HERE
	BL	SUBB
	SUB	R5,R5,#4		//  pc offset
	MOVW	R0,#0x7FF		//  11 bit mask
	MOV	R4,R5
	LSR	R5,R5,#12		//  get bits 22-12
	AND	R5,R5,R0
	LSL	R4,R4,#15		//  get bits 11-1
	ORR	R5,R5,R4
	ORR	R5,R5,#0xF8000000
	ORR	R5,R5,#0xF000
	BL	COMMA			//  assemble BL.W instruction
	_UNNEST


// 	:	( -- //  string> )
// 	Start a new colon definition using next word as its name.

	.word	_RBRAC-MAPOFFSET
_COLON:	.byte  1
	.ascii ":"
	.align 2 	
COLON:
	_NEST
	BL	TOKEN
	BL	SNAME
	_DOLIT
	_NEST
	BL	COMMA
	BL	RBRAC
	_UNNEST

//    IMMEDIATE   ( -- )
// 	Make the last compiled word an immediate word.

	.word	_COLON-MAPOFFSET
_IMMED:	.byte  9
	.ascii "IMMEDIATE"
	.align 2 	
IMMED:
	_NEST
	_DOLIT
	.word	IMEDD
	BL	LAST
	BL	AT
	BL	AT
	BL	ORR
	BL	LAST
	BL	AT
	BL	STORE
	_UNNEST

// **************************************************************************
//  Defining words

//    CONSTANT	( u -- //  string> )
// 	Compile a new constant.

	.word	_IMMED-MAPOFFSET
_CONST:	.byte  8
	.ascii "CONSTANT"
	.align 2 	
CONST:
	_NEST
	BL	TOKEN
	BL	SNAME
	BL	OVERT
	_DOLIT
	_NEST
	BL	COMMA
	_DOLIT
	.word	DOCON-MAPOFFSET
	BL	CALLC
	BL	COMMA
	_UNNEST

//    CREATE	( -- //  string> )
// 	Compile a new array entry without allocating code space.

	.word	_CONST-MAPOFFSET
_CREAT:	.byte  6
	.ascii "CREATE"
	.align 2 	
CREAT:
	_NEST
	BL	TOKEN
	BL	SNAME
	BL	OVERT
	_DOLIT
	_NEST
	BL	COMMA
	_DOLIT
	.word	DOVAR-MAPOFFSET
	BL	CALLC
	_UNNEST

//    VARIABLE	( -- //  string> )
// 	Compile a new variable initialized to 0.

	.word	_CREAT-MAPOFFSET
_VARIA:	.byte  8
	.ascii "VARIABLE"
	.align 2 	
VARIA:
	_NEST
	BL	CREAT
	_DOLIT
	.word	0
	BL	COMMA
	_UNNEST

// **************************************************************************
//  Tools

//    dm+	 ( a u -- a )
// 	Dump u bytes from , leaving a+u on the stack.

// 	.word	_VARIA-MAPOFFSET
// _DMP	.byte  3
// 	.ascii "dm+"
// 	.align 2 	
DMP:
	_NEST
	BL	OVER
	_DOLIT
	.word	4
	BL	UDOTR			// display address
	BL	SPACE
	BL	TOR			// start count down loop
	B.W	PDUM2			// skip first pass
PDUM1:
  BL	DUPP
	BL	CAT
	_DOLIT
	.word	3
	BL	UDOTR			// display numeric data
	BL	ONEP			// increment address
PDUM2:
  BL	DONXT
	.word	PDUM1-MAPOFFSET	// loop till done
	_UNNEST
	.align 2 
//    DUMP	( a u -- )
// 	Dump u bytes from a, in a formatted manner.

	.word	_VARIA-MAPOFFSET
_DUMP:	.byte  4
	.ascii "DUMP"
	.align 2 	
DUMP:
	_NEST
	BL	BASE
	BL	AT
	BL	TOR
	BL	HEX			// save radix,set hex
	_DOLIT
	.word	16
	BL	SLASH			// change count to lines
	BL	TOR
	B.W	DUMP4			// start count down loop
DUMP1:
  BL	CR
	_DOLIT
	.word	16
	BL	DDUP
	BL	DMP			// display numeric
	BL	ROT
	BL	ROT
	BL	SPACE
	BL	SPACE
	BL	TYPEE			// display printable characters
DUMP4:
  BL	DONXT
	.word	DUMP1-MAPOFFSET	// loop till done
DUMP3:
  BL	DROP
	BL	RFROM
	BL	BASE
	BL	STORE			// restore radix
	_UNNEST

//    .S	  ( ... -- ... )
// 	Display the contents of the data stack.

	.word	_DUMP-MAPOFFSET
_DOTS:
	.byte  2
	.ascii ".S"
	.align 2 	
DOTS:
	_NEST
	BL	SPACE
	BL	DEPTH			// stack depth
	BL	TOR			// start count down loop
	B.W	DOTS2			// skip first pass
DOTS1:
  BL	RAT
	BL	PICK
	BL	DOT			// index stack, display contents
DOTS2:
  BL	DONXT
	.word	DOTS1-MAPOFFSET	// loop till done
	BL	SPACE
	_UNNEST

//    >NAME	( ca -- na | F )
// 	Convert code address to a name address.

	.word	_DOTS-MAPOFFSET
_TNAME:	.byte  5
	.ascii ">NAME"
	.align 2 	
TNAME:
	_NEST
	BL	TOR			//  
	BL	CNTXT			//  va
	BL	AT			//  na
TNAM1:
	BL	DUPP			//  na na
	BL	QBRAN
	.word	TNAM2-MAPOFFSET	//  vocabulary end, no match
	BL	DUPP			//  na na
	BL	NAMET			//  na ca
	BL	RAT			//  na ca code
	BL	XORR			//  na f --
	BL	QBRAN
	.word	TNAM2-MAPOFFSET
	BL	CELLM			//  la 
	BL	AT			//  next_na
	B.W	TNAM1
TNAM2:	
	BL	RFROM
	BL	DROP			//  0|na --
	_UNNEST			// 0

//    .ID	 ( na -- )
// 	Display the name at address.

	.word	_TNAME-MAPOFFSET
_DOTID:	.byte  3
	.ascii ".ID"
	.align 2 	
DOTID:
	_NEST
	BL	QDUP			// if zero no name
	BL	QBRAN
	.word	DOTI1-MAPOFFSET
	BL	COUNT
	_DOLIT
	.word	0x1F
	BL	ANDD			// mask lexicon bits
	BL	TYPEE
	_UNNEST			// display name string
DOTI1:
  BL	DOTQP
	.byte	9
	.ascii " {noName}"
	.align 2 	
	_UNNEST

//    SEE	 ( -- //  string> )
// 	A simple decompiler.

	.word	_DOTID-MAPOFFSET
_SEE:	.byte  3
	.ascii "SEE"
	.align 2 	
SEE:
	_NEST
	BL	TICK	//  ca --, starting address
	BL	CR	
	_DOLIT
	.word	20
	BL	TOR
SEE1:
	BL	CELLP			//  a
	BL	DUPP			//  a a
	BL	DECOMP		//  a
	BL	DONXT
	.word	SEE1-MAPOFFSET
	BL	DROP
	_UNNEST

// 	DECOMPILE ( a -- )
// 	Convert code in a.  Display name of command or as data.

	.word	_SEE-MAPOFFSET
_DECOM:	.byte  9
	.ascii "DECOMPILE"
	.align 2 
	
DECOMP:	
	_NEST
	BL	DUPP			//  a a
// 	BL	TOR			//  a
	BL	AT			//  a code
	BL	DUPP			//  a code code
	_DOLIT
	.word	0xF800F800
	BL	ANDD
	_DOLIT
	.word	0xF800F000
	BL	EQUAL			//  a code ?
	BL	QBRAN
	.word	DECOM2-MAPOFFSET	//  not a command
	//  a valid_code --, extract address and display name
	MOVW	R0,#0xFFE
	MOV	R4,R5
	LSL	R5,R5,#21		//  get bits 22-12
	ASR	R5,R5,#9		//  with sign extension
	LSR	R4,R4,#15		//  get bits 11-1
	AND	R4,R4,R0		//  retain only bits 11-1
	ORR	R5,R5,R4		//  get bits 22-1
	NOP
	BL	OVER			//  a offset a
	BL	PLUS			//  a target-4
	BL	CELLP			//  a target
	BL	TNAME			//  a na/0 --, is it a name?
	BL	QDUP			//  name address or zero
	BL	QBRAN
	.word	DECOM1-MAPOFFSET
	BL	SPACE			//  a na
	BL	DOTID			//  a --, display name
// 	BL	RFROM			//  a
	BL	DROP
	_UNNEST
DECOM1:	// BL	RFROM		//  a
	BL	AT			//  data
	BL	UDOT			//  display data
	_UNNEST
DECOM2:
	BL	UDOT
// 	BL	RFROM
	BL	DROP
	_UNNEST

//    WORDS	( -- )
// 	Display the names in the context vocabulary.

	.word	_DECOM-MAPOFFSET
_WORDS:	.byte  5
	.ascii "WORDS"
	.align 2 	
WORDS:
	_NEST
	BL	CR
	BL	CNTXT
	BL	AT			// only in context
WORS1:
	BL	QDUP			// ?at end of list
	BL	QBRAN
	.word	WORS2-MAPOFFSET
	BL	DUPP
	BL	SPACE
	BL	DOTID			// display a name
	BL	CELLM
	BL	AT
	B.W	WORS1
WORS2:
	_UNNEST

// **************************************************************************
//  cold start

//    VER	 ( -- n )
// 	Return the version number of this implementation.

// 	.word	_WORDS-MAPOFFSET
// _VERSN	.byte  3
// 	.ascii "VER"
// 	.align 2 	
VERSN:
	_NEST
	_DOLIT
	.word	VER*256+EXT
	_UNNEST

//    hi	  ( -- )
// 	Display the sign-on message of eForth.

	.word	_WORDS-MAPOFFSET
_HI:	.byte  2
	.ascii "HI"
	.align 2 	
HI:
	_NEST
	BL	CR	// initialize I/O
	BL	DOTQP
	.byte	23
	.ascii "blue pill stm32eForth v"	// model
	.align 2
	BL	BASE
	BL	AT
	BL	HEX	// save radix
	BL	VERSN
	BL	BDIGS
	BL	DIG
	BL	DIG
	_DOLIT
	.word	'.'
	BL	HOLD
	BL	DIGS
	BL	EDIGS
	BL	TYPEE	// format version number
	BL	BASE
	BL	STORE
	BL	CR
	_UNNEST			// restore radix

//    COLD	( -- )
// 	The high level cold start sequence.

	.word	_HI-MAPOFFSET
LASTN:	.byte  4
	.ascii "COLD"
	.align 2 	
	.type COLD,%function 
COLD:
//  Initiate Forth registers
	MOV R3,#UPP&0xffff	//  user area 
 	MOVT R3,#UPP>>16		  
	ADD R2,R3,#0x4f80	// Forth return stack
	ADD R1,R3,#0x4E80 // Forth data stack
	MOV R5,#0			//  tos
	NOP
	_NEST
COLD1:
	_DOLIT 
	.word 0 
	BL ULED // turn off user LED 
	_DOLIT
	.word	UZERO-MAPOFFSET
	_DOLIT
	.word	UPP
	_DOLIT
	.word	ULAST-UZERO
	BL	MOVE 			// initialize user area
	BL	PRESE			// initialize stack and TIB
	BL	TBOOT
	BL	ATEXE			// application boot
	BL	OVERT
	B.W	QUIT			// start interpretation
COLD2:	
CTOP:
	.word	0XFFFFFFFF		//  keep CTOP even


  .end 
