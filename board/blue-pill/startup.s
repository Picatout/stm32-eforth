/*
  console I/O use USART1 
  user LED on PC:13
  external crystal frequency 8 Mhz
*/

  .syntax unified
  .cpu cortex-m3
  .fpu softvfp
  .thumb


  .include "stm32f103.inc"

  /* crystal frequency */
  .equ FXTL,8000000

  /* user LED port */
  .equ USER_LED_PORT, GPIOC
  .equ USER_LED_BIT, (13)

  .macro _led_on 
  ldr r0,=USER_LED_PORT 
  ldr r1,#1<<USER_LED_BIT 
  str r1,[r0,#GPIO_BSRR]
  .endm

  .macro _led_off
  ldr r0,=USER_LED_PORT
  ldr r1,#1<<USER_LED_BIT
  str r1,[r0,GPIO_BRR]
  .endm 

  /* I/O UART */
  .equ IO_UART, USART1_BASE_ADR 
  .equ UART_PORT, GPIOA
  .equ UART_TX_BIT, (9)
  .equ UART_RX_BIT, (10)


  .global  isr_vectors
  .global  default_handler
  .global  reset_handler

   .section  .isr_vector,"a",%progbits
  .type  isr_vectors, %object
  .size  isr_vectors, .-isr_vectors

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
  .word   default_handler  /* Systick */
  
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

/*****************************************************
* default isr handler called on unexpected interrupt
*****************************************************/
    .section  .text.default_handler,"ax",%progbits

  .type default_handler, %function
default_handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  default_handler, .-default_handler

/**************************************
  Reset_Handler execute at MCU reset
***************************************/
    .section  .text.reset_handler,"ax",%progbits
  .type  reset_handler, %function
reset_handler:  
	bl	init_devices	 	/* RCC, GPIOs, USART */



init_devices:

