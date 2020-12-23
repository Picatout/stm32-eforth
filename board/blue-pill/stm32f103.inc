/*
   STM32F103 registers
*/
    /* RAM */
    .equ RAM_ADR, 0x20000000 
    .equ RAM_SIZE, 0x5000
    .equ RAM_END, RAM_ADR+RAM_SIZE

   /* RAM bit band alias */
      .equ RAM_BIT_ALIAS, 0x22000000

    /* FLASH MEMORY */
    /* memory reported by MCU */
    .equ FLASH_ADR, 0x8000000 
    .equ FLASH_SIZE, 0x10000
    .equ FLASH_END, FLASH_ADR+FLASH_SIZE
    /* the MCU as 64K flash not reported by MCU */
    .equ FLASH_HIDDEN_ADR, 0x8010000
    .equ FLASH_HIDDEN_SIZE, 0x10000
    .equ FLASH_HIDDEN_END, FLASH_HIDDEN_ADR+FLASH_HIDDEN_SIZE
    .equ PAGE_SIZE, 1024 // erase block size 

   /* system memory */
      .equ SYS_MEM, 0x1FFFF000
      .equ SYS_MEM_SIZE, 0x800

   /* option memory */
      .equ OPTION, 0x1FFFF800   
      .equ OPT_SIZE, 16


   /* peripherals base address */
      .equ PER_BASE_ADR,  0x40000000
   /* PERIPHERALS bit band alias */
      .equ PER_BIT_ALIAS, 0x42000000 


    /* RCC registers address */
    .equ RCC_BASE_ADR, 0x40021000
    /* RCC registers offset */
    .equ RCC_CR, 0
    .equ RCC_CFGR, 4
    .equ RCC_CIR, 8
    .equ RCC_APB2RSTR, 12
    .equ RCC_APB1RSTR, 16
    .equ RCC_AHBENR, 20
    .equ RCC_APB2ENR, 24
    .equ RCC_APB1ENR, 28
    .equ RCC_BDCR, 32 
    .equ RCC_CSR, 36

    /* FLASH registers address */
    .equ FLASH_BASE_ADR, 0x40022000
    /* FLASH registers offset */
    .equ FLASH_ACR, 0
    .equ FLASH_KEYR, 4
    .equ FLASH_OPTKEYR, 8
    .equ FLASH_SR, 12
    .equ FLASH_CR, 16
    .equ FLASH_AR, 20
    .equ FLASH_OBR, 28
    .equ FLASH_WRPR,32
    .equ RDPRT_KEY, 0x00A5
    .equ FLASH_KEY1, 0x45670123
    .equ FLASH_KEY2, 0xCDEF89AB

    /* GPIOx base address */
    .equ GPIOA_BASE_ADR, 0x40010800
    .equ GPIOB_BASE_ADR, 0x40010C00
    .equ GPIOC_BASE_ADR, 0x40011000
    .equ GPIOD_BASE_ADR, 0x40011400
    .equ GPIOE_BASE_ADR, 0x40011800
    .equ GPIOF_BASE_ADR, 0x40018C00
    .equ GPIOG_BASE_ADR, 0x40012000

    /* gpiox registers offset from base address */
    .equ GPIO_CRL, 0
    .equ GPIO_CRH, 4
    .equ GPIO_IDR, 8
    .equ GPIO_ODR, 12
    .equ GPIO_BSRR, 16
    .equ GPIO_BRR, 20
    .equ GPIO_LOCKR, 24 
    

    /* USART1 registers */
    .equ USART1_BASE_ADR, 0x40013800
    /* USARTx registers offset */
    .equ USART_SR, 0
    .equ USART_DR,4
    .equ USART_BRR,8
    .equ USART_CR1,12
    .equ USART_CR2,16
    .equ USART_CR3,20
    .equ USART_GTPR,24

   /* systick */
    .equ STK_BASE_ADR, 0xE000E010
    /* registers offset */
    .equ STK_CTL, 0
    .equ STK_LOAD, 4
    .equ STK_VAL, 8
    .equ STK_CALIB, 12

   /* system control block */
   .equ SCB_BASE_ADR, 0xE000ED00
   /* registers offset */
   .equ SCB_CPUID, 0
   .equ SCB_ICSR, 4 
   .equ SCB_VTOR, 8
   .equ SCB_AIRCR, 12
   .equ SCB_SCR, 16
   .equ SCB_CCR, 20
   .equ SCB_SHPR1,24
   .equ SCB_SHPR2,28
   .equ SCB_SHPR3,32
   .equ SCB_SHCRS,36
   .equ SCB_CFSR,40
   .equ SCB_HFSR,44
   .equ SCB_MMAR,52
   .equ SCB_BFAR,56
   // key to to write in SCB_AIRCR 
   .equ SCB_VECTKEY,0x5fa 
   
   /* NVIC block */
   .equ NVIC_BASE_ADR, 0xE000E100
   .equ NVIC_ISER0, 0
   .equ NVIC_ISER1, 4
   .equ NVIC_ISER2, 8
   .equ NVIC_ICER0, 0x80
   .equ NVIC_ICER1, 0x84 
   .equ NVIC_ICER2, 0x88 
   .equ NVIC_ISPR0, 0x100
   .equ NVIC_ISPR1, 0x104
   .equ NVIC_ISPR2, 0x108 
   .equ NVIC_ICPR0, 0x180
   .equ NVIC_ICPR1, 0x184
   .equ NVIC_ICPR2, 0x188
   .equ NVIC_IABR0, 0x200
   .equ NVIC_IABR1, 0x204
   .equ NVIC_IABR2, 0x208
   .equ NVIC_IPR_BASE, 0x300 
