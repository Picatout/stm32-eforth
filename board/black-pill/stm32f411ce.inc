/*
   STM32F103 registers
*/
    /* RAM */
    .equ RAM_ADR, 0x20000000 
    .equ RAM_SIZE, 0x20000
    .equ RAM_END, RAM_ADR+RAM_SIZE

   /* RAM bit band alias */
      .equ RAM_BIT_ALIAS, 0x22000000

    /* FLASH MEMORY */
    /* memory reported by MCU */
    .equ FLASH_ADR,  0x8000000 
    .equ FLASH_SIZE, 0x0080000
    .equ FLASH_END, FLASH_ADR+FLASH_SIZE
     // erase block size 
    .equ SECTOR0_SIZE, 16384  // 0x0800 0000 - 0x0800 3FFF
    .equ SECTOR1_SIZE, 16384  // 0x0800 4000 - 0x0800 7FFF
    .equ SECTOR2_SIZE, 16384  // 0x0800 8000 - 0x0800 BFFF
    .equ SECTOR3_SIZE, 16384  // 0x0800 C000 - 0x0800 FFFF
    .equ SECTOR4_SIZE, 65536  // 0x0801 0000 - 0x0801 FFFF
    .equ SECTOR5_SIZE, 131072 // 0x0802 0000 - 0x0803 FFFF
    .equ SECTOR6_SIZE, 131072 // 0x0804 0000 - 0x0805 FFFF
    .equ SECTOR7_SIZE, 131072 // 0x0806 0000 - 0x0807 FFFF


   /* system memory 0x1FFF 0000 - 0x1FFF 77FF */
      .equ SYS_MEM, 0x1FFFF000
      .equ SYS_MEM_SIZE, 0x77FF 

   /* OTP memory 0x1FFF 7800 - 0x1FFF 7A0F */
      .equ OTP_MEM, 0x1FFF7800
      .equ OTP_SIZE, 0x210 

   /* option memory 0x1FFF C000 - 0x1FFF C00F */
      .equ OPTION, 0x1FFFC000   
      .equ OPT_SIZE, 16


   /* peripherals base address */
      .equ PER_BASE_ADR,  0x40000000
   /* PERIPHERALS bit band alias */
      .equ PER_BIT_ALIAS, 0x42000000 


    /* RCC registers address */
    .equ RCC_BASE_ADR, 0x40023800
    /* RCC registers offset */
    .equ RCC_CR, 0
    .equ RCC_PLLCFGR,0X4
    .equ RCC_CFGR, 0X8
    .equ RCC_CIR, 0XC
    .equ RCC_AHB1RSTR, 0X10
    .equ RCC_AHB2RSTR, 0X14
    .equ RCC_APB1RSTR, 0x20
    .equ RCC_APB2RSTR, 0X24
    .equ RCC_AHB1ENR, 0X30
    .equ RCC_AHB2ENR, 0X34 
    .equ RCC_APB1ENR, 0X40
    .equ RCC_APB2ENR, 0X44
    .equ RCC_AHB1LPENR,0X50
    .equ RCC_AHB2LPENR,0X54
    .equ RCC_APB1LPENR,0X60
    .equ RCC_APB2LPENR,0X64
    .equ RCC_BDCR,0X70
    .equ RCC_CSR,0X74
    .equ RCC_SSCGR,0X80
    .equ RCC_PLLI2SCFGR,0X84
    .equ RCC_DCKCFGR, 0X8C 

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
