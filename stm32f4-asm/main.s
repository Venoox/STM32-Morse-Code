// STM32F4 Discovery - Assembly template for CO LAB course
//
// Based on: https://github.com/fcayci/stm32f4-assembly
//
// Turns on an LED attached to GPIOD Pin 12
// We need to enable the clock for GPIOD and set up pin 12 as output.

// Start with enabling thumb 32 mode since Cortex-M4 do not work with arm mode
// Unified syntax is used to enable good of the both words...

// Make sure to run arm-none-eabi-objdump.exe -d prj1.elf to check if
// the assembler used proper instructions. (Like ADDS)

.thumb
.syntax unified
//.arch armv7e-m

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////
// Definitions section. Define all the registers and
// constants here for code readability.

// Constants
.equ ENTER_ASCII, 0xD
// Register Addresses
// You can find the base addresses for all peripherals from Memory Map section
// RM0090 on page 64. Then the offsets can be found on their relevant sections.

// RCC base address is 0x40023800
.equ RCC_BASE, 0x40023800
.equ RCC_AHB1ENR, 0x30 // RCC AHB1 peripheral clock reg (page 180)
.equ RCC_APB1ENR, 0x40 // RCC APB1 peripheral clock reg (page 183)

// GPIOD base address is 0x40020C00
.equ GPIOD_BASE, 0x40020C00 // GPIOD base address)
.equ GPIOD_MODER, 0x00 // GPIOD port mode register (page 281)
.equ GPIOD_ODR, 0x14 // GPIOD output data register (page 283)
.equ GPIOD_BSSR, 0x18 // GPIOD port set/reset register (page 284)
.equ GPIOD_AFRL, 0x20 // GPIOD output data register (page 285)
.equ GPIOD_AFRL_VALUE, 0x07700000 // AF7 on PD5,6

// TIM6
.equ	 TIM6_BASE,    0x40001000
.equ     TIM6_CR1,     0x00 // control register
.equ     TIM6_CNT,     0x24 // timer value
.equ     TIM6_PSC,     0x28 // prescaler
.equ     TIM6_ARR,     0x2c // autoreload
.equ     TIM6_EGR,     0x14
.equ     TIM6_SR,      0x10

// USART2 base address is 0x40004400
.equ USART2_BASE, 0x40004400 // USART2 base address)
.equ USART2_SR, 0x00 // SR register (page 1007)
.equ USART2_DR, 0x04 // SR register (page 1007)
.equ USART2_BRR, 0x08 // BRR register (page 1010)
.equ USART2_BRR_VAL, (52<<4)+0b1010 // BRR register
.equ USART2_CR1, 0x0C // CR1 register
.equ USART2_CR1_VAL, 0x200C // CR1 register

.equ NVIC_BASE, 0xe000e100
.equ NVIC_ISER1, 0x4

.section .bss
RX_NIZ: .space 64

// Start of text section
.section .text
///////////////////////////////////////////////////////////////////////////////
// Vectors
///////////////////////////////////////////////////////////////////////////////
// Vector table start
// Add all other processor specific exceptions/interrupts in order here
	.long    __StackTop                 // Top of the stack. from linker script
	.long    _start +1                  // reset location, +1 for thumb mode
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler

  /* External Interrupts */
  .word     WWDG_IRQHandler                   /* Window WatchDog              */
  .word     PVD_IRQHandler                    /* PVD through EXTI Line detection */
  .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */
  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */
  .word     FLASH_IRQHandler                  /* FLASH                        */
  .word     RCC_IRQHandler                    /* RCC                          */
  .word     EXTI0_IRQHandler                  /* EXTI Line0                   */
  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */
  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */
  .word     EXTI3_IRQHandler                  /* EXTI Line3                   */
  .word     EXTI4_IRQHandler                  /* EXTI Line4                   */
  .word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */
  .word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */
  .word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */
  .word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */
  .word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */
  .word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */
  .word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */
  .word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */
  .word     CAN1_TX_IRQHandler                /* CAN1 TX                      */
  .word     CAN1_RX0_IRQHandler               /* CAN1 RX0                     */
  .word     CAN1_RX1_IRQHandler               /* CAN1 RX1                     */
  .word     CAN1_SCE_IRQHandler               /* CAN1 SCE                     */
  .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */
  .word     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */
  .word     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */
  .word     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  .word     TIM2_IRQHandler                   /* TIM2                         */
  .word     TIM3_IRQHandler                   /* TIM3                         */
  .word     TIM4_IRQHandler                   /* TIM4                         */
  .word     I2C1_EV_IRQHandler                /* I2C1 Event                   */
  .word     I2C1_ER_IRQHandler                /* I2C1 Error                   */
  .word     I2C2_EV_IRQHandler                /* I2C2 Event                   */
  .word     I2C2_ER_IRQHandler                /* I2C2 Error                   */
  .word     SPI1_IRQHandler                   /* SPI1                         */
  .word     SPI2_IRQHandler                   /* SPI2                         */
  .word     USART1_IRQHandler                 /* USART1                       */
  .word     USART2_IRQHandler                 /* USART2                       */
  .word     USART3_IRQHandler                 /* USART3                       */
  .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */
  .word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */
  .word     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */
  .word     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */
  .word     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  .word     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */
  .word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */
  .word     FSMC_IRQHandler                   /* FSMC                         */
  .word     SDIO_IRQHandler                   /* SDIO                         */
  .word     TIM5_IRQHandler                   /* TIM5                         */
  .word     SPI3_IRQHandler                   /* SPI3                         */
  .word     UART4_IRQHandler                  /* UART4                        */
  .word     UART5_IRQHandler                  /* UART5                        */
  .word     TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */
  .word     TIM7_IRQHandler                   /* TIM7                         */
  .word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */
  .word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */
  .word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */
  .word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */
  .word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */
  .word     ETH_IRQHandler                    /* Ethernet                     */
  .word     ETH_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */
  .word     CAN2_TX_IRQHandler                /* CAN2 TX                      */
  .word     CAN2_RX0_IRQHandler               /* CAN2 RX0                     */
  .word     CAN2_RX1_IRQHandler               /* CAN2 RX1                     */
  .word     CAN2_SCE_IRQHandler               /* CAN2 SCE                     */
  .word     OTG_FS_IRQHandler                 /* USB OTG FS                   */
  .word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */
  .word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */
  .word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */
  .word     USART6_IRQHandler                 /* USART6                       */
  .word     I2C3_EV_IRQHandler                /* I2C3 event                   */
  .word     I2C3_ER_IRQHandler                /* I2C3 error                   */
  .word     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */
  .word     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */
  .word     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */
  .word     OTG_HS_IRQHandler                 /* USB OTG HS                   */
  .word     DCMI_IRQHandler                   /* DCMI                         */
  .word     0                                 /* CRYP crypto                  */
  .word     HASH_RNG_IRQHandler               /* Hash and Rng                 */
  .word     FPU_IRQHandler                    /* FPU                          */


/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler

   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler

   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler

   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler

   .weak      WWDG_IRQHandler
   .thumb_set WWDG_IRQHandler,Default_Handler

   .weak      PVD_IRQHandler
   .thumb_set PVD_IRQHandler,Default_Handler

   .weak      TAMP_STAMP_IRQHandler
   .thumb_set TAMP_STAMP_IRQHandler,Default_Handler

   .weak      RTC_WKUP_IRQHandler
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler

   .weak      FLASH_IRQHandler
   .thumb_set FLASH_IRQHandler,Default_Handler

   .weak      RCC_IRQHandler
   .thumb_set RCC_IRQHandler,Default_Handler

   .weak      EXTI0_IRQHandler
   .thumb_set EXTI0_IRQHandler,Default_Handler

   .weak      EXTI1_IRQHandler
   .thumb_set EXTI1_IRQHandler,Default_Handler

   .weak      EXTI2_IRQHandler
   .thumb_set EXTI2_IRQHandler,Default_Handler

   .weak      EXTI3_IRQHandler
   .thumb_set EXTI3_IRQHandler,Default_Handler

   .weak      EXTI4_IRQHandler
   .thumb_set EXTI4_IRQHandler,Default_Handler

   .weak      DMA1_Stream0_IRQHandler
   .thumb_set DMA1_Stream0_IRQHandler,Default_Handler

   .weak      DMA1_Stream1_IRQHandler
   .thumb_set DMA1_Stream1_IRQHandler,Default_Handler

   .weak      DMA1_Stream2_IRQHandler
   .thumb_set DMA1_Stream2_IRQHandler,Default_Handler

   .weak      DMA1_Stream3_IRQHandler
   .thumb_set DMA1_Stream3_IRQHandler,Default_Handler

   .weak      DMA1_Stream4_IRQHandler
   .thumb_set DMA1_Stream4_IRQHandler,Default_Handler

   .weak      DMA1_Stream5_IRQHandler
   .thumb_set DMA1_Stream5_IRQHandler,Default_Handler

   .weak      DMA1_Stream6_IRQHandler
   .thumb_set DMA1_Stream6_IRQHandler,Default_Handler

   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,Default_Handler

   .weak      CAN1_TX_IRQHandler
   .thumb_set CAN1_TX_IRQHandler,Default_Handler

   .weak      CAN1_RX0_IRQHandler
   .thumb_set CAN1_RX0_IRQHandler,Default_Handler

   .weak      CAN1_RX1_IRQHandler
   .thumb_set CAN1_RX1_IRQHandler,Default_Handler

   .weak      CAN1_SCE_IRQHandler
   .thumb_set CAN1_SCE_IRQHandler,Default_Handler

   .weak      EXTI9_5_IRQHandler
   .thumb_set EXTI9_5_IRQHandler,Default_Handler

   .weak      TIM1_BRK_TIM9_IRQHandler
   .thumb_set TIM1_BRK_TIM9_IRQHandler,Default_Handler

   .weak      TIM1_UP_TIM10_IRQHandler
   .thumb_set TIM1_UP_TIM10_IRQHandler,Default_Handler

   .weak      TIM1_TRG_COM_TIM11_IRQHandler
   .thumb_set TIM1_TRG_COM_TIM11_IRQHandler,Default_Handler

   .weak      TIM1_CC_IRQHandler
   .thumb_set TIM1_CC_IRQHandler,Default_Handler

   .weak      TIM2_IRQHandler
   .thumb_set TIM2_IRQHandler,Default_Handler

   .weak      TIM3_IRQHandler
   .thumb_set TIM3_IRQHandler,Default_Handler

   .weak      TIM4_IRQHandler
   .thumb_set TIM4_IRQHandler,Default_Handler

   .weak      I2C1_EV_IRQHandler
   .thumb_set I2C1_EV_IRQHandler,Default_Handler

   .weak      I2C1_ER_IRQHandler
   .thumb_set I2C1_ER_IRQHandler,Default_Handler

   .weak      I2C2_EV_IRQHandler
   .thumb_set I2C2_EV_IRQHandler,Default_Handler

   .weak      I2C2_ER_IRQHandler
   .thumb_set I2C2_ER_IRQHandler,Default_Handler

   .weak      SPI1_IRQHandler
   .thumb_set SPI1_IRQHandler,Default_Handler

   .weak      SPI2_IRQHandler
   .thumb_set SPI2_IRQHandler,Default_Handler

   .weak      USART1_IRQHandler
   .thumb_set USART1_IRQHandler,Default_Handler

   .weak      USART2_IRQHandler
   //.thumb_set USART2_IRQHandler,Default_Handler

   .weak      USART3_IRQHandler
   .thumb_set USART3_IRQHandler,Default_Handler

   .weak      EXTI15_10_IRQHandler
   .thumb_set EXTI15_10_IRQHandler,Default_Handler

   .weak      RTC_Alarm_IRQHandler
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler

   .weak      OTG_FS_WKUP_IRQHandler
   .thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler

   .weak      TIM8_BRK_TIM12_IRQHandler
   .thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler

   .weak      TIM8_UP_TIM13_IRQHandler
   .thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler

   .weak      TIM8_TRG_COM_TIM14_IRQHandler
   .thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler

   .weak      TIM8_CC_IRQHandler
   .thumb_set TIM8_CC_IRQHandler,Default_Handler

   .weak      DMA1_Stream7_IRQHandler
   .thumb_set DMA1_Stream7_IRQHandler,Default_Handler

   .weak      FSMC_IRQHandler
   .thumb_set FSMC_IRQHandler,Default_Handler

   .weak      SDIO_IRQHandler
   .thumb_set SDIO_IRQHandler,Default_Handler

   .weak      TIM5_IRQHandler
   .thumb_set TIM5_IRQHandler,Default_Handler

   .weak      SPI3_IRQHandler
   .thumb_set SPI3_IRQHandler,Default_Handler

   .weak      UART4_IRQHandler
   .thumb_set UART4_IRQHandler,Default_Handler

   .weak      UART5_IRQHandler
   .thumb_set UART5_IRQHandler,Default_Handler

   .weak      TIM6_DAC_IRQHandler
   .thumb_set TIM6_DAC_IRQHandler,Default_Handler

   .weak      TIM7_IRQHandler
   .thumb_set TIM7_IRQHandler,Default_Handler

   .weak      DMA2_Stream0_IRQHandler
   .thumb_set DMA2_Stream0_IRQHandler,Default_Handler

   .weak      DMA2_Stream1_IRQHandler
   .thumb_set DMA2_Stream1_IRQHandler,Default_Handler

   .weak      DMA2_Stream2_IRQHandler
   .thumb_set DMA2_Stream2_IRQHandler,Default_Handler

   .weak      DMA2_Stream3_IRQHandler
   .thumb_set DMA2_Stream3_IRQHandler,Default_Handler

   .weak      DMA2_Stream4_IRQHandler
   .thumb_set DMA2_Stream4_IRQHandler,Default_Handler

   .weak      ETH_IRQHandler
   .thumb_set ETH_IRQHandler,Default_Handler

   .weak      ETH_WKUP_IRQHandler
   .thumb_set ETH_WKUP_IRQHandler,Default_Handler

   .weak      CAN2_TX_IRQHandler
   .thumb_set CAN2_TX_IRQHandler,Default_Handler

   .weak      CAN2_RX0_IRQHandler
   .thumb_set CAN2_RX0_IRQHandler,Default_Handler

   .weak      CAN2_RX1_IRQHandler
   .thumb_set CAN2_RX1_IRQHandler,Default_Handler

   .weak      CAN2_SCE_IRQHandler
   .thumb_set CAN2_SCE_IRQHandler,Default_Handler

   .weak      OTG_FS_IRQHandler
   .thumb_set OTG_FS_IRQHandler,Default_Handler

   .weak      DMA2_Stream5_IRQHandler
   .thumb_set DMA2_Stream5_IRQHandler,Default_Handler

   .weak      DMA2_Stream6_IRQHandler
   .thumb_set DMA2_Stream6_IRQHandler,Default_Handler

   .weak      DMA2_Stream7_IRQHandler
   .thumb_set DMA2_Stream7_IRQHandler,Default_Handler

   .weak      USART6_IRQHandler
   .thumb_set USART6_IRQHandler,Default_Handler

   .weak      I2C3_EV_IRQHandler
   .thumb_set I2C3_EV_IRQHandler,Default_Handler

   .weak      I2C3_ER_IRQHandler
   .thumb_set I2C3_ER_IRQHandler,Default_Handler

   .weak      OTG_HS_EP1_OUT_IRQHandler
   .thumb_set OTG_HS_EP1_OUT_IRQHandler,Default_Handler

   .weak      OTG_HS_EP1_IN_IRQHandler
   .thumb_set OTG_HS_EP1_IN_IRQHandler,Default_Handler

   .weak      OTG_HS_WKUP_IRQHandler
   .thumb_set OTG_HS_WKUP_IRQHandler,Default_Handler

   .weak      OTG_HS_IRQHandler
   .thumb_set OTG_HS_IRQHandler,Default_Handler

   .weak      DCMI_IRQHandler
   .thumb_set DCMI_IRQHandler,Default_Handler

   .weak      HASH_RNG_IRQHandler
   .thumb_set HASH_RNG_IRQHandler,Default_Handler

   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,Default_Handler

///////////////
// Constants //
///////////////
ABECEDA:
.ascii ".-"     @ A
.byte 0,0,0,0
.ascii "-..."     @ B
.byte 0,0
.ascii "-.-."     @ C
.byte 0,0
.ascii "-.."     @ D
.byte 0,0,0
.ascii "."     @ E
.byte 0,0,0,0,0
.ascii "..-."     @ F
.byte 0,0
.ascii "--."     @ G
.byte 0,0,0
.ascii "...."     @ H
.byte 0,0
.ascii ".."     @ I
.byte 0,0,0,0
.ascii ".---"     @ J
.byte 0,0
.ascii "-.-"     @ K
.byte 0,0,0
.ascii ".-.."     @ L
.byte 0,0
.ascii "--"     @ M
.byte 0,0,0,0
.ascii "-."     @ N
.byte 0,0,0,0
.ascii "---"     @ O
.byte 0,0,0
.ascii ".--."     @ P
.byte 0,0
.ascii "--.-"     @ Q
.byte 0,0
.ascii ".-."     @ R
.byte 0,0,0
.ascii "..."     @ S
.byte 0,0,0
.ascii "-"     @ T
.byte 0,0,0,0,0
.ascii "..-"     @ U
.byte 0,0,0
.ascii "...-"     @ V
.byte 0,0
.ascii ".--"     @ W
.byte 0,0,0
.ascii "-..-"     @ X
.byte 0,0
.ascii "-.--"     @ Y
.byte 0,0
.ascii "--.."     @ Z
.byte 0,0
	.align
///////////////////////////////////////////////////////////////////////////////
// Main code starts from here
///////////////////////////////////////////////////////////////////////////////

_start:
	bl INIT_IO
	bl INIT_TIM6
	bl INIT_USART2
	// dummy delay, because first delay doesn't work for some reason
	ldr r0, =100
	bl DELAY_TIM6

	ldr r7, =RX_NIZ // register that holds current position when receiving bytes
/*begin_receiving:
	ldr r0, =RX_NIZ
	bl RECV_USART2
	bl XWORD
	b begin_receiving*/
end: b end

// parameter in r0 is memory location to store bytes
RECV_USART2:
  stmfd r13!, {r0-r3, lr}
      ldr r1, =USART2_BASE
  RECV_LP:
      ldr r2, [r1, #USART2_SR]
      tst r2, #(1 << 5)
      beq RECV_LP
      ldr r3, [r1, #USART2_DR]
      strb r3, [r0], #1
      cmp r3, ENTER_ASCII
      bne RECV_LP
  ldmfd r13!, {r0-r3, pc}


INIT_IO:
	stmfd r13!, {r5-r6, lr}
	// Enable GPIOD Peripheral Clock (bit 3 in AHB1ENR register)
	ldr r6, =RCC_BASE       // Load peripheral clock reg address to r6
	ldr r5, [r6, #RCC_AHB1ENR]                // Read its content to r5
	orr r5, #0x00000008          // Set bit 3 to enable GPIOD clock
	str r5, [r6, #RCC_AHB1ENR]                // Store result in peripheral clock register

	// Make GPIOD Pin12 as output pin (bits 26:27 in MODER register)
	ldr r6, =GPIOD_BASE       // Load GPIOD MODER register address to r6
	ldr r5, [r6, #GPIOD_MODER]                // Read its content to r5
	bic r5, #0x0C000000          // Clear bits 26, 27 for P12
	orr r5, #0x04000000         // Write 01 to bits 26, 27 for P12
	str r5, [r6, #GPIOD_MODER]                // Store result in GPIOD MODER register

	ldmfd r13!, {r5-r6, pc}

INIT_TIM6:
	stmfd r13!, {r0-r2, lr}

	ldr r0, =RCC_BASE
	ldr r1, [r0, #RCC_APB1ENR]
	ldr r2, =0x10
	orr r1, r1, r2
	str r1, [r0, #RCC_APB1ENR]

	ldr r0, =TIM6_BASE

	ldr r1, =16800
	str r1, [r0, #TIM6_PSC]

	ldmfd r13!, {r0-r2, pc}

INIT_USART2:
	stmfd r13!, {r0-r2, lr}
	ldr r0, =RCC_BASE

	// Enable USART2 clock
	ldr r1, [r0, #RCC_APB1ENR]
	orr r1, r1, #0x20000
	str r1, [r0, #RCC_APB1ENR]

	ldr r0, =GPIOD_BASE

	// Set GPIO to AF (alternative function)
	ldr r1, [r0, #GPIOD_AFRL]
	orr r1, GPIOD_AFRL_VALUE
	str r1, [r0, #GPIOD_AFRL]

	ldr r1, [r0, #GPIOD_MODER]
	orr r1, #0x2800
	str r1, [r0, #GPIOD_MODER]

	ldr r0, =USART2_BASE

	// Baud rate
	ldr r1, =USART2_BRR_VAL
	str r1, [r0, #USART2_BRR]

	// Enable
	ldr r1, =USART2_CR1_VAL
	orr r1, r1, #(1 << 5) // Enable receive interrupt
	str r1, [r0, #USART2_CR1]

	// Enable NVIC interrpt
	ldr r0, =NVIC_BASE
	ldr r1, [r0, #NVIC_ISER1]
	orr r1, #(1 << 6) // USART2 interrupt is #38
	str r1, [r0, #NVIC_ISER1]

	ldmfd r13!, {r0-r2, pc}


LED_ON:
	stmfd r13!, {r5-r6, lr}
	// Set GPIOD Pin13 to 1 (bit 13 in ODR register)
	ldr r6, =GPIOD_BASE         // Load GPIOD output data register
	ldr r5, [r6, #GPIOD_ODR]                // Read its content to r5
	orr r5, #0x2000              // write 1 to pin 13
	str r5, [r6, #GPIOD_ODR]                // Store result in GPIOD output data register
	ldmfd r13!, {r5-r6, pc}

LED_OFF:
	stmfd r13!, {r5-r6, lr}
	// Set GPIOD Pin13 to 1 (bit 13 in ODR register)
	ldr r6, =GPIOD_BASE         // Load GPIOD output data register
	ldr r5, [r6, #GPIOD_ODR]                // Read its content to r5
	bic r5, #0x2000              // write 1 to pin 13
	str r5, [r6, #GPIOD_ODR]                // Store result in GPIOD output data register
	ldmfd r13!, {r5-r6, pc}

DELAY_TIM6:
	stmfd r13!, {r1-r3, lr}
	ldr r1, =TIM6_BASE
	ldr r2, =0
	str r2, [r1, #TIM6_SR]

	str r0, [r1, #TIM6_ARR]

	ldr r2, =0x09
	str r2, [r1, #TIM6_CR1]

	loop1:
		ldr r2, [r1, #TIM6_SR]
		cmp r2, #1
		bne loop1

	ldmfd r13!, {r1-r3, pc}

XMCHAR:
	stmfd r13!, {r0-r1, lr}
	mov r1, r0
	cmp r1, '.'
	ITE EQ
	ldreq r0, =150
	ldrne r0, =300
	bl LED_ON
	bl DELAY_TIM6

	// ugasni LED in pocakaj 150ms
	ldr r0, =150
	bl LED_OFF
	bl DELAY_TIM6
	ldmfd r13!, {r0-r1, pc}

XMCODE:
	stmfd r13!, {r0-r1, lr}
	mov r1, r0
crka_loop:
	ldrb r0, [r1], #1
	cmp r0, 0
	beq crka_konec
	bl XMCHAR
	b crka_loop
crka_konec:
	ldr r0, =300
	bl LED_OFF
	bl DELAY_TIM6
	ldmfd r13!, {r0-r1, pc}

GETMCODE:
	stmfd r13!, {r1-r3, lr}
	mov r1, r0
	adr r0, ABECEDA
	sub r1, r1, 'A'
	ldr r3, =6
	mul r2, r1, r3 // odmik
	add r0, r0, r2
	ldmfd r13!, {r1-r3, pc}

XWORD:
	stmfd r13!, {r1-r3, lr}
	mov r1, r0
niz_loop:
	ldrb r0, [r1], #1
	cmp r0, ENTER_ASCII
	beq niz_konec
	bl GETMCODE
	bl XMCODE
	b niz_loop
niz_konec:
	ldr r0, =1000
	bl LED_OFF
	bl DELAY_TIM6
	ldmfd r13!, {r1-r3, pc}

.global USART2_IRQHandler
.type USART2_IRQHandler, %function
USART2_IRQHandler:
	stmfd r13!, {r0-r2, lr}
	ldr r1, =USART2_BASE
	ldr r2, [r1, #USART2_DR]
	strb r2, [r7], #1
	cmp r2, ENTER_ASCII
	bne USART2_IRQHandler_Return
	ldr r0, =RX_NIZ
	bl XWORD
	ldr r7, =RX_NIZ // reset r7 to the start of RX_NIZ
  USART2_IRQHandler_Return:
	ldmfd r13!, {r0-r2, pc}

Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
