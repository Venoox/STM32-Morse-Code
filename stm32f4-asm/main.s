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
.equ     LEDDELAY,      4000
// Register Addresses
// You can find the base addresses for all peripherals from Memory Map section
// RM0090 on page 64. Then the offsets can be found on their relevant sections.

// RCC   base address is 0x40023800
//   AHB1ENR register offset is 0x30
.equ     RCC_AHB1ENR,   0x40023830 // RCC AHB1 peripheral clock reg (page 180)

// GPIOD base address is 0x40020C00
//   MODER register offset is 0x00
//   ODR   register offset is 0x14
.equ     GPIOD_MODER,   0x40020C00 // GPIOD port mode register (page 281)
.equ     GPIOD_ODR,     0x40020C14 // GPIOD output data register (page 283)

.equ     RCC_BASE, 0x40023800
.equ	 RCC_APB1ENR, 0x40

.equ	 TIM6_BASE,    0x40001000
.equ     TIM6_CR1,     0x00 // control register
.equ     TIM6_CNT,     0x24 // timer value
.equ     TIM6_PSC,     0x28 // prescaler
.equ     TIM6_ARR,     0x2c // autoreload
.equ     TIM6_EGR,     0x14
.equ     TIM6_SR,      0x10

.equ	 STK_BASE, 0xE000E010
.equ	 STK_CTRL, 0x00
.equ	 STK_LOAD, 0x04
// Start of text section
.section .text
///////////////////////////////////////////////////////////////////////////////
// Vectors
///////////////////////////////////////////////////////////////////////////////
// Vector table start
// Add all other processor specific exceptions/interrupts in order here
	.long    __StackTop                 // Top of the stack. from linker script
	.long    _start +1                  // reset location, +1 for thumb mode

///////////////////////////////////////////////////////////////////////////////
// Main code starts from here
///////////////////////////////////////////////////////////////////////////////

_start:
	bl INIT_IO
	bl INIT_TIM6
	bl INIT_SYSTICK
	ldr r8, =TIM6_BASE
loop_led:
	mov r0, #500
	bl DELAY_TIM6
	bl LED_ON
	mov r0, #500
	bl DELAY_TIM6
	bl LED_OFF
	b loop_led


INIT_IO:
	stmfd r13!, {r5-r6, lr}
	// Enable GPIOD Peripheral Clock (bit 3 in AHB1ENR register)
	ldr r6, = RCC_AHB1ENR       // Load peripheral clock reg address to r6
	ldr r5, [r6]                // Read its content to r5
	orr r5, #0x00000008          // Set bit 3 to enable GPIOD clock
	str r5, [r6]                // Store result in peripheral clock register

	// Make GPIOD Pin12 as output pin (bits 26:27 in MODER register)
	ldr r6, = GPIOD_MODER       // Load GPIOD MODER register address to r6
	ldr r5, [r6]                // Read its content to r5
	bic r5, #0x0C000000          // Clear bits 26, 27 for P12
	orr r5, #0x04000000         // Write 01 to bits 26, 27 for P12
	str r5, [r6]                // Store result in GPIOD MODER register
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

INIT_SYSTICK:
	stmfd r13!, {r0-r2, lr}

	ldr r0, =STK_BASE

	ldr r1, =15999
	str r1, [r0, #STK_LOAD]

	ldr r1, [r0, #STK_CTRL]
	ldr r2, =0x5
	orr r1, r1, r2
	str r1, [r0, #STK_CTRL]

	ldmfd r13!, {r0-r2, pc}


LED_ON:
	stmfd r13!, {r5-r6, lr}
	// Set GPIOD Pin13 to 1 (bit 13 in ODR register)
	ldr r6, = GPIOD_ODR         // Load GPIOD output data register
	ldr r5, [r6]                // Read its content to r5
	orr r5, #0x2000              // write 1 to pin 13
	str r5, [r6]                // Store result in GPIOD output data register
	ldmfd r13!, {r5-r6, pc}

LED_OFF:
	stmfd r13!, {r5-r6, lr}
	// Set GPIOD Pin13 to 1 (bit 13 in ODR register)
	ldr r6, = GPIOD_ODR         // Load GPIOD output data register
	ldr r5, [r6]                // Read its content to r5
	bic r5, #0x2000              // write 1 to pin 13
	str r5, [r6]                // Store result in GPIOD output data register
	ldmfd r13!, {r5-r6, pc}

DELAY_STK:
	stmfd r13!, {r1-r2, lr}
	ldr r1, =STK_BASE
	loop2:
		ldr r2, [r1, #STK_CTRL]
		and r2, r2, #0x10000
		cmp r2, #0x10000
		bne loop2
		subs r0, r0, #1
		bne loop2

	ldmfd r13!, {r1-r2, pc}

DELAY_TIM6:
	stmfd r13!, {r1-r3, lr}

	ldr r2, =0
	str r2, [r8, #TIM6_SR]

	str r0, [r8, #TIM6_ARR]

	ldr r2, =0x09
	str r2, [r8, #TIM6_CR1]

	loop1:
		ldr r2, [r8, #TIM6_SR]
		cmp r2, #1
		bne loop1

	ldmfd r13!, {r1-r3, pc}

DELAY:
	stmfd r13!, {r1, lr}
	MSEC: ldr r1, =LEDDELAY
	LOOP:    subs r1, r1, 1
	         bne LOOP
	      subs r0, r0, 1
	      bne MSEC

	ldmfd r13!, {r1, pc}
