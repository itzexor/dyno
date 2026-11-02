.syntax unified
.cpu cortex-m0
.thumb
.section .isr_vector,"a",%progbits
.global vectors
vectors:
.word _estack, Reset_Handler, Default_Handler, Default_Handler
.word 0, 0, 0, 0, 0, 0, 0, Default_Handler, 0, 0, Default_Handler, Default_Handler
.word Default_Handler, Default_Handler, Default_Handler, Default_Handler
.word Default_Handler, Default_Handler, Default_Handler, EXTI4_15_IRQHandler
.word Default_Handler, Default_Handler, Default_Handler, Default_Handler
.word Default_Handler, Default_Handler, Default_Handler, Default_Handler
.word Default_Handler, Default_Handler, Default_Handler, Default_Handler
.word Default_Handler, Default_Handler, Default_Handler, Default_Handler
.word I2C2_IRQHandler, Default_Handler, Default_Handler, Default_Handler
.word Default_Handler, Default_Handler, Default_Handler, USB_IRQHandler

.section .text.Reset_Handler,"ax",%progbits
.global Reset_Handler
.thumb_func
Reset_Handler:
  cpsid i
  ldr r0, =_sidata
  ldr r1, =_sdata
  ldr r2, =_edata
1:
  cmp r1, r2
  bcs 2f
  ldr r3, [r0]
  str r3, [r1]
  adds r0, #4
  adds r1, #4
  b 1b
2:
  ldr r1, =_sbss
  ldr r2, =_ebss
  movs r3, #0
3:
  cmp r1, r2
  bcs 4f
  str r3, [r1]
  adds r1, #4
  b 3b
4:
  bl board_main
5: b 5b

.section .text.Default_Handler,"ax",%progbits
.global Default_Handler
.thumb_func
Default_Handler:
  cpsid i
1: b 1b
