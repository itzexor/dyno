ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08002000, LENGTH = 0xD800
  RAM (rwx)  : ORIGIN = 0x200000C0, LENGTH = 0x3F38
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
  .ARM.exidx : { *(.ARM.exidx*) } > FLASH
  .data : {
    . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .;
  } > RAM AT > FLASH
  _sidata = LOADADDR(.data);
  __flash_end__ = LOADADDR(.data) + SIZEOF(.data);
  .bss (NOLOAD) : {
    . = ALIGN(8); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(8); _ebss = .;
  } > RAM
  /DISCARD/ : { *(.comment*) *(.note*) }
}
ASSERT(SIZEOF(.isr_vector) == 192, "Cortex-M0 vector table must contain 48 entries")
ASSERT(__flash_end__ <= 0x0800F800, "image overlaps the settings page")
ASSERT(_ebss <= _estack - 2048, "less than 2 KiB stack headroom")
