/* ---------------------------------------------------------------------------
 * stm32f411ce.ld -- Linker script for the Mini-LiDAR-Board (STM32F411CEU6).
 *
 * Device: STM32F411CEU6, UFQFPN-48
 *   FLASH : 512 KiB @ 0x08000000
 *   SRAM  : 128 KiB @ 0x20000000
 *
 * Memory layout produced:
 *
 *   0x08000000  .isr_vector   vector table (must be first, VTOR defaults here)
 *               .text         code
 *               .rodata       constants, USB descriptors
 *               _sidata ->    initialised-data image
 *   0x20000000  .data         copied from _sidata by Reset_Handler
 *               .bss          zeroed by Reset_Handler
 *               ._user_heap_stack   guard region, link fails if it doesn't fit
 *   0x20020000  <- _estack (full-descending MSP starts here)
 * ------------------------------------------------------------------------- */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Top of RAM: the Cortex-M stack is full-descending, so MSP is initialised
 * to the address one past the end of SRAM. */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Minimum room that must remain between the end of .bss and the stack top.
 * The link fails loudly if we ever overflow RAM, instead of silently
 * producing an image whose stack collides with .bss at run time. */
_Min_Heap_Size  = 0x000;   /* no malloc() anywhere in this firmware */
_Min_Stack_Size = 0x800;   /* 2 KiB: main + worst-case nested ISR frames    */

SECTIONS
{
  /* ---- Vector table --------------------------------------------------- */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* ---- Code ------------------------------------------------------------ */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)         /* ARM/Thumb interworking veneers (unused, harmless) */
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* ---- Read-only data --------------------------------------------------- */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM       : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  /* C++ / newlib init arrays. Kept so that libc's startup contract holds
   * even though this project has no static constructors. */
  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  /* Load address of the .data initialiser image, in FLASH. */
  _sidata = LOADADDR(.data);

  /* ---- Initialised data (FLASH image -> RAM) ---------------------------- */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* ---- Zero-initialised data -------------------------------------------- */
  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* ---- Heap/stack guard -------------------------------------------------- */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM


  .ARM.attributes 0 : { *(.ARM.attributes) }
}
