/* bootloader/app_ota.ld
 *
 * Linker script for the OPT-IN 'light_switch_ota' CMake target (../CMakeLists.txt,
 * gated behind -DOTA_APP_LAYOUT=ON) — the SAME light_switch app sources, linked
 * to run relocated inside the OTA active-app flash slot instead of at the
 * standard Pico flash origin. See ../src/partition_map.h:
 *
 *   APP_OFF  = 0x00010000   (XIP 0x10010000, PART_XIP(APP_OFF))
 *   APP_SIZE = 0x00090000   (576 KB — true-rollback phase re-carve; was
 *                            0x000E0000/896 KB before the BACKUP region
 *                            was added)
 *
 * This is a byte-for-byte copy of pico-sdk's own
 * lib/pico-sdk/src/rp2_common/pico_crt0/rp2040/memmap_default.ld (the SAME
 * base this repo already copied for bootloader/memmap_bootloader.ld — diff
 * against that file's own header comment for the general approach), with
 * exactly two differences:
 *
 *   1. The FLASH MEMORY region below is hardcoded to
 *      ORIGIN = 0x10010000, LENGTH = 0x90000 (APP_OFF/APP_SIZE) instead of
 *      `INCLUDE "pico_flash_region.ld"` / the default 0x10000000 origin.
 *      Exactly the same "why a dedicated linker script, not a
 *      PICO_FLASH_SIZE_BYTES CMake variable" reasoning as
 *      bootloader/memmap_bootloader.ld applies here — see that file's own
 *      header comment for the full story (short version: PICO_FLASH_SIZE_
 *      BYTES is a *global* compile define via pico_base_headers, so setting
 *      it to shrink one target's linker region would corrupt
 *      hardware_flash.c's bounds check for the WHOLE 2 MB chip). This app
 *      still needs hardware_flash.c to see the chip's true 2 MB (it does not
 *      call flash_range_erase/program itself, but pico-vfs's littlefs driver
 *      does, out at FS_OFF = 0x1C0000 — nowhere near 0x090000, so it would be
 *      the very first thing to break if PICO_FLASH_SIZE_BYTES were ever
 *      global-shrunk to this target's own APP_SIZE).
 *
 *   2. The `.boot2` output section (and its 256-byte size ASSERT) is REMOVED
 *      entirely — see the "no boot2" note below.
 *
 * KEEP THIS FLASH LENGTH IN SYNC WITH APP_SIZE in ../src/partition_map.h. A
 * build that grows past 576 KB fails the link with a "region `FLASH'
 * overflowed" error — a deliberate, automatic guard against silently
 * growing into the staging region that immediately follows (STAGING_OFF =
 * 0xA0000).
 *
 * ── Why no `.boot2` (picowota / pico-flashloader relocation approach) ─────
 * A normal Pico app is the FIRST thing the RP2040's boot ROM runs after a
 * cold reset: the boot ROM loads exactly 256 bytes from flash offset 0 (the
 * `.boot2` stage), which configures the QSPI/SSI controller for fast XIP
 * reads and then jumps to the app's own vector table right after it. This
 * app is never booted that way — `bootloader/main.c` (the OTA bootloader,
 * itself a normal standalone Pico app WITH its own boot2 at real flash
 * offset 0) is the thing the boot ROM runs at cold boot. Its `jump_to_app()`
 * runs strictly AFTER the boot ROM + the bootloader's own boot2 have already
 * put the QSPI controller into XIP mode — that hardware configuration is a
 * persistent SSI peripheral register state that survives an ordinary branch
 * (there is no chip reset between "bootloader running" and "app running",
 * just `SCB->VTOR = ...; msr msp, ...; bx <app reset handler>`), so the app
 * being jumped to does not need to redo it, and — critically — MUST NOT have
 * its own `.boot2` sitting at the front of its flash region, because
 * `jump_to_app()` reads the app's initial MSP/reset-handler straight out of
 * `PART_XIP(APP_OFF)`'s first 8 bytes (standard ARMv6-M vector table, word 0
 * = MSP, word 1 = reset handler — see bootloader/main.c's jump_to_app()) with
 * NO 256-byte boot2 skip. So the vector table (`KEEP (*(.vectors))`, first
 * thing pulled into `.text` below) must be the very first bytes of this
 * linked image, at flash offset APP_OFF exactly — hence dropping `.boot2`
 * rather than keeping a dummy/no-op one. The CMake side backs this up by
 * pointing `PICO_TARGET_BOOT_STAGE2` at an empty `bs2_none_library` INTERFACE
 * target for this executable (../CMakeLists.txt), so no object file
 * contributes a `.boot2` input section to this link in the first place —
 * this script's dropping the output section is belt-and-braces, not the only
 * thing preventing a stray 256-byte gap.
 *
 * RAM/SCRATCH_X/SCRATCH_Y regions are unchanged from memmap_default.ld: they
 * describe physical SRAM, not flash, and the OTA bootloader has already
 * returned all of it (single-core, nothing left running) by the time this
 * app's crt0 takes over, so there is nothing to coordinate there.
 *
 * Everything below the MEMORY block (other than the `.boot2` removal noted
 * above) is unmodified from pico-sdk's memmap_default.ld.
 */

MEMORY
{
    /* ORIGIN: RP2040 XIP flash base (0x10000000) + APP_OFF (0x00010000).
       LENGTH: APP_SIZE from ../src/partition_map.h (0x00090000 = 576 KB). */
    FLASH(rx) : ORIGIN = 0x10010000, LENGTH = 0x90000
    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 256k
    SCRATCH_X(rwx) : ORIGIN = 0x20040000, LENGTH = 4k
    SCRATCH_Y(rwx) : ORIGIN = 0x20041000, LENGTH = 4k
}

ENTRY(_entry_point)

SECTIONS
{
    /* No second-stage bootloader in this image — see the big "no boot2"
       comment above. The vector table below is the very first thing in the
       image, at FLASH's ORIGIN (APP_OFF's XIP address). */

    .flash_begin : {
        __flash_binary_start = .;
    } > FLASH

    /* The second stage will always enter the image at the start of .text.
       The debugger will use the ELF entry point, which is the _entry_point
       symbol if present, otherwise defaults to start of .text.
       This can be used to transfer control back to the bootrom on debugger
       launches only, to perform proper flash setup.
    */

    .text : {
        __logical_binary_start = .;
        KEEP (*(.vectors))
        KEEP (*(.binary_info_header))
        __binary_info_header_end = .;
        KEEP (*(.embedded_block))
        __embedded_block_end = .;
        KEEP (*(.reset))
        /* TODO revisit this now memset/memcpy/float in ROM */
        /* bit of a hack right now to exclude all floating point and time critical (e.g. memset, memcpy) code from
         * FLASH ... we will include any thing excluded here in .data below by default */
        *(.init)
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .text*)
        *(.fini)
        /* Pull all c'tors into .text */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        /* Followed by destructors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(SORT(.preinit_array.*)))
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.eh_frame*)
        . = ALIGN(4);
    } > FLASH

    .rodata : {
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .rodata*)
        . = ALIGN(4);
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.flashdata*)))
        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    /* Machine inspectable binary information */
    . = ALIGN(4);
    __binary_info_start = .;
    .binary_info :
    {
        KEEP(*(.binary_info.keep.*))
        *(.binary_info.*)
    } > FLASH
    __binary_info_end = .;
    . = ALIGN(4);

    .ram_vector_table (NOLOAD): {
        *(.ram_vector_table)
    } > RAM

    .uninitialized_data (NOLOAD): {
        . = ALIGN(4);
        *(.uninitialized_data*)
    } > RAM

    .data : {
        __data_start__ = .;
        *(vtable)

        *(.time_critical*)

        /* remaining .text and .rodata; i.e. stuff we exclude above because we want it in RAM */
        *(.text*)
        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);

        *(.data*)

        . = ALIGN(4);
        *(.after_data.*)
        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__mutex_array_start = .);
        KEEP(*(SORT(.mutex_array.*)))
        KEEP(*(.mutex_array))
        PROVIDE_HIDDEN (__mutex_array_end = .);

        . = ALIGN(4);
        *(.jcr)
        . = ALIGN(4);
    } > RAM AT> FLASH

    .tdata : {
        . = ALIGN(4);
		*(.tdata .tdata.* .gnu.linkonce.td.*)
        /* All data end */
        __tdata_end = .;
    } > RAM AT> FLASH
    PROVIDE(__data_end__ = .);

    /* __etext is (for backwards compatibility) the name of the .data init source pointer (...) */
    __etext = LOADADDR(.data);

    .tbss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start__ = .;
        __tls_base = .;
        *(.tbss .tbss.* .gnu.linkonce.tb.*)
        *(.tcommon)

        __tls_end = .;
    } > RAM

    .bss (NOLOAD) : {
        . = ALIGN(4);
        __tbss_end = .;

        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .heap (NOLOAD):
    {
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
    } > RAM
    /* historically on GCC sbrk was growing past __HeapLimit to __StackLimit, however
       to be more compatible, we now set __HeapLimit explicitly to where the end of the heap is */
    __HeapLimit = ORIGIN(RAM) + LENGTH(RAM);

    /* Start and end symbols must be word-aligned */
    .scratch_x : {
        __scratch_x_start__ = .;
        *(.scratch_x.*)
        . = ALIGN(4);
        __scratch_x_end__ = .;
    } > SCRATCH_X AT > FLASH
    __scratch_x_source__ = LOADADDR(.scratch_x);

    .scratch_y : {
        __scratch_y_start__ = .;
        *(.scratch_y.*)
        . = ALIGN(4);
        __scratch_y_end__ = .;
    } > SCRATCH_Y AT > FLASH
    __scratch_y_source__ = LOADADDR(.scratch_y);

    /* .stack*_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later
     *
     * stack1 section may be empty/missing if platform_launch_core1 is not used */

    /* by default we put core 0 stack at the end of scratch Y, so that if core 1
     * stack is not used then all of SCRATCH_X is free.
     */
    .stack1_dummy (NOLOAD):
    {
        *(.stack1*)
    } > SCRATCH_X
    .stack_dummy (NOLOAD):
    {
        KEEP(*(.stack*))
    } > SCRATCH_Y

    .flash_end : {
        KEEP(*(.embedded_end_block*))
        PROVIDE(__flash_binary_end = .);
    } > FLASH

    /* stack limit is poorly named, but historically is maximum heap ptr */
    __StackLimit = ORIGIN(RAM) + LENGTH(RAM);
    __StackOneTop = ORIGIN(SCRATCH_X) + LENGTH(SCRATCH_X);
    __StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);
    __StackOneBottom = __StackOneTop - SIZEOF(.stack1_dummy);
    __StackBottom = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* picolibc and LLVM */
    PROVIDE (__heap_start = __end__);
    PROVIDE (__heap_end = __HeapLimit);
    PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
    PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1));
    PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );

    /* llvm-libc */
    PROVIDE (_end = __end__);
    PROVIDE (__llvm_libc_heap_limit = __HeapLimit);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed")

    ASSERT( __binary_info_header_end - __logical_binary_start <= 256, "Binary info must be in first 256 bytes of the binary")
    /* todo assert on extra code */
}
