# baremetal

The goal of this project is to learn more about the peripherals on the STM32F4 microcontroller by
writing the blinking LED firmware using raw register manipulation. The board I'm using for this
project is the STM32 Black Pill v3.1 that I've got off of AliExpress. It features an STM32F411CE
microcontroller, a 25Mhz external crystall oscillator and an on-board LED.

## Overview

### Compiling and Linking

While the compilation part is pretty straightforward, simply requiring a few flags to be passed
to GCC, the linking part is a bit tricky. STM32F4 MCU has a built-in bootloader that starts
execution at the second address (so at 0x04 byte offset) of its flash memory (which starts at 0x0800 0000).
The beginning of the flash memory must contain what's called a "vector table", which is basically a
memory map of interrupt handlers. That second address in flash memory is a pointer to a reset handler,
a function that is executed first. Note that reset handler is **NOT** your `main()` function.

In order for the linker to generate the desired memory layout for our firmware, we must provide
it a linker script. A linker script is a special file that contains a set of instructions to lay
out the sections, as well as specifies the program entry point and a few other things.

The actual vector table is defined in `vector.c` file. Vector table is represented as a struct,
fields of which are function pointers in the same order as specified in the reference manual.
Every interrupt handler (and the reset handler) is a void-returning function that accepts 0 arguments.
Since it's very unlikely that our firmware will use all 86 interrupt handlers available, we still
need to fill the pointers for those handlers they can't be zero. To do that, we'll use a GCC attribute
`__attribute__((weak, alias("null_handler")))` on a function declaration which basically means that
unless the symbol is explicitly defined, it will be a reference to `null_handler()` that we defined
in `vector.c`. There we also define the reset handler, which enables the access to the floating-point
coprocessor and calls our `main()` function. This was taken from `libopencm3`'s reset handler.

Back in the linker script, we emit a `EXTERN(vector_table)` command, which means that there's a symbol
called `vector_table` and it should not be discarded when linking. We also set the entrypoint using
`ENTRY(reset_handler)`. Next, we specify our memory regions (flash and RAM) and then we define the
sections. The `.text` section is our code section and it's composed of a few input sections:
`.vectors`, which is where our `vector_table` struct goes, `.text*` - code sections produced by
the compiler, and `.rodata` which contains read-only data. The rest of the sections were
copied almost verbatim from `libopencm3`, omitting ones that are required for C++.

To recap: STM32F4 requires a specific memory layout that starts with a map of interrupt handlers,
which we define by using external function declarations with placeholder/fallback values. Setting
this up requires quite a bit of boilerplate so you probably don't want to do it by hand, unless
you know what you're doing and/or have a pretty specific use case.

### Reset and Clock Control (RCC)

#### Clock source

RCC is responsible for resetting the peripherals and controlling clock signals. STM32F4 MCU has
several clock sources but we only really care about two of them:

- High-speed internal clock (HSI): built-in clock that runs at 16Mhz that is used when clock accuracy
is not so important.
- High-speed external clock (HSE): a precise external crystal oscillator or clock source.

Since the board I'm using has a 25Mhz external crystal oscillator, I'll use HSE as the clock source
for the MCU.

#### Phase-Locked Loop (PLL)

PLL is a cicruit that is used to multiply the clock frequency. STM32F4 has two PLLs: the main one
and the I2S one. The latter is used to configure the clock for [I2S](https://en.wikipedia.org/wiki/I%C2%B2S)
peripheral separately. We are not going to use it in this project.

Main PLL configuration consists of the following steps:

- Selecting the PLL clock source, which can either be the HSI or the HSE.
- Setting the division factor (M) for the clock input. Input frequency must range from 1Mhz to 2Mhz.
- Setting the multiplication factor (N) for the VCO. Output frequency must range from 100Mhz to 433Mhz.
- Setting the division factor (P) for the system clock. This frequency should not exceed 100Mhz.
- Setting the division factor (Q) for USB OTF FS and SDIO clocks. This frequency must be 48Mhz.

Here's the configuration I'm using:

- HSE as the PLL clock source.
- M is set to 25. Since the frequency of HSE is 25Mhz, this will give us a 1Mhz input frequency.
- N is set to 192 to set the VCO frequency to 192Mhz.
- P is set to 2 to set the system clock frequency to 96Mhz.
- Q is set to 4 to set the USB OTF FS and SDIO clocks frequency to 48Mhz.

#### Prescalers

The three peripheral buses (AHB, APB1 and APB2) require different clock speeds.

I'll configure the AHB to run at the same frequency as the system clock (96Mhz). According to the
reference manual, the APB1 clock frequency should not exceed 50Mhz, and the APB2 frequency should
not exceed 100Mhz. Given that, I'll set a division factor of 2 for the former (which divides the AHB frequency),
and a division factor of 1 for the latter.

#### Code

Complete clock configuration can be found in `clock_setup()` function. It is basically a re-implementation
of [libopencm3](https://github.com/libopencm3/libopencm3)'s `rcc_clock_setup_pll()` function.
I've copied power and flash configuration from there but I haven't yet figured out
what that's for.

## References

List of resources I've used for this project:

1. [STM32F411CE datasheet](https://www.st.com/resource/en/datasheet/stm32f411ce.pdf)
2. [STM32F411xC/E reference manual](https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
3. [libopencm3](http://libopencm3.org/docs/latest/html/index.html)
4. [Bare Metal Programming Series](https://www.youtube.com/playlist?list=PLP29wDx6QmW7HaCrRydOnxcy8QmW0SNdQ)
