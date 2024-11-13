#include <stdint.h>

typedef uint32_t u32;

#define MMIO32(addr) (*(volatile u32 *)(addr))

// Base address for flash memory registers.
#define FLASH_BASE           (0x40023C00)

#define FLASH_ACR            MMIO32(FLASH_BASE + 0x00)         // Flash access control register.
#define FLASH_ACR_DCEN       (1 << 10)                         // Data cache enable bit.
#define FLASH_ACR_ICEN       (1 << 9)                          // Instruction cache enable bit.

// Base address for power registers.
#define PWR_BASE             (0x40007000)

#define PWR_CR               MMIO32(PWR_BASE + 0x00)           // Power control register.
#define PWR_CR_VOS_MASK      (0b11)
#define PWR_CR_VOS_SHIFT     (14)
#define PWR_CR_VOS_SCALE3    (0b01)
#define PWR_CR_VOS_SCALE2    (0b10)
#define PWR_CR_VOS_SCALE1    (0b11)

// Base address for RCC registers.
#define RCC_BASE             (0x40023800)

#define RCC_CR               MMIO32(RCC_BASE + 0x00)           // RCC clock control register.
#define RCC_CR_PLLRDY        (1 << 25)                         // Main PLL ready bit.
#define RCC_CR_PLLON         (1 << 24)                         // Main PLL enable bit.
#define RCC_CR_HSERDY        (1 << 17)                         // HSE ready bit.
#define RCC_CR_HSEON         (1 << 16)                         // HSE enable bit.
#define RCC_CR_HSIRDY        (1 << 1)                          // HSI ready bit.
#define RCC_CR_HSION         (1 << 0)                          // HSI enable bit.

#define RCC_PLLCFGR          MMIO32(RCC_BASE + 0x04)           // RCC PLL configuration register.
#define RCC_PLLCFGR_PLLSRC   (1 << 22)                         // PLL source bit.
#define RCC_PLLCFGR_PLLQ(q)  (((q) & 0xF) << 24)               // PLLQ bits.
#define RCC_PLLCFGR_PLLP(p)  (((((p) >> 1) - 1) & 0b11) << 16) // PLLP bits.
#define RCC_PLLCFGR_PLLN(n)  (((n) & 0x1FF) << 6)              // PLLN bits.
#define RCC_PLLCFGR_PLLM(m)  (((m) & 0x63) << 0)               // PLLM bits.

#define RCC_CFGR             MMIO32(RCC_BASE + 0x08)           // RCC clock configuration register.
#define RCC_CFGR_PPRE2_SHIFT (13)                              // PPRE2 bit shift.
#define RCC_CFGR_PPRE2_MASK  (0b111)                           // PPRE2 bits mask.
#define RCC_CFGR_PPRE1_SHIFT (10)                              // PPRE1 bit shift.
#define RCC_CFGR_PPRE1_MASK  (0b111)                           // PPRE1 bits mask.
#define RCC_CFGR_HPRE_SHIFT  (4)                               // HPRE bit shift.
#define RCC_CFGR_HPRE_MASK   (0b1111)                          // HPRE bits mask.
#define RCC_CFGR_SWS_SHIFT   (2)                               // SWS bit shift.
#define RCC_CFGR_SWS_MASK    (0b11)                            // SWS bits mask.
#define RCC_CFGR_SW_HSI      (0b00)                            // HSI oscillator used as SYSCLK.
#define RCC_CFGR_SW_HSE      (0b01)                            // HSE oscillator used as SYSCLK.
#define RCC_CFGR_SW_PLL      (0b10)                            // PLL used as system clock.

#define RCC_APB1ENR          MMIO32(RCC_BASE + 0x40)           // RCC APB1 peripheral clock enable register.
#define RCC_APB1ENR_PWREN    (1 << 28)                         // Power interface clock enable bit.

/*
 * Set up clocks to run from PLL.
 */
static void
clock_setup(void)
{
	u32 reg;

	// Enable the HSI oscillator and wait until it's ready.
	// We need it to temporarily use it as a system clock so that we can change the system
	// clock source to PLL.
	RCC_CR |= RCC_CR_HSION;
	while (!(RCC_CR & RCC_CR_HSIRDY));

	// Use HSI as system clock.
	reg = RCC_CFGR;                     // Copy current register value.
	reg &= ~((1 << 1) | (1 << 0));      // Clear bits 0 and 1.
	RCC_CFGR = (reg | RCC_CFGR_SW_HSI); // Set the system clock switch bits.

	// Enable the HSE oscillator and wait until it's ready.
	RCC_CR |= RCC_CR_HSEON;
	while (!(RCC_CR & RCC_CR_HSERDY));

	// Enable the power interface clock.
	RCC_APB1ENR |= RCC_APB1ENR_PWREN;

	// Set the voltage scale. (TODO: why?)
	reg = PWR_CR;
	reg &= ~(PWR_CR_VOS_MASK << PWR_CR_VOS_SHIFT);            // Clear VOS bits.
	PWR_CR = (reg | (PWR_CR_VOS_SCALE1 << PWR_CR_VOS_SHIFT)); // Set VOS bits.

	/*
	 * Set peripheral clocks prescalers:
	 * - run AHB at the same frequency as the main PLL (96Mhz);
	 * - run APB1 at half frequency of AHB so that it does not exceed 50Mhz;
	 * - run APB2 at the same frequency as AHB;
	 */

	// Copy the configuration register.
	reg = RCC_CFGR;

	// Clear HPRE bits here to set the AHB prescaler to 1.
	reg &= ~(RCC_CFGR_HPRE_MASK << RCC_CFGR_HPRE_SHIFT);

	// Set PPRE1 bits to 0b100 to divide the AHB clock by 2.
	reg &= ~(RCC_CFGR_PPRE1_MASK << RCC_CFGR_PPRE1_SHIFT);
	reg |= ((0b100 & RCC_CFGR_PPRE1_MASK) << RCC_CFGR_PPRE1_SHIFT);

	// Clear PPRE2 bits to divide the AHB clock by 1.
	reg &= ~(RCC_CFGR_PPRE2_MASK << RCC_CFGR_PPRE2_SHIFT);

	// Set the modified register value.
	RCC_CFGR = reg;

	// Disable the main PLL before configuring it.
	RCC_CR &= ~RCC_CR_PLLON;

	/* Configure PLL:
	 * - Set the source to HSE by setting the PLLSRC bit. To instead set the source to HSI
	 *   this bit should be cleared.
	 * - Set the input clock division factor to 25. Since we have a 25Mhz external oscillator this
	 *   should give us a 1Mhz frequency.
	 * - Set the multiplication factor to 192. This sets the VCO frequency to 192Mhz.
	 * - Set the division factor to 4 for USB OTF FS and SDIO clocks to give them exactly 48Mhz.
	 * - Set the division factor to 2 for main clock to set its frequency to 96Mhz.
	 */
	RCC_PLLCFGR = RCC_PLLCFGR_PLLSRC
		| RCC_PLLCFGR_PLLM(25)
		| RCC_PLLCFGR_PLLN(192)
		| RCC_PLLCFGR_PLLQ(4)
		| RCC_PLLCFGR_PLLP(2);

	// Enable the main PLL and wait until it's ready.
	RCC_CR |= RCC_CR_PLLON;
	while (!(RCC_CR & RCC_CR_PLLRDY));

	// Configure flash settings (TODO: why?)
	reg = FLASH_ACR;
	reg |= (FLASH_ACR_DCEN | FLASH_ACR_ICEN); // Enable data and instruction caches.
	reg |= 0b11;                              // Set latency bits to 3 wait states.
	FLASH_ACR = reg;

	// Select PLL as the system clock source and wait until it's ready.
	reg = RCC_CFGR;                     // Copy current register value.
	reg &= ~((1 << 1) | (1 << 0));      // Clear bits 0 and 1.
	RCC_CFGR = (reg | RCC_CFGR_SW_PLL); // Set the system clock switch bits.
	while (((RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SW_PLL);

	// Disable the HSI oscillator since we don't need it anymore.
	RCC_CR &= ~RCC_CR_HSION;
}

int
main(void)
{
	clock_setup();

	while (1);

	return 0;
}
