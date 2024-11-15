#include <stdint.h>

typedef uint32_t u32;

#define MMIO32(addr) (*(volatile u32 *)(addr))

#define SCB_BASE         (0x0U)

/* Configuration and control register. */
#define SCB_CCR          MMIO32(SCB_BASE + 0x14U)
#define SCB_CCR_STKALIGN (1 << 9)

/* Floating-point coprocessor access control register. */
#define SCB_CPACR        MMIO32(SCB_BASE + 0x88U)
#define SCB_CPACR_FULL   (0b11)
#define SCB_CPACR_CP10   (1 << 20)
#define SCB_CPACR_CP11   (1 << 22)

/* Stack pointer provided by the linker script */
extern u32 _stack;

#define NVIC_IRQ_COUNT 91U

// Interrupt handler function type.
typedef void (*vector_table_entry_t)(void);

typedef struct {
	u32 *initial_sp;            /* Initial stack pointer value */
	vector_table_entry_t reset; /* The reset handler */
	vector_table_entry_t nmi;
	vector_table_entry_t hard_fault;
	vector_table_entry_t mem_fault;
	vector_table_entry_t bus_fault;
	vector_table_entry_t usage_fault;
	vector_table_entry_t reserved_x001c[4];
	vector_table_entry_t sv_call;
	vector_table_entry_t debug_monitor;
	vector_table_entry_t reserved_x0034;
	vector_table_entry_t pend_sv;
	vector_table_entry_t systick;
	vector_table_entry_t irq[NVIC_IRQ_COUNT];
} vector_table_t;

// External firmware entry point declaration.
int main(void);

void __attribute__ ((weak))
reset_handler(void)
{
	/* Set 8-byte stack alignment. */
	SCB_CCR |= SCB_CCR_STKALIGN;

	/* Allow full access to the floating-point coprocessor */
	SCB_CPACR |= SCB_CPACR_FULL * (SCB_CPACR_CP10 | SCB_CPACR_CP11);

	(void)main();
}

void
blocking_handler(void)
{
	while (1);
}

void
null_handler(void)
{
	/* Do nothing */
}

void nmi_handler(void)           __attribute__((weak, alias("null_handler")));
void hard_fault_handler(void)    __attribute__((weak, alias("blocking_handler")));
void mem_fault_handler(void)     __attribute__((weak, alias("blocking_handler")));
void bus_fault_handler(void)     __attribute__((weak, alias("blocking_handler")));
void usage_fault_handler(void)   __attribute__((weak, alias("blocking_handler")));
void sv_call_handler(void)       __attribute__((weak, alias("null_handler")));
void debug_monitor_handler(void) __attribute__((weak, alias("blocking_handler")));
void pend_sv_handler(void)       __attribute__((weak, alias("null_handler")));
void systick_handler(void)       __attribute__((weak, alias("null_handler")));

void nvic_wwdg_isr(void)         __attribute__((weak, alias("null_handler")));
void pvd_isr(void)               __attribute__((weak, alias("null_handler")));
void tamp_stamp_isr(void)        __attribute__((weak, alias("null_handler")));
void rtc_wkup_isr(void)          __attribute__((weak, alias("null_handler")));
void flash_isr(void)             __attribute__((weak, alias("null_handler")));
void rcc_isr(void)               __attribute__((weak, alias("null_handler")));
void exti0_isr(void)             __attribute__((weak, alias("null_handler")));
void exti1_isr(void)             __attribute__((weak, alias("null_handler")));
void exti2_isr(void)             __attribute__((weak, alias("null_handler")));
void exti3_isr(void)             __attribute__((weak, alias("null_handler")));
void exti4_isr(void)             __attribute__((weak, alias("null_handler")));
void dma1_stream0_isr(void)      __attribute__((weak, alias("null_handler")));
void dma1_stream1_isr(void)      __attribute__((weak, alias("null_handler")));
void dma1_stream2_isr(void)      __attribute__((weak, alias("null_handler")));
void dma1_stream3_isr(void)      __attribute__((weak, alias("null_handler")));
void dma1_stream4_isr(void)      __attribute__((weak, alias("null_handler")));
void dma1_stream5_isr(void)      __attribute__((weak, alias("null_handler")));
void dma1_stream6_isr(void)      __attribute__((weak, alias("null_handler")));
void adc_isr(void)               __attribute__((weak, alias("null_handler")));
void can1_tx_isr(void)           __attribute__((weak, alias("null_handler")));
void can1_rx0_isr(void)          __attribute__((weak, alias("null_handler")));
void can1_rx1_isr(void)          __attribute__((weak, alias("null_handler")));
void can1_sce_isr(void)          __attribute__((weak, alias("null_handler")));
void exti9_5_isr(void)           __attribute__((weak, alias("null_handler")));
void tim1_brk_tim9_isr(void)     __attribute__((weak, alias("null_handler")));
void tim1_up_tim10_isr(void)     __attribute__((weak, alias("null_handler")));
void tim1_trg_com_tim11_isr(void)__attribute__((weak, alias("null_handler")));
void tim1_cc_isr(void)           __attribute__((weak, alias("null_handler")));
void tim2_isr(void)              __attribute__((weak, alias("null_handler")));
void tim3_isr(void)              __attribute__((weak, alias("null_handler")));
void tim4_isr(void)              __attribute__((weak, alias("null_handler")));
void i2c1_ev_isr(void)           __attribute__((weak, alias("null_handler")));
void i2c1_er_isr(void)           __attribute__((weak, alias("null_handler")));
void i2c2_ev_isr(void)           __attribute__((weak, alias("null_handler")));
void i2c2_er_isr(void)           __attribute__((weak, alias("null_handler")));
void spi1_isr(void)              __attribute__((weak, alias("null_handler")));
void spi2_isr(void)              __attribute__((weak, alias("null_handler")));
void usart1_isr(void)            __attribute__((weak, alias("null_handler")));
void usart2_isr(void)            __attribute__((weak, alias("null_handler")));
void usart3_isr(void)            __attribute__((weak, alias("null_handler")));
void exti15_10_isr(void)         __attribute__((weak, alias("null_handler")));
void rtc_alarm_isr(void)         __attribute__((weak, alias("null_handler")));
void usb_fs_wkup_isr(void)       __attribute__((weak, alias("null_handler")));
void tim8_brk_tim12_isr(void)    __attribute__((weak, alias("null_handler")));
void tim8_up_tim13_isr(void)     __attribute__((weak, alias("null_handler")));
void tim8_trg_com_tim14_isr(void)__attribute__((weak, alias("null_handler")));
void tim8_cc_isr(void)           __attribute__((weak, alias("null_handler")));
void dma1_stream7_isr(void)      __attribute__((weak, alias("null_handler")));
void fsmc_isr(void)              __attribute__((weak, alias("null_handler")));
void sdio_isr(void)              __attribute__((weak, alias("null_handler")));
void tim5_isr(void)              __attribute__((weak, alias("null_handler")));
void spi3_isr(void)              __attribute__((weak, alias("null_handler")));
void uart4_isr(void)             __attribute__((weak, alias("null_handler")));
void uart5_isr(void)             __attribute__((weak, alias("null_handler")));
void tim6_dac_isr(void)          __attribute__((weak, alias("null_handler")));
void tim7_isr(void)              __attribute__((weak, alias("null_handler")));
void dma2_stream0_isr(void)      __attribute__((weak, alias("null_handler")));
void dma2_stream1_isr(void)      __attribute__((weak, alias("null_handler")));
void dma2_stream2_isr(void)      __attribute__((weak, alias("null_handler")));
void dma2_stream3_isr(void)      __attribute__((weak, alias("null_handler")));
void dma2_stream4_isr(void)      __attribute__((weak, alias("null_handler")));
void eth_isr(void)               __attribute__((weak, alias("null_handler")));
void eth_wkup_isr(void)          __attribute__((weak, alias("null_handler")));
void can2_tx_isr(void)           __attribute__((weak, alias("null_handler")));
void can2_rx0_isr(void)          __attribute__((weak, alias("null_handler")));
void can2_rx1_isr(void)          __attribute__((weak, alias("null_handler")));
void can2_sce_isr(void)          __attribute__((weak, alias("null_handler")));
void otg_fs_isr(void)            __attribute__((weak, alias("null_handler")));
void dma2_stream5_isr(void)      __attribute__((weak, alias("null_handler")));
void dma2_stream6_isr(void)      __attribute__((weak, alias("null_handler")));
void dma2_stream7_isr(void)      __attribute__((weak, alias("null_handler")));
void usart6_isr(void)            __attribute__((weak, alias("null_handler")));
void i2c3_ev_isr(void)           __attribute__((weak, alias("null_handler")));
void i2c3_er_isr(void)           __attribute__((weak, alias("null_handler")));
void otg_hs_ep1_out_isr(void)    __attribute__((weak, alias("null_handler")));
void otg_hs_ep1_in_isr(void)     __attribute__((weak, alias("null_handler")));
void otg_hs_wkup_isr(void)       __attribute__((weak, alias("null_handler")));
void otg_hs_isr(void)            __attribute__((weak, alias("null_handler")));
void dcmi_isr(void)              __attribute__((weak, alias("null_handler")));
void cryp_isr(void)              __attribute__((weak, alias("null_handler")));
void hash_rng_isr(void)          __attribute__((weak, alias("null_handler")));
void fpu_isr(void)               __attribute__((weak, alias("null_handler")));
void uart7_isr(void)             __attribute__((weak, alias("null_handler")));
void uart8_isr(void)             __attribute__((weak, alias("null_handler")));
void spi4_isr(void)              __attribute__((weak, alias("null_handler")));
void spi5_isr(void)              __attribute__((weak, alias("null_handler")));
void spi6_isr(void)              __attribute__((weak, alias("null_handler")));
void sai1_isr(void)              __attribute__((weak, alias("null_handler")));
void lcd_tft_isr(void)           __attribute__((weak, alias("null_handler")));
void lcd_tft_err_isr(void)       __attribute__((weak, alias("null_handler")));
void dma2d_isr(void)             __attribute__((weak, alias("null_handler")));

__attribute__ ((section(".vectors")))
vector_table_t vector_table = {
	.initial_sp    = &_stack,
	.reset         = reset_handler,
	.nmi           = nmi_handler,
	.hard_fault    = hard_fault_handler,
	.mem_fault     = mem_fault_handler,
	.bus_fault     = bus_fault_handler,
	.usage_fault   = usage_fault_handler,
	.sv_call       = sv_call_handler,
	.debug_monitor = debug_monitor_handler,
	.pend_sv       = pend_sv_handler,
	.systick       = systick_handler,
	.irq = {
		[0]  = nvic_wwdg_isr,
		[1]  = pvd_isr,
		[2]  = tamp_stamp_isr,
		[3]  = rtc_wkup_isr,
		[4]  = flash_isr,
		[5]  = rcc_isr,
		[6]  = exti0_isr,
		[7]  = exti1_isr,
		[8]  = exti2_isr,
		[9]  = exti3_isr,
		[10] = exti4_isr,
		[11] = dma1_stream0_isr,
		[12] = dma1_stream1_isr,
		[13] = dma1_stream2_isr,
		[14] = dma1_stream3_isr,
		[15] = dma1_stream4_isr,
		[16] = dma1_stream5_isr,
		[17] = dma1_stream6_isr,
		[18] = adc_isr,
		[19] = can1_tx_isr,
		[20] = can1_rx0_isr,
		[21] = can1_rx1_isr,
		[22] = can1_sce_isr,
		[23] = exti9_5_isr,
		[24] = tim1_brk_tim9_isr,
		[25] = tim1_up_tim10_isr,
		[26] = tim1_trg_com_tim11_isr,
		[27] = tim1_cc_isr,
		[28] = tim2_isr,
		[29] = tim3_isr,
		[30] = tim4_isr,
		[31] = i2c1_ev_isr,
		[32] = i2c1_er_isr,
		[33] = i2c2_ev_isr,
		[34] = i2c2_er_isr,
		[35] = spi1_isr,
		[36] = spi2_isr,
		[37] = usart1_isr,
		[38] = usart2_isr,
		[39] = usart3_isr,
		[40] = exti15_10_isr,
		[41] = rtc_alarm_isr,
		[42] = usb_fs_wkup_isr,
		[43] = tim8_brk_tim12_isr,
		[44] = tim8_up_tim13_isr,
		[45] = tim8_trg_com_tim14_isr,
		[46] = tim8_cc_isr,
		[47] = dma1_stream7_isr,
		[48] = fsmc_isr,
		[49] = sdio_isr,
		[50] = tim5_isr,
		[51] = spi3_isr,
		[52] = uart4_isr,
		[53] = uart5_isr,
		[54] = tim6_dac_isr,
		[55] = tim7_isr,
		[56] = dma2_stream0_isr,
		[57] = dma2_stream1_isr,
		[58] = dma2_stream2_isr,
		[59] = dma2_stream3_isr,
		[60] = dma2_stream4_isr,
		[61] = eth_isr,
		[62] = eth_wkup_isr,
		[63] = can2_tx_isr,
		[64] = can2_rx0_isr,
		[65] = can2_rx1_isr,
		[66] = can2_sce_isr,
		[67] = otg_fs_isr,
		[68] = dma2_stream5_isr,
		[69] = dma2_stream6_isr,
		[70] = dma2_stream7_isr,
		[71] = usart6_isr,
		[72] = i2c3_ev_isr,
		[73] = i2c3_er_isr,
		[74] = otg_hs_ep1_out_isr,
		[75] = otg_hs_ep1_in_isr,
		[76] = otg_hs_wkup_isr,
		[77] = otg_hs_isr,
		[78] = dcmi_isr,
		[79] = cryp_isr,
		[80] = hash_rng_isr,
		[81] = fpu_isr,
		[82] = uart7_isr,
		[83] = uart8_isr,
		[84] = spi4_isr,
		[85] = spi5_isr,
		[86] = spi6_isr,
		[87] = sai1_isr,
		[88] = lcd_tft_isr,
		[89] = lcd_tft_err_isr,
		[90] = dma2d_isr,
	},
};
