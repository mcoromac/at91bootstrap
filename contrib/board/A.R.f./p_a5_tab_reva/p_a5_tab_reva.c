#include "p_a5_tab_reva.h"
#include "common.h"
#include "ddramc.h"
#include "debug.h"
#include "gpio.h"
#include "hardware.h"
#include "l2cc.h"
#include "matrix.h"
#include "pmc.h"
#include "timer.h"
#include "twi.h"
#include "usart.h"
#include "watchdog.h"
#include "arch/at91_ddrsdrc.h"
#include "arch/at91_pio.h"
#include "arch/at91_pmc.h"
#include "arch/at91_rstc.h"
#include "arch/tz_matrix.h"


static void at91_dbgu_hw_init(void)
{
	const struct pio_desc dbgu_pins[] = {
		{"RXD1", CONFIG_SYS_DBGU_RXD_PIN, 0, PIO_DEFAULT, PIO_PERIPH_D},
		{"TXD1", CONFIG_SYS_DBGU_TXD_PIN, 0, PIO_DEFAULT, PIO_PERIPH_D},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_DBGU_ID);
	*(volatile unsigned int*)AT91C_BASE_FLEXCOM2 = 1;
	pio_configure(dbgu_pins);
}

static void initialize_dbgu(void)
{
	const unsigned int baudrate = 115200;

	at91_dbgu_hw_init();

	if (pmc_check_mck_h32mxdiv())
		usart_init(BAUDRATE(MASTER_CLOCK / 2, baudrate));
	else
		usart_init(BAUDRATE(MASTER_CLOCK, baudrate));
}

#if defined(CONFIG_MATRIX)
static int matrix_configure_slave(void)
{
	unsigned int ddr_port;
	unsigned int ssr_setting, sasplit_setting, srtop_setting;

	/*
	 * Matrix 0 (H64MX)
	 */

	/*
	 * 0: Bridge from H64MX to AXIMX
	 * (Internal ROM, Crypto Library, PKCC RAM): Always Secured
	 */

	/* 1: H64MX Peripheral Bridge */

	/* 2 ~ 9 DDR2 Port1 ~ 7: Non-Secure */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_128M);
	sasplit_setting = (MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(1, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(2, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(3, MATRIX_SASPLIT_VALUE_128M));
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_LANSECH_NS(1)
			| MATRIX_LANSECH_NS(2)
			| MATRIX_LANSECH_NS(3)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_RDNSECH_NS(1)
			| MATRIX_RDNSECH_NS(2)
			| MATRIX_RDNSECH_NS(3)
			| MATRIX_WRNSECH_NS(0)
			| MATRIX_WRNSECH_NS(1)
			| MATRIX_WRNSECH_NS(2)
			| MATRIX_WRNSECH_NS(3));
	/* DDR port 0 not used from NWd */
	for (ddr_port = 1; ddr_port < 8; ddr_port++) {
		matrix_configure_slave_security(AT91C_BASE_MATRIX64,
					(H64MX_SLAVE_DDR2_PORT_0 + ddr_port),
					srtop_setting,
					sasplit_setting,
					ssr_setting);
	}

	/*
	 * 10: Internal SRAM 128K
	 * TOP0 is set to 128K
	 * SPLIT0 is set to 64K
	 * LANSECH0 is set to 0, the low area of region 0 is the Securable one
	 * RDNSECH0 is set to 0, region 0 Securable area is secured for reads.
	 * WRNSECH0 is set to 0, region 0 Securable area is secured for writes
	 */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_128K);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_64K);
	ssr_setting = (MATRIX_LANSECH_S(0)
			| MATRIX_RDNSECH_S(0)
			| MATRIX_WRNSECH_S(0));
	matrix_configure_slave_security(AT91C_BASE_MATRIX64,
					H64MX_SLAVE_INTERNAL_SRAM,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 11:  Internal SRAM 128K (Cache L2) */
	/* 12:  QSPI0 */
	/* 13:  QSPI1 */
	/* 14:  AESB */

	/*
	 * Matrix 1 (H32MX)
	 */

	/* 0: Bridge from H32MX to H64MX: Not Secured */

	/* 1: H32MX Peripheral Bridge 0: Not Secured */

	/* 2: H32MX Peripheral Bridge 1: Not Secured */

	/*
	 * 3: External Bus Interface
	 * EBI CS0 Memory(256M) ----> Slave Region 0, 1
	 * EBI CS1 Memory(256M) ----> Slave Region 2, 3
	 * EBI CS2 Memory(256M) ----> Slave Region 4, 5
	 * EBI CS3 Memory(128M) ----> Slave Region 6
	 * NFC Command Registers(128M) -->Slave Region 7
	 *
	 * NANDFlash(EBI CS3) --> Slave Region 6: Non-Secure
	 */
	srtop_setting =	MATRIX_SRTOP(6, MATRIX_SRTOP_VALUE_128M);
	srtop_setting |= MATRIX_SRTOP(7, MATRIX_SRTOP_VALUE_128M);
	sasplit_setting = MATRIX_SASPLIT(6, MATRIX_SASPLIT_VALUE_128M);
	sasplit_setting |= MATRIX_SASPLIT(7, MATRIX_SASPLIT_VALUE_128M);
	ssr_setting = (MATRIX_LANSECH_NS(6)
			| MATRIX_RDNSECH_NS(6)
			| MATRIX_WRNSECH_NS(6));
	ssr_setting |= (MATRIX_LANSECH_NS(7)
			| MATRIX_RDNSECH_NS(7)
			| MATRIX_WRNSECH_NS(7));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_EXTERNAL_EBI,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 4: NFC SRAM (4K): Non-Secure */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_8K);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_8K);
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_WRNSECH_NS(0));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_NFC_SRAM,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 5:
	 * USB Device High Speed Dual Port RAM (DPR): 1M
	 * USB Host OHCI registers: 1M
	 * USB Host EHCI registers: 1M
	 */
	srtop_setting = (MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_1M)
			| MATRIX_SRTOP(1, MATRIX_SRTOP_VALUE_1M)
			| MATRIX_SRTOP(2, MATRIX_SRTOP_VALUE_1M));
	sasplit_setting = (MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_1M)
			| MATRIX_SASPLIT(1, MATRIX_SASPLIT_VALUE_1M)
			| MATRIX_SASPLIT(2, MATRIX_SASPLIT_VALUE_1M));
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_LANSECH_NS(1)
			| MATRIX_LANSECH_NS(2)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_RDNSECH_NS(1)
			| MATRIX_RDNSECH_NS(2)
			| MATRIX_WRNSECH_NS(0)
			| MATRIX_WRNSECH_NS(1)
			| MATRIX_WRNSECH_NS(2));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_USB,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	return 0;
}

static unsigned int security_ps_peri_id[] = {
	0,
};

static int matrix_config_periheral(void)
{
	unsigned int *peri_id = security_ps_peri_id;
	unsigned int array_size = sizeof(security_ps_peri_id) / sizeof(unsigned int);
	int ret;

	ret = matrix_configure_peri_security(peri_id, array_size);
	if (ret)
		return -1;

	return 0;
}

static int matrix_init(void)
{
	int ret;

	matrix_write_protect_disable(AT91C_BASE_MATRIX64);
	matrix_write_protect_disable(AT91C_BASE_MATRIX32);

	ret = matrix_configure_slave();
	if (ret)
		return -1;

	ret = matrix_config_periheral();
	if (ret)
		return -1;

	return 0;
}
#endif

static void ddramc_reg_config(struct ddramc_register* ddramc_config)
{
	ddramc_config->mdr = AT91C_DDRC2_DBW_16_BITS |
			     AT91C_DDRC2_MD_DDR2_SDRAM;

	ddramc_config->cr = AT91C_DDRC2_NC_DDR10_SDR9 |
			    AT91C_DDRC2_NR_13 |
			    AT91C_DDRC2_CAS_3 |
			    AT91C_DDRC2_DISABLE_RESET_DLL |
			    AT91C_DDRC2_WEAK_STRENGTH_RZQ7 |
			    AT91C_DDRC2_ENABLE_DLL |
			    AT91C_DDRC2_NB_BANKS_8 |
			    AT91C_DDRC2_NDQS_ENABLED |
			    AT91C_DDRC2_DECOD_INTERLEAVED |
			    AT91C_DDRC2_UNAL_SUPPORTED;

	ddramc_config->rtr = 0x388;

	ddramc_config->t0pr = AT91C_DDRC2_TRAS_(7) |
			      AT91C_DDRC2_TRCD_(3) |
			      AT91C_DDRC2_TWR_(3) |
			      AT91C_DDRC2_TRC_(9) |
			      AT91C_DDRC2_TRP_(3) |
			      AT91C_DDRC2_TRRD_(2) |
			      AT91C_DDRC2_TWTR_(2) |
			      AT91C_DDRC2_TMRD_(2);

	ddramc_config->t1pr = AT91C_DDRC2_TRFC_(22) |
			      AT91C_DDRC2_TXSNR_(23) |
			      AT91C_DDRC2_TXSRD_(200) |
			      AT91C_DDRC2_TXP_(2);

	ddramc_config->t2pr = AT91C_DDRC2_TXARD_(2) |
			      AT91C_DDRC2_TXARDS_(8) |
			      AT91C_DDRC2_TRPA_(4) |
			      AT91C_DDRC2_TRTP_(2) |
			      AT91C_DDRC2_TFAW_(8);
}


static void ddramc_init(void)
{
	struct ddramc_register ddramc_reg;
	unsigned int reg;

	ddramc_reg_config(&ddramc_reg);

	pmc_enable_periph_clock(AT91C_ID_MPDDRC);
	pmc_enable_system_clock(AT91C_PMC_DDR);

	reg = AT91C_MPDDRC_RD_DATA_PATH_ONE_CYCLES;
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_RD_DATA_PATH));

	reg = readl(AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);
	reg &= ~AT91C_MPDDRC_RDIV;
	reg &= ~AT91C_MPDDRC_TZQIO;
	reg |= AT91C_MPDDRC_RDIV_DDR2_RZQ_50;
	reg |= AT91C_MPDDRC_TZQIO_(101);
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

	ddram_initialize(AT91C_BASE_MPDDRC, AT91C_BASE_DDRCS, &ddramc_reg);
}


/*static void init_prog_mode_hw(void)
{
	const struct pio_desc progmode_pins[] = {
		{"PROGMODE_IN", AT91C_PIN_PD(26), 0, PIO_PULLUP, PIO_INPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(progmode_pins);
}*/

/*static void check_prog_mode_condition(void)
{
	if( pio_get_value(AT91C_PIN_PD(26)) )
	{
		dbg_info("--- NORMAL OPERATION ---\n");
		return;
	}

	dbg_info("!!! I will now enter PROG MODE !!!\n");

	*(volatile unsigned int*)0xF8045400 = (0xF << 12) | (1 << 11) | (1 << 10) | (3 << 8) | (3 << 6) | (3 << 4) | (3 << 2) | 3; //BUREG0
	*(volatile unsigned int*)0xF8048054 = (0x6683 << 24) | (1 << 2) | 0; //BSC_CR
	*(volatile unsigned int*)0xF8048000 = (0xA5 << 24) | 1; //reset
	while(1);
}*/


#ifdef CONFIG_HW_INIT
void hw_init(void)
{
	at91_disable_wdt();

//	init_prog_mode_hw();

	pmc_cfg_plla(PLLA_SETTINGS);
	pmc_cfg_mck(BOARD_PRESCALER_PLLA);

	writel(AT91C_RSTC_KEY_UNLOCK | AT91C_RSTC_URSTEN, AT91C_BASE_RSTC + RSTC_RMR);

#if defined(CONFIG_MATRIX)
	matrix_init();
#endif

	initialize_dbgu();
//	check_prog_mode_condition();

	timer_init();
	ddramc_init();
	l2cache_prepare();
}
#endif

#ifdef CONFIG_SDCARD
#define ATMEL_SDHC_GCKDIV_VALUE		1

#define SDMMC_CACR		(((volatile unsigned*)AT91C_BASE_SDHC0)[0x230/sizeof(unsigned)])
#define SDMMC_CACR_UNLOCK_VAL	((0x46 << 8) | 1)
#define SDMMC_CACR_LOCK_VAL	((0x46 << 8) | 0)

#define SDMMC_CA0R		(((volatile unsigned*)AT91C_BASE_SDHC0)[0x40/sizeof(unsigned)])
#define SDMMC_CA0R_SLTYPE(x)	((x) << 30)
#define SDMMC_CA0R_SUP1V8	(1 << 26)
#define SDMMC_CA0R_SUP3V3	(1 << 24)

#define SDMMC_MC1R		(((volatile unsigned char*)AT91C_BASE_SDHC0)[0x204])
#define SDMMC_MC1R_FCD		(1 << 7)

void at91_sdhc_hw_init(void)
{
#ifdef CONFIG_SDHC0
	const struct pio_desc sdmmc_pins[] = {
		{"SDMMC0_CK",   AT91C_PIN_PA(0), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_CMD",  AT91C_PIN_PA(1), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT0", AT91C_PIN_PA(2), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT1", AT91C_PIN_PA(3), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT2", AT91C_PIN_PA(4), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT3", AT91C_PIN_PA(5), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT4", AT91C_PIN_PA(6), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT5", AT91C_PIN_PA(7), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT6", AT91C_PIN_PA(8), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT7", AT91C_PIN_PA(9), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_RSTN", AT91C_PIN_PA(10), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_VDDSEL", AT91C_PIN_PA(11), 0, PIO_DEFAULT, PIO_PERIPH_A},
//		{"SDMMC0_WP",   AT91C_PIN_PA(12), 0, PIO_DEFAULT, PIO_PERIPH_A},
//		{"SDMMC0_CD",   AT91C_PIN_PA(13), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#endif

	pio_configure(sdmmc_pins);

	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_ID_SDHC);
	pmc_enable_periph_generated_clk(CONFIG_SYS_ID_SDHC,
					GCK_CSS_UPLL_CLK,
					ATMEL_SDHC_GCKDIV_VALUE);
	SDMMC_CACR = SDMMC_CACR_UNLOCK_VAL;
	SDMMC_CA0R |= SDMMC_CA0R_SLTYPE(1) | SDMMC_CA0R_SUP1V8 | SDMMC_CA0R_SUP3V3;
	SDMMC_CACR = SDMMC_CACR_LOCK_VAL;
	SDMMC_MC1R |= SDMMC_MC1R_FCD;	//force CD for eMMC
}
#endif

#if defined(CONFIG_TWI0)
unsigned int at91_twi0_hw_init(void)
{
	unsigned int base_addr = AT91C_BASE_TWI0;

	const struct pio_desc twi_pins[] = {
		{"TWD0", AT91C_PIN_PC(27), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"TWCK0", AT91C_PIN_PC(28), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(twi_pins);

	pmc_sam9x5_enable_periph_clk(AT91C_ID_TWI0);

	return base_addr;
}
#endif

#if defined(CONFIG_TWI1)
unsigned int at91_twi1_hw_init(void)
{
	const struct pio_desc twi_pins[] = {
		{"TWD1", AT91C_PIN_PD(4), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"TWCK1", AT91C_PIN_PD(5), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(twi_pins);

	pmc_sam9x5_enable_periph_clk(AT91C_ID_TWI1);

	return AT91C_BASE_TWI1;
}
#endif

#if defined(CONFIG_AUTOCONFIG_TWI_BUS)
void at91_board_config_twi_bus(void)
{
	act8865_twi_bus = 0;
}
#endif

#if defined(CONFIG_ACT8865_SET_VOLTAGE)
int at91_board_act8865_set_reg_voltage(void)
{
	return 0;
}
#endif
