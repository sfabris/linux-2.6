/*
 *  linux/arch/arm/mach-pxa/littleton.c
 *
 *  Support for the Marvell Littleton Development Platform.
 *
 *  Author:	Jason Chagas (largely modified code)
 *  Created:	Nov 20, 2006
 *  Copyright:	(C) Copyright 2006 Marvell International Ltd.
 *
 *  2007-11-22  modified to align with latest kernel
 *              eric miao <eric.miao@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c/max7320.h>
#include <linux/i2c/max7321.h>
#include <linux/spi/spi.h>
#include <linux/smc91x.h>
#include <linux/pda_power.h>

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/pxa-regs.h>
#include <mach/mfp-pxa300.h>
#include <mach/gpio.h>
#include <mach/pxafb.h>
#include <mach/ssp.h>
#include <mach/pxa2xx_spi.h>
#include <mach/pxa27x_keypad.h>
#include <mach/littleton.h>
#include <mach/pxa3xx_nand.h>
#include <mach/pxa3xx_pmic.h>
#include <mach/pxa3xx_dvfm.h>
#include <mach/micco.h>
#include <mach/ohci.h>
#include <mach/pxa3xx_u2d.h>
#include <mach/udc.h>
#include <mach/uart.h>
#include <mach/irda.h>
#include <mach/mmc.h>
#include <mach/imm.h>
#include <mach/pxa3xx_pm.h>
#include <mach/pmu.h>
#include <mach/part_table.h>
#include <mach/camera.h>

#include <linux/suspend.h>
#include "devices.h"
#include "generic.h"

#define MAX_SLOTS      3
struct platform_mmc_slot littleton_mmc_slot[MAX_SLOTS];
extern int is_android(void);

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

/* Littleton MFP configurations */
static mfp_cfg_t littleton_mfp_cfg[] __initdata = {
	/* LCD */
	GPIO54_LCD_LDD_0,
	GPIO55_LCD_LDD_1,
	GPIO56_LCD_LDD_2,
	GPIO57_LCD_LDD_3,
	GPIO58_LCD_LDD_4,
	GPIO59_LCD_LDD_5,
	GPIO60_LCD_LDD_6,
	GPIO61_LCD_LDD_7,
	GPIO62_LCD_LDD_8,
	GPIO63_LCD_LDD_9,
	GPIO64_LCD_LDD_10,
	GPIO65_LCD_LDD_11,
	GPIO66_LCD_LDD_12,
	GPIO67_LCD_LDD_13,
	GPIO68_LCD_LDD_14,
	GPIO69_LCD_LDD_15,
	GPIO70_LCD_LDD_16,
	GPIO71_LCD_LDD_17,
	GPIO72_LCD_FCLK,
	GPIO73_LCD_LCLK,
	GPIO74_LCD_PCLK,
	GPIO75_LCD_BIAS,

	/* I2C */
	GPIO21_I2C_SCL,
	GPIO22_I2C_SDA,

	/* BTUART */
	GPIO111_UART2_RTS,
	GPIO112_UART2_RXD,
	GPIO113_UART2_TXD,
	GPIO114_UART2_CTS,

	/* STUART */
	GPIO109_UART3_TXD,
	GPIO110_UART3_RXD,

	/* Keypad */
	GPIO107_KP_DKIN_0,
	GPIO108_KP_DKIN_1,
	GPIO115_KP_MKIN_0,
	GPIO116_KP_MKIN_1,
	GPIO117_KP_MKIN_2,
	GPIO118_KP_MKIN_3,
	GPIO119_KP_MKIN_4,
	GPIO120_KP_MKIN_5,
	GPIO121_KP_MKOUT_0,
	GPIO122_KP_MKOUT_1,
	GPIO123_KP_MKOUT_2,
	GPIO124_KP_MKOUT_3,
	GPIO125_KP_MKOUT_4,

	/* SSP2 */
	GPIO25_SSP2_SCLK,
	GPIO17_SSP2_FRM,
	GPIO27_SSP2_TXD,

	/* SSP3 */
	GPIO91_SSP3_SCLK,
	GPIO92_SSP3_FRM,
	GPIO93_SSP3_TXD,
	GPIO94_SSP3_RXD,
	/* SSP3 NETWORK CLK */
	GPIO126_EXT_CLK,

	/* SSP4 */
	GPIO95_SSP4_SCLK,
	GPIO96_SSP4_FRM,
	GPIO97_SSP4_TXD,
	GPIO98_SSP4_RXD,

	/* NAND */

	/* PMIC */
	GPIO18_GPIO,

	/* IRDA */
	GPIO16_GPIO,

	/* MMC1 */
	GPIO3_MMC1_DAT0,
	GPIO4_MMC1_DAT1,
	GPIO5_MMC1_DAT2,
	GPIO6_MMC1_DAT3,
	GPIO7_MMC1_CLK,
	GPIO8_MMC1_CMD,
	GPIO15_GPIO, /* card detect */

	/* MMC2 */
	GPIO9_MMC2_DAT0,
	GPIO10_MMC2_DAT1,
	GPIO11_MMC2_DAT2,
	GPIO12_MMC2_DAT3,
	GPIO13_MMC2_CLK,
	GPIO14_MMC2_CMD,

#ifdef CONFIG_CPU_PXA310
	/* MMC3 */
	GPIO7_2_MMC3_DAT0,
	GPIO8_2_MMC3_DAT1,
	GPIO9_2_MMC3_DAT2,
	GPIO10_2_MMC3_DAT3,
	GPIO103_MMC3_CLK,
	GPIO105_MMC3_CMD,
#endif

	/* 1 wire*/
	GPIO20_OW_DQ_IN,

	/* max7321 */
	GPIO77_GPIO,

	/* QCI */
	GPIO39_CI_DD_0,
	GPIO40_CI_DD_1,
	GPIO41_CI_DD_2,
	GPIO42_CI_DD_3,
	GPIO43_CI_DD_4,
	GPIO44_CI_DD_5,
	GPIO45_CI_DD_6,
	GPIO46_CI_DD_7,
	GPIO47_CI_DD_8,
	GPIO48_CI_DD_9,
	GPIO49_CI_MCLK,
	GPIO50_CI_PCLK,

	/* Debug Ethernet */
	GPIO90_GPIO,
};

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
static void littleton_irda_transceiver_mode(struct device *dev, int mode)
{
	unsigned long flags;
	int err;
	static int irda_mfp_init;
	int gpio_cir = ((struct pxaficp_platform_data *)
			    dev->platform_data)->gpio_cir;
	int gpio_ir_shdn = ((struct pxaficp_platform_data *)
			    dev->platform_data)->gpio_ir_shdn;

	if (!irda_mfp_init) {
		err = gpio_request(gpio_cir, "IRDA CIR");
		if (err) {
			gpio_free(gpio_cir);
			printk(KERN_ERR "Request GPIO failed,"
			       "gpio: %d return :%d\n", gpio_cir, err);
			return;
		}

		err = gpio_request(gpio_ir_shdn, "IRDA SHDN");
		if (err) {
			gpio_free(gpio_ir_shdn);
			printk(KERN_ERR "Request GPIO failed,"
			       "gpio: %d return :%d\n", gpio_ir_shdn, err);
			return;
		}
		gpio_direction_output(gpio_cir, 0);
		gpio_direction_output(gpio_ir_shdn, 1);

		irda_mfp_init = 1;
	}

	local_irq_save(flags);
	if (mode & IR_SIRMODE) {
		gpio_set_value(gpio_ir_shdn, 0);
	} else if (mode & IR_FIRMODE) {
		/* do not support FIR */
	}
	if (mode & IR_OFF)
		gpio_set_value(gpio_ir_shdn, 1);

	local_irq_restore(flags);
}

static struct pxaficp_platform_data littleton_ficp_platform_data = {
	.transceiver_cap  = IR_SIRMODE | IR_OFF,
	.transceiver_mode = littleton_irda_transceiver_mode,
	.gpio_cir = mfp_to_gpio(MFP_PIN_GPIO16),
	.gpio_ir_shdn = EXT0_GPIO(6),
	.uart_irq = IRQ_STUART,
	.uart_reg_base = __PREG(STUART),
};
#endif /* (CONFIG_PXA_IRDA) || (CONFIG_PXA_IRDA_MODULE) */

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
static mfp_cfg_t littleton_u2d_cfg[] __initdata = {
	/* ULPI*/
	GPIO29_GPIO,
	GPIO38_ULPI_CLK,
	GPIO30_ULPI_DATA_OUT_0,
	GPIO31_ULPI_DATA_OUT_1,
	GPIO32_ULPI_DATA_OUT_2,
	GPIO33_ULPI_DATA_OUT_3,
	GPIO34_ULPI_DATA_OUT_4,
	GPIO35_ULPI_DATA_OUT_5,
	GPIO36_ULPI_DATA_OUT_6,
	GPIO37_ULPI_DATA_OUT_7,
	GPIO33_ULPI_OTG_INTR,
	ULPI_DIR,
	ULPI_NXT,
	ULPI_STP,
};

static void littleton_reset_xcvr(void)
{
	int reset_pin;
	int err;

	reset_pin = ULPI_RESET_PIN;
	err = gpio_request(reset_pin, "U2D Reset");
	if (err) {
		gpio_free(reset_pin);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d return :%d\n", reset_pin, err);
		return;
	}
	gpio_direction_output(reset_pin, 0);
	mdelay(100);
	gpio_set_value(reset_pin, 1);

	gpio_direction_input(reset_pin);

	gpio_free(reset_pin);
}

static mfp_cfg_t pxa310_ulpidat3_enable[] = {
	GPIO33_GPIO,
};

static mfp_cfg_t pxa310_ulpidat3_disable[] = {
	GPIO33_ULPI_DATA_OUT_3,
};

static void littleton_pxa310_ulpi_dat3(int enable)
{
	if (enable)
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa310_ulpidat3_enable));
	else
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa310_ulpidat3_disable));
}

static struct pxa3xx_u2d_mach_info littleton_u2d_info = {
	.reset_xcvr = littleton_reset_xcvr,
	.ulpi_dat3 = littleton_pxa310_ulpi_dat3,
};
#endif /* CONFIG_USB_PXA3XX_U2D || CONFIG_USB_PXA3XX_U2D_MODULE */

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= (LITTLETON_ETH_PHYS + 0x300),
		.end	= (LITTLETON_ETH_PHYS + 0xfffff),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO90)),
		.end	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO90)),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	}
};

static struct smc91x_platdata littleton_smc91x_info = {
	.flags	= SMC91X_USE_8BIT | SMC91X_USE_16BIT |
		  SMC91X_NOWAIT | SMC91X_USE_DMA,
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
	.dev		= {
		.platform_data = &littleton_smc91x_info,
	},
};

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int littleton_matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(1, 3, KEY_0), KEY(0, 0, KEY_1), KEY(1, 0, KEY_2), KEY(2, 0, KEY_3),
	KEY(0, 1, KEY_4), KEY(1, 1, KEY_5), KEY(2, 1, KEY_6), KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8), KEY(2, 2, KEY_9),

	KEY(0, 3, KEY_KPASTERISK), 	/* * */
	KEY(2, 3, KEY_KPDOT), 		/* # */

	KEY(5, 4, KEY_ENTER),

	KEY(5, 0, KEY_UP),
	KEY(5, 1, KEY_DOWN),
	KEY(5, 2, KEY_LEFT),
	KEY(5, 3, KEY_RIGHT),
	KEY(3, 2, KEY_HOME),
	KEY(4, 1, KEY_END),
	KEY(3, 3, KEY_BACK),

	KEY(4, 0, KEY_SEND),
	KEY(4, 2, KEY_VOLUMEUP),
	KEY(4, 3, KEY_VOLUMEDOWN),

	KEY(3, 0, KEY_F22),	/* soft1 */
	KEY(3, 1, KEY_F23),	/* soft2 */
};

static struct pxa27x_keypad_platform_data littleton_keypad_info = {
	.matrix_key_rows	= 6,
	.matrix_key_cols	= 5,
	.matrix_key_map		= littleton_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(littleton_matrix_key_map),

	.enable_rotary0		= 1,
	.rotary0_up_key		= KEY_UP,
	.rotary0_down_key	= KEY_DOWN,

	.debounce_interval	= 30,
};
static void __init littleton_init_keypad(void)
{
	pxa_set_keypad_info(&littleton_keypad_info);
}
#else
static inline void littleton_init_keypad(void) {}
#endif

static struct platform_device micco_ts_device = {
	.name 		= "micco-ts",
	.id 		= -1,
};

static struct platform_device micco_bl_device = {
	.name 		= "micco-bl",
	.id 		= -1,
};

static struct platform_device micco_kp_bl_device = {
	.name 		= "micco-kp-bl",
	.id 		= -1,
};

static struct resource pxa3xx_resource_imm[] = {
	[0] = {
		.name   = "phy_sram",
		.start	= 0x5c000000,
		.end	= 0x5c000000 + 2 * PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name   = "imm_sram",
		.start	= 0x5c000000 + PHYS_SRAM_BANK_SIZE,
		.end	= 0x5c000000 + 2 * PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device pxa3xx_device_imm = {
	.name 		= "pxa3xx-imm",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_imm),
	.resource	= pxa3xx_resource_imm,
};

static struct platform_device micco_charger_device = {
	.name		= "micco-charger",
	.id		= -1,
};

/*
 * External power
 */

static int littleton_is_usb_online(void)
{
	return 1;
}

static char *littleton_supplicants[] = {
	"ds278x-battery", "backup-battery"
};

static struct pda_power_pdata power_supply_info = {
	.is_usb_online   = littleton_is_usb_online,
	.supplied_to     = littleton_supplicants,
	.num_supplicants = ARRAY_SIZE(littleton_supplicants),
};

static struct resource power_supply_resources[] = {
	[0] = {
		.name  = "usb",
	},
};

static struct platform_device power_supply = {
	.name = "pda-power",
	.id   = -1,
	.dev  = {
		.platform_data = &power_supply_info,
	},
	.resource      = power_supply_resources,
	.num_resources = ARRAY_SIZE(power_supply_resources),
};

static struct platform_device battery = {
	.name = "ds278x-battery",
	.id   = -1,
};

static void __init littleton_init_battery(void)
{
#if defined(CONFIG_BATTERY_DS278x) || defined(CONFIG_BATTERY_DS278x_MODULE)
	platform_device_register(&battery);
#endif
}

static struct platform_device *devices[] __initdata = {
	&smc91x_device,
	&micco_ts_device,
	&micco_bl_device,
	&micco_kp_bl_device,
	&pxa3xx_device_imm,
	&micco_charger_device,
	&power_supply,
};

#if defined(CONFIG_FB_PXA) || defined(CONFIG_FB_PXA_MODULE)
/* use bit 30, 31 as the indicator of command parameter number */
#define CMD0(x)		((0x00000000) | ((x) << 9))
#define CMD1(x, x1)	((0x40000000) | ((x) << 9) | 0x100 | (x1))
#define CMD2(x, x1, x2)	((0x80000000) | ((x) << 18) | 0x20000 |\
			 ((x1) << 9) | 0x100 | (x2))

static uint32_t lcd_panel_reset[] = {
	CMD0(0x1), /* reset */
	CMD0(0x0), /* nop */
	CMD0(0x0), /* nop */
	CMD0(0x0), /* nop */
};

static uint32_t lcd_panel_on[] = {
	CMD0(0x29),		/* Display ON */
	CMD2(0xB8, 0xFF, 0xF9),	/* Output Control */
	CMD0(0x11),		/* Sleep out */
	CMD1(0xB0, 0x16),	/* Wake */
};

static uint32_t lcd_panel_off[] = {
	CMD0(0x28),		/* Display OFF */
	CMD2(0xB8, 0x80, 0x02),	/* Output Control */
	CMD0(0x10),		/* Sleep in */
	CMD1(0xB0, 0x00),	/* Deep stand by in */
};

static uint32_t qvga_ram_write_mode[] = {
        CMD1(0xB0, 0x17),
        CMD1(0xBC, 0xAA),
        CMD1(0x36, 0xC0),
        CMD1(0x3B, 0x22),
        CMD1(0xE1, 0x01),
};

static uint32_t qvga_osc_config[] = {
        CMD1(0xe0, 0x0A),
        CMD1(0xe2, 0x00),
        CMD1(0xe3, 0x32),
        CMD2(0xe4, 0x00, 0x03),
        CMD2(0xe5, 0x02, 0x04),
        CMD1(0xe6, 0x03),
        CMD2(0xe7, 0x04, 0x0A),
        CMD1(0xe8, 0x04),
        CMD1(0xe9, 0x10),
        CMD2(0xea, 0x20, 0x00),
        CMD0(0x21),
        CMD0(0x29),
};

static uint32_t lcd_vga_pass_through[] = {
	CMD1(0xB0, 0x16),
	CMD1(0xBC, 0x80),
	CMD1(0xE1, 0x00),
	CMD1(0x36, 0x50),
	CMD1(0x3B, 0x00),
};

static uint32_t lcd_qvga_pass_through[] = {
	CMD1(0xB0, 0x16),
	CMD1(0xBC, 0x81),
	CMD1(0xE1, 0x00),
	CMD1(0x36, 0x50),
	CMD1(0x3B, 0x22),
};

static uint32_t lcd_vga_transfer[] = {
	CMD1(0xcf, 0x02), 	/* Blanking period control (1) */
	CMD2(0xd0, 0x08, 0x04),	/* Blanking period control (2) */
	CMD1(0xd1, 0x01),	/* CKV timing control on/off */
	CMD2(0xd2, 0x14, 0x00),	/* CKV 1,2 timing control */
	CMD2(0xd3, 0x1a, 0x0f),	/* OEV timing control */
	CMD2(0xd4, 0x1f, 0xaf),	/* ASW timing control (1) */
	CMD1(0xd5, 0x14),	/* ASW timing control (2) */
	CMD0(0x21),		/* Invert for normally black display */
	CMD0(0x29),		/* Display on */
};

static uint32_t lcd_qvga_transfer[] = {
	CMD1(0xd6, 0x02),	/* Blanking period control (1) */
	CMD2(0xd7, 0x08, 0x04),	/* Blanking period control (2) */
	CMD1(0xd8, 0x01),	/* CKV timing control on/off */
	CMD2(0xd9, 0x00, 0x08),	/* CKV 1,2 timing control */
	CMD2(0xde, 0x05, 0x0a),	/* OEV timing control */
	CMD2(0xdf, 0x0a, 0x19),	/* ASW timing control (1) */
	CMD1(0xe0, 0x0a),	/* ASW timing control (2) */
	CMD0(0x21),		/* Invert for normally black display */
	CMD0(0x29),		/* Display on */
};

static uint32_t lcd_panel_config[] = {
	CMD2(0xb8, 0xff, 0xf9),	/* Output control */
	CMD0(0x11),		/* sleep out */
	CMD1(0xba, 0x01),	/* Display mode (1) */
	CMD1(0xbb, 0x00),	/* Display mode (2) */
	CMD1(0x3a, 0x60),	/* Display mode 18-bit RGB */
	CMD1(0xbf, 0x10),	/* Drive system change control */
	CMD1(0xb1, 0x56),	/* Booster operation setup */
	CMD1(0xb2, 0x33),	/* Booster mode setup */
	CMD1(0xb3, 0x11),	/* Booster frequency setup */
	CMD1(0xb4, 0x02),	/* Op amp/system clock */
	CMD1(0xb5, 0x35),	/* VCS voltage */
	CMD1(0xb6, 0x40),	/* VCOM voltage */
	CMD1(0xb7, 0x03),	/* External display signal */
	CMD1(0xbd, 0x00),	/* ASW slew rate */
	CMD1(0xbe, 0x00),	/* Dummy data for QuadData operation */
	CMD1(0xc0, 0x11),	/* Sleep out FR count (A) */
	CMD1(0xc1, 0x11),	/* Sleep out FR count (B) */
	CMD1(0xc2, 0x11),	/* Sleep out FR count (C) */
	CMD2(0xc3, 0x20, 0x40),	/* Sleep out FR count (D) */
	CMD2(0xc4, 0x60, 0xc0),	/* Sleep out FR count (E) */
	CMD2(0xc5, 0x10, 0x20),	/* Sleep out FR count (F) */
	CMD1(0xc6, 0xc0),	/* Sleep out FR count (G) */
	CMD2(0xc7, 0x33, 0x43),	/* Gamma 1 fine tuning (1) */
	CMD1(0xc8, 0x44),	/* Gamma 1 fine tuning (2) */
	CMD1(0xc9, 0x33),	/* Gamma 1 inclination adjustment */
	CMD1(0xca, 0x00),	/* Gamma 1 blue offset adjustment */
	CMD2(0xec, 0x01, 0xf0),	/* Horizontal clock cycles */
};

static void ssp_reconfig(struct ssp_dev *dev, int nparam)
{
	static int last_nparam = -1;

	/* check if it is necessary to re-config SSP */
	if (nparam == last_nparam)
		return;

	ssp_disable(dev);
	ssp_config(dev, (nparam == 2) ? 0x0010058a : 0x00100581, 0x18, 0, 0);

	last_nparam = nparam;
}

static struct ssp_dev ssp2;

static void ssp_send_cmd(uint32_t *cmd, int num)
{
	static int ssp_initialized;

	int i;

	if (!ssp_initialized) {
		ssp_init(&ssp2, 2, SSP_NO_IRQ);
		ssp_initialized = 1;
	}

	clk_enable(ssp2.ssp->clk);
	for (i = 0; i < num; i++, cmd++) {
		ssp_reconfig(&ssp2, (*cmd >> 30) & 0x3);
		ssp_write_word(&ssp2, *cmd & 0x3fffffff);

		/* FIXME: ssp_flush() is mandatory here to work */
		ssp_flush(&ssp2);
	}
	clk_disable(ssp2.ssp->clk);
}
static mfp_cfg_t littleton_ssp2_pins[] = {
        /* SSP2 */
        GPIO25_SSP2_SCLK,
        GPIO17_SSP2_FRM,
        GPIO27_SSP2_TXD,
};

void pxa3xx_enable_ssp2_pins(void)
{
        pxa3xx_mfp_config(littleton_ssp2_pins, ARRAY_SIZE(littleton_ssp2_pins));
}

void lcd_ssp_init(void)
{
        pxa3xx_enable_ssp2_pins();
        clk_enable(ssp2.ssp->clk);

        /* grab the port, configure it, then enable it */
        ssp_init(&ssp2, 2, SSP_NO_IRQ);
        ssp_disable(&ssp2);
        ssp_config(&ssp2, 0x0, 0x18, 0x0, 0x001fff81);
        ssp_enable(&ssp2);
        ssp_flush(&ssp2);
}

static mfp_cfg_t littleton_lcd_pins [] = {
        /* LCD */
        GPIO54_LCD_LDD_0,
        GPIO55_LCD_LDD_1,
        GPIO56_LCD_LDD_2,
        GPIO57_LCD_LDD_3,
        GPIO58_LCD_LDD_4,
        GPIO59_LCD_LDD_5,
        GPIO60_LCD_LDD_6,
        GPIO61_LCD_LDD_7,
        GPIO62_LCD_LDD_8,
        GPIO63_LCD_LDD_9,
        GPIO64_LCD_LDD_10,
        GPIO65_LCD_LDD_11,
        GPIO66_LCD_LDD_12,
        GPIO67_LCD_LDD_13,
        GPIO68_LCD_LDD_14,
        GPIO69_LCD_LDD_15,
        GPIO70_LCD_LDD_16,
        GPIO71_LCD_LDD_17,
        GPIO72_LCD_FCLK,
        GPIO73_LCD_LCLK,
        GPIO74_LCD_PCLK,
        GPIO75_LCD_BIAS,
};

static mfp_cfg_t littleton_mlcd_pins [] = {
        MFP_CFG_DRV(GPIO62, AF1, DS08X), //GPIO62_LCD_LDD_8
        MFP_CFG_DRV(GPIO63, AF1, DS08X), //GPIO63_LCD_LDD_9
        MFP_CFG_DRV(GPIO64, AF1, DS08X), //GPIO64_LCD_LDD_10
        MFP_CFG_DRV(GPIO65, AF1, DS08X), //GPIO65_LCD_LDD_11
        MFP_CFG_DRV(GPIO66, AF1, DS08X), //GPIO66_LCD_LDD_12
        MFP_CFG_DRV(GPIO67, AF1, DS08X), //GPIO67_LCD_LDD_13
        MFP_CFG_DRV(GPIO68, AF1, DS08X), //GPIO68_LCD_LDD_14
        MFP_CFG_DRV(GPIO69, AF1, DS08X), //GPIO69_LCD_LDD_15
        MFP_CFG_DRV(GPIO70, AF1, DS08X), //GPIO70_LCD_LDD_16
        MFP_CFG_DRV(GPIO71, AF1, DS08X), //GPIO71_LCD_LDD_17
        MFP_CFG_DRV(GPIO54, AF1, DS08X), //GPIO54_LCD_LDD_0
        MFP_CFG_DRV(GPIO55, AF1, DS08X), //GPIO55_LCD_LDD_1
        MFP_CFG_DRV(GPIO56, AF1, DS08X), //GPIO56_LCD_LDD_2
        MFP_CFG_DRV(GPIO57, AF1, DS08X), //GPIO57_LCD_LDD_3
        MFP_CFG_DRV(GPIO58, AF1, DS08X), //GPIO58_LCD_LDD_4
        MFP_CFG_DRV(GPIO59, AF1, DS08X), //GPIO59_LCD_LDD_5
        MFP_CFG_DRV(GPIO60, AF1, DS08X), //GPIO60_LCD_LDD_6
        MFP_CFG_DRV(GPIO61, AF1, DS08X), //GPIO61_LCD_LDD_7
        MFP_CFG_DRV(GPIO72, AF1, DS08X), //GPIO72_LCD_FCLK
        MFP_CFG_DRV(GPIO73, AF1, DS08X), //GPIO73_LCD_LCLK
        MFP_CFG_DRV(GPIO74, AF1, DS08X), //GPIO74_LCD_PCLK
        MFP_CFG_DRV(GPIO75, AF1, DS08X), //GPIO75_LCD_BIAS
};

void pxa3xx_enable_lcd_pins(void)
{
        pxa3xx_mfp_config(littleton_lcd_pins, ARRAY_SIZE(littleton_lcd_pins));
}

void pxa3xx_enable_mlcd_pins(void)
{
        pxa3xx_mfp_config(littleton_mlcd_pins, ARRAY_SIZE(littleton_mlcd_pins));
}

extern int get_pm_state(void);
static void littleton_lcd_power(int on, struct fb_var_screeninfo *var)
{
	if(PM_SUSPEND_LCDREFRESH != get_pm_state()){
		if (on) {
			ssp_send_cmd(ARRAY_AND_SIZE(lcd_panel_on));
	
			printk("Configuring Littleton LCD panel...\n");
			lcd_ssp_init();
			ssp_send_cmd(ARRAY_AND_SIZE(lcd_panel_reset));
			mdelay(10);
			if (var->xres > 240) {
				/* VGA */
				ssp_send_cmd(ARRAY_AND_SIZE(lcd_vga_pass_through));
				ssp_send_cmd(ARRAY_AND_SIZE(lcd_panel_config));
				ssp_send_cmd(ARRAY_AND_SIZE(lcd_vga_transfer));
			} else {
				/* QVGA */
				ssp_send_cmd(ARRAY_AND_SIZE(lcd_qvga_pass_through));
				ssp_send_cmd(ARRAY_AND_SIZE(lcd_panel_config));
				ssp_send_cmd(ARRAY_AND_SIZE(lcd_qvga_transfer));
			}
		} else
			ssp_send_cmd(ARRAY_AND_SIZE(lcd_panel_off));
	}
}

#ifdef CONFIG_FB_PXA_LCD_VGA
static struct pxafb_mode_info tpo_tdo24mtea1_modes[] = {
	[0] = {
		/* VGA */
		.pixclock	= 38250,
		.xres		= 480,
		.yres		= 640,
		.bpp		= 16,
		.hsync_len	= 8,
		.left_margin	= 8,
		.right_margin	= 24,
		.vsync_len	= 2,
		.upper_margin	= 2,
		.lower_margin	= 4,
		.sync		= 0,
	},
};
#endif

#ifdef CONFIG_FB_PXA_LCD_QVGA
static struct pxafb_mode_info tpo_tdo24mtea1_modes[] = {
	[0] = {
		/* QVGA */
		.pixclock	= 153000,
		.xres		= 240,
		.yres		= 320,
		.bpp		= 16,
		.hsync_len	= 8,
		.left_margin	= 8,
		.right_margin	= 88,
		.vsync_len	= 2,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.sync		= 0,
	},
};
#endif

static struct pxafb_mach_info littleton_lcd_info = {
	.modes			= tpo_tdo24mtea1_modes,
	.num_modes		= 1,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_HSP | LCCR3_VSP,
	.pxafb_lcd_power	= littleton_lcd_power,
};

static void littleton_init_lcd(void)
{
	set_pxa_fb_info(&littleton_lcd_info);
}
#else
static inline void littleton_init_lcd(void) {};
#endif /* CONFIG_FB_PXA || CONFIG_FB_PXA_MODULE */

#if defined(CONFIG_SPI_PXA2XX) || defined(CONFIG_SPI_PXA2XX_MODULE)
static struct pxa2xx_spi_master littleton_spi_info = {
	.num_chipselect		= 1,
};

static void littleton_tdo24m_cs(u32 cmd)
{
	gpio_set_value(LITTLETON_GPIO_LCD_CS, !(cmd == PXA2XX_CS_ASSERT));
}

static struct pxa2xx_spi_chip littleton_tdo24m_chip = {
	.rx_threshold	= 1,
	.tx_threshold	= 1,
	.cs_control	= littleton_tdo24m_cs,
};

static struct spi_board_info littleton_spi_devices[] __initdata = {
	{
		.modalias	= "tdo24m",
		.max_speed_hz	= 1000000,
		.bus_num	= 2,
		.chip_select	= 0,
		.controller_data= &littleton_tdo24m_chip,
	},
};

static void __init littleton_init_spi(void)
{
	int err;

	err = gpio_request(LITTLETON_GPIO_LCD_CS, "LCD_CS");
	if (err) {
		pr_warning("failed to request GPIO for LCS CS\n");
		return;
	}

	gpio_direction_output(LITTLETON_GPIO_LCD_CS, 1);

	pxa2xx_set_spi_info(2, &littleton_spi_info);
	spi_register_board_info(ARRAY_AND_SIZE(littleton_spi_devices));
}
#else
static inline void littleton_init_spi(void) {}
#endif

#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct pxa3xx_nand_platform_data littleton_nand_info;
static void __init littleton_init_nand(void)
{
	if (is_android()) {
		littleton_nand_info.parts = pxa300_android_128m_partitions;
		littleton_nand_info.nr_parts = ARRAY_SIZE(pxa300_android_128m_partitions);
	} else {
		littleton_nand_info.parts = pxa300_128m_partitions;
		littleton_nand_info.nr_parts = ARRAY_SIZE(pxa300_128m_partitions);
	}

	pxa3xx_device_nand.dev.platform_data = &littleton_nand_info;
	platform_device_register(&pxa3xx_device_nand);
}
#else
static inline void littleton_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
static int micco_init_irq(void)
{
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO18));

	return 0;
}

static int micco_ack_irq(void)
{
	return 0;
}

static void littleton_micco_init(void)
{
	u8 value;

	/* Mask interrupts that are not needed */
	micco_write(MICCO_IRQ_MASK_A, 0xFE);
	micco_write(MICCO_IRQ_MASK_B, 0xFF);
	micco_write(MICCO_IRQ_MASK_C, 0xFF);
	micco_write(MICCO_IRQ_MASK_D, 0xFF);

	/* avoid SRAM power off during sleep*/
	micco_write(0x10, 0x05);
	micco_write(0x11, 0xff);
	micco_write(0x12, 0xff);

	/* Enable the ONKEY power down functionality */
	micco_write(MICCO_SYSCTRL_B, 0x20);
	micco_write(MICCO_SYSCTRL_A, 0x60);

	/* IRQ is masked during the power-up sequence and will not be released
	 * until they have been read for the first time */
	micco_read(MICCO_EVENT_A, &value);
	micco_read(MICCO_EVENT_B, &value);
	micco_read(MICCO_EVENT_C, &value);
	micco_read(MICCO_EVENT_D, &value);
}

/* micco_power_module[] should be consistent with enum
 * in arch/arm/mach-pxa/include/mach/pxa3xx_pmic.h */
static struct power_supply_module miccoB0_power_modules[] = {
	/* {command,		power_module}, */
	{VCC_CORE,		BUCK1},
	{VCC_SRAM,		LDO2},
	{VCC_MVT,		LDO1},
	{VCC_3V_APPS,		LDO3},
	{VCC_SDIO,		LDO14},
	{VCC_CAMERA_ANA,	LDO6},
	{VCC_USB,		LDO3},
	{VCC_LCD,		LDO12},
	{VCC_TSI,		0},
	{VCC_CAMERA_IO,		LDO15},
	{VCC_1P8V,		LDO4},
	{VCC_MEM,		BUCK2},
	{HDMI_TX,		LDO9},
	{TECH_3V,		LDO10},
	{TECH_1P8V,		LDO11},
};

static struct power_chip micco_chips[] = {
	{0x00,	"miccoB0",	miccoB0_power_modules},
	{0x10,	"miccoB0",	miccoB0_power_modules},
	{0x11,	"miccoB0",	miccoB0_power_modules},
	{0,	NULL,		NULL},
};

static struct micco_platform_data micco_data = {
	.init_irq = micco_init_irq,
	.ack_irq = micco_ack_irq,
	.platform_init = littleton_micco_init,
	.power_chips = micco_chips,
};
#endif /* CONFIG_PXA3xx_MICCO || CONFIG_PXA3xx_MICCO_MODULE*/

#if defined(CONFIG_GPIO_MAX7320) || defined(CONFIG_GPIO_MAX7320_MODULE)
static struct max7320_platform_data exp0_pdata = {
	.gpio_base	= EXT0_BASE,
};
#endif /* CONFIG_GPIO_MAX7320 || CONFIG_GPIO_MAX7320_MODULE */

#if defined(CONFIG_GPIO_MAX7321) || defined(CONFIG_GPIO_MAX7321_MODULE)
int max7321_setup(struct i2c_client *client, unsigned gpio,
		unsigned ngpio, void *context)
{
	struct max7321_platform_data *exp_data = context;
	int gpio_irq;

	if (exp_data->poweron)
		exp_data->poweron();

	gpio_irq = exp_data->gpio_irq;
	if (gpio_irq < 0)
		return 0;

	if (gpio_request(gpio_irq, "max7321 IRQ")) {
		printk(KERN_ERR "max7321: failed to init the interrupt pin\n");
		return -EBUSY;
	 } else
		gpio_direction_input(gpio_irq);

	return 0;
}

static void board_8385_wlan_poweron(void)
{
	int gpio_power = 0;
	int gpio_reset = 0;

	gpio_power = EXT1_GPIO(0);
	gpio_reset = EXT1_GPIO(1);

	if(gpio_request(gpio_power, "8385 wlan card power down")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_power);
		return;
	}
	if(gpio_request(gpio_reset, "8385 wlan card reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_reset);
		return;
	}
	gpio_direction_output(gpio_power, 1);
	mdelay(5);
	gpio_direction_output(gpio_reset, 1);
	mdelay(5);
	gpio_direction_output(gpio_reset, 0);
	gpio_free(gpio_power);
	gpio_free(gpio_reset);

	return;
}

static void board_umts_poweron(void)
{
	/* do nothing */
	return;
}

static void board_siemens_baseband_poweron(void)
{
	/* TODO */
	printk("*** This board can't be poweron correctly, fixme\n");
	return;
}

static void board_8688_wlan_camera_poweron(void)
{
	/* TODO */
	printk("*** This board can't be poweron correctly, fixme\n");
	return;
}

static void board_camera_poweron(void)
{
	/* TODO */
	printk("*** This board can't be poweron correctly, fixme\n");
	return;
}

static void board_8386_wlan_camera_poweron(void)
{
	int gpio_power = 0;
	int gpio_reset = 0;

	gpio_power = EXT1_GPIO(0);
	gpio_reset = EXT1_GPIO(1);

	if(gpio_request(gpio_power, "8386 wlan card power down")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_power);
		return;
	}
	if(gpio_request(gpio_reset, "8386 wlan card reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_reset);
		return;
	}
	gpio_direction_output(gpio_power, 0);
	gpio_direction_output(gpio_reset, 0);
	mdelay(5);
	gpio_direction_output(gpio_reset, 1);
	gpio_free(gpio_power);
	gpio_free(gpio_reset);

	return;
}

static void board_8385_wlan_camera_poweron(void)
{
	/* TODO */
	printk("*** This board can't be poweron correctly, fixme\n");
	return;
}

static void board_8688_wlan_poweron(void)
{
	int gpio_power = 0;
	int gpio_reset = 0;
	int gpio_wake = 0;
	int gpio_x_p3 = 0;

	gpio_power = EXT1_GPIO(0);
	gpio_reset = EXT1_GPIO(1);
	gpio_wake = EXT1_GPIO(2);
	gpio_x_p3 = EXT1_GPIO(3);

	if(gpio_request(gpio_power, "8688 wlan card power down")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_power);
		return;
	}
	if(gpio_request(gpio_reset, "8688 wlan card reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_reset);
		return;
	}
	if(gpio_request(gpio_wake, "8688 wlan card gpio_wake")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_wake);
		return;
	}
	if(gpio_request(gpio_x_p3, "8688 wlan card gpio_x_p3")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_x_p3);
		return;
	}
	gpio_direction_output(gpio_power, 0);
	gpio_direction_output(gpio_reset, 0);
	gpio_direction_output(gpio_wake, 0);
	gpio_direction_output(gpio_x_p3, 0);
	mdelay(5);
	gpio_direction_output(gpio_reset, 1);
	gpio_direction_output(gpio_x_p3, 1);
	gpio_free(gpio_power);
	gpio_free(gpio_reset);
	gpio_free(gpio_x_p3);

	return;
}

#define BOARD_8385_WLAN 	0
#define BOARD_UMTS		1
#define BOARD_SIEMENS_BASEBAND	2
#define BOARD_8688_WLAN_CAMERA	3
#define BOARD_CAMERA		4
#define BOARD_8386_WLAN_CAMERA	5
#define BOARD_8385_WLAN_CAMERA	6
#define BOARD_8688_WLAN		7
static struct max7321_platform_data exp1_pdata[] = {
	[BOARD_8385_WLAN] = {
		.gpio_base	= EXT1_BASE,
		.gpio_irq	= -1,
		.detected	= 0,
		.context	= &exp1_pdata[BOARD_8385_WLAN],
		.setup		= max7321_setup,
		.poweron	= board_8385_wlan_poweron,
	},
	[BOARD_UMTS] = {
		.gpio_base	= EXT1_BASE,
		.gpio_irq	= -1,
		.detected	= 0,
		.context	= &exp1_pdata[BOARD_UMTS],
		.setup		= max7321_setup,
		.poweron	= board_umts_poweron,
	},
	[BOARD_SIEMENS_BASEBAND] = {
		.gpio_base	= EXT1_BASE,
		.gpio_irq	= -1,
		.detected	= 0,
		.context	= &exp1_pdata[BOARD_SIEMENS_BASEBAND],
		.setup		= max7321_setup,
		.poweron	= board_siemens_baseband_poweron,
	},
	[BOARD_8688_WLAN_CAMERA] = {
		.gpio_base	= EXT1_BASE,
		.gpio_irq	= -1,
		.detected	= 0,
		.context	= &exp1_pdata[BOARD_8688_WLAN_CAMERA],
		.setup		= max7321_setup,
		.poweron	= board_8688_wlan_camera_poweron,
	},
	[BOARD_CAMERA] = {
		.gpio_base	= EXT1_BASE,
		.gpio_irq	= -1,
		.detected	= 0,
		.context	= &exp1_pdata[BOARD_CAMERA],
		.setup		= max7321_setup,
		.poweron	= board_camera_poweron,
	},
	[BOARD_8386_WLAN_CAMERA] = {
		.gpio_base	= EXT1_BASE,
		.gpio_irq	= -1,
		.detected	= 0,
		.context	= &exp1_pdata[BOARD_8386_WLAN_CAMERA],
		.setup		= max7321_setup,
		.poweron	= board_8386_wlan_camera_poweron,
	},
	[BOARD_8385_WLAN_CAMERA] = {
		.gpio_base	= EXT1_BASE,
		.gpio_irq	= -1,
		.detected	= 0,
		.context	= &exp1_pdata[BOARD_8385_WLAN_CAMERA],
		.setup		= max7321_setup,
		.poweron	= board_8385_wlan_camera_poweron,
	},
	[BOARD_8688_WLAN] = {
		.gpio_base	= EXT1_BASE,
		.gpio_irq	= mfp_to_gpio(MFP_PIN_GPIO77),
		.detected	= 0,
		.context	= &exp1_pdata[BOARD_8688_WLAN],
		.setup		= max7321_setup,
		.poweron	= board_8688_wlan_poweron,
	},
};

int is_technology_board(int id)
{
	return exp1_pdata[id].detected;
}
#endif /* CONFIG_GPIO_MAX7321 || CONFIG_GPIO_MAX7321_MODULE */

#if defined(CONFIG_PXA_CAMERA)

/* sensor init */
static int sensor_power_on(int flag)
{
	/*
	 * flag, 0, low resolution
	 * flag, 1, high resolution
	 */
	int gpio_hi = 0;
	int gpio_lo = 0;

	/* MAX7320 P5 PWDN */
	gpio_lo = EXT0_GPIO(5);
	if(gpio_request(gpio_lo, "lo sensor power down"))
		printk("MAX7320 GPIO FAILED\n");
	else
		gpio_direction_output(gpio_lo, 1);
	
	if(is_technology_board(BOARD_8688_WLAN_CAMERA) 
			|| is_technology_board(BOARD_8386_WLAN_CAMERA)){
		/* 8688 & 8686 */
		/* MAX7321 P2 PWDN */

		gpio_hi = EXT1_GPIO(2);	
		if(gpio_request(gpio_hi, "hi sensor power down"))
			printk("MAX7321 GPIO FAILED\n");
		else		
			gpio_direction_output(gpio_hi, 1);
	}

	if(flag){
		if(gpio_hi){
			gpio_direction_output(gpio_hi, 0);
		}
	}else{
		if(gpio_lo){
			gpio_direction_output(gpio_lo, 0);
		}
	}
	
	if(gpio_hi)
		gpio_free(gpio_hi);
	
	if(gpio_lo)		
		gpio_free(gpio_lo);

	return 0;
}
static int sensor_power_off(int flag)
{
	/*
	 * flag, 0, low resolution
	 * flag, 1, high resolution
	 */
	int gpio_hi = 0;
	int gpio_lo = 0;

	flag = flag;  		/* power off all */
	
	/* MAX7320 P5 PWDN */
	gpio_lo = EXT0_GPIO(5);
	if(gpio_request(gpio_lo, "lo sensor power down"))
		printk("MAX7320 GPIO FAILED\n");
	
	if(is_technology_board(BOARD_8688_WLAN_CAMERA) 
			|| is_technology_board(BOARD_8386_WLAN_CAMERA)){
		/* 8688 & 8686 */
		/* MAX7321 P2 PWDN */
		gpio_hi = EXT1_GPIO(2);	
		if(gpio_request(gpio_hi, "hi sensor power down"))
			printk("MAX7321 GPIO FAILED\n");
	}
	
	if(gpio_lo){
		gpio_direction_output(gpio_lo, 1);
		gpio_free(gpio_lo);
	}

	if(gpio_hi){
		gpio_direction_output(gpio_hi, 1);
		gpio_free(gpio_hi);
	}
	
	return 0;
}

static struct sensor_platform_data ov7673_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

static struct sensor_platform_data ov5623_sensor_data = {
	.id = SENSOR_HIGH,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

/* camera platform data */
static mfp_cfg_t sync[] = {
	GPIO51_CI_HSYNC,
	GPIO52_CI_VSYNC,
};

static mfp_cfg_t sync_gpio[] = {
	GPIO51_CI_HSYNC_GPIO,
	GPIO52_CI_VSYNC_GPIO,
};

static void cam_sync_to_gpio(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(sync_gpio));
}

static void cam_sync_from_gpio(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(sync));
}

static int cam_init(void)
{
	return 0;
}

static void cam_deinit(void)
{
}

static void cam_suspend(void)
{
	sensor_power_off(SENSOR_LOW);
}

static void cam_resume(void)
{
	sensor_power_off(SENSOR_LOW);
}

static struct cam_platform_data cam_ops = {
	.vsync_gpio		= MFP_PIN_GPIO52,
	.init			= cam_init,
	.deinit			= cam_deinit,
	.suspend		= cam_suspend,
	.resume			= cam_resume,
	.sync_to_gpio		= cam_sync_to_gpio,
	.sync_from_gpio		= cam_sync_from_gpio,
};

static void __init littleton_init_cam(void)
{
	pxa3xx_device_cam.dev.platform_data = &cam_ops;
	platform_device_register(&pxa3xx_device_cam);
}

/* QCI init over */

#endif

static struct i2c_board_info littleton_i2c_board_info[] = {
#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
	{
		.type	= "micco",
		.addr		= 0x34,
		.platform_data	= &micco_data,
		.irq		= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO18)),
	},
#endif
#if defined(CONFIG_GPIO_MAX7320) || defined(CONFIG_GPIO_MAX7320_MODULE)
	{
		.type	= "max7320",
		.addr		= 0x50,
		.platform_data	= &exp0_pdata,
	},
#endif
#if defined(CONFIG_GPIO_MAX7321) || defined(CONFIG_GPIO_MAX7321_MODULE)
	{
		/* on 8385 WLAN Board */
		.type	= "max7321",
		.addr		= 0x60,
		.platform_data	= &exp1_pdata[BOARD_8385_WLAN],
	},
	{
		/* on UMTS Board */
		.type	= "max7321",
		.addr		= 0x61,
		.platform_data	= &exp1_pdata[BOARD_UMTS],
	},
	{
		/* on Siemens Baseband Radio Interposer Board */
		.type	= "max7321",
		.addr		= 0x62,
		.platform_data	= &exp1_pdata[BOARD_SIEMENS_BASEBAND],
	},
	{
		/* on 8688 WLAN/Camera Board */
		.type	= "max7321",
		.addr		= 0x63,
		.platform_data	= &exp1_pdata[BOARD_8688_WLAN_CAMERA],
	},
	{
		/* on Camera Board */
		.type	= "max7321",
		.addr		= 0x64,
		.platform_data	= &exp1_pdata[BOARD_CAMERA],
	},
	{
		/* on 8686 WLAN/Camera Board */
		.type	= "max7321",
		.addr		= 0x66,
		.platform_data	= &exp1_pdata[BOARD_8386_WLAN_CAMERA],
	},
	{
		/* on 8385 WLAN/Camera Board */
		.type	= "max7321",
		.addr		= 0x6c,
		.platform_data	= &exp1_pdata[BOARD_8385_WLAN_CAMERA],
	},
	{
		/* on 8688 WLAN Board */
		.type	= "max7321",
		.addr		= 0x6d,
		.platform_data	= &exp1_pdata[BOARD_8688_WLAN],
		.irq		= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO77)),
	},
#endif
#if defined(CONFIG_PXA_CAMERA)
	{
		.type	= "sensor_ov7673",
		.addr		= 0x21,
		.platform_data	= &ov7673_sensor_data,
	},
	{	
		.type	= "sensor_ov5623",
		.addr		= 0x30,
		.platform_data	= &ov5623_sensor_data,
	},
#endif
};

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct pxaohci_platform_data littleton_ohci_info = {
	.port_mode	= PMM_PERPORT_MODE,
};
static void __init littleton_init_ohci(void)
{
	pxa_set_ohci_info(&littleton_ohci_info);
}
#else
static inline void littleton_init_ohci(void) {}
#endif /* CONFIG_USB_OHCI_HCD || CONFIG_USB_OHCI_HCD_MODULE */

struct pxa3xx_freq_mach_info littleton_freq_mach_info = {
	.flags = PXA3xx_USE_POWER_I2C,
};

#if defined(CONFIG_MMC) || defined(CONFIG_MMC_MODULE)
static int littleton_mci_init(struct device *dev,
			     irq_handler_t littleton_detect_int,
			     void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err, cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(littleton_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = littleton_mmc_slot[pdev->id].gpio_cd;

	/*
	 * setup GPIO for littleton MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, littleton_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	gpio_free(gpio_cd);
err_request_cd:
	return err;
}

static void littleton_mci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(littleton_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = littleton_mmc_slot[pdev->id].gpio_cd;

	free_irq(cd_irq, data);
	gpio_free(gpio_cd);
}

static struct pxamci_platform_data littleton_mci_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.init 		= littleton_mci_init,
	.exit		= littleton_mci_exit,
};

static struct pxamci_platform_data littleton_mci2_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
};

static void __init littleton_init_mmc(void)
{
	pxa_set_mci_info(&littleton_mci_platform_data);
	pxa3xx_set_mci2_info(&littleton_mci2_platform_data);
}
#else
static inline void littleton_init_mmc(void) {}
#endif

#ifdef CONFIG_PM
static int littleton_init_wakeup(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	src->bits.rtc = 1;
	src->bits.ost = 1;
	src->bits.ext0 = 1;
	src->bits.uart2 = 1;
	src->bits.mkey = 1;
	src->bits.dkey = 1;
	return 0;
}

static int littleton_query_wakeup(unsigned int reg, pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	if (reg & PXA3xx_PM_WE_RTC)
		src->bits.rtc = 1;
	if (reg & PXA3xx_PM_WE_OST)
		src->bits.ost = 1;
	if (reg & PXA3xx_PM_WE_MSL0)
		src->bits.msl = 1;
	if (reg & PXA3xx_PM_WE_EXTERNAL0)
	        src->bits.ext0 = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(3))
		src->bits.uart1 = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(6))
	        src->bits.mkey = 1;
	if (reg & PXA3xx_PM_WE_KP)
	        src->bits.dkey = 1;
	return 0;
}

static int littleton_ext_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.ext0)
			ret |= PXA3xx_PM_WE_EXTERNAL0;
		if (src.bits.ext1)
			ret |= PXA3xx_PM_WE_EXTERNAL1;
	}
	return ret;
}

static int littleton_key_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO115_KP_MKIN_0), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO116_KP_MKIN_1), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO117_KP_MKIN_2), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO118_KP_MKIN_3), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO119_KP_MKIN_4), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO120_KP_MKIN_5), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(6);
		}
		if (src.bits.dkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO107_KP_DKIN_0), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO108_KP_DKIN_1), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO115_KP_MKIN_0), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO116_KP_MKIN_1), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO117_KP_MKIN_2), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO118_KP_MKIN_3), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO119_KP_MKIN_4), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO120_KP_MKIN_5), MFP_EDGE_NONE);
		}
		if (src.bits.dkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO107_KP_DKIN_0), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO108_KP_DKIN_1), MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int littleton_mmc_wakeup(pm_wakeup_src_t src, int enable)
{
	return 0;
}

static int littleton_uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.uart1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO99_UART1_RXD), MFP_EDGE_FALL);
			ret |= PXA3xx_PM_WE_GENERIC(3);
		}
		if (src.bits.uart2) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO112_UART2_RXD), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO114_UART2_CTS), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(4);
		}
		if (src.bits.uart3) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO110_UART3_RXD), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(5);
		}
	} else {
		if (src.bits.uart1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO99_UART1_RXD), MFP_EDGE_NONE);
		}
		if (src.bits.uart2) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO112_UART2_RXD), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO114_UART2_CTS), MFP_EDGE_NONE);
		}
		if (src.bits.uart3) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO110_UART3_RXD), MFP_EDGE_NONE);
		}
	}
	return ret;
}

static struct pxa3xx_peripheral_wakeup_ops wakeup_ops = {
	.init	= littleton_init_wakeup,
	.query	= littleton_query_wakeup,
	.ext    = littleton_ext_wakeup,
	.key    = littleton_key_wakeup,
	.mmc    = littleton_mmc_wakeup,
	.uart   = littleton_uart_wakeup,
};
#endif

static void __init littleton_init(void)
{
	/* initialize MFP configurations */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(littleton_mfp_cfg));

	/* dvfm device */
	set_pxa3xx_freq_info(&littleton_freq_mach_info);

	/* performance monitor unit */
	pxa3xx_set_pmu_info(NULL);

	/*
	 * Note: we depend bootloader set the correct
	 * value to MSC register for SMC91x.
	 */
	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* littleton_init_spi(); */

	/* uart */
/*	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);
*/
	littleton_init_lcd();

	littleton_init_ohci();

	littleton_init_nand();

#if defined(CONFIG_PXA_CAMERA)

	/* initialize camera */
	littleton_init_cam();
#endif
	i2c_register_board_info(0, ARRAY_AND_SIZE(littleton_i2c_board_info));

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
	/* u2d */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(littleton_u2d_cfg));
	pxa_set_u2d_info(&littleton_u2d_info);
#elif defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_USB_PXA27X_UDC_MODULE)
	pxa_set_udc_info(NULL);
#endif

	littleton_init_keypad();

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
	pxa_set_ficp_info(&littleton_ficp_platform_data);
#endif
	/* MMC card detect & write protect for controller 0 */
	littleton_mmc_slot[0].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO15);
	littleton_init_mmc();
#ifdef CONFIG_PM
	pxa3xx_wakeup_register(&wakeup_ops);
#endif
	littleton_init_battery();
}

MACHINE_START(LITTLETON, "Marvell Form Factor Development Platform (aka Littleton)")
	.phys_io	= 0x40000000,
	.boot_params	= 0xa0000100,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq	= pxa3xx_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= littleton_init,
MACHINE_END
