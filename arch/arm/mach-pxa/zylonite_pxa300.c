/*
 * linux/arch/arm/mach-pxa/zylonite_pxa300.c
 *
 * PXA300/PXA310 specific support code for the
 * PXA3xx Development Platform (aka Zylonite)
 *
 * Copyright (C) 2007 Marvell Internation Ltd.
 * 2007-08-21: eric miao <eric.miao@marvell.com>
 *             initial version
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/irq.h>
 
#include <asm/irq.h>

#include <asm/gpio.h>
#include <mach/mfp-pxa300.h>
#include <mach/i2c.h>
#include <mach/zylonite.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa3xx_nand.h>
#include <mach/arava.h>
#include <mach/pxa3xx_u2d.h>
#include <mach/udc.h>
#include <mach/irda.h>
#include <mach/camera.h>
#include <mach/imm.h>
#include <mach/pxa3xx_pm.h>
#include <mach/part_table.h>
#include <mach/pxafb.h>

#include "devices.h"

#include "generic.h"

/* PXA300/PXA310 common configurations */
static mfp_cfg_t common_mfp_cfg[] __initdata = {
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
	GPIO76_LCD_VSYNC,
	GPIO127_LCD_CS_N,
	GPIO20_PWM3_OUT,	/* backlight */

	/* BTUART */
	GPIO111_UART2_RTS,
	GPIO112_UART2_RXD,
	GPIO113_UART2_TXD,
	GPIO114_UART2_CTS,

	/* STUART */
	GPIO109_UART3_TXD,
	GPIO110_UART3_RXD,

	/* AC97 */
	GPIO23_AC97_nACRESET,
	GPIO24_AC97_SYSCLK,
	GPIO29_AC97_BITCLK,
	GPIO25_AC97_SDATA_IN_0,
	GPIO27_AC97_SDATA_OUT,
	GPIO28_AC97_SYNC,

	/* SSP3 */
	GPIO91_SSP3_SCLK,
	GPIO92_SSP3_FRM,
	GPIO93_SSP3_TXD,
	GPIO94_SSP3_RXD,

	/* WM9713 IRQ */
	GPIO26_GPIO,

	/* Keypad */
	GPIO107_KP_DKIN_0,
	GPIO108_KP_DKIN_1,
	GPIO115_KP_MKIN_0,
	GPIO116_KP_MKIN_1,
	GPIO117_KP_MKIN_2,
	GPIO118_KP_MKIN_3,
	GPIO119_KP_MKIN_4,
	GPIO120_KP_MKIN_5,
	GPIO2_2_KP_MKIN_6,
	GPIO3_2_KP_MKIN_7,
	GPIO121_KP_MKOUT_0,
	GPIO122_KP_MKOUT_1,
	GPIO123_KP_MKOUT_2,
	GPIO124_KP_MKOUT_3,
	GPIO125_KP_MKOUT_4,
	GPIO4_2_KP_MKOUT_5,
	GPIO5_2_KP_MKOUT_6,
	GPIO6_2_KP_MKOUT_7,

	/* MMC1 */
	GPIO3_MMC1_DAT0,
	GPIO4_MMC1_DAT1,
	GPIO5_MMC1_DAT2,
	GPIO6_MMC1_DAT3,
	GPIO7_MMC1_CLK,
	GPIO8_MMC1_CMD,	/* CMD0 for slot 0 */
	GPIO15_GPIO,	/* CMD1 default as GPIO for slot 0 */

	/* MMC2 */
	GPIO9_MMC2_DAT0,
	GPIO10_MMC2_DAT1,
	GPIO11_MMC2_DAT2,
	GPIO12_MMC2_DAT3,
	GPIO13_MMC2_CLK,
	GPIO14_MMC2_CMD,

	/* USB Host */
	GPIO0_2_USBH_PEN,
	GPIO1_2_USBH_PWR,

	/* Standard I2C */
	GPIO21_I2C_SCL,
	GPIO22_I2C_SDA,

	/* USB host*/
	GPIO0_2_USBH_PEN,
	GPIO1_2_USBH_PWR,

	/* SSP3 */
	GPIO91_SSP3_SCLK,
	GPIO92_SSP3_FRM,
	GPIO93_SSP3_TXD,
	GPIO94_SSP3_RXD,

	/* SSP4 */
	GPIO95_SSP4_SCLK,
	GPIO96_SSP4_FRM,
	GPIO97_SSP4_TXD,
	GPIO98_SSP4_RXD,

	/* IRDA */
	GPIO16_GPIO,
};

static mfp_cfg_t pxa300_mfp_cfg[] __initdata = {
	/* FFUART */
	GPIO30_UART1_RXD,
	GPIO31_UART1_TXD,
	GPIO32_UART1_CTS,
	GPIO37_UART1_RTS,
	GPIO33_UART1_DCD,
	GPIO34_UART1_DSR,
	GPIO35_UART1_RI,
	GPIO36_UART1_DTR,

	/* Ethernet */
	GPIO2_nCS3,
	GPIO99_GPIO,
};

static mfp_cfg_t pxa310_mfp_cfg[] __initdata = {
	/* FFUART */
	GPIO99_UART1_RXD,
	GPIO100_UART1_TXD,
	GPIO101_UART1_CTS,
	GPIO106_UART1_RTS,

	/* Ethernet */
	GPIO2_nCS3,
	GPIO102_GPIO,

#ifdef CONFIG_CPU_PXA310
	/* MMC3 */
	GPIO7_2_MMC3_DAT0,
	GPIO8_2_MMC3_DAT1,
	GPIO9_2_MMC3_DAT2,
	GPIO10_2_MMC3_DAT3,
	GPIO103_MMC3_CLK,
	GPIO105_MMC3_CMD,
#endif

	/* ULPI*/
	GPIO38_ULPI_CLK,
	GPIO30_ULPI_DATA_OUT_0,
	GPIO31_ULPI_DATA_OUT_1,
	GPIO32_ULPI_DATA_OUT_2,
	GPIO33_ULPI_DATA_OUT_3,
	GPIO34_ULPI_DATA_OUT_4,
	GPIO35_ULPI_DATA_OUT_5,
	GPIO36_ULPI_DATA_OUT_6,
	GPIO37_ULPI_DATA_OUT_7,
	ULPI_DIR,
	ULPI_NXT,
	ULPI_STP,
};

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
static void zylonite_pxa300_irda_transceiver_mode(struct device *dev, int mode)
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

		err = gpio_request(gpio_ir_shdn, "IRDA CIR");
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
	if (mode & IR_SIRMODE)
		gpio_set_value(gpio_ir_shdn, 0);
	else if (mode & IR_OFF)
		gpio_set_value(gpio_ir_shdn, 1);
	local_irq_restore(flags);
}

static struct pxaficp_platform_data zylonite_pxa300_ficp_platform_data = {
	.transceiver_cap  = IR_SIRMODE | IR_OFF,
	.transceiver_mode = zylonite_pxa300_irda_transceiver_mode,
	.gpio_cir = mfp_to_gpio(MFP_PIN_GPIO16),
	.gpio_ir_shdn = EXT0_GPIO(9),
	.uart_irq = IRQ_STUART,
	.uart_reg_base = __PREG(STUART),
};
#endif /* (CONFIG_PXA_IRDA) || (CONFIG_PXA_IRDA_MODULE) */

#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_PXA27X_UDC_MODULE)
static mfp_cfg_t pxa300_otg_init_pins[] = {
	GPIO106_GPIO,
};

static mfp_cfg_t pxa300_otg_pins[] = {
	GPIO106_USB_P2_7,
};

int zylonite_pxa300_udc_is_miniA(void)
{
	int otg_id = mfp_to_gpio(MFP_PIN_GPIO106);
	int id_value;
	int err;

	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_otg_init_pins));
	err = gpio_request(otg_id, "OTG ID");
	if (err) {
		gpio_free(otg_id);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d return :%d\n", otg_id, err);
		return 0;
	}
	gpio_direction_input(otg_id);
	id_value = gpio_get_value(otg_id);
	gpio_free(otg_id);
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_otg_pins));

	return (id_value == 0);
}

static struct pxa2xx_udc_mach_info zylonite_pxa300_udc_info = {
	.udc_is_miniA = zylonite_pxa300_udc_is_miniA,
};
#endif

static struct resource pxa3xx_resource_pwm_bl[] = {
	[0] = {
		.start	= 0x40C00010,/* PWM3 */
		.end	= 0x40C0001b,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device pxa3xx_device_pwm_bl = {
	.name 		= "pxa3xx_pwm_bl",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_pwm_bl),
	.resource	= pxa3xx_resource_pwm_bl,
};

static struct resource pxa3xx_resource_gpio_bl[] = {
	[0] = {
		.start	= mfp_to_gpio(MFP_PIN_GPIO20),
		.end	= mfp_to_gpio(MFP_PIN_GPIO20),
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device pxa3xx_device_gpio_bl = {
	.name 		= "pxa3xx_gpio_bl",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_gpio_bl),
	.resource	= pxa3xx_resource_gpio_bl,
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

#if defined(CONFIG_PXA_CAMERA)

#define QCI_HI_PWDN_PIN		EXT0_GPIO(10)
#define QCI_LO_PWDN_PIN		EXT0_GPIO(11)

/* sensor init */
static int sensor_power_on(int flag)
{
	/*
	 * flag, 0, low resolution
	 * flag, 1, high resolution
	 */

	if (gpio_request(QCI_HI_PWDN_PIN, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", QCI_HI_PWDN_PIN);
		return -EIO;
	}

	if (gpio_request(QCI_LO_PWDN_PIN, "CAM_EANBLE_LO_SENSOR")){
		gpio_free(QCI_HI_PWDN_PIN);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", QCI_LO_PWDN_PIN);
		return -EIO;
	}
	
	gpio_direction_output(QCI_HI_PWDN_PIN, 1);
	gpio_direction_output(QCI_LO_PWDN_PIN, 1);
	
	if(flag){
		gpio_direction_output(QCI_HI_PWDN_PIN, 0);
		gpio_direction_output(QCI_LO_PWDN_PIN, 1);
	}else{
		gpio_direction_output(QCI_LO_PWDN_PIN, 0);
		gpio_direction_output(QCI_HI_PWDN_PIN, 1);
	}

	gpio_free(QCI_HI_PWDN_PIN);
	gpio_free(QCI_LO_PWDN_PIN);

	return 0;
}

static int sensor_power_off(int flag)
{
	/*
	 * flag, 0, low resolution
	 * flag, 1, high resolution
	 */
	flag = flag; /* power off all */
	
	if (gpio_request(QCI_HI_PWDN_PIN, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", QCI_HI_PWDN_PIN);
		return -EIO;
	}

	if (gpio_request(QCI_LO_PWDN_PIN, "CAM_EANBLE_LO_SENSOR")){
		gpio_free(QCI_HI_PWDN_PIN);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", QCI_LO_PWDN_PIN);
		return -EIO;
	}
	
	gpio_direction_output(QCI_HI_PWDN_PIN, 1);
	gpio_direction_output(QCI_LO_PWDN_PIN, 1);

	gpio_free(QCI_HI_PWDN_PIN);
	gpio_free(QCI_LO_PWDN_PIN);
	
	return 0;
}

static struct sensor_platform_data ov7660_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

static struct sensor_platform_data ov2630_sensor_data = {
	.id = SENSOR_HIGH,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

/* QCI init */

static mfp_cfg_t pxa300_cam_pins[] = {
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
};

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
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_cam_pins));

	/* verified in pxa300, to prevent confilit with usb */
	if (cpu_is_pxa300()) {
		if (gpio_request(UTMI_SWITCH_PIN, "CAM init")) {
			printk(KERN_ERR "Request GPIO failed,"
			       "gpio: %d \n", UTMI_SWITCH_PIN);

			return -1;
		}

		if (gpio_request(UTMI_TESTEN_PIN, "CAM init")){
			gpio_free(UTMI_SWITCH_PIN);
			printk(KERN_ERR "Request GPIO failed,"
			       "gpio: %d\n", UTMI_TESTEN_PIN);

			return -1;
		}
		
		gpio_direction_output(UTMI_TESTEN_PIN, 1);
		gpio_direction_output(UTMI_SWITCH_PIN, 0);
	}
	
	return 0;
}

static void cam_deinit(void)
{
 	if (cpu_is_pxa300()) {
		gpio_free(UTMI_TESTEN_PIN);
		gpio_free(UTMI_SWITCH_PIN);
	}
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

static void __init zylonite_init_cam(void)
{
	pxa3xx_device_cam.dev.platform_data = &cam_ops;
	platform_device_register(&pxa3xx_device_cam);
}

/* QCI init over */
#endif

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
static mfp_cfg_t pxa300_utmi_pins[] = {
	GPIO100_U2D_RESET,
	GPIO38_UTM_CLK,
	GPIO39_UTM_PHYDATA_0,
	GPIO40_UTM_PHYDATA_1,
	GPIO41_UTM_PHYDATA_2,
	GPIO42_UTM_PHYDATA_3,
	GPIO43_UTM_PHYDATA_4,
	GPIO44_UTM_PHYDATA_5,
	GPIO45_UTM_PHYDATA_6,
	GPIO46_UTM_PHYDATA_7,
	GPIO101_U2D_XCVR_SEL,
	GPIO102_U2D_TERM_SEL,
	GPIO103_U2D_SUSPEND,
	GPIO104_UTM_LINESTATE_0,
	GPIO105_UTM_LINESTATE_1,
	GPIO52_U2D_TXVALID,
	GPIO53_UTM_TXREADY,
	GPIO48_UTM_RXVALID,
	GPIO47_UTM_RXACTIVE,
	GPIO50_U2D_RXERROR,
	GPIO51_U2D_OPMODE_0,
	GPIO106_U2D_OPMODE_1,
};

static void zylonite_pxa300_reset_xcvr(void)
{
	int reset_pin;
	int err;

#ifdef CONFIG_CPU_PXA310
	if (cpu_is_pxa310())
		reset_pin = ULPI_RESET_PIN;
	else
#endif
		reset_pin = mfp_to_gpio(PXA300_U2D_RESET);

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

#ifdef CONFIG_CPU_PXA310
	if (cpu_is_pxa310())
		gpio_direction_input(reset_pin);
#endif

	gpio_free(reset_pin);
}

static void zylonite_pxa300_reset_utmi(void)
{
	int err;
	if (cpu_is_pxa300()) {
		err = gpio_request(UTMI_SWITCH_PIN, "UTMI switch");
		if (err) {
			gpio_free(UTMI_SWITCH_PIN);
			printk(KERN_ERR "Request GPIO failed,"
			       "gpio: %d return :%d\n", UTMI_SWITCH_PIN, err);
			return;
		}

		err = gpio_request(UTMI_TESTEN_PIN, "UTMI testen");
		if (err) {
			gpio_free(UTMI_TESTEN_PIN);
			printk(KERN_ERR "Request GPIO failed,"
			       "gpio: %d return :%d\n", UTMI_TESTEN_PIN, err);
			return;
		}
		gpio_direction_output(UTMI_SWITCH_PIN, 1);
		gpio_direction_output(UTMI_TESTEN_PIN, 0);

		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_utmi_pins));
		gpio_free(UTMI_SWITCH_PIN);
		gpio_free(UTMI_TESTEN_PIN);
	}
}

static mfp_cfg_t pxa300_softdis_enable[] = {
	GPIO102_GPIO,
 };
 
static mfp_cfg_t pxa300_softdis_disable[] = {
	GPIO102_U2D_TERM_SEL,
};

#define U2D_TERM_SEL	MFP_PIN_GPIO102
static void zylonite_pxa300_soft_dis(int enable)
{
	int u2d_ts = U2D_TERM_SEL;
	int err;

	if (enable) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_softdis_enable));
		err = gpio_request(u2d_ts, "U2D Termsel");
		if (err) {
			gpio_free(u2d_ts);
			printk(KERN_ERR "Request GPIO failed,"
			       "return :%d\n", err);
		}

		gpio_direction_output(u2d_ts, 0);
	} else {
		gpio_free(u2d_ts);
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_softdis_disable));
	}
}

static mfp_cfg_t pxa310_ulpidat3_enable[] = {
	GPIO33_GPIO,
};

static mfp_cfg_t pxa310_ulpidat3_disable[] = {
	GPIO33_ULPI_DATA_OUT_3,
};

static void zylonite_pxa310_ulpi_dat3(int enable)
{
	if (enable)
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa310_ulpidat3_enable));
	else
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa310_ulpidat3_disable));
}

static struct pxa3xx_u2d_mach_info zylonite_pxa300_u2d_info = {
	.reset_xcvr = zylonite_pxa300_reset_xcvr,
	.reset_utmi = zylonite_pxa300_reset_utmi,
	.soft_dis = zylonite_pxa300_soft_dis,
	.ulpi_dat3 = zylonite_pxa310_ulpi_dat3,
};
#endif /* CONFIG_USB_PXA3XX_U2D || CONFIG_USB_PXA3XX_U2D_MODULE */

#define NUM_LCD_DETECT_PINS	7

static int lcd_detect_pins[] __initdata = {
	MFP_PIN_GPIO71,	/* LCD_LDD_17 - ORIENT */
	MFP_PIN_GPIO70, /* LCD_LDD_16 - LCDID[5] */
	MFP_PIN_GPIO75, /* LCD_BIAS   - LCDID[4] */
	MFP_PIN_GPIO73, /* LCD_LCLK   - LCDID[3] */
	MFP_PIN_GPIO72, /* LCD_FCLK   - LCDID[2] */
	MFP_PIN_GPIO127,/* LCD_CS_N   - LCDID[1] */
	MFP_PIN_GPIO76, /* LCD_VSYNC  - LCDID[0] */
};

extern void (*zylonite_lcd_power)(int, struct fb_var_screeninfo *);
static mfp_cfg_t sharp_ls037_1p2_mode[] = {
	/* mode select pins for rev1.2 lead free panel */
	GPIO71_GPIO,		/* L_DD_17 */
	GPIO76_GPIO,		/* L_VSYNC */
};

static mfp_cfg_t sharp_ls037_1p1_mode[] = {
	/* mode select pins for rev1.1 lead panel */
	GPIO71_GPIO,		/* L_DD_17 */
	GPIO76_GPIO,		/* L_VSYNC */
	GPIO75_GPIO,		/* L_BIAS */
};

static void zylonite_pxa300_lcd_power(int on,
		struct fb_var_screeninfo *var)
{
	int gpio_ldd17 = mfp_to_gpio(MFP_PIN_GPIO71);
	int gpio_vsync = mfp_to_gpio(MFP_PIN_GPIO76);
	int gpio_bias = mfp_to_gpio(MFP_PIN_GPIO75);

	if (on) {
		if (lcd_id & 0x21) {
			/* OLED/VGA/QVGA Rev1.2 Lead Free panel */
			pxa3xx_mfp_config(ARRAY_AND_SIZE(sharp_ls037_1p2_mode));
			if (gpio_request(gpio_ldd17, "sharp ls037 lcd mode")) {
				return;
			}
			if (gpio_request(gpio_vsync, "sharp ls037 lcd mode")) {
				gpio_free(gpio_ldd17);
				return;
			}

			gpio_direction_output(gpio_ldd17, 0);
			if (var->xres > 240)	/* VGA */
				gpio_direction_output(gpio_vsync, 0);
			else			/* QVGA */
				gpio_direction_output(gpio_vsync, 1);

			gpio_free(gpio_ldd17);
			gpio_free(gpio_vsync);
		} else if (lcd_id & 0x20) { 
			/* OLED/VGA/QVGA Rev1.1 Lead panel */
			pxa3xx_mfp_config(ARRAY_AND_SIZE(sharp_ls037_1p1_mode));
			if (gpio_request(gpio_ldd17, "sharp ls037 lcd mode")) {
				return;
			}

			if (gpio_request(gpio_vsync, "sharp ls037 lcd mode")) {
				gpio_free(gpio_ldd17);
				return;
			}

			if (gpio_request(gpio_bias, "sharp ls037 lcd mode")) {
				gpio_free(gpio_ldd17);
				gpio_free(gpio_vsync);
				return;
			}

			gpio_direction_output(gpio_bias, 0);
			gpio_direction_output(gpio_ldd17, 0);
			if (var->xres > 240)	/* VGA */
				gpio_direction_output(gpio_vsync, 1);
			else			/* QVGA */
				gpio_direction_output(gpio_vsync, 0);

			gpio_free(gpio_ldd17);
			gpio_free(gpio_vsync);
			gpio_free(gpio_bias);
		}
	}
}

static void __init zylonite_detect_lcd_panel(void)
{
	unsigned long mfpr_save[NUM_LCD_DETECT_PINS];
	int i, gpio, id = 0;

	/* save the original MFP settings of these pins and configure
	 * them as GPIO Input, DS01X, Pull Neither, Edge Clear
	 */
	for (i = 0; i < NUM_LCD_DETECT_PINS; i++) {
		mfpr_save[i] = pxa3xx_mfp_read(lcd_detect_pins[i]);
		pxa3xx_mfp_write(lcd_detect_pins[i], 0x8440);
	}

	for (i = 0; i < NUM_LCD_DETECT_PINS; i++) {
		id = id << 1;
		gpio = mfp_to_gpio(lcd_detect_pins[i]);
		gpio_request(gpio, "lcd detect");
		gpio_direction_input(gpio);

		if (gpio_get_value(gpio))
			id = id | 0x1;
		gpio_free(gpio);
	}

	/* lcd id, flush out bit 1 */
	lcd_id = id & 0x3d;

	/* lcd orientation, portrait or landscape */
	lcd_orientation = (id >> 6) & 0x1;

	/* restore the original MFP settings */
	for (i = 0; i < NUM_LCD_DETECT_PINS; i++)
		pxa3xx_mfp_write(lcd_detect_pins[i], mfpr_save[i]);

	zylonite_lcd_power = zylonite_pxa300_lcd_power;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
#if defined(CONFIG_PXA3xx_ARAVA) || defined(CONFIG_PXA3xx_ARAVA_MODULE)
#define PECR_E1IS	(1 << 31)
#define PECR_E1IE	(1 << 30)
#define PECR_E0IS	(1 << 29)
#define PECR_E0IE	(1 << 28)
#define PECR_DIR1	(1 << 5)
#define PECR_DIR0	(1 << 4)

static int arava_init_irq(void)
{
	PECR |= PECR_E0IE;
	PECR &= ~PECR_DIR0;

	return 0;
}

static int arava_ack_irq(void)
{
	PECR |= PECR_E0IS;

	return 0;
}

static struct arava_platform_data arava_data = {
	.init_irq = arava_init_irq,
	.ack_irq = arava_ack_irq,
};
#endif /* CONFIG_PXA3xx_ARAVA || CONFIG_PXA3xx_ARAVA_MODULE*/
 
static struct pca953x_platform_data gpio_exp[] = {
	[0] = {
		.gpio_base	= EXT0_BASE,
	},
	[1] = {
		.gpio_base	= EXT1_BASE,
	},
};

static struct i2c_board_info zylonite_i2c_board_info[] = {
	{
		.type		= "pca9539",
		.addr		= 0x74,
		.platform_data	= &gpio_exp[0],
		.irq		= IRQ_GPIO(18),
	}, {
		.type		= "pca9539",
		.addr		= 0x75,
		.platform_data	= &gpio_exp[1],
		.irq		= IRQ_GPIO(19),
	},
#if defined(CONFIG_PXA3xx_ARAVA) || defined(CONFIG_PXA3xx_ARAVA_MODULE)
	{
		.type		= "arava",
		.addr		= 0x49,
		.platform_data	= &arava_data,
		.irq		= IRQ_WAKEUP0,
	},
#endif
#if defined(CONFIG_PXA_CAMERA)
	{
		.type		= "sensor_ov7660",
		.addr		= 0x21,
		.platform_data	= &ov7660_sensor_data,
	},
	{	
		.type		= "sensor_ov2630",
		.addr		= 0x30,
		.platform_data	= &ov2630_sensor_data,
	},
#endif
};

static void __init zylonite_init_i2c(void)
{
	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(zylonite_i2c_board_info));
}
#else
static inline void zylonite_init_i2c(void) {}
#endif

#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct pxa3xx_nand_platform_data zylonite_nand_info;
static void __init zylonite_init_nand(void)
{
       zylonite_nand_info.parts = pxa300_128m_partitions;
       zylonite_nand_info.nr_parts = ARRAY_SIZE(pxa300_128m_partitions);

       pxa3xx_device_nand.dev.platform_data = &zylonite_nand_info;
       platform_device_register(&pxa3xx_device_nand);
}
#else
static inline void zylonite_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

#ifdef CONFIG_PM
static int zylonite_init_wakeup(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	src->bits.rtc = 1;
	src->bits.ost = 1;
	src->bits.ext0 = 1;
	src->bits.uart1 = 1;
	src->bits.mkey = 1;
	src->bits.dkey = 1;
}

static int zylonite_query_wakeup(unsigned int reg, pm_wakeup_src_t *src)
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
}

static int zylonite_ext_wakeup(pm_wakeup_src_t src, int enable)
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

static int zylonite_key_wakeup(pm_wakeup_src_t src, int enable)
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
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO2_2_KP_MKIN_6), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO3_2_KP_MKIN_7), MFP_EDGE_BOTH);
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
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO2_2_KP_MKIN_6), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO3_2_KP_MKIN_7), MFP_EDGE_NONE);
		}
		if (src.bits.dkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO107_KP_DKIN_0), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO108_KP_DKIN_1), MFP_EDGE_NONE);
		}
	}
	return ret;
}

struct pxa3xx_sleep_pin {
	unsigned int    gpioex0;
	unsigned int    gpioex1;
	unsigned int    gpioex0_saved;
	unsigned int    gpioex1_saved;
};

static struct pxa3xx_sleep_pin pins = {
	.gpioex0_saved = 0,
	.gpioex1_saved = 0,
};

static int zylonite_mmc_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mmc1_cd || src.bits.mmc2_cd) {
			if (!pins.gpioex0_saved) {
				pins.gpioex0 = pxa3xx_mfp_read(MFP_CFG_PIN(GPIO18_GPIO));
				pins.gpioex0_saved = 1;
			}
			pxa3xx_mfp_set_afds(MFP_CFG_PIN(GPIO18_GPIO), MFP_AF2, 0);
			pxa3xx_mfp_set_lpm(MFP_CFG_PIN(GPIO18_GPIO), MFP_PULL_HIGH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO18_GPIO), MFP_EDGE_FALL);
			ret |= PXA3xx_PM_WE_GENERIC(1);
		}
#ifdef CONFIG_CPU_PXA310
		if (cpu_is_pxa310()) {
			if (src.bits.mmc3_cd) {
				if (!pins.gpioex1_saved) {
					pins.gpioex1 = pxa3xx_mfp_read(MFP_CFG_PIN(GPIO19_GPIO));
					pins.gpioex1_saved = 1;
				}
				pxa3xx_mfp_set_afds(MFP_CFG_PIN(GPIO19_GPIO), MFP_AF2, 0);
				pxa3xx_mfp_set_lpm(MFP_CFG_PIN(GPIO19_GPIO), MFP_PULL_HIGH);
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO19_GPIO), MFP_EDGE_FALL);
				ret |= PXA3xx_PM_WE_GENERIC(2);
			}
		}
#endif
		if (src.bits.mmc1_dat1) {
			pxa3xx_mfp_set_lpm(MFP_CFG_PIN(GPIO4_MMC1_DAT1), MFP_LPM_PULL_HIGH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO4_MMC1_DAT1), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(10);
		}
		if (src.bits.mmc2_dat1) {
			pxa3xx_mfp_set_lpm(MFP_CFG_PIN(GPIO10_MMC2_DAT1), MFP_LPM_PULL_HIGH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO10_MMC2_DAT1), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(11);
		}
#ifdef CONFIG_CPU_PXA310
		if (cpu_is_pxa310()) {
			if (src.bits.mmc3_dat1) {
				pxa3xx_mfp_set_lpm(MFP_CFG_PIN(GPIO8_2_MMC3_DAT1), MFP_LPM_PULL_HIGH);
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO8_2_MMC3_DAT1), MFP_EDGE_BOTH);
				ret |= PXA3xx_PM_WE_GENERIC(12);
			}
		}
#endif
	} else {
		if (src.bits.mmc1_cd || src.bits.mmc2_cd) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO18_GPIO), MFP_EDGE_NONE);
			if (pins.gpioex0_saved) {
				pxa3xx_mfp_write(MFP_CFG_PIN(GPIO18_GPIO), pins.gpioex0);
				pins.gpioex0_saved = 0;
			}
		}
#ifdef CONFIG_CPU_PXA310
		if (cpu_is_pxa310()) {
			if (src.bits.mmc3_cd) {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO19_GPIO), MFP_EDGE_NONE);
				if (pins.gpioex1_saved) {
					pxa3xx_mfp_write(MFP_CFG_PIN(GPIO19_GPIO), pins.gpioex1);
					pins.gpioex1_saved = 0;
				}
			}
		}
#endif
		if (src.bits.mmc1_dat1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO4_MMC1_DAT1), MFP_EDGE_NONE);
		}
		if (src.bits.mmc2_dat1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO10_MMC2_DAT1), MFP_EDGE_NONE);
		}
#ifdef CONFIG_CPU_PXA310
		if (cpu_is_pxa310()) {
			if (src.bits.mmc3_dat1) {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO8_2_MMC3_DAT1), MFP_EDGE_NONE);
			}
		}
#endif
	}
	return ret;
}

static int zylonite_uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.uart1) {
			if (cpu_is_pxa300())
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO30_UART1_RXD), MFP_EDGE_FALL);
#ifdef CONFIG_CPU_PXA310
			if (cpu_is_pxa310())
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO99_UART1_RXD), MFP_EDGE_FALL);
#endif
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
			if (cpu_is_pxa300())
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO30_UART1_RXD), MFP_EDGE_NONE);
#ifdef CONFIG_CPU_PXA310
			if (cpu_is_pxa310())
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO99_UART1_RXD), MFP_EDGE_NONE);
#endif
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
	.init	= zylonite_init_wakeup,
	.query	= zylonite_query_wakeup,
	.ext    = zylonite_ext_wakeup,
	.key    = zylonite_key_wakeup,
	.mmc    = zylonite_mmc_wakeup,
	.uart   = zylonite_uart_wakeup,
};
#endif
 
void __init zylonite_pxa300_init(void)
{
	if (cpu_is_pxa300() || cpu_is_pxa310()) {
		/* initialize MFP */
		pxa3xx_mfp_config(ARRAY_AND_SIZE(common_mfp_cfg));

		/* detect LCD panel */
		zylonite_detect_lcd_panel();

		/* GPIO pin assignment */
		gpio_touch_irq = mfp_to_gpio(MFP_PIN_GPIO26);

		/* initialize NAND */
		zylonite_init_nand();

#if defined(CONFIG_PXA_CAMERA)

		/* initialize camera */
		zylonite_init_cam();
#endif
		/* register I2C devices */
		i2c_register_board_info(0,
			ARRAY_AND_SIZE(zylonite_i2c_board_info));

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
		pxa_set_u2d_info(&zylonite_pxa300_u2d_info);
#endif
#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
		pxa_set_ficp_info(&zylonite_pxa300_ficp_platform_data);
#endif

		/* MMC card detect & write protect for controller 0 */
		zylonite_mmc_slot[0].gpio_cd  = EXT_GPIO(0);
		zylonite_mmc_slot[0].gpio_wp  = EXT_GPIO(2);

		/* WM9713 IRQ */
		wm9713_irq = mfp_to_gpio(MFP_PIN_GPIO26);

		zylonite_init_i2c();

		platform_device_register(&pxa3xx_device_imm);
#ifdef CONFIG_PM
		pxa3xx_wakeup_register(&wakeup_ops);
#endif
	}

	if (cpu_is_pxa300()) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_mfp_cfg));
		gpio_eth_irq = mfp_to_gpio(MFP_PIN_GPIO99);
		platform_device_register(&pxa3xx_device_pwm_bl);
#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_USB_PXA27X_UDC_MODULE)
		pxa_set_udc_info(&zylonite_pxa300_udc_info);
#endif
	}

#ifdef CONFIG_CPU_PXA310
	if (cpu_is_pxa310()) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa310_mfp_cfg));
		gpio_eth_irq = mfp_to_gpio(MFP_PIN_GPIO102);
		platform_device_register(&pxa3xx_device_gpio_bl);
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO20), 1);

		/* MMC card detect & write protect for controller 2 */
		zylonite_mmc_slot[2].gpio_cd = EXT_GPIO(30);
		zylonite_mmc_slot[2].gpio_wp = EXT_GPIO(31);
	}
#endif

	/* GPIOs for Debug LEDs */
	gpio_debug_led1 = EXT_GPIO(25);
	gpio_debug_led2 = EXT_GPIO(26);
}
