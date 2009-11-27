/*
 * linux/arch/arm/mach-pxa/zylonite_pxa320.c
 *
 * PXA320 specific support code for the
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
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/mfp-pxa320.h>
#include <mach/zylonite.h>
#include <mach/pxa3xx_nand.h>
#include <mach/arava.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa3xx_u2d.h>
#include <mach/irda.h>
#include <mach/udc.h>
#include <mach/imm.h>
#include <mach/pxa3xx_pm.h>
#include <mach/part_table.h>
#include <mach/pxafb.h>
#include <mach/camera.h>

#include "devices.h"

#include "generic.h"

static mfp_cfg_t mfp_cfg[] __initdata = {
	/* LCD */
	GPIO6_2_LCD_LDD_0,
	GPIO7_2_LCD_LDD_1,
	GPIO8_2_LCD_LDD_2,
	GPIO9_2_LCD_LDD_3,
	GPIO10_2_LCD_LDD_4,
	GPIO11_2_LCD_LDD_5,
	GPIO12_2_LCD_LDD_6,
	GPIO13_2_LCD_LDD_7,
	GPIO63_LCD_LDD_8,
	GPIO64_LCD_LDD_9,
	GPIO65_LCD_LDD_10,
	GPIO66_LCD_LDD_11,
	GPIO67_LCD_LDD_12,
	GPIO68_LCD_LDD_13,
	GPIO69_LCD_LDD_14,
	GPIO70_LCD_LDD_15,
	GPIO71_LCD_LDD_16,
	GPIO72_LCD_LDD_17,
	GPIO73_LCD_CS_N,
	GPIO74_LCD_VSYNC,
	GPIO14_2_LCD_FCLK,
	GPIO15_2_LCD_LCLK,
	GPIO16_2_LCD_PCLK,
	GPIO17_2_LCD_BIAS,
	GPIO14_PWM3_OUT,	/* backlight */

	/* FFUART */
	GPIO41_UART1_RXD,
	GPIO42_UART1_TXD,
	GPIO43_UART1_CTS,
	GPIO44_UART1_DCD,
	GPIO45_UART1_DSR,
	GPIO46_UART1_RI,
	GPIO47_UART1_DTR,
	GPIO48_UART1_RTS,

	/* STUART */
	GPIO107_UART3_TXD,
	GPIO108_UART3_RXD,

	/* AC97 */
	GPIO34_AC97_SYSCLK,
	GPIO35_AC97_SDATA_IN_0,
	GPIO37_AC97_SDATA_OUT,
	GPIO38_AC97_SYNC,
	GPIO39_AC97_BITCLK,
	GPIO40_AC97_nACRESET,

	/* SSP3 */
	GPIO89_SSP3_SCLK,
	GPIO90_SSP3_FRM,
	GPIO91_SSP3_TXD,
	GPIO92_SSP3_RXD,

	/* WM9713 IRQ */
	GPIO15_GPIO,

	/* I2C */
	GPIO32_I2C_SCL,
	GPIO33_I2C_SDA,

	/* Keypad */
	GPIO105_KP_DKIN_0,
	GPIO106_KP_DKIN_1,
	GPIO113_KP_MKIN_0,
	GPIO114_KP_MKIN_1,
	GPIO115_KP_MKIN_2,
	GPIO116_KP_MKIN_3,
	GPIO117_KP_MKIN_4,
	GPIO118_KP_MKIN_5,
	GPIO119_KP_MKIN_6,
	GPIO120_KP_MKIN_7,
	GPIO121_KP_MKOUT_0,
	GPIO122_KP_MKOUT_1,
	GPIO123_KP_MKOUT_2,
	GPIO124_KP_MKOUT_3,
	GPIO125_KP_MKOUT_4,
	GPIO126_KP_MKOUT_5,
	GPIO127_KP_MKOUT_6,
	GPIO5_2_KP_MKOUT_7,

	/* SSP3 */
	GPIO89_SSP3_SCLK,
	GPIO90_SSP3_FRM,
	GPIO91_SSP3_TXD,
	GPIO92_SSP3_RXD,

	/* SSP4 */
	GPIO93_SSP4_SCLK,
	GPIO94_SSP4_FRM,
	GPIO95_SSP4_TXD,
	GPIO96_SSP4_RXD,

	/* Ethernet */
	GPIO4_nCS3,
	GPIO90_GPIO,

	/* MMC1 */
	GPIO18_MMC1_DAT0,
	GPIO19_MMC1_DAT1,
	GPIO20_MMC1_DAT2,
	GPIO21_MMC1_DAT3,
	GPIO22_MMC1_CLK,
	GPIO23_MMC1_CMD,/* CMD0 for slot 0 */
	GPIO31_GPIO,	/* CMD1 default as GPIO for slot 0 */

	/* MMC2 */
	GPIO24_MMC2_DAT0,
	GPIO25_MMC2_DAT1,
	GPIO26_MMC2_DAT2,
	GPIO27_MMC2_DAT3,
	GPIO28_MMC2_CLK,
	GPIO29_MMC2_CMD,

	/* Debug LEDs */
	GPIO1_2_GPIO | MFP_LPM_DRIVE_HIGH,
	GPIO4_2_GPIO | MFP_LPM_DRIVE_HIGH,

	/* USB host*/
	GPIO2_2_USBH_PEN,
	GPIO3_2_USBH_PWR,

	/* IRDA */
	GPIO13_GPIO,
	GPIO97_GPIO,
};

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
static void zylonite_pxa320_irda_transceiver_mode(struct device *dev, int mode)
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
	if (mode & IR_SIRMODE) {
		gpio_set_value(gpio_cir, 0);
		gpio_set_value(gpio_ir_shdn, 0);
	} else if (mode & IR_FIRMODE) {
		/* do not support FIR */
	}
	if (mode & IR_OFF) {
		gpio_set_value(gpio_cir, 0);
		gpio_set_value(gpio_ir_shdn, 1);
	}
	local_irq_restore(flags);
}

static struct pxaficp_platform_data zylonite_pxa320_ficp_platform_data = {
	.transceiver_cap  = IR_SIRMODE | IR_OFF,
	.transceiver_mode = zylonite_pxa320_irda_transceiver_mode,
	.gpio_ir_shdn = mfp_to_gpio(MFP_PIN_GPIO97),
	.gpio_cir = mfp_to_gpio(MFP_PIN_GPIO13),
	.uart_irq = IRQ_STUART,
	.uart_reg_base = __PREG(STUART),
};
#endif /* (CONFIG_PXA_IRDA) || (CONFIG_PXA_IRDA_MODULE) */

#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_PXA27X_UDC_MODULE)
static mfp_cfg_t pxa320_otg_init_pins[] = {
	GPIO104_GPIO,
};

static mfp_cfg_t pxa320_otg_pins[] = {
	GPIO100_USB_P2_4,
	GPIO101_USB_P2_8,
	GPIO104_USB_P2_7,
};

int zylonite_pxa320_udc_is_miniA(void)
{
	int otg_id = mfp_to_gpio(MFP_PIN_GPIO104);
	int id_value;
	int err;

	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa320_otg_init_pins));
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
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa320_otg_pins));

	return (id_value == 0);
}

static struct pxa2xx_udc_mach_info zylonite_pxa320_udc_info = {
	.udc_is_miniA = zylonite_pxa320_udc_is_miniA,
};
#endif

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
static mfp_cfg_t pxa320_utmi_pins[] = {
	GPIO46_GPIO,
	GPIO98_U2D_RESET,
	GPIO10_UTM_CLK,
	GPIO49_U2D_PHYDATA_0,
	GPIO50_U2D_PHYDATA_1,
	GPIO51_U2D_PHYDATA_2,
	GPIO52_U2D_PHYDATA_3,
	GPIO53_U2D_PHYDATA_4,
	GPIO54_U2D_PHYDATA_5,
	GPIO55_U2D_PHYDATA_6,
	GPIO56_U2D_PHYDATA_7,
	GPIO99_U2D_XCVR_SEL,
	GPIO100_U2D_TERM_SEL,
	GPIO101_U2D_SUSPEND,
	GPIO102_UTM_LINESTATE_0,
	GPIO103_UTM_LINESTATE_1,
	GPIO83_U2D_TXVALID,
	GPIO73_UTM_TXREADY,
	GPIO58_UTM_RXVALID,
	GPIO59_UTM_RXACTIVE,
	GPIO60_U2D_RXERROR,
	GPIO61_U2D_OPMODE0,
	GPIO62_U2D_OPMODE1,
 };
 
void zylonite_pxa320_reset_xcvr(void)
{
	int reset_pin;
	int err;

	reset_pin = mfp_to_gpio(PXA320_U2D_RESET);
	err = gpio_request(reset_pin, "U2D Reset");
	if (err) {
		gpio_free(reset_pin);
		printk(KERN_ERR "Request GPIO failed,"
			"return :%d\n", err);
	}
	gpio_direction_output(reset_pin, 0);
	mdelay(100);
	gpio_set_value(reset_pin, 1);
}

void zylonite_pxa320_reset_utmi(void)
{
	if (cpu_is_pxa320())
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa320_utmi_pins));
}

static mfp_cfg_t pxa320_softdis_enable[] = {
	GPIO100_GPIO,
  };
  
static mfp_cfg_t pxa320_softdis_disable[] = {
	GPIO100_U2D_TERM_SEL,
};

#define U2D_TERM_SEL	MFP_PIN_GPIO100
static void zylonite_pxa320_soft_dis(int enable)
{
	int u2d_ts = U2D_TERM_SEL;
	int err;

	if (enable) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa320_softdis_enable));
		err = gpio_request(u2d_ts, "U2D Termsel");
		if (err) {
			gpio_free(u2d_ts);
			printk(KERN_ERR "Request GPIO failed,"
			       "return :%d\n", err);
		}

		gpio_direction_output(u2d_ts, 0);
	} else {
		gpio_free(u2d_ts);
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa320_softdis_disable));
	}
}

static struct pxa3xx_u2d_mach_info zylonite_pxa320_u2d_info = {
	.reset_xcvr = zylonite_pxa320_reset_xcvr,
	.reset_utmi = zylonite_pxa320_reset_utmi,
	.soft_dis = zylonite_pxa320_soft_dis,
};
#endif /* CONFIG_USB_PXA3XX_U2D || CONFIG_USB_PXA3XX_U2D_MODULE */

#define NUM_LCD_DETECT_PINS	7

static int lcd_detect_pins[] __initdata = {
	MFP_PIN_GPIO72,   /* LCD_LDD_17 - ORIENT */
	MFP_PIN_GPIO71,   /* LCD_LDD_16 - LCDID[5] */
	MFP_PIN_GPIO17_2, /* LCD_BIAS   - LCDID[4] */
	MFP_PIN_GPIO15_2, /* LCD_LCLK   - LCDID[3] */
	MFP_PIN_GPIO14_2, /* LCD_FCLK   - LCDID[2] */
	MFP_PIN_GPIO73,   /* LCD_CS_N   - LCDID[1] */
	MFP_PIN_GPIO74,   /* LCD_VSYNC  - LCDID[0] */
	/*
	 * set the MFP_PIN_GPIO 14/15/17 to alternate function other than
	 * GPIO to avoid input level confliction with 14_2, 15_2, 17_2
	 */
	MFP_PIN_GPIO14,
	MFP_PIN_GPIO15,
	MFP_PIN_GPIO17,
};

static int lcd_detect_mfpr[] __initdata = {
	/* AF0, DS 1X, Pull Neither, Edge Clear */
	0x8440, 0x8440, 0x8440, 0x8440, 0x8440, 0x8440, 0x8440,
	0xc442, /* Backlight, Pull-Up, AF2 */
	0x8445, /* AF5 */
	0x8445, /* AF5 */
};

extern void (*zylonite_lcd_power)(int, struct fb_var_screeninfo *);
static mfp_cfg_t sharp_ls037_1p2_mode[] = {
	/* mode select pins for rev1.2 lead free panel */
	GPIO72_GPIO,		/* L_DD_17 */
	GPIO74_GPIO,		/* L_VSYNC */
};

static mfp_cfg_t sharp_ls037_1p1_mode[] = {
	/* mode select pins for rev1.1 lead panel */
	GPIO17_2_GPIO,		/* L_BIAS */
	GPIO72_GPIO,		/* L_DD_17 */
	GPIO74_GPIO,		/* L_VSYNC */
};

static void zylonite_pxa320_lcd_power(int on,
		struct fb_var_screeninfo *var)
{
	int gpio_ldd17 = mfp_to_gpio(MFP_PIN_GPIO72);
	int gpio_vsync = mfp_to_gpio(MFP_PIN_GPIO74);
	int gpio_bias = mfp_to_gpio(MFP_PIN_GPIO17_2);

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
	unsigned long mfpr_save[ARRAY_SIZE(lcd_detect_pins)];
	int i, gpio, id = 0;

	/* save the original MFP settings of these pins and configure them
	 * as GPIO Input, DS01X, Pull Neither, Edge Clear
	 */
	for (i = 0; i < ARRAY_SIZE(lcd_detect_pins); i++) {
		mfpr_save[i] = pxa3xx_mfp_read(lcd_detect_pins[i]);
		pxa3xx_mfp_write(lcd_detect_pins[i], lcd_detect_mfpr[i]);
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
	for (i = 0; i < ARRAY_SIZE(lcd_detect_pins); i++)
		pxa3xx_mfp_write(lcd_detect_pins[i], mfpr_save[i]);

	zylonite_lcd_power = zylonite_pxa320_lcd_power;
}

#if defined(CONFIG_PXA3xx_ARAVA) || defined(CONFIG_PXA3xx_ARAVA_MODULE)
#define PECR_E1IS	(1 << 31)
#define PECR_E1IE	(1 << 30)
#define PECR_E0IS	(1 << 29)
#define PECR_E0IE	(1 << 28)
#define PECR_DIR1	(1 << 5)
#define PECR_DIR0	(1 << 4)

static int arava_init_irq(void)
{
	PECR |= PECR_E1IE;
	PECR &= ~PECR_DIR1;

	return 0;
}

static int arava_ack_irq(void)
{
	PECR |= PECR_E1IS;

	return 0;
}

static struct arava_platform_data arava_data = {
	.init_irq = arava_init_irq,
	.ack_irq = arava_ack_irq,
};
#endif /* CONFIG_PXA3xx_ARAVA || CONFIG_PXA3xx_ARAVA_MODULE */

#if defined(CONFIG_PXA_CAMERA)

#define QCI_HI_PWDN_PIN		MFP_PIN_GPIO102	
#define QCI_LO_PWDN_PIN		MFP_PIN_GPIO103	

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
static mfp_cfg_t pxa320_cam_pins[] = {
	/* QCI */
	GPIO49_CI_DD_0,		
	GPIO50_CI_DD_1,		
	GPIO51_CI_DD_2,		
	GPIO52_CI_DD_3,		
	GPIO53_CI_DD_4,		
	GPIO54_CI_DD_5,		
	GPIO55_CI_DD_6,		
	GPIO56_CI_DD_7,		
	GPIO57_CI_DD_8,		
	GPIO58_CI_DD_9,		
	GPIO59_CI_MCLK,		
	GPIO60_CI_PCLK,		
	GPIO61_CI_HSYNC,		
	GPIO62_CI_VSYNC,		

	GPIO102_CI_HI_PWDN,	
	GPIO103_CI_LO_PWDN,	
};

/* camera platform data */

static int cam_init(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa320_cam_pins));
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
	.vsync_gpio	= -1,
	.init		= cam_init,
	.deinit		= cam_deinit,
	.suspend	= cam_suspend,
	.resume		= cam_resume,
};

static void __init zylonite_init_cam(void)
{
	pxa3xx_device_cam.dev.platform_data = &cam_ops;
	platform_device_register(&pxa3xx_device_cam);
}

/* QCI init over */

#endif

static struct i2c_board_info zylonite_i2c_board_info[] = {
#if defined(CONFIG_PXA3xx_ARAVA) || defined(CONFIG_PXA3xx_ARAVA_MODULE)
	{
		.type		= "arava",
		.addr		= 0x49,
		.platform_data	= &arava_data,
		.irq		= IRQ_WAKEUP1,
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


#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct pxa3xx_nand_platform_data zylonite_nand_info;
static void __init zylonite_init_nand(void)
{
	zylonite_nand_info.parts = pxa320_128m_partitions;
	zylonite_nand_info.nr_parts = ARRAY_SIZE(pxa320_128m_partitions);
	pxa3xx_device_nand.dev.platform_data = &zylonite_nand_info;
	platform_device_register(&pxa3xx_device_nand);
}
#else
static inline void zylonite_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

static struct resource pxa3xx_resource_imm[] = {
	[0] = {
		.name   = "phy_sram",
		.start	= 0x5c000000,
		.end	= 0x5c000000 + 6 * PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name   = "imm_sram",
		.start	= 0x5c000000 + PHYS_SRAM_BANK_SIZE,
		.end	= 0x5c000000 + 6 * PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device pxa3xx_device_imm = {
	.name 		= "pxa3xx-imm",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_imm),
	.resource	= pxa3xx_resource_imm,
};

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
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO113_KP_MKIN_0), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO114_KP_MKIN_1), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO115_KP_MKIN_2), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO116_KP_MKIN_3), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO117_KP_MKIN_4), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO118_KP_MKIN_5), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO119_KP_MKIN_6), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO120_KP_MKIN_7), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(6);
		}
		if (src.bits.dkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO105_KP_DKIN_0), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO106_KP_DKIN_1), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO113_KP_MKIN_0), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO114_KP_MKIN_1), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO115_KP_MKIN_2), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO116_KP_MKIN_3), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO117_KP_MKIN_4), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO118_KP_MKIN_5), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO119_KP_MKIN_6), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO120_KP_MKIN_7), MFP_EDGE_NONE);
		}
		if (src.bits.dkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO105_KP_DKIN_0), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO106_KP_DKIN_1), MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int zylonite_mmc_wakeup(pm_wakeup_src_t src, int enable)
{
	return 0;
}

static int zylonite_uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.uart1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO41_UART1_RXD), MFP_EDGE_FALL);
			ret |= PXA3xx_PM_WE_GENERIC(3);
		}
		if (src.bits.uart2) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO110_UART2_RXD), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO112_UART2_CTS), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(4);
		}
		if (src.bits.uart3) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO108_UART3_RXD), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(5);
		}
	} else {
		if (src.bits.uart1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO41_UART1_RXD), MFP_EDGE_NONE);
		}
		if (src.bits.uart2) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO110_UART2_RXD), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO112_UART2_CTS), MFP_EDGE_NONE);
		}
		if (src.bits.uart3) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO108_UART3_RXD), MFP_EDGE_NONE);
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

void __init zylonite_pxa320_init(void)
{
	if (cpu_is_pxa320()) {
		/* initialize MFP */
		pxa3xx_mfp_config(ARRAY_AND_SIZE(mfp_cfg));

		/* detect LCD panel */
		zylonite_detect_lcd_panel();

		/* GPIO pin assignment */
		gpio_eth_irq	= mfp_to_gpio(MFP_PIN_GPIO9);
		gpio_debug_led1	= mfp_to_gpio(MFP_PIN_GPIO1_2);
		gpio_debug_led2	= mfp_to_gpio(MFP_PIN_GPIO4_2);
		gpio_touch_irq  = mfp_to_gpio(MFP_PIN_GPIO15);

		/* WM9713 IRQ */
		wm9713_irq = mfp_to_gpio(MFP_PIN_GPIO15);

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
		pxa_set_u2d_info(&zylonite_pxa320_u2d_info);
#endif
#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_USB_PXA27X_UDC_MODULE)
		pxa_set_udc_info(&zylonite_pxa320_udc_info);
#endif
#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
		pxa_set_ficp_info(&zylonite_pxa320_ficp_platform_data);
#endif
		/* MMC card detect & write protect for controller 0 */
		zylonite_mmc_slot[0].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO1);
		zylonite_mmc_slot[0].gpio_wp  = mfp_to_gpio(MFP_PIN_GPIO5);

		platform_device_register(&pxa3xx_device_imm);
#ifdef CONFIG_PM
		pxa3xx_wakeup_register(&wakeup_ops);
#endif
	}
}
