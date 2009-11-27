#ifndef __ASM_ARCH_LITTLETON_H
#define __ASM_ARCH_LITTLETON_H
#define LITTLETON_GPIO_LCD_CS	(17)

#define LITTLETON_ETH_PHYS	0x30000000

/* on Main Board */
#define EXT0_BASE		NR_BUILTIN_GPIO
#define EXT0_GPIO(x)		(EXT0_BASE + (x))

/* on technology Board */
#define EXT1_BASE		EXT0_BASE + 8
#define EXT1_GPIO(x)		(EXT1_BASE + (x))

#define ULPI_RESET_PIN		(EXT0_GPIO(1))
struct platform_mmc_slot {
	int gpio_cd;
	int gpio_wp;
};

extern struct platform_mmc_slot littleton_mmc_slot[];

#endif /* __ASM_ARCH_LITTLETON_H */
