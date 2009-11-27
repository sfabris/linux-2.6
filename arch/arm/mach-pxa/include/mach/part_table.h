#ifndef	__ASMARM_ARCH_PART_TABLE_H__
#define	__ASMARM_ARCH_PART_TABLE_H__

/* We predefine sample partition tables here. Platform just pick one. */
static struct mtd_partition pxa320_128m_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x040000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x040000,
		.size        = 0x200000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[2] = {
		.name        = "Filesystem",
		.offset      = 0x0240000,
		.size        = 0x3000000,     /* only mount 48M fs */
	},
	[3] = {
		.name        = "MassStorage",
		.offset      = 0x3240000, 
		.size        = 0x3d60000,
	},
	[4] = {
		.name        = "BBT",
		.offset      = 0x6FA0000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},

	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};

static struct mtd_partition pxa300_128m_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x0100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x0100000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[2] = {
		.name        = "Filesystem",
		.offset      = 0x0400000,
		.size        = 0x3000000,     /* only mount 48M fs */
	},
	[3] = {
		.name        = "MassStorage",
		.offset      = 0x3400000,
		.size        = 0x3BA0000,		/* 61.25M */
	},
	[4] = {
		.name        = "BBT",
		.offset      = 0x6FA0000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};

static struct mtd_partition pxa300_android_128m_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x100000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[2] = {
		.name        = "system",
		.offset      = 0x0400000,
		.size        = 0x4000000,     /* mount 64M fs */
	},
	[3] = {
		.name        = "userdata",
		.offset      = 0x4400000,
		.size        = 0x2000000, 
	},
	[4] = {
		.name        = "MassStorage",
		.offset      = 0x6400000,
		.size        = 0x1900000,	/* 11.625M */
	},
	[5] = {
		.name        = "BBT",
		.offset      = 0x7d00000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};


static struct mtd_partition pxa930_128m_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x00100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x00100000,
		.size        = 0x00200000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[2] = {
		.name        = "Filesystem",
		.offset      = 0x00300000,
		.size        = 0x03000000,     /* only mount 48M fs */
	},
	[3] = {
		.name        = "MassStorage",
		.offset      = 0x03300000,
		.size        = 0x03E60000,		/*  62.375M */
	},
	[4] = {
		.name        = "NVM",
		.offset      = 0x07160000,
		.size        = 0x00020000,		/* 128K */
	},
	[5] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0x7180000, 
		.size        = 0x0800000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[6] = {
		.name        = "MassStorage2",
		.offset      = 0x7980000, 
		.size        = 0x0380000,	/* 3.5M */
	},
	[7] = {
		.name        = "BBT",
		.offset      = 0x7d00000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};

static struct mtd_partition pxa930_256m_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x00100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x0100000,
		.size        = 0x0200000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[2] = {
		.name        = "Filesystem",
		.offset      = 0x0300000,
		.size        = 0x3000000,     /* only mount 48M fs */
	},
	[3] = {
		.name        = "MassStorage",
		.offset      = 0x3300000,
		.size        = 0xBE60000,		/*  190.375M */
	},
	[4] = {
		.name        = "NVM",
		.offset      = 0xF160000,
		.size        = 0x0020000,     /* 128K */
	},
	[5] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0xF180000,
		.size        = 0x0800000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[6] = {
		.name        = "MassStorage2",
		.offset      = 0xF980000,
		.size        = 0x0100000,	/* 1M */
	},
	[7] = {
		.name        = "BBT",
		.offset      = 0xfa80000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};
static struct mtd_partition pxa930_128m_v75_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "NVM",
		.offset      = 0x100000,
		.size        = 0x020000,
	},
	[2] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0x120000, 
		.size        = 0x800000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},	
	[3] = {
		.name        = "Kernel",
		.offset      = 0x920000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[4] = {
		.name        = "Filesystem",
		.offset      = 0x0c20000,
		.size        = 0x3000000,     /* only mount 48M fs */
	},
	[5] = {
		.name        = "MassStorage",
		.offset      = 0x3c20000,
		.size        = 0x40e0000,		/* 64.875M */
	},
	[6] = {
		.name        = "BBT",
		.offset      = 0x7d00000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};
static struct mtd_partition pxa930_256m_v75_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "NVM",
		.offset      = 0x100000,
		.size        = 0x020000,
	},
	[2] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0x120000, 
		.size        = 0x800000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},	
	[3] = {
		.name        = "Kernel",
		.offset      = 0x920000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[4] = {
		.name        = "Filesystem",
		.offset      = 0x0c20000,
		.size        = 0x3000000,     /* only mount 48M fs */
	},
	[5] = {
		.name        = "MassStorage",
		.offset      = 0x3c20000,
		.size        = 0xbe60000,		/* 190.375M  */
	},
	[6] = {
		.name        = "BBT",
		.offset      = 0xfa80000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};
/* This partition table is specific for Android project. Because Android
 * init will mount partition according to the partition name.
 */
static struct mtd_partition android_128m_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x100000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[2] = {
		.name        = "system",
		.offset      = 0x0400000,
		.size        = 0x3000000,     /* mount 48M fs */
	},
	[3] = {
		.name        = "userdata",
		.offset      = 0x3400000,
		.size        = 0x3D60000,     /* mount 61.375M fs */
	},
	[4] = {
		.name        = "NVM",
		.offset      = 0x7160000,
		.size        = 0x0020000,     /* 128K */
	},
	[5] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0x7180000,
		.size        = 0x0800000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[6] = {
		.name        = "MassStorage",
		.offset      = 0x7980000,
		.size        = 0x0380000,	/* 3.5M */
	},
	[7] = {
		.name        = "BBT",
		.offset      = 0x7d00000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};

static struct mtd_partition android_256m_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "Kernel",
		.offset      = 0x100000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[2] = {
		.name        = "system",
		.offset      = 0x0400000,
		.size        = 0x4000000,     /* mount 64M fs */
	},
	[3] = {
		.name        = "userdata",
		.offset      = 0x4400000,
		.size        = 0xAD60000,     /* mount 96M fs */
	},
	[4] = {
		.name        = "NVM",
		.offset      = 0xF160000,
		.size        = 0x0020000,     /* 128K */
	},
	[5] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0xF180000,
		.size        = 0x0800000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[6] = {
		.name        = "MassStorage",
		.offset      = 0xF980000,
		.size        = 0x0100000,	/* 1M */
	},
	[7] = {
		.name        = "BBT",
		.offset      = 0xfa80000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};

static struct mtd_partition android_128m_v75_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "NVM",
		.offset      = 0x100000,
		.size        = 0x020000,
	},
	[2] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0x120000, 
		.size        = 0x800000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},	
	[3] = {
		.name        = "Kernel",
		.offset      = 0x920000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[4] = {
		.name        = "system",
		.offset      = 0x0c20000,
		.size        = 0x4000000,     /* mount 64M fs */
	},
	[5] = {
		.name        = "userdata",
		.offset      = 0x4c20000,
		.size        = 0x20E0000,     /* mount 32.875M */
	},
	[6] = {
		.name        = "filesystem",
		.offset      = 0x6d00000,
		.size        = 0x1000000,     /* mount 16M fs */
	},
	[7] = {
		.name        = "BBT",
		.offset      = 0x7d00000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};

static struct mtd_partition android_256m_v75_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE, /* force read-only */
	},
	[1] = {
		.name        = "NVM",
		.offset      = 0x100000,
		.size        = 0x020000,
	},
	[2] = {
		.name        = "Arbel and Greyback Image",
		.offset      = 0x120000, 
		.size        = 0x800000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},	
	[3] = {
		.name        = "Kernel",
		.offset      = 0x920000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	[4] = {
		.name        = "system",
		.offset      = 0x0c20000,
		.size        = 0x4000000,     /* mount 64M fs */
	},
	[5] = {
		.name        = "userdata",
		.offset      = 0x4c20000,
		.size        = 0x9e60000,     /* mount 158.375M fs */
	},
	[6] = {
		.name        = "filesystem",
		.offset      = 0xea80000,
		.size        = 0x1000000,     /* mount 16M fs */
	},
	[7] = {
		.name        = "BBT",
		.offset      = 0xfa80000,
		.size        = 0x0080000,
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */
	},
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.
	 * And the max relocation blocks is not same on different platform.
	 * Please take care it when define the partition table.
	 */
};
#endif

