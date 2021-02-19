/*
 * Code for AM335X EVM.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/mfd/tps65217.h>
#include <linux/pwm_backlight.h>
#include <linux/input/ti_tsc.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/rtc/rtc-omap.h>
#include <linux/opp.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>
#include <plat/dma-33xx.h>

#include <media/soc_camera.h>
#include <media/mt9t112.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"

#include "control.h"
#include <linux/proc_fs.h>

#ifdef CONFIG_CAN_MCP251X
#include <linux/can/platform/mcp251x.h>
#endif
/*cyttsp*/
#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_platform.h>
/*cyttsp*/

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
/*touchscreen config*/
/*cyttsp*/
#define CYTTSP5_USE_I2C
/* #define CYTTSP5_USE_SPI */
#ifdef CYTTSP5_USE_I2C
    #define CYTTSP5_I2C_TCH_ADR 0x24
    #define CYTTSP5_LDR_TCH_ADR 0x24
 #if defined(CONFIG_OK335XD)
 	#define CYTTSP5_I2C_IRQ_GPIO GPIO_TO_PIN(0,19)
	#define CYTTSP5_I2C_RST_GPIO GPIO_TO_PIN(0,20)
 #elif defined(CONFIG_OK335XS)
 	#define CYTTSP5_I2C_IRQ_GPIO GPIO_TO_PIN(1,9)
 	#define CYTTSP5_I2C_RST_GPIO GPIO_TO_PIN(1,8)
 #elif defined(CONFIG_OK335XS2)
 	#define CYTTSP5_I2C_IRQ_GPIO GPIO_TO_PIN(2,0)
 	#define CYTTSP5_I2C_RST_GPIO GPIO_TO_PIN(3,0)
 #endif
   // #define CYTTSP5_I2C_IRQ_GPIO GPIO_TO_PIN(0,19) /* J6.9, C19, GPMC_AD14/GPIO_38 */
   // #define CYTTSP5_I2C_RST_GPIO GPIO_TO_PIN(0,20) /* J6.10, D18, GPMC_AD13/GPIO_37 */
#endif

#ifdef CYTTSP5_USE_SPI
/* Change GPIO numbers when using I2C and SPI at the same time
 * Following is possible alternative:
 * IRQ: J6.17, C18, GPMC_AD12/GPIO_36
 * RST: J6.24, D17, GPMC_AD11/GPIO_35
 */
 #if defined(CONFIG_OK335XD)
        #define CYTTSP5_I2C_IRQ_GPIO GPIO_TO_PIN(0,19)
        #define CYTTSP5_I2C_RST_GPIO GPIO_TO_PIN(0,20)
 #elif defined(CONFIG_OK335XS)
        #define CYTTSP5_I2C_IRQ_GPIO GPIO_TO_PIN(1,9)
        #define CYTTSP5_I2C_RST_GPIO GPIO_TO_PIN(1,8)
 #elif defined(CONFIG_OK335XS2)
        #define CYTTSP5_I2C_IRQ_GPIO GPIO_TO_PIN(2,0)
        #define CYTTSP5_I2C_RST_GPIO GPIO_TO_PIN(3,0)
 #endif

   //#define CYTTSP5_SPI_IRQ_GPIO GPIO_TO_PIN(0,19) /* J6.9, C19, GPMC_AD14/GPIO_38 */
   //#define CYTTSP5_SPI_RST_GPIO GPIO_TO_PIN(0,20) /* J6.10, D18, GPMC_AD13/GPIO_37 */
#endif

/* Check GPIO numbers if both I2C and SPI are enabled */
#if defined(CYTTSP5_USE_I2C) && defined(CYTTSP5_USE_SPI)
   #if CYTTSP5_I2C_IRQ_GPIO == CYTTSP5_SPI_IRQ_GPIO || \
							CYTTSP5_I2C_RST_GPIO == CYTTSP5_SPI_RST_GPIO
       #error "GPIO numbers should be different when both I2C and SPI are on!"
   #endif
#endif
/*cyttsp*/
/*touchscreen config*/



/* BBB PHY IDs */
#define BBB_PHY_ID		0x7c0f1
#define BBB_PHY_MASK		0xfffffffe

/* AM335X EVM Phy ID and Debug Registers */
#if defined(CONFIG_OK335XS2)
//#define AM335X_EVM_PHY_ID       0x080017
#define AM335X_EVM_PHY_ID       0x7c0f0 
//#define AM335X_EVM_PHY_ID       0x20005c90 
#else
#define AM335X_EVM_PHY_ID		0x4dd074
#endif

#define AM335X_EVM_PHY_MASK		0xfffffffe
#define AR8051_PHY_DEBUG_ADDR_REG	0x1d
#define AR8051_PHY_DEBUG_DATA_REG	0x1e
#define AR8051_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR8051_RGMII_TX_CLK_DLY		BIT(8)


#define AM33XX_CTRL_REGADDR(reg)				\
		AM33XX_L4_WK_IO_ADDRESS(AM33XX_SCM_BASE + (reg))

/* bit 3: 0 - enable, 1 - disable for pull enable */
#define AM33XX_PULL_DISA		(1 << 3)
#define AM33XX_PULL_ENBL		(0 << 3)

int selected_pad;
int pad_mux_value;

#define SCREEN_TYPE_R 0
#define SCREEN_TYPE_C 1
#define SCREEN_SIZE_800X480_5  0
#define SCREEN_SIZE_800X480_7  1
#define SCREEN_SIZE_1024X600_7  10
#define SCREEN_SIZE_1280X800_10   11
#define SCREEN_SIZE_800X600_8  2
#define SCREEN_SIZE_480X272_4  3
#define SCREEN_SIZE_800X600_10 4
#define SCREEN_SIZE_1280X720_vga 5
#define SCREEN_SIZE_1024X768_vga 6
#define SCREEN_SIZE_800X600_vga 7
#define SCREEN_SIZE_320X240_3   8 
#define SCREEN_SIZE_640X480_5   9
int screen_type;
int screen_size;

static const struct display_panel disp_panel = {
	WVGA,
	32,
	8,
	COLOR_ACTIVE,
};

/* LCD backlight platform Data */
#define AM335X_BACKLIGHT_MAX_BRIGHTNESS        100
#define AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS    60
#define AM335X_PWM_PERIOD_NANO_SECONDS        (5000 * 10)

static struct platform_pwm_backlight_data am335x_backlight_data0 = {
	.pwm_id         = "ecap.0",
	.ch             = -1,
	.lth_brightness	= 21,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 16,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

struct da8xx_lcdc_platform_data  NHD_480272MF_ATXI_pdata_4 = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-4.3-ATXI#-T-1",
};

struct da8xx_lcdc_platform_data  NHD_800480MF_ATXI_pdata_5 = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-5.0-ATXI#-T-1",
};

struct da8xx_lcdc_platform_data  NHD_800480MF_ATXI_pdata_7 = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-7.0-ATXI#-T-1",
};
struct da8xx_lcdc_platform_data  NHD_1024600MF_ATXI_pdata_7 = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-7.0-1024-ATXI#-T-1",
};
struct da8xx_lcdc_platform_data  NHD_800600MF_ATXI_pdata_8 = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-8.0-ATXI#-T-1",
};
struct da8xx_lcdc_platform_data  NHD_800600MF_ATXI_pdata_10 = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-10.4-ATXI#-T-1",
};
struct da8xx_lcdc_platform_data  NHD_1280800MF_ATXI_pdata_10 = {
        .manu_name              = "NHD",
        .controller_data        = &lcd_cfg,
        .type                   = "NHD-10.1-ATXI#-T-1",
};
struct da8xx_lcdc_platform_data NHD_1280720MF_ATXI_pdata_vga = {

	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "nxp-1280x720@60",
};

struct da8xx_lcdc_platform_data NHD_1024768MF_ATXI_pdata_vga = {

	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "nxp-1024x768@60",
};

struct da8xx_lcdc_platform_data NHD_800600MF_ATXI_pdata_vga = {

	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "nxp-800x600@60",
};
struct da8xx_lcdc_platform_data NHD_320240MF_ATXI_pdata_3 = {

	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "nxp-320x240@60",
};
struct da8xx_lcdc_platform_data NHD_640480MF_ATXI_pdata_5 = {

        .manu_name              = "NHD",
        .controller_data        = &lcd_cfg,
        .type                   = "nxp-640x480@60",
};
#include "common.h"

#include <linux/lis3lv02d.h>

static const struct display_panel dvi_panel = {
	WVGA,
	16,
	16,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config dvi_cfg = {
	&dvi_panel,
	.ac_bias    = 255,
	.ac_bias_intrpt    = 0,
	.dma_burst_sz    = 16,
	.bpp      = 16,
	.fdd      = 0x80,
	.tft_alt_mode    = 0,
	.stn_565_mode    = 0,
	.mono_8bit_mode    = 0,
	.invert_line_clock  = 1,
	.invert_frm_clock  = 1,
	.sync_edge    = 0,
	.sync_ctrl    = 1,
	.raster_order    = 0,
};

struct da8xx_lcdc_platform_data dvi_pdata = {
	.manu_name    = "BBToys",
	.controller_data  = &dvi_cfg,
	.type      = "1024x768@60",
};

struct da8xx_lcdc_platform_data hdmi_pdata = {
	.manu_name    = "NXP HDMI",
	.controller_data  = &dvi_cfg,
	.type      = "nxp-1280x720@60",
};

/* Touchscreen Controller Data for AM335xEVM */
/* Calibrated on AM335xEVM Rev. 1.1A and 1.2A */
/* The values have to be fine tuned for other revisions, if requred */
static struct tsc_data am335xevm_touchscreen_data = {
	.wires = 4,
	.x = {
		.min = 0x8c,
		.max = 0xf6a,
		.inverted = 0,
	},
	.y = {
		.min = 0x13f,
		.max = 0xf19,
		.inverted = 1,
	},
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static struct adc_data am335x_adc_data = {
	.adc_channels = 4,
};

static struct mfd_tscadc_board tscadc = {
	.tsc_init = &am335xevm_touchscreen_data,
	.adc_init = &am335x_adc_data,
};

static struct adc_data am335x_adc_data_s2 = {
	.adc_channels = 8,
};

static struct mfd_tscadc_board adc_s2 = {
	.adc_init = &am335x_adc_data_s2,
};


static u8 am335x_iis_serializer_direction1[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	TX_MODE,	RX_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data am335x_evm_snd_data1 = {
	.tx_dma_offset	= 0x46400000,	/* McASP1 */
	.rx_dma_offset	= 0x46400000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_iis_serializer_direction1),
	.tdm_slots	= 2,
	.serial_dir	= am335x_iis_serializer_direction1,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 32,
	.rxnumevt	= 32,
	.get_context_loss_count	=
			omap_pm_get_dev_context_loss_count,
};


static u8 am335x_evm_sk_iis_serializer_direction1[] = {
	TX_MODE,	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data am335x_evm_sk_snd_data1 = {
	.tx_dma_offset	= 0x46400000,	/* McASP1 */
	.rx_dma_offset	= 0x46400000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_evm_sk_iis_serializer_direction1),
	.tdm_slots	= 2,
	.serial_dir	= am335x_evm_sk_iis_serializer_direction1,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 32,
	.rxnumevt	= 32,
	.get_context_loss_count	=
			omap_pm_get_dev_context_loss_count,
};


static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(0, 6),
		.gpio_wp        = GPIO_TO_PIN(2, 0),//forlinx
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/*
	 * Setting SYSBOOT[5] should set xdma_event_intr0 pin to mode 3 thereby
	 * allowing clkout1 to be available on xdma_event_intr0.
	 * However, on some boards (like EVM-SK), SYSBOOT[5] isn't properly
	 * latched.
	 * To be extra cautious, setup the pin-mux manually.
	 * If any modules/usecase requries it in different mode, then subsequent
	 * module init call will change the mux accordingly.
	 */
	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

struct evm_dev_cfg {
	void (*device_init)(int evm_id, int profile);

/*
* If the device is required on both baseboard & daughter board (ex i2c),
* specify DEV_ON_BASEBOARD
*/
#define DEV_ON_BASEBOARD	0
#define DEV_ON_DGHTR_BRD	1
	u32 device_on;

	u32 profile;	/* Profiles (0-7) in which the module is present */
};

/* AM335X - CPLD Register Offsets */
#define	CPLD_DEVICE_HDR	0x00 /* CPLD Header */
#define	CPLD_DEVICE_ID	0x04 /* CPLD identification */
#define	CPLD_DEVICE_REV	0x0C /* Revision of the CPLD code */
#define	CPLD_CFG_REG	0x10 /* Configuration Register */

static u32 am335x_evm_id;
static struct omap_board_config_kernel am335x_evm_config[] __initdata = {
};

/*
* EVM Config held in On-Board eeprom device.
*
* Header Format
*
*  Name			Size	Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				Example "A33515BB" = "AM335x 15x15 Base Board"
*
*  Version		4	Hardware version code for board	in ASCII.
*				"1.0A" = rev.01.0A
*
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is WWYY4P16nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*
*  Configuration option	32	Codes(TBD) to show the configuration
*				setup on this board.
*
*  Available		32720	Available space for other non-volatile data.
*/
struct am335x_evm_eeprom_config {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
};

/*
* EVM Config held in daughter board eeprom device.
*
* Header Format
*
*  Name			Size		Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				example "A335GPBD" = "AM335x
*				General Purpose Daughterboard"
*
*  Version		4	Hardware version code for board in
*				in ASCII. "1.0A" = rev.01.0A
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is: WWYY4P13nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*  Configuration Option	32	Codes to show the configuration
*				setup on this board.
*  CPLD Version	8		CPLD code version for board in ASCII
*				"CPLD1.0A" = rev. 01.0A of the CPLD
*  Available	32700		Available space for other non-volatile
*				codes/data
*/

struct am335x_eeprom_config1 {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
	u8	cpld_ver[8];
};

/*
*Beaglebone cape EEPROM config
*
*-------------------------------------------------------------------
*Name		offset	size 	contents
*--------------------------------------------------------------------

*Header 	0 	4 	0xAA, 0x55, 0x33, 0xEE
*EEPROM Format
*
*Revision	4 	2 	Revision number of the overall format
*				of this EEPROM in ASCII =A0
*
*Board Name 	6 	32 	Name of board in ASCII
*
*Version 	38 	4 	Hardware version code for board in ASCII
*
*Manufacturer 	42 	16 	ASCII name of the manufacturer
*
*Part Number 	60 	16 	ASCII Characters for the part number
*
*Number of Pins 74 	2 	Number of pins used by the daughter board
*
*Serial Number	76	12	Serial number of the board. This is a 12
*				character string which is:
*				WWYY4P13nnnn where:
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*/

struct bone_cape_eeprom_config {
	u8	header[4];
	u8 	revision[2];
	u8	board_name[32];
	u8	version[4];
	u8	manufacturer[16];
	u8	part_no[16];
	u8 	no_of_pins[2];
	u8 	serial_no[12];
};


static bool daughter_brd_detected;


static int am33xx_evmid = -EINVAL;

/*
* am335x_evm_set_id - set up board evmid
* @evmid - evm id which needs to be configured
*
* This function is called to configure board evm id.
*/
void am335x_evm_set_id(unsigned int evmid)
{
	am33xx_evmid = evmid;
	return;
}

/*
* am335x_evm_get_id - returns Board Type (EVM/BB/EVM-SK ...)
*
* Note:
*	returns -EINVAL if Board detection hasn't happened yet.
*/
int am335x_evm_get_id(void)
{
	return am33xx_evmid;
}
EXPORT_SYMBOL(am335x_evm_get_id);


/* Module pin mux for LCDC */
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};


/* Pin mux for nand flash module */
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
//	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};

/* Module pin mux for SPI flash */
static struct pinmux_config spi1_pin_mux[] = 
{
#if defined(CONFIG_CAN_MCP251X)
	{"gpmc_ad13.gpio1_13",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
#endif
	{"mcasp0_aclkx.spi1_sclk", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL| AM33XX_INPUT_EN},
	{"mcasp0_fsx.spi1_d0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL| AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"mcasp0_axr0.spi1_d1", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL| AM33XX_INPUT_EN},
	{"mcasp0_ahclkr.spi1_cs0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL| AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for rgmii1 */
static struct pinmux_config rgmii1_pin_mux[] = {
	{"mii1_txen.rgmii1_tctl", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.rgmii1_rctl", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.rgmii1_td3", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.rgmii1_td2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.rgmii1_td1", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.rgmii1_td0", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.rgmii1_tclk", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxclk.rgmii1_rclk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.rgmii1_rd3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.rgmii1_rd2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.rgmii1_rd1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.rgmii1_rd0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rgmii2 */

/* Module pin mux for rmii1 */

static struct pinmux_config i2c1_pin_mux[] = {
	{"spi0_d1.i2c1_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_cs0.i2c1_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Pin mux for GPMC bus */
/* Module pin mux for mcasp1 */
static struct pinmux_config mcasp1_pin_mux[] = {
	{"mcasp0_fsr.mcasp1_fsx",     OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_aclkr.mcasp1_aclkx", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_axr1.mcasp1_axr0",   OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_ahclkx.mcasp1_axr1", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_common_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc0_wp_only_pin_mux[] = {
	{"gpmc_csn3.gpio2_0", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc0_cd_only_pin_mux[] = {
	{"spi0_cs1.gpio0_6",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for mmc1 */
static struct pinmux_config mmc1_common_pin_mux[] = {
	{"gpmc_ad11.mmc1_dat3",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad10.mmc1_dat2",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad9.mmc1_dat1",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad8.mmc1_dat0",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.mmc1_clk",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn2.mmc1_cmd",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_clk.gpio2_1",	OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};


/* Module pin mux for uart2 */
static struct pinmux_config uart2_pin_mux[] = {
	{"spi0_sclk.uart2_rxd", OMAP_MUX_MODE1 | AM33XX_SLEWCTRL_SLOW |AM33XX_PIN_INPUT_PULLUP},
	{"spi0_d0.uart2_txd", OMAP_MUX_MODE1 | AM33XX_PULL_UP |AM33XX_PULL_DISA |AM33XX_SLEWCTRL_SLOW},
//	{"mii1_rxerr.gpio3_2",OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config uart4_pin_mux[] = {
     {"uart0_ctsn.uart4_rxd", OMAP_MUX_MODE1 | AM33XX_SLEWCTRL_SLOW |AM33XX_PIN_INPUT_PULLUP},
     {"uart0_rtsn.uart4_txd", OMAP_MUX_MODE1 | AM33XX_PULL_UP |AM33XX_PULL_DISA |AM33XX_SLEWCTRL_SLOW},
     {NULL, 0},
};


static struct pinmux_config d_can0_pin_mux[] = {
     {"uart1_ctsn.d_can0_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
     {"uart1_rtsn.d_can0_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
     {NULL, 0},
};


/* pinmux for gpio based key */
static struct pinmux_config gpio_keys_pin_mux_forlinx[] = {
    {"gpmc_a4.gpio1_20", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
    {"gpmc_a5.gpio1_21", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
    {"gpmc_a6.gpio1_22", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a7.gpio1_23", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
    {"gpmc_a8.gpio1_24", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
    {"gpmc_a9.gpio1_25", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
    {NULL, 0},
};

/* pinmux for led device */
static struct pinmux_config gpio_led_mux[] = {
	{"gpmc_a0.gpio1_16", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a1.gpio1_17", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a2.gpio1_18", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_a3.gpio1_19", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"emu1.gpio3_8", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

static struct pinmux_config mini_gpio_mux[] = {
		{"spi0_d0.gpio0_3", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"spi0_sclk.gpio0_2", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"uart1_txd.gpio0_15", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"uart1_rxd.gpio0_14", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mii1_rxd3.gpio2_18", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mii1_rxd2.gpio2_19", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mii1_txd3.gpio0_16", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mii1_txd2.gpio0_17", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_ad11.gpio0_27", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_ad12.gpio1_12", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_ad10.gpio0_26", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	//	{"gpmc_ad13.gpio1_13", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_ad9.gpio0_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_ad14.gpio1_14", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_ad8.gpio0_22", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_ad15.gpio1_15", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_csn2.gpio1_31", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mcasp0_aclkr.gpio3_18", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_csn1.gpio1_30", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mcasp0_axr1.gpio3_20", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_clk.gpio2_1", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mcasp0_fsr.gpio3_19", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mcasp0_fsx.gpio3_15", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mcasp0_axr0.gpio3_16", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mcasp0_aclkx.gpio3_14", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_a8.gpio1_24", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	//	{"gpmc_wpn.gpio0_31", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_a11.gpio1_27", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_a10.gpio1_26", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mii1_txclk.gpio3_9", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_a7.gpio1_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mii1_rxdv.gpio3_4", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"mii1_rxclk.gpio3_10", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"uart0_rtsn.gpio1_9", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"uart0_ctsn.gpio1_8", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"uart1_ctsn.gpio0_12", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"uart1_rtsn.gpio0_13", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		//  {"gpmc_a11.gpio1_19", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_a4.gpio1_20", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_a5.gpio1_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_a6.gpio1_22", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
		{"gpmc_csn0.gpio1_29", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},//boot
		{"spi0_cs0.gpio0_5", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},//boot
		{"spi0_d1.gpio0_4", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},//boot
		{"xdma_event_intr0.gpio0_19", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
		
		{"xdma_event_intr1.gpio0_20", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},	
		{NULL, 0},
};
/*ft5XXX*/
static struct pinmux_config tsc_gpio_pin_mux[] = {
	{"gpmc_csn3.gpio2_0", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"mii1_col.gpio3_0", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"xdma_event_intr0.gpio0_19", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};


/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
	{
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
	}
}


/* Keys mapping */
static const uint32_t am335x_evm_matrix_keys[] = {
	KEY(0, 0, KEY_MENU),
	KEY(1, 0, KEY_BACK),
	KEY(2, 0, KEY_LEFT),

	KEY(0, 1, KEY_RIGHT),
	KEY(1, 1, KEY_POWER),
	KEY(2, 1, KEY_DOWN),
};

const struct matrix_keymap_data am335x_evm_keymap_data = {
	.keymap      = am335x_evm_matrix_keys,
	.keymap_size = ARRAY_SIZE(am335x_evm_matrix_keys),
};

static const unsigned int am335x_evm_keypad_row_gpios[] = {
	GPIO_TO_PIN(1, 25), GPIO_TO_PIN(1, 26), GPIO_TO_PIN(1, 27)
};

static const unsigned int am335x_evm_keypad_col_gpios[] = {
	GPIO_TO_PIN(1, 21), GPIO_TO_PIN(1, 22)
};



/*
* @evm_id - evm id which needs to be configured
* @dev_cfg - single evm structure which includes
*				all module inits, pin-mux defines
* @profile - if present, else PROFILE_NONE
* @dghtr_brd_flg - Whether Daughter board is present or not
*/
static void _configure_device(int evm_id, struct evm_dev_cfg *dev_cfg,
	int profile)
{
	int i;

	am335x_evm_set_id(evm_id);

	/*
	* Only General Purpose & Industrial Auto Motro Control
	* EVM has profiles. So check if this evm has profile.
	* If not, ignore the profile comparison
	*/

	/*
	* If the device is on baseboard, directly configure it. Else (device on
	* Daughter board), check if the daughter card is detected.
	*/
	if (profile == PROFILE_NONE) {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->device_on == DEV_ON_BASEBOARD)
				dev_cfg->device_init(evm_id, profile);
			else if (daughter_brd_detected == true)
				dev_cfg->device_init(evm_id, profile);
		}
	} else {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->profile & profile) {
				if (dev_cfg->device_on == DEV_ON_BASEBOARD)
					dev_cfg->device_init(evm_id, profile);
				else if (daughter_brd_detected == true)
					dev_cfg->device_init(evm_id, profile);
			}
		}
	}
}


/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for usb1 drvvbus */
static struct pinmux_config usb1_pin_mux[] = {	
	{"usb1_drvvbus.usb1_drvvbus",	 OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_crs.gpio3_1",		OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};



/* Module pin mux for eCAP0 */
static struct pinmux_config ecap0_pin_mux[] = {
	{"ecap0_in_pwm0_out.ecap0_in_pwm0_out",
		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

#define AM335XEVM_WLAN_PMENA_GPIO	GPIO_TO_PIN(1, 30)
#define AM335XEVM_WLAN_IRQ_GPIO		GPIO_TO_PIN(3, 17)
//#define AM335XEVM_SK_WLAN_IRQ_GPIO      GPIO_TO_PIN(0, 31)

struct wl12xx_platform_data am335xevm_wlan_data = {
	.irq = OMAP_GPIO_IRQ(AM335XEVM_WLAN_IRQ_GPIO),
#ifdef CONFIG_MACH_AM335XEVM_WILINK8
        .board_ref_clock = WL12XX_REFCLOCK_38,
        .board_tcxo_clock = WL12XX_TCXOCLOCK_26,
#else
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4Mhz */
#endif
	.bt_enable_gpio = GPIO_TO_PIN(3, 21),
	.wlan_enable_gpio = GPIO_TO_PIN(1, 16),
};

static struct pinmux_config uart1_pin_mux[] = {
    {"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
    {"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
    {NULL, 0},
};


#ifdef CONFIG_LINUX
   #if defined(CONFIG_CAN_MCP251X)
	#if defined(CONFIG_OK335XD)
		#define AM335XEVM_MCP2515_IRQ_GPIO              GPIO_TO_PIN(1, 13)
	#elif defined(CONFIG_OK335XS)
		#define AM335XEVM_MCP2515_IRQ_GPIO              GPIO_TO_PIN(0, 31)
	#endif
	#if defined(CONFIG_OK335XD)||defined(CONFIG_OK335XS)
		static struct mcp251x_platform_data mcp251x_info={
        		.oscillator_frequency = 8000000,
      		 	.transceiver_enable = NULL,
        		.power_enable = NULL,
		};
	#endif
   #endif
#endif
/*spi1  con9*/

static bool backlight_enable;

/* Setup pwm-backlight */
static struct platform_device am335x_backlight = {
	.name           = "pwm-backlight",
	.id             = -1,
};

static struct pwmss_platform_data  pwm_pdata[3] = {
        {
                .version = PWM_VERSION_1,
        },
        {
                .version = PWM_VERSION_1,
        },
        {
                .version = PWM_VERSION_1,
        },
};

static void ecap_init(int evm_id, int profile)
{
	backlight_enable = true;
	setup_pin_mux(ecap0_pin_mux);
}

/*pwm for buzzer*/
static void buzzer_init(int evm_id, int profile)
{
}


static int __init backlight_init(void)
{
	int index = 0;

	index = 0;
	am335x_backlight.dev.platform_data = &am335x_backlight_data0;
	am33xx_register_ecap(index, &pwm_pdata[index]);
	platform_device_register(&am335x_backlight);

	return 0;
}
late_initcall(backlight_init);

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}


static void lcd_init(int evm_id, int profile)
{
	struct da8xx_lcdc_platform_data *lcdc_pdata;
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}

	if(screen_size == SCREEN_SIZE_800X600_8)
		lcdc_pdata = &NHD_800600MF_ATXI_pdata_8;
	else if(screen_size == SCREEN_SIZE_800X480_5)
		lcdc_pdata = &NHD_800480MF_ATXI_pdata_5;
	else if(screen_size == SCREEN_SIZE_800X480_7)
		lcdc_pdata = &NHD_800480MF_ATXI_pdata_7;
	else if(screen_size == SCREEN_SIZE_1024X600_7)
		lcdc_pdata = &NHD_1024600MF_ATXI_pdata_7;
	else if(screen_size == SCREEN_SIZE_800X600_10)
		lcdc_pdata = &NHD_800600MF_ATXI_pdata_10;
	else if(screen_size == SCREEN_SIZE_480X272_4)
		lcdc_pdata = &NHD_480272MF_ATXI_pdata_4;
	else if(screen_size == SCREEN_SIZE_1280X720_vga)
		lcdc_pdata = &NHD_1280720MF_ATXI_pdata_vga;
	else if(screen_size == SCREEN_SIZE_1024X768_vga)
		lcdc_pdata = &NHD_1024768MF_ATXI_pdata_vga;
	else if(screen_size == SCREEN_SIZE_320X240_3)
		lcdc_pdata = &NHD_320240MF_ATXI_pdata_3;
    	else if(screen_size == SCREEN_SIZE_640X480_5)
                lcdc_pdata = &NHD_640480MF_ATXI_pdata_5;
	else if(screen_size == SCREEN_SIZE_1280X800_10)
               lcdc_pdata = &NHD_1280800MF_ATXI_pdata_10;
	else
		lcdc_pdata = &NHD_800600MF_ATXI_pdata_vga;
			
	
	lcdc_pdata->get_context_loss_count = omap_pm_get_dev_context_loss_count;

	if (am33xx_register_lcdc(lcdc_pdata))
		pr_info("Failed to register LCDC device\n");

	return;
}

static void tscadc_init(int evm_id, int profile)
{
	int err;

	err = am33xx_register_mfd_tscadc(&tscadc);
	if (err)
		pr_err("failed to register touchscreen device\n");
}

static void adc_init(int evm_id, int profile)
{
	int err;

	err = am33xx_register_mfd_tscadc(&adc_s2);
	if (err)
		pr_err("failed to register touchscreen device\n");
}


struct cssp_cam_platform_data {
	struct i2c_board_info *cam_i2c_board_info;
	const char *cam_clk_name;
	int dma_ch;
	int cssp_reset_pin;
};


#define BEAGLEBONE_CAMERA_ORIENTATION GPIO_TO_PIN(0, 30)
static void net_init(int evm_id, int profile)
{
	setup_pin_mux(rgmii1_pin_mux);
	return;
}

static void usb_init(int evm_id, int profile)
{
	setup_pin_mux(usb0_pin_mux);
	setup_pin_mux(usb1_pin_mux);

	//fix usb vbus error by set gpio3_1 high
	#define GPIO3_1 3<<5|1
	gpio_request(GPIO3_1,"gpio3_1");
	gpio_direction_output(GPIO3_1,1);
	gpio_set_value(GPIO3_1,1);
	return;
}


/* setup uart1 for ok335xd */
static void uart_init(int evm_id, int profile)
{
	setup_pin_mux(uart1_pin_mux);
	setup_pin_mux(uart2_pin_mux);
	setup_pin_mux(uart4_pin_mux);
	return;
}

/* NAND partition information */
static struct mtd_partition am335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "SPL",
		.offset         = 0,			/* Offset = 0x0 */
		.size           = SZ_512K * 4,
	},
	{
		.name           = "SPL.backup1",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x200000 */
		.size           = SZ_512K * 4,
	},
	{
		.name           = "SPL.backup2",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x400000 */
		.size           = SZ_512K * 4,
	},
	{
		.name           = "SPL.backup3",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x600000 */
		.size           = SZ_512K * 4,
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x800000 */
		.size           = 4 * SZ_512K,
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0xa00000 */
		.size           = 4 * SZ_512K,
	},
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0xc00000 */
		.size           = 16 * SZ_512K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x1400000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

/* SPI 0/1 Platform Data */
/* SPI flash information */
static struct mtd_partition am335x_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name       = "SPL",
		.offset     = 0,			/* Offset = 0x0 */
		.size       = SZ_128K,
	},
	{
		.name       = "U-Boot",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size       = 2 * SZ_128K,
	},
	{
		.name       = "U-Boot Env",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size       = 2 * SZ_4K,
	},
	{
		.name       = "Kernel",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x62000 */
		.size       = 28 * SZ_128K,
	},
	{
		.name       = "File System",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x3E2000 */
		.size       = MTDPART_SIZ_FULL,		/* size ~= 4.1 MiB */
	}
};

static const struct flash_platform_data am335x_spi_flash = {
	.type      = "w25q64",
	.name      = "spi_flash",
	.parts     = am335x_spi_partitions,
	.nr_parts  = ARRAY_SIZE(am335x_spi_partitions),
};

/*
 * SPI Flash works at 80Mhz however SPI Controller works at 48MHz.
 * So setup Max speed to be less than that of Controller speed
 */
static struct spi_board_info am335x_spi1_slave_info[] = {
#if defined(CONFIG_CAN_MCP251X)
		{
				.modalias      = "mcp2515", /* device node name */
				.platform_data=&mcp251x_info,
				.max_speed_hz    = 10000000,
				.bus_num         = 2,
				.chip_select     = 0,
				.mode            = SPI_MODE_0,  /* CPOL=0, CPHA=0 */
				.irq			 = OMAP_GPIO_IRQ(AM335XEVM_MCP2515_IRQ_GPIO),
		}
#elif defined(CONFIG_SPI_SPIDEV) 
	{	.modalias      = "spidev",
                .max_speed_hz  = 48000000,//48Mbps
                .bus_num       = 2,
                .chip_select   = 0,
                .mode = SPI_MODE_1,
        },
#endif
};

static struct gpmc_timings am335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 170,
	.cs_wr_off = 170,

	.adv_on = 50 ,
	.adv_rd_off = 120,
	.adv_wr_off = 120,
	.we_off = 90,
	.oe_off = 90,

	.access = 90,
	.rd_cycle = 150,
	.wr_cycle = 150,

	.wr_access = 100,
	.wr_data_mux_bus = 0,
};

static void nand_init(int evm_id, int profile)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		{ NULL, 0 },
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);
	if (!pdata)
		return;
#ifdef CONFIG_MTD_NAND_OMAP_ECC_BCH8_CODE_HW
	pdata->ecc_opt =OMAP_ECC_BCH8_CODE_HW;
#elif  CONFIG_MTD_NAND_OMAP_ECC_BCH16_CODE_HW
	pdata->ecc_opt = OMAP_ECC_BCH16_CODE_HW;
#else
    pdata->ecc_opt =OMAP_ECC_HAMMING_CODE_HW;
#endif

//	pdata->elm_used = true;
	pdata->dev_ready = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

/* TPS65217 voltage regulator support */

/* 1.8V */
static struct regulator_consumer_supply tps65217_dcdc1_consumers[] = {
	{
		.supply = "vdds_osc",
	},
	{
		.supply = "vdds_pll_ddr",
	},
	{
		.supply = "vdds_pll_mpu",
	},
	{
		.supply = "vdds_pll_core_lcd",
	},
	{
		.supply = "vdds_sram_mpu_bb",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdda_usb0_1p8v",
	},
	{
		.supply = "vdds_ddr",
	},
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_hvx_1p8v",
	},
	{
		.supply = "vdda_adc",
	},
	{
		.supply = "ddr2",
	},
};

/* 1.1V */
static struct regulator_consumer_supply tps65217_dcdc2_consumers[] = {
	{
		.supply = "vdd_core"
	},
};

/* 1.1V */
static struct regulator_consumer_supply tps65217_dcdc3_consumers[] = {
	{
		.supply = "vdd_mpu",
	},
};

/* 1.8V LDO */
static struct regulator_consumer_supply tps65217_ldo1_consumers[] = {
	{
		.supply = "vdds_rtc",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo2_consumers[] = {
	{
		.supply = "vdds_any_pn",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo3_consumers[] = {
	{
		.supply = "vdds_hvx_ldo3_3p3v",
	},
	{
		.supply = "vdda_usb0_3p3v",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo4_consumers[] = {
	{
		.supply = "vdds_hvx_ldo4_3p3v",
	},
};

/*
 * FIXME: Some BeagleBones reuire a ramp_delay to settle down the set
 * voltage from 0.95v to 1.25v. By default a minimum of 70msec is set
 * based on experimentation. This will be removed/modified to exact
 * value, once the root cause is known.
 *
 * The reason for extended ramp time requirement on BeagleBone is not
 * known and the delay varies from board - board, if the board hangs
 * with this 70msec delay then try to increase the value.
 */
static struct tps65217_rdelay dcdc2_ramp_delay = {
	.ramp_delay = 70000,
};

static struct regulator_init_data tps65217_regulator_data[] = {
	/* dcdc1 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 1800000,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc1_consumers),
		.consumer_supplies = tps65217_dcdc1_consumers,
	},

	/* dcdc2 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc2_consumers),
		.consumer_supplies = tps65217_dcdc2_consumers,
		.driver_data = &dcdc2_ramp_delay,
	},

	/* dcdc3 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 1500000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc3_consumers),
		.consumer_supplies = tps65217_dcdc3_consumers,
	},

	/* ldo1 */
	{
		.constraints = {
			.min_uV = 1000000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo1_consumers),
		.consumer_supplies = tps65217_ldo1_consumers,
	},

	/* ldo2 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo2_consumers),
		.consumer_supplies = tps65217_ldo2_consumers,
	},

	/* ldo3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo3_consumers),
		.consumer_supplies = tps65217_ldo3_consumers,
	},

	/* ldo4 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo4_consumers),
		.consumer_supplies = tps65217_ldo4_consumers,
	},
};

struct tps65217_bl_pdata bone_lcd3_bl_pdata[] = {
	{
		.isel = TPS65217_BL_ISET1,
		.fdim = TPS65217_BL_FDIM_200HZ,
	},
};
/*cypress5*/
#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
#define CYTTSP5_HID_DESC_REGISTER 1

#define CY_VKEYS_X 800
#define CY_VKEYS_Y 480
#define CY_MAXX 800
#define CY_MAXY 480
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255
#define CY_PROXIMITY_MIN_VAL	0
#define CY_PROXIMITY_MAX_VAL	1

#define CY_ABS_MIN_T 0
#define CY_ABS_MAX_T 15

/* Button to keycode conversion */
static u16 cyttsp5_btn_keys[] = {
		/* use this table to map buttons to keycodes (see input.h) */
		KEY_HOMEPAGE,		/* 172 */ /* Previously was KEY_HOME (102) */
						/* New Android versions use KEY_HOMEPAGE */
		KEY_MENU,		/* 139 */
		KEY_BACK,		/* 158 */
		KEY_SEARCH,		/* 217 */
		KEY_VOLUMEDOWN,		/* 114 */
		KEY_VOLUMEUP,		/* 115 */
		KEY_CAMERA,		/* 212 */
		KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp5_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp5_btn_keys[0],
    .size = ARRAY_SIZE(cyttsp5_btn_keys),
	.tag = 0,
};

static struct cyttsp5_core_platform_data _cyttsp5_core_platform_data = {
	.irq_gpio = CYTTSP5_I2C_IRQ_GPIO,
	.rst_gpio = CYTTSP5_I2C_RST_GPIO,
	.hid_desc_register = CYTTSP5_HID_DESC_REGISTER,
	.xres = cyttsp5_xres,
	.init = cyttsp5_init,
	.power = cyttsp5_power,
	.detect = cyttsp5_detect,
	.irq_stat = cyttsp5_irq_stat,
    .sett = {
     			NULL,	/* Reserved */
				NULL,	/* Command Registers */
				NULL,	/* Touch Report */
				NULL,	/* Cypress Data Record */
				NULL,	/* Test Record */
				NULL,	/* Panel Configuration Record */
				NULL,	/* &cyttsp5_sett_param_regs, */
				NULL,	/* &cyttsp5_sett_param_size, */
				NULL,	/* Reserved */
				NULL,	/* Reserved */
				NULL,	/* Operational Configuration Record */
				NULL, /* &cyttsp5_sett_ddata, *//* Design Data Record */
				NULL, /* &cyttsp5_sett_mdata, *//* Manufacturing Data Record */
				NULL,	/* Config and Test Registers */
				&cyttsp5_sett_btn_keys,	/* button-to-keycode table */
			},
	.flags = CY_CORE_FLAG_WAKE_ON_GESTURE
			 | CY_CORE_FLAG_RESTORE_PARAMETERS,
	.easy_wakeup_gesture = CY_CORE_EWG_NONE,
};

static const int16_t cyttsp5_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -127, 127, 0, 0,
	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
	ABS_MT_DISTANCE, 0, 255, 0, 0,	/* Used with hover */
};

struct touch_framework cyttsp5_framework = {
	.abs = (uint16_t *)&cyttsp5_abs[0],
	.size = ARRAY_SIZE(cyttsp5_abs),
	.enable_vkeys = 0,
};

static struct cyttsp5_mt_platform_data _cyttsp5_mt_platform_data = {
	.frmwrk = &cyttsp5_framework,
	.flags = CY_MT_FLAG_INV_X | CY_MT_FLAG_INV_Y,
	.inp_dev_name = CYTTSP5_MT_NAME,
	.vkeys_x = CY_VKEYS_X,
	.vkeys_y = CY_VKEYS_Y,
};

static struct cyttsp5_btn_platform_data _cyttsp5_btn_platform_data = {
	.inp_dev_name = CYTTSP5_BTN_NAME,
};

static const int16_t cyttsp5_prox_abs[] = {
	ABS_DISTANCE, CY_PROXIMITY_MIN_VAL, CY_PROXIMITY_MAX_VAL, 0, 0,
};

struct touch_framework cyttsp5_prox_framework = {
   .abs = (uint16_t *)&cyttsp5_prox_abs[0],
   .size = ARRAY_SIZE(cyttsp5_prox_abs),
};

static struct cyttsp5_proximity_platform_data
	_cyttsp5_proximity_platform_data = {
		.frmwrk = &cyttsp5_prox_framework,
		.inp_dev_name = CYTTSP5_PROXIMITY_NAME,
};

static struct cyttsp5_platform_data _cyttsp5_platform_data = {
	.core_pdata = &_cyttsp5_core_platform_data,
	.mt_pdata = &_cyttsp5_mt_platform_data,
	.loader_pdata = &_cyttsp5_loader_platform_data,
	.btn_pdata = &_cyttsp5_btn_platform_data,
	.prox_pdata = &_cyttsp5_proximity_platform_data,
};

static ssize_t cyttsp5_virtualkeys_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
		return sprintf(buf,
				__stringify(EV_KEY) ":"
				__stringify(KEY_BACK) ":1360:90:160:180"
			":" __stringify(EV_KEY) ":"
				__stringify(KEY_MENU) ":1360:270:160:180"
			":" __stringify(EV_KEY) ":"
				__stringify(KEY_HOMEPAGE) ":1360:450:160:180"
			":" __stringify(EV_KEY) ":"
				__stringify(KEY_SEARCH) ":1360:630:160:180"
			"\n");
}

static struct kobj_attribute cyttsp5_virtualkeys_attr = {
		.attr = {
					.name = "virtualkeys.cyttsp5_mt",
					.mode = S_IRUGO,
				},
		.show = &cyttsp5_virtualkeys_show,
};

static struct attribute *cyttsp5_properties_attrs[] = {
		&cyttsp5_virtualkeys_attr.attr,
		NULL
};

static struct attribute_group cyttsp5_properties_attr_group = {
		.attrs = cyttsp5_properties_attrs,
};
#endif /* !CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT */


static void __init omap4_panda_cyttsp5_init(void)
#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
{
	struct kobject *properties_kobj;
	int ret = 0;

	/* Initialize muxes for GPIO pins */
#ifdef CYTTSP5_USE_I2C
;//for ok335xd	omap_mux_init_gpio(CYTTSP5_I2C_RST_GPIO, OMAP_PIN_OUTPUT);
//for ok335xd	omap_mux_init_gpio(CYTTSP5_I2C_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif
#ifdef CYTTSP5_USE_SPI
;//for ok335xd	omap_mux_init_gpio(CYTTSP5_SPI_RST_GPIO, OMAP_PIN_OUTPUT);
//for ok335xd	omap_mux_init_gpio(CYTTSP5_SPI_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
					             &cyttsp5_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("%s: failed to create board_properties\n", __func__);
}
#else /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT */
{
#ifdef CYTTSP5_USE_I2C
;//for ok335xd		omap_mux_init_gpio(CYTTSP5_I2C_RST_GPIO, OMAP_PIN_OUTPUT);
//for ok335xd		omap_mux_init_gpio(CYTTSP5_I2C_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif
#ifdef CYTTSP5_USE_SPI
;//for ok335xd		omap_mux_init_gpio(CYTTSP5_SPI_RST_GPIO, OMAP_PIN_OUTPUT);
//for ok335xd		omap_mux_init_gpio(CYTTSP5_SPI_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT */

#ifdef CYTTSP5_USE_SPI
/*
 * This dummy structure is required, otherwise OMAP2 MCSPI driver
 * (assuming spi_board_info has a valid controller_data that is mapped to
 * omap2_mcspi_device_config) will cause a NULL pointer exception.
 */
static struct omap2_mcspi_device_config cyttsp5_mcspi_config = {
};
#endif

/*cypress5*/

static struct tps65217_board beaglebone_tps65217_info = {
	.tps65217_init_data = &tps65217_regulator_data[0],
	.bl_pdata = bone_lcd3_bl_pdata,
	.status_off = true,
};



/*ʹ\D3õ\E7\D7\E8\C6\C1ʱע\B2\E1*/
static struct i2c_board_info am335x_i2c1_boardinfo1[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
};

/*ʹ\D3õ\E7\C8\DD\C6\C1ʱע\B2\E1*/
static struct i2c_board_info am335x_i2c1_boardinfo2[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
	{
		I2C_BOARD_INFO("ft5x0x_ts", 0x38),
	},

	{
		I2C_BOARD_INFO("Goodix-TS", 0x14),
	},
#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
#ifdef CYTTSP5_USE_I2C
        {
                I2C_BOARD_INFO(CYTTSP5_I2C_NAME, CYTTSP5_I2C_TCH_ADR),
                .irq =  OMAP_GPIO_IRQ(CYTTSP5_I2C_IRQ_GPIO),
                .platform_data = &_cyttsp5_platform_data,
        },
#endif
#endif
};

static void i2c_init(int evm_id, int profile)
{
	setup_pin_mux(i2c1_pin_mux);
	#ifndef	CONFIG_ANDROID
		if(screen_type == SCREEN_TYPE_R)
			omap_register_i2c_bus(2, 100, am335x_i2c1_boardinfo1,ARRAY_SIZE(am335x_i2c1_boardinfo1));
		else if(screen_type == SCREEN_TYPE_C)
			omap_register_i2c_bus(2, 100, am335x_i2c1_boardinfo2,ARRAY_SIZE(am335x_i2c1_boardinfo2));
	#else
		  	omap_register_i2c_bus(2, 100, am335x_i2c1_boardinfo2,ARRAY_SIZE(am335x_i2c1_boardinfo2));
	#endif
	return;
}
static void sound_init(int evm_id, int profile)
{
	/* Configure McASP */
	setup_pin_mux(mcasp1_pin_mux);
	switch (evm_id) {
	case EVM_SK:
		am335x_register_mcasp(&am335x_evm_sk_snd_data1, 1);
		break;
	default:
		am335x_register_mcasp(&am335x_evm_snd_data1, 1);
	}

	return;
}

static void mmc1_rtl8189eus_init(int evm_id, int profile)
{
	setup_pin_mux(mmc1_common_pin_mux);
	if(gpio_request(GPIO_TO_PIN(2,1),"gpio2_1") < 0){
		return;
	}
	gpio_direction_output(GPIO_TO_PIN(2,1),1);
	gpio_set_value(GPIO_TO_PIN(2,1),1);
	am335x_mmc[1].mmc = 2;
	am335x_mmc[1].name = "rtl8189es";
	am335x_mmc[1].caps = MMC_CAP_4_BIT_DATA | MMC_CAP_NONREMOVABLE | MMC_CAP_SD_HIGHSPEED;
	am335x_mmc[1].nonremovable = true;
	am335x_mmc[1].gpio_cd = -EINVAL;
	am335x_mmc[1].gpio_wp = -EINVAL;
	am335x_mmc[1].ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */
}
#ifdef CONFIG_TI_ST
/* TI-ST for WL12xx BT */

/* Bluetooth Enable PAD for EVM Rev 1.1 and up */
#define AM33XX_CONTROL_PADCONF_MCASP0_AHCLKX_OFFSET		0x09AC

/* Bluetooth Enable PAD for EVM Rev 1.0 */
#define AM33XX_CONTROL_PADCONF_GPMC_CSN2_OFFSET			0x0884

int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}

int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	/* Configure BT_EN pin so that suspend/resume works correctly on rev 1.1 */
	selected_pad = AM33XX_CONTROL_PADCONF_MCASP0_AHCLKX_OFFSET;
	/* Configure BT_EN pin so that suspend/resume works correctly on rev 1.0 */
	/*selected_pad = AM33XX_CONTROL_PADCONF_GPMC_CSN2_OFFSET;*/

	gpio_direction_output(kim_data->nshutdown, 0);
	msleep(1);
	gpio_direction_output(kim_data->nshutdown, 1);

	/* Enable pullup on the enable pin for keeping BT active during suspend */
	pad_mux_value = readl(AM33XX_CTRL_REGADDR(selected_pad));
	pad_mux_value &= (~AM33XX_PULL_DISA);
	writel(pad_mux_value, AM33XX_CTRL_REGADDR(selected_pad));

	return 0;
}

int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	gpio_direction_output(kim_data->nshutdown, 0);

	/* Disable pullup on the enable pin to allow BT shut down during suspend */
	pad_mux_value = readl(AM33XX_CTRL_REGADDR(selected_pad));
	pad_mux_value |= AM33XX_PULL_DISA;
	writel(pad_mux_value, AM33XX_CTRL_REGADDR(selected_pad));

	return 0;
}

struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = GPIO_TO_PIN(3, 21),
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};

static struct platform_device wl12xx_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

#ifdef CONFIG_MACH_AM335XEVM_WILINK8
static struct platform_device nfcwilink_device = {
        .name = "nfcwilink",
        .id = -1,
};
#endif

static inline void __init am335xevm_init_btwilink(void)
{
	pr_info("am335xevm: bt init\n");

	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
#ifdef CONFIG_MACH_AM335XEVM_WILINK8
	platform_device_register(&nfcwilink_device);
#endif
}
#endif



static void can_init(int evm_id, int profile)
{
	setup_pin_mux(d_can0_pin_mux);
	am33xx_d_can_init(0);
}

static void mmc_init(int evm_id, int profile)
{
	switch (evm_id) {
	case BEAGLE_BONE_A3:
	case BEAGLE_BONE_OLD:
	case EVM_SK:
	case BEAGLE_BONE_BLACK:
		setup_pin_mux(mmc0_common_pin_mux);
		setup_pin_mux(mmc0_cd_only_pin_mux);
		break;
	default:
		setup_pin_mux(mmc0_common_pin_mux);
		setup_pin_mux(mmc0_cd_only_pin_mux);
		setup_pin_mux(mmc0_wp_only_pin_mux);
		break;
	}

	omap2_hsmmc_init(am335x_mmc);
	return;
}

static struct i2c_board_info i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65217", TPS65217_I2C_ID),
		.platform_data  = &beaglebone_tps65217_info,
	},
	{
		  I2C_BOARD_INFO("rx8010", 0x32),
	},
        {
		  I2C_BOARD_INFO("ds1307", 0x68),
	},
   {
            I2C_BOARD_INFO("24c256", 0x50),
   },
};


/* Configure GPIOs for GPIO Keys */
static struct gpio_keys_button am335x_evm_gpio_buttons_forlinx[] = {
	{
		.code                   = BTN_0,
		.gpio                   = GPIO_TO_PIN(1, 20),
		.desc                   = "SW1",
	},
	{
		.code                   = BTN_1,
		.gpio                   = GPIO_TO_PIN(1, 21),
		.desc                   = "SW2",
	},
	{
		.code                   = BTN_2,
		.gpio                   = GPIO_TO_PIN(1, 22),
		.desc                   = "SW3",
		.wakeup                 = 1,
	},
	{
		.code                   = BTN_3,
		.gpio                   = GPIO_TO_PIN(1, 23),
		.desc                   = "SW4",
	},
	{
		.code                   = BTN_4,
		.gpio                   = GPIO_TO_PIN(1, 24),
		.desc                   = "SW5",
		.wakeup                 = 1,
	},
	{
		.code                   = BTN_5,
		.gpio                   = GPIO_TO_PIN(1, 25),
		.desc                   = "SW6",
	},

};

static struct gpio_keys_platform_data am335x_evm_gpio_key_info_forlinx = {
	.buttons        = am335x_evm_gpio_buttons_forlinx,
	.nbuttons       = ARRAY_SIZE(am335x_evm_gpio_buttons_forlinx),
};

static struct platform_device am335x_evm_gpio_keys_forlinx = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &am335x_evm_gpio_key_info_forlinx,
	},
};

static void keys_init(int evm_id, int profile)
{
	int err;
	setup_pin_mux(gpio_keys_pin_mux_forlinx);
	err = platform_device_register(&am335x_evm_gpio_keys_forlinx);
    if (err)
          pr_err("failed to register gpio key device\n");
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "usr0",
		.gpio			= GPIO_TO_PIN(1, 16),	/* D1 */
		.active_low		= 1,
	},
	{
		.name			= "usr1",
		.gpio			= GPIO_TO_PIN(1, 17),	/* D2 */
		.active_low		= 1,
	},
	{
		.name			= "usr2",
		.gpio			= GPIO_TO_PIN(1, 18),	/* D3 */
		.active_low		= 1,
	},
	{
		.name			= "usr3",
		.gpio			= GPIO_TO_PIN(1, 19),	/* D4 */
		.active_low		= 1,
	},
	{
		.name			= "heartbeat",
		.gpio			= GPIO_TO_PIN(3, 8),	/* D4 */
		.default_trigger	= "heartbeat",
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static void led_init(int evm_id, int profile)
{
	int err;
	
	setup_pin_mux(gpio_led_mux);
	err = platform_device_register(&leds_gpio);
	if (err)
		pr_err("failed to register gpio led device\n");
}

void mini_gpio_init(int evm_id, int profile)
{
	setup_pin_mux(mini_gpio_mux);	
}
static struct evm_dev_cfg mini_dev_cfg[] = {
		    {mini_gpio_init,DEV_ON_BASEBOARD,PROFILE_ALL},
			{NULL, 0, 0},
};
/*335s2 test*/
void mini_setup(void)
{
	_configure_device(EVM_SK, mini_dev_cfg, PROFILE_NONE);
}

/* setup spi1 */
static void spi_init(int evm_id, int profile)
{
	setup_pin_mux(spi1_pin_mux);
	spi_register_board_info(am335x_spi1_slave_info,ARRAY_SIZE(am335x_spi1_slave_info));
	return;
}

static void sgx_init(int evm_id, int profile)
{
	if (omap3_has_sgx()) {
		am33xx_gpu_init();
	}
}

static int __init screentype_setup(char *str)
{
	if(!strncmp(str,"C",1))
		screen_type = SCREEN_TYPE_C;
	else 
		screen_type = SCREEN_TYPE_R;
	
	return 1;
}
__setup("screentype=", screentype_setup);

static int __init screensize_setup(char *str)
{
	if(!strncmp(str,"800x600-8",9))
		screen_size = SCREEN_SIZE_800X600_8;
	else if(!strncmp(str,"800x480-5",9))
		screen_size = SCREEN_SIZE_800X480_5;
	else if(!strncmp(str,"800x480-7",9))
		screen_size = SCREEN_SIZE_800X480_7;
	else if(!strncmp(str,"1024x600-7",10))
		screen_size = SCREEN_SIZE_1024X600_7;
	else if(!strncmp(str,"800x600-10",10))
		screen_size = SCREEN_SIZE_800X600_10;
	else if(!strncmp(str,"480x272-4",9))
		screen_size = SCREEN_SIZE_480X272_4;
	else if(!strncmp(str,"1280x720-vga",12))
		screen_size = SCREEN_SIZE_1280X720_vga;
	else if(!strncmp(str,"1024x768-vga",12))
		screen_size = SCREEN_SIZE_1024X768_vga;
	else if(!strncmp(str,"320x240-3",12))
		screen_size = SCREEN_SIZE_320X240_3;
	else  if(!strncmp(str,"640x480-5",12))
                screen_size = SCREEN_SIZE_640X480_5;
	else  if(!strncmp(str,"1280x800-10",10))
               screen_size = SCREEN_SIZE_1280X800_10;
        else
		screen_size = SCREEN_SIZE_800X600_vga;
	return 1;
}
__setup("screensize=", screensize_setup);


#define procfs_name "boardname"

int proc_read(char *buffer,char **buffer_location,off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	
	if (offset > 0) {
	/* we have finished to read, return 0 */
	ret = 0;
	} else {
	/* fill the buffer, return the buffer size */
#if defined(CONFIG_OK335XD)	
	ret = sprintf(buffer, "OK335xD");
#elif defined(CONFIG_OK335XS)
	ret = sprintf(buffer, "OK335xS");
#elif defined(CONFIG_OK335XS2)
	ret = sprintf(buffer, "OK335xS2");
#endif
	}

	return ret;
}

int proc_init(void)
{
	struct proc_dir_entry *Our_Proc_File;

	Our_Proc_File = create_proc_entry(procfs_name, 0644, NULL);

	if (Our_Proc_File == NULL) {
		remove_proc_entry(procfs_name, NULL);
		printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
		procfs_name);
		return -ENOMEM;
	}

	Our_Proc_File->read_proc = proc_read;
	//Our_Proc_File->owner = THIS_MODULE;
	Our_Proc_File->mode = S_IFREG | S_IRUGO;
	Our_Proc_File->uid = 0;
	Our_Proc_File->gid = 0;
	Our_Proc_File->size = 37;

	return 0; /* everything is ok */
}

/*\B5\E7\D7\E8\C6\C1\C9豸ע\B2\E1*/
static struct evm_dev_cfg mfd_dev_cfg[] = {
	{tscadc_init,		 DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

/* AD\C9豸ע\B2\E1*/
static struct evm_dev_cfg ad_dev_cfg[] = {
	{adc_init,		 DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};


/*SP706P wdt*/
static struct pinmux_config gpio_sp706p_wdt_pin_mux[] = {
		    {"emu0.gpio3_7", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
			//  {"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
			    {NULL, 0},
};

static void sp706p_init(int evm_id, int profile)
{
		    setup_pin_mux(gpio_sp706p_wdt_pin_mux);
}
/* ok335x*/
static struct evm_dev_cfg ok335x_dev_cfg[] = {
	{mmc1_rtl8189eus_init,DEV_ON_BASEBOARD,PROFILE_ALL},
	{mmc_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	#if defined(CONFIG_ANDROID)
	{tscadc_init,DEV_ON_BASEBOARD, PROFILE_ALL},
	#endif
	{net_init , DEV_ON_BASEBOARD, PROFILE_ALL},
	{lcd_init , DEV_ON_BASEBOARD, PROFILE_ALL},
	{i2c_init , DEV_ON_BASEBOARD, PROFILE_ALL},
	{ecap_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{keys_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{led_init , DEV_ON_BASEBOARD, PROFILE_ALL},
	{usb_init , DEV_ON_BASEBOARD, PROFILE_ALL},
	{nand_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{uart_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{spi_init , DEV_ON_BASEBOARD, PROFILE_ALL},
	{can_init , DEV_ON_BASEBOARD, PROFILE_ALL},
	{sound_init,DEV_ON_BASEBOARD, PROFILE_ALL},

	{buzzer_init,DEV_ON_BASEBOARD, PROFILE_ALL},
#ifdef CONFIG_SP706P_WDT
	{sp706p_init,DEV_ON_BASEBOARD, PROFILE_ALL},
#endif

	{sgx_init , DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

static int am33xx_evm_tx_clk_dly_phy_fixup(struct phy_device *phydev)
{
	phy_write(phydev, AR8051_PHY_DEBUG_ADDR_REG,
		  AR8051_DEBUG_RGMII_CLK_DLY_REG);
	phy_write(phydev, AR8051_PHY_DEBUG_DATA_REG, AR8051_RGMII_TX_CLK_DLY);
	return 0;
}

#define AM33XX_VDD_CORE_OPP50_UV		1100000
#define AM33XX_OPP120_FREQ		600000000
#define AM33XX_OPPTURBO_FREQ		720000000

#define AM33XX_ES2_0_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_0_OPP120_FREQ	720000000
#define AM33XX_ES2_0_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_0_OPPNITRO_FREQ	1000000000

#define AM33XX_ES2_1_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_1_OPP120_FREQ	720000000
#define AM33XX_ES2_1_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_1_OPPNITRO_FREQ	1000000000
#ifdef CONFIG_MACH_AM335XEVM_VIBRATOR
int am335xevm_vibrator_init(void);
#endif

/* ok335x */
static void setup_ok335x(void)
{
	/*which doesn't have Write Protect pin */
	am335x_mmc[0].gpio_wp = -EINVAL;

	_configure_device(EVM_SK, ok335x_dev_cfg, PROFILE_NONE);
	am33xx_cpsw_init(AM33XX_CPSW_MODE_RGMII, NULL, NULL);
	/* Atheros Tx Clk delay Phy fixup */
	phy_register_fixup_for_uid(AM335X_EVM_PHY_ID, AM335X_EVM_PHY_MASK,
				   am33xx_evm_tx_clk_dly_phy_fixup);
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */
	.mode           = (MUSB_HOST << 4) | MUSB_OTG,
	.power		= 500,
	.instances	= 1,
};


static void __init am335x_evm_i2c_init(void)
{
	/* Initially assume General Purpose EVM Config */
	am335x_evm_id = EVM_SK;
	omap_register_i2c_bus(1, 100, i2c0_boardinfo,ARRAY_SIZE(i2c0_boardinfo));
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
	am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

	return am33xx_gpio0_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

static void __init am335x_evm_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	omap4_panda_cyttsp5_init();
	am335x_evm_i2c_init();
	omap_sdrc_init(NULL, NULL);
	usb_musb_init(&musb_board_data);
	setup_pin_mux(tsc_gpio_pin_mux);
	
	omap_board_config = am335x_evm_config;
	omap_board_config_size = ARRAY_SIZE(am335x_evm_config);

	/*\B8\F9\BE\DD\C4ں˴\AB\B2ξ\F6\B6\A8\CAǷ\F1ʹ\D3õ\E7\D7\E8\C6\C1*/
#ifndef CONFIG_ANDROID
	if(screen_type == SCREEN_TYPE_R)
		_configure_device(EVM_SK, mfd_dev_cfg, PROFILE_NONE);//\B5\E7\D7\E8\C6\C1\BA\CDadһ\C6\F0ע\B2\E1
	else
		_configure_device(EVM_SK, ad_dev_cfg, PROFILE_NONE);//\CF\D6\D4\DAʹ\D3õ\E7\C8\DD\C6\C1\A3\AC\BD\F6ע\B2\E1ad
#endif

	/*main setup*/
	setup_ok335x();

	/*create  /proc/boardname to export info to userspace*/
	proc_init();

	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
}

static void __init am335x_evm_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(AM335XEVM, "am335xevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END

MACHINE_START(AM335XIAEVM, "am335xiaevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_irq	= ti81xx_init_irq,
	.init_early	= am33xx_init_early,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END
