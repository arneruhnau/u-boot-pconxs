/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <asm/arch/crm_regs.h>

#include <linux/fb.h>
#include "../drivers/video/mxcfb.h"
#include <ipu_pixfmt.h>

DECLARE_GLOBAL_DATA_PTR;

#define CLKCTL_CCGR2 0x70

static u32 system_rev;

static void display_init(void);
static void usbotg_init(void);

#ifdef CONFIG_CMD_I2C
static void setup_iomux_i2c(unsigned int);
#endif
#ifdef CONFIG_FEC_MXC
static void setup_iomux_enet(void);
extern void trizeps7sdl_IomuxConfig(void);
#endif
static void setup_iomux_asrc(void);
static void setup_iomux_audmux(void);
static void setup_iomux_gpio(void);
static void setup_iomux_hdmi(void);
#ifdef CONFIG_VIDEO_IPUV3
static void setup_iomux_ipu1(void);
#endif
static void setup_iomux_mlb(void);
static void setup_iomux_pwm(void);
static void setup_iomux_weim(void);
static void keep_power_supply_alive(void);


#ifdef CONFIG_VIDEO_IPUV3
static void enable_lvds(struct display_info_t const* dev);
#endif

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);
	return 0;
}

#define UART_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT0__UART1_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT1__UART1_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};
iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_SD3_DAT5__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT4__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_CMD__UART2_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_CLK__UART2_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#ifdef CONFIG_FSL_ESDHC

#define USDHC_PAD_CTRL (\
	PAD_CTL_PUS_47K_UP | \
	PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_80ohm | \
	PAD_CTL_SRE_FAST  | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__SD1_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__SD1_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__SD1_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__SD1_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__SD1_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__SD1_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_KEY_COL2__GPIO4_IO10 	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO_2__GPIO1_IO02 	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_ALE__SD4_RESET	| MUX_PAD_CTRL(USDHC_PAD_CTRL)
};

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for(index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
			case 0:
				imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
#define TRIZEPS7_WLAN_SHUTDOWN IMX_GPIO_NR(4, 10)
#define TRIZEPS7_WLAN_RESET IMX_GPIO_NR(1, 2)
				gpio_direction_output(TRIZEPS7_WLAN_RESET, 0);
				udelay(10);
				gpio_direction_output(TRIZEPS7_WLAN_SHUTDOWN, 0);
				udelay(20);
				gpio_direction_output(TRIZEPS7_WLAN_SHUTDOWN, 1);
				udelay(10);
				gpio_direction_output(TRIZEPS7_WLAN_RESET, 1);
				break;
			case 1:
				imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
				break;	
			default:
				printf("Warning: you configured more USDHC controllers"
				       "(%d) than supported by the board (%d)\n",
				       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
				return status;
		}
		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		
	}
	return status;
}

#endif

#if defined(CONFIG_FEC_MXC)
int board_eth_init(bd_t *bis)
{
	struct iomuxc_base_regs *const iomuxc_regs = (struct iomuxc_base_regs *)IOMUXC_BASE_ADDR;
	enable_fec_anatop_clock(ENET_50MHz);
	setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_ENET_CLK_SEL_MASK);
	setup_iomux_enet();
	return cpu_eth_init(bis);
}
#endif

u32 get_board_rev(void)
{
	system_rev = 0x63000;
	return system_rev;
}

static void display_init(void)
{
	imx_iomux_v3_setup_pad(MX6_PAD_DI0_PIN4__GPIO4_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL));
	imx_iomux_v3_setup_pad(MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL));

#ifndef CONFIG_VIDEO_IPUV3
#define IPAN5_BACKLIGHT_ENABLE IMX_GPIO_NR(4, 20)
#define IPAN5_DISPLAY_POWER IMX_GPIO_NR(5, 20)
	gpio_direction_output(IPAN5_DISPLAY_POWER, 0);
	gpio_direction_output(IPAN5_BACKLIGHT_ENABLE, 0);
#endif
}

int board_early_init_f(void)
{
	setup_iomux_asrc();
	setup_iomux_audmux();
	setup_iomux_gpio();
	setup_iomux_hdmi();
#ifdef CONFIG_VIDEO_IPUV3
	setup_iomux_ipu1();
#endif
	setup_iomux_mlb();
	setup_iomux_pwm();
	setup_iomux_weim();
	setup_iomux_uart();
#ifdef CONFIG_CMD_I2C
	setup_iomux_i2c(I2C1_BASE_ADDR);
	setup_iomux_i2c(I2C2_BASE_ADDR);
#endif
	keep_power_supply_alive();
	usbotg_init();
	display_init();
	return 0;
}

static void keep_power_supply_alive(void)
{
	#define TARGET_POWER_SUPPLY_PIN IMX_GPIO_NR(4, 14)
	gpio_direction_output(TARGET_POWER_SUPPLY_PIN, 0);
}

static void usbotg_init(void) {
#define USB_OTG_OC IMX_GPIO_NR(6, 10)
	gpio_direction_output(USB_OTG_OC, 1);
}

int board_init(void)
{
	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"esdhc2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{NULL,   0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

#ifdef CONFIG_FEC_MXC

#define ENET_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),                   /*                 GP1_22 */
	MX6_PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),                     /*                 GP1_31 */
	MX6_PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),                    /* /RESET          GP6_31 */
	MX6_PAD_ENET_RXD0__GPIO1_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL),                   /* MODE0=RXD0      GP1_27 */
	MX6_PAD_ENET_RXD1__GPIO1_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL),                   /* MODE1=RXD1      GP1_26 */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),                 /* MODE2=CSR_DV    GP1_25 */
	MX6_PAD_ENET_RX_ER__GPIO1_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),                  /* PHYAD0          GP1_24 */
	MX6_PAD_ENET_TX_EN__ENET_TX_EN | MUX_PAD_CTRL(NO_PAD_CTRL),                 /* TX_EN           GP1_28 */ 
	MX6_PAD_ENET_TXD1__ENET_TX_DATA1 | MUX_PAD_CTRL(NO_PAD_CTRL),                /* TXD1            GP1_29 */
	MX6_PAD_ENET_TXD0__ENET_TX_DATA0 | MUX_PAD_CTRL(NO_PAD_CTRL),                /* TXD0            GP1_30 */
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),        /* REFCTRL         GP6_26 */
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),                     /* GP17 \ENET1_INT GP7_12 */
};

iomux_v3_cfg_t enet_pads_final[] = {
	MX6_PAD_ENET_RXD0__ENET_RX_DATA0 | MUX_PAD_CTRL(NO_PAD_CTRL),                /* MODE0=RXD0      GP1_27 */
	MX6_PAD_ENET_RXD1__ENET_RX_DATA1 | MUX_PAD_CTRL(NO_PAD_CTRL),                /* MODE1=RXD1      GP1_26 */
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN | MUX_PAD_CTRL(NO_PAD_CTRL),                /* MODE2=CSR_DV    GP1_25 */
	MX6_PAD_ENET_RX_ER__ENET_RX_ER | MUX_PAD_CTRL(NO_PAD_CTRL),                 /* PHYAD0          GP1_24 */
	MX6_PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),                    /* VK. /RESET GP6_31 */
};

int mx6_rgmii_rework(char *devname, int phy_addr)
{
	return 0;
}

static void setup_iomux_enet(void)
{
	trizeps7sdl_IomuxConfig();
	imx_iomux_v3_setup_pad(MX6_PAD_GPIO_16__ENET_REF_CLK | MUX_PAD_CTRL(0xE1));

	imx_iomux_v3_setup_pad(NEW_PAD_CTRL(MX6_PAD_EIM_BCLK__GPIO6_IO31, 0x48));
#define ENET_RESET IMX_GPIO_NR(6, 31)
	gpio_direction_output(ENET_RESET, 1);
	gpio_direction_output(IMX_GPIO_NR(1, 24), 0); // PHY ADR=0
	gpio_direction_output(IMX_GPIO_NR(1, 25), 1); // Mode 0
	gpio_direction_output(IMX_GPIO_NR(1, 26), 1); // Mode 1
	gpio_direction_output(IMX_GPIO_NR(1, 27), 1); // Mode 2
	gpio_direction_output(ENET_RESET, 0);
	gpio_direction_output(IMX_GPIO_NR(7, 12), 1);
	udelay(50);
	gpio_direction_output(ENET_RESET, 1);
	udelay(10);
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	udelay(500);
	imx_iomux_v3_setup_multiple_pads(enet_pads_final, ARRAY_SIZE(enet_pads_final));
	gpio_direction_input(IMX_GPIO_NR(7, 12));
}

#endif

#ifdef CONFIG_CMD_I2C

#define I2C_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_HYS | \
	PAD_CTL_ODE | \
	PAD_CTL_SRE_FAST \
)

static void setup_iomux_i2c(unsigned int module_base) {
	switch (module_base) {
		case I2C1_BASE_ADDR:
			imx_iomux_v3_setup_pad(MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL));
			imx_iomux_v3_setup_pad(MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL));
			break;
		case I2C2_BASE_ADDR:
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL));
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL));
			break;
		default:
			break;
	}
}
#endif

int checkboard(void)
{
	printf("Board: MX6DL-TRIZEPS7\n");
	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
int display_count = 1;
struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect = NULL,
	.enable = &enable_lvds,
	.mode	= {
		.name		= "EDT-WVGA",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 30000,
		.left_margin	= 40,
		.right_margin	= 40,
		.upper_margin	= 29, /* fixed */
		.lower_margin	= 13,
		.hsync_len	= 48, /* hsync_len + right_margin == 88 */
		.vsync_len	= 3,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED | FB_SYNC_SWAP_RGB,
		.flag		= 0
	}
}};

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
}

#endif

int overwrite_console(void)
{
	return 1;
}

#define ASRC_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

static void setup_iomux_asrc() {
	imx_iomux_v3_setup_pad(MX6_PAD_GPIO_0__ASRC_EXT_CLK | MUX_PAD_CTRL(ASRC_PAD_CTRL));
}

#define AUDMUX_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const audmux_pads[] = {
	MX6_PAD_CSI0_DAT7__AUD3_RXD | MUX_PAD_CTRL(AUDMUX_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__AUD3_TXD | MUX_PAD_CTRL(AUDMUX_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__AUD3_TXFS | MUX_PAD_CTRL(AUDMUX_PAD_CTRL)
};

static void setup_iomux_audmux() {
	imx_iomux_v3_setup_multiple_pads(audmux_pads, ARRAY_SIZE(audmux_pads));
}

#define GPIO_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const gpio_pads[] = {
	MX6_PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO_8__GPIO1_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_KEY_COL4__GPIO4_IO14 |MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_KEY_ROW4__GPIO4_IO15 |MUX_PAD_CTRL(NO_PAD_CTRL),

	MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(GPIO_PAD_CTRL),

	MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_D1__GPIO2_IO01 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_D2__GPIO2_IO02 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_D3__GPIO2_IO03 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_D4__GPIO2_IO04 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_D6__GPIO2_IO06 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_D7__GPIO2_IO07 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_LBA__GPIO2_IO27 | MUX_PAD_CTRL(GPIO_PAD_CTRL),

	MX6_PAD_GPIO_19__GPIO4_IO05 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_KEY_COL1__GPIO4_IO08 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_KEY_COL2__GPIO4_IO10 | MUX_PAD_CTRL(GPIO_PAD_CTRL),

	MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__GPIO5_IO29 | MUX_PAD_CTRL(GPIO_PAD_CTRL),

	MX6_PAD_NANDF_CLE__GPIO6_IO07 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_WP_B__GPIO6_IO09 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_RB0__GPIO6_IO10 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_CS0__GPIO6_IO11 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_CS1__GPIO6_IO14 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_CS2__GPIO6_IO15 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_CS3__GPIO6_IO16 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(GPIO_PAD_CTRL),

	MX6_PAD_SD3_DAT2__GPIO7_IO06 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD3_DAT3__GPIO7_IO07 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_gpio() {
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));
}

#define HDMI_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const hdmi_pads[] = {
	MX6_PAD_KEY_ROW2__HDMI_TX_CEC_LINE | MUX_PAD_CTRL(HDMI_PAD_CTRL),
};

static void setup_iomux_hdmi() {
	imx_iomux_v3_setup_multiple_pads(hdmi_pads, ARRAY_SIZE(hdmi_pads));
}

#ifdef CONFIG_VIDEO_IPUV3

#define IPU1_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const ipu1_pads[] = {

	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_DAT12__IPU1_CSI0_DATA12 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_DAT13__IPU1_CSI0_DATA13 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_DAT14__IPU1_CSI0_DATA14 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_DAT15__IPU1_CSI0_DATA15 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_DAT16__IPU1_CSI0_DATA16 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_DAT17__IPU1_CSI0_DATA17 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_DAT18__IPU1_CSI0_DATA18 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_DAT19__IPU1_CSI0_DATA19 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_PIN4__IPU1_DI0_PIN04 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
};

static void setup_iomux_ipu1() {
	imx_iomux_v3_setup_multiple_pads(ipu1_pads, ARRAY_SIZE(ipu1_pads));
}
#endif
#define MLB_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const mlb_pads[] = {
	MX6_PAD_GPIO_3__MLB_CLK | MUX_PAD_CTRL(MLB_PAD_CTRL),
	MX6_PAD_GPIO_2__MLB_DATA | MUX_PAD_CTRL(MLB_PAD_CTRL),
	MX6_PAD_GPIO_6__MLB_SIG | MUX_PAD_CTRL(MLB_PAD_CTRL),
};

static void setup_iomux_mlb() {
	imx_iomux_v3_setup_multiple_pads(mlb_pads, ARRAY_SIZE(mlb_pads));
}

#define PWM_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const pwm_pads[] = {
	MX6_PAD_GPIO_9__PWM1_OUT | MUX_PAD_CTRL(PWM_PAD_CTRL),
	MX6_PAD_GPIO_1__PWM2_OUT | MUX_PAD_CTRL(PWM_PAD_CTRL)
};

static void setup_iomux_pwm() {
	imx_iomux_v3_setup_multiple_pads(pwm_pads, ARRAY_SIZE(pwm_pads));
}

#define WEIM_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_FAST | \
	PAD_CTL_HYS \
)
#define WEIM_DATA_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)
#define WEIM_WAIT_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_60ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

iomux_v3_cfg_t const weim_pads[] = {
	MX6_PAD_EIM_A16__EIM_ADDR16 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A17__EIM_ADDR17 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A18__EIM_ADDR18 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A19__EIM_ADDR19 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A20__EIM_ADDR20 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A21__EIM_ADDR21 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A22__EIM_ADDR22 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A23__EIM_ADDR23 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A24__EIM_ADDR24 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_A25__EIM_ADDR25 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_CS0__EIM_CS0_B | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_CS1__EIM_CS1_B | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_D16__EIM_DATA16 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D17__EIM_DATA17 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D18__EIM_DATA18 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D19__EIM_DATA19 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D20__EIM_DATA20 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D21__EIM_DATA21 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D22__EIM_DATA22 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D23__EIM_DATA23 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D24__EIM_DATA24 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D25__EIM_DATA25 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D26__EIM_DATA26 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D27__EIM_DATA27 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D28__EIM_DATA28 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D29__EIM_DATA29 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D30__EIM_DATA30 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_D31__EIM_DATA31 | MUX_PAD_CTRL(WEIM_DATA_PAD_CTRL),
	MX6_PAD_EIM_DA0__EIM_AD00 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA1__EIM_AD01 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA2__EIM_AD02 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA3__EIM_AD03 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA4__EIM_AD04 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA5__EIM_AD05 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA6__EIM_AD06 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA7__EIM_AD07 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA8__EIM_AD08 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA9__EIM_AD09 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA10__EIM_AD10 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA11__EIM_AD11 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA12__EIM_AD12 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA13__EIM_AD13 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA14__EIM_AD14 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_DA15__EIM_AD15 | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_EB0__EIM_EB0_B | MUX_PAD_CTRL(WEIM_PAD_CTRL),
	MX6_PAD_EIM_WAIT__EIM_WAIT_B | MUX_PAD_CTRL(WEIM_WAIT_PAD_CTRL),
};

static void setup_iomux_weim() {
	imx_iomux_v3_setup_multiple_pads(weim_pads, ARRAY_SIZE(weim_pads));
}
