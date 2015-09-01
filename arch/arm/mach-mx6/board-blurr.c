/*
 * Copyright (C) 2013 EMSYM.com. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-blurr_mx6q.h"
#include "board-blurr_mx6dl.h"

#define BLURR_VOLUME_UP	        IMX_GPIO_NR(1, 4)
#define BLURR_VOLUME_DN	        IMX_GPIO_NR(1, 5)
#define BLURR_POWER_OFF	        IMX_GPIO_NR(3, 29)

#define BLURR_BT_RESET	        IMX_GPIO_NR(1, 2)
#define BLURR_USR_DEF_RED_LED	IMX_GPIO_NR(1, 2)

#define BLURR_CSI0_PWN	        IMX_GPIO_NR(1, 16)
#define BLURR_CSI0_RST	        IMX_GPIO_NR(1, 17)
#define BLURR_ACCL_INT	        IMX_GPIO_NR(1, 18)
#define BLURR_MIPICSI_PWN	IMX_GPIO_NR(1, 19)
#define BLURR_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define BLURR_RGMII_RST	        IMX_GPIO_NR(1, 25)
#define BLURR_RGMII_INT	        IMX_GPIO_NR(1, 26)

#define BLURR_USBH1_PWR_EN	IMX_GPIO_NR(1, 29)
#define BLURR_DISP0_PWR_EN	IMX_GPIO_NR(1, 30)



#define BLURR_GPS_RESET	        IMX_GPIO_NR(2, 28)

#define BLURR_GPS_EN	        IMX_GPIO_NR(3, 0)
#define BLURR_DISP0_RST_B	IMX_GPIO_NR(3, 8)

#define BLURR_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define BLURR_USB_H1_PWR	IMX_GPIO_NR(1, 29)

#define BLURR_DISP0_RD	        IMX_GPIO_NR(3, 28)

#define BLURR_CAN1_STBY	        IMX_GPIO_NR(4, 5)
#define BLURR_ECSPI1_CS0        IMX_GPIO_NR(4, 9)

#define BLURR_HDMI_CEC_IN	IMX_GPIO_NR(4, 11)
#define BLURR_PCIE_DIS_B	IMX_GPIO_NR(4, 14)

#define BLURR_DI0_D0_CS	        IMX_GPIO_NR(5, 0)
#define BLURR_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)
//touchscreen
#define BLURR_CAP_TCH_INT1	IMX_GPIO_NR(2, 23)
#define BLURR_CABC_EN1	        IMX_GPIO_NR(2, 25)
#define BLURR_CAP_TCH_INT0	IMX_GPIO_NR(3, 28)
#define BLURR_CABC_EN0	        IMX_GPIO_NR(3, 30)

#define BLURR_SD3_CD		IMX_GPIO_NR(7, 0)
#define BLURR_SD2_CD		IMX_GPIO_NR(6, 18)
#define BLURR_SD2_WP		IMX_GPIO_NR(6, 17)
#define BLURR_DISP0_WR_REVB	IMX_GPIO_NR(6, 9)

#define BLURR_DI1_D0_CS	        IMX_GPIO_NR(6, 31)

#define BLURR_HEADPHONE_DET	IMX_GPIO_NR(1, 9)
#define BLURR_PCIE_RST_B_REVB	IMX_GPIO_NR(7, 12)
#define BLURR_PMIC_INT_B	IMX_GPIO_NR(7, 13)
#define BLURR_PFUZE_INT	        IMX_GPIO_NR(7, 13)

#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9

static struct clk *sata_clk;
static struct clk *clko;
static int mma8451_position = 1;
static int mag3110_position = 2;
static int max11801_mode = 1;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;
extern bool enet_to_gpio_6;

static int max17135_regulator_init(struct max17135 *max17135);

static const struct esdhc_platform_data emsym_blurr_sd2_data __initconst = {
	.cd_gpio = BLURR_SD2_CD,
	.wp_gpio = BLURR_SD2_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data emsym_blurr_sd3_data __initconst = {
        .cd_gpio = BLURR_SD3_CD,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,//ESDHC_CD_PERMANENT
};

static int __init gpmi_nand_platform_init(void)
{
	iomux_v3_cfg_t *nand_pads = NULL;
	u32 nand_pads_cnt;

	if (cpu_is_mx6q()) {
		nand_pads = mx6q_blurr_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6q_blurr_gpmi_nand);
	} else if (cpu_is_mx6dl()) {
		nand_pads =mx6dl_blurr_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6dl_blurr_gpmi_nand);

	}
	BUG_ON(!nand_pads);
	return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
}

static struct gpmi_nand_platform_data
mx6q_gpmi_nand_platform_data __initdata = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          =4,
        .enable_bbt              =1,
        .enable_ddr              =0,
};
static int __init board_support_onfi_nand(char *p)
{
mx6q_gpmi_nand_platform_data.enable_ddr=1;
return 0;
}

early_param("onfi_support",board_support_onfi_nand);

static const struct anatop_thermal_platform_data
	emsym_blurr_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void emsym_blurr_init_uart(void)
{
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(0, NULL);
        imx6q_add_imx_uart(1, NULL);
        imx6q_add_imx_uart(3, NULL);
        imx6q_add_imx_uart(4, NULL);
}

static int emsym_blurr_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = emsym_blurr_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	.gpio_irq = MX6_ENET_IRQ,
};

static int emsym_blurr_spi_cs[] = {
	BLURR_ECSPI1_CS0,
};

static const struct spi_imx_master emsym_blurr_spi_data __initconst = {
	.chipselect     = emsym_blurr_spi_cs,
	.num_chipselect = ARRAY_SIZE(emsym_blurr_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition emsym_blurr_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data emsym_blurr__spi_flash_data = {
	.name = "m25p80",
	.parts = emsym_blurr_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(emsym_blurr_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info emsym_blurr_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &emsym_blurr__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(emsym_blurr_spi_nor_device,
				ARRAY_SIZE(emsym_blurr_spi_nor_device));
}


static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(BLURR_CSI0_PWN, 1);
	else
		gpio_set_value(BLURR_CSI0_PWN, 0);

	msleep(2);
}

static void mx6q_csi0_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_blurr_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_blurr_csi0_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_blurr_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_blurr_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(BLURR_CSI0_RST, "cam-reset");
	gpio_direction_output(BLURR_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(BLURR_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(BLURR_CSI0_PWN, 1);
	msleep(5);
	gpio_set_value(BLURR_CSI0_PWN, 0);
	msleep(5);
	gpio_set_value(BLURR_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(BLURR_CSI0_RST, 1);
	msleep(5);
	gpio_set_value(BLURR_CSI0_PWN, 1);

	/* For MX6Q:
	 * GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 *
	 * For MX6DL:
	 * GPR13 bit 0-2 IPU_CSI0_MUX
	 *   000 MIPI_CSI0
	 *   100 IPU CSI0
	 */
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

static void mx6q_mipi_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(BLURR_MIPICSI_PWN, 1);
	else
		gpio_set_value(BLURR_MIPICSI_PWN, 0);

	msleep(2);
}

static void mx6q_mipi_sensor_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_blurr_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_blurr_mipi_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_blurr_mipi_sensor_pads,
			ARRAY_SIZE(mx6dl_blurr_mipi_sensor_pads));

	/* Camera reset */
	gpio_request(BLURR_MIPICSI_RST, "cam-reset");
	gpio_direction_output(BLURR_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(BLURR_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(BLURR_MIPICSI_PWN, 1);
	msleep(5);
	gpio_set_value(BLURR_MIPICSI_PWN, 0);
	msleep(5);
	gpio_set_value(BLURR_MIPICSI_RST, 0);
	msleep(1);
	gpio_set_value(BLURR_MIPICSI_RST, 1);
	msleep(5);
	gpio_set_value(BLURR_MIPICSI_PWN, 1);

	/*for mx6dl, mipi virtual channel 1 connect to csi 1*/
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 3, 3, 1);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 1,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};


static struct imxi2c_platform_data emsym_blurr_i2c_data = {
	.bitrate = 100000,
};

static struct fsl_mxc_lightsensor_platform_data ls_data = {
	.rext = 499,	/* calibration: 499K->700K */
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
        {
                 I2C_BOARD_INFO("sgtl5000", 0x0a),
	 },
         {
                 I2C_BOARD_INFO("rtc-ds1307", 0x68),
                 .type="ds1307",
	 },

};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(BLURR_CAP_TCH_INT0),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(BLURR_CAP_TCH_INT1),
	},
 
	{
		I2C_BOARD_INFO("mxc_ldb_i2c", 0x50),
		.platform_data = (void *)1,	/* lvds port1 */
	},
};

static void emsym_blurr_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(BLURR_USB_OTG_PWR, 1);
	else
		gpio_set_value(BLURR_USB_OTG_PWR, 0);
}

static void emsym_blurr_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(BLURR_USB_H1_PWR, 1);
	else
		gpio_set_value(BLURR_USB_H1_PWR, 0);
}

static void __init emsym_blurr_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(BLURR_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO BLURR_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(BLURR_USB_OTG_PWR, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(BLURR_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO BLURR_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(BLURR_USB_H1_PWR, 0);
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(emsym_blurr_usbotg_vbus);
	mx6_set_host1_vbus_func(emsym_blurr_host1_vbus);

}

/* HW Initialization, if return 0, initialization is successful. */
static int emsym_blurr_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void emsym_blurr_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data emsym_blurr_sata_data = {
	.init = emsym_blurr_sata_init,
	.exit = emsym_blurr_sata_exit,
};
#endif

static void emsym_blurr_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(BLURR_CAN1_STBY, 1);
	} else {
		gpio_set_value(BLURR_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	emsym_blurr_flexcan0_pdata __initconst = {
	.transceiver_switch = emsym_blurr_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6_reset_mipi_dsi(void)
{
	//gpio_set_value(BLURR_DISP_PWR_EN, 1);
	//gpio_set_value(BLURR_DISP_RST_B, 1);
	udelay(10);
	//gpio_set_value(BLURR_DISP_RST_B, 0);
	udelay(50);
	//gpio_set_value(BLURR_DISP_RST_B, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data BLURR_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x BLURR board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_blurr_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_blurr_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_blurr_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_blurr_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_blurr_i2c2_pads,
			ARRAY_SIZE(mx6dl_blurr_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_blurr_i2c2_pads,
			ARRAY_SIZE(mx6q_blurr_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
	.phy_reg_vlev = 0x0294,
	.phy_reg_cksymtx = 0x800d,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};



static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};
static void blurr_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
	//gpio_set_value(SABRESD_AUX_5V_EN, 0);
}

static void blurr_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
	//gpio_set_value(SABRESD_AUX_5V_EN, 1);
}
static const struct pm_platform_data emsym_blurr_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = blurr_suspend_enter,
	.suspend_exit = blurr_suspend_exit,
};

static struct regulator_consumer_supply BLURR_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data BLURR_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(BLURR_vmmc_consumers),
	.consumer_supplies = BLURR_vmmc_consumers,
};

static struct fixed_voltage_config BLURR_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &BLURR_vmmc_init,
};

static struct platform_device BLURR_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &BLURR_vmmc_reg_config,
	},
};
/*****************audio*************************/
static struct imx_ssi_platform_data mx6_sabresd_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};



static struct platform_device sgtl5000_devices = {
	.name = "imx-sgtl5000",
};

static struct mxc_audio_platform_data sgtl5000_data;
static int mxc_sgtl5000_init(void)
{
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	/* both audio codec and comera use CLKO clk*/
	rate = clk_round_rate(clko, 24000000);
	clk_set_rate(clko, rate);
        
	sgtl5000_data.sysclk = rate;
        clk_enable(clko);
	return 0;
}
static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.init = mxc_sgtl5000_init,
	//.amp_enable = smd_sgtl5000_amp_enable,
	.hp_gpio =BLURR_HEADPHONE_DET,
	.hp_active_low = 0,
};
#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_sabrelite_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vdda,
};

static struct regulator_init_data sgtl5000_sabrelite_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddio,
};

static struct regulator_init_data sgtl5000_sabrelite_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		=3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 1800000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddd_reg_initdata,
};

static struct platform_device sgtl5000_sabrelite_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddd_reg_config,
	},
};
#endif /* CONFIG_SND_SOC_SGTL5000 */

static int __init imx6q_init_audio(void)
{

	mxc_register_device(&sgtl5000_devices,&sgtl5000_data);
	imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_sabrelite_vdda_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddio_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddd_reg_devices);
#endif
	return 0;
}
//**********************************************************************

#ifndef CONFIG_IMX_PCIE
static void pcie_3v3_power(void)
{
	/* disable PCIE_3V3 first */
	//gpio_request(BLURR_PCIE_PWR_EN, "pcie_3v3_en");
	//gpio_direction_output(BLURR_PCIE_PWR_EN, 0);
	//mdelay(10);
	/* enable PCIE_3V3 again */
	//gpio_set_value(BLURR_PCIE_PWR_EN, 1);
	//gpio_free(BLURR_PCIE_PWR_EN);
}

static void pcie_3v3_reset(void)
{
	/* reset miniPCIe */
	gpio_request(BLURR_PCIE_RST_B_REVB, "pcie_reset_rebB");
	gpio_direction_output(BLURR_PCIE_RST_B_REVB, 0);
	/* The PCI Express Mini CEM specification states that PREST# is
	deasserted minimum 1ms after 3.3vVaux has been applied and stable*/
	mdelay(1);
	gpio_set_value(BLURR_PCIE_RST_B_REVB, 1);
	gpio_free(BLURR_PCIE_RST_B_REVB);
}
#endif




#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button imx6q_buttons[] = {
	GPIO_BUTTON(BLURR_VOLUME_UP, KEY_VOLUMEUP, 1, "volume-up", 0, 1),
	GPIO_BUTTON(BLURR_VOLUME_DN, KEY_VOLUMEDOWN, 1, "volume-down", 0, 1),
	GPIO_BUTTON(BLURR_POWER_OFF, KEY_POWER, 1, "power", 1, 1),
};

static struct gpio_keys_platform_data imx6q_button_data = {
	.buttons	= imx6q_buttons,
	.nbuttons	= ARRAY_SIZE(imx6q_buttons),
};

static struct platform_device imx6q_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_button_data,
	}
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&imx6q_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct platform_pwm_backlight_data emsym_blurr_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};
static struct platform_pwm_backlight_data emsym_blurr_pwm_backlight_data1 = {
	.pwm_id = 1,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};
static struct mxc_dvfs_platform_data BLURR_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data emsym_blurr_pcie_data __initconst = {
	//.pcie_pwr_en	= BLURR_PCIE_PWR_EN,
	.pcie_rst	= BLURR_PCIE_RST_B_REVB,
	.pcie_wake_up	= BLURR_PCIE_WAKE_B,
	.pcie_dis	= BLURR_PCIE_DIS_B,
#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
	.type_ep	= 1,
#else
	.type_ep	= 0,
#endif
};

/*!
 * Board specific initialization.
 */
static void __init emsym_blurr_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_blurr_pads,
			ARRAY_SIZE(mx6q_blurr_pads));
		if (enet_to_gpio_6) {
			iomux_v3_cfg_t enet_gpio_pad =
				MX6Q_PAD_GPIO_6__ENET_IRQ_TO_GPIO_6;
			mxc_iomux_v3_setup_pad(enet_gpio_pad);
		} else {
			iomux_v3_cfg_t i2c3_pad =
				MX6Q_PAD_GPIO_6__I2C3_SDA;
			mxc_iomux_v3_setup_pad(i2c3_pad);
		}
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_blurr_pads,
			ARRAY_SIZE(mx6dl_blurr_pads));

		if (enet_to_gpio_6) {
			iomux_v3_cfg_t enet_gpio_pad =
				MX6DL_PAD_GPIO_6__ENET_IRQ_TO_GPIO_6;
			mxc_iomux_v3_setup_pad(enet_gpio_pad);
		} else {
			iomux_v3_cfg_t i2c3_pad =
				MX6DL_PAD_GPIO_6__I2C3_SDA;
			mxc_iomux_v3_setup_pad(i2c3_pad);
		}
	}


#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = BLURR_dvfscore_data.reg_id;
	soc_reg_id = BLURR_dvfscore_data.soc_id;
	emsym_blurr_init_uart();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(BLURR_fb_data); i++)
			imx6q_add_ipuv3fb(i, &BLURR_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(BLURR_fb_data); i++)
			imx6q_add_ipuv3fb(i, &BLURR_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &emsym_blurr_i2c_data);
	imx6q_add_imx_i2c(1, &emsym_blurr_i2c_data);
	imx6q_add_imx_i2c(2, &emsym_blurr_i2c_data);
	if (cpu_is_mx6dl())
		imx6q_add_imx_i2c(3, &emsym_blurr_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	ret = gpio_request(BLURR_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(BLURR_PFUZE_INT);
		mx6q_blurr_init_pfuze100(BLURR_PFUZE_INT);
	}
	/* SPI */
	imx6q_add_ecspi(0, &emsym_blurr_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &emsym_blurr_anatop_thermal_data);

	if (enet_to_gpio_6)
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(
			IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ,
			OBSRV_MUX1_MASK);
	else
		fec_data.gpio_irq = -1;
	imx6_init_fec(fec_data);

	imx6q_add_pm_imx(0, &emsym_blurr_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(1, &emsym_blurr_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &emsym_blurr_sd3_data);
        imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	emsym_blurr_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &emsym_blurr_sata_data);
#else
		emsym_blurr_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&BLURR_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/*
	 * Disable HannStar touch panel CABC function,
	 * this function turns the panel's backlight automatically
	 * according to the content shown on the panel which
	 * may cause annoying unstable backlight issue.
	 */
	gpio_request(BLURR_CABC_EN0, "cabc-en0");
	gpio_direction_output(BLURR_CABC_EN0, 0);
	gpio_request(BLURR_CABC_EN1, "cabc-en1");
	gpio_direction_output(BLURR_CABC_EN1, 0);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
        imx6q_add_mxc_pwm_backlight(0, &emsym_blurr_pwm_backlight_data);
        imx6q_add_mxc_pwm_backlight(1, &emsym_blurr_pwm_backlight_data1);
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&BLURR_dvfscore_data);
	imx6q_add_device_buttons();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}
	/*
	ret = gpio_request_array(emsym_blurr_flexcan_gpios,
			ARRAY_SIZE(emsym_blurr_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&emsym_blurr_flexcan0_pdata);
	*/
        imx6q_add_flexcan0(NULL);
	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);


	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&emsym_blurr_pcie_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init emsym_blurr_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer emsym_blurr_timer = {
	.init   = emsym_blurr_timer_init,
};

static void __init emsym_blurr_reserve(void)
{
	phys_addr_t phys;
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_BLURR data structure.
 */
MACHINE_START(IMX6_EMSYM_BLURR, "EMSYM blurr, i.MX 6Quad/DualLite/Solo CPU module.")
	/* Maintainer: emsym.com */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = emsym_blurr_board_init,
	.timer = &emsym_blurr_timer,
	.reserve = emsym_blurr_reserve,
MACHINE_END
