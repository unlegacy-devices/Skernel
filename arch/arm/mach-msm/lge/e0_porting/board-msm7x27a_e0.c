/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <linux/memblock.h>
#include <asm/mach-types.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/usbdiag.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/usb/android.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_battery.h>
#include <linux/atmel_maxtouch.h>
#include <linux/msm_adc.h>
#include <linux/msm_ion.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <linux/persistent_ram.h>
#endif

#include "../../devices.h"
#include "../../timer.h"
#include "../../board-msm7x27a-regulator.h"
#include "../../devices-msm7x2xa.h"
#include "../../pm.h"

#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>

#include CONFIG_LGE_BOARD_HEADER_FILE
#include "../../pm-boot.h"
#include "../../board-msm7627a.h"
#include "./board-input/board-input.h"

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <asm/setup.h>
#endif /*CONFIG_ANDROID_RAM_CONSOLE*/

#ifdef CONFIG_LGE_BOOT_MODE
#include <mach/lge/lge_boot_mode.h>
#endif
#define RESERVE_KERNEL_EBI1_SIZE	0x3A000
#define MSM_RESERVE_AUDIO_SIZE		0xF0000
#define BOOTLOADER_BASE_ADDR		0x10000

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define MSM7X27_EBI1_CS0_SIZE	0xFD00000
#define LGE_RAM_CONSOLE_START	(MSM7X27_EBI1_CS0_BASE + MSM7X27_EBI1_CS0_SIZE)
#endif


#ifdef CONFIG_ANDROID_RAM_CONSOLE
struct persistent_ram_descriptor ram_console_desc = {
        .name = "ram_console",
        .size = LGE_RAM_CONSOLE_SIZE - 1,
};
	
struct persistent_ram ram_console_ram = {
        .start = LGE_RAM_CONSOLE_START,
        .size = LGE_RAM_CONSOLE_SIZE - 1,
        .num_descs = 1,
        .descs = &ram_console_desc,
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
};
#endif

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static struct platform_device msm_wlan_ar6000_pm_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
};
#endif

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc;

	if (adap_id < 0 || adap_id > 1)
		return;

	/* Each adapter gets 2 lines from the table */
	if (config_type)
		rc = msm_gpios_request_enable(&qup_i2c_gpios_hw[adap_id*2], 2);
	else
		rc = msm_gpios_request_enable(&qup_i2c_gpios_io[adap_id*2], 2);
	if (rc < 0)
		pr_err("QUP GPIO request/enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {
	.clk_freq		= 400000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_RESERVE_MDP_SIZE		0xE00000
#define MSM7x25A_MSM_RESERVE_MDP_SIZE	0x1500000
#define MSM_RESERVE_ADSP_SIZE		0x1300000
#define MSM7x25A_MSM_RESERVE_ADSP_SIZE	0xB91000
#define CAMERA_ZSL_SIZE			(SZ_1M * 60)
#endif

#ifdef CONFIG_ION_MSM
#define MSM_ION_HEAP_NUM	5
static struct platform_device ion_dev;
static int msm_ion_camera_size;
static int msm_ion_audio_size;
static int msm_ion_sf_size;
static int msm_ion_camera_size_carving;
#endif

#define CAMERA_HEAP_BASE	0x0
#ifdef CONFIG_CMA
#define CAMERA_HEAP_TYPE	ION_HEAP_TYPE_DMA
#else
#define CAMERA_HEAP_TYPE	ION_HEAP_TYPE_CARVEOUT
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc = 0;
	unsigned gpio;

	gpio = GPIO_HOST_VBUS_EN;

	rc = gpio_request(gpio, "i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", gpio);
		return;
	}
	gpio_direction_output(gpio, !!on);
	gpio_set_value_cansleep(gpio, !!on);
	gpio_free(gpio);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};

static void __init msm7x2x_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct regulator *reg_hsusb;
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		reg_hsusb = regulator_get(NULL, "usb");
		if (IS_ERR(reg_hsusb)) {
			rc = PTR_ERR(reg_hsusb);
			pr_err("%s: could not get regulator: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_set_voltage(reg_hsusb, 3300000, 3300000);
		if (rc) {
			pr_err("%s: could not set voltage: %d\n",
					__func__, rc);
			goto reg_free;
		}

		return 0;
	}
	/* else fall through */
reg_free:
	regulator_put(reg_hsusb);
out:
	reg_hsusb = NULL;
	return rc;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (IS_ERR_OR_NULL(reg_hsusb))
		return reg_hsusb ? PTR_ERR(reg_hsusb) : -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	return enable ?
		regulator_enable(reg_hsusb) :
		regulator_disable(reg_hsusb);
}

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret = 0;

	if (init)
		ret = msm_pm_app_rpc_init(callback);
	else
		msm_pm_app_rpc_deinit(callback);

	return ret;
}
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init	 = msm_hsusb_pmic_notif_init,
#else
	.vbus_power		 = msm_hsusb_vbus_power,
#endif
	.rpc_connect		 = hsusb_rpc_connect,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.chg_init		 = hsusb_chg_init,
	.chg_connected		 = hsusb_chg_connected,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
};
#endif

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif

static struct msm_pm_platform_data msm7x27a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 1,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS,
	.p_addr = 0,
};

static unsigned reserve_mdp_size = MSM_RESERVE_MDP_SIZE;
static int __init reserve_mdp_size_setup(char *p)
{
	reserve_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("reserve_mdp_size", reserve_mdp_size_setup);

static unsigned reserve_adsp_size = MSM_RESERVE_ADSP_SIZE;
static int __init reserve_adsp_size_setup(char *p)
{
	reserve_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("reserve_adsp_size", reserve_adsp_size_setup);

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design     = 3200,
	.voltage_max_design     = 4200,
	.voltage_fail_safe      = 3340,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage	 = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage = msm_psy_batt_data.voltage_max_design;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata = {
	.dev_names = msm_adc_surf_device_names,
	.num_adc = ARRAY_SIZE(msm_adc_surf_device_names),
	.target_hw = MSM_8x25,
};

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) && defined(CONFIG_MSM_SHARED_GPIO_FOR_UART2DM)
static struct msm_gpio uart2dm_gpios[] = {
	{GPIO_CFG(19, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rfr_n" },
	{GPIO_CFG(20, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_cts_n" },
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

static void msm7x27a_cfg_uart2dm_serial(void)
{
	int ret;
	ret = msm_gpios_request_enable(uart2dm_gpios,
					ARRAY_SIZE(uart2dm_gpios));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);
}
#else
static void msm7x27a_cfg_uart2dm_serial(void) { }
#endif

static struct platform_device *msm7627a_surf_ffa_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1,
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&msm_kgsl_3d0,
};

static struct platform_device *common_devices[] __initdata = {
	&android_usb_device,
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
	&msm_batt_device,
	&msm_adc_device,
#ifdef CONFIG_ION_MSM
	&ion_dev,
#endif
};

static unsigned reserve_kernel_ebi1_size = RESERVE_KERNEL_EBI1_SIZE;
static int __init reserve_kernel_ebi1_size_setup(char *p)
{
	reserve_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("reserve_kernel_ebi1_size", reserve_kernel_ebi1_size_setup);

static unsigned reserve_audio_size = MSM_RESERVE_AUDIO_SIZE;
static int __init reserve_audio_size_setup(char *p)
{
	reserve_audio_size = memparse(p, NULL);
	return 0;
}
early_param("reserve_audio_size", reserve_audio_size_setup);

static void fix_sizes(void)
{
	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		reserve_mdp_size = MSM7x25A_MSM_RESERVE_MDP_SIZE;
		reserve_adsp_size = MSM7x25A_MSM_RESERVE_ADSP_SIZE;
	} else {
		reserve_mdp_size = MSM_RESERVE_MDP_SIZE;
		reserve_adsp_size = MSM_RESERVE_ADSP_SIZE;
	}

	if (get_ddr_size() > SZ_512M)
		reserve_adsp_size = CAMERA_ZSL_SIZE;
#ifdef CONFIG_ION_MSM
	msm_ion_audio_size = MSM_RESERVE_AUDIO_SIZE;
	msm_ion_sf_size = reserve_mdp_size;
#ifdef CONFIG_CMA
	if (get_ddr_size() > SZ_256M)
		reserve_adsp_size = CAMERA_ZSL_SIZE;
	msm_ion_camera_size = reserve_adsp_size;
	msm_ion_camera_size_carving = 0;
#else
	msm_ion_camera_size = reserve_adsp_size;
	msm_ion_camera_size_carving = msm_ion_camera_size;
#endif
#endif
}
#ifdef CONFIG_ION_MSM
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct ion_co_heap_pdata co_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
};

static struct ion_co_heap_pdata co_mm_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
};

static u64 msm_dmamask = DMA_BIT_MASK(32);

static struct platform_device ion_cma_device = {
	.name = "ion-cma-device",
	.id = -1,
	.dev = {
		.dma_mask = &msm_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};
#endif

/**
 * These heaps are listed in the order they will be allocated.
 * Don't swap the order unless you know what you are doing!
 */
struct ion_platform_heap msm7x27a_heaps[] = {
		{
			.id	= ION_SYSTEM_HEAP_ID,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= ION_VMALLOC_HEAP_NAME,
		},
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		/* ION_ADSP = CAMERA */
		{
			.id	= ION_CAMERA_HEAP_ID,
			.type	= CAMERA_HEAP_TYPE,
			.name	= ION_CAMERA_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_mm_ion_pdata,
			.priv	= (void *)&ion_cma_device.dev,
		},
		/* AUDIO HEAP 1 */
		{
			.id	= ION_AUDIO_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
		},
		/* ION_MDP = SF */
		{
			.id	= ION_SF_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_SF_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
		},
		/* AUDIO HEAP 2*/
		{
			.id	= ION_AUDIO_HEAP_BL_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_BL_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
			.base = BOOTLOADER_BASE_ADDR,
		},
#endif
};

static struct ion_platform_data ion_pdata = {
	.nr = MSM_ION_HEAP_NUM,
	.has_outer_cache = 1,
	.heaps = msm7x27a_heaps,
};

static struct platform_device ion_dev = {
	.name = "ion-msm",
	.id = 1,
	.dev = { .platform_data = &ion_pdata },
};
#endif

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_ion_devices(void)
{
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	ion_pdata.heaps[1].size = msm_ion_camera_size;
	ion_pdata.heaps[2].size = RESERVE_KERNEL_EBI1_SIZE;
	ion_pdata.heaps[3].size = msm_ion_sf_size;
	ion_pdata.heaps[4].size = msm_ion_audio_size;
#endif
}

static void __init reserve_ion_memory(void)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += RESERVE_KERNEL_EBI1_SIZE;
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += msm_ion_camera_size_carving;
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += msm_ion_sf_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	fix_sizes();
	size_ion_devices();
	reserve_ion_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

static void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	memblock_remove(MSM8625_NON_CACHE_MEM, SZ_2K);
	memblock_remove(BOOTLOADER_BASE_ADDR, msm_ion_audio_size);
	msm_reserve();
#ifdef CONFIG_CMA
	dma_declare_contiguous(
			&ion_cma_device.dev,
			msm_ion_camera_size,
			CAMERA_HEAP_BASE,
			0x26000000);
#endif
}

static void __init msm7x27a_device_i2c_init(void)
{
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;
}

#define MSM_EBI2_PHYS		0xa0d00000
#define MSM_EBI2_XMEM_CS2_CFG1	0xa0d10030

static void __init msm7x27a_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void __iomem *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_PHYS, sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
		ebi2_cfg |= (1 << 31);

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);
}

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data = &msm7x27a_proccomm_regulator_data
	}
};

static void msm_adsp_add_pdev(void)
{
	int rc = 0;
	struct rpc_board_dev *rpc_adsp_pdev;

	rpc_adsp_pdev = kzalloc(sizeof(struct rpc_board_dev), GFP_KERNEL);
	if (rpc_adsp_pdev == NULL) {
		pr_err("%s: Memory Allocation failure\n", __func__);
		return;
	}
	rpc_adsp_pdev->prog = ADSP_RPC_PROG;

	rpc_adsp_pdev->pdev = msm_adsp_device;
	rc = msm_rpc_add_board_dev(rpc_adsp_pdev, 1);
	if (rc < 0) {
		pr_err("%s: return val: %d\n",	__func__, rc);
		kfree(rpc_adsp_pdev);
	}
}

#define UART1DM_RX_GPIO		45

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static int __init msm7x27a_init_ar6000pm(void)
{
	msm_wlan_ar6000_pm_device.dev.platform_data = &ar600x_wlan_power;
	return platform_device_register(&msm_wlan_ar6000_pm_device);
}
#else
static int __init msm7x27a_init_ar6000pm(void) { return 0; }
#endif

static void __init msm7x27a_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
		pr_err("%s: could not register regulator device: %d\n",
				__func__, rc);
}

static void __init msm7x27a_add_footswitch_devices(void)
{
	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);
}

static void __init msm7x27a_add_platform_devices(void)
{
  platform_add_devices(msm7627a_surf_ffa_devices,
			ARRAY_SIZE(msm7627a_surf_ffa_devices));

	platform_add_devices(common_devices,
			ARRAY_SIZE(common_devices));
}

static void __init msm7x27a_uartdm_config(void)
{
	msm7x27a_cfg_uart2dm_serial();
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);

	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
}

static void __init msm7x27a_otg_gadget(void)
{
		msm_otg_pdata.swfi_latency =
		msm7x27a_pm_data[
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
		msm_device_otg.dev.platform_data = &msm_otg_pdata;
		msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
}

static void __init msm7x27a_pm_init(void)
{
	msm_pm_set_platform_data(msm7x27a_pm_data,
      ARRAY_SIZE(msm7x27a_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));

	msm_pm_register_irqs();
}

void __init msm7627a_add_io_devices(void)
{
	add_bosch_sensors();
	add_gp2ap_proximity();
	add_melfes_touchscreen();
	add_gpio_keypad();
	add_gpio_earsense();
	add_pmic_leds();

	msm_init_pmic_vibrator();
}

static void __init msm7x2x_init(void)
{
	msm7x2x_misc_init();

	msm7x27a_init_regulators();

	msm_adsp_add_pdev();

	msm7x27a_device_i2c_init();
	msm7x27a_init_ebi2();
	msm7x27a_uartdm_config();
	msm7x27a_otg_gadget();
	msm7x27a_add_footswitch_devices();
	msm7x27a_add_platform_devices();
	pr_err("---------- msm7x27a_init_ar6000pm start -------------");
	msm7x27a_init_ar6000pm();
	pr_err("---------- msm7x27a_init_ar6000pm end -------------");

  msm7627a_init_mmc();

	msm_fb_add_devices();

	msm7x2x_init_host();
	msm7x27a_pm_init();
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	pr_err("---------- msm7627a_bt_power_init start -------------");
  msm7627a_bt_power_init();
	pr_err("---------- msm7627a_bt_power_init end -------------");
#endif
#ifdef CONFIG_MSM7X27A_AUDIO
  //lge_add_sound_devices();
#endif
  //msm7627a_camera_init();
	msm7627a_add_io_devices();
	msm7x25a_kgsl_3d0_init();
	msm8x25_kgsl_3d0_init();
	lge_add_gpio_i2c_devices();
#ifdef CONFIG_LGE_POWER_ON_STATUS_PATCH
	lge_board_pwr_on_status();
#endif
#ifdef CONFIG_LGE_BOOT_MODE
	lge_add_boot_mode_devices();
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE) && defined(CONFIG_LGE_HANDLE_PANIC)
	lge_add_panic_handler_devices();
#endif
	lge_add_pm_devices();
}

static void __init msm7x2x_init_early(void)
{
	msm_msm7627a_allocate_memory_regions();
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	persistent_ram_early_init(&ram_console_ram);
#endif
}

MACHINE_START(MSM7X25A_E0EU, "MSM7225A E0EU")
	.atag_offset	= 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
