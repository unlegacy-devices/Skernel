/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio_event.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input/ft5x06_ts.h>
#include <linux/leds-msm-tricolor.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <mach/rpc_server_handset.h>
#include <mach/pmic.h>
#include <linux/ktime.h>

#include "../../devices.h"
#include "../../board-msm7627a.h"
#include "../../devices-msm7x2xa.h"
#include CONFIG_LGE_BOARD_HEADER_FILE

/* handset device */
static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500,
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static unsigned int keypad_row_gpios[] = {36, 37, 38};
static unsigned int keypad_col_gpios[] = {33, 32};

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(keypad_row_gpios) + (row))

static const unsigned short keypad_keymap_v3[] = {
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(1, 2)] = KEY_HOMEPAGE,
};

int v3_matrix_info_wrapper(struct gpio_event_input_devs *input_dev,
			   struct gpio_event_info *info, void **data, int func)
{
	int ret;
	int i;
	if (func == GPIO_EVENT_FUNC_RESUME) {
		for(i = 0; i < ARRAY_SIZE(keypad_row_gpios); i++)  {
			gpio_tlmm_config(GPIO_CFG(keypad_row_gpios[i], 0, GPIO_CFG_INPUT,
				 GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	        }
	}
	
	ret = gpio_event_matrix_func(input_dev, info, data, func);
	return ret ;
}

static int v3_gpio_matrix_power(const struct gpio_event_platform_data *pdata, bool on)
{
	return 0;
}

static struct gpio_event_matrix_info v3_keypad_matrix_info = {
	.info.func	= v3_matrix_info_wrapper,
	.keymap		= keypad_keymap_v3,
	.output_gpios	= keypad_col_gpios,
	.input_gpios	= keypad_row_gpios,
	.noutputs	= ARRAY_SIZE(keypad_col_gpios),
	.ninputs	= ARRAY_SIZE(keypad_row_gpios),	
	.settle_time.tv64 = 40 * NSEC_PER_USEC,
	.poll_time.tv64 = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_PRINT_UNMAPPED_KEYS | GPIOKPF_DRIVE_INACTIVE
};

static struct gpio_event_info *v3_keypad_info[] = {
	&v3_keypad_matrix_info.info
};

static struct gpio_event_platform_data v3_keypad_data = {
	.name		= "vee3_keypad",
	.info		= v3_keypad_info,
	.info_count	= ARRAY_SIZE(v3_keypad_info),
	.power          = v3_gpio_matrix_power,
};

struct platform_device keypad_device_v3 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &v3_keypad_data,
	},
};

/* input platform device */
static struct platform_device *v3_input_devices[] __initdata = {
	&hs_pdev,
	&keypad_device_v3,
};

#if defined (CONFIG_SENSORS_BMM050) || defined (CONFIG_SENSORS_BMA250)
static struct gpio_i2c_pin accel_i2c_pin[] = {
	[0] = {
		.sda_pin	= SENSOR_GPIO_I2C_SDA,
		.scl_pin	= SENSOR_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= ACCEL_GPIO_INT,
	},
};

static struct gpio_i2c_pin ecom_i2c_pin[] = {
	[0] = {
		.sda_pin	= SENSOR_GPIO_I2C_SDA,
		.scl_pin	= SENSOR_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= ECOM_GPIO_INT,
	},
};

static struct i2c_gpio_platform_data sensor_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device sensor_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &sensor_i2c_pdata,
};

static struct i2c_board_info sensor_i2c_bdinfo[] = {
	[0] = {
#if defined (CONFIG_SENSORS_BMA2X2)
		I2C_BOARD_INFO("bma2x2", ACCEL_I2C_ADDRESS),
		.type = "bma2x2",
#else
		I2C_BOARD_INFO("bma250", ACCEL_I2C_ADDRESS),
		.type = "bma250",
#endif		
	},
	[1] = {
		I2C_BOARD_INFO("bmm050", ECOM_I2C_ADDRESS),
		.type = "bmm050",
	},
};

static void __init v3_init_i2c_sensor(int bus_num)
{
	sensor_i2c_device.id = bus_num;
	lge_init_gpio_i2c_pin(&sensor_i2c_pdata, accel_i2c_pin[0], &sensor_i2c_bdinfo[0]);
	lge_init_gpio_i2c_pin(&sensor_i2c_pdata, ecom_i2c_pin[0], &sensor_i2c_bdinfo[1]);
	i2c_register_board_info(bus_num, sensor_i2c_bdinfo, ARRAY_SIZE(sensor_i2c_bdinfo));
	platform_device_register(&sensor_i2c_device);
}
#endif

static struct proximity_platform_data proxi_pdata = {
	.irq_num = PROXI_GPIO_DOUT,
	.power = prox_power_set,
	.methods = 0,
	.operation_mode = 0,
	.debounce = 0,
	.cycle = 2,
};

static struct i2c_board_info prox_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("proximity_gp2ap", PROXI_I2C_ADDRESS),
		.type = "proximity_gp2ap",
		.platform_data = &proxi_pdata,
	},
};

static struct gpio_i2c_pin proxi_i2c_pin[] = {
	[0] = {
		.sda_pin	= PROXI_GPIO_I2C_SDA,
		.scl_pin	= PROXI_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= PROXI_GPIO_DOUT,
	},
};

static struct i2c_gpio_platform_data proxi_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device proxi_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &proxi_i2c_pdata,
};

static void __init v3_init_i2c_prox(int bus_num)
{
	proxi_i2c_device.id = bus_num;
	lge_init_gpio_i2c_pin(&proxi_i2c_pdata, proxi_i2c_pin[0], &prox_i2c_bdinfo[0]);
	i2c_register_board_info(bus_num, &prox_i2c_bdinfo[0], 1);
	platform_device_register(&proxi_i2c_device);
}

#if defined(CONFIG_TOUCHSCREEN_MELFAS_MMS128S)
static struct gpio_i2c_pin ts_i2c_pin[] = {
	[0] = {
		.sda_pin	= TS_GPIO_I2C_SDA,
		.scl_pin	= TS_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= TS_GPIO_IRQ,
	},
};

static struct i2c_gpio_platform_data ts_i2c_pdata = {
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.udelay			= 1,
};

static struct platform_device ts_i2c_device = {
	.name	= "i2c-gpio",
	.dev.platform_data = &ts_i2c_pdata,
};


static struct regulator *regulator_ts;
static char is_touch_Initialized = 0;
int ts_set_vreg(unsigned char onoff)
{
	int rc;
	
#if defined(CONFIG_MACH_MSM7X25A_V3)
	if (1) {
		regulator_ts = regulator_get(NULL, "rfrx1");
		if (regulator_ts == NULL)
			pr_err("%s: regulator_get(regulator_ts) failed\n",__func__);
			
		rc = regulator_set_voltage(regulator_ts, 3000000, 3000000);
		if (rc < 0)
			pr_err("%s: regulator_set_voltage(regulator_ts) failed\n", __func__);
		
		is_touch_Initialized = 1;
	}

	if (onoff) {
		rc = regulator_enable(regulator_ts);
		if (rc < 0)
			pr_err("%s: regulator_enable(regulator_ts) failed\n", __func__);

	} else {
		rc = regulator_disable(regulator_ts);
		if (rc < 0)
			pr_err("%s: regulator_disble(regulator_ts) failed\n", __func__);
	}

#endif

	return rc;
}

static struct touch_platform_data ts_pdata = {
	.ts_x_min = TS_X_MIN,
	.ts_x_max = TS_X_MAX,
	.ts_y_min = TS_Y_MIN,
	.ts_y_max = TS_Y_MAX,
	.power 	  = ts_set_vreg,
	.irq 	  = TS_GPIO_IRQ,
	.scl      = TS_GPIO_I2C_SCL,
	.sda      = TS_GPIO_I2C_SDA,
};

static struct i2c_board_info ts_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("touch_mcs8000", TS_I2C_SLAVE_ADDR),
		.type = "touch_mcs8000",
		.platform_data = &ts_pdata,
	},
};

/* this routine should be checked for nessarry */
static int init_gpio_i2c_pin_touch(
	struct i2c_gpio_platform_data *i2c_adap_pdata,
	struct gpio_i2c_pin gpio_i2c_pin,
	struct i2c_board_info *i2c_board_info_data)
{
	i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
	i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;

	gpio_request(TS_GPIO_I2C_SDA, "Melfas_I2C_SDA");
	gpio_request(TS_GPIO_I2C_SCL, "Melfas_I2C_SCL");
	gpio_request(TS_GPIO_IRQ, "Melfas_I2C_INT");

	gpio_tlmm_config(
		GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(
		GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(gpio_i2c_pin.sda_pin, 1);
	gpio_set_value(gpio_i2c_pin.scl_pin, 1);

	if (gpio_i2c_pin.reset_pin) {
		gpio_tlmm_config(
			GPIO_CFG(gpio_i2c_pin.reset_pin, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(gpio_i2c_pin.reset_pin, 1);
	}

	if (gpio_i2c_pin.irq_pin) {
		gpio_tlmm_config(
			GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		i2c_board_info_data->irq =
			MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
	}

	gpio_free(TS_GPIO_I2C_SDA);
	gpio_free(TS_GPIO_I2C_SCL);
	gpio_free(TS_GPIO_IRQ);

	return 0;
}

static void __init v3eu_init_i2c_touch(int bus_num)
{
	ts_i2c_device.id = bus_num;

	init_gpio_i2c_pin_touch(
		&ts_i2c_pdata, ts_i2c_pin[0], &ts_i2c_bdinfo[0]);
	i2c_register_board_info(
		bus_num, &ts_i2c_bdinfo[0], 1);
	platform_device_register(&ts_i2c_device);
}
#endif

void __init msm7627a_add_io_devices(void)
{
	hs_platform_data.ignore_end_key = true;
	platform_add_devices(v3_input_devices, ARRAY_SIZE(v3_input_devices));

#if defined(CONFIG_TOUCHSCREEN_MELFAS_MMS128S)
	lge_add_gpio_i2c_device(v3eu_init_i2c_touch);
#endif

#if defined (CONFIG_SENSORS_BMM050) ||defined(CONFIG_SENSORS_BMA250)
	lge_add_gpio_i2c_device(v3_init_i2c_sensor);
#endif
	lge_add_gpio_i2c_device(v3_init_i2c_prox);

	msm_init_pmic_vibrator();
}

void __init qrd7627a_add_io_devices(void)
{
}
