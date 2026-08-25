/*
 * Copyright (c) 2025 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "ral.h"
#include "ralf_sx126x.h"

#include "lbm_sx126x_common.h"

LOG_MODULE_DECLARE(lbm_driver, CONFIG_LORA_LOG_LEVEL);

struct lbm_sx126x_standalone_config {
	struct lbm_sx126x_config common;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec busy;
	struct gpio_dt_spec dio1;
};

struct lbm_sx126x_standalone_data {
	struct lbm_sx126x_data common;
	struct gpio_callback dio1_callback;
};

bool lbm_sx126x_bus_is_busy(const struct device *dev)
{
	const struct lbm_sx126x_standalone_config *config = dev->config;

	return gpio_pin_get_dt(&config->busy);
}

void lbm_sx126x_bus_reset(const struct device *dev)
{
	const struct lbm_sx126x_standalone_config *config = dev->config;

	gpio_pin_set_dt(&config->reset, 1);
	k_sleep(K_MSEC(20));
	gpio_pin_set_dt(&config->reset, 0);
	k_sleep(K_MSEC(10));
}

void lbm_sx126x_bus_irq_enable(const struct device *dev)
{
	const struct lbm_sx126x_standalone_config *config = dev->config;

	gpio_pin_interrupt_configure_dt(&config->dio1, GPIO_INT_EDGE_TO_ACTIVE);
}

void lbm_sx126x_bus_irq_disable(const struct device *dev)
{
	const struct lbm_sx126x_standalone_config *config = dev->config;

	(void)gpio_pin_interrupt_configure_dt(&config->dio1, GPIO_INT_DISABLE);
}

static void sx126x_dio1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct lbm_sx126x_standalone_data *data =
		CONTAINER_OF(cb, struct lbm_sx126x_standalone_data, dio1_callback);

	LOG_DBG("");
	/* Submit work to process the interrupt immediately */
	k_work_schedule(&data->common.lbm_common.op_done_work, K_NO_WAIT);
}

int lbm_sx126x_bus_init(const struct device *dev)
{
	const struct lbm_sx126x_standalone_config *config = dev->config;
	struct lbm_sx126x_standalone_data *data = dev->data;

	if (gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE) ||
	    gpio_pin_configure_dt(&config->busy, GPIO_INPUT) ||
	    gpio_pin_configure_dt(&config->dio1, GPIO_INPUT)) {
		LOG_ERR("GPIO configuration failed.");
		return -EIO;
	}

	gpio_init_callback(&data->dio1_callback, sx126x_dio1_callback, BIT(config->dio1.pin));
	if (gpio_add_callback(config->dio1.port, &data->dio1_callback) < 0) {
		LOG_ERR("Could not set GPIO callback for DIO1 interrupt.");
		return -EIO;
	}

	return 0;
}

#define SX126X_DEFINE(node_id, sx_variant)                                                         \
	static const struct lbm_sx126x_standalone_config config_##node_id = {                      \
		.common = {                                                                         \
			.lbm_common.ralf = RALF_SX126X_INSTANTIATE(DEVICE_DT_GET(node_id)),         \
			.spi = SPI_DT_SPEC_GET(                                                     \
				node_id, SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB),  \
			.ant_enable = GPIO_DT_SPEC_GET_OR(node_id, antenna_enable_gpios, {0}),      \
			.tx_enable = GPIO_DT_SPEC_GET_OR(node_id, tx_enable_gpios, {0}),            \
			.rx_enable = GPIO_DT_SPEC_GET_OR(node_id, rx_enable_gpios, {0}),            \
			.dio3_tcxo_startup_delay_ms =                                               \
				DT_PROP_OR(node_id, tcxo_power_startup_delay_ms, 0),                \
			.dio3_tcxo_voltage = DT_PROP_OR(node_id, dio3_tcxo_voltage, UINT8_MAX),     \
			.dio2_rf_switch = DT_PROP(node_id, dio2_tx_enable),                         \
			.rx_boosted = DT_PROP(node_id, rx_boosted),                                 \
			.variant = sx_variant,                                                      \
		},                                                                                   \
		.reset = GPIO_DT_SPEC_GET(node_id, reset_gpios),                                    \
		.busy = GPIO_DT_SPEC_GET(node_id, busy_gpios),                                      \
		.dio1 = GPIO_DT_SPEC_GET(node_id, dio1_gpios),                                      \
	};                                                                                          \
	static struct lbm_sx126x_standalone_data data_##node_id;                                   \
	DEVICE_DT_DEFINE(node_id, lbm_sx126x_init, NULL, &data_##node_id, &config_##node_id,       \
			 POST_KERNEL, CONFIG_LORA_INIT_PRIORITY, &lbm_lora_api)

#define SX1261_DEFINE(node_id) SX126X_DEFINE(node_id, VARIANT_SX1261)
#define SX1262_DEFINE(node_id) SX126X_DEFINE(node_id, VARIANT_SX1262)

DT_FOREACH_STATUS_OKAY(semtech_sx1261, SX1261_DEFINE);
DT_FOREACH_STATUS_OKAY(semtech_sx1262, SX1262_DEFINE);
