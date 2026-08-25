/*
 * Copyright (c) 2025 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "lbm_common.h"

enum sx126x_variant {
	VARIANT_SX1261,
	VARIANT_SX1262,
};

/* Config/data fields common to every bus this radio is reachable over.
 * Must be first element of the bus-specific config/data, per the
 * lbm_lora_config_common / lbm_lora_data_common convention.
 */
struct lbm_sx126x_config {
	struct lbm_lora_config_common lbm_common;
	struct spi_dt_spec spi;
	struct gpio_dt_spec ant_enable;
	struct gpio_dt_spec tx_enable;
	struct gpio_dt_spec rx_enable;
	int dio3_tcxo_startup_delay_ms;
	uint8_t dio3_tcxo_voltage;
	bool dio2_rf_switch;
	bool rx_boosted;
	enum sx126x_variant variant;
};

struct lbm_sx126x_data {
	struct lbm_lora_data_common lbm_common;
	const struct device *dev;
	bool asleep;
};

/**
 * @brief Initialise the device's bus (SPI/reset/busy/IRQ) hardware
 *
 * Implemented by the bus-specific glue (lbm_sx126x_standalone.c for a
 * discrete SPI-attached chip, lbm_sx126x_stm32wl.c for the STM32WL's
 * internal radio).
 *
 * @retval 0 On success
 * @retval -errno On failure
 */
int lbm_sx126x_bus_init(const struct device *dev);

/**
 * @brief Query the chip's BUSY line
 */
bool lbm_sx126x_bus_is_busy(const struct device *dev);

/**
 * @brief Toggle the chip's NRESET line
 */
void lbm_sx126x_bus_reset(const struct device *dev);

/**
 * @brief Enable the radio IRQ line
 */
void lbm_sx126x_bus_irq_enable(const struct device *dev);

/**
 * @brief Disable the radio IRQ line
 */
void lbm_sx126x_bus_irq_disable(const struct device *dev);

/* Shared init function, called from each bus's DEVICE_DT_*DEFINE */
int lbm_sx126x_init(const struct device *dev);
