/*
 * Copyright (c) 2025 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32wl_subghz_radio

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stm32wlxx_ll_exti.h>
#include <stm32wlxx_ll_pwr.h>
#include <stm32wlxx_ll_rcc.h>

#include "ral.h"
#include "ralf_sx126x.h"

#include "lbm_sx126x_common.h"

LOG_MODULE_DECLARE(lbm_driver, CONFIG_LORA_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "Exactly one STM32WL internal radio instance is supported");

/* The internal radio has no BUSY/NRESET/DIO1 GPIOs: BUSY is a PWR status
 * flag, reset goes through the RCC, and the radio IRQ is a single combined
 * "Radio IRQ, Busy" NVIC line (see st,stm32wl-subghz-radio.yaml).
 */

bool lbm_sx126x_bus_is_busy(const struct device *dev)
{
	ARG_UNUSED(dev);

	return LL_PWR_IsActiveFlag_RFBUSYS();
}

void lbm_sx126x_bus_reset(const struct device *dev)
{
	ARG_UNUSED(dev);

	LL_RCC_RF_EnableReset();
	k_sleep(K_MSEC(20));
	LL_RCC_RF_DisableReset();
	k_sleep(K_MSEC(10));
}

void lbm_sx126x_bus_irq_enable(const struct device *dev)
{
	ARG_UNUSED(dev);

	irq_enable(DT_INST_IRQN(0));
}

void lbm_sx126x_bus_irq_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	irq_disable(DT_INST_IRQN(0));
}

void lbm_lora_irq_rearm(const struct device *dev)
{
	lbm_sx126x_bus_irq_enable(dev);
}

static void stm32wl_radio_isr(const struct device *dev)
{
	struct lbm_sx126x_data *data = dev->data;

	/* The combined "Radio IRQ, Busy" line stays asserted until the radio's own IRQ
	 * status is cleared over SPI, which only happens once op_done_work runs. Mask at
	 * the NVIC here to avoid re-entering this ISR continuously in the meantime;
	 * lbm_lora_irq_rearm() unmasks it again once that work item has cleared the
	 * radio's IRQ status.
	 */
	irq_disable(DT_INST_IRQN(0));
	k_work_schedule(&data->lbm_common.op_done_work, K_NO_WAIT);
}

int lbm_sx126x_bus_init(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), stm32wl_radio_isr,
		    DEVICE_DT_INST_GET(0), 0);
	LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_44);

	return 0;
}

static const struct lbm_sx126x_config config_inst = {
	.lbm_common.ralf = RALF_SX126X_INSTANTIATE(DEVICE_DT_INST_GET(0)),
	.spi = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB),
	.ant_enable = GPIO_DT_SPEC_INST_GET_OR(0, antenna_enable_gpios, {0}),
	.tx_enable = GPIO_DT_SPEC_INST_GET_OR(0, tx_enable_gpios, {0}),
	.rx_enable = GPIO_DT_SPEC_INST_GET_OR(0, rx_enable_gpios, {0}),
	.dio3_tcxo_startup_delay_ms = DT_INST_PROP_OR(0, tcxo_power_startup_delay_ms, 0),
	.dio3_tcxo_voltage = DT_INST_PROP_OR(0, dio3_tcxo_voltage, UINT8_MAX),
	.dio2_rf_switch = DT_INST_PROP(0, dio2_tx_enable),
	.rx_boosted = DT_INST_PROP(0, rx_boosted),
	/* The internal radio is a single SX1261/SX1262-class core whose two output
	 * pins (RFO_LP, RFO_HP) match the SX1261 (low power) and SX1262 (high power)
	 * variants' PA configs respectively, including the RFO_HP TX_CLAMP_CFG erratum
	 * workaround LBM's own sx126x driver already applies generically based on
	 * pa_cfg.device_sel. Pick the matching variant from the board's wiring.
	 */
	.variant = DT_INST_ENUM_HAS_VALUE(0, power_amplifier_output, rfo_lp) ? VARIANT_SX1261
									     : VARIANT_SX1262,
};
static struct lbm_sx126x_data data_inst;

DEVICE_DT_INST_DEFINE(0, lbm_sx126x_init, NULL, &data_inst, &config_inst, POST_KERNEL,
		       CONFIG_LORA_INIT_PRIORITY, &lbm_lora_api);
