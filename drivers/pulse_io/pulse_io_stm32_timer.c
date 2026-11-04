/*
 * Copyright (c) 2026 Dent IoT
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file pulse_io_stm32_timer.c
 * STM32 Pulse IO backend, SYMBOL mode with two DMA streams:
 *
 *  - stream "tx": circular 2-word buffer (BSRR set/reset pair) -> GPIO BSRR.
 *    The pin toggles once per timer update period, so each timer period is
 *    one symbol with alternating level.
 *  - stream "arr": linear buffer with one word per symbol -> TIMx_ARR. Each
 *    word sets the length of the next timer period (ARR+1 ticks), i.e. the
 *    symbol duration.
 *
 * Both streams are triggered by the same timer update event, so one DMA word
 * per stream is consumed per period and the buffer needs only ~1 word per
 * symbol instead of one word per tick.
 */

#include <stdint.h>
#include <sys/errno.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pulse_io.h>
#include <zephyr/dt-bindings/dma/stm32_dma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#include <stm32_ll_gpio.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>

#define DT_DRV_COMPAT st_stm32_timer_pulse_io

LOG_MODULE_REGISTER(pulse_io_stm32_timer, CONFIG_PULSE_IO_LOG_LEVEL);

#define PULSE_IO_STM32_DEFAULT_BUFSZ 256

/* buffer-size = 0 selects the build-time default. */
#define PULSE_IO_STM32_BUFSZ(inst)                                                                \
	(DT_INST_PROP(inst, buffer_size) == 0 ? PULSE_IO_STM32_DEFAULT_BUFSZ                          \
					      : DT_INST_PROP(inst, buffer_size))

#if defined(CONFIG_DCACHE_LINE_SIZE) && (CONFIG_DCACHE_LINE_SIZE > 0)
#define PULSE_IO_STM32_BUF_ALIGN CONFIG_DCACHE_LINE_SIZE
#else
#define PULSE_IO_STM32_BUF_ALIGN sizeof(uint32_t)
#endif

/* One timer period per symbol: an ARR word written by DMA at update event
 * E_k only becomes the reload value at E_{k+1}, so it sets the length of the
 * period starting at E_{k+2}. The first symbol's period is therefore covered
 * by the two leading dummy periods whose length is written directly before
 * the counter starts, and two trailing dummy words keep the ARR stream alive
 * until the last symbol's period has completed.
 */
#define PULSE_IO_STM32_PIPE_DUMMY_WORDS 2

struct pulse_io_stm32_config {
	TIM_TypeDef *timer;
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	uint32_t prescaler;
	const struct device *dma_dev;
	uint32_t dma_channel_tx;
	uint32_t dma_slot_tx;
	uint32_t dma_priority_tx;
	uint32_t dma_channel_arr;
	uint32_t dma_slot_arr;
	uint32_t dma_priority_arr;
	struct gpio_dt_spec gpio;
	volatile uint32_t *bsrr;
	uint16_t mask;
	size_t buf_size;
};

struct pulse_io_stm32_channel {
	int reserved;
};

struct pulse_io_stm32_data {
	struct k_sem done;
	int result;
	bool configured;
	bool in_use;
	struct k_spinlock busy_spinlock;
	bool busy;
	enum pulse_io_dir dir;
	uint32_t resolution_hz;
	bool idle_high;
	struct pulse_io_stm32_channel chan;
	uint32_t bsrr_pattern[2];
	uint32_t *buf;
	struct dma_block_config block_tx;
	struct dma_block_config block_arr;
	struct dma_config dma_cfg_tx;
	struct dma_config dma_cfg_arr;
};

static inline uint32_t set_mask(const struct pulse_io_stm32_config *config)
{
	return (uint32_t)config->mask;
}

static inline uint32_t reset_mask(const struct pulse_io_stm32_config *config)
{
	return (uint32_t)config->mask << 16;
}

/* Physical BSRR words for the logical high/low levels, honouring the
 * GPIO_ACTIVE_LOW polarity from the devicetree specifier.
 */
static inline uint32_t high_word(const struct pulse_io_stm32_config *config)
{
	if ((config->gpio.dt_flags & GPIO_ACTIVE_LOW) != 0U) {
		return reset_mask(config);
	}
	return set_mask(config);
}

static inline uint32_t low_word(const struct pulse_io_stm32_config *config)
{
	if ((config->gpio.dt_flags & GPIO_ACTIVE_LOW) != 0U) {
		return set_mask(config);
	}
	return reset_mask(config);
}

static inline void pulse_io_stm32_set_idle(const struct device *dev, bool idle_high)
{
	const struct pulse_io_stm32_config *config = dev->config;

	*config->bsrr = idle_high ? high_word(config) : low_word(config);
}

static int pulse_io_stm32_check_chan(const struct device *dev,
				     const struct pulse_io_channel *chan)
{
	const struct pulse_io_stm32_data *data = dev->data;

	if (chan != (const struct pulse_io_channel *)&data->chan || !data->in_use) {
		return -EINVAL;
	}

	return 0;
}

static int pulse_io_stm32_get_capabilities(const struct device *dev,
					   struct pulse_io_caps *caps)
{
	const struct pulse_io_stm32_config *config = dev->config;

	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	uint32_t tim_clk;
	int ret = clock_control_get_rate(clk,
			(clock_control_subsys_t)&config->pclken[1], &tim_clk);
	if (ret < 0) {
		return ret;
	}

	uint32_t min_tick = 1000000000LU * (uint64_t)(config->prescaler + 1U) / (uint64_t)tim_clk;
	/* One timer tick is min_tick ns; the 16-bit ARR allows at most
	 * 0x10000 ticks per period, and each period is one symbol.
	 */
	uint64_t max_tick64 = (uint64_t)min_tick * 0x10000U;
	uint32_t max_tick = max_tick64 > UINT32_MAX ? UINT32_MAX : (uint32_t)max_tick64;

	*caps = (struct pulse_io_caps){
		.tx_min_chunk_symbols = 1U,
		.tx_max_streaming = config->buf_size,
		.modes = PULSE_IO_MODE_SYMBOL,
		.min_tick_ns = min_tick,
		.max_tick_ns = max_tick,
		.max_duration_ticks = 0x10000U,
		.tx_loop_max = 0U,
		.tx_channel_mask = BIT(0),
		.rx_channel_mask = 0U,
		.cell_duty_bits = 0U,
		.num_channels = 1U,
		.supports_tx = true,
		/* Rx is possible, but not implemented */
		.supports_rx = false,
	};

	return 0;
}

static int pulse_io_stm32_channel_get(const struct device *dev,
				      uint8_t channel_idx,
				      struct pulse_io_channel **chan)
{
	struct pulse_io_stm32_data *data = dev->data;
	int ret = -EBUSY;

	if (channel_idx != 0U) {
		return -ENODEV;
	}

	K_SPINLOCK(&data->busy_spinlock) {
		if (!data->in_use) {
			data->in_use = true;
			*chan = (struct pulse_io_channel *)&data->chan;
			ret = 0;
		}
	}

	return ret;
}

static int pulse_io_stm32_stop_locked(const struct device *dev)
{
	const struct pulse_io_stm32_config *config = dev->config;
	struct pulse_io_stm32_data *data = dev->data;

	if (data->busy) {
		(void)dma_stop(config->dma_dev, config->dma_channel_tx);
		(void)dma_stop(config->dma_dev, config->dma_channel_arr);
		LL_TIM_DisableDMAReq_UPDATE(config->timer);
		LL_TIM_DisableCounter(config->timer);
		data->busy = false;
		k_sem_reset(&data->done);
	}

	/* Return the line to the configured idle level. */
	pulse_io_stm32_set_idle(dev, data->idle_high);

	return 0;
}

static int pulse_io_stm32_channel_release(const struct device *dev,
					  struct pulse_io_channel *chan)
{
	struct pulse_io_stm32_data *data = dev->data;
	int ret;

	ret = pulse_io_stm32_check_chan(dev, chan);
	if (ret != 0) {
		return ret;
	}

	/* Stop any in-flight transfer and restore idle before releasing. */
	K_SPINLOCK(&data->busy_spinlock) {
		(void)pulse_io_stm32_stop_locked(dev);
	}

	data->in_use = false;
	data->configured = false;

	return 0;
}

static int pulse_io_stm32_channel_configure(const struct device *dev,
					    struct pulse_io_channel *chan,
					    const struct pulse_io_config *cfg)
{
	const struct pulse_io_stm32_config *config = dev->config;
	struct pulse_io_stm32_data *data = dev->data;
	TIM_TypeDef *timer = config->timer;
	const struct device *clk;
	uint32_t tim_clk;
	uint32_t arr;
	uint64_t tick_freq;
	int ret;

	ret = pulse_io_stm32_check_chan(dev, chan);
	if (ret != 0) {
		return ret;
	}

	if (cfg == NULL) {
		return -EINVAL;
	}

	if (cfg->dir != PULSE_IO_DIR_TX) {
		return -ENOTSUP;
	}

	if (cfg->mode != PULSE_IO_MODE_SYMBOL) {
		return -ENOTSUP;
	}

	if (cfg->resolution_hz == 0U) {
		return -EINVAL;
	}

	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	if (config->pclk_len < 2U) {
		LOG_ERR("timer clock source not specified");
		return -EINVAL;
	}

	ret = clock_control_get_rate(clk, (clock_control_subsys_t)&config->pclken[1],
				     &tim_clk);
	if (ret < 0) {
		LOG_ERR("timer clock rate get failed (%d)", ret);
		return ret;
	}

	/* The timer update event fires every (PSC+1)*(ARR+1) timer ticks, so
	 * the per-tick frequency is tim_clk / ((PSC+1)*(ARR+1)). Solve for ARR
	 * given the requested resolution: the period is ARR+1 ticks.
	 */
	tick_freq = (uint64_t)tim_clk / (uint64_t)(config->prescaler + 1U);
	if (tick_freq < cfg->resolution_hz) {
		LOG_ERR("resolution %u Hz too high for %llu Hz tick"
			"(decrease st,prescaler)", cfg->resolution_hz, tick_freq);
		return -EINVAL;
	}
	arr = (uint32_t)(tick_freq / cfg->resolution_hz) - 1U;

	if (arr > 0xFFFFU) {
		LOG_ERR("timer auto-reload %u out of range for %u Hz"
			"(increase st,prescaler)", arr, cfg->resolution_hz);
		return -EINVAL;
	}

	if (!gpio_is_ready_dt(&config->gpio)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->gpio,
			cfg->idle_high ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW);
	if (ret != 0) {
		LOG_ERR("GPIO pin configure failed (%d)", ret);
		return ret;
	}

	pulse_io_stm32_set_idle(dev, cfg->idle_high);
	LL_TIM_SetPrescaler(timer, config->prescaler);
	/* Nominal period for the leading pipeline dummy periods; the real
	 * per-symbol periods are fed by the "arr" DMA stream.
	 */
	LL_TIM_SetAutoReload(timer, arr);
	if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE(timer)) {
		LL_TIM_SetCounterMode(timer, LL_TIM_COUNTERMODE_UP);
	}
	if (IS_TIM_CLOCK_DIVISION_INSTANCE(timer)) {
		LL_TIM_SetClockDivision(timer, LL_TIM_CLOCKDIVISION_DIV1);
	}
#ifdef IS_TIM_REPETITION_COUNTER_INSTANCE
	if (IS_TIM_REPETITION_COUNTER_INSTANCE(timer)) {
		LL_TIM_SetRepetitionCounter(timer, 0U);
	}
#endif
	LL_TIM_EnableARRPreload(timer);
	/* Generate an update event to immediately load PSC/ARR. */
	LL_TIM_GenerateEvent_UPDATE(timer);

	data->resolution_hz = (uint32_t)(tick_freq / (arr + 1U));
	data->idle_high = cfg->idle_high;
	data->dir = cfg->dir;
	data->configured = true;

	LOG_DBG("configured: tim_clk=%u presc=%u arr=%u -> %u Hz",
		tim_clk, config->prescaler, arr, data->resolution_hz);

	return 0;
}

static void pulse_io_stm32_dma_callback(const struct device *dma_dev,
					void *user_data, uint32_t channel,
					int status)
{
	const struct device *dev = (const struct device *) user_data;
	const struct pulse_io_stm32_config *config = dev->config;
	struct pulse_io_stm32_data *data = dev->data;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status != 0) {
		data->result = -EIO;
	} else {
		data->result = 0;
	}

	/* The "arr" stream finished: the last symbol's period has completed.
	 * Stop the timer and its update-DMA request.
	 */
	LL_TIM_DisableDMAReq_UPDATE(config->timer);
	LL_TIM_DisableCounter(config->timer);
	K_SPINLOCK(&data->busy_spinlock) {
		data->busy = false;
	}

	/* Return the line to the idle level. */
	pulse_io_stm32_set_idle(dev, data->idle_high);

	k_sem_give(&data->done);
}

/* Encode a SYMBOL-mode stream into per-symbol ARR words. Consecutive symbols
 * with the same level are merged (the circular BSRR pattern forces the level
 * to alternate every period). Returns the number of words written to buf
 * (merged symbol count + pipeline dummy words), or 0 on overflow.
 */
static size_t pulse_io_stm32_encode(const struct device *dev,
				    const struct pulse_symbol *symbols, size_t count,
				    uint32_t *buf, size_t cap)
{
	size_t n = 0U;

	/* cap must hold one word per merged symbol plus the trailing dummies. */
	if (count + PULSE_IO_STM32_PIPE_DUMMY_WORDS > cap) {
		LOG_ERR("DMA buffer overflow (%zu symbols, cap %zu)"
			"(need more buffer-size)", count, cap);
		return 0U;
	}

	for (size_t i = 0U; i < count; i++) {
		uint32_t duration = symbols[i].duration;

		if (duration == 0U) {
			duration = 1U;
		}
		if (duration > 0x10000U) {
			LOG_ERR("symbol %zu duration %u exceeds 16-bit ARR"
				" (max %u ticks)", i, symbols[i].duration, 0x10000U);
			return 0U;
		}

		/* Merge with the previous symbol if the level did not change:
		 * the circular BSRR pattern forces the level to alternate every
		 * period, so runs of equal level must be coalesced.
		 */
		if (n > 0U && symbols[i].level == symbols[i - 1].level) {
			uint32_t merged = buf[n - 1] + 1U + duration - 1U;

			if (merged > 0xFFFFU) {
				LOG_ERR("merged symbol duration exceeds 16-bit ARR");
				return 0U;
			}
			buf[n - 1] = merged;
			continue;
		}

		buf[n++] = duration - 1U;
	}

	/* Trailing dummy words so the ARR stream's completion callback fires
	 * only after the last symbol's period has fully elapsed.
	 */
	for (size_t i = 0U; i < PULSE_IO_STM32_PIPE_DUMMY_WORDS; i++) {
		buf[n++] = 0U;
	}

	ARG_UNUSED(dev);
	return n;
}

static int pulse_io_stm32_transmit_sync(const struct device *dev,
					struct pulse_io_channel *chan,
					const struct pulse_io_tx_req *req,
					k_timeout_t timeout)
{
	const struct pulse_io_stm32_config *config = dev->config;
	struct pulse_io_stm32_data *data = dev->data;
	size_t words;
	bool finished;
	int ret;

	ret = pulse_io_stm32_check_chan(dev, chan);
	if (ret != 0) {
		return ret;
	}

	if (!data->configured || req == NULL || req->symbols == NULL ||
	    req->count == 0U) {
		return -EINVAL;
	}

	if (req->loop_count != 0U) {
		return -ENOTSUP;
	}

	words = pulse_io_stm32_encode(dev, req->symbols, req->count,
				     data->buf, config->buf_size);
	if (words == 0U) {
		return -ENOMEM;
	}

	/* The circular BSRR pattern: word 0 drives the leading pipeline dummy
	 * period (idle level), word 1 the first symbol; the level then
	 * alternates every period. Merging in encode() guarantees the symbol
	 * levels strictly alternate, matching this pattern.
	 */
	uint32_t first_level = req->symbols[0].level;

	data->bsrr_pattern[0] = (first_level != 0U) ? low_word(config) : high_word(config);
	data->bsrr_pattern[1] = (first_level != 0U) ? high_word(config) : low_word(config);

	k_sem_reset(&data->done);

	/* Make sure the DMA engine sees the encoded buffer on cached cores. */
	sys_cache_data_flush_range(data->buf, words * sizeof(uint32_t));
	sys_cache_data_flush_range(data->bsrr_pattern, sizeof(data->bsrr_pattern));

	/* Stream "tx": circular, 2 words, memory -> BSRR. */
	memset(&data->block_tx, 0, sizeof(data->block_tx));
	data->block_tx.source_address = (uint32_t)(uintptr_t)data->bsrr_pattern;
	data->block_tx.dest_address   = (uint32_t)(uintptr_t)config->bsrr;
	data->block_tx.block_size = sizeof(data->bsrr_pattern);
	data->block_tx.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->block_tx.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	memset(&data->dma_cfg_tx, 0, sizeof(data->dma_cfg_tx));
	data->dma_cfg_tx.dma_slot = config->dma_slot_tx;
	data->dma_cfg_tx.channel_direction = MEMORY_TO_PERIPHERAL;
	data->dma_cfg_tx.channel_priority = config->dma_priority_tx;
	data->dma_cfg_tx.source_data_size = sizeof(uint32_t);
	data->dma_cfg_tx.dest_data_size = sizeof(uint32_t);
	data->dma_cfg_tx.source_burst_length = sizeof(uint32_t);
	data->dma_cfg_tx.dest_burst_length = sizeof(uint32_t);
	data->dma_cfg_tx.block_count = 1U;
	data->dma_cfg_tx.head_block = &data->block_tx;
	/* Circular mode comes from STM32_DMA_MODE_CYCLIC in the DT cell. */
	data->dma_cfg_tx.dma_callback = NULL;
	data->dma_cfg_tx.user_data = NULL;

	/* Stream "arr": linear, one word per symbol, memory -> TIMx_ARR. */
	memset(&data->block_arr, 0, sizeof(data->block_arr));
	data->block_arr.source_address = (uint32_t)(uintptr_t)data->buf;
	data->block_arr.dest_address   = (uint32_t)(uintptr_t)&config->timer->ARR;
	data->block_arr.block_size = words * sizeof(uint32_t);
	data->block_arr.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->block_arr.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	memset(&data->dma_cfg_arr, 0, sizeof(data->dma_cfg_arr));
	data->dma_cfg_arr.dma_slot = config->dma_slot_arr;
	data->dma_cfg_arr.channel_direction = MEMORY_TO_PERIPHERAL;
	data->dma_cfg_arr.channel_priority = config->dma_priority_arr;
	data->dma_cfg_arr.source_data_size = sizeof(uint32_t);
	data->dma_cfg_arr.dest_data_size = sizeof(uint32_t);
	data->dma_cfg_arr.source_burst_length = sizeof(uint32_t);
	data->dma_cfg_arr.dest_burst_length = sizeof(uint32_t);
	data->dma_cfg_arr.block_count = 1U;
	data->dma_cfg_arr.head_block = &data->block_arr;
	data->dma_cfg_arr.dma_callback = pulse_io_stm32_dma_callback;
	data->dma_cfg_arr.user_data = (void *)dev;

	ret = dma_config(config->dma_dev, config->dma_channel_tx, &data->dma_cfg_tx);
	if (ret != 0) {
		LOG_ERR("dma_config(tx) failed (%d)", ret);
		(void)dma_stop(config->dma_dev, config->dma_channel_tx);
		return ret;
	}
	ret = dma_config(config->dma_dev, config->dma_channel_arr, &data->dma_cfg_arr);
	if (ret != 0) {
		LOG_ERR("dma_config(arr) failed (%d)", ret);
		(void)dma_stop(config->dma_dev, config->dma_channel_tx);
		(void)dma_stop(config->dma_dev, config->dma_channel_arr);
		return ret;
	}

	ret = dma_start(config->dma_dev, config->dma_channel_tx);
	if (ret != 0) {
		LOG_ERR("dma_start(tx) failed (%d)", ret);
		(void)dma_stop(config->dma_dev, config->dma_channel_tx);
		(void)dma_stop(config->dma_dev, config->dma_channel_arr);
		return ret;
	}
	ret = dma_start(config->dma_dev, config->dma_channel_arr);
	if (ret != 0) {
		LOG_ERR("dma_start(arr) failed (%d)", ret);
		(void)dma_stop(config->dma_dev, config->dma_channel_tx);
		(void)dma_stop(config->dma_dev, config->dma_channel_arr);
		return ret;
	}

	/* Enable the timer update-DMA request and start the counter; each
	 * update event triggers one word on both streams.
	 */
	K_SPINLOCK(&data->busy_spinlock)
	{
		data->busy = true;
	}
	LL_TIM_SetCounter(config->timer, 0U);
	LL_TIM_EnableDMAReq_UPDATE(config->timer);
	LL_TIM_EnableCounter(config->timer);

	ret = k_sem_take(&data->done, timeout);
	if (ret != 0) {
		K_SPINLOCK(&data->busy_spinlock)
		{
			finished = !data->busy;
		}
		if (finished) {
			/* The DMA completed just as the timeout expired. */
			return data->result;
		}
		(void)dma_stop(config->dma_dev, config->dma_channel_tx);
		(void)dma_stop(config->dma_dev, config->dma_channel_arr);
		LL_TIM_DisableDMAReq_UPDATE(config->timer);
		LL_TIM_DisableCounter(config->timer);
		K_SPINLOCK(&data->busy_spinlock)
		{
			data->busy = false;
		}
		/* Return the line to the configured idle level. */
		pulse_io_stm32_set_idle(dev, data->idle_high);
		return -ETIMEDOUT;
	}

	return data->result;
}

static int pulse_io_stm32_receive_sync(const struct device *dev,
				      struct pulse_io_channel *chan,
				      const struct pulse_io_rx_req *req,
				      size_t *count, k_timeout_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
	ARG_UNUSED(req);
	ARG_UNUSED(count);
	ARG_UNUSED(timeout);

	return -ENOTSUP;
}

static int pulse_io_stm32_stop(const struct device *dev,
			      struct pulse_io_channel *chan)
{
	struct pulse_io_stm32_data *data = dev->data;
	int ret;

	ret = pulse_io_stm32_check_chan(dev, chan);
	if (ret != 0) {
		return ret;
	}

	K_SPINLOCK(&data->busy_spinlock)
	{
		(void)pulse_io_stm32_stop_locked(dev);
	}

	return 0;
}

static DEVICE_API(pulse_io, pulse_io_stm32_api) = {
	.get_capabilities = pulse_io_stm32_get_capabilities,
	.channel_get = pulse_io_stm32_channel_get,
	.channel_release = pulse_io_stm32_channel_release,
	.channel_configure = pulse_io_stm32_channel_configure,
	.transmit_sync = pulse_io_stm32_transmit_sync,
	.receive_sync = pulse_io_stm32_receive_sync,
	.stop = pulse_io_stm32_stop,
};

static int pulse_io_stm32_init(const struct device *dev)
{
	const struct pulse_io_stm32_config *config = dev->config;
	struct pulse_io_stm32_data *data = dev->data;
	const struct device *clk;
	int ret;

	k_sem_init(&data->done, 0, 1);

	if (!device_is_ready(config->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	/* Enable the timer clock. */
	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(clk, (clock_control_subsys_t)&config->pclken[0]);
	if (ret < 0) {
		LOG_ERR("timer clock enable failed (%d)", ret);
		return ret;
	}

	if (config->pclk_len > 1U) {
		ret = clock_control_configure(clk,
			(clock_control_subsys_t)&config->pclken[1], NULL);
		if (ret != 0) {
			LOG_ERR("timer clock source config failed (%d)", ret);
			return ret;
		}
	}

	if (!gpio_is_ready_dt(&config->gpio)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	/* Park the line low until a channel is configured with its idle
	 * level; a configured idle-high channel is set high in configure().
	 */
	ret = gpio_pin_configure_dt(&config->gpio, GPIO_OUTPUT_LOW);
	if (ret != 0) {
		LOG_ERR("GPIO pin configure failed (%d)", ret);
		return ret;
	}

	return 0;
}

#define PULSE_IO_STM32_INIT(inst)                                                   \
	BUILD_ASSERT(DT_INST_PROP(inst, st_prescaler) <= 0xFFFF,                        \
		     "st,prescaler must fit the 16-bit TIMx_PSC register");             \
\
	static const struct stm32_pclken                                            \
		pulse_io_stm32_pclken_##inst[DT_NUM_CLOCKS(                         \
			DT_PHANDLE(DT_DRV_INST(inst), timer))] =                    \
			STM32_DT_CLOCKS(DT_PHANDLE(DT_DRV_INST(inst), timer));      \
\
	static uint32_t pulse_io_stm32_buf_##inst                                   \
		[PULSE_IO_STM32_BUFSZ(inst)] __aligned(PULSE_IO_STM32_BUF_ALIGN);   \
\
	static struct pulse_io_stm32_data pulse_io_stm32_data_##inst = {            \
		.buf = pulse_io_stm32_buf_##inst,                                   \
	};                                                                          \
\
	static const struct pulse_io_stm32_config pulse_io_stm32_cfg_##inst = {     \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(                                \
			DT_PHANDLE(DT_DRV_INST(inst), timer)),                      \
		.pclken = pulse_io_stm32_pclken_##inst,                             \
		.pclk_len = DT_NUM_CLOCKS(DT_INST_PARENT(inst)),                    \
		.prescaler = DT_INST_PROP(inst, st_prescaler),                     \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, tx)),      \
		.dma_channel_tx = DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel),    \
		.dma_slot_tx = DT_INST_DMAS_CELL_BY_NAME(inst, tx, slot),          \
		.dma_priority_tx = STM32_DMA_CONFIG_PRIORITY(                      \
			DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel_config)),       \
		.dma_channel_arr = DT_INST_DMAS_CELL_BY_NAME(inst, arr, channel),  \
		.dma_slot_arr = DT_INST_DMAS_CELL_BY_NAME(inst, arr, slot),        \
		.dma_priority_arr = STM32_DMA_CONFIG_PRIORITY(                     \
			DT_INST_DMAS_CELL_BY_NAME(inst, arr, channel_config)),      \
		.gpio = GPIO_DT_SPEC_INST_GET(inst, gpios),                         \
		.bsrr = (volatile uint32_t *)                                       \
			&((GPIO_TypeDef *)DT_REG_ADDR(DT_GPIO_CTLR(                 \
				DT_DRV_INST(inst), gpios)))->BSRR,                  \
		.mask = BIT(DT_GPIO_PIN(DT_DRV_INST(inst), gpios)),                 \
		.buf_size = PULSE_IO_STM32_BUFSZ(inst),                             \
	};                                                                          \
\
	DEVICE_DT_INST_DEFINE(inst, pulse_io_stm32_init, NULL,                      \
			      &pulse_io_stm32_data_##inst,                          \
			      &pulse_io_stm32_cfg_##inst, POST_KERNEL,              \
			      CONFIG_PULSE_IO_INIT_PRIORITY,                        \
			      &pulse_io_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(PULSE_IO_STM32_INIT)
