/*
 * Copyright (c) 2021, Yonatan Schachter
 * Copyright (c) 2021, Matias Nitsche
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <drivers/uart.h>
#undef KHZ
#undef MHZ

#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/regs/intctrl.h>
#include <uart_raspberrypi_pio_tx.pio.h>
#include <uart_raspberrypi_pio_rx.pio.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(uart_pio, LOG_LEVEL_DBG);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT raspberrypi_rp2_pio_uart

#define SM_TX 0
#define SM_RX 1

struct uart_rpi_config {
	pio_hw_t *const pio_dev;
	uint32_t baudrate;
  uint32_t pin;
};

enum uart_rpi_state { IDLE = 0, STATE_RX, STATE_TX };

struct uart_rpi_data
{
  bool tx_enabled;
  bool rx_enabled;
  uart_callback_t user_cb;
  void* user_data;

  int dma_chan;
  uint8_t* current_transfer_buf;
  size_t current_transfer_size;
  uint8_t* next_transfer_buf;
  size_t next_transfer_size;
  enum uart_rpi_state state;
};

static void uart_rpi_pio_rx(const struct device *dev)
{
  struct uart_rpi_data* data = dev->data;
  const struct uart_rpi_config* config = dev->config;

  if (!data->rx_enabled)
  {
    pio_sm_set_enabled(config->pio_dev, SM_TX, false);
    pio_sm_clear_fifos(config->pio_dev, SM_RX);
    pio_sm_restart(config->pio_dev, SM_RX);
    pio_sm_set_consecutive_pindirs(config->pio_dev, SM_RX, config->pin, 1, false);
    pio_sm_set_enabled(config->pio_dev, SM_RX, true);
    data->rx_enabled = true;
    data->tx_enabled = false;
  }
}

static void uart_rpi_pio_tx(const struct device *dev)
{
  struct uart_rpi_data* data = dev->data;
  const struct uart_rpi_config* config = dev->config;  

  if (!data->tx_enabled)
  {      
    pio_sm_set_enabled(config->pio_dev, SM_RX, false);
    gpio_disable_pulls(config->pin);
    pio_gpio_init(config->pio_dev, config->pin);
    pio_sm_set_pins_with_mask(config->pio_dev, SM_TX, 1u << config->pin, 1u << config->pin);
    pio_sm_set_pindirs_with_mask(config->pio_dev, SM_TX, 1u << config->pin, 1u << config->pin);

    pio_sm_clear_fifos(config->pio_dev, SM_TX);
    pio_sm_restart(config->pio_dev, SM_TX);    
    pio_sm_set_enabled(config->pio_dev, SM_TX, true);
    data->tx_enabled = true;
    data->rx_enabled = false;
  }
}

static int uart_rpi_poll_in(const struct device *dev, unsigned char *c)
{
  const struct uart_rpi_config* config = dev->config;
  struct uart_rpi_data* data = dev->data;

  if (data->state != IDLE)
  {
    /* only allow direct RX if no DMA in progress */
    return -EBUSY;
  }

  /* Prepare PIO state machines for RX if necessary */
  
  uart_rpi_pio_rx(dev);

  /* Attempt to receive */

  if (pio_sm_is_rx_fifo_empty(config->pio_dev, SM_RX))
  {
    return -1;
  }

  /* get pointer to most significant byte of word (data is shifted to right)
   * on reception */

  io_rw_8* rxfifo_shift = (io_rw_8*)&config->pio_dev->rxf[SM_RX] + 3;
  *c = *rxfifo_shift;

  return 0;
}

static void uart_rpi_poll_out(const struct device *dev, unsigned char c)
{
  const struct uart_rpi_config* config = dev->config;
  struct uart_rpi_data* data = dev->data;

  if (data->state != IDLE)
  {
    /* only allow direct TX if no DMA in progress */
    return;
  }

  /* Prepare PIO state machines for TX if necessary */
  
  uart_rpi_pio_tx(dev);

  /* Perform send (write to least significant byte of FIFO word) */

  io_rw_8* txfifo_shift = (io_rw_8*)&config->pio_dev->txf[SM_TX];
  *txfifo_shift = c;
}

static void uart_rpi_txinit(const struct uart_rpi_config* config)
{
  pio_hw_t* const pio = config->pio_dev;
  uint32_t pin = config->pin;
  
  /* Configure PIO program */

  uint offset = pio_add_program(pio, &uart_tx_program);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, offset + uart_tx_wrap_target, offset + uart_tx_wrap);
  sm_config_set_sideset(&c, 2, true, false);
  // OUT shifts to right, no autopull
  sm_config_set_out_shift(&c, true, false, 32);

  // We are mapping both OUT and side-set to the same pin, because sometimes
  // we need to assert user data onto the pin (with OUT) and sometimes
  // assert constant values (start/stop bit)
  sm_config_set_out_pins(&c, pin, 1);
  sm_config_set_sideset_pins(&c, pin);

  // We only need TX in this SM, so get an 8-deep FIFO!
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

  // SM transmits 1 bit per 8 execution cycles.
  float div = (float)clock_get_hz(clk_sys) / (8 * config->baudrate);
  sm_config_set_clkdiv(&c, div);

  pio_sm_init(pio, SM_TX, offset, &c);
}

static void uart_rpi_rxinit(const struct uart_rpi_config* config)
{
  pio_hw_t* const pio = config->pio_dev;
  uint32_t pin = config->pin;
  
  /* Configure PIO program */

  uint offset = pio_add_program(pio, &uart_rx_program);

  pio_sm_config c = uart_rx_program_get_default_config(offset);
  sm_config_set_in_pins(&c, pin); // for WAIT, IN
  sm_config_set_jmp_pin(&c, pin); // for JMP
  // Shift to right, autopull disabled
  sm_config_set_in_shift(&c, true, false, 32);
  // Deeper FIFO as we're not doing any TX in this SM
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  // SM transmits 1 bit per 8 execution cycles.
  float div = (float)clock_get_hz(clk_sys) / (8 * config->baudrate);
  sm_config_set_clkdiv(&c, div);
  
  pio_sm_init(pio, SM_RX, offset, &c);
}

static int uart_rpi_callback_set(const struct device* dev,
  uart_callback_t callback, void* user_data)
{
  struct uart_rpi_data* data = dev->data;

  if (data->state != IDLE)
  {
    return -EBUSY;
  }
  else
  {
    data->user_cb = callback;
    data->user_data = user_data;
    return 0;
  }
}

static void dma_handler(const void *arg)
{
  const struct device* dev = arg;
  struct uart_rpi_data* data = dev->data;
  struct uart_event evt;

  /* clear interrupt flag */
  
  dma_hw->ints0 = 1u << data->dma_chan;

  /* Invoke user callback */

  if (data->user_cb)
  {
    if (data->state == STATE_TX)
    {
      data->state = IDLE;
      evt.type = UART_TX_DONE;
      evt.data.tx.buf = data->current_transfer_buf;
      evt.data.tx.len = data->current_transfer_size;
      (*data->user_cb)(dev, &evt, data->user_data);
    }
    else if (data->state == STATE_RX)
    {
      /* signal transfer complete */
      
      evt.type = UART_RX_RDY;
      evt.data.rx.buf = data->current_transfer_buf;
      evt.data.rx.len = data->current_transfer_size;
      evt.data.rx.offset = 0;
      (*data->user_cb)(dev, &evt, data->user_data);

      evt.type = UART_RX_BUF_RELEASED;
      evt.data.rx_buf.buf = data->current_transfer_buf;
      (*data->user_cb)(dev, &evt, data->user_data);

      data->current_transfer_size = data->next_transfer_size;
      data->current_transfer_buf = data->next_transfer_buf;

      if (data->current_transfer_size == 0)
      {
        /* no more transfers pending */

        dma_channel_abort(data->dma_chan);

        data->state = IDLE;
        evt.type = UART_RX_DISABLED;
        (*data->user_cb)(dev, &evt, data->user_data);
      }
      else
      {
        /* request next buffer */

        evt.type = UART_RX_BUF_REQUEST;
        (*data->user_cb)(dev, &evt, data->user_data);
      }
    }
  }
}

static int uart_rpi_tx(const struct device* dev, const uint8_t* buf, size_t len,
  int32_t timeout)
{
  const struct uart_rpi_config* config = dev->config;
  struct uart_rpi_data* data = dev->data;

  if (timeout != 0)
  {
    return -ENOTSUP;
  }

  if (data->state != IDLE)
  {
    return -EBUSY;
  }

  /* Start TX state machine, if needed */

  uart_rpi_pio_tx(dev);

  /* Setup and start DMA */
  
  dma_channel_config c = dma_channel_get_default_config(data->dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_write_increment(&c, false);
  channel_config_set_dreq(&c, pio_get_dreq(config->pio_dev, SM_TX, true));

  data->state = STATE_TX;
  data->current_transfer_buf = (uint8_t*)buf;
  data->current_transfer_size = len;

  dma_channel_configure(data->dma_chan, &c, (io_rw_8*)&config->pio_dev->txf[SM_TX], buf, len, true);
  dma_channel_set_trans_count(data->dma_chan, 0, false);

  return 0;
}

static int uart_rpi_tx_abort(const struct device* dev)
{
  struct uart_rpi_data* data = dev->data;
  
  if (data->state == STATE_TX)
  {
    /* abort transfer, leave state machine enabled in case TX is reinitiated */
    
    dma_channel_abort(data->dma_chan);

    data->state = IDLE;

    if (data->user_cb)
    {
      struct uart_event evt;
      evt.type = UART_TX_ABORTED;
      evt.data.tx.buf = data->current_transfer_buf;
      evt.data.tx.len = data->current_transfer_size;
      (*data->user_cb)(dev, &evt, data->user_data);
    }

    return 0;
  }
  else
  {
    return -EFAULT;
  }
}

static int uart_rpi_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
  int32_t timeout)
{
  const struct uart_rpi_config* config = dev->config;  
  struct uart_rpi_data* data = dev->data;

  if (timeout != 0)
  {
    return -ENOTSUP;
  }

  if (data->state != IDLE)
  {
    return -EBUSY;
  }

   /* Start RX state machine, if needed */

  uart_rpi_pio_rx(dev);

  /* Setup and start DMA */
  
  dma_channel_config c = dma_channel_get_default_config(data->dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, pio_get_dreq(config->pio_dev, SM_RX, false));

  data->state = STATE_RX;
  data->current_transfer_size = len;
  data->current_transfer_buf = buf;
  data->next_transfer_size = 0;
  data->next_transfer_buf = NULL;

  LOG_DBG("RX enabling (len: %d)", len);

  irq_disable(DMA_IRQ_0);

  /* receive from most significant byte of FIFO word */
  
  dma_channel_configure(data->dma_chan, &c, buf, (io_rw_8*)&config->pio_dev->rxf[SM_RX] + 3, len, true);
  dma_channel_set_trans_count(data->dma_chan, 0, false);

  if (data->user_cb)
  {
    struct uart_event evt;
    evt.type = UART_RX_BUF_REQUEST;
    (*data->user_cb)(dev, &evt, data->user_data);
  }
  irq_enable(DMA_IRQ_0);

  return 0;
}

static int uart_rpi_rx_buf_rsp(const struct device* dev, uint8_t* buf, size_t len)
{
  struct uart_rpi_data* data = dev->data;

  if (data->state != STATE_RX)
  {
    /* ensure we're called in the expected state */
    return -EINVAL;
  }

  if (len == 0)
  {
    return -EINVAL;
  }

  data->next_transfer_size = len;
  data->next_transfer_buf = buf;

  /* live-update of DMA (but don't trigger) */
  
  dma_channel_set_trans_count(data->dma_chan, len, false);
  dma_channel_set_write_addr(data->dma_chan, buf, false);

  return 0;
}

static int uart_rpi_rx_disable(const struct device* dev)
{
  struct uart_rpi_data* data = dev->data;
  
  if (data->state == STATE_RX)
  {
    /* abort transfer, leave state machine enabled in case RX is reinitiated */
    
    dma_channel_abort(data->dma_chan);

    data->state = IDLE;

    if (data->user_cb)
    {
      struct uart_event evt;
      
      /* UART_RX_RDY if any data received */
      evt.type = UART_RX_BUF_RELEASED;
      evt.data.rx_buf.buf = data->current_transfer_buf;
      (*data->user_cb)(dev, &evt, data->user_data);

      evt.type = UART_RX_DISABLED;
      (*data->user_cb)(dev, &evt, data->user_data);
    }

    return 0;
  }
  else
  {
    return -EFAULT;
  }
}

DEVICE_DECLARE(DT_DRV_INST(0));

static int uart_rpi_init(const struct device* dev)
{
  const struct uart_rpi_config* config = dev->config;
  struct uart_rpi_data* data = dev->data;

  /* Initial DMA setup */

  data->dma_chan = dma_claim_unused_channel(true);
  if (data->dma_chan < 0) LOG_ERR("no free DMA chan");
  
  dma_channel_set_irq0_enabled(data->dma_chan, true);

  IRQ_CONNECT(DMA_IRQ_0, 3, dma_handler, DEVICE_GET(DT_DRV_INST(0)), 0);
  irq_enable(DMA_IRQ_0);

  /* Setup pin */

  gpio_pull_up(config->pin);

  /* Setup PIO state machines */

  uart_rpi_txinit(config);
  uart_rpi_rxinit(config);

  return 0;
}

static const struct uart_driver_api uart_rpi_driver_api = {
#ifdef CONFIG_UART_ASYNC_API
	.callback_set	  = uart_rpi_callback_set,
	.tx		          = uart_rpi_tx,
	.tx_abort	      = uart_rpi_tx_abort,
	.rx_enable	    = uart_rpi_rx_enable,
	.rx_buf_rsp	    = uart_rpi_rx_buf_rsp,
	.rx_disable	    = uart_rpi_rx_disable,
#endif
  .poll_in        = uart_rpi_poll_in,
	.poll_out       = uart_rpi_poll_out,
};

#define RPI_UART_PIO_INIT(idx)						\
	static const struct uart_rpi_config uart_rpi_cfg_##idx = {	\
		.pio_dev = (pio_hw_t *const)(CONCAT(CONCAT(PIO, DT_INST_PROP(idx, pio)),_BASE)),	\
		.baudrate = DT_INST_PROP(idx, current_speed),		\
		.pin = DT_INST_PROP(idx, pin),			\
	};								\
									\
	static struct uart_rpi_data uart_rpi_data_##idx;		\
									\
	DEVICE_DEFINE(DT_DRV_INST(idx), DT_INST_LABEL(idx), &uart_rpi_init, \
				device_pm_control_nop,			\
				&uart_rpi_data_##idx, 			\
				&uart_rpi_cfg_##idx, PRE_KERNEL_1,	\
				CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
				&uart_rpi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RPI_UART_PIO_INIT)
