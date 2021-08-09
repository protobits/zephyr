/*
 * Copyright (c) 2021, Pete Johanson
 * Copyright (c) 2017 Christer Weinigel.
 * Copyright (c) 2017, I-SENSE group of ICCS
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <soc.h>
#include <string.h>
#include <usb/usb_device.h>
#include <sys/util.h>
#include <hardware/regs/usb.h>
#include <hardware/structs/usb.h>
#include <hardware/resets.h>

#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(usb_dc_raspberrypi);

#define DT_DRV_COMPAT raspberrypi_rp2_usbd

#define USB_BASE_ADDRESS	DT_INST_REG_ADDR(0)
#define USB_IRQ			DT_INST_IRQ_BY_NAME(0, usbctrl, irq)
#define USB_IRQ_PRI		DT_INST_IRQ_BY_NAME(0, usbctrl, priority)
#define USB_NUM_BIDIR_ENDPOINTS	DT_INST_PROP(0, num_bidir_endpoints)
/* Size of a USB SETUP packet */
#define SETUP_SIZE 8

/* Helper macros to make it easier to work with endpoint numbers */
#define EP0_IDX 0
#define EP0_IN (EP0_IDX | USB_EP_DIR_IN)
#define EP0_OUT (EP0_IDX | USB_EP_DIR_OUT)

#define EP_MPS 64U

#define DATA_BUFFER_SIZE 64U

// Needed for pico-sdk
#ifndef typeof
#define typeof  __typeof__
#endif

#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

/* Endpoint state */
struct usb_dc_raspberrypi_ep_state {
	uint16_t ep_mps;		      /** Endpoint max packet size */
	enum usb_dc_ep_transfer_type ep_type; /** Endpoint type */
	uint8_t ep_stalled;		      /** Endpoint stall flag */
	usb_dc_ep_callback cb;		      /** Endpoint callback function */
	uint32_t read_offset;		      /** Current offset in read buffer */
	struct k_sem write_sem;		      /** Write boolean semaphore */
	io_rw_32 *endpoint_control;
	io_rw_32 *buffer_control;
	uint8_t *data_buffer;
	uint8_t next_pid;
};

/* Driver state */
struct usb_dc_raspberrypi_state {
	usb_dc_status_callback status_cb; /* Status callback */
	struct usb_dc_raspberrypi_ep_state out_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	struct usb_dc_raspberrypi_ep_state in_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	bool setup_available;
	bool should_set_address;
	uint8_t addr;
};

static struct usb_dc_raspberrypi_state usb_dc_raspberrypi_state;

/* Internal functions */

static struct usb_dc_raspberrypi_ep_state *usb_dc_raspberrypi_get_ep_state(uint8_t ep)
{
	struct usb_dc_raspberrypi_ep_state *ep_state_base;

	if (USB_EP_GET_IDX(ep) >= USB_NUM_BIDIR_ENDPOINTS) {
		return NULL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_state_base = usb_dc_raspberrypi_state.out_ep_state;
	} else {
		ep_state_base = usb_dc_raspberrypi_state.in_ep_state;
	}

	return ep_state_base + USB_EP_GET_IDX(ep);
}

static int usb_dc_raspberrypi_start_xfer(uint8_t ep, const void* data, size_t len)
{
	struct usb_dc_raspberrypi_ep_state* ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	uint32_t val = len | USB_BUF_CTRL_AVAIL;

	if (USB_EP_DIR_IS_IN(ep))
	{
		/* If writing to host, put data in place */

		if (data)
		{
			memcpy(ep_state->data_buffer, data, len);
			val |= USB_BUF_CTRL_FULL;			
		}
	}
	else
	{
		ep_state->read_offset = 0;

		/* TODO: this seems to be needed to get the setup stage working,
		 * otherwise I get DATA SEQ errors. This should be improved. */

		if (USB_EP_GET_IDX(ep) == 0)
		{
			ep_state->next_pid = 1;
		}
	}

	LOG_DBG("xfer ep %d len %d pid: %d", ep, len, ep_state->next_pid);
	val |= ep_state->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
  ep_state->next_pid ^= 1u;

	/* TODO: check when this delay is actually needed */
#if 0
	*ep_state->buffer_control = val & ~USB_BUF_CTRL_AVAIL;
	
	// 12 cycle delay.. (should be good for 48*12Mhz = 576Mhz)
	// Don't need delay in host mode as host is in charge
	__asm volatile (
			"b 1f\n"
			"1: b 1f\n"
			"1: b 1f\n"
			"1: b 1f\n"
			"1: b 1f\n"
			"1: b 1f\n"
			"1:\n"
				: : : "memory");

	/* Start the transfer */

	*ep_state->buffer_control = val | USB_BUF_CTRL_AVAIL;
#else
	*ep_state->buffer_control = val;
#endif

	return 0;
}

static void usb_dc_raspberrypi_handle_setup(void)
{
	struct usb_dc_raspberrypi_ep_state* ep = usb_dc_raspberrypi_get_ep_state(EP0_OUT);
	LOG_DBG("");

	usb_dc_raspberrypi_state.setup_available = true;

	/* Reset PID to 1 for EP0 IN */
	usb_dc_raspberrypi_get_ep_state(EP0_IN)->next_pid = 1;

	ep->cb(EP0_OUT, USB_DC_EP_SETUP);
}

static void usb_dc_raspberrypi_handle_buff_status(void)
{
	struct usb_dc_raspberrypi_ep_state *ep;
	enum usb_dc_ep_cb_status_code status_code;
	uint8_t status = usb_hw->buf_status;
	unsigned int i;
	unsigned int bit = 1U;

	LOG_DBG("status: %d", status);

	for (i = 0U; status && i < USB_NUM_BIDIR_ENDPOINTS * 2; i++) {
		if (status & bit) {
			usb_hw_clear->buf_status = bit;
			bool in = !(i & 1U);
			uint8_t ep_addr = (i >> 1U) | (in ? USB_EP_DIR_IN : USB_EP_DIR_OUT);
			ep = usb_dc_raspberrypi_get_ep_state(ep_addr);
			status_code = in ? USB_DC_EP_DATA_IN : USB_DC_EP_DATA_OUT;

			LOG_DBG("buff ep %i in? %i", (i >> 1), in);

			if (i == 0 && in && usb_dc_raspberrypi_state.should_set_address)
			{
				usb_dc_raspberrypi_state.should_set_address = false;
				usb_hw->dev_addr_ctrl = usb_dc_raspberrypi_state.addr;
			}

			if (in)
			{
				k_sem_give(&ep->write_sem);
			}

			ep->cb(ep_addr, status_code);

			status &= ~bit;
		}

		bit <<= 1U;
	}
}

static void usb_dc_raspberrypi_isr(const void *arg)
{
	// USB interrupt handler
	uint32_t status = usb_hw->ints;
	uint32_t handled = 0;

	// Setup packet received
	if (status & USB_INTS_SETUP_REQ_BITS) {
		handled |= USB_INTS_SETUP_REQ_BITS;
		usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
		usb_dc_raspberrypi_handle_setup();
	}

	// Buffer status, one or more buffers have completed
	if (status & USB_INTS_BUFF_STATUS_BITS) {
		handled |= USB_INTS_BUFF_STATUS_BITS;
		usb_dc_raspberrypi_handle_buff_status();
	}

	// Connection status update
	if (status & USB_INTS_DEV_CONN_DIS_BITS) {
		LOG_DBG("buf %u ep %u", *usb_dc_raspberrypi_get_ep_state(0x81)->buffer_control, *usb_dc_raspberrypi_get_ep_state(0x81)->endpoint_control);
		handled |= USB_INTS_DEV_CONN_DIS_BITS;
		usb_hw_clear->sie_status = USB_SIE_STATUS_CONNECTED_BITS;
		usb_dc_raspberrypi_state.status_cb(usb_hw->sie_status & USB_SIE_STATUS_CONNECTED_BITS ? USB_DC_CONNECTED : USB_DC_DISCONNECTED, NULL);
	}

	// Bus is reset
	if (status & USB_INTS_BUS_RESET_BITS) {
		int i;
		
		LOG_WRN("BUS RESET");
		handled |= USB_INTS_BUS_RESET_BITS;
		usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
		usb_hw->dev_addr_ctrl = 0;

		/* The DataInCallback will never be called at this point for any pending
		 * transactions. Reset the IN semaphores to prevent perpetual locked state.
		 * */
		
		for (i = 0; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
			k_sem_give(&usb_dc_raspberrypi_state.in_ep_state[i].write_sem);
		}

		usb_dc_raspberrypi_state.status_cb(USB_DC_RESET, NULL);
	}

	#if 0
	if (status & USB_INTS_EP_STALL_NAK_BITS)
	{
		handled |= USB_INTS_EP_STALL_NAK_BITS;
		if (usb_hw->ep_nak_stall_status & USB_EP_STATUS_STALL_NAK_EP1_IN_BITS)
		{
			LOG_DBG("EP1 NAK (%d)", usb_hw->ep_nak_stall_status);
		}			
		usb_hw_clear->ep_nak_stall_status = 0xFFFFFFFF;
	}
	#endif

	if (status & USB_INTS_ERROR_DATA_SEQ_BITS)
	{
		LOG_WRN("data seq");
		usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
		handled |= USB_INTS_ERROR_DATA_SEQ_BITS;
	}

	if (status ^ handled)
	{
		LOG_ERR("unhandled IRQ: 0x%x", (uint)(status ^ handled));
	}
}

void usb_dc_raspberrypi_init_bidir_endpoint(uint8_t i)
{
	usb_dc_raspberrypi_state.out_ep_state[i].buffer_control = &usb_dpram->ep_buf_ctrl[i].out;
	usb_dc_raspberrypi_state.in_ep_state[i].buffer_control = &usb_dpram->ep_buf_ctrl[i].in;

	if (i != EP0_IDX) {
		usb_dc_raspberrypi_state.out_ep_state[i].endpoint_control = &usb_dpram->ep_ctrl[i - 1].out;
		usb_dc_raspberrypi_state.in_ep_state[i].endpoint_control = &usb_dpram->ep_ctrl[i - 1].in;

		usb_dc_raspberrypi_state.out_ep_state[i].data_buffer = &usb_dpram->epx_data[((i - 1) * 2 + 1) * DATA_BUFFER_SIZE];
		usb_dc_raspberrypi_state.in_ep_state[i].data_buffer = &usb_dpram->epx_data[((i - 1) * 2) * DATA_BUFFER_SIZE];
	} else {
		usb_dc_raspberrypi_state.out_ep_state[i].data_buffer = &usb_dpram->ep0_buf_a[0];
		usb_dc_raspberrypi_state.in_ep_state[i].data_buffer = &usb_dpram->ep0_buf_a[0];

	}

	k_sem_init(&usb_dc_raspberrypi_state.in_ep_state[i].write_sem, 1, 1);
}

static int usb_dc_raspberrypi_init(void)
{
	unsigned int i;

	// Reset usb controller
	reset_block(RESETS_RESET_USBCTRL_BITS);
	unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
	
	// Clear any previous state in dpram just in case
	memset(usb_dpram, 0, sizeof(*usb_dpram)); // <1>
	
	// Mux the controller to the onboard usb phy
	usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
	
	// Force VBUS detect so the device thinks it is plugged into a host
	usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
	
	// Enable the USB controller in device mode.
	usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
	
	// Enable an interrupt per EP0 transaction
	usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS /*| USB_SIE_CTRL_EP0_INT_NAK_BITS*/; // <2>
	
	// Enable interrupts for when a buffer is done, when the bus is reset,
	// and when a setup packet is received, and device connection status
	usb_hw->inte =
		USB_INTS_BUFF_STATUS_BITS |
		USB_INTS_BUS_RESET_BITS |
	  USB_INTS_DEV_CONN_DIS_BITS |
		USB_INTS_SETUP_REQ_BITS | /*USB_INTS_EP_STALL_NAK_BITS |*/
		USB_INTS_ERROR_BIT_STUFF_BITS |
		USB_INTS_ERROR_CRC_BITS |
		USB_INTS_ERROR_DATA_SEQ_BITS |
		USB_INTS_ERROR_RX_OVERFLOW_BITS |
		USB_INTS_ERROR_RX_TIMEOUT_BITS;

	// Set up endpoints (endpoint control registers)
	// described by device configuration
	// usb_setup_endpoints();
	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		usb_dc_raspberrypi_init_bidir_endpoint(i);
	}
	
	IRQ_CONNECT(USB_IRQ, USB_IRQ_PRI,
		    usb_dc_raspberrypi_isr, 0, 0);
	irq_enable(USB_IRQ);

	// Present full speed device by enabling pull up on DP
	usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;

	return 0;
}

/* Zephyr USB device controller API implementation */

int usb_dc_attach(void)
{
	int ret;

	LOG_DBG("");

	ret = usb_dc_raspberrypi_init();
	if (ret) {
		return ret;
	}

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	ep_state->cb = cb;

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	LOG_DBG("");

	usb_dc_raspberrypi_state.status_cb = cb;
}

int usb_dc_set_address(const uint8_t addr)
{
	LOG_DBG("addr %u (0x%02x)", addr, addr);

	usb_dc_raspberrypi_state.should_set_address = true;
	usb_dc_raspberrypi_state.addr = addr;

	return 0;
}

int usb_dc_ep_start_read(uint8_t ep, size_t len)
{
	int ret;

	LOG_DBG("ep 0x%02x len %d", ep, len);

	/* we flush EP0_IN by doing a 0 length receive on it */
	if (!USB_EP_DIR_IS_OUT(ep) && (ep != EP0_IN || len)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	if (len > EP_MPS) {
		len = EP_MPS;
	}
	
	ret = usb_dc_raspberrypi_start_xfer(ep, NULL, len);
	
	return ret;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data * const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	LOG_DBG("ep %x, mps %d, type %d", cfg->ep_addr, cfg->ep_mps,
		cfg->ep_type);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
		LOG_ERR("invalid endpoint configuration");
		return -1;
	}

	if (ep_idx > (USB_NUM_BIDIR_ENDPOINTS - 1)) {
		LOG_ERR("endpoint index/address out of range");
		return -1;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const ep_cfg)
{
	uint8_t ep = ep_cfg->ep_addr;
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("ep 0x%02x, previous ep_mps %u, ep_mps %u, ep_type %u",
		ep_cfg->ep_addr, ep_state->ep_mps, ep_cfg->ep_mps,
		ep_cfg->ep_type);

	ep_state->ep_mps = ep_cfg->ep_mps;
	ep_state->ep_type = ep_cfg->ep_type;

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);
	uint8_t val;

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}
	
	if (!ep_state->endpoint_control) {
		usb_hw_set->ep_stall_arm = USB_EP_DIR_IS_OUT(ep) ? USB_EP_STALL_ARM_EP0_OUT_BITS : USB_EP_STALL_ARM_EP0_IN_BITS ;
	} else {
		val = *ep_state->endpoint_control;
		val |= USB_BUF_CTRL_STALL;

		*ep_state->endpoint_control = val;
	}

	ep_state->ep_stalled = 1U;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);
	uint8_t val;

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	if (!ep_state->endpoint_control) {
		usb_hw_clear->ep_stall_arm = USB_EP_DIR_IS_OUT(ep) ? USB_EP_STALL_ARM_EP0_OUT_BITS : USB_EP_STALL_ARM_EP0_IN_BITS ;
	} else {
		val = *ep_state->endpoint_control;
		val &= ~USB_BUF_CTRL_STALL;

		*ep_state->endpoint_control = val;
	}

	ep_state->ep_stalled = 0U;
	ep_state->read_offset = 0U;

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state || !stalled) {
		return -EINVAL;
	}

	*stalled = ep_state->ep_stalled;

	return 0;
}

static inline uint32_t usb_dc_ep_raspberrypi_buffer_offset(volatile uint8_t *data_buffer)
{
	// TODO: Bits 0-5 are ignored by the controller so make sure these are 0
	return (uint32_t)data_buffer ^ (uint32_t)usb_dpram;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("ep 0x%02x (id: %d) -> type %d", ep, USB_EP_GET_IDX(ep), ep_state->ep_type);

	/* clear buffer state (EP0 starts with PID=1 for setup phase) */
	
	*ep_state->buffer_control = (USB_EP_GET_IDX(ep) == 0 ? USB_BUF_CTRL_DATA1_PID : 0);

	// EP0 doesn't have an endpoint_control
	if (ep_state->endpoint_control)
	{
		uint32_t val = EP_CTRL_ENABLE_BITS
			| EP_CTRL_INTERRUPT_PER_BUFFER /*| (USB_EP_GET_IDX(ep) == 1 ? EP_CTRL_INTERRUPT_ON_NAK : 0)*/
			| (ep_state->ep_type << EP_CTRL_BUFFER_TYPE_LSB)
			| usb_dc_ep_raspberrypi_buffer_offset(ep_state->data_buffer);

		*ep_state->endpoint_control = val;
	}

	if (USB_EP_DIR_IS_OUT(ep) /*&& ep != EP0_OUT*/) {
		return usb_dc_ep_start_read(ep, EP_MPS);
	}
	#if 0
	else
	{
		usb_dc_raspberrypi_start_xfer(ep, NULL, ep_state->ep_mps);
	}
	#endif

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}
	
	// EP0 doesn't have an endpoint_control
	if (!ep_state->endpoint_control) {
		return 0;
	}

	uint8_t val = *ep_state->endpoint_control;
	val &= ~EP_CTRL_ENABLE_BITS;

	*ep_state->endpoint_control = val;

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
		    const uint32_t data_len, uint32_t * const ret_bytes)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);
	uint32_t len = data_len;
	int ret = 0;

	LOG_DBG("ep 0x%02x, len %u", ep, data_len);

	if (!ep_state || !USB_EP_DIR_IS_IN(ep)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	if (ep == EP0_IN && len > USB_MAX_CTRL_MPS) {
		len = USB_MAX_CTRL_MPS;
	}
	else if (len > ep_state->ep_mps) {
		len = ep_state->ep_mps;
	}

	ret = k_sem_take(&ep_state->write_sem, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Unable to get write lock (%d)", ret);
		return -EAGAIN;
	}

	if (!k_is_in_isr()) {
		irq_disable(USB_IRQ);
	}

	ret = usb_dc_raspberrypi_start_xfer(ep, data, len);
	
	if (ret < 0) {
		LOG_ERR("xfer failed (%d)", ret);
		k_sem_give(&ep_state->write_sem);
		ret = -EIO;
	}

	if (!k_is_in_isr()) {
		irq_enable(USB_IRQ);
	}

	if (!ret && ret_bytes) {
		*ret_bytes = len;
	}

	return ret;
}

uint32_t usb_dc_raspberrypi_get_ep_buffer_len(const uint8_t ep)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);
	uint32_t buf_ctl = *ep_state->buffer_control;

	return buf_ctl & USB_BUF_CTRL_LEN_MASK;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len,
			uint32_t *read_bytes)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);
	uint32_t read_count;

	if (!ep_state) {
		LOG_ERR("Invalid Endpoint %x", ep);
		return -EINVAL;
	}

	if (!USB_EP_DIR_IS_OUT(ep)) { /* check if OUT ep */
		LOG_ERR("Wrong endpoint direction: 0x%02x", ep);
		return -EINVAL;
	}

	if (usb_dc_raspberrypi_state.setup_available)
	{
		read_count = SETUP_SIZE;
	}
	else
	{
		read_count = usb_dc_raspberrypi_get_ep_buffer_len(ep) - ep_state->read_offset;
	}
	
	LOG_DBG("ep 0x%02x, %u bytes, %u+%u, %p", ep, max_data_len,
		ep_state->read_offset, read_count, data);

	if (data) {
		read_count = MIN(read_count, max_data_len);

		if (usb_dc_raspberrypi_state.setup_available)
		{
			memcpy(data, (const void*)&usb_dpram->setup_packet, read_count);
		}
		else
		{
			memcpy(data, ep_state->data_buffer +
				ep_state->read_offset, read_count);
		}

		ep_state->read_offset += read_count;
	}
	else if (max_data_len) {
		LOG_ERR("Wrong arguments");
	}

	if (read_bytes) {
		*read_bytes = read_count;
	}

	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)

{
	struct usb_dc_raspberrypi_ep_state* ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	if (!ep_state || !USB_EP_DIR_IS_OUT(ep)) { /* Check if OUT ep */
		LOG_ERR("Not valid endpoint: %02x", ep);
		return -EINVAL;
	}

	size_t bytes_received = (usb_dc_raspberrypi_state.setup_available ?
		SETUP_SIZE : usb_dc_raspberrypi_get_ep_buffer_len(ep));

	usb_dc_raspberrypi_state.setup_available = false;

	/* If no more data in the buffer, start a new read transaction.
	 */
	LOG_DBG("received %d offset: %d", bytes_received, ep_state->read_offset);
	if (bytes_received == ep_state->read_offset) {
		return usb_dc_ep_start_read(ep, EP_MPS);
	}

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t * const read_bytes)
{	
	if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
		return -EINVAL;
	}

	if (!max_data_len) {
		return 0;
	}

	if (usb_dc_ep_read_continue(ep) != 0) {
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_flush(const uint8_t ep)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	struct usb_dc_raspberrypi_ep_state *ep_state = usb_dc_raspberrypi_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	return ep_state->ep_mps;
}

int usb_dc_detach(void)
{
	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_reset(void)
{
	LOG_ERR("Not implemented");

	return 0;
}
