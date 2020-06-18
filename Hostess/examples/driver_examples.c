/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

static void button_on_PA16_pressed(void)
{
}

/**
 * Example of using EXTERNAL_IRQ_0
 */
void EXTERNAL_IRQ_0_example(void)
{

	ext_irq_register(PIN_PA16, button_on_PA16_pressed);
}

/**
 * Example of using TARGET_IO to write "Hello World" using the IO abstraction.
 */
void TARGET_IO_example(void)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&TARGET_IO, &io);
	usart_sync_enable(&TARGET_IO);

	io_write(io, (uint8_t *)"Hello World!", 12);
}

/**
 * Example of using SPI_0 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_SPI_0[12] = "Hello World!";

static void complete_cb_SPI_0(const struct spi_s_async_descriptor *const desc)
{
	/* Transfer completed */
}

void SPI_0_example(void)
{
	struct io_descriptor *io;
	spi_s_async_get_io_descriptor(&SPI_0, &io);

	spi_s_async_register_callback(&SPI_0, SPI_S_CB_TX, (FUNC_PTR)complete_cb_SPI_0);
	spi_s_async_enable(&SPI_0);
	io_write(io, example_SPI_0, 12);
}

void delay_example(void)
{
	delay_ms(5000);
}

static struct timer_task TIMER_0_task1, TIMER_0_task2;
/**
 * Example of using TIMER_0.
 */
static void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
}

static void TIMER_0_task2_cb(const struct timer_task *const timer_task)
{
}

void TIMER_0_example(void)
{
	TIMER_0_task1.interval = 100;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
	TIMER_0_task2.interval = 200;
	TIMER_0_task2.cb       = TIMER_0_task2_cb;
	TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_add_task(&TIMER_0, &TIMER_0_task2);
	timer_start(&TIMER_0);
}

#ifndef GET_STATUS
#define GET_STATUS 0
#define CLEAR_FEATURE 1
#define SET_FEATURE 3
#define GET_DESCRIPTOR 6

#define PORT_CONNECTION 0
#define PORT_RESET 4
#define PORT_LOW_SPEED 9
#define PORT_HIGH_SPEED 10
#define C_PORT_CONNECTION 16
#define C_PORT_RESET 20

#define DEVICE 1
#endif

/** USB request instance */
static union {
	uint8_t  u8[8];
	uint16_t u16[4];
} usb_req;

/** USB status returned by requests */
static volatile int8_t usb_rh_ftr_change = -1;

/** USB pipe 0 */
static struct usb_h_pipe *usb_p0 = NULL;

/** Request end */
static volatile bool usb_req_end = false;

/** Device descriptor buffer (18 bytes) */
static uint8_t *dev_desc;

/**
 * Break here in debug mode
 */
static void _debug_break(void)
{
	__asm("BKPT #0");
}

/** Get device descriptor
 */
static void _get_device_descriptor(void)
{
	int32_t rc;
	usb_req.u8[0]  = 0x80;              /* bmRequestType 10000000B */
	usb_req.u8[1]  = GET_DESCRIPTOR;    /* bRequest */
	usb_req.u16[1] = (DEVICE << 8) | 0; /* wValue: type (high) | index (low) */
	usb_req.u16[2] = 0;                 /* wIndex */
	usb_req.u16[3] = 18;                /* wLength: 18 */
	rc             = usb_h_control_xfer(usb_p0, usb_req.u8, dev_desc, 18, 5000);
	if (rc != ERR_NONE) {
		_debug_break();
	}
}

/** Pipe 0 transfer callback
 *  \param p0 Pointer to pipe 0 instance
 */
void _p0_xfer_cb(struct usb_h_pipe *p0)
{
	if (p0->x.ctrl.status == USB_H_OK) {
		usb_req_end = true;
	}
}

/** Create pipe 0 for default endpoint transfer
 */
static void _create_pipe0(uint8_t max_pkt_size)
{
	uint8_t speed = usb_h_rh_check_status(&USB_0_inst, 1, PORT_LOW_SPEED)
	                    ? (usb_h_rh_check_status(&USB_0_inst, 1, PORT_HIGH_SPEED) ? USB_SPEED_HS : USB_SPEED_FS)
	                    : USB_SPEED_LS;
	/* Create pipe for default endpoint access */
	usb_p0 = usb_h_pipe_allocate(&USB_0_inst, 0, 0, max_pkt_size, 0, 0, speed, false);
	if (!usb_p0) {
		_debug_break();
		return;
	}
	/* Register pipe callback */
	usb_h_pipe_register_callback(usb_p0, _p0_xfer_cb);
}

/** Root hub change callback
 */
static void _rh_change_cb(struct usb_h_desc *drv, uint8_t port, uint8_t ftr)
{
	/* Use usb_h_rh_check_status() to check ftr state and handle it */
	usb_rh_ftr_change = ftr;
}

/**
 * Example of using USB_0 to get USB device descriptor
 * Use USB analyzer or set break point at following code to check returned descriptor
 * \code
 *      } else if (run_once[0]) {
 *          // Set your breakpoints below to check returned descriptor
 *          run_once[0] = false;
 *      }
 * \endcode
 * \param[in] dev_desc_buf Pointer to 18 bytes buffer to fill device descriptor
 */
void USB_0_example(void *dev_desc_buf)
{
	struct usb_h_desc *usb         = &USB_0_inst;
	volatile bool      run_once[1] = {true};

	dev_desc = (uint8_t *)dev_desc_buf;

	/* Register root hub change callback */
	usb_h_register_callback(usb, USB_H_CB_ROOTHUB_CHANGE, (FUNC_PTR)_rh_change_cb);

	usb_h_enable(usb);
	/* Check device connection */
	while (1) {
		if (usb_rh_ftr_change >= 0) {
			bool ftr_state = usb_h_rh_check_status(usb, 1, usb_rh_ftr_change);
			if (usb_rh_ftr_change == PORT_CONNECTION) {
				if (ftr_state) {
					/* Connected, try to reset */
					usb_h_rh_reset(usb, 1);
				} else {
					/* Disconnected */
					if (usb_p0) {
						usb_h_pipe_free(usb_p0);
						usb_p0 = NULL;
					}
					run_once[0] = true;
				}
			}
			if (usb_rh_ftr_change == PORT_RESET) {
				/* Read device descriptor */
				if (!usb_p0) {
					_create_pipe0(64);
				}
				_get_device_descriptor();
			}
			usb_rh_ftr_change = -1;
		}
		/* Display request results */
		if (usb_req_end) {
			usb_req_end = false;
			if (usb_p0->x.ctrl.count < 18) {
				/* Modify pipe0 parameters */
				int32_t rc = usb_h_pipe_set_control_param(usb_p0, 0, 0, dev_desc[7], USB_SPEED_FS);
				if (rc != USB_H_OK) {
					_debug_break();
				} else {
					_get_device_descriptor();
				}
			} else if (run_once[0]) {
				/* Set your breakpoints below to check returned descriptor */
				run_once[0] = false;
			}
		}
	}
}
