/**
 * \file
 *
 * \brief SAM USB HPL
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include <compiler.h>
#include <hal_atomic.h>
#include <hpl_usb.h>

#include "hpl_usb_host.h"

#include <hpl_usb_config.h>
#include <string.h>
#include <utils_assert.h>

#define hri_usbhost_is_syncing(a, b) hri_usb_is_syncing(a, b)
#define hri_usbhost_wait_for_sync(a, b) hri_usb_wait_for_sync(a, b)
#define hri_usbhost_get_CTRLA_reg(a, b) hri_usb_get_CTRLA_reg(a, b)
#define hri_usbhost_clear_CTRLA_ENABLE_bit(a) hri_usb_clear_CTRLA_ENABLE_bit(a)
#define hri_usbhost_read_CTRLA_reg(a) hri_usb_read_CTRLA_reg(a)
#define hri_usbhost_write_CTRLA_reg(a, b) hri_usb_write_CTRLA_reg(a, b)
#define hri_usbhost_write_DESCADD_reg(a, b) hri_usb_write_DESCADD_reg(a, b)

/**
 * \brief Dummy callback function
 * \return Always false.
 */
static bool _dummy_func_no_return(uint32_t unused0, uint32_t unused1)
{
	(void)unused0;
	(void)unused1;
	return false;
}

/**
 * \brief Load USB calibration value from NVM
 */
static void _usb_load_calib(void)
{
#define NVM_USB_PAD_TRANSN_POS 45
#define NVM_USB_PAD_TRANSN_SIZE 5
#define NVM_USB_PAD_TRANSP_POS 50
#define NVM_USB_PAD_TRANSP_SIZE 5
#define NVM_USB_PAD_TRIM_POS 55
#define NVM_USB_PAD_TRIM_SIZE 3
	Usb *    hw = USB;
	uint32_t pad_transn
	    = (*((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRANSN_POS / 32)) >> (NVM_USB_PAD_TRANSN_POS % 32))
	      & ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);
	uint32_t pad_transp
	    = (*((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRANSP_POS / 32)) >> (NVM_USB_PAD_TRANSP_POS % 32))
	      & ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);
	uint32_t pad_trim = (*((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRIM_POS / 32)) >> (NVM_USB_PAD_TRIM_POS % 32))
	                    & ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);
	if (pad_transn == 0x1F) {
		pad_transn = 5;
	}
	if (pad_transp == 0x1F) {
		pad_transp = 29;
	}
	if (pad_trim == 0x7) {
		pad_trim = 5;
	}

	hw->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN(pad_transn) | USB_PADCAL_TRANSP(pad_transp) | USB_PADCAL_TRIM(pad_trim);

	hw->DEVICE.QOSCTRL.bit.CQOS = 3;
	hw->DEVICE.QOSCTRL.bit.DQOS = 3;
}

/** Timeout between control data packets : 500ms */
#define USB_CTRL_DPKT_TIMEOUT (500)
/** Timeout of status packet : 50ms */
#define USB_CTRL_STAT_TIMEOUT (50)

/**
 * @brief      Private USB host controller driver data structure
 */
struct _usb_h_prvt {
	UsbHostDescriptor desc_table[CONF_USB_H_NUM_PIPE_SP];

	/** Pointer to pipe pool start address */
	struct usb_h_pipe *pipe_pool;
	/** Pipes to unfreeze after wakeup */
	uint16_t pipes_unfreeze;
	/** Delayed suspend time in ms */
	int8_t suspend_start;
	/** Delayed resume time in ms */
	int8_t resume_start;
	/** Control transfer request user count */
	int8_t n_ctrl_req_user;
	/** SOF user count (callback, suspend, resume, ctrl request) */
	int8_t n_sof_user;
	/** Pipe pool size in number of pipes */
	uint8_t pipe_pool_size;
};
struct _usb_h_prvt USB_HOST_INSTANCE_prvt;

struct usb_h_pipe USB_HOST_INSTANCE_pipe[CONF_USB_H_NUM_PIPE_SP];

uint8_t usb_ctrl_buffer[64];

static struct usb_h_desc *_usb_h_dev = NULL;

/**
 * @brief      Initialize the private data for Host Controller Driver
 *
 * @param[out] prvt            Pointer to the private data instance
 * @param[in]  pipe_pool       The pipe pool
 * @param[in]  pipe_pool_size  The pipe pool size
 */
static inline void _usbhcd_prvt_init(struct _usb_h_prvt *prvt, struct usb_h_pipe *pipe_pool, uint8_t pipe_pool_size)
{
	prvt->pipe_pool      = pipe_pool;
	prvt->pipe_pool_size = pipe_pool_size;
}

/**
 * \internal
 * \brief End the transfer: set state to idle and invoke callback
 * \param pipe Pointer to pipe instance
 * \param code Status code
 */
static void _usb_h_end_transfer(struct usb_h_pipe *pipe, int32_t code);

/** \internal
 *  \brief Abort on going transfer
 *  \param pipe Pointer to pipe instance
 *  \param code Status code
 */
static void _usb_h_abort_transfer(struct usb_h_pipe *pipe, int32_t code);

/** \internal
 *  \brief Load transfer buffer, size and count
 *  \param pipe Pointer to pipe instance
 *  \param x_buf   Pointer to location to put transfer buffer pointer
 *  \param x_size  Pointer to location to put transfer size
 *  \param x_count Pointer to location to put transfer count
 */
static inline void _usb_h_load_x_param(struct usb_h_pipe *pipe, uint8_t **x_buf, uint32_t *x_size, uint32_t *x_count);

/** \internal
 *  \brief Save modified transfer count
 *  \param pipe Pointer to pipe instance
 *  \param x_count Pointer to location to put transfer count
 */
static inline void _usb_h_save_x_param(struct usb_h_pipe *pipe, uint32_t x_count);

/** \internal
 *  \brief Reset toggle to specific value for pipe transfer
 *  \param pipe Pointer to pipe instance
 *  \param tgl  Toggle value
 */
static void _usb_h_reset_tgl(struct usb_h_pipe *pipe, uint8_t tgl);

/** \brief Send ZLP OUT packet
 *  \param pipe Pointer to pipe instance
 */
static void _usb_h_out_zlp_ex(struct usb_h_pipe *pipe);

/** \brief Send OUT packet
 *  \param pipe Pointer to pipe instance
 *  \param buffer Pointer to transfer buffer
 *  \param size transfer size
 */
static void _usb_h_out_ex(struct usb_h_pipe *pipe, uint8_t *buffer, uint32_t size);

/** \internal
 *  \brief Start waiting IN packets
 *  \param pipe Pointer to pipe instance
 */
static void _usb_h_in_req(struct usb_h_pipe *pipe);

/** \brief OUT packet process
 *  \param pipe Pointer to pipe instance
 */
static void _usb_h_out(struct usb_h_pipe *pipe);

/** \brief IN packet process
 *  \param pipe Pointer to pipe instance
 */
static void _usb_h_in(struct usb_h_pipe *pipe);

/** \brief Send control setup packet
 *  \param pipe Pointer to pipe instance
 */
static void _usb_h_setup_ex(struct usb_h_pipe *pipe);

/** Look up table PSIZE -> size of bytes */
static const uint16_t psize_2_size[] = {8, 16, 32, 64, 128, 256, 512, 1024};

/** \internal
 *  \brief Convert bank size of bytes to PIPCFG.PSIZE
 *  \param[in] size Size of bytes
 */
static int8_t _usb_h_get_psize(uint16_t size)
{
	uint8_t i;
	for (i = 0; i < sizeof(psize_2_size) / sizeof(uint16_t); i++) {
		/* Size should be exactly PSIZE values */
		if (size <= psize_2_size[i]) {
			return i;
		}
	}
	return 7;
}

/** \internal
 *  \brief Apply pipe configuration to physical pipe
 *  \param     drv      Pointer to driver instance
 *  \param[in] phy      Physical pipe number (index)
 *  \param[in] dev      Device address
 *  \param[in] ep       Endpoint address
 *  \param[in] type     Transfer type
 *  \param[in] size     Endpoint max packet size
 *  \param[in] n_bank   Number of banks
 *  \param[in] interval Interval of ms
 *  \param[in] speed    Transfer speed
 *  \return \c true if pipe configuration is OK
 */
static bool _usb_h_pipe_configure(struct usb_h_desc *drv, uint8_t phy, uint8_t dev, uint8_t ep, uint8_t type,
                                  uint16_t size, uint8_t n_bank, uint8_t interval, uint8_t speed)
{
	bool     dir = ep & 0x80;
	uint32_t cfg = USB_HOST_PCFG_PTYPE(type + 1) | (n_bank == 1 ? 0 : USB_HOST_PCFG_BK)
	               | (type == 0 /* CTRL */ ? 0 : (dir ? USB_HOST_PCFG_PTOKEN(1) : USB_HOST_PCFG_PTOKEN(2)));
	struct _usb_h_prvt *pd                                   = (struct _usb_h_prvt *)drv->prvt;
	pd->desc_table[phy].HostDescBank[0].CTRL_PIPE.bit.PDADDR = dev;
	pd->desc_table[phy].HostDescBank[0].CTRL_PIPE.bit.PEPNUM = ep & 0xF;
	pd->desc_table[phy].HostDescBank[0].PCKSIZE.bit.SIZE     = _usb_h_get_psize(size);

	hri_usbhost_write_PCFG_reg(drv->hw, phy, cfg);
	hri_usbhost_write_BINTERVAL_reg(drv->hw, phy, interval);

	return true;
}

/** \internal
 *  \brief Reset pipes
 *  \param     drv        Pointer to driver instance
 *  \param[in] warm_reset Handles runtime USB reset
 */
static void _usb_h_reset_pipes(struct usb_h_desc *drv, bool warm_reset)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	struct usb_h_pipe * p;
	uint8_t             i;
	/* Reset pipes */
	for (i = 0; i < pd->pipe_pool_size; i++) {
		p = &pd->pipe_pool[i];
		if (warm_reset) {
			/* Skip free pipes */
			if (p->x.general.state == USB_H_PIPE_S_FREE) {
				continue;
			}
			/* Restore physical pipe configurations */
			_usb_h_pipe_configure(drv, i, p->dev, p->ep, p->type, p->max_pkt_size, p->bank, p->interval, p->speed);
			/* Abort transfer (due to reset) */
			if (p->type == 0 /* Control */) {
				/* Force a callback for control endpoints */
				p->x.general.state = USB_H_PIPE_S_SETUP;
			}
			_usb_h_end_transfer(p, USB_H_RESET);
		} else {
			p->done            = (usb_h_pipe_cb_xfer_t)_dummy_func_no_return;
			p->x.general.state = USB_H_PIPE_S_FREE;
		}
	}
	if (warm_reset) {
		return;
	}
}

/** \internal
 *  \brief Add one Control Request user
 *  \param drv Pointer to driver instance
 */
static inline void _usb_h_add_req_user(struct usb_h_desc *drv)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	pd->n_ctrl_req_user++;
}

/** \internal
 *  \brief Remove one Control Request user
 *  \param drv Pointer to driver instance
 */
static inline void _usb_h_rm_req_user(struct usb_h_desc *drv)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	if (pd->n_ctrl_req_user) {
		pd->n_ctrl_req_user--;
	}
}

/** \internal
 *  \brief Add one SOF IRQ user and enable SOF interrupt
 *  \param drv Pointer to driver instance
 */
static inline void _usb_h_add_sof_user(struct usb_h_desc *drv)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	pd->n_sof_user++;
	hri_usbhost_set_INTEN_HSOF_bit(drv->hw);
}

/** \internal
 *  \brief Remove one SOF IRQ user
 *  \param drv Pointer to driver instance
 */
static inline void _usb_h_rm_sof_user(struct usb_h_desc *drv)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	if (pd->n_sof_user) {
		pd->n_sof_user--;
	}
}

/** \internal
 *  \brief Manage delayed suspend
 *  \param drv Pointer to driver instance
 *  \param pd Pointer to private data instance
 */
static inline void _usb_h_delayed_suspend(struct usb_h_desc *drv, struct _usb_h_prvt *pd)
{
	if (--pd->resume_start == 0) {
		_usb_h_rm_sof_user(drv); /* SOF user: delayed resume */

		/* Restore pipes unfrozen */
		for (uint8_t pi = 0; pi < CONF_USB_H_NUM_PIPE_SP; pi++) {
			if (pd->pipes_unfreeze & (1 << pi)) {
				hri_usbhost_set_PSTATUS_reg(drv->hw, pi, USB_HOST_PSTATUS_PFREEZE);
			}
		}
		/* Notify suspend */
		drv->rh_cb(drv, 1, 2); /* Port 1 PORT_SUSPEND changed */
	}
}

/** \internal
 *  \brief Manage delayed resume
 *  \param drv Pointer to driver instance
 *  \param pd Pointer to private data instance
 */
static inline void _usb_h_delayed_resume(struct usb_h_desc *drv, struct _usb_h_prvt *pd)
{
	if (--pd->resume_start == 0) {
		_usb_h_rm_sof_user(drv); /* SOF user: delayed resume */

		/* Restore pipes unfrozen */
		for (uint8_t pi = 0; pi < CONF_USB_H_NUM_PIPE_SP; pi++) {
			if (pd->pipes_unfreeze & (1 << pi)) {
				hri_usbhost_clear_PSTATUS_reg(drv->hw, pi, USB_HOST_PSTATUS_PFREEZE);
			}
		}
		/* Notify resume */
		drv->rh_cb(drv, 1, 2); /* Port 1 PORT_SUSPEND changed */
	}
}

/** \internal
 *  \brief Manage control endpoints timeouts
 */
static inline void _usb_h_ctrl_timeout(struct usb_h_desc *drv, struct _usb_h_prvt *pd)
{
	uint8_t i;
	for (i = 0; i < CONF_USB_H_NUM_PIPE_SP; i++) {
		struct usb_h_pipe *p = &pd->pipe_pool[i];
		/* Skip none control or not busy pipes */
		if (p->x.general.state <= USB_H_PIPE_S_SETUP || p->type != 0 /* Control */) {
			continue;
		}
		/* Check timeouts */
		if (p->x.ctrl.pkt_timeout > 0) {
			p->x.ctrl.pkt_timeout--;
		}
		if (p->x.ctrl.req_timeout > 0) {
			p->x.ctrl.req_timeout--;
		}
		if (p->x.ctrl.pkt_timeout == 0 || p->x.ctrl.req_timeout == 0) {
			/* Stop request */
			_usb_h_abort_transfer(p, USB_H_TIMEOUT);
		}
	}
}

/** \internal
 *  \brief Manage periodic endpoints starts
 *  \param drv Pointer to driver instance
 *  \param pd Pointer to private data instance
 */
static inline void _usb_h_periodic_start(struct usb_h_desc *drv, struct _usb_h_prvt *pd)
{
	uint8_t i;
	for (i = 0; i < pd->pipe_pool_size; i++) {
		struct usb_h_pipe *p = &pd->pipe_pool[i];
		/* Skip none periodic or not busy pipes */
		if (!p->periodic_start) {
			continue;
		}
		_usb_h_rm_sof_user(drv); /* SOF User: periodic start */
		p->periodic_start = 0;
		hri_usbhost_clear_PSTATUS_reg(drv->hw, i, USB_HOST_PSTATUS_PFREEZE);
	}
}

/** \internal
 *  \brief Return pipe error and ACK errors
 *  \param drv Pointer to driver instance
 *  \param[in] pi  Pipe index
 */
static int32_t _usb_h_pipe_get_error(UsbHostDescriptor *hw, uint8_t bank)
{
	uint32_t error = hri_usbhostdescriptor_read_STATUS_PIPE_reg(hw, bank);
	hri_usbhostdescriptor_clear_STATUS_PIPE_reg(hw, bank, 0);
	switch (error
	        & (USB_HOST_STATUS_PIPE_DTGLER | USB_HOST_STATUS_PIPE_TOUTER | USB_HOST_STATUS_PIPE_PIDER
	           | USB_HOST_STATUS_PIPE_DAPIDER)) {
	case USB_HOST_STATUS_PIPE_DTGLER:
		return USB_H_ERR;
	case USB_HOST_STATUS_PIPE_TOUTER:
		return USB_H_TIMEOUT;
	case USB_HOST_STATUS_PIPE_PIDER:
	case USB_HOST_STATUS_PIPE_DAPIDER:
	default:
		return USB_H_ERR;
	}
}

/** \internal
 *  \brief Handle pipe interrupts
 *  \param drv Pointer to driver instance
 *  \param[in] isr Interrupt status
 */
static inline void _usb_h_handle_pipe(struct usb_h_desc *drv, uint32_t isr)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	struct usb_h_pipe * p;
	int8_t              pi = 31 - clz(isr & USB_HOST_PINTSMRY_MASK);
	uint32_t            pipisr, pipimr;

	if (pi < 0) {
		return;
	}
	p      = &pd->pipe_pool[pi];
	pipisr = hri_usbhost_read_PINTFLAG_reg(drv->hw, pi);
	pipimr = hri_usbhost_read_PINTEN_reg(drv->hw, pi);
	/* STALL */
	if (pipisr & USB_HOST_PINTFLAG_STALL) {
		hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_STALL);
		_usb_h_abort_transfer(p, USB_H_STALL);
		return;
	}

	/* Error */
	if (pipisr & USB_HOST_PINTFLAG_PERR) {
		hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_PERR);
		/* Get and ACK error */
		p->x.general.status = _usb_h_pipe_get_error(&(pd->desc_table[pi]), 0);
		_usb_h_abort_transfer(p, USB_H_ERR);
		return;
	}

	/* TRFAIL */
	if (pipisr & USB_HOST_PINTFLAG_TRFAIL) {
		hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_TRFAIL);
		uint8_t status_bk = hri_usbhostdescriptor_read_STATUS_BK_reg(&(pd->desc_table[pi]), 0);
		if (status_bk) {
			hri_usbhostdescriptor_clear_STATUS_BK_reg(&(pd->desc_table[pi]), 0, 0);
			/* Ignore ERRORFLOW and handle CRCERR */
			if (p->type != 0x1 /* ISO */ && status_bk == USB_HOST_STATUS_BK_ERRORFLOW) {
				/* Ignore ERRORFLOW on none ISO pipes */
			} else {
				p->x.general.status = USB_H_ERR;
				_usb_h_abort_transfer(p, USB_H_ERR);
			}
		}
		return;
	}

	/* TRCPT: transfer complete */
	if (pipisr & pipimr & USB_HOST_PINTFLAG_TRCPT(3)) {
		hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_TRCPT(3));
		/* TXOUT: packet sent */
		if (hri_usbhost_read_PCFG_PTOKEN_bf(drv->hw, pi) == 1) {
			_usb_h_in(p);
			return;
		} else {
			_usb_h_out(p);
			return;
		}
	}

	/* SETUP: packet sent */
	if (pipisr & pipimr & USB_HOST_PINTFLAG_TXSTP) {
		hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_TXSTP);
		hri_usbhost_clear_PINTEN_reg(drv->hw, pi, USB_HOST_PINTFLAG_TXSTP);
		/* Reset data toggle for DATA */
		hri_usbhost_set_PSTATUS_DTGL_bit(drv->hw, pi);
		/* Start DATA phase */
		if (p->x.ctrl.setup[0] & 0x80) { /* IN */
			p->x.ctrl.state = USB_H_PIPE_S_DATI;
			/* Start IN requests */
			_usb_h_in_req(p);
		} else {                                            /* OUT */
			if (p->x.ctrl.setup[6] || p->x.ctrl.setup[7]) { /* wLength */
				p->x.ctrl.state = USB_H_PIPE_S_DATO;
				/* Start OUT */
				_usb_h_out_ex(p, p->x.ctrl.data, p->x.ctrl.size);
			} else { /* No DATA phase */
				p->x.ctrl.state = USB_H_PIPE_S_STATI;
				/* Start IN ZLP request */
				_usb_h_in_req(p);
			}
		}
		return;
	}

	ASSERT(false); /* Error system */
}

/** \internal
 *  \brief Handle root hub changes
 *  \param drv Pointer to driver instance
 *  \param[in] isr Interrupt status
 */
static inline void _usb_h_handle_rhc(struct usb_h_desc *drv, uint32_t isr)
{
	struct _usb_h_prvt *pd  = (struct _usb_h_prvt *)drv->prvt;
	uint32_t            imr = hri_usbhost_read_INTEN_reg(drv->hw);

	/* Bus reset sent */
	if (isr & USB_HOST_INTFLAG_RST) {
		hri_usbhost_clear_INTFLAG_RST_bit(drv->hw);
		_usb_h_reset_pipes(drv, true);
		drv->rh_cb(drv, 1, 4); /* PORT_RESET: reset complete */
		return;
	} else if (isr & imr & USB_HOST_INTFLAG_DDISC) {
		/* Disconnect */
		hri_usbhost_clear_INTFLAG_reg(drv->hw, USB_HOST_INTFLAG_DDISC | USB_HOST_INTFLAG_DCONN);
		/* Disable disconnect interrupt.
		 * Disable wakeup/resumes interrupts,
		 * in case of disconnection during suspend mode
		 */
		hri_usbhost_clear_INTEN_reg(drv->hw,
		                            USB_HOST_INTFLAG_DDISC | USB_HOST_INTFLAG_WAKEUP | USB_HOST_INTFLAG_DNRSM
		                                | USB_HOST_INTFLAG_UPRSM);
		/* Stop reset signal, in case of disconnection during reset */
		hri_usbhost_clear_CTRLB_BUSRESET_bit(drv->hw);
		/* Enable connection and wakeup interrupts */
		hri_usbhost_clear_INTFLAG_reg(drv->hw,
		                              USB_HOST_INTFLAG_DCONN | USB_HOST_INTFLAG_WAKEUP | USB_HOST_INTFLAG_DNRSM
		                                  | USB_HOST_INTFLAG_UPRSM);
		hri_usbhost_set_INTEN_reg(drv->hw,
		                          USB_HOST_INTFLAG_DCONN | USB_HOST_INTFLAG_WAKEUP | USB_HOST_INTFLAG_DNRSM
		                              | USB_HOST_INTFLAG_UPRSM);
		pd->suspend_start = 0;
		pd->resume_start  = 0;
		drv->rh_cb(drv, 1, 0); /* PORT_CONNECTION: connect status changed */
		return;
	} else if (isr & imr & USB_HOST_INTFLAG_DCONN) {
		/* Reserve the CONN flag for connection check */
		hri_usbhost_clear_INTEN_reg(drv->hw, USB_HOST_INTFLAG_DCONN);
		/* Enable disconnection interrupt */
		hri_usbhost_clear_INTFLAG_DDISC_bit(drv->hw);
		hri_usbhost_set_INTEN_reg(drv->hw, USB_HOST_INTFLAG_DDISC);
		/* Enable SOF */
		hri_usbhost_set_CTRLB_SOFE_bit(drv->hw);
		pd->suspend_start = 0;
		pd->resume_start  = 0;
		drv->rh_cb(drv, 1, 0); /* PORT_CONNECTION: connect status changed */
		return;
	}
	/* Wake up to power */
	if ((isr & USB_HOST_INTFLAG_WAKEUP) && (imr & USB_HOST_INTFLAG_DCONN)) {
		hri_usbhost_clear_INTFLAG_reg(drv->hw, USB_HOST_INTFLAG_WAKEUP);
#if CONF_USB_H_VBUS_CTRL
		CONF_USB_H_VBUS_CTRL_FUNC(drv, 1, true);
#endif
	}
	/* Resume */
	if (isr & imr & (USB_HOST_INTFLAG_WAKEUP | USB_HOST_INTFLAG_UPRSM | USB_HOST_INTFLAG_DNRSM)) {
		hri_usbhost_clear_INTFLAG_reg(drv->hw,
		                              USB_HOST_INTFLAG_WAKEUP | USB_HOST_INTFLAG_UPRSM | USB_HOST_INTFLAG_DNRSM);
		hri_usbhost_clear_INTEN_reg(drv->hw, USB_HOST_INTFLAG_WAKEUP | USB_HOST_INTFLAG_UPRSM | USB_HOST_INTFLAG_DNRSM);
		hri_usbhost_set_INTEN_reg(drv->hw, USB_HOST_INTFLAG_RST | USB_HOST_INTFLAG_DDISC);

		/* Enable SOF */
		hri_usbhost_set_CTRLB_SOFE_bit(drv->hw);

		/* Wait 50ms before restarting transfer */
		pd->resume_start = 50;
		_usb_h_add_sof_user(drv);
		return;
	}
#if 0
	ASSERT(false); /* Unexpected interrupt */
#else
	/* Just ignore unexpected interrupts */
	hri_usbhost_clear_INTFLAG_reg(drv->hw, isr);
#endif
}

/** \internal
 *  \brief Handle SOF/MicroSOF interrupt
 *  \param drv Pointer to driver instance
 */
static inline void _usb_h_handle_sof(struct usb_h_desc *drv)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	hri_usbhost_clear_INTFLAG_HSOF_bit(drv->hw);
	/* Micro SOF */
	if (hri_usbhost_read_FNUM_MFNUM_bf(drv->hw)) {
		if (pd->resume_start <= 0 || pd->suspend_start <= 0) {
			/* No resume and suspend on going, notify Micro SOF */
			drv->sof_cb(drv);
		}
		return;
	}
	/* Manage periodic starts */
	_usb_h_periodic_start(drv, pd);

	/* Manage a delay to enter in suspend */
	if (pd->suspend_start > 0) {
		_usb_h_delayed_suspend(drv, pd);
		return; /* Abort SOF events */
	}
	/* Manage a delay to exit of suspend */
	if (pd->resume_start > 0) {
		_usb_h_delayed_resume(drv, pd);
		return; /* Abort SOF events */
	}
	/* Manage the timeout on control endpoints */
	_usb_h_ctrl_timeout(drv, pd);

	/* Notify SOF */
	drv->sof_cb(drv);

	/* Disable SOF interrupt if no SOF users */
	if (pd->n_sof_user <= 0) {
		hri_usbhost_clear_INTEN_HSOF_bit(drv->hw);
	}
}

/** \internal
 *  \brief USB Host interrupt handler
 *  \param[in] Pointer to driver instance
 */
static void _usb_h_handler(void)
{
	uint32_t isr = hri_usbhost_read_PINTSMRY_reg(_usb_h_dev->hw);
	/* Pipe interrupts */
	if (isr) {
		_usb_h_handle_pipe(_usb_h_dev, isr);
		return;
	}

	isr = hri_usbhost_read_INTFLAG_reg(_usb_h_dev->hw);
	/* SOF */
	if (isr & USB_HOST_INTFLAG_HSOF) {
		_usb_h_handle_sof(_usb_h_dev);
		return;
	}

	/* Reset sent, connect/disconnect, wake up */
	if (isr
	    & (USB_HOST_INTFLAG_RST | USB_HOST_INTFLAG_DCONN | USB_HOST_INTFLAG_DDISC | USB_HOST_INTFLAG_WAKEUP
	       | USB_HOST_INTFLAG_UPRSM | USB_HOST_INTFLAG_DNRSM)) {
		/* Root hub change detected */
		_usb_h_handle_rhc(_usb_h_dev, isr);
		return;
	}
}

int32_t _usb_h_init(struct usb_h_desc *drv, void *hw, void *prvt)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)&USB_HOST_INSTANCE_prvt;

	_usbhcd_prvt_init(
	    &USB_HOST_INSTANCE_prvt, USB_HOST_INSTANCE_pipe, sizeof(USB_HOST_INSTANCE_pipe) / sizeof(struct usb_h_pipe));

	ASSERT(drv && hw && pd->pipe_pool && pd->pipe_pool_size);

	if (!hri_usbhost_is_syncing(hw, USB_SYNCBUSY_SWRST)) {
		if (hri_usbhost_get_CTRLA_reg(hw, USB_CTRLA_ENABLE)) {
			hri_usbhost_clear_CTRLA_ENABLE_bit(hw);
			hri_usbhost_wait_for_sync(hw, USB_SYNCBUSY_ENABLE);
		}
		hri_usbhost_write_CTRLA_reg(hw, USB_CTRLA_SWRST);
	}
	hri_usbhost_wait_for_sync(hw, USB_SYNCBUSY_SWRST);

#if CONF_USB_H_VBUS_CTRL
	CONF_USB_H_VBUS_CTRL_FUNC(drv, 1, false);
#endif

	drv->sof_cb = (usb_h_cb_sof_t)_dummy_func_no_return;
	drv->rh_cb  = (usb_h_cb_roothub_t)_dummy_func_no_return;

	drv->prvt = pd;
	drv->hw   = hw;

	_usb_h_reset_pipes(drv, false);
	_usb_load_calib();

	pd->suspend_start   = 0;
	pd->resume_start    = 0;
	pd->pipes_unfreeze  = 0;
	pd->n_ctrl_req_user = 0;
	pd->n_sof_user      = 0;

	_usb_h_dev = drv;
	NVIC_EnableIRQ(USB_IRQn);
	hri_usbhost_write_CTRLA_reg(hw, USB_CTRLA_RUNSTDBY | USB_CTRLA_MODE_HOST);
	hri_usbhost_write_DESCADD_reg(hw, (uint32_t)pd->desc_table);
	hri_usbhost_write_CTRLB_reg(hw, USB_HOST_CTRLB_SPDCONF(0) | USB_HOST_CTRLB_VBUSOK);

	/* Force re-connection on initialization */
	hri_usbhost_write_INTEN_reg(hw, USB_HOST_INTENSET_DDISC | USB_HOST_INTENSET_WAKEUP);

	return USB_H_OK;
}

void _usb_h_deinit(struct usb_h_desc *drv)
{
	_usb_h_disable(drv);
	NVIC_DisableIRQ(USB_IRQn);
	NVIC_ClearPendingIRQ(USB_IRQn);
}

void _usb_h_enable(struct usb_h_desc *drv)
{
	ASSERT(drv && drv->hw);
	uint8_t ctrla;

	ctrla = hri_usbhost_read_CTRLA_reg(drv->hw);
	if ((ctrla & USB_CTRLA_ENABLE) == 0) {
		hri_usbhost_write_CTRLA_reg(drv->hw, ctrla | USB_CTRLA_ENABLE);
		hri_usbhost_wait_for_sync(drv->hw, USB_SYNCBUSY_ENABLE);
	}

	/* Clear all interrupts that may have been set by a previous host mode */
	hri_usbhost_clear_INTFLAG_reg(drv->hw, USB_HOST_INTFLAG_MASK);

#if CONF_USB_H_VBUS_CTRL
	CONF_USB_H_VBUS_CTRL_FUNC(drv, 1, true);
#endif

	/* Enable interrupts to detect connection */
	hri_usbhost_set_INTEN_reg(
	    drv->hw, USB_HOST_INTENSET_DCONN | USB_HOST_INTENSET_RST | USB_HOST_INTENSET_HSOF | USB_HOST_INTENSET_WAKEUP);
}

void _usb_h_disable(struct usb_h_desc *drv)
{
	ASSERT(drv && drv->hw);
	uint8_t ctrla;

	ctrla = hri_usbhost_read_CTRLA_reg(drv->hw);
	if (ctrla & USB_CTRLA_ENABLE) {
		hri_usbhost_write_CTRLA_reg(drv->hw, ctrla & ~USB_CTRLA_ENABLE);
		hri_usbhost_wait_for_sync(drv->hw, USB_SYNCBUSY_ENABLE);
	}
#if CONF_USB_H_VBUS_CTRL
	CONF_USB_H_VBUS_CTRL_FUNC(drv, 1, false);
#endif
}

int32_t _usb_h_register_callback(struct usb_h_desc *drv, enum usb_h_cb_type type, FUNC_PTR cb)
{
	FUNC_PTR f;
	ASSERT(drv && drv->prvt && drv->hw);

	f = (cb == NULL) ? (FUNC_PTR)_dummy_func_no_return : (FUNC_PTR)cb;
	switch (type) {
	case USB_H_CB_SOF:
		if ((FUNC_PTR)drv->sof_cb != f) {
			hal_atomic_t flags;
			atomic_enter_critical(&flags);
			drv->sof_cb = (usb_h_cb_sof_t)f;
			if (cb) {
				_usb_h_add_sof_user(drv); /* SOF user: callback */
			} else {
				_usb_h_rm_sof_user(drv); /* SOF user: callback */
			}
			atomic_leave_critical(&flags);
		}
		break;
	case USB_H_CB_ROOTHUB_CHANGE:
		drv->rh_cb = (usb_h_cb_roothub_t)f;
		break;
	default:
		return USB_H_ERR_ARG;
	}
	return USB_H_OK;
}

uint16_t _usb_h_get_frame_n(struct usb_h_desc *drv)
{
	ASSERT(drv && drv->hw);
	return hri_usbhost_read_FNUM_FNUM_bf(drv->hw);
}

uint8_t _usb_h_get_microframe_n(struct usb_h_desc *drv)
{
	ASSERT(drv && drv->hw);
	return hri_usbhost_read_FNUM_MFNUM_bf(drv->hw);
}

void _usb_h_suspend(struct usb_h_desc *drv)
{
	ASSERT(drv);
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	uint8_t             i;
	if (pd->n_ctrl_req_user) {
		/* Delay suspend after setup requests */
		pd->suspend_start = -1;
		return;
	}
	/* Save pipe freeze states and freeze pipes */
	pd->pipes_unfreeze = 0;
	for (i = 0; i < CONF_USB_H_NUM_PIPE_SP; i++) {
		/* Skip frozen pipes */
		if (hri_usbhost_get_PSTATUS_PFREEZE_bit(drv->hw, i)) {
			continue;
		}
		/* Log unfrozen pipes */
		pd->pipes_unfreeze |= 1 << i;
		/* Freeze it to suspend */
		hri_usbhost_set_PSTATUS_PFREEZE_bit(drv->hw, i);
	}
	/* Wait 3 SOFs before entering in suspend state */
	_usb_h_add_sof_user(drv); /* SOF user: delayed suspend */
	pd->suspend_start = 3;
}

void _usb_h_resume(struct usb_h_desc *drv)
{
	ASSERT(drv);
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	if (hri_usbhost_get_CTRLB_SOFE_bit(drv->hw)) {
		/* Currently in IDLE mode (!=Suspend) */
		if (pd->suspend_start != 0) {
			/* Suspend mode on going
			 * Stop it and start resume event */
			pd->suspend_start = 0;
			pd->resume_start  = 1;

			_usb_h_add_sof_user(drv); /* SOF user: delayed resume */
		}
		return;
	}

	_usb_h_add_sof_user(drv); /* SOF user: delayed resume */

	/* Enable SOF */
	hri_usbhost_set_CTRLB_SOFE_bit(drv->hw);
	/* Do resume to downstream */
	hri_usbhost_set_CTRLB_RESUME_bit(drv->hw);
	/* Force a wakeup interrupt to do delayed resume */
	hri_usbhost_set_INTEN_reg(drv->hw, USB_HOST_INTENSET_WAKEUP);
}

void _usb_h_rh_reset(struct usb_h_desc *drv, uint8_t port)
{
	Usb *hw = (Usb *)drv->hw;
	(void)port;
	hri_usbhost_set_CTRLB_BUSRESET_bit(hw);
}

void _usb_h_rh_suspend(struct usb_h_desc *drv, uint8_t port)
{
	(void)port;
	_usb_h_suspend(drv);
}

void _usb_h_rh_resume(struct usb_h_desc *drv, uint8_t port)
{
	(void)port;
	_usb_h_resume(drv);
}

bool _usb_h_rh_check_status(struct usb_h_desc *drv, uint8_t port, uint8_t ftr)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	Usb *               hw = (Usb *)drv->hw;

	uint16_t ctrlb   = hri_usbhost_read_CTRLB_reg(hw);
	uint8_t  status  = hri_usbhost_read_STATUS_reg(hw);
	uint32_t intflag = hri_usbhost_read_INTFLAG_reg(hw);

	switch (ftr) {
	case 0: /* CONNECTION */
	case 1: /* ENABLE */
	case 8: /* POWER */
		return (intflag & (USB_HOST_INTFLAG_DDISC | USB_HOST_INTFLAG_DCONN)) == USB_HOST_INTFLAG_DCONN;
	case 2: /* SUSPEND */
		if (pd->suspend_start) {
			return true;
		} else {
			return !(ctrlb & USB_HOST_CTRLB_SOFE);
		}
	case 4: /* RESET */
		return (ctrlb & USB_HOST_CTRLB_BUSRESET);
	case 9: /* LS */
		return (status & USB_HOST_STATUS_SPEED_Msk) == USB_HOST_STATUS_SPEED(2);
	case 10: /* HS */
	default:
		return false;
	}
}

/** \internal
 *  \brief Check if a physical pipe is control pipe
 *  \param[in] hw   Pointer to hardware instance
 *  \param[in] pipe Physical pipe number (index)
 */
static inline bool _usb_h_is_ctrl_pipe(void *hw, uint8_t pipe)
{
	uint32_t cfg = hri_usbhost_get_PCFG_reg(hw, pipe, USB_HOST_PCFG_PTYPE_Msk);
	return (cfg == USB_HOST_PCFG_PTYPE(1));
}

/** \internal
 *  \brief Return pipe index
 *  \param[in] pipe Pointer to pipe instance
 */
static int8_t _usb_h_pipe_i(struct usb_h_pipe *pipe)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)pipe->hcd->prvt;
	ASSERT(pipe >= pd->pipe_pool && pipe <= &pd->pipe_pool[pd->pipe_pool_size]);
	return (pipe - pd->pipe_pool);
}

/** \internal
 *  \brief Return allocated pipe index
 *  \param     drv Pointer to driver instance
 *  \param[in] dev Device address
 *  \param[in] ep  Endpoint address
 *  \return pipe index or -1 if error
 */
static int8_t _usb_h_find_pipe(struct usb_h_desc *drv, uint8_t dev, uint8_t ep)
{
	uint8_t             i;
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	for (i = 0; i < pd->pipe_pool_size; i++) {
		if (pd->pipe_pool[i].x.general.state == USB_H_PIPE_S_FREE) {
			/* Skip free pipes */
			continue;
		}
		/* Check pipe allocation */
		if (pd->pipe_pool[i].dev == dev && pd->pipe_pool[i].ep == ep) {
			return i;
		}
	}
	return -1;
}

/**
 *  \brief Return pipe index of a free pipe
 *  \param     drv     Pointer to driver instance
 *  \param[in] start_i Search start index
 *  \param[in] n_bank  Expected number of banks
 */
static int8_t _usb_h_find_free_pipe(struct usb_h_desc *drv, uint8_t start_i, uint8_t n_bank)
{
	uint8_t             i;
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	for (i = start_i; i < pd->pipe_pool_size; i++) {
		if (pd->pipe_pool[i].x.general.state != USB_H_PIPE_S_FREE) {
			continue;
		}
		return i;
	}
	return -1;
}

/** \internal
 *  \brief Return decoded interval in ms from bInterval
 *  \param[in] speed    USB speed
 *  \param[in] interval bInterval in endpoint descriptor
 */
static uint16_t _usb_h_int_interval(uint8_t speed, uint8_t interval)
{
	if (speed != USB_SPEED_HS) {
		return interval;
	}
	if (interval > 16) {
		interval = 16;
	}
	return (2 << (interval - 1));
}

/** \internal
 *  \brief Set device address of a physical pipe
 *  \param     hw       Pointer to hardware register base
 *  \param[in] pipe     Pipe number(index)
 *  \param[in] dev_addr Device address
 */
static void _usb_h_pipe_set_addr(struct usb_h_desc *drv, uint8_t pipe, uint8_t dev_addr)
{
	struct _usb_h_prvt *pd                                    = (struct _usb_h_prvt *)drv->prvt;
	pd->desc_table[pipe].HostDescBank[0].CTRL_PIPE.bit.PDADDR = dev_addr;
}

struct usb_h_pipe *_usb_h_pipe_allocate(struct usb_h_desc *drv, uint8_t dev, uint8_t ep, uint16_t max_pkt_size,
                                        uint8_t attr, uint8_t interval, uint8_t speed, bool minimum_rsc)
{
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
	struct usb_h_pipe * p;

	uint8_t      type       = attr & 0x3;
	uint8_t      n_bank     = ((max_pkt_size >> 11) & 0x3) + 1;
	uint16_t     mps        = max_pkt_size & 0x3FF;
	uint16_t     actual_mps = psize_2_size[_usb_h_get_psize(mps)];
	int8_t       pipe;
	hal_atomic_t flags;

	/* Calculate number of banks */
	if (n_bank > 1) {
		if (type != 1 /*ISO*/ && type != 3 /*INT*/) {
			return NULL; /* Invalid argument */
		}
	} else {
		if (!minimum_rsc && (type == 2 /* Bulk */ || type == 1 /* ISO */)) {
			/* Allocate more banks for performance */
			n_bank = 2;
		} else {
			/* In this case: n_bank = 1; high_bw = false; */
		}
	}
	/* Check parameter */
	if (n_bank > 3 || mps > 1024) {
		return NULL;
	}
	atomic_enter_critical(&flags);
	/* Re-allocate is not allowed, use usb_h_pipe_set_control_param() */
	if (_usb_h_find_pipe(drv, dev, ep) >= 0) {
		atomic_leave_critical(&flags);
		return NULL;
	} else {
		/* Find free pipe */
		pipe = _usb_h_find_free_pipe(drv, 0, n_bank);
		if (pipe < 0) {
			atomic_leave_critical(&flags);
			return NULL;
		}
	}
	/* Physical pipe 0 is for control only */
	if (pipe == 0) {
		/* Pipe 0 is for default endpoint only */
		if (ep == 0) {
			if (mps > 64) {
				atomic_leave_critical(&flags);
				return NULL;
			}
		} else {
			/* Find free in other pipes */
			pipe = _usb_h_find_free_pipe(drv, 1, n_bank);
		}
	}
	/* Allocate pipe */
	p      = &pd->pipe_pool[pipe];
	p->hcd = drv;

	p->x.general.state = USB_H_PIPE_S_CFG;
	atomic_leave_critical(&flags);

	/* Fill pipe information */
	p->dev          = dev;
	p->ep           = ep;
	p->max_pkt_size = actual_mps;
	p->type         = attr & 0x3;
	p->speed        = speed;
	p->bank         = n_bank;
	p->high_bw_out  = false;
	if (speed == USB_SPEED_HS && (type == 0x01 /* ISO */ || type == 0x11 /* INT */)) {
		uint16_t ms = _usb_h_int_interval(speed, interval);
		p->interval = (ms > 0xFF) ? 0xFF : (uint8_t)ms;
	} else {
		/* Set minimal bulk interval to 1ms to avoid taking all bandwidth */
		if (type == 0x02 /* Bulk */ && interval < 1) {
			p->interval = 1;
		} else {
			p->interval = interval;
		}
	}
	p->zlp    = 0;
	p->toggle = 0;

	/* Try to allocate physical pipe */
	if (_usb_h_pipe_configure(drv, pipe, p->dev, p->ep, p->type, p->max_pkt_size, p->bank, p->interval, p->speed)) {
		/* Initialize pipe state */
		p->x.general.status = 0;

		/* Enable general error and stall interrupts */
		hri_usbpipe_clear_PINTEN_reg(
		    drv->hw, pipe, USB_HOST_PINTFLAG_STALL | USB_HOST_PINTFLAG_TRFAIL | USB_HOST_PINTFLAG_PERR);
		hri_usbpipe_set_PINTEN_reg(
		    drv->hw, pipe, USB_HOST_PINTFLAG_STALL | USB_HOST_PINTFLAG_TRFAIL | USB_HOST_PINTFLAG_PERR);
		p->x.general.state = USB_H_PIPE_S_IDLE;
		return p;
	}

	/* Not allocated, restore state */
	pd->pipe_pool[pipe].x.general.state = USB_H_PIPE_S_FREE;
	return NULL;
}

int32_t _usb_h_pipe_free(struct usb_h_pipe *pipe)
{
	hal_atomic_t flags;
	uint8_t      pi;

	ASSERT(pipe && pipe->hcd && pipe->hcd->hw && pipe->hcd->prvt);

	/* Not able to free a busy pipe */
	atomic_enter_critical(&flags);
	if (pipe->x.general.state == USB_H_PIPE_S_FREE) {
		/* Already free, no error */
		atomic_leave_critical(&flags);
		return USB_H_OK;
	} else if (pipe->x.general.state != USB_H_PIPE_S_IDLE) {
		atomic_leave_critical(&flags);
		return USB_H_BUSY;
	}
	pipe->x.general.state = USB_H_PIPE_S_CFG;
	atomic_leave_critical(&flags);

	pi = _usb_h_pipe_i(pipe);

	/* Free physical pipe */
	hri_usbhost_write_PCFG_reg(pipe->hcd->hw, pi, USB_HOST_PCFG_PTYPE(0));

	/* Update state */
	pipe->x.general.state = USB_H_PIPE_S_FREE;
	return USB_H_OK;
}

int32_t _usb_h_pipe_set_control_param(struct usb_h_pipe *pipe, uint8_t dev, uint8_t ep, uint16_t max_pkt_size,
                                      uint8_t speed)
{
	hal_atomic_t flags;
	uint8_t      pi;
	uint16_t     size;
	uint32_t     cfg;
	uint8_t      epn = ep & 0xF;

	ASSERT(pipe && pipe->hcd && pipe->hcd->hw && pipe->hcd->prvt);

	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)pipe->hcd->prvt;

	/* Check pipe states */
	atomic_enter_critical(&flags);
	if (pipe->x.general.state == USB_H_PIPE_S_FREE) {
		atomic_leave_critical(&flags);
		return USB_H_ERR_NOT_INIT;
	}
	if (pipe->x.general.state != USB_H_PIPE_S_IDLE) {
		atomic_leave_critical(&flags);
		return USB_H_BUSY;
	}
	pipe->x.general.state = USB_H_PIPE_S_CFG;
	atomic_leave_critical(&flags);

	pi   = _usb_h_pipe_i(pipe);
	cfg  = hri_usbhost_read_PCFG_reg(pipe->hcd->hw, pi);
	size = sizeof(usb_ctrl_buffer);
	/* Check endpoint number and packet size */
	if (size < max_pkt_size) {
		pipe->x.general.state = USB_H_PIPE_S_IDLE;
		return USB_H_ERR_ARG;
	}
	/* Operation not supported by none-control endpoints */
	if ((cfg & USB_HOST_PCFG_PTYPE_Msk) != USB_HOST_PCFG_PTYPE(1)) {
		pipe->x.general.state = USB_H_PIPE_S_IDLE;
		return USB_H_ERR_UNSP_OP;
	}
	/* Modify HSTPIPCFG */
	hri_usbhostdescriptor_write_PCKSIZE_SIZE_bf(pd->desc_table, 0, _usb_h_get_psize(max_pkt_size));
	hri_usbhostdescriptor_write_CTRL_PIPE_PEPNUM_bf(pd->desc_table, 0, epn);
	/* Modify dev, ep, max packet size */
	pipe->dev          = dev;
	pipe->speed        = speed;
	pipe->ep           = ep;
	pipe->max_pkt_size = max_pkt_size;
	/* Modify device address */
	_usb_h_pipe_set_addr(pipe->hcd, pi, dev);

	/* Re enable interrupts */
	hri_usbhost_set_PINTEN_reg(
	    pipe->hcd->hw, pi, USB_HOST_PINTFLAG_STALL | USB_HOST_PINTFLAG_TRFAIL | USB_HOST_PINTFLAG_PERR);

	pipe->x.general.state = USB_H_PIPE_S_IDLE;
	return USB_H_OK;
}

int32_t _usb_h_pipe_register_callback(struct usb_h_pipe *pipe, usb_h_pipe_cb_xfer_t cb)
{
	ASSERT(pipe);
	pipe->done = cb;
	return 0;
}

static void _usb_h_reset_tgl(struct usb_h_pipe *pipe, uint8_t tgl)
{
	struct usb_h_desc *drv = pipe->hcd;
	uint8_t            pi  = _usb_h_pipe_i(pipe);
	if (tgl) {
		hri_usbhost_set_PSTATUS_DTGL_bit(drv->hw, pi);
	} else {
		hri_usbhost_clear_PSTATUS_DTGL_bit(drv->hw, pi);
	}
}

static void _usb_h_out_zlp_ex(struct usb_h_pipe *pipe)
{
	struct usb_h_desc * drv = pipe->hcd;
	struct _usb_h_prvt *pd  = (struct _usb_h_prvt *)pipe->hcd->prvt;
	uint8_t             pi  = _usb_h_pipe_i(pipe);

	hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_TRCPT(3));
	hri_usbhost_set_PINTEN_reg(drv->hw, pi, USB_HOST_PINTENSET_TRCPT(3));
	hri_usbhost_set_PSTATUS_DTGL_bit(drv->hw, pi);

	pd->desc_table[pi].HostDescBank[0].ADDR.reg                      = (uint32_t)usb_ctrl_buffer;
	pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT        = 0;
	pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

	hri_usbhost_write_PCFG_PTOKEN_bf(drv->hw, pi, 2);

	hri_usbpipe_set_PSTATUS_BK0RDY_bit(drv->hw, pi);
	hri_usbhost_clear_PSTATUS_PFREEZE_bit(drv->hw, pi);
}

static void _usb_h_out_ex(struct usb_h_pipe *pipe, uint8_t *buffer, uint32_t size)
{
	struct usb_h_desc * drv = pipe->hcd;
	struct _usb_h_prvt *pd  = (struct _usb_h_prvt *)pipe->hcd->prvt;
	uint8_t             pi  = _usb_h_pipe_i(pipe);

	hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_TRCPT(3));
	hri_usbhost_set_PINTEN_reg(drv->hw, pi, USB_HOST_PINTENSET_TRCPT(3));

	pd->desc_table[pi].HostDescBank[0].ADDR.reg                      = (uint32_t)buffer;
	pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT        = size;
	pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

	hri_usbhost_write_PCFG_PTOKEN_bf(drv->hw, pi, 2);

	hri_usbpipe_set_PSTATUS_BK0RDY_bit(drv->hw, pi);
	hri_usbhost_clear_PSTATUS_PFREEZE_bit(drv->hw, pi);
}

static void _usb_h_in_req(struct usb_h_pipe *pipe)
{
	struct usb_h_desc * drv = pipe->hcd;
	struct _usb_h_prvt *pd  = (struct _usb_h_prvt *)pipe->hcd->prvt;
	uint8_t             pi  = _usb_h_pipe_i(pipe);

	hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_TRCPT(3));
	hri_usbhost_set_PINTEN_reg(drv->hw, pi, USB_HOST_PINTENSET_TRCPT(3));

	if (pipe->type == 0) {
		pd->desc_table[pi].HostDescBank[0].ADDR.reg                      = (uint32_t)usb_ctrl_buffer;
		pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT        = 0;
		pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = pipe->max_pkt_size;
	} else {
		uint32_t n_next                                           = pipe->x.bii.size - pipe->x.bii.count;
		pd->desc_table[pi].HostDescBank[0].ADDR.reg               = (uint32_t)&pipe->x.bii.data[pipe->x.bii.count];
		pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
		if (n_next > 16384) {
			n_next = 16384;
		}
		pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = n_next;
	}

	hri_usbhost_write_PCFG_PTOKEN_bf(drv->hw, pi, 1);
	hri_usbpipe_clear_PSTATUS_BK0RDY_bit(drv->hw, pi);
	hri_usbhost_clear_PSTATUS_PFREEZE_bit(drv->hw, pi);
}

static void _usb_h_setup_ex(struct usb_h_pipe *pipe)
{
	struct usb_h_desc * drv  = pipe->hcd;
	struct _usb_h_prvt *pd   = (struct _usb_h_prvt *)pipe->hcd->prvt;
	uint8_t             pi   = _usb_h_pipe_i(pipe);
	uint8_t *           src8 = pipe->x.ctrl.setup;
	volatile uint8_t *  dst8 = usb_ctrl_buffer;
	uint8_t             i;

	hri_usbhost_clear_PINTFLAG_reg(drv->hw, pi, USB_HOST_PINTFLAG_TXSTP);
	for (i = 0; i < 8; i++) {
		*dst8++ = *src8++;
	}
	pd->desc_table[pi].HostDescBank[0].ADDR.reg                      = (uint32_t)usb_ctrl_buffer;
	pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT        = 8;
	pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

	hri_usbhost_write_PCFG_PTOKEN_bf(drv->hw, pi, 0);
	hri_usbhost_clear_PSTATUS_DTGL_bit(drv->hw, pi);
	hri_usbpipe_set_PSTATUS_BK0RDY_bit(drv->hw, pi);
	hri_usbhost_clear_PSTATUS_PFREEZE_bit(drv->hw, pi);
	hri_usbhost_set_PINTEN_reg(drv->hw, pi, USB_HOST_PINTENSET_TXSTP);
}

static inline void _usb_h_load_x_param(struct usb_h_pipe *pipe, uint8_t **buf, uint32_t *x_size, uint32_t *x_count)
{
	if (pipe->type != 0 /* None Control */) {
		*buf     = pipe->x.bii.data;
		*x_size  = pipe->x.bii.size;
		*x_count = pipe->x.bii.count;
	} else {
		*buf     = pipe->x.ctrl.data;
		*x_size  = pipe->x.ctrl.size;
		*x_count = pipe->x.ctrl.count;
	}
}

static inline void _usb_h_save_x_param(struct usb_h_pipe *pipe, uint32_t x_count)
{
	if (pipe->type != 0 /* None Control */) {
		pipe->x.bii.count = x_count;
	} else {
		pipe->x.ctrl.count = x_count;
	}
}

static void _usb_h_in(struct usb_h_pipe *pipe)
{
	uint8_t             pi = _usb_h_pipe_i(pipe);
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)pipe->hcd->prvt;
	uint8_t *           src, *dst;
	uint32_t            size, count, i;
	uint32_t            n_rx = 0;
	uint32_t            n_remain;
	bool                shortpkt = false, full = false;

	if (pipe->x.general.state == USB_H_PIPE_S_STATI) {
		/* Control status : ZLP IN done */
		_usb_h_end_transfer(pipe, USB_H_OK);
		return;
	} else if (pipe->x.general.state != USB_H_PIPE_S_DATI) {
		return;
	}
	/* Read byte count */
	n_rx = pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT;
	if (n_rx < pipe->max_pkt_size) {
		shortpkt = true;
	}
	if (n_rx) {
		_usb_h_load_x_param(pipe, &dst, &size, &count);
		n_remain = size - count;
		src      = (uint8_t *)pd->desc_table[pi].HostDescBank[0].ADDR.reg;
		dst      = &dst[count];
		if (n_rx >= n_remain) {
			n_rx = n_remain;
			full = true;
		}
		count += n_rx;
		for (i = 0; i < n_rx; i++) {
			*dst++ = *src++;
		}
		_usb_h_save_x_param(pipe, count);
	}

	/* Reset timeout for control pipes */
	if (pipe->type == 0) {
		pipe->x.ctrl.pkt_timeout = USB_CTRL_DPKT_TIMEOUT;
	}
	/* Finish on error or short packet */
	if (full || shortpkt) {
		if (pipe->type == 0) { /* Control transfer: DatI -> StatO */
			pipe->x.ctrl.state       = USB_H_PIPE_S_STATO;
			pipe->x.ctrl.pkt_timeout = USB_CTRL_STAT_TIMEOUT;
			hri_usbhost_set_PSTATUS_DTGL_bit(pipe->hcd->hw, pi);
			_usb_h_out_zlp_ex(pipe);
		} else {
			_usb_h_end_transfer(pipe, USB_H_OK);
		}
	} else {
		/* Just wait another packet */
		_usb_h_in_req(pipe);
	}
}

static void _usb_h_out(struct usb_h_pipe *pipe)
{
	uint8_t             pi = _usb_h_pipe_i(pipe);
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)pipe->hcd->prvt;
	uint8_t *           src;
	uint32_t            size, count;
	uint32_t            n_tx = 0;
	uint32_t            n_remain;

	if (pipe->x.general.state == USB_H_PIPE_S_STATO) {
		/* Control status : ZLP OUT done */
		_usb_h_end_transfer(pipe, USB_H_OK);
		return;
	} else if (pipe->x.general.state != USB_H_PIPE_S_DATO) {
		return;
	}
	/* ACK OUT */

	/* Reset packet timeout for control pipes */
	if (pipe->type == 0 /* Control */) {
		pipe->x.ctrl.pkt_timeout = USB_CTRL_DPKT_TIMEOUT;
	}

	n_tx = pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT;
	/* ZLP cleared if it's short packet */
	if (n_tx < pipe->max_pkt_size) {
		pipe->zlp = 0;
	}

	_usb_h_load_x_param(pipe, &src, &size, &count);
	if (n_tx) {
		count += n_tx;
		_usb_h_save_x_param(pipe, count);
	}
	n_remain = size - count;

	/* Now set n_tx to next transfer size */
	if (n_remain > 16320) {
		n_tx = 16320;
	} else {
		n_tx = n_remain;
	}

	/* For Control, all data is done, to STATUS stage */
	if (pipe->type == 0 /* Control */ && pipe->x.ctrl.count >= pipe->x.ctrl.size && !pipe->zlp) {
		pipe->x.ctrl.state = USB_H_PIPE_S_STATI;

		/* Start IN ZLP request */
		pipe->x.ctrl.pkt_timeout = USB_CTRL_STAT_TIMEOUT;
		_usb_h_reset_tgl(pipe, 1);
		_usb_h_in_req(pipe);
		return;
	}

	/* All transfer done, including ZLP */
	if (count >= size && !pipe->zlp) {
		/* At least one bank there, wait to freeze pipe */
		if (pipe->type != 0 /* None CTRL */) {
			/* Busy interrupt when all banks are empty */
			_usb_h_end_transfer(pipe, USB_H_OK);
		} else {
			/* No busy interrupt for control EPs */
		}
	} else {
		_usb_h_out_ex(pipe, &src[count], n_remain);
	}
}

int32_t _usb_h_control_xfer(struct usb_h_pipe *pipe, uint8_t *setup, uint8_t *data, uint16_t length, int16_t timeout)
{
	hal_atomic_t flags;
	int8_t       pi = _usb_h_pipe_i(pipe);

	ASSERT(pipe && pipe->hcd && pipe->hcd->hw && pipe->hcd->prvt);
	ASSERT(pi >= 0 && pi < CONF_USB_H_NUM_PIPE_SP);

	/* Check parameters */
	if (pipe->type != 0) { /* Not control */
		return USB_H_ERR_UNSP_OP;
	}
	/* Check state */
	atomic_enter_critical(&flags);
	if (pipe->x.general.state == USB_H_PIPE_S_FREE) {
		atomic_leave_critical(&flags);
		return USB_H_ERR_NOT_INIT;
	} else if (pipe->x.general.state != USB_H_PIPE_S_IDLE) {
		atomic_leave_critical(&flags);
		return USB_H_BUSY;
	}
	pipe->x.general.state = USB_H_PIPE_S_SETUP;
	_usb_h_add_sof_user(pipe->hcd); /* SOF user: control timeout */
	_usb_h_add_req_user(pipe->hcd);
	atomic_leave_critical(&flags);

	pipe->x.ctrl.setup       = setup;
	pipe->x.ctrl.data        = data;
	pipe->x.ctrl.size        = length;
	pipe->x.ctrl.count       = 0;
	pipe->x.ctrl.req_timeout = timeout;
	pipe->x.ctrl.pkt_timeout = length ? USB_CTRL_DPKT_TIMEOUT : (-1);
	pipe->x.ctrl.status      = 0;

	/* Start with setup packet */
	_usb_h_setup_ex(pipe);
	return USB_H_OK;
}

int32_t _usb_h_bulk_int_iso_xfer(struct usb_h_pipe *pipe, uint8_t *data, uint32_t length, bool auto_zlp)
{
	hal_atomic_t flags;
	int8_t       pi       = _usb_h_pipe_i(pipe);
	bool         dir      = pipe->ep & 0x80;
	bool         iso_pipe = pipe->type == 1;

	ASSERT(pipe && pipe->hcd && pipe->hcd->hw && pipe->hcd->prvt);
	ASSERT(pi >= 0 && pi < CONF_USB_H_NUM_PIPE_SP);

	/* Check parameters */
	if (pipe->type == 0) { /* Control */
		return USB_H_ERR_UNSP_OP;
	}
	/* Check state */
	atomic_enter_critical(&flags);
	if (pipe->x.general.state == USB_H_PIPE_S_FREE) {
		atomic_leave_critical(&flags);
		return USB_H_ERR_NOT_INIT;
	} else if (pipe->x.general.state != USB_H_PIPE_S_IDLE) {
		atomic_leave_critical(&flags);
		return USB_H_BUSY;
	}
	pipe->x.general.state = dir ? USB_H_PIPE_S_DATI : USB_H_PIPE_S_DATO;
	atomic_leave_critical(&flags);

	pipe->x.bii.data   = data;
	pipe->x.bii.size   = length;
	pipe->x.bii.count  = 0;
	pipe->x.bii.status = 0;
	pipe->zlp          = auto_zlp;

	if (dir) { /* IN */
		pipe->periodic_start = false;
		_usb_h_in_req(pipe);
	} else {
		pipe->periodic_start = iso_pipe;
		_usb_h_out_ex(pipe, data, length > 16320 ? 16320 : length);
	}
	return USB_H_OK;
}

int32_t _usb_h_high_bw_out(struct usb_h_pipe *pipe, uint8_t *data, uint32_t length, uint16_t trans_pkt_size[3])
{
	return USB_H_ERR_UNSP_OP;
}

bool _usb_h_pipe_is_busy(struct usb_h_pipe *pipe)
{
	uint8_t s;
	ASSERT(pipe);
	s = pipe->x.general.state;
	return !(s == USB_H_PIPE_S_IDLE || s == USB_H_PIPE_S_FREE);
}

static void _usb_h_end_transfer(struct usb_h_pipe *pipe, int32_t code)
{
	uint8_t             s  = pipe->x.general.state;
	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)pipe->hcd->prvt;
	if (s < USB_H_PIPE_S_SETUP || s > USB_H_PIPE_S_STATO) {
		return; /* Not busy */
	}
	if (pipe->type == 0 /* Control */) {
		_usb_h_rm_req_user(pipe->hcd);
		_usb_h_rm_sof_user(pipe->hcd); /* SOF user: control timeout */
	}
	pipe->x.general.state  = USB_H_PIPE_S_IDLE;
	pipe->x.general.status = code;
	if (pipe->done) {
		pipe->done(pipe);
	}
	/* Suspend delayed due to control request: start it */
	if (pd->n_ctrl_req_user == 0 && pd->suspend_start < 0) {
		_usb_h_suspend(pipe->hcd);
	}
}

static void _usb_h_abort_transfer(struct usb_h_pipe *pipe, int32_t code)
{
	struct usb_h_desc * drv = (struct usb_h_desc *)pipe->hcd;
	struct _usb_h_prvt *pd  = (struct _usb_h_prvt *)pipe->hcd->prvt;
	uint8_t             pi  = _usb_h_pipe_i(pipe);

	/* Stop transfer */
	hri_usbhost_set_PSTATUS_PFREEZE_bit(drv->hw, pi);

	/* Update byte count */
	if ((pipe->ep & 0x80) == 0) {
		pipe->x.bii.count += pd->desc_table[pi].HostDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE;
	}

	/* Disable interrupts */
	hri_usbhost_clear_PINTEN_reg(drv->hw, pi, USB_HOST_PINTENCLR_TRCPT(3));
	_usb_h_end_transfer(pipe, code);
}

void _usb_h_pipe_abort(struct usb_h_pipe *pipe)
{
	ASSERT(pipe && pipe->hcd && pipe->hcd->hw && pipe->hcd->prvt);
	_usb_h_abort_transfer(pipe, USB_H_ABORT);
}

/**
 * \brief USB interrupt handler
 */
void USB_Handler(void)
{

	_usb_h_handler();
}
