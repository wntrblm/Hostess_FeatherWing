#pragma once

#include "hal_usb_host.h"
#include "hpl_usb_host.h"
#include "wtr_usb_defs.h"

static int32_t wtr_usb_send_get_dev_desc_request(struct usb_h_pipe *pipe_0, uint8_t *req_buf, uint8_t *rep_buf,
                                                 uint8_t len) {
    struct usb_req *req = (struct usb_req *)(req_buf);
    req->bmRequestType = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_IN;
    req->bRequest = USB_REQ_GET_DESC;
    req->wValue = (USB_DT_DEVICE << 8);
    req->wIndex = 0;
    req->wLength = 18;
    return usb_h_control_xfer(pipe_0, (uint8_t *)req, (uint8_t *)rep_buf, len, 500);
}

static int32_t wtr_usb_send_get_conf_desc_request(struct usb_h_pipe *pipe_0, uint8_t cfg_idx, uint8_t *req_buf,
                                                  uint8_t *rep_buf, uint16_t len) {
    struct usb_req *req = (struct usb_req *)(req_buf);
    req->bmRequestType = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_IN;
    req->bRequest = USB_REQ_GET_DESC;
    req->wValue = (USB_DT_CONFIG << 8) | cfg_idx;
    req->wIndex = 0;
    req->wLength = len;
    return usb_h_control_xfer(pipe_0, (uint8_t *)req, (uint8_t *)rep_buf, len, 500);
}

static int32_t wtr_usb_send_set_address_request(struct usb_h_pipe *pipe_0, uint8_t address, uint8_t *req_buf) {
    struct usb_req *req = (struct usb_req *)(req_buf);
    req->bmRequestType = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_OUT;
    req->bRequest = USB_REQ_SET_ADDRESS;
    req->wValue = address;
    req->wIndex = 0;
    req->wLength = 0;
    return usb_h_control_xfer(pipe_0, (uint8_t *)req, NULL, 0, 500);
}

static int32_t wtr_usb_send_set_config_request(struct usb_h_pipe *pipe_0, uint8_t cfg, uint8_t *req_buf) {
    struct usb_req *req = (struct usb_req *)(req_buf);
    req->bmRequestType = USB_REQT_RECIP_DEVICE | USB_REQT_TYPE_STANDARD | USB_REQT_DIR_OUT;
    req->bRequest = USB_REQ_SET_CONFIG;
    req->wValue = cfg;
    req->wIndex = 0;
    req->wLength = 0;
    return usb_h_control_xfer(pipe_0, (uint8_t *)req, NULL, 0, 50);
}