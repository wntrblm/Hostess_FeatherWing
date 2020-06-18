/**
 * \file
 *
 * \brief USB Host Stack Device Driver Definition.
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
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
 */

#ifndef _USB_USBHD_H_
#define _USB_USBHD_H_

#include "usbhc.h"

/**
 * \internal
 * \brief Get Core that the device attached to
 * \param[in] dev Pointer to instance of device driver
 * \return Pointer to instance of core driver
 */
#define usbhd_get_core(dev) usbhc_get_dev_core(dev)

/**
 * \internal
 * \brief Take the control resources on root device for requests
 *
 * The resources are released after request done.
 * If no request issued, use \ref usbhc_release_control to release.
 *
 * \param[in]  d Pointer to device instance
 * \param[out] r Pointer to fill a pointer to root device instance
 */
#define usbhd_take_control(d, r) usbhc_take_control(d, r, USBHC_USED_BY_DEV)

#endif /* _USB_USBHD_H_ */
