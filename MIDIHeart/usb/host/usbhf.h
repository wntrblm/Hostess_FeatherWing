/**
 * \file
 *
 * \brief USB Host Stack Function Driver Definition.
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

#ifndef _USB_USBHF_H_
#define _USB_USBHF_H_

#include "usbhd.h"

/**
 * \internal
 * \brief Get Device that using the function
 * \param[in] func Pointer to instance of function driver
 * \return Pointer to instance of device driver
 */
#define usbhf_get_dev(func) usbhc_get_func_dev(USBHF_PTR(func))

/**
 * \internal
 * \brief Get Core that the function attached to
 * \param[in] func Pointer to instance of function driver
 * \return Pointer to instance of core driver
 */
#define usbhf_get_core(func) usbhc_get_dev_core(usbhf_get_dev(func))

/**
 * \internal
 * \brief Take the control resources on root device for requests
 *
 * The resources are released after request done.
 *
 * \param[in]  f Pointer to function instance
 * \param[out] r Pointer to fill a pointer to root device instance
 */
#define usbhf_take_control(f, r) usbhd_take_control(usbhf_get_dev(f), r)

#endif /* USBHC_H_ */
