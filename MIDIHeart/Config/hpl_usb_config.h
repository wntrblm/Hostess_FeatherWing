/* Auto-generated config file hpl_usb_config.h */
#ifndef HPL_USB_CONFIG_H
#define HPL_USB_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

#define CONF_USB_N_0 0
#define CONF_USB_N_1 1
#define CONF_USB_N_2 2
#define CONF_USB_N_3 3
#define CONF_USB_N_4 4
#define CONF_USB_N_5 5
#define CONF_USB_N_6 6
#define CONF_USB_N_7 7
#define CONF_USB_N_8 8
#define CONF_USB_N_9 9
#define CONF_USB_N_10 10
#define CONF_USB_N_11 11
#define CONF_USB_N_12 12
#define CONF_USB_N_13 13
#define CONF_USB_N_14 14
#define CONF_USB_N_15 15

#define CONF_USB_D_EP_N_MAX (USB_EPT_NUM - 1)
#define CONF_USB_D_N_EP_MAX (CONF_USB_D_EP_N_MAX * 2 - 1)

// <h> USB Host HAL Configurations

// <q> USB Host Instance Owner Support
// <i> Add owner field in driver instances, for upper layer to use
// <i> Add owner pointer in driver descriptor and pipe descriptor
// <i> Turn off it to optimize memory while it's not used
// <id> usbh_inst_owner_sp
#ifndef CONF_USB_H_INST_OWNER_SP
#define CONF_USB_H_INST_OWNER_SP 1
#endif

// </h>

// <h> USB Host Implement Configurations

// <y> Number of root hub ports
// <CONF_USB_N_1"> 1
// <i> There is only one port in USB host root hub for this chip.
// <i> According to the status code of root hub change callback, the max supported port number is 31.
// <id> usbh_arch_n_rh_port
#ifndef CONF_USB_H_NUM_RH_PORT
#define CONF_USB_H_NUM_RH_PORT CONF_USB_N_1
#endif

// <y> Max number of pipes supported
// <i> Limits the number of pipes can be used in upper layer.
// <CONF_USB_N_1"> 1 (Pipe0 only)
// <CONF_USB_N_2"> 2 (Pipe0 + 1 Pipe)
// <CONF_USB_N_3"> 3 (Pipe0 + 2 pipes)
// <CONF_USB_N_4"> 4 (Pipe0 + 3 pipes)
// <CONF_USB_N_5"> 5 (Pipe0 + 4 pipes)
// <CONF_USB_N_6"> 6 (Pipe0 + 5 pipes)
// <CONF_USB_N_7"> 7 (Pipe0 + 6 pipes)
// <CONF_USB_N_8"> 8 (Pipe0 + 7 pipes)
// <id> usbh_num_pipe_sp
#ifndef CONF_USB_H_NUM_PIPE_SP
#define CONF_USB_H_NUM_PIPE_SP CONF_USB_N_4
#endif

// <e> USB Host VBus supply control
// <id> usbh_arch_vbus_ctrl
#ifndef CONF_USB_H_VBUS_CTRL
#define CONF_USB_H_VBUS_CTRL 1
#endif

// <s> External function name for VBus on/off control
// <i> The function prototype: void function_name(void *drv, uint8_t port, bool onoff).
// <id> usbh_arch_vbus_ctrl_func
#ifndef CONF_USB_H_VBUS_CTRL_FUNC_NAME
#define CONF_USB_H_VBUS_CTRL_FUNC_NAME "my_vbus_ctrl_func"
#endif

extern void my_vbus_ctrl_func(void *drv, uint8_t port, bool onoff);
#ifndef CONF_USB_H_VBUS_CTRL_FUNC
#define CONF_USB_H_VBUS_CTRL_FUNC my_vbus_ctrl_func
#endif
// </e>

// </h>

// <<< end of configuration section >>>

#endif // HPL_USB_CONFIG_H
