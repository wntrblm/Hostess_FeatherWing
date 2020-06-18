/* Auto-generated config file usbh_config.h */
#ifndef USBH_CONFIG_H
#define USBH_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

// ---- USB Host Stack Core Options ----

// <o> Control process buffer size <64-1024>
// <i> Control process buffer is shared by all default endpoint 0 transfers
// <i> The size should be max supported descriptor size plus setup packet size (8) plus VID and PID (4)
// <id> usbh_ctrl_buf_size
#ifndef CONF_USBH_CTRL_BUF_SIZE
#define CONF_USBH_CTRL_BUF_SIZE 1024
#endif

// <o> Max power (in mA) <0-10000:100>
// <i> Max power can supply by root hub, in mA.
// <i> 0 - The power is not checked (unlimited).
// <id> usbh_power_max
#ifndef CONF_USBH_POWER_MAX
#define CONF_USBH_POWER_MAX 500
#endif

// VID and PID are logged in shared control buffer
#ifndef CONF_USBH_VENDOR_DEV_SP
#define CONF_USBH_VENDOR_DEV_SP 1
#endif

// Multiple devices support in the USB bus tree
#ifndef CONF_USBH_MULTI_DEV_SP
#define CONF_USBH_MULTI_DEV_SP 0
#endif

// Hub support in the USB bus tree
#ifndef CONF_USBH_HUB_SP
#define CONF_USBH_HUB_SP 0
#endif

// <<< end of configuration section >>>

#endif // USBH_CONFIG_H
