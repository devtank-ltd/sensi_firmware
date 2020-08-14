#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/crs.h>

#include "log.h"
#include "cmd.h"
#include "uarts.h"
#include "uart_rings.h"


/*
Endpoint Address
Bits 0..3b Endpoint Number.
Bits 4..6b Reserved. Set to Zero
Bits 7 Direction 0 = In, 1 = Out
*/

#define USB_COM1_END_ADDR  0x81

#define USB_DATA1_END_ADDR 0x02 // write addr is OR 0x80

#define USB_COM_PCK_SZ     16

static bool connected = false;


static const char *usb_strings[] = {
        "Devtank Ltd",
        "IO Board",
        "Prototype",
        "Comm"
        "Devtank UART",
};

static uint8_t usbd_control_buffer[256];

static usbd_device *usbd_dev = NULL;


static struct
{
    uint8_t data_addr;
    uint8_t com_addr;
} end_addrs[] =
{
    {
        USB_DATA1_END_ADDR,
        USB_COM1_END_ADDR
    },
};


static const struct usb_device_descriptor usb_dev_desc =
{
    .bLength            = USB_DT_DEVICE_SIZE,
    .bDescriptorType    = USB_DT_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = USB_CLASS_CDC,
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,
    .bMaxPacketSize0    = USB_DATA_PCK_SZ,
    .idVendor           = 0x0483,
    .idProduct          = 0x5740,
    .bcdDevice          = 0x0200,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};


#define UART_COMM_ENDP_INIT(_com_ep_)                    \
{                                                        \
    {                                                    \
        .bLength          = USB_DT_ENDPOINT_SIZE,        \
        .bDescriptorType  = USB_DT_ENDPOINT,             \
        .bEndpointAddress = _com_ep_,                    \
        .bmAttributes     = USB_ENDPOINT_ATTR_INTERRUPT, \
        .wMaxPacketSize   = USB_COM_PCK_SZ,              \
        .bInterval        = 255,                         \
    },                                                   \
}

#define UART_DATA_ENDP_INIT(_data_ep_)                  \
{                                                       \
    {                                                   \
        .bLength          = USB_DT_ENDPOINT_SIZE,       \
        .bDescriptorType  = USB_DT_ENDPOINT,            \
        .bEndpointAddress = _data_ep_,                  \
        .bmAttributes     = USB_ENDPOINT_ATTR_BULK,     \
        .wMaxPacketSize   = USB_DATA_PCK_SZ,            \
        .bInterval        = 1,                          \
    },                                                  \
    {                                                   \
        .bLength          = USB_DT_ENDPOINT_SIZE,       \
        .bDescriptorType  = USB_DT_ENDPOINT,            \
        .bEndpointAddress = (_data_ep_ | 0x80),         \
        .bmAttributes     = USB_ENDPOINT_ATTR_BULK,     \
        .wMaxPacketSize   = USB_DATA_PCK_SZ,            \
        .bInterval        = 1,                          \
    }                                                   \
}


typedef struct
{
        struct usb_cdc_header_descriptor          header;
        struct usb_cdc_call_management_descriptor call_management_desc;
        struct usb_cdc_acm_descriptor             acm_desc;
        struct usb_cdc_union_descriptor           cdc_union_desc;
} __attribute__((packed)) usb_func_descs;

#define USB_FUNC_DESC_INIT(_cinf, _dinf_)                                         \
{                                                                                 \
    .header =                                                                     \
    {                                                                             \
        .bFunctionLength    = sizeof(struct usb_cdc_header_descriptor),           \
        .bDescriptorType    = CS_INTERFACE,                                       \
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,                                \
        .bcdCDC             = 0x0110,                                             \
    },                                                                            \
    .call_management_desc =                                                       \
    {                                                                             \
        .bFunctionLength    = sizeof(struct usb_cdc_call_management_descriptor),  \
        .bDescriptorType    = CS_INTERFACE,                                       \
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,                       \
        .bmCapabilities     = 0,                                                  \
        .bDataInterface     = _dinf_,                                             \
    },                                                                            \
    .acm_desc =                                                                   \
    {                                                                             \
        .bFunctionLength    = sizeof(struct usb_cdc_acm_descriptor),              \
        .bDescriptorType    = CS_INTERFACE,                                       \
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,                                   \
        .bmCapabilities     = 0,                                                  \
    },                                                                            \
    .cdc_union_desc =                                                             \
    {                                                                             \
        .bFunctionLength        = sizeof(struct usb_cdc_union_descriptor),        \
        .bDescriptorType        = CS_INTERFACE,                                   \
        .bDescriptorSubtype     = USB_CDC_TYPE_UNION,                             \
        .bControlInterface      = _cinf,                                          \
        .bSubordinateInterface0 = _dinf_,                                         \
 }                                                                                \
}


#define USB_COMM_INTF_DESC_INIT(_i_num_, _p_ep_, _desc_) \
{                                                                   \
    {                                                               \
        .bLength            = USB_DT_INTERFACE_SIZE,                \
        .bDescriptorType    = USB_DT_INTERFACE,                     \
        .bInterfaceNumber   = _i_num_,                              \
        .bAlternateSetting  = 0,                                    \
        .bNumEndpoints      = 1,                                    \
        .bInterfaceClass    = USB_CLASS_CDC,                        \
        .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,                 \
        .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,                  \
        .iInterface         = 0,                                    \
                                                                    \
        .endpoint           = _p_ep_,                               \
                                                                    \
        .extra              = _desc_,                               \
        .extralen           = sizeof(usb_func_descs)                \
    }                                                               \
}


#define USB_DATA_INTERFACE_DESCRIPTOR_INIT(_i_num_, _p_ep_)         \
{                                                                   \
    {                                                               \
        .bLength            = USB_DT_INTERFACE_SIZE,                \
        .bDescriptorType    = USB_DT_INTERFACE,                     \
        .bInterfaceNumber   = _i_num_,                              \
        .bAlternateSetting  = 0,                                    \
        .bNumEndpoints      = 2,                                    \
        .bInterfaceClass    = USB_CLASS_DATA,                       \
        .bInterfaceSubClass = 0,                                    \
        .bInterfaceProtocol = 0,                                    \
        .iInterface         = 0,                                    \
                                                                    \
        .endpoint           = _p_ep_,                               \
    }                                                               \
}



static const usb_func_descs usb_func_descs1 = USB_FUNC_DESC_INIT(0, 1);

static const struct usb_endpoint_descriptor usb_uart_comm_ep1[] = UART_COMM_ENDP_INIT(USB_COM1_END_ADDR);

static const struct usb_endpoint_descriptor usb_uart_data_ep1[] = UART_DATA_ENDP_INIT(USB_DATA1_END_ADDR);

static const struct usb_interface_descriptor uart_comm_iface1[] = USB_COMM_INTF_DESC_INIT(0, usb_uart_comm_ep1, &usb_func_descs1);

static const struct usb_interface_descriptor uart_data_iface1[] = USB_DATA_INTERFACE_DESCRIPTOR_INIT(1, usb_uart_data_ep1);



static const struct usb_interface usb_ifaces[] =
{
    { .altsetting = uart_comm_iface1, .num_altsetting = 1, },
    { .altsetting = uart_data_iface1, .num_altsetting = 1, },
};


static const struct usb_config_descriptor usb_config =
{
    .bLength             = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType     = USB_DT_CONFIGURATION,
    .wTotalLength        = 0,
    .bNumInterfaces      = ARRAY_SIZE(usb_ifaces),
    .bConfigurationValue = 1,
    .iConfiguration      = 0,
    .bmAttributes        = USB_CONFIG_ATTR_DEFAULT,
    .bMaxPower           = 50,
    .interface           = usb_ifaces,
};


static enum usbd_request_return_codes usb_control_request(usbd_device                    *_   __attribute((unused)),
                                                          struct usb_setup_data          *req,
                                                          uint8_t                       **__  __attribute((unused)),
                                                          uint16_t                       *len,
                                                          usbd_control_complete_callback *___ __attribute((unused)))
{
    switch (req->bRequest)
    {
        case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
            return USBD_REQ_HANDLED;
        case USB_CDC_REQ_SET_LINE_CODING:
            if (*len < sizeof(struct usb_cdc_line_coding))
                return USBD_REQ_NOTSUPP;
            return USBD_REQ_HANDLED;
    }

    return USBD_REQ_NOTSUPP;
}


static void usb_data_rx_cb(usbd_device *_ __attribute((unused)), uint8_t end_point)
{
    char buf[USB_DATA_PCK_SZ];
    int len = usbd_ep_read_packet(usbd_dev, end_point, buf, USB_DATA_PCK_SZ);

    if (!len)
        return;

    for(unsigned n = 0; n < ARRAY_SIZE(end_addrs); n++)
    {
        if (end_addrs[n].data_addr == end_point)
        {
            if (n) // UART 1+ pass straight through.
                uart_ring_out(n, buf, len);
            else // UART 0 is command.
                uart_ring_in(0, buf, len);
            break;
        }
    }
}


static void usb_set_config_cb(usbd_device *usb_dev,
                              uint16_t     wValue __attribute((unused)))
{
    log_debug(DEBUG_SYS, "USB connected");
    connected = true;

    for(unsigned n = 0; n < ARRAY_SIZE(end_addrs); n++)
    {
        usbd_ep_setup(usb_dev, end_addrs[n].com_addr,         USB_ENDPOINT_ATTR_INTERRUPT, USB_COM_PCK_SZ,  NULL);
        usbd_ep_setup(usb_dev, end_addrs[n].data_addr,        USB_ENDPOINT_ATTR_BULK,      USB_DATA_PCK_SZ, usb_data_rx_cb);
        usbd_ep_setup(usb_dev, end_addrs[n].data_addr | 0x80, USB_ENDPOINT_ATTR_BULK,      USB_DATA_PCK_SZ, NULL);
    }

    usbd_register_control_callback(
                usb_dev,
                USB_REQ_TYPE_CLASS |
                USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE |
                USB_REQ_TYPE_RECIPIENT,
                usb_control_request);
}


void usb_init()
{
    crs_autotrim_usb_enable();
    rcc_set_usbclk_source(RCC_HSI48);

    usbd_dev = usbd_init(&st_usbfs_v2_usb_driver,
                         &usb_dev_desc,
                         &usb_config,
                         usb_strings, ARRAY_SIZE(usb_strings),
                         usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, usb_set_config_cb);
    nvic_set_priority(NVIC_USB_IRQ, 2);
    nvic_enable_irq(NVIC_USB_IRQ);
}


void usb_isr()
{
    usbd_poll(usbd_dev);
}


unsigned usb_uart_send(unsigned uart, void * data, unsigned len, bool flush)
{
    if (uart >= ARRAY_SIZE(end_addrs))
        return 0;

    uint8_t w_addr = end_addrs[uart].data_addr | 0x80;

    if (len > USB_DATA_PCK_SZ)
        len = USB_DATA_PCK_SZ;

    if (!connected)
        return 0;

    usbd_poll(usbd_dev);

    unsigned r = usbd_ep_write_packet(usbd_dev, w_addr, data, len);
    /* If it's a full packet and there isn't one following, the receiving end may just sit waiting for more before reporting it.*/
    if (flush && r == USB_DATA_PCK_SZ)
        while (usbd_ep_write_packet(usbd_dev, w_addr, "\0", 1) <= 0)
            usbd_poll(usbd_dev);

    return r;
}
