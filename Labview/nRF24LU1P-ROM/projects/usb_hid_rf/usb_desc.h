/* Copyright (c) 2006 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 */

/** @file
 * Definition of the USB configuration descriptor type
 */

/** @{
 * @ingroup nordic_usb
 */
#ifndef  USB_DESC_H__
#define  USB_DESC_H__

#include <stdint.h>
#include <hal_usb_desc.h>
#include <hal_usb_hid_desc.h>

#define USB_DESC_TEMPLATE

//-----------------------------------------------------------------------------
// Vendor ID and Product ID definitions
//-----------------------------------------------------------------------------
#define VID   0x1915
#define PID   0x007B // TODO: Must be changed?

//-----------------------------------------------------------------------------
// Endpoint packet size definitions
//-----------------------------------------------------------------------------

#define MAX_PACKET_SIZE_EP0 0x20
// Identical packets for ep 1/2. If different packet sizes are needed, 
// an additional report descriptor must be made
#define EP1_2_PACKET_SIZE 0x20    							//32 bytes

//-----------------------------------------------------------------------------
// Endpoint polling interval(in ms).
// Minimum interval(Maximum bandwidth) = 1
//-----------------------------------------------------------------------------

#define EP1_POLLING_INTERVAL 1
#define EP2_POLLING_INTERVAL 1

//-----------------------------------------------------------------------------
// HID Configuration Descriptor type Definition.
// From "USB Device Class Definition for Human Interface Devices (HID)".
// Section 7.1:
// "When a Get_Descriptor(Configuration) request is issued,
// it returns the Configuration descriptor, all Interface descriptors,
// all Endpoint descriptors, and the HID descriptor for each interface."
// If more endpoints/interfaces are added, they must also be added here.
//-----------------------------------------------------------------------------

typedef struct {
    hal_usb_conf_desc_t conf;

    hal_usb_if_desc_t if0;      // user data
    hal_usb_hid_desc_t hid1;
    hal_usb_ep_desc_t ep1in;    
    hal_usb_ep_desc_t ep2out;
} usb_conf_desc_templ_t;

//-----------------------------------------------------------------------------
// String descriptor type Definition
//-----------------------------------------------------------------------------

#define USB_STRING_DESC_COUNT 2

typedef struct {
     volatile uint8_t* idx[USB_STRING_DESC_COUNT];
} usb_string_desc_templ_t;

//-----------------------------------------------------------------------------
// Structure containing device, string and configuration descriptors.
//-----------------------------------------------------------------------------

typedef struct {
     const hal_usb_dev_desc_t* dev;
     const usb_conf_desc_templ_t* conf;
     const usb_string_desc_templ_t* string;
     uint8_t string_zero[4];
} usb_descs_templ_t;

//-----------------------------------------------------------------------------
// External references
//-----------------------------------------------------------------------------

extern code usb_string_desc_templ_t g_usb_string_desc;
extern code const usb_conf_desc_templ_t g_usb_conf_desc;
extern code const hal_usb_dev_desc_t g_usb_dev_desc;
extern code hal_usb_hid_t g_usb_hid_hids[];

#endif  /* _USB_DESC_H_ */

/** @} */
