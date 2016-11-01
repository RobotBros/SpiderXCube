/* Copyright (c) 2008 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 133 $
 */

#include "usb_desc.h"
#include "nordic_common.h"               

//-----------------------------------------------------------------------------
// Device descriptor
//-----------------------------------------------------------------------------

code const hal_usb_dev_desc_t g_usb_dev_desc =
{
  sizeof(hal_usb_dev_desc_t),         // bLength
  USB_DESC_DEVICE,                    // bDescriptorType
  SWAP(0x0200),                       // bcdUSB
  0x00,                               // bDeviceClass
  0x00,                               // bDeviceSubClass
  0x00,                               // bDeviceProtocol
  MAX_PACKET_SIZE_EP0,                // bMaxPacketSize0
  SWAP(VID),                          // idVendor (VID)
  SWAP(PID),                          // idProduct (PID)
  SWAP(0x0100),                       // bcdDevice
  0x01,                               // iManufacturer
  0x02,                               // iProduct
  0x00,                               // iSerialNumber
  0x01                                // bNumConfigurations
};
                                        
//-----------------------------------------------------------------------------
// Report descriptor
//-----------------------------------------------------------------------------

code uint8_t g_usb_hid_report_data[] = 
{
  0x06, 0x00, 0xff,           // USAGE_PAGE (Vendor defined page 1)
  0x09, 0x00,                 // USAGE (Vendor Usage 1)
  0xa1, 0x01,                 // COLLECTION (Application)
  0x15, 0x00,                   //   LOGICAL_MINIMUM (0)
  0x25, 0xff,                   //   LOGICAL_MAXIMUM (255)
  0x75, 0x08,                   //   REPORT_SIZE (8 bit)
  0x95, EP1_2_PACKET_SIZE,      //   REPORT_COUNT (EP1_2_PACKET_SIZE)
  0x09, 0x00,                   //   USAGE (Vendor Usage 1)
  0x81, 0x02,                   //   INPUT (Data,Var,Abs)
  0x09, 0x00,                   //   USAGE (Vendor Usage 1)
  0x91, 0x02,                   //   OUTPUT (Data,Var,Abs)
  0xc0                        // END_COLLECTION
};

// Array of hid descriptors and corresponding report descriptors.
code hal_usb_hid_t g_usb_hid_hids[] = 
{
  { &g_usb_conf_desc.hid1, g_usb_hid_report_data, sizeof(g_usb_hid_report_data) },
// Add your own report descriptors like this if needed:
//{ &g_usb_conf_desc.myHidDescriptor, g_usb_hid_report_myReport, sizeof(g_usb_hid_report_myReport) },  
};

//-----------------------------------------------------------------------------
// Configuration, interface and endpoint descriptors
// From "USB Device Class Definition for Human Interface Devices (HID)".
// Section 7.1:
// "When a Get_Descriptor(Configuration) request is issued,
// it returns the Configuration descriptor, all Interface descriptors,
// all Endpoint descriptors, and the HID descriptor for each interface."
//-----------------------------------------------------------------------------
code const usb_conf_desc_templ_t g_usb_conf_desc =
{
  { // configuration_descriptor 
    sizeof(hal_usb_conf_desc_t),          // Length
    USB_DESC_CONFIGURATION,               // Type                             
    SWAP(sizeof(usb_conf_desc_templ_t)),  // Totallength
    0x01,                                 // NumInterfaces
    0x01,                                 // bConfigurationValue
    0x00,                                 // iConfiguration
    0x80,                                 // bmAttributes (0x80 No Remote Wakeup at the moment)
    0x32                                  // MaxPower (in 2mA units) = 100 * 2mA
  },
//-----------------------------------------------------------------------------
// HID Data interface descriptors
//-----------------------------------------------------------------------------
  // Interface descriptor
  { 
    sizeof(hal_usb_if_desc_t),            // bLength
    USB_DESC_INTERFACE,                   // bDescriptorType
    0x00,                                 // bInterfaceNumber
    0x00,                                 // bAlternateSetting
    0x02,                                 // bNumEndpoints 
    USB_DEVICE_CLASS_HUMAN_INTERFACE,     // bInterfaceClass
    0x00,                                 // bInterfaceSubClass 
    0x00,                                 // bInterfaceProtocol 
    0x00,                                 // iInterface
  },
  // HID descriptor
  {
    sizeof(hal_usb_hid_desc_t),           // bLength
    USB_CLASS_DESCRIPTOR_HID,             // bDescriptorType
    SWAP(0x0110),                         // bcdHID -  HID Spec 1.11
    0x00,                                 // bCountryCode 
    0x01,                                 // bNumDescriptors - Number of HID class descriptors to follow
    USB_CLASS_DESCRIPTOR_REPORT,          // bDescriptorType - Report descriptor type 
    SWAP(sizeof(g_usb_hid_report_data)),  // wDescriptorLength - Report descriptor length
  },
  // Endpoint Descriptor EP1IN 
  {
    sizeof(hal_usb_ep_desc_t),            // bLength
    USB_DESC_ENDPOINT,                    // bDescriptorType
    0x81,                                 // bEndpointAddress
    USB_ENDPOINT_TYPE_INTERRUPT,          // bmAttributes
    SWAP(EP1_2_PACKET_SIZE),              // wMaxPacketSize
    EP1_POLLING_INTERVAL                  // bInterval
  },
  // Endpoint Descriptor EP2OUT 
  {
    sizeof(hal_usb_ep_desc_t),            // bLength
    USB_DESC_ENDPOINT,                    // bDescriptorType
    0x02,                                 // bEndpointAddress
    USB_ENDPOINT_TYPE_INTERRUPT,          // bmAttributes
    SWAP(EP1_2_PACKET_SIZE),              // wMaxPacketSize
    EP2_POLLING_INTERVAL                  // bInterval
  }
};

//-----------------------------------------------------------------------------
// String descriptors
//-----------------------------------------------------------------------------

// Number of chars times 2 to get unicode length. 
// Add two for descriptor type and size.
#define USB_STRING_IDX_1_DESC_LENGTH 20*2 + 2 
#define USB_STRING_IDX_2_DESC_LENGTH 35*2 + 2

code uint8_t g_usb_string_desc_1[] = {
  USB_STRING_IDX_1_DESC_LENGTH, 
  USB_DESC_STRING, 
  'N',00,
  'o',00,
  'r',00,
  'd',00,
  'i',00,
  'c',00,
  ' ',00,
  'S',00,
  'e',00,
  'm',00,
  'i',00,
  'c',00,
  'o',00,
  'n',00,
  'd',00,
  'u',00,
  'c',00,
  't',00,
  'o',00,
  'r',00 
};

code uint8_t g_usb_string_desc_2[] = {
  USB_STRING_IDX_2_DESC_LENGTH,
  USB_DESC_STRING, 
  'N',00,
  'o',00,
  'r',00,
  'd',00,
  'i',00,
  'c',00,
  ' ',00,
  'S',00,
  'e',00,
  'm',00,
  'i',00,
  'c',00,
  'o',00,
  'n',00,
  'd',00,
  'u',00,
  'c',00,
  't',00,
  'o',00,
  'r',00, 
  ' ',00, 
  'U',00,
  'S',00,
  'B',00,
  ' ',00,
  'R',00,
  'F',00,
  ' ',00,
  'A',00,
  'd',00,
  'a',00,
  'p',00,
  't',00,
  'e',00, 
  'r',00,
};

code usb_string_desc_templ_t g_usb_string_desc = {
  g_usb_string_desc_1,
  g_usb_string_desc_2,
};