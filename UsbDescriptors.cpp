
/*-
 * $Copyright$
-*/

#include <usb/UsbTypes.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#if !defined(HOSTBUILD)
#define FIXED_DATA   __attribute__((aligned(4), section(".fixeddata")))
#else
#define FIXED_DATA
#endif

extern const ::usb::UsbDeviceDescriptor_t usbDeviceDescriptor FIXED_DATA = {
    .m_bLength              = sizeof(::usb::UsbDeviceDescriptor_t),                     /* m_bLength */
    .m_bDescriptorType      = ::usb::UsbDescriptorTypeId_t::e_Device,                   /* m_bDescriptorType */
    .m_bcdUsb               = { 0x00, 0x02 },                                           /* m_bcdUsb */
    .m_bDeviceClass         = ::usb::UsbInterfaceClass_e::e_UsbInterface_BaseClass,     /* m_bDeviceClass */
    .m_bDeviceSubClass      = 0x00,                                                     /* m_bDeviceSubClass */
    .m_bDeviceProtocol      = 0x00,                                                     /* m_bDeviceProtocol */
    .m_bMaxPacketSize0      = 64,                                                       /* m_bMaxPacketSize0 -- Should be 64 for a USB Full Speed Device. */
    .m_idVendor             = { 0xad, 0xde },                                           /* m_idVendor */
    .m_idProduct            = { 0xef, 0xbe },                                           /* m_idProduct */
    .m_bcdDevice            = { 0xfe, 0xca },                                           /* m_bcdDevice */
    .m_iManufacturer        = ::usb::UsbStringDescriptorId_t::e_StrDesc_Manufacturer,   /* e_iManufacturer */
    .m_iProduct             = ::usb::UsbStringDescriptorId_t::e_StrDesc_Product,        /* e_iProduct */
    .m_iSerialNumber        = ::usb::UsbStringDescriptorId_t::e_StrDesc_SerialNumber,   /* e_iSerialNumber */
    .m_bNumConfigurations   = 1                                                         /* e_bNumConfigurations */
};

/*******************************************************************************
 * USB Device String Descriptors
 ******************************************************************************/
static const ::usb::UsbLangId_t usbSupportedLanguageIds[] = { 0x0409, 0x0000 };

extern const auto usbStringDescriptorManufacturer   FIXED_DATA  = ::usb::UsbStringDescriptor(u"PhiSch.org");
extern const auto usbStringDescriptorSerialNumber   FIXED_DATA  = ::usb::UsbStringDescriptor(u"D2209DFF-B80D-4E44-A8E5-466ADCCE7E30");
extern const auto usbStringDescriptorProduct        FIXED_DATA  = ::usb::UsbStringDescriptor(u"PhiSch.org USB HID Demo (on STM32F4 BlackPill)");
extern const auto usbStringDescriptorConfiguration  FIXED_DATA  = ::usb::UsbStringDescriptor(u"PhiSch.org USB Human Interface Device (HID) Configuration");
extern const auto usbStringDescriptorInterface      FIXED_DATA  = ::usb::UsbStringDescriptor(u"PhiSch.org USB Human Interface Device (HID) Interface");

extern const ::usb::UsbStringDescriptorTable_t usbStringDescriptors FIXED_DATA = {
        .m_languageIds      = ::usb::UsbLangIdStringDescriptor_t(usbSupportedLanguageIds),
        .m_padding          = {},
        .m_stringDescriptors = {
            .m_descriptors = {
                .m_languageIds      = (const usb::UsbLangIdStringDescriptor_t *) 0xdeadc0de,
                .m_manufacturer     = usbStringDescriptorManufacturer.data(),
                .m_product          = reinterpret_cast<const uint8_t *>(&usbStringDescriptorProduct),
                .m_serialNumber     = usbStringDescriptorSerialNumber.data(),
                .m_configuration    = usbStringDescriptorConfiguration.data(),
                .m_interface        = usbStringDescriptorInterface.data()
            }
        }
};

/*
 * The USB HID Report Descriptor is somewhat of black magic to me.
 * 
 * This is the Example HID Report Descriptor for a Mouse from the USB HID 1.11 Specification, Appendix E.10.
 */
extern const uint8_t hidMouseReportDescriptor[50] FIXED_DATA = {
    0x05,   0x01,
    0x09,   0x02,
    0xA1,   0x01,
    0x09,   0x01,
    
    0xA1,   0x00,
    0x05,   0x09,
    0x19,   0x01,
    0x29,   0x03,
    
    0x15,   0x00,
    0x25,   0x01,
    0x95,   0x03,
    0x75,   0x01,
    
    0x81,   0x02,
    0x95,   0x01,
    0x75,   0x05,
    0x81,   0x01,
    
    0x05,   0x01,
    0x09,   0x30,
    0x09,   0x31,
    0x15,   0x81,
    
    0x25,   0x7F,
    0x75,   0x08,
    0x95,   0x02,
    0x81,   0x06,

    0xc0,   0xc0
};

extern const struct UsbConfigurationDescriptor_s {
    struct ::usb::UsbConfigurationDescriptorT<void *, 0>    m_configDescrHdr;
    struct ::usb::UsbInterfaceDescriptorT<0>                m_hidInterface;
    struct ::usb::UsbHidDescriptor                          m_hidDescriptor;
    struct ::usb::UsbEndpointDescriptor                     m_hidEndpoint;
} __attribute__((packed)) usbConfigurationDescriptor FIXED_DATA = {
    .m_configDescrHdr = {
        .m_bLength                  = sizeof( decltype(usbConfigurationDescriptor.m_configDescrHdr)),
        .m_bDescriptorType          = ::usb::UsbDescriptorTypeId_e::e_Configuration,
        .m_wTotalLength = {
            .m_loByte               = (sizeof(decltype(usbConfigurationDescriptor)) >> 0),
            .m_hiByte               = (sizeof(decltype(usbConfigurationDescriptor)) >> 8)
        },
        .m_bNumInterfaces           = 1,
        .m_bConfigurationValue      = 1,
        .m_iConfiguration           = 4, /* Index of m_configuration within usbStringDescriptors.m_stringDescriptorTable */
        .m_bmAttributes             = 0x80      // USB 2.0 Spec demands this Bit to always be set
                                    | 0x40      // Set to 0x40 for Self-powered Device, 0x00 for Bus-powered
                                    | 0x20,     // Set to 0x20 to indicate Remote Wake-up is supported. Needed to detect HID Device on macOS
        .m_bMaxPower                = 5,        // Power consumption in Units of 2mA
        .m_interfaces               = {}
    },
    .m_hidInterface = {
        .m_bLength                  = sizeof(decltype(usbConfigurationDescriptor.m_hidInterface)),
        .m_bDescriptorType          = ::usb::UsbDescriptorTypeId_e::e_Interface,
        .m_bInterfaceNumber         = 0,
        .m_bAlternateSetting        = 0,
        .m_bNumEndpoints            = 1,
        .m_bInterfaceClass          = ::usb::UsbInterfaceClass_e::e_UsbInterface_HumanInterfaceDevice,
        .m_bInterfaceSubClass       = ::usb::UsbHid_SubclassCode_e::e_BootInterface,
        .m_bInterfaceProtocol       = ::usb::UsbHid_ProtocolCode_e::e_Mouse,
        .m_iInterface               = 5, /* Index of m_interface within usbStringDescriptors.m_stringDescriptorTable */
        .m_endpoints                = {}
    },
    .m_hidDescriptor = {
        .m_bLength                  = sizeof(decltype(usbConfigurationDescriptor.m_hidDescriptor)),
        .m_bDescriptorType          = ::usb::UsbHidDescriptorTypeId_e::e_Hid,
        .m_bcdHID = {
            .m_loByte               = 0x01,
            .m_hiByte               = 0x01
        },
        .m_bCountryCode             = ::usb::UsbHidCountryCode_e::e_NotSupported,
        .m_bNumDescriptors          = 1,
        .m_bDescriptorTypeName      = ::usb::UsbHidDescriptorTypeId_e::e_HidReport,
        .m_wDescriptorLength = {
            .m_loByte               = sizeof(hidMouseReportDescriptor),
            .m_hiByte               = 0x00
        },
    },
    .m_hidEndpoint = {
        .m_bLength                  = sizeof(decltype(usbConfigurationDescriptor.m_hidEndpoint)),
        .m_bDescriptorType          = ::usb::UsbDescriptorTypeId_e::e_Endpoint,
        .m_bEndpointAddress         = 0x81,
        .m_bmAttributes             = 3,    // IRQ
        .m_wMaxPacketSize = {
            .m_loByte               = 8,
            .m_hiByte               = 0
        },
        .m_bInterval                = 0x0A
    }
};

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */
