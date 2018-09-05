#include <usb/configuration.h>
#include <support/usb/devices/ftdi.h>

static int _fill_conf_desc(int conf_index, usb_configuration_descriptor_t *conf_desc, int buffer_size);

static usb_device_configuration_t _conf;
static const usb_device_configuration_driver_t _conf_driver = { 
	.FillConfigurationDescriptor = _fill_conf_desc };
static usb_device_interface_t _ftdi_if1;
static usb_device_interface_t _ftdi_if2;

void usb_device_add_configurations()
{
	usb_device_config_create(&_conf, 0x11, &_conf_driver);	
	usb_device_interface_create(&_ftdi_if1, __usb_ftdi_driver);
	usb_device_config_add_interface(&_conf, &_ftdi_if1);
//	usb_device_interface_create(&_ftdi_if2, __usb_ftdi_driver);
//	usb_device_config_add_interface(&_conf, &_ftdi_if2);
	usb_device_config_register(&_conf);
}

static int _fill_conf_desc(int conf_index, usb_configuration_descriptor_t *conf_desc, int buffer_size)
{
	if (buffer_size < sizeof(usb_configuration_descriptor_t))
		return 0;

	switch(conf_index)
	{
		case 0:
			conf_desc->Attributes = USB_CONFIG_BUS_POWERED;
			conf_desc->MaxPower = USB_CONFIG_POWER(200);
			return sizeof(usb_configuration_descriptor_t);
	}
	return 0;
}

bool usb_device_setup_vendor_request(usb_request_t *req, void **pdata, int *plength)
{
	return ftdi_vendor_request(req, pdata, plength);
}
