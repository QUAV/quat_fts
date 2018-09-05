#ifndef usb_comms_h
#define usb_comms_h

void usb_comms_initialize();
void usb_comms_write(unsigned view, const char *format, ...);

#endif // usb_comms_h


