#ifndef USB_STREAM
#define USB_STREAM

#include <esp_err.h>


esp_err_t usb_init(void);

int usb_send_data(const uint8_t *data, size_t len, size_t width, size_t height);

#endif /* USB */
