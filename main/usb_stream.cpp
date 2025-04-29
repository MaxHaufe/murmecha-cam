#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"


#include "usb_stream.hpp"

#include <esp_err.h>
#include <esp_vfs_dev.h>
#include <driver/usb_serial_jtag.h>
#include <driver/usb_serial_jtag_vfs.h>


/**
 * Initializes the USB Serial/JTAG interface
 *
 * @return ESP_OK on success or error code on failure
 */
#define TAG "USB_DATA"

esp_err_t usb_init() {
    ESP_LOGI(TAG, "Initializing USB Serial/JTAG interface");

    // Initialize USB Serial/JTAG driver
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    return ESP_OK;
}

/**
* Sends data over USB Serial/JTAG interface
 *
 * @param data Pointer to the data buffer to be sent
 * @param len Length of the data in bytes
 * @return Number of bytes written or -1 on error
 */
int usb_send_data(const uint8_t *data, const size_t len, const size_t width, const size_t height) {
    ESP_LOGI(TAG, "Sending %d bytes over USB", len);
    // Send start marker
    uint8_t header[16];
    header[0] = 0xAA;
    header[1] = 0xBB;
    header[2] = 0xCC;
    header[3] = 0xDD;

    const auto len_cast = static_cast<uint32_t>(len);
    memcpy(&header[4], &len_cast, sizeof(uint32_t));

    const auto width_cast = static_cast<uint32_t>(width);
    memcpy(&header[8], &width_cast, sizeof(uint32_t));

    const auto height_cast = static_cast<uint32_t>(height);
    memcpy(&header[12], &height_cast, sizeof(uint32_t));

    usb_serial_jtag_write_bytes(header, sizeof(header) , 100 / portTICK_PERIOD_MS);

    // Send image data in chunks
    size_t total_written = 0;

    while (total_written < len) {
        constexpr size_t chunk_size = 512;
        const size_t to_write = (len - total_written < chunk_size) ? (len - total_written) : chunk_size;

        const int written = usb_serial_jtag_write_bytes(data + total_written,
                                                        to_write,
                                                        100 / portTICK_PERIOD_MS);
        if (written < 0) return -1;

        total_written += written;
        // vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    // Send end marker
    constexpr uint8_t footer[4] = {0xDD, 0xCC, 0xBB, 0xAA};
    usb_serial_jtag_write_bytes(footer, 4, 100 / portTICK_PERIOD_MS);

    return static_cast<int>(total_written);
}
