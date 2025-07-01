

//#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include <freertos/semphr.h>

#include <esp_http_server.h>

#include <inttypes.h>

#include "esp_camera.h"

// using namespace cv;
// using namespace std;

#include "system.hpp"
//#include "webserver.hpp"
#include "camera.hpp"
#include "image_processing.hpp"
#include "usb_stream.hpp"

// global vars
SemaphoreHandle_t imgMutex = xSemaphoreCreateMutex();
std::vector<uint8_t> imageBuffer;
std::vector<uint8_t> procImageBuffer;

// anscheinend braucht man das
extern "C"
{
void app_main(void);
}

static const char *TAG = "ESP32_Server";

void receiveImageTask(camera_fb_t *img) {

    ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());

    std::vector<uint8_t> newImage(img->buf, img->buf + img->len);

    xSemaphoreTake(imgMutex, portMAX_DELAY);
    imageBuffer = std::move(newImage);
    // procImageBuffer = processImage(newImage);
    xSemaphoreGive(imgMutex);
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing...");
    ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());

    // connectWiFi();
    // startServer();

    usb_init();
    // esp_log_set_level_master(ESP_LOG_NONE); //only for recording

    if (ESP_OK != init_camera()) {
        return;
    }
    while (1) {
//          UBaseType_t high_water1 = uxTaskGetStackHighWaterMark(NULL);
//          ESP_LOGI(TAG, "Stack free: %lu", high_water1);
//          ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *fb = esp_camera_fb_get();

        // if jpg, convert to RGB
        if (fb->format == PIXFORMAT_JPEG) {
//              uint8_t * buf = nullptr;
//              size_t buf_len = 0;
//              bool converted = frame2bmp(fb, &buf, &buf_len);
            size_t size_rgb888 = fb->width * fb->height * 3;
            ESP_LOGI(TAG, "width: %lu, height: %lu", fb->width, fb->height);
            ESP_LOGI(TAG, "required buffer: %zu", size_rgb888);
//            auto *buf = static_cast<uint8_t *>(malloc(size_rgb888));
            auto *buf = static_cast<uint8_t *>(heap_caps_malloc(size_rgb888, MALLOC_CAP_SPIRAM));
//            return;
            if (!buf) {
                ESP_LOGE(TAG, "Failed to allocate memory for RGB buffer");
                esp_camera_fb_return(fb);
                return;
            }

            bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, buf);
            if (converted) {
                usb_send_data(buf, size_rgb888, fb->width, fb->height);
            }
            free(buf);

        } else {
            usb_send_data(fb->buf, fb->len, fb->width, fb->height);
        }

        // process stuff here
        // processImage(fb);


        // receiveImageTask(fb);
        // std::vector<uint8_t> newImage(fb->buf, fb->buf + fb->len);
        // send_image(fb);
//          ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", fb->len);
        esp_camera_fb_return(fb);


//          UBaseType_t high_water2 = uxTaskGetStackHighWaterMark(NULL);
//          ESP_LOGI(TAG, "Stack free: %lu", high_water2); // 18648 without debugger

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // esp_vfs_spiffs_conf_t conf = {
    //     .base_path = "/spiffs",
    //     .partition_label = NULL,
    //     .max_files = 5,
    //     .format_if_mount_failed = true};
    // ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));


    // xTaskCreatePinnedToCore(
    //      receiveImageTask, "ReceiveImage", 8192, (void *)copy, 1, NULL, 1);
}
