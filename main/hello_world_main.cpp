

//#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include <freertos/semphr.h>

// #include <esp_http_server.h>

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
    if (ESP_OK != MurmechaCam::init_camera()) {
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    std::vector<framesize_t> sizes = {
        FRAMESIZE_96X96,    // 96x96
        FRAMESIZE_QQVGA,    // 160x120
        FRAMESIZE_128X128,    // 128x128
        FRAMESIZE_QCIF,     // 176x144
        FRAMESIZE_HQVGA,    // 240x176
        FRAMESIZE_240X240,  // 240x240
        FRAMESIZE_QVGA,     // 320x240
        // FRAMESIZE_320X320,  // 320x320
        FRAMESIZE_CIF,      // 400x296
        FRAMESIZE_HVGA,     // 480x320
        FRAMESIZE_VGA,      // 640x480
        };

    for (const auto& res: sizes) {
        esp_camera_deinit();
        if (ESP_OK != MurmechaCam::init_camera(res)) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        int i=0;

        while (i<10) {
            //          UBaseType_t high_water1 = uxTaskGetStackHighWaterMark(NULL);
            //          ESP_LOGI(TAG, "Stack free: %lu", high_water1);
            //          ESP_LOGI(TAG, "Taking picture...");


            ImageData img = MurmechaCam::get_rgb_image();
            usb_send_data(img.buf.get(), img.len, img.width, img.height);



            //          UBaseType_t high_water2 = uxTaskGetStackHighWaterMark(NULL);
            //          ESP_LOGI(TAG, "Stack free: %lu", high_water2); // 18648 without debugger

            vTaskDelay(pdMS_TO_TICKS(200));
            i ++;
        }
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