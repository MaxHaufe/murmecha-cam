//#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include <freertos/semphr.h>

#include <esp_http_server.h>

#include <inttypes.h>
#include <string>


#include "esp_camera.h"

// using namespace cv;
// using namespace std;

#include "system.hpp"
//#include "webserver.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include "camera.hpp"
#include "image_processing.hpp"
#include "usb_stream.hpp"

// global vars
SemaphoreHandle_t imgMutex = xSemaphoreCreateMutex();
std::vector<uint8_t> imageBuffer;
std::vector<uint8_t> procImageBuffer;

using namespace cv;

// anscheinend braucht man das
extern "C" {
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

esp_err_t init_spiffs() {
    ESP_LOGI("spiffs_init", "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 10,
        .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("spiffs_init", "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE("spiffs_init", "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE("spiffs_init", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE("spiffs_init", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI("spiffs_init", "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing...");
    ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());

    /* SPIFFS init */
    init_spiffs();
    ESP_LOGI(TAG, "spiffs init done");

    // connectWiFi();
    // startServer();

    usb_init();
    // esp_log_set_level_master(ESP_LOG_NONE); //only for recording
    if (ESP_OK != MurmechaCam::init_camera()) {
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    std::string test_img = "/spiffs/captured_image_1748952173.png";

    Mat img = imread(test_img, IMREAD_COLOR);
    cvtColor(img, img, COLOR_BGR2RGB);

    while (true) {
//          UBaseType_t high_water1 = uxTaskGetStackHighWaterMark(NULL);
//          ESP_LOGI(TAG, "Stack free: %lu", high_water1);
//          ESP_LOGI(TAG, "Taking picture...");

        // ImageData img = MurmechaCam::get_rgb_image();
        // usb_send_data(img.buf, img.len, img.width, img.height);

        uint8_t *buffer = img.data;
        size_t len = img.total() * img.channels();
        size_t width = img.cols;
        size_t height = img.rows;
        usb_send_data(buffer, len, width, height);


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
