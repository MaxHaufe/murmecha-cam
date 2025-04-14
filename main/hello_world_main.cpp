

#include <esp_wifi.h>
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
#include "webserver.hpp"
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

void receiveImageTask(camera_fb_t *img )
{

     ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());

     std::vector<uint8_t> newImage(img->buf, img->buf + img->len);    

     xSemaphoreTake(imgMutex, portMAX_DELAY);
     imageBuffer = std::move(newImage);
     // procImageBuffer = processImage(newImage);
     xSemaphoreGive(imgMutex);
}

void app_main(void)
{
     ESP_LOGI(TAG, "Initializing...");
     ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());

     // connectWiFi();
     // startServer();

     usb_init();

     if (ESP_OK != init_camera())
     {
          return;
     }
     while (1)
     {
          UBaseType_t high_water1 = uxTaskGetStackHighWaterMark(NULL);
          ESP_LOGI(TAG, "Stack free: %lu", high_water1);
          ESP_LOGI(TAG, "Taking picture...");
          camera_fb_t *img = esp_camera_fb_get();

          // process stuff here
          processImage(img);

          // receiveImageTask(img);
          // std::vector<uint8_t> newImage(img->buf, img->buf + img->len);
          // send_image(img);
          ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", img->len);
          esp_camera_fb_return(img);


          UBaseType_t high_water2 = uxTaskGetStackHighWaterMark(NULL);
          ESP_LOGI(TAG, "Stack free: %lu", high_water2); // 18648 without debugger

          vTaskDelay(pdMS_TO_TICKS(5000));
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
