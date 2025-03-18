#undef EPS
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#define EPS 192

#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include <freertos/semphr.h>

#include <esp_http_server.h>

#include <inttypes.h>

// using namespace cv;
// using namespace std;

#include "system.hpp"
#include "webserver.hpp"

// global vars
SemaphoreHandle_t imgMutex = xSemaphoreCreateMutex();
std::vector<uint8_t> imageBuffer;

// anscheinend braucht man das
extern "C"
{
     void app_main(void);
}

static const char *TAG = "ESP32_Server";

void receiveImageTask(void *param)
{
     for (;;)
     {
          ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());

          std::vector<uint8_t> newImage(1024, 255);

          xSemaphoreTake(imgMutex, portMAX_DELAY);
          imageBuffer = std::move(newImage);
          xSemaphoreGive(imgMutex);

          vTaskDelay(pdMS_TO_TICKS(5000));
     }
}

void app_main(void)
{
     ESP_LOGI(TAG, "Initializing...");

     esp_vfs_spiffs_conf_t conf = {
         .base_path = "/spiffs",
         .partition_label = NULL,
         .max_files = 5,
         .format_if_mount_failed = true};
     ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

     connectWiFi();
     startServer();

     xTaskCreatePinnedToCore(
         receiveImageTask, "ReceiveImage", 8192, NULL, 1, NULL, 1);

     ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());
}
