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
#include "WiFiCreds.h" // Include the header file with credentials
#include <esp_http_server.h>

#include <iostream>
#include <inttypes.h>

// using namespace cv;
// using namespace std;

static const char *TAG = "ESP32_Server";
std::vector<uint8_t> imageBuffer;
SemaphoreHandle_t imgMutex;
httpd_handle_t server = NULL;

// anscheinend braucht man das
extern "C"
{
     void app_main(void);
}

void connectWiFi()
{
     ESP_ERROR_CHECK(nvs_flash_init());
     ESP_ERROR_CHECK(esp_netif_init());
     ESP_ERROR_CHECK(esp_event_loop_create_default());
     esp_netif_create_default_wifi_sta();
     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

     wifi_config_t wifi_config = {};
     strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
     strcpy((char *)wifi_config.sta.password, WIFI_PASSWORD);

     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
     ESP_ERROR_CHECK(esp_wifi_start());

     ESP_LOGI(TAG, "Connecting to WiFi...");
     while (esp_wifi_connect() != ESP_OK)
     {
          vTaskDelay(pdMS_TO_TICKS(1000));
          ESP_LOGI(TAG, "Retrying WiFi connection...");
     }
     ESP_LOGI(TAG, "Connected to WiFi");
}

esp_err_t handleImageRequest(httpd_req_t *req)
{
     xSemaphoreTake(imgMutex, portMAX_DELAY);
     if (imageBuffer.empty())
     {
          httpd_resp_send(req, "No image available", HTTPD_RESP_USE_STRLEN);
     }
     else
     {
          httpd_resp_set_type(req, "image/jpeg");
          httpd_resp_send(req, (const char *)imageBuffer.data(), imageBuffer.size());
     }
     xSemaphoreGive(imgMutex);
     return ESP_OK;
}

httpd_uri_t uri_get = {
    .uri = "/current_image",
    .method = HTTP_GET,
    .handler = handleImageRequest,
    .user_ctx = NULL};

void startServer()
{
     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
     if (httpd_start(&server, &config) == ESP_OK)
     {
          httpd_register_uri_handler(server, &uri_get);
     }
}

void receiveImageTask(void *param)
{    
     for (;;)
     {
          ESP_LOGI(TAG, "Running on core %d", xPortGetCoreID());
          std::vector<uint8_t> newImage(1024, 255);

          xSemaphoreTake(imgMutex, portMAX_DELAY);
          imageBuffer = std::move(newImage);
          xSemaphoreGive(imgMutex);

          vTaskDelay(pdMS_TO_TICKS(500));
     }
}

void app_main(void)
{
     ESP_LOGI(TAG, "Initializing...");
     imgMutex = xSemaphoreCreateMutex();

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
     ESP_LOGI(TAG, "Running on core %d", xPortGetCoreID());
}
