#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include <freertos/semphr.h>
#include <esp_http_server.h>

#include "webserver.hpp"
#include "system.hpp"
#include "WiFiCreds.h" // Include the header file with credentials

static const char *TAG = "Webserver";

void connectWiFi()
{
     ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());
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

esp_err_t handleRootRequest(httpd_req_t *req)
{
     const char *resp_str = "Hello World";
     httpd_resp_send(req, resp_str, strlen(resp_str));
     return ESP_OK;
}

httpd_handle_t server = NULL;
httpd_uri_t uri_current_image = {
    .uri = "/current_image",
    .method = HTTP_GET,
    .handler = handleImageRequest,
    .user_ctx = NULL};

httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = handleRootRequest,
    .user_ctx = NULL};

void startServer()
{
     ESP_LOGI(TAG, "[%s] Running on core %d", __func__, xPortGetCoreID());
     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
     if (httpd_start(&server, &config) == ESP_OK)
     {
          httpd_register_uri_handler(server, &uri_current_image);
          httpd_register_uri_handler(server, &uri_root);
     }
}
