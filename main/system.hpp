#ifndef SYSTEM
#define SYSTEM

#include <esp_http_server.h>
#include <freertos/semphr.h>
#include <vector>

extern std::vector<uint8_t> imageBuffer;
extern SemaphoreHandle_t imgMutex;

#endif /* SYSTEM */
