#ifndef SYSTEM
#define SYSTEM

#include <esp_http_server.h>
#include <freertos/semphr.h>
#include <vector>
#include <inttypes.h>


extern std::vector<uint8_t> imageBuffer;
extern std::vector<uint8_t> procImageBuffer;
extern SemaphoreHandle_t imgMutex;

#endif /* SYSTEM */
