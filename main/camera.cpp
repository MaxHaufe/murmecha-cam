#include <esp_log.h>

#include "esp_camera.h"
# include "camera.hpp"

#include <esp_heap_caps.h>
#include <stdexcept>

// https://github.com/espressif/esp32-camera/blob/master/examples/camera_example/main/take_picture.c

static const char *TAG = "Camera";
// https://www.tehonline.co.uk/cdn/shop/files/esp32-s3-wroom-n16R8-ov5640-cam-2-connections.jpg?v=1729939997&width=1445
// https://wiki.sophior.com/books/esp32-s3-wroom-1/page/configuration-and-initialization
#define CAM_PIN_PWDN -1 // was: 38
#define CAM_PIN_RESET -1   //software reset will be performed
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_D0 11
#define CAM_PIN_D1 9
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 18
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,

    // TODO: for NT99141
    // .pin_d7 = CAM_PIN_D0,
    // .pin_d6 = CAM_PIN_D1,
    // .pin_d5 = CAM_PIN_D2,
    // .pin_d4 = CAM_PIN_D3,
    // .pin_d3 = CAM_PIN_D4,
    // .pin_d2 = CAM_PIN_D5,
    // .pin_d1 = CAM_PIN_D6,
    // .pin_d0 = CAM_PIN_D7,
    // .xclk_freq_hz = 6000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    //     .frame_size = FRAMESIZE_QVGA,

    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,


    // lowering the frequency is necessary for PIXFORMAT modes other than JPEG: https://github.com/espressif/esp32-camera/issues/556
    // .xclk_freq_hz = 20000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    // .xclk_freq_hz = 16000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .xclk_freq_hz = 10000000, //EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    // .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    //     .pixel_format = PIXFORMAT_GRAYSCALE,//YUV422,GRAYSCALE,RGB565,JPEG
    .pixel_format = PIXFORMAT_RGB565,//YUV422,GRAYSCALE,RGB565,JPEG // TODO: this works with VGA
    //    .frame_size = FRAMESIZE_UXGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
    //    .frame_size = FRAMESIZE_UXGA, // sometimes not working
    .frame_size = FRAMESIZE_VGA,
    //     .frame_size = FRAMESIZE_QVGA,

    // TODO: workaround wenn nicht geht -> 20 Mhz clock speed -> flash -> VGA 16Mhz clock speed -> flash
    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY //CAMERA_GRAB_LATEST. Sets when buffers should be filled
};


void printSensorInfo() {
    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr) {
        ESP_LOGI(TAG, "Camera sensor info:");
        ESP_LOGI(TAG, "PID: 0x%02X", s->id.PID);
        ESP_LOGI(TAG, "VER: 0x%02X", s->id.VER);
        ESP_LOGI(TAG, "MIDL: 0x%02X", s->id.MIDL);
        ESP_LOGI(TAG, "MIDH: 0x%02X", s->id.MIDH);

        // // Common sensor IDs:
        // if (s->id.PID == camera_sensor[]) {
        //     ESP_LOGI(TAG, "Detected: NT99141");
        // } else if (s->id.PID == 0x26 && s->id.VER == 0x42) {
        //     ESP_LOGI(TAG, "Detected: OV2640");
        // } else if (s->id.PID == 0x56 && s->id.VER == 0x48) {
        //     ESP_LOGI(TAG, "Detected: OV5640");
        // } else if (s->id.PID == 0x77 && s->id.VER == 0x21) {
        //     ESP_LOGI(TAG, "Detected: OV7725");
        // } else {
        //     ESP_LOGI(TAG, "Unknown sensor");
        // }
    }
}

ImageData::~ImageData() {
    free(this->buf);
}



esp_err_t MurmechaCam::init_camera() {
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    printSensorInfo();

    return ESP_OK;
}

ImageData MurmechaCam::get_rgb_image() {
    camera_fb_t *fb = esp_camera_fb_get();


    // if jpg, convert to RGB
    if (fb->format == PIXFORMAT_JPEG) {

        size_t size_rgb888 = fb->width * fb->height * 3;
        ESP_LOGI(TAG, "width: %lu, height: %lu", fb->width, fb->height);
        ESP_LOGI(TAG, "required buffer: %zu", size_rgb888);
        //            auto *buf = static_cast<uint8_t *>(malloc(size_rgb888));
        auto buf = static_cast<uint8_t *>(heap_caps_malloc(size_rgb888, MALLOC_CAP_SPIRAM));
        //            return;
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate memory for RGB buffer");

            throw std::runtime_error("Failed to allocate RGB buffer");
        }

        // FIXME: for some reason this method does not return rgb but bgr....
        // https://github.com/espressif/esp32-camera/issues/379
        bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, buf);
        auto img = ImageData(buf, size_rgb888, fb->width, fb->height);
        esp_camera_fb_return(fb); // give back
        if (!converted) {
            // TODO: error handling
            // usb_send_data(buf, size_rgb888, fb->width, fb->height)
            throw std::runtime_error("Failed to convert RGB buffer");
        }
        return img;
        // free(buf);
    }

    // rgb image (no jpg conversion

    // this might seem stupid, but it allows me to return the buffer immediately
    auto buf = static_cast<uint8_t *>(heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM));
    if (!buf) {
        // TODO: error handling
        throw std::runtime_error("Failed to allocate memory for RGB buffer");
    }

    memcpy(buf, fb->buf, fb->len);

    auto img  = ImageData(buf, fb->len, fb->width, fb->height);
    esp_camera_fb_return(fb);

    return img;

    // usb_send_data(fb->buf, fb->len, fb->width, fb->height);

    // process stuff here
    // processImage(fb);


    // receiveImageTask(fb);
    // std::vector<uint8_t> newImage(fb->buf, fb->buf + fb->len);
    // send_image(fb);
    //          ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", fb->len);

}

