#undef EPS
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#define EPS 192

#include "esp_system.h"
#include <esp_log.h>
#include <iostream>
#include "esp_camera.h"
#include <vector>
#include <inttypes.h>

#include "image_processing.hpp"

using namespace cv;

static const char *TAG = "IMG_PROC";

void processImage(camera_fb_t *fb) {
    // it seems like imdecode does not work: https://github.com/joachimBurket/esp32-opencv/issues/18
    Mat img2 = Mat(fb->height, fb->width, CV_8UC3, fb->buf);


    // Mat img = Mat(fb->height, fb->width, CV_8UC3, Scalar(0, 0, 255));
    // std::cout << img << std::endl;


    std::vector img_data(fb->buf, fb->buf + fb->len);


    // printf("Free heap before: %lu bytes\n", esp_get_free_heap_size());
    // Mat img2 = imdecode(img_data, IMREAD_COLOR);
    // printf("Free heap after: %lu bytes\n", esp_get_free_heap_size());
    // std::cout << img2 << std::endl;

    int a = 5;
    int b = a + 54;

    printf("a=%d b=%d\n", a, b);
    return;
}
