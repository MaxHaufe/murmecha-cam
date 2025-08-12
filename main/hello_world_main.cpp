//#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include <freertos/semphr.h>

#include <esp_http_server.h>

#include <inttypes.h>
#include <string>
#include <set>


#include "esp_camera.h"

// using namespace cv;
// using namespace std;

#include "system.hpp"
//#include "webserver.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include "zhangsuen.hpp"
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

// Get labels near a line with given radius
std::set<int> getNearbyLabels(const Mat &labeledImg, Point start, Point end, int radius = 10) {
    std::set<int> labels;

    // Calculate line length and generate points along the line
    double length = norm(end - start);
    int numPoints = static_cast<int>(length);

    for (int i = 0; i <= numPoints; i++) {
        double t = static_cast<double>(i) / numPoints;
        int x = static_cast<int>(start.x + t * (end.x - start.x));
        int y = static_cast<int>(start.y + t * (end.y - start.y));

        // Check neighborhood around this point
        int yMin = max(0, y - radius);
        int yMax = min(labeledImg.rows, y + radius);
        int xMin = max(0, x - radius);
        int xMax = min(labeledImg.cols, x + radius);

        Rect region(xMin, yMin, xMax - xMin, yMax - yMin);
        Mat roi = labeledImg(region);

        // Find unique non-zero labels in this region
        for (int row = 0; row < roi.rows; row++) {
            for (int col = 0; col < roi.cols; col++) {
                uchar pixel = roi.at<uchar>(row, col);
                if (pixel > 0) {
                    labels.insert(static_cast<int>(pixel));
                }
            }
        }
    }

    return labels;
}

// Connect two labels by merging them
void connectLabels(Mat &labeledImg, int label1, int label2) {
    if (label1 == label2) return;

    int higher = max(label1, label2);
    int lower = min(label1, label2);

    // Replace all pixels with higher label with lower label
    for (int row = 0; row < labeledImg.rows; row++) {
        for (int col = 0; col < labeledImg.cols; col++) {
            if (labeledImg.at<uchar>(row, col) == higher) {
                labeledImg.at<uchar>(row, col) = lower;
            }
        }
    }
}

// Main function to connect labels across poles
Mat connectAcrossPoles(const Mat &labeledSkeleton) {
    Mat result = labeledSkeleton.clone();

    // Pole boundary lines: (start, end)
    std::vector<std::pair<Point, Point> > poleLines = {
        {Point(103, 161), Point(221, 211)}, // Top left pole - left side
        {Point(119, 132), Point(229, 194)}, // Top left pole - right side
        {Point(317, 436), Point(306, 318)}, // Bottom pole - left side
        {Point(281, 437), Point(286, 315)} // Bottom pole - right side
    };

    // Process each pole (pairs of lines)
    for (size_t i = 0; i < poleLines.size(); i += 2) {
        Point leftStart = poleLines[i].first;
        Point leftEnd = poleLines[i].second;
        Point rightStart = poleLines[i + 1].first;
        Point rightEnd = poleLines[i + 1].second;

        // Get labels near left and right lines
        std::set<int> leftLabels = getNearbyLabels(result, leftStart, leftEnd);
        std::set<int> rightLabels = getNearbyLabels(result, rightStart, rightEnd);

        // Connect closest pairs
        // TODO: improve to only connect given some criteria, as of now we connect everything
        for (int leftLabel: leftLabels) {
            for (int rightLabel: rightLabels) {
                if (leftLabel != rightLabel) {
                    connectLabels(result, leftLabel, rightLabel);
                }
            }
        }
    }

    return result;
}


// convenience function, maybe I dont even need to use this
cv::Mat relabelConsecutive(const cv::Mat &img) {
    cv::Mat result = img.clone();

    // Find unique labels
    std::set<uchar> uniqueLabels;
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            uniqueLabels.insert(img.at<uchar>(i, j));
        }
    }

    // Create mapping from old labels to new consecutive labels
    std::unordered_map<uchar, uchar> labelMap;
    uchar newLabel = 0;
    for (uchar oldLabel: uniqueLabels) {
        labelMap[oldLabel] = newLabel++;
    }

    // Apply relabeling
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            result.at<uchar>(i, j) = labelMap[img.at<uchar>(i, j)];
        }
    }

    return result;
}

// Find coordinates of max value, min saturation point for each label
std::vector<Point> findMaxCoordinates(const Mat &labeledImg, const Mat &hsvImage) {
    std::vector<Point> coordinates;

    // Find maximum label value
    double minVal, maxVal;
    minMaxLoc(labeledImg, &minVal, &maxVal);
    int maxLabel = static_cast<int>(maxVal);

    // Split HSV channels
    std::vector<Mat> hsvChannels;
    split(hsvImage, hsvChannels);
    Mat h = hsvChannels[0];
    Mat s = hsvChannels[1];
    Mat v = hsvChannels[2];

    for (int lbl = 1; lbl <= maxLabel; lbl++) {
        // skip 0 label(background)
        // Create mask for current label
        Mat trailMask = (labeledImg == lbl);

        // Check if mask has any pixels
        if (countNonZero(trailMask) == 0) {
            coordinates.push_back(Point(-1, -1)); // Invalid point
            continue;
        }

        // Find maximum V value in masked region
        double maxValue = -std::numeric_limits<double>::infinity();
        for (int y = 0; y < v.rows; y++) {
            for (int x = 0; x < v.cols; x++) {
                if (trailMask.at<uchar>(y, x) > 0) {
                    double val = v.at<uchar>(y, x);
                    if (val > maxValue) {
                        maxValue = val;
                    }
                }
            }
        }

        // Find minimum S value among pixels with maximum V value
        double minSaturation = std::numeric_limits<double>::infinity();
        Point bestPoint(-1, -1);

        for (int y = 0; y < v.rows; y++) {
            for (int x = 0; x < v.cols; x++) {
                if (trailMask.at<uchar>(y, x) > 0 &&
                    v.at<uchar>(y, x) == maxValue) {
                    double sat = s.at<uchar>(y, x);
                    if (sat < minSaturation) {
                        minSaturation = sat;
                        bestPoint = Point(x, y);
                    }
                }
            }
        }

        coordinates.push_back(bestPoint);
    }

    return coordinates;
}

# define DEBUG true

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

    ESP_LOGI(TAG, "Starting img proc");

    std::string test_img = "/spiffs/captured_image_1748952173.png";

    Mat img = imread(test_img, IMREAD_COLOR);
    Mat hsvImg;

    cvtColor(img, img, COLOR_BGR2RGB);


    size_t len = img.total() * img.channels();
    // usb_send_data(img.data, len, img.cols, img.rows);

    Mat dst;
    bilateralFilter(img, dst, 5, 50, 50);
    img = std::move(dst);
    // usb_send_data(img.data, len, img.cols, img.rows);
    ESP_LOGI(TAG, "After filter");

    cvtColor(img, img, COLOR_RGB2HSV);
    hsvImg = img.clone();

    // const uint8_t lower[3] = {55,0,45};
    // const uint8_t upper[3] = {90, 255,255};

    // this instead of uint arr works
    Scalar lower(55, 0, 45);
    Scalar upper(90, 255, 255);

    inRange(img, lower, upper, dst);
    img = std::move(dst);

    ESP_LOGI(TAG, "After range");

    //TODO: thinning might be too expensive, may not be feasible

    // I cant include ximgproc, therefore ZhanSuen has to be implemented from scratch
    // ximgproc::thinning(img, dst, ximgproc::THINNING_ZHANGSUEN);


    thin(img);
    ESP_LOGI(TAG, "After thin");

    Mat stats, centroids;
    int nLabels = connectedComponentsWithStats(img, dst, stats, centroids);
    img = std::move(dst);

    dst = img.clone();
    for (int label = 1; label < nLabels; ++label) {
        // label 0 is background
        int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < 40) {
            // TODO: with the new (less noisy) cam this might not even be necessary
            // Set all pixels of this label to background (0)
            dst.setTo(0, img == label);
        }
    }

    img = std::move(dst);

    // only for plotting
    if constexpr (DEBUG) {
        img.convertTo(dst, CV_8U, 255.0 / (nLabels - 1));
        img = std::move(dst);
    }

    ESP_LOGI(TAG, "After labeling");


    double minVal, maxVal;
    cv::Point minLoc, maxLoc;

    cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);

    dst = connectAcrossPoles(img);
    img = std::move(dst);

    dst = relabelConsecutive(img);
    img = std::move(dst);

    ESP_LOGI(TAG, "After Connect Poles & relabel");

    // FIXME: these two are actually only for plotting
    if (DEBUG) {
        cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
        img.convertTo(dst, CV_8U, 255.0 / maxVal); // TODO: if no label div by 0???
        img = std::move(dst);
    }


    std::vector<Point> maxCoords = findMaxCoordinates(img, hsvImg);
    if constexpr (DEBUG) {
        Mat rgbImg;
        cvtColor(img, rgbImg, COLOR_GRAY2BGR);

        // Set max points to red
        for (const Point &coord: maxCoords) {
            if (coord.x >= 0 && coord.y >= 0) {
                // Draw 5x5 red square centered on point
                for (int dy = -2; dy <= 2; dy++) {
                    for (int dx = -2; dx <= 2; dx++) {
                        int x = coord.x + dx;
                        int y = coord.y + dy;
                        if (x >= 0 && x < rgbImg.cols && y >= 0 && y < rgbImg.rows) {
                            rgbImg.at<Vec3b>(y, x) = Vec3b(0, 0, 255); // BGR: Red
                        }
                    }
                }
            }
        }
        img = std::move(rgbImg);
    }
    ESP_LOGI(TAG, "After find Max Coords");
    // usb_send_data(dst.data, dst.total() * dst.channels(), dst.cols, dst.rows);


    // while (true) {
    //          UBaseType_t high_water1 = uxTaskGetStackHighWaterMark(NULL);
    //          ESP_LOGI(TAG, "Stack free: %lu", high_water1);
    //          ESP_LOGI(TAG, "Taking picture...");

    // ImageData img = MurmechaCam::get_rgb_image();
    // usb_send_data(img.buf, img.len, img.width, img.height);


    //          UBaseType_t high_water2 = uxTaskGetStackHighWaterMark(NULL);
    //          ESP_LOGI(TAG, "Stack free: %lu", high_water2); // 18648 without debugger

    //     vTaskDelay(pdMS_TO_TICKS(200));
    // }

    // esp_vfs_spiffs_conf_t conf = {
    //     .base_path = "/spiffs",
    //     .partition_label = NULL,
    //     .max_files = 5,
    //     .format_if_mount_failed = true};
    // ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));


    // xTaskCreatePinnedToCore(
    //      receiveImageTask, "ReceiveImage", 8192, (void *)copy, 1, NULL, 1);
}
