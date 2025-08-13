#include "image_processing.hpp"
#include "zhangsuen.hpp"

#include <set>
#include <chrono>
#include <iostream>
#include <string>

#undef EPS
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#define EPS 192
using namespace cv;




class Timer {
    std::chrono::high_resolution_clock::time_point start_time;
    std::string name;

public:
    Timer(const std::string& timer_name) : name(timer_name) {
        start_time = std::chrono::high_resolution_clock::now();
    }

    ~Timer() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        std::cout << name << ": " << duration.count() / 1000 << " ms\n";
    }
};



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
//TODO: if I end up using this it might make more sense to modify in-place for memory
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

void processImage(Mat &img) {
    /*
     * Modifies in-place
     * TODO: remove clone overhead
     * TODO: add timing
     * TODO: i don't know if it is feasible to keep 3 images in memory, img, dst, and hsvimage
     */

    Timer full("full");

    Mat hsvImg;
    Mat dst;
    cvtColor(img, img, COLOR_BGR2RGB);


    // size_t len = img.total() * img.channels();
    // usb_send_data(img.data, len, img.cols, img.rows);



    {
        Timer t("filter");
        bilateralFilter(img, dst, 5, 50, 50);
    }
    img = std::move(dst);
    // usb_send_data(img.data, len, img.cols, img.rows);

    cvtColor(img, img, COLOR_RGB2HSV);
    hsvImg = img.clone();

    // const uint8_t lower[3] = {55,0,45};
    // const uint8_t upper[3] = {90, 255,255};

    // this instead of uint arr works
    Scalar lower(55, 0, 45);
    Scalar upper(90, 255, 255);

    inRange(img, lower, upper, dst);
    img = std::move(dst);

    //TODO: thinning might be too expensive, may not be feasible

    // I cant include ximgproc, therefore ZhanSuen has to be implemented from scratch
    // ximgproc::thinning(img, dst, ximgproc::THINNING_ZHANGSUEN);



    if (DEBUG) {
            // TODO: this is only for debugging
        Mat img_smooth = img.clone();
        Mat  img_all = img.clone();
        Mat  img_destair = img.clone();
        Mat img_acute = img.clone();
        thin(img_smooth, true);
        thin(img_acute, false, true);
        thin(img_destair, false, false, true);
        thin(img_all, true, true, true);

    }

    {
        Timer t("thin");
        thin(img);
    }


    // Mat stats, centroids;
    // int nLabels = connectedComponentsWithStats(img, dst, stats, centroids);
    // img = std::move(dst);
    //
    // dst = img.clone();
    // for (int label = 1; label < nLabels; ++label) {
    //     // label 0 is background
    //     int area = stats.at<int>(label, cv::CC_STAT_AREA);
    //     if (area < 40) {
    //         // TODO: with the new (less noisy) cam this might not even be necessary
    //         // Set all pixels of this label to background (0)
    //         dst.setTo(0, img == label);
    //     }
    // }
    //
    // img = std::move(dst);

    // Use basic CCL (much faster)
    int nLabels;
    {
        Timer t("ccl");
        nLabels= connectedComponents(img, dst);
    }
    img = std::move(dst);

    // Calculate areas manually (still faster than connectedComponentsWithStats)
    std::vector<int> areas(nLabels, 0);
    {
        Timer t("size calc");
        for (int y = 0; y < img.rows; y++) {
            for (int x = 0; x < img.cols; x++) {
                int label = img.at<int>(y, x);
                areas[label]++;
            }
        }
    }

    // Filter by area
    dst = img.clone();
    for (int label = 1; label < nLabels; ++label) {
        if (areas[label] < 40) {
            dst.setTo(0, img == label);
        }
    }
    img = std::move(dst);

    // only for plotting
    if constexpr (DEBUG) {
        img.convertTo(dst, CV_8U, 255.0 / (nLabels - 1));
        img = std::move(dst);
    }


    double minVal, maxVal;
    Point minLoc, maxLoc;

    minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);

    {
        Timer t("connect Poles");
        dst = connectAcrossPoles(img);
    }
    img = std::move(dst);

    dst = relabelConsecutive(img);
    img = std::move(dst);


    // TODO: these two are actually only for plotting
    if (DEBUG) {
        cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
        img.convertTo(dst, CV_8U, 255.0 / maxVal); // TODO: if no label div by 0???
        img = std::move(dst);
    }


    std::vector<Point> maxCoords;
    {
        Timer t("findMaxCoords");
        maxCoords= findMaxCoordinates(img, hsvImg);
    }
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

}
