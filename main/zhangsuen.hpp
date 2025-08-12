#ifndef __ZHANGSUEN_H__
#define __ZHANGSUEN_H__

// https://github.com/yati-sagade/zhang-suen-thinning

#include <set>
#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <iomanip>
// #include "opencv2/highgui.hpp"
#include <opencv2/core/mat.hpp>



typedef std::pair<int, int> MyPoint;
typedef unsigned char uchar_t;

int num_one_pixel_neighbours(const cv::Mat& image, const MyPoint& point);

int num_zero_pixel_neighbours(const cv::Mat& image, const MyPoint& point);

int connectivity(const cv::Mat& image, const MyPoint& point);

int yokoi_connectivity(const cv::Mat& image, const MyPoint& point);

void delete_pixels(const cv::Mat& image, const std::set<MyPoint>& points);

void remove_staircases(cv::Mat& image);

void zhangsuen_thin(cv::Mat& img);

void thin(cv::Mat& img, bool need_boundary_smoothing=false,
          bool need_acute_angle_emphasis=false, bool destair=false);

void boundary_smooth(cv::Mat& image);

void acute_angle_emphasis(cv::Mat& image);

bool match(const cv::Mat& image, const std::vector<MyPoint>& points,
           const std::vector<uchar_t>& values);

bool match_templates(const cv::Mat& image, const MyPoint& point, int k);

#endif