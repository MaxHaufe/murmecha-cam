//
// Created by max on 22/08/25.
//

#ifndef CURVEDIR_HPP
#define CURVEDIR_HPP

#include <vector>
#include <cmath>
#include <stdexcept>

#include <opencv2/opencv.hpp>
using namespace cv;


class CurveDirectionCalculator {
public:
    // Calculate derivative at the last point using polynomial fitting
    static double calculateDirection(const std::vector<Point> &points, int degree = 1) {
        // if (points.size() < degree + 1) {
        //     throw std::invalid_argument("Not enough points for polynomial degree");
        // }
        if (points.size() < 2) {
            throw std::invalid_argument("Need at least 2 points");
        }
        // int n = points.size();

        // For linear case (most common), use optimized least squares
        // if (degree == 1) {
        return calculateLinearSlope(points);
        // }

        // General polynomial fitting using normal equations
        // return calculatePolynomialDerivative(points, degree);
    }

private:
    static double calculateLinearSlope(const std::vector<Point> &points) {
        int n = points.size();
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;

        for (const auto &p: points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_xy += p.x * p.y;
            sum_x2 += p.x * p.x;
        }

        double denom = n * sum_x2 - sum_x * sum_x;
        if (std::abs(denom) < 1e-12) {
            throw std::runtime_error("Singular matrix - points may be vertically aligned");
        }

        return (n * sum_xy - sum_x * sum_y) / denom;
    }
};


// // Segment fitting evaluation using direction as goodness-of-fit measure
// class SegmentFitEvaluator {
// public:
//     static double evaluateSegmentsFit(const std::vector<Point> &previousPoints,
//                                       const std::vector<std::vector<Point> > &trailSegments,
//                                       int degree = 1) {
//         if (previousPoints.empty() || trailSegments.empty()) {
//             return 0.0;
//         }
//
//         // Get reference direction from previous points
//         const double referenceDirection = CurveDirectionCalculator::calculateDirection(previousPoints, degree);
//
//         double totalScore = 0.0;
//         int validSegments = 0;
//
//         for (const auto &segment: trailSegments) {
//             if (segment.size() >= degree + 1) {
//                 const double segmentDirection = CurveDirectionCalculator::calculateDirection(segment, degree);
//
//                 // Score based on direction similarity (higher = better fit)
//                 double directionDiff = std::abs(segmentDirection - referenceDirection);
//                 // double score = std::exp(-directionDiff); // Exponential decay for differences
//
//                 totalScore += directionDiff;
//                 validSegments++;
//             }
//         }
//
//         // Normalize by number of valid segments
//         return validSegments > 0 ? totalScore / validSegments : 0.0;
//     }
// };
//

#endif //CURVEDIR_HPP
