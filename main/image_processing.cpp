#include "image_processing.hpp"
#include "zhangsuen.hpp"

#include <set>
#include <chrono>
#include <iostream>
#include <string>
// #include "SkeletonGraph.hpp"
#include "Graph.hpp"
#undef EPS
#include <map>
#include <numeric>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#define EPS 192
using namespace cv;


class Timer {
    std::chrono::high_resolution_clock::time_point start_time;
    std::string name;

public:
    Timer(const std::string &timer_name) : name(timer_name) {
        start_time = std::chrono::high_resolution_clock::now();
    }

    ~Timer() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        std::cout << name << ": " << duration.count() / 1000 << " ms\n";
    }
};

//Data Structure for Murmecha cam init, holds properties for pole locations, center point, cutoff, and inner and outer radius
// TODO: pass image size as well for error checks?!?!?!
class MurmechaEyePreprocessor {
public:
    // Enum for pole line access
    enum PoleLineIndex {
        POLE0_LEFT,
        POLE0_RIGHT,
        POLE1_LEFT,
        POLE1_RIGHT,
        POLE2_LEFT,
        POLE2_RIGHT
    };

    enum PoleLineSide {
        LEFT_SIDE,
        RIGHT_SIDE
    };

private:
    // Pole boundary lines: (start, end)
    std::vector<std::pair<Point, Point> > poleLines;
    std::vector<std::pair<Point, Point> > poleLinesProcessed;
    uint16_t cutoffLeftX;
    uint16_t cutoffRightX;


    Point centerPoint;
    Point centerPointProcessed;
    uint16_t maskingRadiusInner;
    uint16_t maskingRadiusOuter;

public:
    MurmechaEyePreprocessor(const Point center, const uint16_t outerRadius) : centerPoint(center),
                                                                              maskingRadiusInner(0),
                                                                              maskingRadiusOuter(outerRadius) {
        poleLines.resize(6);
        poleLinesProcessed.resize(6);


        cutoffLeftX = centerPoint.x - maskingRadiusOuter;
        cutoffRightX = centerPoint.x + maskingRadiusOuter;
        centerPointProcessed = Point(centerPoint.x - cutoffLeftX, centerPoint.y);
    }


    std::vector<std::pair<Point, Point> > getPoleLinesProcessed() const {
        return poleLinesProcessed;
    }

    // std::pair<std::pair<Point, Point>, std::pair<Point, Point>> getPoleLineByNumber(int i) {
    //     // returns left, right
    //     return std::make_pair(poleLinesProcessed[2 * i], poleLinesProcessed[2 * i + 1]);
    // }


    void setPoleLineByIndex(PoleLineIndex index, const Point &start, const Point &end) {
        poleLines[static_cast<size_t>(index)] = std::make_pair(start, end);

        // since we crop away the left (and right) part of the image, we need to adjust the x-coords
        Point newStart(start.x - cutoffLeftX, start.y);
        Point newEnd(end.x - cutoffLeftX, end.y);

        poleLinesProcessed[static_cast<size_t>(index)] = std::make_pair(newStart, newEnd);
    }

    // Center point accessors
    const Point &getCenterPoint() const { return centerPointProcessed; }
    void setCenterPoint(const Point &center) { centerPoint = center; }

    // Masking radius accessors
    uint16_t getMaskingRadiusInner() const { return maskingRadiusInner; }
    void setMaskingRadiusInner(uint16_t radius) { maskingRadiusInner = radius; }

    uint16_t getMaskingRadiusOuter() const { return maskingRadiusOuter; }
    void setMaskingRadiusOuter(uint16_t radius) { maskingRadiusOuter = radius; }

    // Mat ROI(Mat &img) {
    //     //crop image left and right
    //
    //     // color certain regions of the image black:
    //     // mask everything around centerPointProcessed outside of maskingRadiusOuter
    //     // mask everything around centerPointProcessed inside of maskingRadiusInner
    //     // mask all processed Poles
    //     return img;
    // }

    Mat ROI(Mat &img) {
        //crop image left and right

        // color certain regions of the image black:

        const int cropWidth = cutoffRightX - cutoffLeftX;
        const Rect cropRect(cutoffLeftX, 0, cropWidth, img.rows);
        Mat result = img(cropRect).clone(); // Need clone for resize
        Mat mask = Mat::zeros(result.size(), CV_8UC1);

        // mask everything around centerPointProcessed outside of maskingRadiusOuter
        if (maskingRadiusOuter > 0) {
            mask.setTo(0);
            circle(mask, centerPointProcessed, maskingRadiusOuter, Scalar(255), -1);
            bitwise_not(mask, mask);
            result.setTo(Scalar(0, 0, 0), mask);
        }

        // mask everything around centerPointProcessed inside of maskingRadiusInner
        if (maskingRadiusInner > 0) {
            mask.setTo(0);
            circle(mask, centerPointProcessed, maskingRadiusInner, Scalar(255), -1);
            result.setTo(Scalar(0, 0, 0), mask);
        }

        // mask all processed Poles
        for (int i = 0; i < 3; i++) {
            const auto &leftLine = poleLinesProcessed[i * 2]; // POLE_LEFT
            const auto &rightLine = poleLinesProcessed[i * 2 + 1]; // POLE_RIGHT

            // Create quadrilateral from 4 points
            std::vector<Point> quad = {
                leftLine.first, leftLine.second,
                rightLine.second, rightLine.first
            };

            fillPoly(result, quad, Scalar(0, 0, 0));
        }

        return result;
    }
};


// Connect two labels by merging them
void connectLabels(Mat &labeledImg, const int label1, const int label2, const Point p1, const Point p2) {
    if (label1 == label2) return;


    // Replace all pixels with higher label with lower label
    for (int row = 0; row < labeledImg.rows; row++) {
        for (int col = 0; col < labeledImg.cols; col++) {
            if (labeledImg.at<uchar>(row, col) == label2) {
                labeledImg.at<uchar>(row, col) = label1;
            }
        }
    }

    //interpolate all pixels between p1 and p2, set it to label 1

    line(labeledImg, p1, p2, label1, 1);
}

//TODO: when optimizing the different resolutions, the dist has to change as well
// TODO; make this unordered map
std::vector<std::pair<int, Point> > getAdjacentLabels(const Mat &labeledSkeleton,
                                                      const std::pair<Point, Point> &line,
                                                      const MurmechaEyePreprocessor::PoleLineSide side,
                                                      const uint8_t dist = 50) {
    // IMPORTANT: this only works because when we set the poleLines we set the points as (bottom, top)
    const auto bottom = line.first;
    const auto top = line.second;

    const Point lineVec = top - bottom;
    Point dirVec; // is orthogonal to lineVec
    // I feel like this should be the other way around but OpenCVs coordinate system is flipped
    if (side == MurmechaEyePreprocessor::RIGHT_SIDE) {
        dirVec = Point(-lineVec.y, lineVec.x);
    } else {
        // left
        dirVec = Point(lineVec.y, -lineVec.x);
    }
    Vec2f normDirVec;
    normalize(Vec2f(dirVec.x, dirVec.y), normDirVec);


    // now, create a rectangle, find the other corners of the rect in which we want to search adjacent pts
    const auto scaledVec = static_cast<Point>(dist * normDirVec);
    const auto newBottom = bottom + scaledVec;
    const auto newTop = top + scaledVec;

    const std::vector rectPts = {top, bottom, newBottom, newTop};

    // create mask for extraction
    Mat mask = Mat::zeros(labeledSkeleton.size(), CV_8UC1);
    fillPoly(mask, rectPts, Scalar(255));
    const auto bounding = boundingRect(rectPts);

    // limit iters with rect
    std::vector<std::pair<int, Point> > ret;
    for (int y = bounding.y; y < bounding.y + bounding.height; y++) {
        for (int x = bounding.x; x < bounding.x + bounding.width; x++) {
            if (mask.at<uint8_t>(y, x) == 0) {
                // x,y truly inside the polygon of our rectPts?
                continue;
            }
            int n = 0;
            Point p(x, y);
            const auto lbl = labeledSkeleton.at<uint8_t>(p);
            for (const auto &offset: offsets8) {
                const Point nbr = p + offset;
                if (labeledSkeleton.at<uint8_t>(nbr) == lbl) {
                    n++;
                }
            }
            if (n == 1) {
                //endpoint
                ret.push_back(std::pair(lbl, p));
            }
        }
    }
    return ret;
}

// TODO: test for uneven labels > left 3 & right 4
void connectAcrossPolesImproved(Mat &img, const std::vector<std::pair<Point, Point> > &poleLines) {
    //3 poles, for every one of them,...
    for (int i = 0; i < 3; i++) {
        //get labels adjacent
        // IMPORTANT: this only works because when we set the poleLines we set the points as (bottom, top)
        auto leftPole = poleLines[2 * i];
        auto rightPole = poleLines[2 * i + 1];
        //get labels adjacent
        auto left = getAdjacentLabels(img, leftPole, MurmechaEyePreprocessor::PoleLineSide::LEFT_SIDE);
        auto right = getAdjacentLabels(img, rightPole,
                                       MurmechaEyePreprocessor::PoleLineSide::RIGHT_SIDE);

        // if left and right have equally many labels -> good, match all
        // if not, some will be left to be not matched.
        // TODO: here we are trying to find optimal assignments between left and right.
        //  this is what the Hungarian Algorithm was made for.
        //  https://de.wikipedia.org/wiki/Ungarische_Methode
        //  Since the maps rarely ever have more than 5 entries, I could not care less about efficiency
        //  brute force go brr
        int nLeft = left.size();
        int nRight = right.size();
        const int nPairs = std::min(nLeft, nRight);

        if (nPairs == 0) {
            continue;
        }


        std::vector leftVec(left.begin(), left.end());
        std::vector rightVec(right.begin(), right.end());

        double minTotalDistance = std::numeric_limits<double>::max();
        std::vector<std::tuple<int, Point, int, Point> > bestAssignment;

        // all combinations
        std::vector leftCombination(nLeft, 0);
        std::fill(leftCombination.end() - nPairs, leftCombination.end(), 1);

        do {
            std::vector rightCombination(nRight, 0);
            std::fill(rightCombination.end() - nPairs, rightCombination.end(), 1);

            do {
                std::vector<int> selectedLeft, selectedRight;
                for (int j = 0; j < nLeft; j++) {
                    if (leftCombination[j]) selectedLeft.push_back(j);
                }
                for (int j = 0; j < nRight; j++) {
                    if (rightCombination[j]) selectedRight.push_back(j);
                }

                // try all right
                std::ranges::sort(selectedRight);
                do {
                    double totalDistance = 0.0;
                    std::vector<std::tuple<int, Point, int, Point> > currentAssignment;

                    for (int j = 0; j < nPairs; j++) {
                        const double dist = norm(leftVec[selectedLeft[j]].second - rightVec[selectedRight[j]].second);
                        totalDistance += dist;
                        currentAssignment.push_back({
                            leftVec[selectedLeft[j]].first, leftVec[selectedLeft[j]].second,
                            rightVec[selectedRight[j]].first, rightVec[selectedRight[j]].second
                        });
                    }

                    if (totalDistance < minTotalDistance) {
                        minTotalDistance = totalDistance;
                        bestAssignment = currentAssignment;
                    }
                } while (std::ranges::next_permutation(selectedRight).found);
            } while (std::ranges::next_permutation(rightCombination).found);
        } while (std::ranges::next_permutation(leftCombination).found);


        for (const auto &[lbl1, p1, lbl2, p2]: bestAssignment) {
            connectLabels(img, lbl1, lbl2, p1, p2);
        }
    }
}


// convenience function, maybe I dont even need to use this
//TODO: if I end up using this it might make more sense to modify in-place for memory
// TODO: I could delete this,...
Mat relabelConsecutive(const Mat &img) {
    Mat result = img.clone();

    // Find unique labels
    std::set<uchar> uniqueLabels;
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            uniqueLabels.insert(img.at<uchar>(i, j));
        }
    }

    std::unordered_map<uchar, uchar> labelMap;
    uchar newLabel = 0;
    for (uchar oldLabel: uniqueLabels) {
        labelMap[oldLabel] = newLabel++;
    }

    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            result.at<uchar>(i, j) = labelMap[img.at<uchar>(i, j)];
        }
    }

    return result;
}

// Find coordinates of max value, min saturation point for each label
//TODO: this takes 16 s!!!!" FIXME
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

uint16_t distanceMap(const Point p, const Point center, const uint16_t radiusInner, const uint16_t radiusOuter) {
    /*
     * TODO: after camera calibration scarmuzza, replace this by the map we get from calibration.
     * We need this for trail pruning.
     * Idea: remove all spurs (endpoint -> intersection segments) that are shorter than x pxls
     * due to the distortion, this value should not be constant
     * Important: near the inner radius (closest to the robot) trail thickness is ~22 px.
     *  at the outside radius (furthest away from the robot) trail thickness is ~ 6 px
     * make trail thickness and spurRemovalThreshold a function of distance to the image center
     * given center point, inner and outer radius and min,max trail thickness -> we can create a map
     */

    // TODO: hyperparameter, maybe this can be optimized by GA
    constexpr uint16_t minThick = 6;
    constexpr uint16_t maxThick = 22;

    const auto dx = p.x - center.x;
    const auto dy = p.y - center.y;
    const auto dist = sqrt(dx * dx + dy * dy);

    //edge cases
    if (dist <= radiusInner) {
        return maxThick;
    }
    if (dist >= radiusOuter) {
        return minThick;
    }
    const auto ratio = (dist - radiusInner) / (radiusOuter - radiusInner);
    return static_cast<uint16_t>(maxThick - ratio * (maxThick - minThick));
}


std::unordered_map<uint8_t, Graph> pruneTrails(Mat &img, const Point center, const uint16_t radiusInner,
                                               const uint16_t radiusOuter) {
    /*
     * Consider a skeletonized, labeled image before pole connection.
     *
     * Small pixel differences may greatly impact the skeleton. Removing one pixel from the original image greatly changes
     * the look of the skeleton (J.R. Parker, Algos for Img Proc & CV)
     *
     * prune all spurs under a dynamic threshold
     * TODO: make sure that we do not accidentally prune an endpoint ?! -> it is very much possible but rare to have an intersection where
     *  there is actually none
     */
    Point minLoc, maxLoc;
    double minVal, maxVal;
    minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
    std::set<uint8_t> uniqueValues;

    MatConstIterator_ it = img.begin<uint8_t>();
    MatConstIterator_ end = img.end<uint8_t>();

    for (; it != end; ++it) {
        uniqueValues.insert(*it);
    }

    // label -> Graph
    std::unordered_map<uint8_t, Graph> graphs;

    for (const auto lbl: uniqueValues) {
        if (lbl == 0) {
            continue;
        }
        Graph g{};
        g.buildGraph(img, lbl);
        // prune all the spurs that exceed the max spur length
        std::vector<GraphNode *> nodesToPrune;
        for (const auto p: g.getEndpoints()) {
            const auto pruningDist = distanceMap(p->pos, center, radiusInner, radiusOuter);
            const auto path = g.getEndSegment(*p);
            if (path.size() > 0 && path.size() < pruningDist) {
                // g.pruneSpurByEndpoint(pIdx, img); // Important: DO NOT delete while iterating through the endpoints
                // collect nodes to remove
                nodesToPrune.insert(nodesToPrune.end(), path.begin(), path.end());

                std::cout << "Point " << p->pos << " of lbl " << static_cast<int>(lbl) << " with len " << path.size() <<
                        " marked for pruning" << std::endl;
            }
        }

        // remove after collection
        g.removeNodes(nodesToPrune, img);

        //remove 1 (shortest) endpoint if uneven number of endpoints
        if (g.getEndpoints().size() % 2 == 1) {
            int dist = std::numeric_limits<int>::max();
            std::vector<GraphNode *> shortestPath;
            for (const auto &ep: g.getEndpoints()) {
                const auto path = g.getEndSegment(*ep);
                if (path.size() < dist) {
                    dist = path.size();
                    shortestPath = path;
                }
            }
            std::cout << "Deleting one path because uneven" << std::endl;
            g.removeNodes(shortestPath, img);
        }

        // remove small cycles
        g.pruneCycles(img);

        graphs[lbl] = std::move(g);
    }

    return graphs;
}


void processImage(Mat &img) {
    /*
     * Modifies in-place
     * TODO: remove clone overhead
     * TODO: i don't know if it is feasible to keep 3 images in memory, img, dst, and hsvimage -> ok
     * TODO: maybe see where the images live (ram/psram)
     */

    Timer full("full");

    Mat hsvImg;
    Mat dst;
    cvtColor(img, img, COLOR_BGR2RGB);


    // size_t len = img.total() * img.channels();
    // usb_send_data(img.data, len, img.cols, img.rows);

    //ROI
    Point center(321, 253);
    uint16_t radius_outer = 239;
    MurmechaEyePreprocessor preproc(center, radius_outer);
    // It does not matter which pole is pole 1, pole 2, etc....
    //It DOES however matter what is left and what is right.
    // start somewhere in the image and go around radially in the image CLOCKWISE
    // IMPORTANT: It is important to set the lines (Bottom Point, Top Point)
    preproc.setPoleLineByIndex(MurmechaEyePreprocessor::POLE0_LEFT, Point(314, 179), Point(303, 13));
    preproc.setPoleLineByIndex(MurmechaEyePreprocessor::POLE0_RIGHT, Point(335, 179), Point(363, 15));
    preproc.setPoleLineByIndex(MurmechaEyePreprocessor::POLE1_LEFT, Point(390, 289), Point(538, 355));
    preproc.setPoleLineByIndex(MurmechaEyePreprocessor::POLE1_RIGHT, Point(380, 300), Point(506, 406));
    preproc.setPoleLineByIndex(MurmechaEyePreprocessor::POLE2_LEFT, Point(254, 294), Point(124, 387));
    preproc.setPoleLineByIndex(MurmechaEyePreprocessor::POLE2_RIGHT, Point(249, 280), Point(97, 333));
    preproc.setMaskingRadiusInner(75); // everything inside this radius around the center will be black
    Mat preProcImg = preproc.ROI(img);
    img = std::move(preProcImg);

    auto poleLinesProcessed = preproc.getPoleLinesProcessed();

    hsvImg = img.clone();
    cvtColor(hsvImg, hsvImg, COLOR_RGB2HSV); // before filtering for best maxPoint extraction


    //TODO: maybe std::move to std::img
    {
        Timer t("filter");
        // bilateralFilter(img, dst, 5, 50, 50);
        // img = std::move(dst);

        // medianBlur(img, dst, 5);
        // img = std::move(dst);
        // GaussianBlur(img, dst, Size(15,15), 5.0, 0, BORDER_CONSTANT);
        /*
        * TODO: this seems to work pretty well, however parameterizing this is a pain since it has direct effect on the
        *   color threshold as well
        *
         */
        GaussianBlur(img, dst, Size(5, 5), 1.5, 0, BORDER_CONSTANT);
        img = std::move(dst);
    }

    // usb_send_data(img.data, len, img.cols, img.rows);

    cvtColor(img, img, COLOR_RGB2HSV);
    // hsvImg = img.clone();

    // const uint8_t lower[3] = {55,0,45};
    // const uint8_t upper[3] = {90, 255,255};

    // this instead of uint arr works
    // Scalar lower(55, 0, 45);
    //I changed this to 40 for the "fresh" lines using the py hsv picker
    Scalar lower(30, 0, 45);
    Scalar upper(90, 255, 255); {
        Timer t("inRange");
        inRange(img, lower, upper, dst);
    }
    img = std::move(dst);

    //morph open -> TODO: depending on the future noise we might not even need this
    //morph close to close the holes in the "fresh" trial -> TODO: we might also not need this, since I think it is introduced by the camera noise

    {
        // TODO: maybe morph close after thinning too??
        Timer t("morph");
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        // Morphological opening (remove noise)
        // morphologyEx(img, img, MORPH_OPEN, kernel);

        // Morphological closing (close small holes)
        // TODO: re-enable this or make klernel bigger, also open to get rid of spurs
        morphologyEx(img, img, MORPH_CLOSE, kernel);
    }


    //TODO: thinning might be too expensive, may not be feasible

    // I cant include ximgproc, therefore ZhanSuen has to be implemented from scratch
    // ximgproc::thinning(img, dst, ximgproc::THINNING_ZHANGSUEN);


    // if (DEBUG) {
    //     // TODO: this is only for debugging
    //     Mat img_normal = img.clone();
    //     Mat img_smooth = img.clone();
    //     Mat img_all = img.clone();
    //     Mat img_destair = img.clone();
    //     Mat img_acute = img.clone();
    //     thin(img_normal);
    //     thin(img_smooth, true);
    //     thin(img_acute, false, true);
    //     thin(img_destair, false, false, true);
    //     thin(img_all, true, true, true);
    //     if (true) {
    //     }
    // }
    {
        Timer t("thin");
        thin(img, true, true, true);
    }


    // labeling
    int nLabels; {
        Timer t("ccl total"); {
            Timer t("pure ccl");
            nLabels = connectedComponents(img, dst);
            img = std::move(dst);
        }

        // Calculate areas
        std::vector<int> areas(nLabels, 0); {
            Timer t("size calc");
            for (int y = 0; y < img.rows; y++) {
                for (int x = 0; x < img.cols; x++) {
                    int label = img.at<int>(y, x);
                    areas[label]++;
                }
            }
        } {
            Timer t("filter by area");
            // TODO: why does this take 15 seconds FIXME
            // Filter by area
            dst = img.clone();
            for (int label = 1; label < nLabels; ++label) {
                if (areas[label] < 20)
                //TODO: hyperaprameter, has to be optimized and scaled down when using a lower Res
                {
                    dst.setTo(0, img == label);
                }
            }
            img = std::move(dst);
        }
    }


    // only for plotting
    if constexpr (DEBUG) {
        img.convertTo(dst, CV_8U, 255.0 / (nLabels - 1));
        img = std::move(dst);
    }


    double minVal, maxVal;
    Point minLoc, maxLoc; {
        Timer t("connect Poles");
        connectAcrossPolesImproved(img, poleLinesProcessed);


        // TODO: this can be made more efficient
        dst = relabelConsecutive(img);
        img = std::move(dst);
    }


    // TODO: these two are actually only for plotting
    if (DEBUG) {
        minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
        img.convertTo(dst, CV_8U, 255.0 / maxVal); // TODO: if no label div by 0???
        img = std::move(dst);
    }

    {
        Timer t("pruning and decomposing");
        auto graphs = pruneTrails(img, preproc.getCenterPoint(), preproc.getMaskingRadiusInner(),
                                 preproc.getMaskingRadiusOuter());

        for (auto &[_, g]: graphs) {
            if (g.getEndpoints().size() > 3) g.decomposeTrails(img, hsvImg);
        }
    }


    std::vector<Point> maxCoords; {
        Timer t("findMaxCoords");
        maxCoords = findMaxCoordinates(img, hsvImg);
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
