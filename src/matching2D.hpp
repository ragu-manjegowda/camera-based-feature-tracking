#ifndef matching2D_hpp
#define matching2D_hpp

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdio.h>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dataStructures.h"

enum DetectorTypeIndex
{
    FAST = 0,
    BRISK = 1,
    ORB = 2,
    AKAZE = 3,
    SIFT = 4,
    SHITOMASI = 5,
    HARRIS = 6
};

static const std::vector<std::string> detectorTypeString{
    "FAST", "BRISK", "ORB", "AKAZE", "SIFT", "SHITOMASI", "HARRIS"};

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints,
                        cv::Mat &img,
                        bool bVis = false);

void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints,
                           cv::Mat &img,
                           bool bVis = false);

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints,
                        cv::Mat &img,
                        DetectorTypeIndex detectorTypeIndex,
                        bool bVis = false);

void descKeypoints(std::vector<cv::KeyPoint> &keypoints,
                   cv::Mat &img,
                   cv::Mat &descriptors,
                   std::string descriptorType);

void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource,
                      cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches,
                      std::string descriptorType,
                      std::string matcherType,
                      std::string selectorType);

DetectorTypeIndex getDetectorTypeIndex(std::string &detectorType);
const std::string &getDetectorTypeString(DetectorTypeIndex detectorTypeIndex);

#endif /* matching2D_hpp */
