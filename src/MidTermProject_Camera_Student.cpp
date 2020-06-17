/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <vector>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";  // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0;  // first file index to load (assumes Lidar and camera names
                            // have identical naming convention)
    int imgEndIndex = 9;    // last file index to load
    int imgFillWidth =
        4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize =
        2;  // no. of images which are held in memory (ring buffer) at the same time
    deque<DataFrame> dataBuffer;  // list of data frames which are held in memory at the
                                  // same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;

        if (dataBuffer.size() >= dataBufferSize)
        {
            dataBuffer.pop_front();
        }
        
        dataBuffer.push_back(frame);

        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints;  // create empty feature list for current image

        /*
         * Enable string-based selection based on detectorType
         * -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
         */

        // Default to SIFT detector
        string detectorType = "SIFT";
        bool visDetector = true;

        DetectorTypeIndex detectorTypeIndex = getDetectorTypeIndex(detectorType);

        switch (detectorTypeIndex)
        {
            case DetectorTypeIndex::SHITOMASI:
            {
                detKeypointsShiTomasi(keypoints, imgGray, visDetector);
                break;
            }
            case DetectorTypeIndex::HARRIS:
            {
                detKeypointsHarris(keypoints, imgGray, visDetector);
                break;
            }
            case DetectorTypeIndex::FAST:
            case DetectorTypeIndex::BRISK:
            case DetectorTypeIndex::ORB:
            case DetectorTypeIndex::AKAZE:
            case DetectorTypeIndex::SIFT:
            {
                detKeypointsModern(keypoints, imgGray, detectorTypeIndex, visDetector);
                break;
            }
            default:
            {
                throw invalid_argument("Invalid detector type");
            }
        }

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        vector<cv::KeyPoint> keypointsROI;

        if (bFocusOnVehicle)
        {
            /*
             * Removal logic with vector is not optimized, since this is temporary and
             * gets removed in final project.
             */
            removeKeypointsOutsideBox(vehicleRect, keypoints, keypointsROI);
            keypoints = keypointsROI;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.back()).keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        /*
         * Enable string-based selection based on descriptorType
         *  -> BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
         */
        cv::Mat descriptors;
        string descriptorType = "BRISK";
        DescriptorTypeIndex descriptorTypeIndex = getDescriptorTypeIndex(descriptorType);
        descKeypoints((dataBuffer.back()).keypoints, (dataBuffer.back()).cameraImg,
                      descriptors, descriptorTypeIndex);

        // push descriptors for current frame to end of data buffer
        (dataBuffer.back()).descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1)  // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_FLANN";         // MAT_BF, MAT_FLANN
            string descriptorType = "DES_BINARY";  // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";        // SEL_NN, SEL_KNN

            matchDescriptors(dataBuffer[dataBuffer.size() - 2].keypoints,
                             dataBuffer[dataBuffer.size() - 1].keypoints,
                             dataBuffer[dataBuffer.size() - 2].descriptors,
                             dataBuffer[dataBuffer.size() - 1].descriptors, matches,
                             descriptorType, matcherType, selectorType);

            // store matches in current data frame
            (dataBuffer.back()).kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = (dataBuffer[dataBuffer.size() - 1].cameraImg).clone();
                cv::drawMatches(dataBuffer[dataBuffer.size() - 2].cameraImg,
                                dataBuffer[dataBuffer.size() - 2].keypoints,
                                dataBuffer[dataBuffer.size() - 1].cameraImg,
                                dataBuffer[dataBuffer.size() - 1].keypoints, matches,
                                matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(),
                                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 8);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0);  // wait for key to be pressed
            }
            bVis = false;
        }

    }  // eof loop over all images

    return 0;
}
