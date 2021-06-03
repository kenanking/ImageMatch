/**
 * @file detector.cpp
 * @author Yan Tang (360383464@qq.com)
 * @brief 
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "detector.h"

namespace photogrammetry
{
    std::vector<cv::Point> HarrisCornerDetect(const cv::Mat &srcImg, double threshold)
    {
        std::vector<cv::Point> corners;

        cv::Mat imageGrey = srcImg.clone();
        if (imageGrey.channels() != 1)
            cv::cvtColor(imageGrey, imageGrey, cv::COLOR_BGR2GRAY);

        cv::Mat interestImg, interestImgNorm, interestImgNormUInt8;
        interestImg = cv::Mat::zeros(imageGrey.size(), CV_32FC1);

        int blockSize = 2;
        int apertureSize = 3;
        double k = 0.04;

        cv::cornerHarris(imageGrey, interestImg, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

        cv::normalize(interestImg, interestImgNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1);

        cv::convertScaleAbs(interestImgNorm, interestImgNormUInt8);

        int radius = 5;

        // 筛选响应值大于阈值的点作为角点
        for (int i = 0; i < interestImgNorm.rows; i++)
            for (int j = 0; j < interestImgNorm.cols; j++)
                if ((int)interestImgNorm.at<float>(i, j) > threshold)
                    corners.push_back(cv::Point(j, i)); // i->y, j->x

        return corners;
    }

    std::vector<cv::Point> MoravecCornerDetect(const cv::Mat &srcImg, int windowSize, int threshold)
    {
        std::vector<cv::Point> corners;

        if (windowSize % 2 == 0)
            windowSize += 1;

        cv::Mat img = srcImg.clone();
        if (img.channels() != 1)
            cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);

        int r = windowSize / 2;
        cv::GaussianBlur(img, img, cv::Size(windowSize, windowSize), 0, 0);

        cv::Mat interestImg = cv::Mat::zeros(img.size(), CV_32FC1);

        /// 计算兴趣值
        for (int i = r; i < img.rows - r; i++)
            for (int j = r; j < img.cols - r; j++)
            {
                double value[4] = {0.0};
                double minValue = 0.0;

                for (int k = -r; k <= r; k++)
                {
                    value[0] += pow(img.at<uchar>(i + k, j) - img.at<uchar>(i + k + 1, j), 2);
                    value[1] += pow(img.at<uchar>(i, j + k) - img.at<uchar>(i, j + k + 1), 2);
                    value[2] += pow(img.at<uchar>(i + k, j + k) - img.at<uchar>(i + k + 1, j + k + 1), 2);
                    value[3] += pow(img.at<uchar>(i + k, j - k) - img.at<uchar>(i + k + 1, j - k - 1), 2);
                }

                minValue = std::min(std::min(value[0], value[1]), std::min(value[2], value[3]));
                interestImg.at<float>(i, j) = minValue;
            }

        /// 选取候选点
        int maxValue;
        cv::Point point;
        for (int i = r; i < img.rows - r;)
        {
            for (int j = r; j < img.cols - r;)
            {
                point.x = -1;
                point.y = -1;
                maxValue = 0;
                for (int m = -r; m < r; m++)
                {
                    for (int n = -r; n < r; n++)
                    {
                        if (interestImg.at<float>(i + m, j + n) > maxValue)
                        {
                            maxValue = interestImg.at<float>(i + m, j + n);
                            point.y = i + m;
                            point.x = j + n;
                        }
                    }
                }
                if (maxValue > threshold)
                {
                    corners.push_back(point);
                }
                j += windowSize;
            }
            i += windowSize;
        }

        return corners;
    }

    std::vector<cv::Point> SIFTCornerDetect(const cv::Mat &srcImg)
    {
        std::vector<cv::Point> corners;

        std::vector<cv::KeyPoint> keypoints;
        cv::Ptr<cv::FeatureDetector> detector = cv::SIFT::create();
        detector->detect(srcImg, keypoints);

        for (cv::KeyPoint kpt : keypoints)
            corners.push_back(cv::Point(kpt.pt));

        return corners;
    }

    void DrawCorners(const cv::Mat &srcImg, cv::Mat &outputImg, std::vector<cv::Point> corners)
    {
        outputImg = srcImg.clone();
        static std::default_random_engine e;
        static std::uniform_int_distribution<int> u(0, 255);

        for (cv::Point corner : corners)
        {
            cv::Scalar color(u(e), u(e), u(e));

            cv::circle(outputImg, corner, 5, color, 2);
        }
    }
}