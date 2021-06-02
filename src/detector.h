/**
 * @file detector.h
 * @author Yan Tang (360383464@qq.com)
 * @brief 
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <vector>
#include <random>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace photogrammetry
{
    /**
     * @brief Harris算子角点检测
     * 
     * @param srcImg 检测图像
     * @param threshold Harris响应值的阈值
     * @return std::vector<cv::Point> 
     */
    std::vector<cv::Point> HarrisCornerDetect(const cv::Mat &srcImg, double threshold = 120);

    /**
     * @brief Moravec角点检测
     * 
     * @param srcImg 检测图像
     * @param windowSize 窗口大小
     * @param threshold 阈值
     * @return std::vector<cv::Point> 
     */
    std::vector<cv::Point> MoravecCornerDetect(const cv::Mat &srcImg, int windowSize = 5, int threshold = 700);

    /**
     * @brief SIFT角点检测
     * 
     * @param srcImg 检测图像
     * @return std::vector<cv::Point> 
     */
    std::vector<cv::Point> SIFTCornerDetect(const cv::Mat &srcImg);

    /**
     * @brief 在图像上绘制角点
     * 
     * @param srcImg 原图像
     * @param outputImg 绘制角点后图像
     * @param corners 角点
     */
    void DrawCorners(const cv::Mat &srcImg, cv::Mat &outputImg, std::vector<cv::Point> corners);

}

#endif // DETECTOR_H_