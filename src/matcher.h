/**
 * @file matcher.h
 * @author Yan Tang (360383464@qq.com)
 * @brief 图像匹配的相关函数定义
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef MATCHER_H_
#define MATCHER_H_

#include <random>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace photogrammetry
{

    struct Match
    {
        cv::Point2d srcPt; // 左片像点坐标
        cv::Point2d dstPt; // 右片像点坐标
        double dist;       // 相似性测度的计算值
    };

    /**
     * @brief 影像匹配基类
     * 
     */
    class BaseMatcher
    {
    public:
        /**
         * @brief 设置匹配窗口大小
         * 
         * @param windowSize 窗口大小，如：7、9、11，默认为15
         */

        void setWindowSize(int windowSize);

        /**
         * @brief 设置阈值
         * 
         * @param threshold 相关系数阈值，默认为0.8
         */
        void setThreshold(double threshold);

        /**
         * @brief 
         * 
         * @param srcImg 参考图像
         * @param dstImg 目标图像
         * @param outputImg 输出匹配结果的图像
         * @param matches 匹配上的特征点
         */
        void drawMatches(const cv::Mat &srcImg,
                         const cv::Mat &dstImg,
                         cv::Mat &outputImg,
                         std::vector<Match> &matches);

    protected:
        int windowSize = 15;    // 窗口大小
        double threshold = 0.8; // 阈值

        /**
         * @brief 计算两个窗口的相关系数
         * 
         * @param srcWindow 
         * @param dstWindow 
         * @return double 
         */
        double computeCorrelationIdx(const cv::Mat &srcWindow, const cv::Mat &dstWindow);

        /**
         * @brief 判断当前点是否在边界上
         * 
         * @param srcImg 
         * @param pt 
         * @return true 
         * @return false 
         */
        bool isVaildPoint(const cv::Mat &srcImg, const cv::Point &pt);
    };

    /**
     * @brief 相关系数法匹配
     * 
     */
    class CorrelationMatcher : public BaseMatcher
    {
    public:
        /**
         * @brief 扫描目标图像全图进行同名点匹配
         * 
         * @param srcImg 参考图像
         * @param dstImg 目标图像
         * @param srcPts 参考图像上的特征点
         * @param matches 输出匹配上的特征点
         */
        void match(const cv::Mat &srcImg,
                   const cv::Mat &dstImg,
                   const std::vector<cv::Point> &srcPts,
                   std::vector<Match> &matches);

        /**
         * @brief 在参考图像和目标图像上同时提取特征点进行匹配
         * 
         * @param srcImg 参考图像
         * @param dstImg 目标图像
         * @param srcPts 参考图像上的特征点
         * @param dstPts 目标图像上的特征点
         * @param matches 输出匹配上的特征点
         */
        void matchImproved(const cv::Mat &srcImg,
                           const cv::Mat &dstImg,
                           const std::vector<cv::Point> &srcPts,
                           const std::vector<cv::Point> &dstPts,
                           std::vector<Match> &matches);
    };

    /**
     * @brief 单点最小二乘匹配
     * 
     */
    class LsqMatcher : public BaseMatcher
    {
    public:
        /**
         * @brief 单点最小二乘匹配
         * 
         * @param srcImg 参考图像
         * @param dstImg 目标图像
         * @param match 输入整像素匹配初值，输出亚像素匹配结果
         * @return true 
         * @return false 
         */
        bool subPixelMatch(const cv::Mat &srcImg, const cv::Mat &dstImg, Match &match);

    private:
        double a0 = 0, a1 = 1, a2 = 0;
        double b0 = 0, b1 = 0, b2 = 0;
        double h0 = 0, h1 = 1;
    };
}

#endif // MATCHER_H_