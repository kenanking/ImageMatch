/**
 * @file main.cpp
 * @author Yan Tang (360383464@qq.com)
 * @brief 影像匹配
 * @version 0.1
 * @date 2021-05-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "../src/detector.h"
#include "../src/matcher.h"
#include "../src/tic_toc.h"

int main(int argc, char **argv)
{
    // ========================== 相关参数设置 ==============================
    std::string img_1_path = "../img/LOR50.bmp";
    std::string img_2_path = "../img/LOR49.bmp";
    // std::string img_1_path = "../img/yosemite1.jpg";
    // std::string img_2_path = "../img/yosemite2.jpg";

    int corner_threshold = 700; // Moravec角点检测阈值

    std::string img_1_corner_path = "../result/LOR50_corner.jpg";
    std::string img_2_corner_path = "../result/LOR49_corner.jpg";
    // std::string img_1_corner_path = "../result/yosemite1_corner.jpg";
    // std::string img_2_corner_path = "../result/yosemite2_corner.jpg";

    int corr_window_size = 25;    // 相关系数匹配窗口大小
    double corr_threshold = 0.85; // 相关系数匹配阈值

    int lsq_window_size = 5;     // 最小二乘匹配窗口大小
    double lsq_threshold = 0.95; //最小二乘匹配窗口大小

    std::string img_match_path = "../result/LOR_corr_match_improved.jpg";
    // std::string img_match_path = "../result/LOR_corr_match.jpg";
    // std::string img_match_path = "../result/yosemite_corr_match_improved.jpg";
    // std::string img_match_path = "../result/yosemite_corr_match.jpg";

    std::string result_corr_path = "../result/LOR_corr_result_improved.txt";
    std::string result_lsq_path = "../result/LOR_lsq_result_improved.txt";
    // std::string result_corr_path = "../result/yosemite_corr_result_improved.txt";
    // std::string result_lsq_path = "../result/yosemite_lsq_result_improved.txt";
    // std::string result_corr_path = "../result/yosemite_corr_result.txt";
    // std::string result_lsq_path = "../result/yosemite_lsq_result.txt";

    // =====================================================================

    //-- 读取图像
    cv::Mat img_1 = cv::imread(img_1_path, CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(img_2_path, CV_LOAD_IMAGE_COLOR);
    if (img_1.data == nullptr || img_2.data == nullptr)
    {
        std::cout << "图像打开失败！" << std::endl;
        return 1;
    }

    // 在两张影像上分别提取角点
    std::vector<cv::Point> corners_1, corners_2;

    TicToc t = TicToc();
    // corners_1 = photogrammetry::HarrisCornerDetect(img_1, 80);
    // corners_2 = photogrammetry::HarrisCornerDetect(img_2, 80);
    corners_1 = photogrammetry::MoravecCornerDetect(img_1, 5, corner_threshold);
    corners_2 = photogrammetry::MoravecCornerDetect(img_2, 5, corner_threshold);
    // corners_1 = photogrammetry::SIFTCornerDetect(img_1);
    // corners_2 = photogrammetry::SIFTCornerDetect(img_2);
    double time = t.toc();

    std::cout << "使用Moravec算子进行角点检测，阈值设置为：" << corner_threshold << std::endl;
    std::cout << "特征点提取用时：" << time << "秒" << std::endl;
    std::cout << "图像1中特征点数量：" << corners_1.size() << std::endl;
    std::cout << "图像2中特征点数量：" << corners_2.size() << std::endl;
    std::cout << std::endl;

    // 显示角点
    cv::Mat img_1_corner, img_2_corner;
    photogrammetry::DrawCorners(img_1, img_1_corner, corners_1);
    photogrammetry::DrawCorners(img_2, img_2_corner, corners_2);
    cv::imwrite(img_1_corner_path, img_1_corner);
    cv::imwrite(img_2_corner_path, img_2_corner);
    cv::imshow("img_1 角点检测后图像", img_1_corner);
    cv::imshow("img_2 角点检测后图像", img_2_corner);
    cv::waitKey(0);

    // 进行相关系数匹配
    std::cout << "开始相关系数匹配：" << std::endl;
    photogrammetry::CorrelationMatcher corrMatcher;
    corrMatcher.setWindowSize(corr_window_size);
    corrMatcher.setThreshold(corr_threshold);

    std::vector<photogrammetry::Match> corrMatches;

    t.tic();
    // corrMatcher.match(img_1, img_2, corners_1, corrMatches);
    corrMatcher.matchImproved(img_1, img_2, corners_1, corners_2, corrMatches);
    time = t.toc();

    std::cout << "相关系数匹配窗口大小：" << corr_window_size << "\t阈值：" << corr_threshold << std::endl;
    std::cout << "相关系数匹配用时：" << time << "秒" << std::endl;
    std::cout << "相关系数匹配到同名点：" << corrMatches.size() << std::endl;
    std::cout << std::endl;

    std::ofstream ofs;
    ofs.open(result_corr_path);
    for (auto match : corrMatches)
    {
        ofs << "srcX: " << match.srcPt.x
            << "\tsrcY: " << match.srcPt.y
            << "\tdstX: " << match.dstPt.x
            << "\tdstY: " << match.dstPt.y
            << "\tidx: " << match.dist << std::endl;
    }
    ofs.close();

    // 显示匹配结果
    cv::Mat img_match;
    corrMatcher.drawMatches(img_1, img_2, img_match, corrMatches);
    cv::imwrite(img_match_path, img_match);
    cv::imshow("匹配结果：", img_match);
    cv::waitKey(0);

    // 在相关系数匹配的基础上进行最小二乘匹配
    photogrammetry::LsqMatcher lsqMatcher;
    lsqMatcher.setWindowSize(lsq_window_size);
    lsqMatcher.setThreshold(lsq_threshold);
    std::vector<photogrammetry::Match> lsqMatches;

    t.tic();
    for (auto match : corrMatches)
    {
        if (lsqMatcher.subPixelMatch(img_1, img_2, match))
            lsqMatches.push_back(match);
    }
    time = t.toc();

    std::cout << "最小二乘匹配窗口大小：" << lsq_window_size << "\t阈值：" << lsq_threshold << std::endl;
    std::cout << "最小二乘匹配用时：" << time << "秒" << std::endl;
    std::cout << "最小二乘匹配到同名点：" << lsqMatches.size() << std::endl;
    std::cout << std::endl;

    ofs.open(result_lsq_path);
    for (auto match : lsqMatches)
    {
        ofs << "srcX: " << match.srcPt.x
            << "\tsrcY: " << match.srcPt.y
            << "\tdstX: " << match.dstPt.x
            << "\tdstY: " << match.dstPt.y
            << "\tidx: " << match.dist << std::endl;
    }
    ofs.close();

    std::cout << "运行结束!" << std::endl;

    return 0;
}
