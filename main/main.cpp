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
    // if (argc != 3)
    // {
    //     std::cout << "使用方式：ImageMatch img1 img2" << std::endl;
    //     return 1;
    // }
    std::string img_path_1 = "../img/LOR50.bmp";
    std::string img_path_2 = "../img/LOR49.bmp";
    // std::string img_path_1 = "../img/star.jpg";
    // std::string img_path_1 = "../img/yosemite1.jpg";
    // std::string img_path_2 = "../img/yosemite2.jpg";

    //-- 读取图像
    cv::Mat img_1 = cv::imread(img_path_1, CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(img_path_2, CV_LOAD_IMAGE_COLOR);
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
    corners_1 = photogrammetry::MoravecCornerDetect(img_1, 5, 700);
    corners_2 = photogrammetry::MoravecCornerDetect(img_2, 5, 700);
    // corners_1 = photogrammetry::SIFTCornerDetect(img_1);
    // corners_2 = photogrammetry::SIFTCornerDetect(img_2);
    double time = t.toc();

    std::cout << "特征点提取用时：" << time << "秒" << std::endl;
    std::cout << "图像1中特征点数量：" << corners_1.size() << std::endl;
    std::cout << "图像2中特征点数量：" << corners_2.size() << std::endl;
    std::cout << std::endl;

    // 显示角点
    cv::Mat img_1_corner, img_2_corner;
    photogrammetry::DrawCorners(img_1, img_1_corner, corners_1);
    photogrammetry::DrawCorners(img_2, img_2_corner, corners_2);
    cv::imwrite("../result/LOR50_corner.jpg", img_1_corner);
    cv::imwrite("../result/LOR49_corner.jpg", img_2_corner);
    cv::imshow("img_1 角点检测后图像", img_1_corner);
    cv::imshow("img_2 角点检测后图像", img_2_corner);
    cv::waitKey(0);

    // 进行相关系数匹配
    std::cout << "开始相关系数匹配：" << std::endl;
    photogrammetry::CorrelationMatcher corrMatcher;
    corrMatcher.setWindowSize(25);
    corrMatcher.setThreshold(0.85);

    std::vector<photogrammetry::Match> corrMatches;

    t.tic();
    // corrMatcher.match(img_1, img_2, corners_1, corrMatches);
    corrMatcher.matchImproved(img_1, img_2, corners_1, corners_2, corrMatches);
    time = t.toc();

    std::cout << "相关系数匹配用时：" << time << "秒" << std::endl;
    std::cout << "相关系数匹配到同名点：" << corrMatches.size() << std::endl;
    std::cout << std::endl;

    std::ofstream ofs;
    ofs.open("../result/LOR_corr_result.txt");
    for (auto match : corrMatches)
    {
        std::cout << "srcX: " << match.srcPt.x
                  << "\tsrcY: " << match.srcPt.y
                  << "\tdstX: " << match.dstPt.x
                  << "\tdstY: " << match.dstPt.y
                  << "\tidx: " << match.dist << std::endl;
        ofs << "srcX: " << match.srcPt.x
            << "\tsrcY: " << match.srcPt.y
            << "\tdstX: " << match.dstPt.x
            << "\tdstY: " << match.dstPt.y
            << "\tidx: " << match.dist << std::endl;
    }
    ofs.close();
    std::cout << std::endl;

    // 显示匹配结果
    cv::Mat img_match;
    corrMatcher.drawMatches(img_1, img_2, img_match, corrMatches);
    cv::imwrite("../result/LOR_corr_match.jpg", img_match);
    cv::imshow("匹配结果：", img_match);
    cv::waitKey(0);

    // 在相关系数匹配的基础上进行最小二乘匹配
    photogrammetry::LsqMatcher lsqMatcher;
    lsqMatcher.setWindowSize(5);
    lsqMatcher.setThreshold(0.95);
    std::vector<photogrammetry::Match> lsqMatches;

    t.tic();
    for (auto match : corrMatches)
    {
        if (lsqMatcher.subPixelMatch(img_1, img_2, match))
            lsqMatches.push_back(match);
    }
    time = t.toc();

    std::cout << "最小二乘匹配用时：" << time << "秒" << std::endl;
    std::cout << "最小二乘匹配到同名点：" << lsqMatches.size() << std::endl;

    ofs.open("../result/LOR_lsq_result.txt");
    for (auto match : lsqMatches)
    {
        std::cout << "srcX: " << match.srcPt.x
                  << "\tsrcY: " << match.srcPt.y
                  << "\tdstX: " << match.dstPt.x
                  << "\tdstY: " << match.dstPt.y
                  << "\tidx: " << match.dist << std::endl;

        ofs << "srcX: " << match.srcPt.x
            << "\tsrcY: " << match.srcPt.y
            << "\tdstX: " << match.dstPt.x
            << "\tdstY: " << match.dstPt.y
            << "\tidx: " << match.dist << std::endl;
    }
    ofs.close();
    std::cout << std::endl;

    std::cout << "运行结束!" << std::endl;

    return 0;
}
