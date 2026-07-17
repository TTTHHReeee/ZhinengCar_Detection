#ifndef __NCNN_DEMO_H
#define __NCNN_DEMO_H

#include <string>
#include <unordered_map>

#include <opencv2/core.hpp>

#include "lq_ncnn.hpp"

class NcnnClassifier
{


// public:
// typedef struct{
//     int class_id;
//     float confidence;
// }RESULT;

public:
    NcnnClassifier();
    ~NcnnClassifier();

    NcnnClassifier(const NcnnClassifier &) = delete;
    NcnnClassifier &operator=(const NcnnClassifier &) = delete;
    NcnnClassifier(NcnnClassifier &&) = delete;
    NcnnClassifier &operator=(NcnnClassifier &&) = delete;

public:
    bool     ncnn_Init(void);
    //int     Red_Detection(cv::Rect &out_rect, cv::Point &out_center, cv::Point &bottom_point);
    //cv::Mat  roi_Extract(const cv::Rect &red_rect);
    int     Main_Detection(const cv::Mat &roi_rgb);   // 返回 0/1/2，-1=低于阈值

    // 获取当前帧 BGR 图像
    //cv::Mat  GetBGR(void) const { return bgr_; }
    // 获取最近一次 roi_Extract 计算出的 ROI 矩形
    //cv::Rect GetLastROI(void) const { return last_roi_; }
    // 类别名 → 数字映射
    static int MapLabelToId(const std::string &label);

private:
    LQ_NCNN  ncnn_;
    cv::Mat   bgr_;
    cv::Rect  last_roi_;
    cv::Point2f rd_bot_;

 
    inline static const std::unordered_map<std::string, int> kLabelMap = {
        {"Secure",  1},
        {"see", 1},
        {"Police_car",  2},
        {"Secure_car", 2},
        {"gun", 3},
        {"boom", 3},
    };
};

#endif
