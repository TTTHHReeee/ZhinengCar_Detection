#ifndef _FIND_RED_BOX_HPP_
#define _FIND_RED_BOX_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>

constexpr int BEV_W = 50;
constexpr int BEV_H = 50;

/* ====== 红色阈值参数 ====== */
constexpr int VV = 165;
constexpr int UU = 128;
constexpr int V_U = -130;
constexpr int V_Y = -130;

struct Rect {
    int x, y, width, height;
};

typedef struct {
    bool found;
    bool slow_down = false;
    int Left_Or_Right = 0;
    int flag = -1;
    cv::Point2f IpmRed_pts[4];
    cv::Point2f RAW_pts[4];
    cv::Point2f IpmRec_pts[4];
    cv::Point2f RawRec_pts[4];
    Rect bbox;
} DetectionResult;

extern const float kRot[3][3];
extern const float kInvRot[3][3];
extern int SEARCH_TOP;
extern const int SEARCH_BOTTOM;

DetectionResult findRedBoxAndIpm(const cv::Mat &yuyv,
                 const float rot[3][3],
                 const float inv_rot[3][3],
                 DetectionResult &target);

bool find_seed_run(const uint8_t* yuyv, uint32_t stride, int W, int H,
                   int search_left, int search_right,
                   int search_top, int search_bottom,
                   int& seed_y, int& seed_left, int& seed_right);

bool find_seed_run_topdown(const uint8_t* yuyv, uint32_t stride, int W, int H,
                           int& seed_y, int& seed_left, int& seed_right);

bool bfs_component(const uint8_t* yuyv, uint32_t stride, int W, int H,
                   int seed_y, int seed_left, int seed_right,
                   Rect& bbox, std::vector<cv::Point2f>& points,
                   DetectionResult &target);

bool Detect_Red_Target(const uint8_t* yuyv, int width, int height, int stride,
                       Rect& bbox, std::vector<cv::Point2f>& out_points,
                       DetectionResult &target);

void detect_slow_down(const uint8_t* yuyv, uint32_t stride, int W, int H,
                      DetectionResult &target);

void fit_oriented_rect(const std::vector<cv::Point2f>& points,
                       cv::Point2f corners[4], cv::Point2f& center);

bool filter_brick(DetectionResult &target);

bool expand_roi_from_red(const DetectionResult &target,
                         const float inv_rot[3][3],
                         cv::Point2f IpmRec_out[4],
                         cv::Point2f RawRec_out[4]);

cv::Mat warp_yuyv_to_bev_rgb(const cv::Mat &yuyv,
                              const cv::Point2f raw_corners[4],
                              int bev_w, int bev_h);

void draw_rotated_quad(cv::Mat &img, const cv::Point2f corners[4], cv::Scalar color);

#endif