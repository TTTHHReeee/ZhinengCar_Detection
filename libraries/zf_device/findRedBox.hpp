#ifndef _FIND_RED_BOX_HPP_          // 1. 头文件保护
#define _FIND_RED_BOX_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>

// 3. 常量定义
// constexpr int TARGET_CROP_W = 50 ;   ///< ROI 输出宽度（像素）
//constexpr int TARGET_CROP_H = 40;   ///< ROI 输出高度（像素）
constexpr int BEV_W = 50;
constexpr int BEV_H = 50;
/* ====== 红色阈值参数 ====== */
constexpr int VV = 150;  // v下界
constexpr int UU = 128;  // u上界
constexpr int V_U = -130;  // v-u下界
constexpr int V_Y = -130;  // v-y下界
// constexpr int AreaMin = 200;   ///< 未使用
// constexpr int AreaMax = 20;    ///< 未使用

// 4. 枚举定义


// 5. 结构体定义
// 目标矩形结构体
/// 轴对齐矩形（整数坐标）
struct Rect {
    int x, y, width, height;
};

typedef struct {
    bool found;  //< 是否检测到目标
    // Rect red_rect;                    // 未使用

    // cv::Point2f raw_pca_pts[4];       // 未使用
    
    cv::Point2f IpmRed_pts[4]; //< 红色色块在 IPM 空间中的四个角点（顺时针：左上、右上、右下、左下）
    cv::Point2f RAW_pts[4]; //< BEV PCA 回投原图的四个角点
    cv::Point2f IpmRec_pts[4]; //< 图片区域在 IPM 空间中的四个角点（顺时针：左上、右上、右下、左下）
    cv::Point2f RawRec_pts[4]; //< 图片区域在原始图像中的四个角点
    Rect bbox; //< 轴对齐包围盒

    // uint8_t bev_viz[240][320];        // 未使用 240KB
    // bool bev_viz_valid;
    // float bev_off_x, bev_off_y, bev_scale;
    // uint8_t roi_rgb[50 * 40 * 3];     // 未使用
    // int roi_w, roi_h;
} DetectionResult;

extern const float kRot[3][3];
extern const float kInvRot[3][3];


// 6. extern 全局变量声明
//extern int xxx_var;

// 7. 函数声明
/* 在赛道区域内寻找红色长方形(起跑线, 现实 12x5cm)
 * 关键: 红色像素先经逆透视变到 BEV 世界坐标, 在 BEV 域做 PCA 求角点.
 *       BEV 域无透视畸变, 红矩形为真矩形, PCA 包围盒精确贴合 4 个真实
 *       角点, 与目标板倾斜无关; cropTarget 沿 BEV 短边方向延伸 12cm,
 *       方向误差不再被放大.
 * yuyv    : 原始 YUYV 打包图像, CV_8UC2, 分辨率 320x240
 * rot     : 逆透视矩阵 image->world (3x3), 160x120 域
 * inv_rot : 逆透视矩阵 world->image (3x3), 160x120 域
 * outRect : 输出四个角点(BEV 角点)
 * 返回    : 0 表示找到, -1 表示未找到. */
DetectionResult findRedBoxAndIpm(const cv::Mat &yuyv,
                 const float rot[3][3],
                 const float inv_rot[3][3],
                 DetectionResult &target);
/**
 * @brief 在 YUYV 图像中搜索最长红色种子段（自底向上）
 */
bool find_seed_run(const uint8_t* yuyv, uint32_t stride, int W, int H,
                   int search_left, int search_right,
                   int search_top, int search_bottom,
                   int& seed_y, int& seed_left, int& seed_right);

/**
 * @brief 从种子段出发进行四邻域 BFS，提取红色连通域
 */
bool bfs_component(const uint8_t* yuyv, uint32_t stride, int W, int H,
                   int seed_y, int seed_left, int seed_right,
                   Rect& bbox, std::vector<cv::Point2f>& points);

/**
 * @brief 检测红色色块：找种子段 → BFS提取连通域 → 输出包围盒
 */
bool Detect_Red_Target(const uint8_t* yuyv, int width, int height, int stride,
                       Rect& bbox, std::vector<cv::Point2f>& out_points);

/**
 * @brief 对连通域像素进行 PCA 拟合，得到定向矩形的四个角点
 */
void fit_oriented_rect(const std::vector<cv::Point2f>& points,
                       cv::Point2f corners[4], cv::Point2f& center);

/**
 * @brief 从已检测到的红色色块 BEV 角点扩展出上方图片区域
 * @param target       已调用 findRedBoxAndIpm 的结果（需 target.found == true）
 * @param inv_rot      逆透视矩阵 world->image (3x3)
 * @param IpmRec_out   [输出] 扩展 ROI 的 BEV 域角点
 * @param RawRec_out   [输出] 扩展 ROI 回投原图的角点
 * @return true=扩展成功, false=target 未找到或色块过小
 */
bool expand_roi_from_red(const DetectionResult &target,
                         const float inv_rot[3][3],
                         cv::Point2f IpmRec_out[4],
                         cv::Point2f RawRec_out[4]);

bool crop_yuyv_to_rgb(const cv::Mat &yuyv,
                      const cv::Point2f raw_corners[4],
                      cv::Mat &rgb_out,
                      cv::Point2f adj_corners[4]);

/**
 * @brief 将原图四边形区域展开为轴对齐 BEV 图（供分类模型推理）
 * @param frame       原始 BGR 图像
 * @param raw_corners 原图四边形角点（顺时针:左上/右上/右下/左下）
 * @param bev_w       输出宽度(像素)
 * @param bev_h       输出高度(像素)
 * @return BEV 鸟瞰图 BGR
 */
cv::Mat warp_raw_to_bev(const cv::Mat &frame,
                        const cv::Point2f raw_corners[4],
                        int bev_w, int bev_h);

void draw_rotated_quad(cv::Mat &img, const cv::Point2f corners[4], cv::Scalar color);

#endif