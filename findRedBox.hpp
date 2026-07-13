#ifndef _FIND_RED_BOX_HPP_          // 1. 头文件保护
#define _FIND_RED_BOX_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>

// 3. 常量定义
constexpr int TARGET_CROP_W = 50 ;   ///< ROI 输出宽度（像素）
constexpr int TARGET_CROP_H = 40;   ///< ROI 输出高度（像素）
constexpr int AreaMin = 200;   ///< 色块最小面积（像素）
constexpr int AreaMax = 20;   ///< 色块最大面积（像素）

// 4. 枚举定义


// 5. 结构体定义
// 目标矩形结构体
/// 轴对齐矩形（整数坐标）
struct Rect {
    int x, y, width, height;
};

typedef struct {
    bool found;  //< 是否检测到目标
    Rect red_rect;

    cv::Point2f raw_pca_pts[4]; //< 原图 BFS 点直接 PCA 的四个角点（调试用）
    
    cv::Point2f IpmRed_pts[4]; //< 红色色块在 IPM 空间中的四个角点（顺时针：左上、右上、右下、左下）
    cv::Point2f RAW_pts[4]; //< BEV PCA 回投原图的四个角点
    cv::Point2f IpmRec_pts[4]; //< 图片区域在 IPM 空间中的四个角点（顺时针：左上、右上、右下、左下）
    cv::Point2f RawRec_pts[4]; //< 图片区域在原始图像中的四个角点
    Rect bbox; //< 轴对齐包围盒
    uint8_t bev_viz[240][320]; //< BEV 鸟瞰图（灰度，红点聚集处=白）
    bool bev_viz_valid; //< BEV 图是否有效
    float bev_off_x; //< BEV 图对应最小世界x（cm）
    float bev_off_y; //< BEV 图对应最小世界y（cm）
    float bev_scale; //< BEV 图每像素对应 cm
    uint8_t roi_rgb[50 * 40 * 3]; //< 裁剪 ROI RGB888 图像数据
    int roi_w; //< 裁剪宽度
    int roi_h; //< 裁剪高度
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

/* 裁剪并逆透视 12x12cm 目标板(位于红色长方形 12cm 远端长边上方)
 * yuyv     : 原始 YUYV 打包图像(与 findRedBoxAndIpm 同帧同图), 320x240
 * rot      : 逆透视矩阵 image->world (3x3), 160x120 域
 * inv_rot  : 逆透视矩阵 world->image (3x3), 160x120 域
 * outTarget: 输出 40x40 RGB888 俯视图 (近边在底, 远边在顶, 左右保持)
 * outSize  : 输出边长(像素), 默认 40
 * 返回     : true=成功, false=未先调用 findRedBoxAndIpm 或矩阵退化
 * 前置条件: 必须先成功调用 findRedRect 缓存 BEV 角点. */
bool CropRect(const cv::Mat &yuyv,
                const float rot[3][3],
                const float inv_rot[3][3],
                DetectionResult &outTarget
                );

// ---------- 计算图片区域四边形 ----------
//计算 IPM 空间中红色矩形的中心、主轴和副轴方向
void compute_image_roi_quad(const cv::Point2f raw_corners[4],
                            const float rot[3][3],
                            const float inv_rot[3][3],
                            float image_quad[4][2]);


#endif