#include "findRedBox.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

const float kRot[3][3] = {
   {1.601359f, 0.089964f, -262.164134f},
   {0.000000f, -2.824870f, 299.139344f},
   {-0.000000f, 0.038385f, 1.000000f},
};
const float kInvRot[3][3] = {
   {3.162774f, 2.244444f, 157.764331f},
   {-0.000000f, -0.353999f, 105.894904f},
   {0.000000f, 0.013588f, 1.000000f},
};

/*色块识别部分*/
constexpr int kMinSeedRun = 6;          // 最小种子长度
constexpr int kMinComponentPixels = 20; // 最小连通域像素数
constexpr   int min_Area = 20;
constexpr  int max_Area = 60;
// 点运算（内积、加减、缩放、模长、归一化）
inline float dot(const cv::Point2f& a, const cv::Point2f& b) { return a.x*b.x + a.y*b.y; }
inline cv::Point2f add(const cv::Point2f& a, const cv::Point2f& b) { return {a.x+b.x, a.y+b.y}; }
inline cv::Point2f sub(const cv::Point2f& a, const cv::Point2f& b) { return {a.x-b.x, a.y-b.y}; }
inline cv::Point2f scale(const cv::Point2f& a, float s) { return {a.x*s, a.y*s}; }
inline float norm(const cv::Point2f& a) { return std::sqrt(dot(a,a)); }
inline cv::Point2f normalize(const cv::Point2f& a, const cv::Point2f& fallback = {1,0}) {
    float n = norm(a);
    return (n < 1e-4f) ? fallback : cv::Point2f{a.x/n, a.y/n};
}

/*
- wx: BEV世界坐标x
- wy: BEV世界坐标y
- invR[3][3]: 世界坐标到图像坐标的转换矩阵
- R[3][3]: 图像坐标到世界坐标的转换矩阵
- ix: 图像坐标x
- iy: 图像坐标y
*/
/* ====== world -> image ====== */
static inline void w2i(float Wx, float Wy, const float invR[3][3], float* ix, float* iy)
{
    float d = invR[2][0]*Wy + invR[2][1]*Wx + invR[2][2];
    if (d < 1e-4f && d > -1e-4f) d = (d >= 0) ? 1e-4f : -1e-4f;
    float inv_d = 1.0f / d;
    *ix = (invR[1][0]*Wy + invR[1][1]*Wx + invR[1][2]) * inv_d;
    *iy = (invR[0][0]*Wy + invR[0][1]*Wx + invR[0][2]) * inv_d;
}

/* ====== image -> world ====== */
static inline void i2w(float Ix, float Iy, const float R[3][3], float* Wx, float* Wy)
{
    float d = R[2][0]*Iy + R[2][1]*Ix + R[2][2];
    if (d < 1e-4f && d > -1e-4f) d = (d >= 0) ? 1e-4f : -1e-4f;
    float inv_d = 1.0f / d;
    *Wx = (R[1][0]*Iy + R[1][1]*Ix + R[1][2]) * inv_d;
    *Wy = (R[0][0]*Iy + R[0][1]*Ix + R[0][2]) * inv_d;
}

/**
 * 实现说明：
 * 1. 读取指定像素的亮度 Y 分量。
 * 2. 奇偶像素分别落在像素对中的不同位置。
 */
inline int yuyv_pixel_offset(uint32_t stride_bytes, int x, int y)
{
    return y * static_cast<int>(stride_bytes) + (x & ~1) * 2;
}

inline uint8_t yuyv_pixel_y(const uint8_t *frame_yuyv, uint32_t stride_bytes, int x, int y)
{
    const int base = yuyv_pixel_offset(stride_bytes, x, y);
    return frame_yuyv[base + ((x & 1) ? 2 : 0)];
}

/**
 * 实现说明：
 * 1. 读取指定像素所在像素对的 U 分量。
 * 2. 该值会被相邻两个像素共同使用。
 */
inline uint8_t yuyv_pixel_u(const uint8_t *frame_yuyv, uint32_t stride_bytes, int x, int y)
{
    return frame_yuyv[yuyv_pixel_offset(stride_bytes, x, y) + 1];
}

/**
 * 实现说明：
 * 1. 读取指定像素所在像素对的 V 分量。
 * 2. 红色判定主要依赖该分量与 U/Y 的差值关系。
 */
inline uint8_t yuyv_pixel_v(const uint8_t *frame_yuyv, uint32_t stride_bytes, int x, int y)
{
    return frame_yuyv[yuyv_pixel_offset(stride_bytes, x, y) + 3];
}

/**
 * 实现说明：
 * 1. 直接基于 YUYV 原始值判断某像素是否满足红色目标阈值。
 * 2. 同时限制亮度、U/V 和差值关系，以排除暗色噪声。
 */
inline bool is_red_pixel(const uint8_t *frame_yuyv, uint32_t stride_bytes, int x, int y)
{
    const int yy = yuyv_pixel_y(frame_yuyv, stride_bytes, x, y);
    const int uu = yuyv_pixel_u(frame_yuyv, stride_bytes, x, y);
    const int vv = yuyv_pixel_v(frame_yuyv, stride_bytes, x, y);
    // return (yy >= 28) && (vv >= 150) && (uu <= 128) && ((vv - uu) >= 48) && ((vv - yy) >= 14);
    return (vv >= 150) && (uu <= 128) && ((vv - uu) >= -130) && ((vv - yy) >= -130) ;
}

/**
 * 实现说明：
 * 1. 在已拿到行指针的情况下快速判断该行某像素是否为红色。
 * 2. 供逐行种子搜索复用，减少偏移换算开销。
 */
inline bool is_red_pixel_in_row(const uint8_t *row_ptr, int x)
{
    const int pair_offset = (x & ~1) * 2;
    const int yy = static_cast<int>(row_ptr[pair_offset + ((x & 1) ? 2 : 0)]);
    const int uu = static_cast<int>(row_ptr[pair_offset + 1]);
    const int vv = static_cast<int>(row_ptr[pair_offset + 3]);
    // return (yy >= 28) && (vv >= 150) && (uu <= 128) && ((vv - uu) >= 48) && ((vv - yy) >= 14);
    return (vv >= 150) && (uu <= 128) && ((vv - uu) >= -130) && ((vv - yy) >= -130) ;
}

/**
 * @brief 在 YUYV 图像中搜索红色种子段
 */
bool find_seed_run(const uint8_t* yuyv, uint32_t stride, int W, int H,int search_left, int search_right, int search_top, int search_bottom, int& seed_y, int& seed_left, int& seed_right) {
    for (int y = search_bottom; y >= search_top; --y) {
        const uint8_t* row = yuyv + y * stride;
        int best_len = 0, best_left = -1, best_right = -1;
        int run_start = -1;
        for (int x = search_left; x <= search_right; ++x) {
            if (is_red_pixel_in_row(row, x)) {
                if (run_start < 0) run_start = x;
            } else {
                if (run_start >= 0) {
                    int len = x - run_start;
                    if (len > best_len) {
                        best_len = len;
                        best_left = run_start;
                        best_right = x - 1;
                    }
                    run_start = -1;
                }
            }
        }
        //如果最后一个像素是红色，需要检查是否是种子段
        if (run_start >= 0) {
            int len = search_right - run_start + 1;
            if (len > best_len) {
                best_left = run_start;
                best_right = search_right;
            }
        }
        if (best_len >= 6) {
            seed_y = y;
            seed_left = best_left;
            seed_right = best_right;
            return true;
        }
    }
    return false;
}

/**
 * @brief 从种子段出发，通过 BFS 提取红色连通域
 */
bool bfs_component(const uint8_t* yuyv, uint32_t stride, int W, int H,
                   int seed_y, int seed_left, int seed_right,
                   Rect& bbox, std::vector<cv::Point2f>& points) {
    std::vector<uint8_t> visited(H * W, 0);
    std::vector<int> queue(W * H);
    int head = 0, tail = 0;

    const uint8_t* seed_row = yuyv + seed_y * stride;
    for (int x = seed_left; x <= seed_right; ++x) {
        if (is_red_pixel_in_row(seed_row, x)) {
            visited[seed_y * W + x] = 1;
            queue[tail++] = seed_y * W + x;
        }
    }

    int min_x = W, max_x = 0, min_y = H, max_y = 0;
    points.clear();

    while (head < tail) {
        int p = queue[head++];
        int x = p % W, y = p / W;
        min_x = std::min(min_x, x); max_x = std::max(max_x, x);
        min_y = std::min(min_y, y); max_y = std::max(max_y, y);
        points.push_back({(float)x, (float)y});  //红色联通域的全部像素坐标

        const int dx[4] = {-1, 1, 0, 0};//左右上下
        const int dy[4] = {0, 0, -1, 1};
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i], ny = y + dy[i];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (visited[ny * W + nx]) continue;
            if (!is_red_pixel(yuyv, stride, nx, ny)) continue;
            visited[ny * W + nx] = 1; //已入队标记
            queue[tail++] = ny * W + nx;
        }
    }

    if (points.size() < kMinComponentPixels) return false;

    bbox.x = min_x;
    bbox.y = min_y;
    bbox.width  = max_x - min_x + 1;
    bbox.height = max_y - min_y + 1;
    // 筛选异常连通域（宽度或高度小于 5/2 像素，长宽比不在 1.5-6.0 范围）
    int bw = bbox.width;
    int bh = bbox.height;
    if (bw < 5 || bh < 2) return false;
    // double aspect = (double)bw / (double)bh;
    // if (aspect < 1.5 || aspect > 6.0) return false;
    // int min_Area = 50;
    // int max_Area = 150;
    // if (bbox.width * bbox.height < min_Area || bbox.width * bbox.height > max_Area) return false;
    return true;
}

// /**
//  * @brief 通过 PCA 拟合定向矩形
//  */
void fit_oriented_rect(const std::vector<cv::Point2f>& points,
                       cv::Point2f corners[4], cv::Point2f& center) {
    int N = (int)points.size();
    double sx = 0, sy = 0;
    for (auto& p : points) { sx += p.x; sy += p.y; }
    center.x = (float)(sx / N);
    center.y = (float)(sy / N);

    double cov_xx = 0, cov_xy = 0, cov_yy = 0;
    for (auto& p : points) {
        double dx = p.x - center.x, dy = p.y - center.y;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }
    //
    float theta = 0.5f * (float)std::atan2(2.0 * cov_xy, cov_xx - cov_yy + 1e-9); // 求主轴方向
    float c = std::cos(theta), s = std::sin(theta);
    cv::Point2f major_dir{c, s}; // 主轴方向（长边方向）
    cv::Point2f minor_dir{-s, c}; // 副轴方向（短边方向）

    float min_maj = 1e9f, max_maj = -1e9f, min_min = 1e9f, max_min = -1e9f;
    for (auto& p : points) {
        float dx = p.x - center.x, dy = p.y - center.y;
        float mj = dx * major_dir.x + dy * major_dir.y;  // 投影到主轴
        float mn = dx * minor_dir.x + dy * minor_dir.y; // 投影到副轴
        if (mj < min_maj) min_maj = mj; // 沿主轴的最小投影
        if (mj > max_maj) max_maj = mj; // 沿主轴的最大投影
        if (mn < min_min) min_min = mn; // 沿副轴的最小投影
        if (mn > max_min) max_min = mn; // 沿副轴的最大投影
    }
    // 四个角点坐标
    corners[3] = add(center, add(scale(major_dir, min_maj), scale(minor_dir, max_min)));
    corners[2] = add(center, add(scale(major_dir, max_maj), scale(minor_dir, max_min)));
    corners[1] = add(center, add(scale(major_dir, max_maj), scale(minor_dir, min_min)));
    corners[0] = add(center, add(scale(major_dir, min_maj), scale(minor_dir, min_min)));
}


bool Detect_Red_Target(const uint8_t* yuyv, int width, int height, int stride,Rect& bbox, std::vector<cv::Point2f>& out_points) {
    int seed_y = -1, seed_left = -1, seed_right = -1;
    bool found = find_seed_run(yuyv, stride, width, height,
                               0, width-1, 0, height-1,
                               seed_y, seed_left, seed_right);
    if (!found) return false;
    std::vector<cv::Point2f> points;
    if (!bfs_component(yuyv, stride, width, height,
                       seed_y, seed_left, seed_right,
                       bbox, points))
        return false;
    out_points = points;
    return true;
}
DetectionResult findRedBoxAndIpm(const cv::Mat &yuyv,
                 const float rot[3][3],
                 const float inv_rot[3][3],
                 DetectionResult &target) {
    // 1. 初始化为未找到
    target.found = false;
    // 2. 参数校验
    if (yuyv.empty() || yuyv.type() != CV_8UC2) {
        return DetectionResult{false, {0, 0, 0, 0}};
    }
    if (rot == nullptr || inv_rot == nullptr) {
        return DetectionResult{false, {0, 0, 0, 0}};
    }
    // 3. 检测红色目标，拿到 BFS 连通域像素
    std::vector<cv::Point2f> red_points;
    bool found = Detect_Red_Target(yuyv.ptr<uchar>(0), yuyv.cols, yuyv.rows,
                                   yuyv.step, target.bbox, red_points);
    if (!found) return target;

    // 4. 每个红色像素 i2w → BEV，收集 BEV 点集
    std::vector<cv::Point2f> bev_points;
    bev_points.reserve(red_points.size());
    for (auto& p : red_points) {
        float wx, wy;
        i2w(p.x, p.y, rot, &wx, &wy);
        bev_points.push_back(cv::Point2f(wx, wy));
    }

    if (bev_points.size() < 8) return target;

    // 4.5 原图 BFS 点直接 PCA（调试对照）
    //cv::Point2f raw_center;
    //fit_oriented_rect(red_points, target.raw_pca_pts, raw_center);
    // printf("[原图PCA获取的坐标信息] N=%zu pts(%.0f,%.0f)-(%.0f,%.0f)-(%.0f,%.0f)-(%.0f,%.0f)\n",
    //        red_points.size(),
    //        target.raw_pca_pts[0].x, target.raw_pca_pts[0].y,
    //        target.raw_pca_pts[1].x, target.raw_pca_pts[1].y,
    //        target.raw_pca_pts[2].x, target.raw_pca_pts[2].y,
    //        target.raw_pca_pts[3].x, target.raw_pca_pts[3].y);

    // 5. BEV 域 PCA → 4 个紧贴色块的旋转角点
    cv::Point2f center;
    fit_oriented_rect(bev_points, target.IpmRed_pts, center);
    printf("[BEV PCA] N=%zu pts(%.0f,%.0f)-(%.0f,%.0f)-(%.0f,%.0f)-(%.0f,%.0f)\n",
           bev_points.size(),
           target.IpmRed_pts[0].x, target.IpmRed_pts[0].y,
           target.IpmRed_pts[1].x, target.IpmRed_pts[1].y,
           target.IpmRed_pts[2].x, target.IpmRed_pts[2].y,
           target.IpmRed_pts[3].x, target.IpmRed_pts[3].y);
    // // 5. BEV 域求角点（临时用极值法，先验证 i2w 正常）
    // float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
    // for (auto& p : bev_points) {
    //     if (p.x < min_x) min_x = p.x;
    //     if (p.x > max_x) max_x = p.x;
    //     if (p.y < min_y) min_y = p.y;
    //     if (p.y > max_y) max_y = p.y;
    // }
    // target.IpmRed_pts[0] = cv::Point2f(min_x, max_y);
    // target.IpmRed_pts[1] = cv::Point2f(max_x, max_y);
    // target.IpmRed_pts[2] = cv::Point2f(max_x, min_y);
    // target.IpmRed_pts[3] = cv::Point2f(min_x, min_y);

    // 5.5 生成 BEV 鸟瞰图，存映射参数
    std::memset(target.bev_viz, 0, sizeof(target.bev_viz));
    float v_min_x = 1e9f, v_max_x = -1e9f, v_min_y = 1e9f, v_max_y = -1e9f;
    for (auto& p : bev_points) {
        if (p.x < v_min_x) v_min_x = p.x;
        if (p.x > v_max_x) v_max_x = p.x;
        if (p.y < v_min_y) v_min_y = p.y;
        if (p.y > v_max_y) v_max_y = p.y;
    }
    const float pad = 15.0f;
    target.bev_off_x = v_min_x - pad;
    target.bev_off_y = v_min_y - pad;
    float vw = v_max_x - v_min_x + pad * 2;
    float vh = v_max_y - v_min_y + pad * 2;
    target.bev_scale = std::min(319.0f / vw, 239.0f / vh);
    for (auto& p : bev_points) {
        int bx = (int)((p.x - target.bev_off_x) * target.bev_scale);
        int by = (int)((p.y - target.bev_off_y) * target.bev_scale);
        if (bx >= 0 && bx < 320 && by >= 0 && by < 240) {
            by = 239 - by;
            target.bev_viz[by][bx] = 255;
        }
    }
    target.bev_viz_valid = true;

    // 6. w2i 回投原图 → RAW_pts，再极角排序成顺时针
    for (int i = 0; i < 4; i++) {
        float ix, iy;
        w2i(target.IpmRed_pts[i].x, target.IpmRed_pts[i].y, inv_rot, &ix, &iy);
        target.RAW_pts[i] = cv::Point2f(ix, iy);
    }
    // 极角排序：左上[0]→右上[1]→右下[2]→左下[3] 顺时针
    float cx = 0, cy = 0;
    for (int i = 0; i < 4; i++) { cx += target.RAW_pts[i].x; cy += target.RAW_pts[i].y; }
    cx *= 0.25f; cy *= 0.25f;
    std::sort(target.RAW_pts, target.RAW_pts + 4,
              [cx, cy](const cv::Point2f& a, const cv::Point2f& b) {
                  float aa = atan2f(a.y - cy, a.x - cx);
                  float ab = atan2f(b.y - cy, b.x - cx);
                  if (aa < -1.5708f) aa += 6.2832f;
                  if (ab < -1.5708f) ab += 6.2832f;
                  return aa < ab;
              });
    std::swap(target.RAW_pts[1], target.RAW_pts[3]);

    target.found = true;
    printf("[RAW 回投] RAW_pts: (%.0f,%.0f)-(%.0f,%.0f)-(%.0f,%.0f)-(%.0f,%.0f)\n",
           target.RAW_pts[0].x, target.RAW_pts[0].y,
           target.RAW_pts[1].x, target.RAW_pts[1].y,
           target.RAW_pts[2].x, target.RAW_pts[2].y,
           target.RAW_pts[3].x, target.RAW_pts[3].y);

    // ============================================================
    // 步骤7：图片区域扩展（图像空间操作）
    //   - 底边 = RAW_pts 图像上方两点(y小的那端)
    //   - 上边 = 底边沿副轴向上平移 EXT_PX
    //   - 副轴 = 底边向量旋转 90°, 强制 y<0
    // ============================================================
    {
        constexpr float PAD_PX =  0.0f;
        constexpr float EXT_PX = 25.0f;

        // 找 y 最小的那对边(图像上方)
        float ey01 = (target.RAW_pts[0].y + target.RAW_pts[1].y) * 0.5f;
        float ey23 = (target.RAW_pts[2].y + target.RAW_pts[3].y) * 0.5f;
        int t0, t1;
        if (ey01 < ey23) { t0 = 0; t1 = 1; }
        else             { t0 = 2; t1 = 3; }

        cv::Point2f bL = target.RAW_pts[t0];
        cv::Point2f bR = target.RAW_pts[t1];
        cv::Point2f rMj(bR.x - bL.x, bR.y - bL.y);
        float rml = sqrtf(rMj.x * rMj.x + rMj.y * rMj.y);
        if (rml < 1e-4f) goto skip_extend;
        rMj.x /= rml; rMj.y /= rml;

        bL.x -= rMj.x * PAD_PX; bL.y -= rMj.y * PAD_PX;
        bR.x += rMj.x * PAD_PX; bR.y += rMj.y * PAD_PX;

        cv::Point2f rMi(-rMj.y, rMj.x);
        if (rMi.y > 0) { rMi.x = -rMi.x; rMi.y = -rMi.y; }

        target.IpmRec_pts[0] = cv::Point2f(bL.x + rMi.x * EXT_PX, bL.y + rMi.y * EXT_PX);
        target.IpmRec_pts[1] = cv::Point2f(bR.x + rMi.x * EXT_PX, bR.y + rMi.y * EXT_PX);
        target.IpmRec_pts[2] = bR;
        target.IpmRec_pts[3] = bL;
    }
    skip_extend:
    return target;
}