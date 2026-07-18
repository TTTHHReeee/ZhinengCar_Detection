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

/* ====== 红色色块检测参数 ====== */
constexpr int kMinSeedRun = 6;
constexpr int kMinComponentPixels = 20;
constexpr int min_Area = 30;
constexpr int max_Area = 800;

/* ====== BEV 域 ROI 扩展比例 ====== */
constexpr float RATIO_LR  = 0.10f;
constexpr float RATIO_GAP = 0.3f;
constexpr float RATIO_HT  = 2.50f;
constexpr float MIN_LR    = 0.5f;
constexpr float MIN_GAP   = 0.7f;
constexpr float MIN_ROI_W = 14.0f;

/* ====== 红色种子搜索范围（原图像素坐标） ====== */
constexpr int SEARCH_LEFT   = 35;
constexpr int SEARCH_RIGHT  = 285;
constexpr int SEARCH_TOP_FIX = 75;
const int SEARCH_BOTTOM = 200;
int SEARCH_TOP = SEARCH_TOP_FIX;

// 点运算
inline float dot(const cv::Point2f& a, const cv::Point2f& b) { return a.x*b.x + a.y*b.y; }
inline cv::Point2f add(const cv::Point2f& a, const cv::Point2f& b) { return {a.x+b.x, a.y+b.y}; }
inline cv::Point2f sub(const cv::Point2f& a, const cv::Point2f& b) { return {a.x-b.x, a.y-b.y}; }
inline cv::Point2f scale(const cv::Point2f& a, float s) { return {a.x*s, a.y*s}; }
inline float norm(const cv::Point2f& a) { return std::sqrt(dot(a,a)); }

void draw_rotated_quad(cv::Mat &img, const cv::Point2f corners[4], cv::Scalar color) {
    for (int i = 0; i < 4; ++i)
        cv::line(img,
            cv::Point((int)corners[i].x, (int)corners[i].y),
            cv::Point((int)corners[(i+1)&3].x, (int)corners[(i+1)&3].y),
            color, 1);
}

/* ====== world -> image ====== */
static inline void w2i(float Wx, float Wy, const float invR[3][3], float* ix, float* iy)
{
    float d = invR[2][0]*Wx + invR[2][1]*Wy + invR[2][2];
    if (d < 1e-4f && d > -1e-4f) d = (d >= 0) ? 1e-4f : -1e-4f;
    float inv_d = 1.0f / d;
    *ix = (invR[0][0]*Wx + invR[0][1]*Wy + invR[0][2]) * inv_d;
    *iy = (invR[1][0]*Wx + invR[1][1]*Wy + invR[1][2]) * inv_d;
}

/* ====== image -> world ====== */
static inline void i2w(float Ix, float Iy, const float R[3][3], float* Wx, float* Wy)
{
    float d = R[2][0]*Ix + R[2][1]*Iy + R[2][2];
    if (d < 1e-4f && d > -1e-4f) d = (d >= 0) ? 1e-4f : -1e-4f;
    float inv_d = 1.0f / d;
    *Wx = (R[0][0]*Ix + R[0][1]*Iy + R[0][2]) * inv_d;
    *Wy = (R[1][0]*Ix + R[1][1]*Iy + R[1][2]) * inv_d;
}

inline int yuyv_pixel_offset(uint32_t stride_bytes, int x, int y)
{
    return y * static_cast<int>(stride_bytes) + (x & ~1) * 2;
}

inline uint8_t yuyv_pixel_y(const uint8_t *frame_yuyv, uint32_t stride_bytes, int x, int y)
{
    const int base = yuyv_pixel_offset(stride_bytes, x, y);
    return frame_yuyv[base + ((x & 1) ? 2 : 0)];
}

inline uint8_t yuyv_pixel_u(const uint8_t *frame_yuyv, uint32_t stride_bytes, int x, int y)
{
    return frame_yuyv[yuyv_pixel_offset(stride_bytes, x, y) + 1];
}

inline uint8_t yuyv_pixel_v(const uint8_t *frame_yuyv, uint32_t stride_bytes, int x, int y)
{
    return frame_yuyv[yuyv_pixel_offset(stride_bytes, x, y) + 3];
}

inline bool is_red_pixel(const uint8_t *frame_yuyv, uint32_t stride_bytes, int x, int y)
{
    const int yy = yuyv_pixel_y(frame_yuyv, stride_bytes, x, y);
    const int uu = yuyv_pixel_u(frame_yuyv, stride_bytes, x, y);
    const int vv = yuyv_pixel_v(frame_yuyv, stride_bytes, x, y);
    return (vv >= VV) && (uu <= UU) && ((vv - uu) >= V_U) && ((vv - yy) >= V_Y);
}

inline bool is_red_pixel_in_row(const uint8_t *row_ptr, int x)
{
    const int pair_offset = (x & ~1) * 2;
    const int yy = static_cast<int>(row_ptr[pair_offset + ((x & 1) ? 2 : 0)]);
    const int uu = static_cast<int>(row_ptr[pair_offset + 1]);
    const int vv = static_cast<int>(row_ptr[pair_offset + 3]);
    return (vv >= VV) && (uu <= UU) && ((vv - uu) >= V_U) && ((vv - yy) >= V_Y);
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
    const int SEARCH_AREA = 320 * (SEARCH_BOTTOM - SEARCH_TOP + 1);
    std::vector<uint8_t> visited(SEARCH_AREA, 0);  // ~23KB
    std::vector<int> queue(SEARCH_AREA);            // ~92KB
    int head = 0, tail = 0;
    
    const int ROW_OFFSET = SEARCH_TOP * W;  //搜索区域上方被跳过的像素总数 。

    const uint8_t* seed_row = yuyv + seed_y * stride;
    for (int x = seed_left; x <= seed_right; ++x) {
        if (is_red_pixel_in_row(seed_row, x)) {
            // // 用 SEARCH_TOP 偏移
            // int rel_idx = (ny - SEARCH_TOP) * W + nx;
            int rel_idx = seed_y * W + x - ROW_OFFSET;
            visited[rel_idx] = 1;
            queue[tail++] = rel_idx;
        }
    }

    int min_x = W, max_x = 0, min_y = H, max_y = 0;
    points.clear();

    while (head < tail) {
        int p = queue[head++];
        //int x = p % W, y = p / W;
        int x = p % W, y = p / W + SEARCH_TOP;
        min_x = std::min(min_x, x); max_x = std::max(max_x, x);
        min_y = std::min(min_y, y); max_y = std::max(max_y, y);
        points.push_back({(float)x, (float)y});  //红色联通域的全部像素坐标

        const int dx[4] = {-1, 1, 0, 0};//左右上下
        const int dy[4] = {0, 0, -1, 1};
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i], ny = y + dy[i];
            if (nx < SEARCH_LEFT || nx > SEARCH_RIGHT || ny < SEARCH_TOP || ny > SEARCH_BOTTOM) continue;
            int rel_idx = ny * W + nx - ROW_OFFSET;  //相对坐标，0~35519
            //if (visited[ny * W + nx]) continue;
            if (visited[rel_idx]) continue;
            if (!is_red_pixel(yuyv, stride, nx, ny)) continue;
            //visited[ny * W + nx] = 1; //已入队标记
            visited[rel_idx] = 1;
            queue[tail++] = rel_idx;
        }
    }

    if (points.size() < kMinComponentPixels) return false;

    bbox.x = min_x;
    bbox.y = min_y;
    bbox.width  = max_x - min_x + 1;
    bbox.height = max_y - min_y + 1;
    // 仅过滤极小噪声，旋转宽高比检查移至 BEV PCA 之后
    int bw = bbox.width;
    int bh = bbox.height;
    if (bw < 5 || bh < 2) return false;
    int min_Area = 30;
    // int max_Area = 400;
    if (bbox.width * bbox.height < min_Area) return false;
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
    int seed_y = -1, seed_left = -1, seed_right = -1, search_left = SEARCH_LEFT, search_right = SEARCH_RIGHT, search_top = SEARCH_TOP, search_bottom =SEARCH_BOTTOM;
    bool found = find_seed_run(yuyv, stride, width, height,
                               search_left, search_right, search_top, search_bottom,
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
/**
 * 检查 BEV 域 PCA 矩形宽高比是否在合理范围（12cm×5cm ≈ 2.4）
 */
static inline bool check_bev_aspect(const cv::Point2f corners[4])
{
    float ow = sqrtf((corners[1].x-corners[0].x)*(corners[1].x-corners[0].x)
                   + (corners[1].y-corners[0].y)*(corners[1].y-corners[0].y));
    float oh = sqrtf((corners[3].x-corners[0].x)*(corners[3].x-corners[0].x)
                   + (corners[3].y-corners[0].y)*(corners[3].y-corners[0].y));
    if (ow < oh) std::swap(ow, oh);
    float aspect = ow / oh;
    return (aspect >= 1.8f && aspect <= 3.5f);
}

/**
 * 将 4 个角点按极角排序为顺时针：左上[0]→右上[1]→右下[2]→左下[3]
 */
static inline void sort_corners_cw(cv::Point2f pts[4])
{
    float cx = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) * 0.25f;
    float cy = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) * 0.25f;
    std::sort(pts, pts + 4,
        [cx, cy](const cv::Point2f& a, const cv::Point2f& b) {
            float aa = atan2f(a.y - cy, a.x - cx);
            float ab = atan2f(b.y - cy, b.x - cx);
            if (aa < -1.5708f) aa += 6.2832f;
            if (ab < -1.5708f) ab += 6.2832f;
            return aa < ab;
        });
    std::swap(pts[1], pts[3]);
}

/**
 * 从色块角点计算主轴/副轴方向向量
 * corners 为顺时针排列：左上、右上、右下、左下
 */
static inline void compute_axes(const cv::Point2f corners[4],
                                cv::Point2f& major_dir, cv::Point2f& minor_dir)
{
    cv::Point2f e1 = corners[1] - corners[0];  // 顶边
    cv::Point2f e2 = corners[3] - corners[0];  // 左边
    float L1 = sqrtf(e1.x*e1.x + e1.y*e1.y);
    float L2 = sqrtf(e2.x*e2.x + e2.y*e2.y);
    if (L1 > L2) {
        major_dir = cv::Point2f(e1.x/L1, e1.y/L1);
    } else {
        major_dir = cv::Point2f(e2.x/L2, e2.y/L2);
    }
    minor_dir = cv::Point2f(-major_dir.y, major_dir.x);
    if (minor_dir.y > 0) minor_dir = cv::Point2f(-minor_dir.x, -minor_dir.y);
}

/**
 * 计算四个角点在主/副轴上的投影极值
 * 返回: min_maj, max_maj, min_min, max_min（通过引用参数）
 */
static inline void compute_projected_extents(const cv::Point2f corners[4],
                                              const cv::Point2f& center,
                                              const cv::Point2f& major_dir,
                                              const cv::Point2f& minor_dir,
                                              float& min_maj, float& max_maj,
                                              float& min_min, float& max_min)
{
    min_maj = 1e9f; max_maj = -1e9f;
    min_min = 1e9f; max_min = -1e9f;
    for (int i = 0; i < 4; i++) {
        float dx = corners[i].x - center.x;
        float dy = corners[i].y - center.y;
        float mj = dx * major_dir.x + dy * major_dir.y;
        float mn = dx * minor_dir.x + dy * minor_dir.y;
        if (mj < min_maj) min_maj = mj;
        if (mj > max_maj) max_maj = mj;
        if (mn < min_min) min_min = mn;
        if (mn > max_min) max_min = mn;
    }
}

/**
 * 在 BEV 域从红色色块角点扩展出上方图片区域的角点
 * 输入 IpmRed_pts 为 BEV 域 PCA 输出的色块角点（顺时针）
 * 输出 IpmRec_pts 为扩展后的 BEV 域角点
 * 成功返回 true，色块过小返回 false
 */
static bool expand_roi_bev(const cv::Point2f ipm_red[4],
                            cv::Point2f ipm_rec[4],
                            const float inv_rot[3][3],
                            cv::Point2f raw_rec[4])
{

    cv::Point2f center(
    (ipm_red[0].x + ipm_red[1].x + ipm_red[2].x + ipm_red[3].x) * 0.25f,
    (ipm_red[0].y + ipm_red[1].y + ipm_red[2].y + ipm_red[3].y) * 0.25f);

    cv::Point2f major_dir, minor_dir;
    // 计算主轴/副轴方向向量
    compute_axes(ipm_red, major_dir, minor_dir);
    // 计算色块在主/副轴上的投影极值
    float min_maj, max_maj, min_min, max_min;
    compute_projected_extents(ipm_red, center, major_dir, minor_dir,
                              min_maj, max_maj, min_min, max_min);
    // 计算色块宽度和高度
    float marker_w = max_maj - min_maj;
    float marker_h = max_min - min_min;
    if (marker_w < 1.0f || marker_h < 0.5f) return false;
    // 计算 ROI 扩展范围
    float lr_pad    = fmaxf(MIN_LR,  marker_w * RATIO_LR);
    float gap       = fmaxf(MIN_GAP, marker_h * RATIO_GAP);
    float ext_h     = marker_h * RATIO_HT;
    float roi_start = min_min - gap;
    float lef_maj   = min_maj - lr_pad;
    float rig_maj   = max_maj + lr_pad;

    // ROI 起始端紧贴色块远端
    cv::Point2f pL(center.x + major_dir.x * lef_maj + minor_dir.x * roi_start,
                   center.y + major_dir.y * lef_maj + minor_dir.y * roi_start);
    cv::Point2f pR(center.x + major_dir.x * rig_maj + minor_dir.x * roi_start,
                   center.y + major_dir.y * rig_maj + minor_dir.y * roi_start);

    // 沿远方向(y减小)扩展 ROI 远端
    cv::Point2f ext_dir(-minor_dir.x, -minor_dir.y);
    cv::Point2f pTl(pL.x + ext_dir.x * ext_h, pL.y + ext_dir.y * ext_h);
    cv::Point2f pTr(pR.x + ext_dir.x * ext_h, pR.y + ext_dir.y * ext_h);

    ipm_rec[0] = pTl; ipm_rec[1] = pTr;
    ipm_rec[2] = pR;  ipm_rec[3] = pL;
    // 回投到原图 → RAW_pts
    for (int i = 0; i < 4; i++) {
        float ix, iy;
        w2i(ipm_rec[i].x, ipm_rec[i].y, inv_rot, &ix, &iy);
        raw_rec[i] = cv::Point2f(ix, iy);
    }
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
        return target;
    }
    if (rot == nullptr || inv_rot == nullptr) {
        return target;
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

    cv::Point2f center;
    // BEV PCA
    fit_oriented_rect(bev_points, target.IpmRed_pts, center);

    // BEV 域旋转矩形真实宽高比检查
    //if (!check_bev_aspect(target.IpmRed_pts)) return target;

    // 6. w2i 回投原图 → RAW_pts，再极角排序成顺时针
    for (int i = 0; i < 4; i++) {
        float ix, iy;
        w2i(target.IpmRed_pts[i].x, target.IpmRed_pts[i].y, inv_rot, &ix, &iy);
        target.RAW_pts[i] = cv::Point2f(ix, iy);
    }
    // 极角排序：左上[0]→右上[1]→右下[2]→左下[3] 顺时针
    sort_corners_cw(target.RAW_pts);

    target.found = true;
    return target;
}

/**
 * 在 BEV 域从红色色块角点扩展出上方图片区域的角点，并回投到原图。
 * 前置条件: 必须先成功调用 findRedBoxAndIpm 获得 IpmRed_pts。
 * 返回 true 表示扩展成功，RawRec_pts 已填充。
 */
bool expand_roi_from_red(const DetectionResult &target,
                         const float inv_rot[3][3],
                         cv::Point2f IpmRec_pts_out[4],
                         cv::Point2f RawRec_pts_out[4])
{
    if (!target.found) return false;
    return expand_roi_bev(target.IpmRed_pts, IpmRec_pts_out, inv_rot, RawRec_pts_out);
}

// bool crop_yuyv_to_rgb(const cv::Mat &yuyv,
//                       const cv::Point2f raw_corners[4],
//                       cv::Mat &rgb_out,
//                       cv::Point2f adj_corners[4])
// {
//     float min_x = 320, max_x = 0, min_y = 240, max_y = 0;
//     for (int i = 0; i < 4; i++) {
//         if (raw_corners[i].x < min_x) min_x = raw_corners[i].x;
//         if (raw_corners[i].x > max_x) max_x = raw_corners[i].x;
//         if (raw_corners[i].y < min_y) min_y = raw_corners[i].y;
//         if (raw_corners[i].y > max_y) max_y = raw_corners[i].y;
//     }
//     int crop_x = (int)min_x - 2, crop_y = (int)min_y - 2;
//     int crop_w = (int)max_x - crop_x + 3;
//     int crop_h = (int)max_y - crop_y + 3;
//     if (crop_x < 0) { crop_w += crop_x; crop_x = 0; }
//     if (crop_y < 0) { crop_h += crop_y; crop_y = 0; }
//     if (crop_x + crop_w > 320) crop_w = 320 - crop_x;
//     if (crop_y + crop_h > 240) crop_h = 240 - crop_y;
//     crop_x &= ~1;
//     crop_w &= ~1;
//     if (crop_x + crop_w > 320) crop_w = 320 - crop_x;
//     if (crop_w <= 0 || crop_h <= 0) return false;
//     cv::Mat yuyv_crop = yuyv(cv::Rect(crop_x, crop_y, crop_w, crop_h));
//     cv::cvtColor(yuyv_crop, rgb_out, cv::COLOR_YUV2RGB_YUYV);
//     for (int i = 0; i < 4; i++) {
//         adj_corners[i].x = raw_corners[i].x - crop_x;
//         adj_corners[i].y = raw_corners[i].y - crop_y;
//     }
//     return true;
// }

cv::Mat warp_raw_to_bev(const cv::Mat &frame,
                        const cv::Point2f raw_corners[4],
                        int bev_w, int bev_h)
{

    cv::Point2f bev_dst[4] = {{0,0}, {(float)(bev_w-1),0},
                               {(float)(bev_w-1),(float)(bev_h-1)},
                               {0,(float)(bev_h-1)}};
    cv::Mat M = cv::getPerspectiveTransform(raw_corners, bev_dst);
    cv::Mat bev;
    cv::warpPerspective(frame, bev, M, cv::Size(bev_w, bev_h), cv::INTER_LINEAR); // BEV变换
    return bev;
}

