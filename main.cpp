#include "tuchuan.hpp"
#include "viewer_bridge.hpp"
#include "findRedBox.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include <signal.h>
#include <unistd.h>

bool g_running = true;
void signal_handler(int) { g_running = false; }

// BGR cv::Mat → YUYV 字节缓冲区（每行 stride = W*2）
static void bgr_to_yuyv(const cv::Mat &bgr, uint8_t *yuyv, uint32_t stride) {
    int W = bgr.cols, H = bgr.rows;
    for (int y = 0; y < H; ++y) {
        const cv::Vec3b *row = bgr.ptr<cv::Vec3b>(y);
        uint8_t *out = yuyv + y * stride;
        for (int x = 0; x < W; ++x) {
            int b = row[x][0], g = row[x][1], r = row[x][2];
            int pair_off = (x & ~1) * 2;
            out[pair_off + ((x & 1) ? 2 : 0)] = ( 66*r + 129*g +  25*b + 128) >> 8; // Y
            out[pair_off + 1]                    = ((-38*r -  74*g + 112*b + 128) >> 8) + 128; // U
            out[pair_off + 3]                    = ((112*r -  94*g -  18*b + 128) >> 8) + 128; // V
        }
    }
}

// 绘制旋转四边形（顺时针：左上、右上、右下、左下）
static void draw_rotated_quad(cv::Mat &img, const cv::Point2f corners[4], cv::Scalar color) {
    for (int i = 0; i < 4; ++i)
        cv::line(img,
            cv::Point((int)corners[i].x, (int)corners[i].y),
            cv::Point((int)corners[(i+1)&3].x, (int)corners[(i+1)&3].y),
            color, 1);
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);

    const char *viewer_ip   = "192.168.43.64";
    int         viewer_port = 19090;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--viewer-ip") == 0 && i + 1 < argc)
            viewer_ip = argv[++i];
        else if (std::strcmp(argv[i], "--viewer-port") == 0 && i + 1 < argc)
            viewer_port = std::atoi(argv[++i]);
    }

    if (viewer_bridge_init(viewer_ip, viewer_port))
        std::cout << "UDP viewer 桥接已启用 -> " << viewer_ip << ":" << viewer_port << std::endl;
    else
        std::cout << "UDP viewer 初始化失败, 仅使用 HTTP MJPEG" << std::endl;

    printf("网络初始化完成\n");

    cv::VideoCapture cap;
    cap.open(0);
    if (!cap.isOpened()) cap.open("/dev/video0");
    bool has_camera = cap.isOpened();
    if (has_camera) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  320);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
        cap.set(cv::CAP_PROP_FPS, 30);
    }
    printf("相机初始化完成 has_camera=%d\n", (int)has_camera);
    CameraStreamServer server;
    server.start_server(8080);
    const uint32_t yuyv_stride = 640;
    std::vector<uint8_t> yuyv_buf(yuyv_stride * 240);

    while (g_running && server.is_running()) {
        cv::Mat frame;
        if (has_camera) {
            try {
                cap.read(frame);
            } catch (...) {
                has_camera = false;
                fprintf(stderr, "[WARN]相机异常，回退\n");
            }
            if (!frame.empty() && (frame.cols != 320 || frame.rows != 240 || frame.channels() != 3 || frame.step != (size_t)frame.cols * 3)) {
                has_camera = false;
                fprintf(stderr, "[WARN]相机帧损坏，回退\n");
            }
        }
        if (frame.empty()) {
            frame = cv::Mat(240, 320, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(frame, "No Camera", cv::Point(80, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);
        }
        // 1. BGR 转 YUYV
        bgr_to_yuyv(frame, yuyv_buf.data(), yuyv_stride);

        // 2. 完整检测链路
        cv::Mat yuyv_mat(240, 320, CV_8UC2, yuyv_buf.data());
        server.update_yuyv_frame(yuyv_mat);
        DetectionResult target;
        target = findRedBoxAndIpm(yuyv_mat, kRot, kInvRot, target);

        cv::Mat display = frame.clone();

        // 3. 画轴对齐 bbox（绿）+ BEV PCA 旋转角点（蓝） + 原图 BFS 点直接 PCA（黄）
        if (target.found) {
            // cv::rectangle(display,
            //     cv::Rect(target.bbox.x, target.bbox.y,
            //              target.bbox.width, target.bbox.height),
            //     cv::Scalar(0, 255, 0), 1);
            draw_rotated_quad(display, target.RAW_pts, cv::Scalar(255, 0, 0));
            //原图PCA结果框定
            //draw_rotated_quad(display, target.raw_pca_pts, cv::Scalar(0, 255, 255));
            draw_rotated_quad(display, target.RawRec_pts, cv::Scalar(0, 255, 0));
            printf("RawRec_pts: (%.0f,%.0f)-(%.0f,%.0f)-(%.0f,%.0f)-(%.0f,%.0f)\n",
                   target.RawRec_pts[0].x, target.RawRec_pts[0].y,
                   target.RawRec_pts[1].x, target.RawRec_pts[1].y,
                   target.RawRec_pts[2].x, target.RawRec_pts[2].y,
                   target.RawRec_pts[3].x, target.RawRec_pts[3].y);
            printf("bbox: %d, %d, %d, %d\n", target.bbox.x, target.bbox.y, target.bbox.width, target.bbox.height);

            // 4. ROI 裁剪 (原图 BGR) → 50×40, 使用 RawRec_pts(图像坐标)
            cv::Point2f roi_src[4] = {target.RawRec_pts[0], target.RawRec_pts[1],target.RawRec_pts[2], target.RawRec_pts[3]};
            cv::Point2f roi_dst[4] = {{0,0},{49,0},{49,39},{0,39}};
            cv::Mat M_roi = cv::getPerspectiveTransform(roi_src, roi_dst);

            cv::Mat roi_crop;
            cv::warpPerspective(frame, roi_crop, M_roi, cv::Size(50, 40));

            cv::Mat roi_ipm;
            cv::warpPerspective(frame, roi_ipm, M_roi, cv::Size(50, 40));

            // 5. 右侧面板: 原始尺寸 ROI, 黑色画布补齐到 240 行
            int rw = std::max(roi_ipm.cols, roi_crop.cols);
            cv::Mat right(240, rw > 0 ? rw : 50, CV_8UC3, cv::Scalar(0, 0, 0));
            roi_ipm.copyTo(right(cv::Rect(0, 0,  roi_ipm.cols, roi_ipm.rows)));
            roi_crop.copyTo(right(cv::Rect(0, 45, roi_crop.cols, roi_crop.rows)));
            cv::Rect keep(75, 20, 211, 116);  // x:75→285, y:20→135
            cv::Mat mask(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));  // 全黑画布
            display(keep).copyTo(mask(keep));   // 只拷贝 keep 区域
            display = mask;
            cv::hconcat(display, right, display);
            // 显示结果只显示左上（75，20）右上（285，20）右下（285，135）左下（75，135）四个点包围的区域，其他区域为黑色
          
        }
        server.update_frame(display);
        }

    viewer_bridge_deinit();

    return 0;

}



