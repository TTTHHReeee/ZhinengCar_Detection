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

        if (target.found) {
            bool roi_valid = expand_roi_from_red(target, kInvRot,
                                                  target.IpmRec_pts, target.RawRec_pts);
            draw_rotated_quad(display, target.RAW_pts, cv::Scalar(255, 0, 0));
            //设置区域
            for(int i = 10; i < 240; i += 10){
                cv::line(display, cv::Point(0,i), cv::Point(320,i) , cv::Scalar(255, 255, 255), 1);
            }

            cv::line(display, cv::Point(0,75), cv::Point(320,75) , cv::Scalar(0, 0, 0), 1);
    

            //红色种子搜索范围
                        // cv::line(display, cv::Point(35, 20), cv::Point(285, 20), cv::Scalar(0, 0, 0)); //左上到右上
                        // cv::line(display, cv::Point(35, 20), cv::Point(35, 130), cv::Scalar(0, 0, 0)); //左上到左下
                        // cv::line(display, cv::Point(35, 130), cv::Point(285, 130), cv::Scalar(0, 0, 0)); //左下到右下
                        // cv::line(display, cv::Point(285, 20), cv::Point(285, 130), cv::Scalar(0, 0, 0)); //右上到右下


            if (roi_valid)
                draw_rotated_quad(display, target.RawRec_pts, cv::Scalar(0, 255, 0));

            cv::Point2f* crop_src = roi_valid ? target.RawRec_pts : target.RAW_pts;
            cv::Point2f roi_dst[4] = {{0,0}, {(float)(BEV_W-1), 0},
                                       {(float)(BEV_W-1), (float)(BEV_H-1)},
                                       {0, (float)(BEV_H-1)}};
            cv::Mat M_roi = cv::getPerspectiveTransform(crop_src, roi_dst);
            cv::Mat roi_ipm;
            cv::warpPerspective(frame, roi_ipm, M_roi, cv::Size(BEV_W, BEV_H));
            server.update_roi_frame(roi_ipm);
        }
        server.update_frame(display);
        }
    

    server.stop_server();
    viewer_bridge_deinit();

    return 0;
}



