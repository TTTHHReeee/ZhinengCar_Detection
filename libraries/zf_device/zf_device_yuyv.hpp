/**
 * @file    zf_device_yuyv.hpp
 * @brief   V4L2 YUYV 摄像头驱动类，支持获取原始 YUYV、灰度图、RGB565、BGR 图像
 * @author  逐飞科技 & ChatGPT
 * @date    2026-02-13
 *
 * 本类直接通过 V4L2 接口操作摄像头，获取原始 YUYV 格式数据，
 * 并提供零拷贝的 Y 分量提取（灰度图）以及可选的 RGB565 / BGR 转换。
 * 适用于需要高效色块识别（直接操作 UV 分量）和 ncnn 推理的场景。
 */

#ifndef ZF_DEVICE_YUYV_HPP
#define ZF_DEVICE_YUYV_HPP

#include <cstdint>      // uint8_t, uint16_t
#include <cstring>      // memset
#include <fcntl.h>      // open
#include <unistd.h>     // close, read, write
#include <sys/ioctl.h>  // ioctl
#include <sys/mman.h>   // mmap, munmap
#include <linux/videodev2.h>  // V4L2 结构体定义
#include <cerrno>       // errno
#include <cstdio>       // printf, perror
#include <vector>       // std::vector
#include <opencv2/opencv.hpp>  // cv::Mat, cv::cvtColor 等



#define UVC_WIDTH   320
#define UVC_HEIGHT  240
/**
 * @class ZfDeviceYUYV
 * @brief V4L2 YUYV 摄像头驱动封装类
 *
 * 使用 V4L2 的 MMAP 模式采集 YUYV 格式图像。
 * 内部维护原始 YUYV 缓冲区，并提供以下派生图像：
 * - 灰度图（直接从 Y 分量提取，零拷贝）
 * - RGB565（用于 LCD 直驱）
 * - BGR Mat（用于 OpenCV 处理或 ncnn 推理）
 *
 * 所有派生图像均采用惰性生成策略（dirty flag），
 * 仅在首次访问时转换，同一帧内多次访问不会重复转换。
 */
class ZfDeviceYUYV {
public:
    /**
     * @brief 构造函数，初始化成员变量
     */
    ZfDeviceYUYV();

    /**
     * @brief 析构函数，自动释放摄像头资源
     */
    ~ZfDeviceYUYV();

    /* ==================== 初始化与控制 ==================== */

    /**
     * @brief 初始化摄像头并启动流
     * @param path  摄像头设备节点路径，例如 "/dev/video0"
     * @param width 期望的图像宽度（实际可能略有不同）
     * @param height 期望的图像高度
     * @param fps   期望的帧率（部分摄像头不支持自定义帧率，忽略失败）
     * @return true  初始化成功
     * @return false 初始化失败（设备不存在、格式不支持等）
     *
     * 注意：本函数会打开设备、设置 YUYV 格式、申请 MMAP 缓冲区并开始流。
     *       调用后摄像头将持续采集，直到调用 release() 或析构。
     */
    bool init(const char* path, int width, int height, int fps = 30);

    /**
     * @brief 等待并获取一帧原始 YUYV 数据
     * @return const unsigned char* 指向最新帧的 YUYV 数据指针
     *         （长度为 width * height * 2 字节），失败返回 nullptr
     *
     * 每次调用会阻塞直到新帧到达，并更新内部 YUYV 数据指针。
     * 同时会将灰度图、RGB565、BGR 的 dirty 标志置为 true，
     * 下次访问这些派生图像时会重新生成。
     *
     * 注意：返回的指针指向内核空间的 MMAP 缓冲区，在下次 wait_frame()
     *       调用前有效。如果需要长期保存，请拷贝数据。
     */
    const unsigned char* wait_frame();

    /**
     * @brief 释放摄像头资源（停止流、解除映射、关闭设备）
     *
     * 可以在程序结束前显式调用，析构函数也会自动调用。
     * 释放后可以再次调用 init() 重新初始化。
     */
    void release();

    /* ==================== 图像获取接口 ==================== */

    /**
     * @brief 获取原始 YUYV 数据指针（最近一次 wait_frame 的结果）
     * @return const unsigned char* YUYV 数据指针，未调用 wait_frame 时返回 nullptr
     *
     * 与 wait_frame() 返回的指针相同，但不会阻塞。
     */
    const unsigned char* get_yuyv_data() const { return yuyv_data_; }

    /**
     * @brief 获取灰度图像数据指针（从 YUYV 中提取 Y 分量）
     * @return uint8_t* 灰度图首地址，大小为 width * height 字节
     *
     * 内部实现：直接从 YUYV 缓冲区每隔一个字节取 Y 值，
     * 拼成连续的单通道灰度图。速度极快（~0.2ms @640x480）。
     *
     * 注意：返回的指针指向类内部缓存的 cv::Mat，
     *       在下一次 wait_frame() 后失效。
     */
    uint8_t* get_gray_image_ptr();

    /**
     * @brief 获取 RGB565 图像数据指针
     * @return uint16_t* RGB565 图像首地址，大小为 width * height * 2 字节
     *
     * 内部实现：先将 YUYV 转为 BGR，再转为 BGR565（RGB565）。
     * 速度较慢（~2-3ms @640x480），建议仅在需要显示到 LCD 时调用。
     *
     * 注意：返回的指针指向类内部缓存的 cv::Mat，
     *       在下一次 wait_frame() 后失效。
     */
    uint16_t* get_rgb_image_ptr();

    /**
     * @brief 获取 BGR 格式的 cv::Mat（深拷贝副本）
     * @return cv::Mat BGR 三通道图像，数据类型 CV_8UC3
     *
     * 内部实现：YUYV → BGR 转换，返回副本以避免外部修改内部缓存。
     * 适用于需要 OpenCV 进一步处理（如裁剪 ROI 送 ncnn）的场景。
     *
     * 注意：每次调用都会进行一次完整的 YUYV→BGR 转换（~1-2ms），
     *       同一帧内多次调用会重复转换。如果只需一次，请保存返回值。
     */
    cv::Mat get_bgr_mat();

    /* ==================== 属性查询 ==================== */

    /**
     * @brief 获取图像宽度
     * @return int 实际宽度（可能与初始化时请求的不同）
     */
    int get_width() const { return width_; }

    /**
     * @brief 获取图像高度
     * @return int 实际高度
     */
    int get_height() const { return height_; }

private:
    /* ==================== 私有成员变量 ==================== */

    int fd_;                     ///< V4L2 设备文件描述符，-1 表示未打开
    int width_;                  ///< 实际图像宽度
    int height_;                 ///< 实际图像高度
    int fps_;                    ///< 帧率（仅记录，实际由驱动决定）
    bool streaming_;             ///< 流是否正在运行

    const unsigned char* yuyv_data_;   ///< 指向最新 YUYV 帧的指针（MMAP 缓冲区）

    /* ---- 派生图像缓存 ---- */
    cv::Mat gray_mat_;           ///< 灰度图缓存 (CV_8UC1)
    cv::Mat rgb565_mat_;         ///< RGB565 图缓存 (CV_8UC2)
    cv::Mat bgr_mat_;            ///< BGR 图缓存 (CV_8UC3)
    bool gray_dirty_;            ///< 灰度图是否需要重新生成
    bool rgb565_dirty_;          ///< RGB565 是否需要重新生成
    bool bgr_dirty_;             ///< BGR 是否需要重新生成

    /* ---- V4L2 MMAP 缓冲区 ---- */
    struct Buffer {
        void* start;             ///< 映射后的起始地址
        size_t length;           ///< 缓冲区长度
    };
    std::vector<Buffer> buffers_;   ///< MMAP 缓冲区数组
    int current_buffer_index_;      ///< 当前出队的缓冲区索引

    /* ==================== 私有辅助函数 ==================== */

    /**
     * @brief 封装 ioctl 系统调用，自动处理 EINTR 中断
     * @param request V4L2 请求码
     * @param arg     请求参数指针
     * @return true  调用成功
     * @return false 调用失败（errno 不为 EINTR）
     */
    bool xioctl(int request, void* arg);

    /**
     * @brief 申请并映射 V4L2 缓冲区（MMAP 模式）
     * @return true  成功
     * @return false 失败
     */
    bool init_mmap();

    /**
     * @brief 开始视频流
     * @return true  成功
     * @return false 失败
     */
    bool start_stream();

    /**
     * @brief 停止视频流
     * @return true  成功
     * @return false 失败
     */
    bool stop_stream();

    /* ---- 图像生成函数（惰性调用） ---- */
    void generate_gray();        ///< 从 YUYV 提取 Y 分量生成灰度图
    void generate_rgb565();      ///< YUYV → BGR → RGB565
    void generate_bgr();         ///< YUYV → BGR
};

#endif // ZF_DEVICE_YUYV_HPP