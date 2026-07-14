/**
 * @file    zf_device_yuyv.cpp
 * @brief   ZfDeviceYUYV 类的实现
 * @author  逐飞科技 & ChatGPT
 * @date    2026-02-13
 */

#include "zf_device_yuyv.hpp"

/* ==================== 构造函数 ==================== */

ZfDeviceYUYV::ZfDeviceYUYV()
    : fd_(-1)
    , width_(0)
    , height_(0)
    , fps_(30)
    , streaming_(false)
    , yuyv_data_(nullptr)
    , current_buffer_index_(-1)
    , gray_dirty_(true)
    , rgb565_dirty_(true)
    , bgr_dirty_(true)
{
    // 所有成员已在初始化列表中置初值
}

/* ==================== 析构函数 ==================== */

ZfDeviceYUYV::~ZfDeviceYUYV()
{
    release();  // 确保资源释放
}

/* ==================== 内部辅助函数 ==================== */

bool ZfDeviceYUYV::xioctl(int request, void* arg)
{
    int r;
    do {
        r = ioctl(fd_, request, arg);   // 执行 ioctl
    } while (r == -1 && errno == EINTR); // 如果被信号中断则重试
    return r != -1;
}

/* ==================== 初始化 ==================== */

bool ZfDeviceYUYV::init(const char* path, int width, int height, int fps)
{
    // 1. 打开摄像头设备
    fd_ = open(path, O_RDWR);
    if (fd_ == -1) {
        perror("open video device");
        return false;
    }

    // 2. 查询设备能力
    struct v4l2_capability cap;
    if (!xioctl(VIDIOC_QUERYCAP, &cap)) {
        perror("VIDIOC_QUERYCAP");
        close(fd_); fd_ = -1;
        return false;
    }
    // 检查是否支持视频捕获
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "Error: device does not support video capture.\n");
        close(fd_); fd_ = -1;
        return false;
    }
    // 检查是否支持流式 I/O
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "Error: device does not support streaming I/O.\n");
        close(fd_); fd_ = -1;
        return false;
    }

    // 3. 设置图像格式为 YUYV
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;  // 请求 YUYV 格式
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;    // 逐行扫描

    if (!xioctl(VIDIOC_S_FMT, &fmt)) {
        perror("VIDIOC_S_FMT");
        close(fd_); fd_ = -1;
        return false;
    }

    // 保存实际设置的分辨率（可能与请求的不同）
    width_  = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    printf("[YUYV] Camera initialized: %dx%d\n", width_, height_);

    // 4. 设置帧率（可选，部分摄像头不支持）
    struct v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    if (!xioctl(VIDIOC_S_PARM, &parm)) {
        // 帧率设置失败不致命，继续运行
        perror("VIDIOC_S_PARM (non-fatal)");
    }

    // 5. 申请并映射 MMAP 缓冲区
    if (!init_mmap()) {
        release();
        return false;
    }

    // 6. 开始视频流
    if (!start_stream()) {
        release();
        return false;
    }

    streaming_ = true;
    return true;
}

/* ==================== MMAP 缓冲区初始化 ==================== */

bool ZfDeviceYUYV::init_mmap()
{
    // 请求 4 个缓冲区
    struct v4l2_requestbuffers req = {};
    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (!xioctl(VIDIOC_REQBUFS, &req)) {
        perror("VIDIOC_REQBUFS");
        return false;
    }

    // 分配缓冲区容器
    buffers_.resize(req.count);

    for (unsigned int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        // 查询缓冲区信息（获取偏移量和长度）
        if (!xioctl(VIDIOC_QUERYBUF, &buf)) {
            perror("VIDIOC_QUERYBUF");
            return false;
        }

        // 映射内核缓冲区到用户空间
        buffers_[i].length = buf.length;
        buffers_[i].start  = mmap(NULL, buf.length,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED,
                                  fd_, buf.m.offset);
        if (buffers_[i].start == MAP_FAILED) {
            perror("mmap");
            return false;
        }
    }
    return true;
}

/* ==================== 流控制 ==================== */

bool ZfDeviceYUYV::start_stream()
{
    // 将所有缓冲区入队
    for (unsigned int i = 0; i < buffers_.size(); ++i) {
        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (!xioctl(VIDIOC_QBUF, &buf)) {
            perror("VIDIOC_QBUF");
            return false;
        }
    }

    // 开始流传输
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!xioctl(VIDIOC_STREAMON, &type)) {
        perror("VIDIOC_STREAMON");
        return false;
    }
    return true;
}

bool ZfDeviceYUYV::stop_stream()
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!xioctl(VIDIOC_STREAMOFF, &type)) {
        perror("VIDIOC_STREAMOFF");
        return false;
    }
    return true;
}

/* ==================== 获取帧 ==================== */

const unsigned char* ZfDeviceYUYV::wait_frame()
{
    if (!streaming_) return nullptr;

    struct v4l2_buffer buf = {};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    // 出队一个已填充好的缓冲区（阻塞直到有数据）
    if (!xioctl(VIDIOC_DQBUF, &buf)) {
        perror("VIDIOC_DQBUF");
        return nullptr;
    }

    current_buffer_index_ = buf.index;
    yuyv_data_ = static_cast<const unsigned char*>(buffers_[buf.index].start);

    // 标记所有派生图像为“脏”，下次访问时需要重新生成
    gray_dirty_   = true;
    rgb565_dirty_ = true;
    bgr_dirty_    = true;

    // 立即将该缓冲区重新入队，以便摄像头继续使用
    if (!xioctl(VIDIOC_QBUF, &buf)) {
        perror("VIDIOC_QBUF (re-queue)");
        // 即使失败，仍返回当前帧数据，但后续可能丢帧
    }

    return yuyv_data_;
}

/* ==================== 释放资源 ==================== */

void ZfDeviceYUYV::release()
{
    // 停止流
    if (streaming_) {
        stop_stream();
        streaming_ = false;
    }

    // 解除所有 MMAP 映射
    for (auto& buf : buffers_) {
        if (buf.start != MAP_FAILED) {
            munmap(buf.start, buf.length);
            buf.start = MAP_FAILED;
        }
    }
    buffers_.clear();

    // 关闭设备文件
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }

    // 释放 OpenCV Mat 占用的内存
    gray_mat_.release();
    rgb565_mat_.release();
    bgr_mat_.release();
    yuyv_data_ = nullptr;

    // 重置状态
    gray_dirty_   = true;
    rgb565_dirty_ = true;
    bgr_dirty_    = true;
    current_buffer_index_ = -1;
}

/* ==================== 图像生成函数 ==================== */

void ZfDeviceYUYV::generate_gray()
{
    if (!yuyv_data_) return;

    // 创建单通道灰度图
    gray_mat_ = cv::Mat(height_, width_, CV_8UC1);

    // 遍历每一行，从 YUYV 中提取 Y 分量
    // YUYV 布局: [Y0][U0][Y1][V0] [Y2][U2][Y3][V2] ...
    // Y 值位于偶数偏移位置（0, 2, 4, ...）
    for (int row = 0; row < height_; ++row) {
        const unsigned char* src = yuyv_data_ + row * width_ * 2;  // 每行宽度 * 2 字节
        unsigned char* dst       = gray_mat_.ptr<unsigned char>(row);
        for (int col = 0; col < width_; ++col) {
            dst[col] = src[col * 2];   // 取 Y 值
        }
    }
    gray_dirty_ = false;
}

void ZfDeviceYUYV::generate_rgb565()
{
    if (!yuyv_data_) return;

    // 先转为 BGR（中间临时 Mat）
    cv::Mat bgr_temp;
    cv::Mat yuyv_mat(height_, width_, CV_8UC2, const_cast<unsigned char*>(yuyv_data_));
    cv::cvtColor(yuyv_mat, bgr_temp, cv::COLOR_YUV2BGR_YUYV);

    // 再从 BGR 转为 BGR565（RGB565）
    cv::cvtColor(bgr_temp, rgb565_mat_, cv::COLOR_BGR2BGR565);
    rgb565_dirty_ = false;
}

void ZfDeviceYUYV::generate_bgr()
{
    if (!yuyv_data_) return;

    // YUYV → BGR
    cv::Mat yuyv_mat(height_, width_, CV_8UC2, const_cast<unsigned char*>(yuyv_data_));
    cv::cvtColor(yuyv_mat, bgr_mat_, cv::COLOR_YUV2BGR_YUYV);
    bgr_dirty_ = false;
}

/* ==================== 公开接口 ==================== */

uint8_t* ZfDeviceYUYV::get_gray_image_ptr()
{
    if (gray_dirty_) generate_gray();
    return gray_mat_.data;   // 返回灰度图数据指针
}

uint16_t* ZfDeviceYUYV::get_rgb_image_ptr()
{
    if (rgb565_dirty_) generate_rgb565();
    return reinterpret_cast<uint16_t*>(rgb565_mat_.data);  // 返回 RGB565 数据指针
}

cv::Mat ZfDeviceYUYV::get_bgr_mat()
{
    if (bgr_dirty_) generate_bgr();
    return bgr_mat_.clone();  // 返回深拷贝副本，避免外部修改内部缓存
}