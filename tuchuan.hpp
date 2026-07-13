#ifndef __TUCHUAN_HPP__
#define __TUCHUAN_HPP__

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <string>
#include <vector>

// 默认端口号
#define CAMERA_STREAM_DEFAULT_PORT 8080

class CameraStreamServer {
public:
  CameraStreamServer(void);
  ~CameraStreamServer(void);

  /*******************************************************************
   * @brief       启动摄像头图传服务器
   * @param       port            服务器监听端口(默认8080)
   * @return      返回启动状态
   * @retval      0               启动成功
   * @retval      -1              启动失败
   * @note        在后台线程中启动HTTP服务器，支持浏览器访问
   * 访问 http://<开发板IP>:<port> 即可查看实时画面
   ******************************************************************/
  int start_server(int port = CAMERA_STREAM_DEFAULT_PORT);

  /*******************************************************************
   * @brief       更新摄像头帧数据
   * @param       frame           OpenCV Mat格式的图像帧
   * @note        将最新的摄像头帧推送到服务器，供客户端获取
   * 自动编码为JPEG格式并计算帧率
   ******************************************************************/
  void update_frame(const cv::Mat &frame);

  void update_yuyv_frame(const cv::Mat &yuyv);

  /*******************************************************************
   * @brief       停止摄像头图传服务器
   * @note        停止服务器并释放所有资源
   ******************************************************************/
  void stop_server(void);

  /*******************************************************************
   * @brief       检查服务器是否正在运行
   * @return      返回服务器运行状态
   * @retval      true            服务器正在运行
   * @retval      false           服务器已停止
   ******************************************************************/
  bool is_running(void);

private:
  // 服务器socket文件描述符
  int server_sock_fd;
  // 服务器端口
  int server_port;
  // 服务器运行状态
  bool running;
  // 服务器线程ID
  pthread_t server_thread_id;

  // 帧数据互斥锁
  pthread_mutex_t frame_mutex;
  // 帧数据条件变量
  pthread_cond_t frame_cond;
  // socket互斥锁
  pthread_mutex_t sock_mutex;

  // 当前JPEG数据
  std::vector<unsigned char> current_jpeg;
  // 最新帧ID
  uint64_t latest_frame_id;
  // 最新捕获时间戳(毫秒)
  uint64_t latest_capture_ts_ms;
  // EMA帧率
  double ema_fps;

  // 原始帧数据（用于保存高质量图片/视频）
  cv::Mat original_frame;
  // 原始帧互斥锁
  pthread_mutex_t original_frame_mutex;
  
  // 视频录制相关
  bool is_recording;
  cv::VideoWriter video_writer;
  std::string video_filename;
  pthread_mutex_t recording_mutex;
  int frame_width;
  int frame_height;
  double frame_rate;
  bool use_jpeg_fallback;
  std::string jpeg_dir;
  std::string jpeg_base_name;
  int jpeg_frame_count;

  pthread_mutex_t yuyv_mutex;
  pthread_cond_t yuyv_cond;
  std::vector<uint8_t> yuyv_frame;
  int yuyv_w;
  int yuyv_h;
  uint64_t yuyv_frame_id;

  // 内部方法
  std::string get_local_ip(void);
  void close_server_socket(void);
  uint64_t now_ms(void);
  std::string format_timestamp(uint64_t ts_ms);
  void send_response(int sock, const char *content_type, const char *body,
                     size_t body_len);
  void send_stats_response(int sock);
  void send_mjpeg_stream(int sock);
  void send_debug_mjpeg_stream(int sock);
  void handle_client_request(int sock);
  void handle_snapshot_request(int sock, const std::string &prefix);
  void start_recording(const std::string &prefix);
  void stop_recording(void);
  void record_frame(const cv::Mat &frame);

  // 静态线程函数
  static void *server_thread_func(void *arg);
  static void *client_thread_func(void *arg);
  static void signal_handler(int sig);

  // 全局实例指针(用于信号处理)
  static CameraStreamServer *instance;
};

#endif // !__TUCHUAN_HPP__