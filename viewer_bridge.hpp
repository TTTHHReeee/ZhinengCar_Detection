#ifndef _VIEWER_BRIDGE_HPP_
#define _VIEWER_BRIDGE_HPP_

#include <cstddef>
#include <cstdint>

#define VIEWER_BRIDGE_CHUNK_SIZE_DEFAULT 1400

/**
 * 初始化 UDP viewer 桥接链路
 * @param viewer_ip   上位机 IP 地址
 * @param viewer_port 上位机 UDP 监听端口
 * @param chunk_size  每包负载字节数 (默认 1400, 范围 900~1450)
 * @return true 成功, false 失败
 */
bool viewer_bridge_init(const char *viewer_ip, int viewer_port,
                        size_t chunk_size = VIEWER_BRIDGE_CHUNK_SIZE_DEFAULT);

/**
 * 释放 UDP viewer 桥接链路资源
 */
void viewer_bridge_deinit(void);

/**
 * 查询桥接链路是否可用
 */
bool viewer_bridge_is_enabled(void);

/**
 * 发送一帧 MJPG 图像到上位机 viewer
 * @param frame_id     帧号
 * @param width        图像宽度
 * @param height       图像高度
 * @param jpeg_data    JPEG 数据指针
 * @param jpeg_size    JPEG 数据长度
 * @param packet_count 输出: 实际发送的分片数
 * @return true 发送成功, false 失败
 */
bool viewer_bridge_send_mjpg_frame(uint32_t frame_id, uint32_t width,
                                   uint32_t height, const uint8_t *jpeg_data,
                                   size_t jpeg_size,
                                   uint32_t &packet_count);

/**
 * 发送一帧 Gray8 原始灰度图到上位机 viewer
 * @param frame_id     帧号
 * @param width        图像宽度
 * @param height       图像高度
 * @param gray_data    灰度数据指针 (每像素1字节)
 * @param gray_size    数据长度 (= width * height)
 * @param packet_count 输出: 实际发送的分片数
 * @return true 发送成功, false 失败
 */
bool viewer_bridge_send_gray8_frame(uint32_t frame_id, uint32_t width,
                                    uint32_t height, const uint8_t *gray_data,
                                    size_t gray_size,
                                    uint32_t &packet_count);

#endif
