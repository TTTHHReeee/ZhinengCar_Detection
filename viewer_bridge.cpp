#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "viewer_bridge.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// ─── 协议常量（与上位机 1.py 保持一致） ───
#define VIEWER_MAGIC             0x5646534C  // "LSFV"
#define VIEWER_FORMAT_MJPG       1
#define VIEWER_FORMAT_GRAY8      3
#define VIEWER_MAX_CHUNKS        128
#define VIEWER_FLAG_BEGIN        0x01
#define VIEWER_FLAG_END          0x02
#define VIEWER_CHUNK_SIZE_MIN    900
#define VIEWER_CHUNK_SIZE_MAX    1450

// ─── 28 字节包头（与上位机 struct.Struct("<IIIIHHHHHBBI") 匹配） ───
#pragma pack(push, 1)
struct viewer_packet_header
{
    uint32_t magic;
    uint32_t frame_id;
    uint32_t frame_size;
    uint32_t chunk_offset;
    uint16_t chunk_size;
    uint16_t chunk_index;
    uint16_t chunk_count;
    uint16_t width;
    uint16_t height;
    uint8_t  format;
    uint8_t  flags;
    uint32_t timestamp_ms;
};
#pragma pack(pop)

// ─── 全局状态 ───
static int                  g_udp_sock = -1;
static size_t               g_chunk_size = VIEWER_BRIDGE_CHUNK_SIZE_DEFAULT;

// ─── sendmmsg 发送上下文(线程局部) ───
struct send_context
{
    bool     initialized = false;
    bool     layout_valid = false;
    uint8_t  layout_format = 0;
    uint32_t layout_width = 0;
    uint32_t layout_height = 0;
    uint32_t layout_payload_size = 0;
    size_t   layout_chunk_size_bytes = 0;
    size_t   layout_chunk_count = 0;

    std::array<viewer_packet_header, VIEWER_MAX_CHUNKS>     headers{};
    std::array<struct iovec,   VIEWER_MAX_CHUNKS * 2>       iovecs{};
    std::array<struct mmsghdr, VIEWER_MAX_CHUNKS>           messages{};
    std::array<uint32_t,       VIEWER_MAX_CHUNKS>           chunk_offsets{};

    void ensure_initialized()
    {
        if (initialized) return;

        for (size_t i = 0; i < VIEWER_MAX_CHUNKS; ++i)
        {
            iovecs[i * 2].iov_base = &headers[i];
            iovecs[i * 2].iov_len  = sizeof(viewer_packet_header);
            messages[i].msg_hdr.msg_iov    = &iovecs[i * 2];
            messages[i].msg_hdr.msg_iovlen = 2;
        }
        initialized = true;
    }

    void ensure_payload_layout(
        uint8_t format, uint32_t width, uint32_t height,
        size_t payload_size, size_t chunk_size_bytes, size_t chunk_count)
    {
        if (layout_valid &&
            layout_format == format &&
            layout_width == width &&
            layout_height == height &&
            layout_payload_size == payload_size &&
            layout_chunk_size_bytes == chunk_size_bytes &&
            layout_chunk_count == chunk_count)
        {
            return;
        }

        layout_valid             = true;
        layout_format            = format;
        layout_width             = width;
        layout_height            = height;
        layout_payload_size      = static_cast<uint32_t>(payload_size);
        layout_chunk_size_bytes  = chunk_size_bytes;
        layout_chunk_count       = chunk_count;

        for (size_t i = 0; i < chunk_count; ++i)
        {
            const size_t offset     = i * chunk_size_bytes;
            const size_t remain     = payload_size - offset;
            const uint16_t frag_sz  = static_cast<uint16_t>(
                std::min(remain, chunk_size_bytes));

            chunk_offsets[i] = static_cast<uint32_t>(offset);

            viewer_packet_header &hdr = headers[i];
            hdr.magic          = VIEWER_MAGIC;
            hdr.frame_id       = 0;
            hdr.frame_size     = static_cast<uint32_t>(payload_size);
            hdr.chunk_offset   = static_cast<uint32_t>(offset);
            hdr.chunk_size     = frag_sz;
            hdr.chunk_index    = static_cast<uint16_t>(i);
            hdr.chunk_count    = static_cast<uint16_t>(chunk_count);
            hdr.width          = static_cast<uint16_t>(width);
            hdr.height         = static_cast<uint16_t>(height);
            hdr.format         = format;
            hdr.flags          = 0;
            if (i == 0)                       hdr.flags |= VIEWER_FLAG_BEGIN;
            if (i + 1 == chunk_count)         hdr.flags |= VIEWER_FLAG_END;
            hdr.timestamp_ms   = 0;

            iovecs[i * 2 + 1].iov_len = frag_sz;
        }
    }
};

// ─── 单调时钟毫秒 (与上位机 timestamp_ms 对应) ───
static uint32_t monotonic_ms(void)
{
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count() & 0xffffffffULL);
}

// ═══════════════════════════════════════════════════════════════
// 公开接口
// ═══════════════════════════════════════════════════════════════

bool viewer_bridge_init(const char *viewer_ip, int viewer_port, size_t chunk_size)
{
    viewer_bridge_deinit();

    if (!viewer_ip || viewer_ip[0] == '\0' || viewer_port <= 0)
        return false;

    g_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_sock < 0)
        return false;

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(viewer_port));
    if (inet_pton(AF_INET, viewer_ip, &addr.sin_addr) != 1)
    {
        close(g_udp_sock);
        g_udp_sock = -1;
        return false;
    }

    // connect UDP socket 以便使用 sendmmsg
    if (connect(g_udp_sock, reinterpret_cast<struct sockaddr *>(&addr),
                sizeof(addr)) < 0)
    {
        close(g_udp_sock);
        g_udp_sock = -1;
        return false;
    }

    g_chunk_size = std::max<size_t>(VIEWER_CHUNK_SIZE_MIN,
                     std::min<size_t>(chunk_size, VIEWER_CHUNK_SIZE_MAX));

    return true;
}

void viewer_bridge_deinit(void)
{
    if (g_udp_sock >= 0)
    {
        close(g_udp_sock);
        g_udp_sock = -1;
    }
}

bool viewer_bridge_is_enabled(void)
{
    return g_udp_sock >= 0;
}

bool viewer_bridge_send_mjpg_frame(
    uint32_t frame_id, uint32_t width, uint32_t height,
    const uint8_t *jpeg_data, size_t jpeg_size,
    uint32_t &packet_count)
{
    packet_count = 0;

    if (!jpeg_data || jpeg_size == 0 || g_udp_sock < 0)
        return false;

    const size_t chunk_count =
        (jpeg_size + g_chunk_size - 1) / g_chunk_size;
    if (chunk_count == 0 || chunk_count > VIEWER_MAX_CHUNKS)
        return false;

    // ── 准备发送上下文(线程局部, 减少重复分配) ──
    static thread_local send_context ctx{};
    ctx.ensure_initialized();
    ctx.ensure_payload_layout(
        VIEWER_FORMAT_MJPG, width, height,
        jpeg_size, g_chunk_size, chunk_count);

    const uint32_t ts = monotonic_ms();

    for (size_t i = 0; i < chunk_count; ++i)
    {
        viewer_packet_header &hdr = ctx.headers[i];
        hdr.frame_id     = frame_id;
        hdr.timestamp_ms = ts;

        struct iovec &p_iov = ctx.iovecs[i * 2 + 1];
        p_iov.iov_base = const_cast<uint8_t *>(jpeg_data + ctx.chunk_offsets[i]);

        ctx.messages[i].msg_len = 0;
    }

    const int sent = sendmmsg(
        g_udp_sock, ctx.messages.data(),
        static_cast<unsigned int>(chunk_count), MSG_DONTWAIT);

    if (sent < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            return false;
        return false;
    }

    if (static_cast<size_t>(sent) != chunk_count)
        return false;

    packet_count = static_cast<uint32_t>(chunk_count);
    return true;
}

bool viewer_bridge_send_gray8_frame(
    uint32_t frame_id, uint32_t width, uint32_t height,
    const uint8_t *gray_data, size_t gray_size,
    uint32_t &packet_count)
{
    packet_count = 0;

    if (!gray_data || gray_size == 0 || g_udp_sock < 0)
        return false;

    const size_t chunk_count =
        (gray_size + g_chunk_size - 1) / g_chunk_size;
    if (chunk_count == 0 || chunk_count > VIEWER_MAX_CHUNKS)
        return false;

    // ── 准备发送上下文(线程局部) ──
    static thread_local send_context ctx_gray{};
    ctx_gray.ensure_initialized();
    ctx_gray.ensure_payload_layout(
        VIEWER_FORMAT_GRAY8, width, height,
        gray_size, g_chunk_size, chunk_count);

    const uint32_t ts = monotonic_ms();

    for (size_t i = 0; i < chunk_count; ++i)
    {
        viewer_packet_header &hdr = ctx_gray.headers[i];
        hdr.frame_id     = frame_id;
        hdr.timestamp_ms = ts;

        struct iovec &p_iov = ctx_gray.iovecs[i * 2 + 1];
        p_iov.iov_base = const_cast<uint8_t *>(gray_data + ctx_gray.chunk_offsets[i]);

        ctx_gray.messages[i].msg_len = 0;
    }

    const int sent = sendmmsg(
        g_udp_sock, ctx_gray.messages.data(),
        static_cast<unsigned int>(chunk_count), MSG_DONTWAIT);

    if (sent < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            return false;
        return false;
    }

    if (static_cast<size_t>(sent) != chunk_count)
        return false;

    packet_count = static_cast<uint32_t>(chunk_count);
    return true;
}
