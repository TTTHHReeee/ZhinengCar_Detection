#include "tuchuan.hpp" // 修改为 .hpp
/* 第三方库 */
#include <opencv2/opencv.hpp>

/* 系统库 */
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <ifaddrs.h>
#include <iomanip>
#include <iostream>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/videodev2.h>
#include <mutex>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

// 静态成员初始化
CameraStreamServer *CameraStreamServer::instance = nullptr;

// HTML查看器内容
const char *viewer_html = R"HTML(
<!DOCTYPE html> 
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>摄像头实时查看</title>
    <style>
        body { margin: 0; padding: 20px; background: #1a1a1a; font-family: Arial, sans-serif; }
        .container { max-width: 1200px; margin: 0 auto; background: #2d2d2d; border-radius: 10px; padding: 20px; box-shadow: 0 5px 20px rgba(0,0,0,0.5); }
        h1 { color: #fff; text-align: center; margin-bottom: 20px; }
        #stream { width: 100%; border-radius: 8px; background: #000; }
        .controls { margin-top: 20px; text-align: center; }
        button { background: #4CAF50; color: white; border: none; padding: 12px 24px; margin: 5px; border-radius: 5px; cursor: pointer; font-size: 16px; }
        button:hover { background: #45a049; }
        button:disabled { opacity: 0.5; cursor: not-allowed; }
        .record-btn { background: #f44336; }
        .record-btn:hover { background: #d32f2f; }
        .record-btn.recording { background: #666; animation: recPulse 1s infinite; }
        .info { color: #aaa; margin-top: 15px; font-size: 14px; line-height: 1.6; }
        .hint { color: #f5a623; font-size: 13px; margin-top: 8px; }
        .status { display: inline-block; width: 10px; height: 10px; border-radius: 50%; background: #4CAF50; margin-right: 8px; animation: pulse 2s infinite; }
        .status.recording { background: #f44336; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        @keyframes recPulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.6; } }
        .rec-timer { color: #f44336; font-size: 18px; font-weight: bold; margin-left: 10px; display: none; }
        .rec-timer.active { display: inline; }
        .filename-config { margin-top: 15px; text-align: center; }
        .filename-config label { color: #aaa; font-size: 14px; margin-right: 10px; }
        .filename-config input { background: #1a1a1a; color: #fff; border: 1px solid #555; padding: 8px 12px; border-radius: 5px; font-size: 14px; width: 200px; }
        .filename-config input:focus { outline: none; border-color: #4CAF50; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎥 摄像头实时查看 <span class="status" id="statusDot"></span><span class="rec-timer" id="recTimer">● REC</span></h1>
        <img id="stream" src="/stream" alt="摄像头画面">
        <div class="controls">
            <button class="record-btn" id="recordBtn" onclick="toggleRecording()">🔴 开始录制</button>
            <button onclick="takeSnapshot()">📷 截图</button>
            <button onclick="reconnect()">🔄 重新连接</button>
            <button onclick="toggleFullscreen()">⛶ 全屏</button>
        </div>
        <div class="filename-config">
            <label>视频文件名前缀:</label>
            <input type="text" id="filenamePrefix" value="video" placeholder="video" />
            <span style="color: #777; font-size: 12px; margin-left: 10px;">格式: 前缀_年月日_时分秒.avi</span>
        </div>
        <div class="info">
            <p>• 点击"开始录制"录制视频(AVI格式) • 点击"停止录制"结束并保存 • 视频流: <span id="url"></span></p>
            <p>• 快捷键: 按 <strong>R</strong> 键开始/停止录制 • 按 <strong>F</strong> 键全屏</p>
            <p>• 延迟: <strong><span id="latency">--</span></strong> • 实时帧率: <strong><span id="fps">--</span></strong></p>
            <p id="clock-hint" class="hint"></p>
        </div>
    </div>
    <script>
        document.getElementById('url').textContent = window.location.origin + '/stream';
        const img = document.getElementById('stream');
        
        const savedPrefix = localStorage.getItem('filenamePrefix') || 'video';
        document.getElementById('filenamePrefix').value = savedPrefix;
        
        document.getElementById('filenamePrefix').addEventListener('change', function() {
            const prefix = this.value.trim() || 'video';
            localStorage.setItem('filenamePrefix', prefix);
            console.log('文件名前缀已保存:', prefix);
        });
        
        let isRecording = false;
        
        function toggleRecording() {
            const prefix = document.getElementById('filenamePrefix').value.trim() || 'video';
            const btn = document.getElementById('recordBtn');
            const statusDot = document.getElementById('statusDot');
            const recTimer = document.getElementById('recTimer');
            
            if (!isRecording) {
                fetch('/record/start?prefix=' + encodeURIComponent(prefix))
                    .then(r => r.json())
                    .then(data => {
                        if (data.success) {
                            isRecording = true;
                            btn.textContent = '⏹ 停止录制';
                            btn.classList.add('recording');
                            statusDot.classList.add('recording');
                            recTimer.classList.add('active');
                            showNotification('🔴 ' + data.message, '#f44336');
                        } else {
                            showNotification('❌ 录制失败: ' + (data.message || '未知错误'), '#f44336');
                        }
                    })
                    .catch(err => {
                        showNotification('❌ 请求失败: ' + err, '#f44336');
                    });
            } else {
                fetch('/record/stop')
                    .then(r => r.json())
                    .then(data => {
                        if (data.success) {
                            isRecording = false;
                            btn.textContent = '🔴 开始录制';
                            btn.classList.remove('recording');
                            statusDot.classList.remove('recording');
                            recTimer.classList.remove('active');
                            showNotification('✅ ' + data.message, '#4CAF50');
                            if (data.filename) {
                                setTimeout(function() {
                                    var a = document.createElement('a');
                                    a.href = '/download?file=' + encodeURIComponent(data.filename);
                                    a.download = data.filename;
                                    a.style.display = 'none';
                                    document.body.appendChild(a);
                                    a.click();
                                    document.body.removeChild(a);
                                }, 500);
                            }
                        } else {
                            showNotification('⚠️ ' + (data.message || '停止录制异常'), '#f5a623');
                        }
                    })
                    .catch(err => {
                        showNotification('❌ 请求失败: ' + err, '#f44336');
                    });
            }
        }
        
        function showNotification(msg, bgColor) {
            const notification = document.createElement('div');
            notification.textContent = msg;
            notification.style.cssText = 'position:fixed;top:20px;right:20px;background:' + bgColor + ';color:white;padding:15px 25px;border-radius:5px;box-shadow:0 2px 10px rgba(0,0,0,0.3);z-index:9999;';
            document.body.appendChild(notification);
            setTimeout(() => document.body.removeChild(notification), 4000);
        }
        
        function reconnect() {
            img.src = '/stream?t=' + new Date().getTime();
        }
        
        function toggleFullscreen() {
            if (!document.fullscreenElement) {
                img.requestFullscreen();
            } else {
                document.exitFullscreen();
            }
        }

        document.addEventListener('keydown', function(event) {
            const activeElement = document.activeElement;
            if (activeElement && (activeElement.tagName === 'INPUT' || activeElement.tagName === 'TEXTAREA')) {
                return;
            }
            if (event.key === 'r' || event.key === 'R') {
                event.preventDefault();
                toggleRecording();
            } else if (event.key === 'f' || event.key === 'F') {
                event.preventDefault();
                toggleFullscreen();
            } else if (event.key === 'c' || event.key === 'C') {
                event.preventDefault();
                reconnect();
            }
        });

        async function updateStats() {
            try {
                const response = await fetch('/stats');
                if (!response.ok) throw new Error('stats fetch failed');
                const data = await response.json();
                const latencyEl = document.getElementById('latency');
                const fpsEl = document.getElementById('fps');
                const hintEl = document.getElementById('clock-hint');
                hintEl.textContent = '';

                const captureTs = Number(data.latestCaptureTsMs) || 0;
                const serverTs = Number(data.serverTsMs) || 0;

                if (captureTs && serverTs) {
                    const internalLatency = Math.max(0, serverTs - captureTs);
                    latencyEl.textContent = internalLatency + ' ms';
                } else {
                    latencyEl.textContent = '--';
                }

                if (data.estimatedFps && data.estimatedFps > 0) {
                    fpsEl.textContent = Number(data.estimatedFps).toFixed(1) + ' FPS';
                } else {
                    fpsEl.textContent = '--';
                }
            } catch (err) {
                document.getElementById('latency').textContent = 'N/A';
                document.getElementById('fps').textContent = 'N/A';
                document.getElementById('clock-hint').textContent = '';
            }
        }

        function takeSnapshot() {
            fetch('/snapshot')
                .then(r => r.blob())
                .then(blob => {
                    const url = URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
                    a.href = url;
                    a.download = 'snapshot_' + ts + '.png';
                    a.click();
                    URL.revokeObjectURL(url);
                    showNotification('📷 截图已保存', '#4CAF50');
                })
                .catch(err => showNotification('❌ 截图失败: ' + err, '#f44336'));
        }

        setInterval(updateStats, 1000);
        updateStats();
    </script>
</body>
</html>
)HTML";

const char *debug_html = R"HTML(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>红色阈值调试</title>
<style>
body{margin:0;padding:10px;background:#1a1a1a;font-family:Arial;color:#fff}
.container{display:flex;flex-wrap:wrap;gap:10px}
.panel{background:#2d2d2d;border-radius:8px;padding:10px}
.img-panel{flex:2;min-width:400px}
.ctrl-panel{flex:1;min-width:320px;max-width:400px}
h2{font-size:16px;margin:0 0 8px;color:#4CAF50}
canvas{display:block;width:100%;border-radius:4px;background:#000}
.slider-row{display:flex;gap:8px;align-items:center;margin-bottom:8px}
.slider-row span.lbl{width:56px;font-size:12px;color:#aaa;text-align:right;flex-shrink:0}
.slider-row input{flex:1;margin:0 4px}
.slider-row span.val{width:32px;font-size:12px;color:#4CAF50;text-align:center;flex-shrink:0}
.info{font-size:11px;color:#777;margin-top:12px;line-height:1.6;border-top:1px solid #444;padding-top:8px}
</style>
</head>
<body>
<div class="container">
<div class="panel img-panel">
<h2>YUYV 原始画面（红=通过阈值）</h2>
<canvas id="cvs" width="320" height="240"></canvas>
</div>
<div class="panel ctrl-panel">
<h2>阈值调节</h2>
<div class="slider-row">
<span class="lbl">Y 下限</span><input type="range" id="y_lo" min="0" max="255" value="28"><span class="val" id="y_lo_v">28</span>
<span class="lbl">Y 上限</span><input type="range" id="y_hi" min="0" max="255" value="255"><span class="val" id="y_hi_v">255</span>
</div>
<div class="slider-row">
<span class="lbl">U 下限</span><input type="range" id="u_lo" min="0" max="255" value="0"><span class="val" id="u_lo_v">0</span>
<span class="lbl">U 上限</span><input type="range" id="u_hi" min="0" max="255" value="128"><span class="val" id="u_hi_v">128</span>
</div>
<div class="slider-row">
<span class="lbl">V 下限</span><input type="range" id="v_lo" min="0" max="255" value="150"><span class="val" id="v_lo_v">150</span>
<span class="lbl">V 上限</span><input type="range" id="v_hi" min="0" max="255" value="255"><span class="val" id="v_hi_v">255</span>
</div>
<div class="slider-row">
<span class="lbl">V-U 下限</span><input type="range" id="vu_lo" min="-255" max="255" value="48"><span class="val" id="vu_lo_v">48</span>
<span class="lbl">V-U 上限</span><input type="range" id="vu_hi" min="-255" max="255" value="255"><span class="val" id="vu_hi_v">255</span>
</div>
<div class="slider-row">
<span class="lbl">V-Y 下限</span><input type="range" id="vy_lo" min="-255" max="255" value="14"><span class="val" id="vy_lo_v">14</span>
<span class="lbl">V-Y 上限</span><input type="range" id="vy_hi" min="-255" max="255" value="255"><span class="val" id="vy_hi_v">255</span>
</div>
<div class="info">
<p>条件: Y∈[<b id="d_y_lo">28</b>,<b id="d_y_hi">255</b>] U∈[<b id="d_u_lo">0</b>,<b id="d_u_hi">128</b>] V∈[<b id="d_v_lo">150</b>,<b id="d_v_hi">255</b>]</p>
<p>V-U∈[<b id="d_vu_lo">48</b>,<b id="d_vu_hi">255</b>] V-Y∈[<b id="d_vy_lo">14</b>,<b id="d_vy_hi">255</b>]</p>
<p>红色像素: <b id="red_count">0</b></p>
</div>
</div>
</div>
<script>
const cvs=document.getElementById('cvs');
const ctx=cvs.getContext('2d');
const ids=['y_lo','y_hi','u_lo','u_hi','v_lo','v_hi','vu_lo','vu_hi','vy_lo','vy_hi'];
const vids=['y_lo_v','y_hi_v','u_lo_v','u_hi_v','v_lo_v','v_hi_v','vu_lo_v','vu_hi_v','vy_lo_v','vy_hi_v'];
const dids=['d_y_lo','d_y_hi','d_u_lo','d_u_hi','d_v_lo','d_v_hi','d_vu_lo','d_vu_hi','d_vy_lo','d_vy_hi'];

function getVals(){
  let v={};
  ids.forEach((id,i)=>{v[id]=parseInt(document.getElementById(id).value);
    document.getElementById(vids[i]).textContent=v[id];
    document.getElementById(dids[i]).textContent=v[id];});
  return v;
}
ids.forEach(id=>document.getElementById(id).addEventListener('input',()=>{getVals();processLast();}));

let gW=320,gH=240,gYuyv=null;
function clamp(v,lo,hi){return v<lo?lo:(v>hi?hi:v);}
function yuv2r(y,u,v){return clamp(y+1.402*(v-128),0,255);}
function yuv2g(y,u,v){return clamp(y-0.344*(u-128)-0.714*(v-128),0,255);}
function yuv2b(y,u,v){return clamp(y+1.772*(u-128),0,255);}

function processLast(){
  if(!gYuyv)return;
  if(cvs.width!==gW||cvs.height!==gH){cvs.width=gW;cvs.height=gH;}
  let img=ctx.createImageData(gW,gH);
  let d=img.data,stride=gW*2;
  let v=getVals();
  let cnt=0;
  for(let y=0;y<gH;y++){
    let rowOff=y*stride;
    for(let x=0;x<gW;x++){
      let pairBase=(x&~1)*2;
      let yy=gYuyv[rowOff+pairBase+((x&1)?2:0)];
      let uu=gYuyv[rowOff+pairBase+1];
      let vv=gYuyv[rowOff+pairBase+3];
      let i=(y*gW+x)*4;
      let isRed=(yy>=v.y_lo&&yy<=v.y_hi&&uu>=v.u_lo&&uu<=v.u_hi&&vv>=v.v_lo&&vv<=v.v_hi&&(vv-uu)>=v.vu_lo&&(vv-uu)<=v.vu_hi&&(vv-yy)>=v.vy_lo&&(vv-yy)<=v.vy_hi);
      if(isRed){d[i]=255;d[i+1]=0;d[i+2]=0;cnt++;}
      else{d[i]=yuv2r(yy,uu,vv);d[i+1]=yuv2g(yy,uu,vv);d[i+2]=yuv2b(yy,uu,vv);}
      d[i+3]=255;
    }
  }
  ctx.putImageData(img,0,0);
  document.getElementById('red_count').textContent=cnt;
}

fetch('/debug_stream')
.then(r=>r.body.getReader())
.then(reader=>{
  let buf=new Uint8Array(0);
  let bdMarker='\r\n--frame';
  let bdBuf=[];
  for(let i=0;i<bdMarker.length;i++)bdBuf.push(bdMarker.charCodeAt(i));
  function findBd(arr,from){
    for(let i=from;i<=arr.length-bdBuf.length;i++){
      let ok=true;
      for(let j=0;j<bdBuf.length;j++){if(arr[i+j]!==bdBuf[j]){ok=false;break;}}
      if(ok)return i;
    }
    return -1;
  }
  function pump(){
    reader.read().then(({done,value})=>{
      if(done)return;
      let tmp=new Uint8Array(buf.length+value.length);
      tmp.set(buf);tmp.set(value,buf.length);
      buf=tmp;
      let pos=0;
      while(true){
        let b0=findBd(buf,pos);
        if(b0===-1)break;
        let hdrEnd=b0;
        while(hdrEnd<buf.length-3&&!(buf[hdrEnd]==0x0D&&buf[hdrEnd+1]==0x0A&&buf[hdrEnd+2]==0x0D&&buf[hdrEnd+3]==0x0A)){
          hdrEnd++;
        }
        if(hdrEnd>=buf.length-3)break;
        let dataStart=hdrEnd+4;
        let b1=findBd(buf,dataStart);
        if(b1===-1)break;
        let payload=buf.slice(dataStart,b1);
        if(payload.length>=4){
          let w=payload[0]|(payload[1]<<8);
          let h=payload[2]|(payload[3]<<8);
          if(w>0&&h>0&&payload.length===4+w*h*2){
            gW=w;gH=h;gYuyv=new Uint8Array(payload.slice(4));
            processLast();
          }
        }
        pos=b1+bdBuf.length;
      }
      if(buf.length>200000)buf=buf.slice(buf.length-100000);
      pump();
    });
  }
  pump();
})
.catch(()=>{setTimeout(()=>location.reload(),2000);});

getVals();
</script>
</body>
</html>
)HTML";

CameraStreamServer::CameraStreamServer(void)
    : server_sock_fd(-1), server_port(CAMERA_STREAM_DEFAULT_PORT),
      running(false), server_thread_id(0), latest_frame_id(0),
      latest_capture_ts_ms(0), ema_fps(0.0), is_recording(false),
      frame_width(320), frame_height(240), frame_rate(30.0),
      use_jpeg_fallback(false), jpeg_frame_count(0),
      yuyv_w(0), yuyv_h(0), yuyv_frame_id(0) {
  pthread_mutex_init(&frame_mutex, NULL);
  pthread_cond_init(&frame_cond, NULL);
  pthread_mutex_init(&sock_mutex, NULL);
  pthread_mutex_init(&original_frame_mutex, NULL);
  pthread_mutex_init(&recording_mutex, NULL);
  pthread_mutex_init(&yuyv_mutex, NULL);
  pthread_cond_init(&yuyv_cond, NULL);
}

CameraStreamServer::~CameraStreamServer(void) {
  stop_server();
  if (is_recording) {
    stop_recording();
  }
  pthread_mutex_destroy(&frame_mutex);
  pthread_cond_destroy(&frame_cond);
  pthread_mutex_destroy(&sock_mutex);
  pthread_mutex_destroy(&original_frame_mutex);
  pthread_mutex_destroy(&recording_mutex);
  pthread_mutex_destroy(&yuyv_mutex);
  pthread_cond_destroy(&yuyv_cond);
}


std::string CameraStreamServer::get_local_ip(void) {
  struct ifaddrs *ifaddr, *ifa;
  char host[NI_MAXHOST];

  if (getifaddrs(&ifaddr) == -1) {
    return "127.0.0.1";
  }

  std::string result = "127.0.0.1";
  std::string fallback_ip = "";
  int best_priority = -1;

  for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == NULL)
      continue;

    int family = ifa->ifa_addr->sa_family;

    if (family == AF_INET) {
      if (strcmp(ifa->ifa_name, "lo") == 0)
        continue;

      int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host,
                          NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      if (s != 0)
        continue;

      int priority = 0;
      bool is_up = (ifa->ifa_flags & IFF_UP) != 0;
      bool is_running = (ifa->ifa_flags & IFF_RUNNING) != 0;

      if (is_up && is_running) {
        priority = 100;
        if (strncmp(ifa->ifa_name, "wlan", 4) == 0)
          priority += 20;
        else if (strncmp(ifa->ifa_name, "eth", 3) == 0)
          priority += 15;
        else if (strncmp(ifa->ifa_name, "en", 2) == 0)
          priority += 15;
        else
          priority += 5;
      } else if (is_up) {
        priority = 50;
        if (strncmp(ifa->ifa_name, "wlan", 4) == 0)
          priority += 10;
        else if (strncmp(ifa->ifa_name, "eth", 3) == 0)
          priority += 8;
        else if (strncmp(ifa->ifa_name, "en", 2) == 0)
          priority += 8;
      } else {
        priority = 10;
      }

      if (priority > best_priority) {
        best_priority = priority;
        result = host;
      }

      if (fallback_ip.empty() && strcmp(host, "127.0.0.1") != 0) {
        fallback_ip = host;
      }
    }
  }

  freeifaddrs(ifaddr);

  if (result == "127.0.0.1" && !fallback_ip.empty()) {
    result = fallback_ip;
  }

  return result;
}

void CameraStreamServer::close_server_socket(void) {
  pthread_mutex_lock(&sock_mutex);
  if (server_sock_fd >= 0) {
    shutdown(server_sock_fd, SHUT_RDWR);
    close(server_sock_fd);
    server_sock_fd = -1;
  }
  pthread_mutex_unlock(&sock_mutex);
}

uint64_t CameraStreamServer::now_ms(void) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string CameraStreamServer::format_timestamp(uint64_t ts_ms) {
  if (ts_ms == 0)
    return "--";
  time_t seconds = static_cast<time_t>(ts_ms / 1000);
  int ms = static_cast<int>(ts_ms % 1000);
  struct tm tm_time;
  localtime_r(&seconds, &tm_time);
  char date_buf[64];
  strftime(date_buf, sizeof(date_buf), "%Y-%m-%d %H:%M:%S", &tm_time);
  char buf[80];
  snprintf(buf, sizeof(buf), "%s.%03d", date_buf, ms);
  return std::string(buf);
}

void CameraStreamServer::send_response(int sock, const char *content_type,
                                       const char *body, size_t body_len) {
  std::ostringstream header;
  header << "HTTP/1.1 200 OK\r\n";
  header << "Content-Type: " << content_type << "\r\n";
  header << "Content-Length: " << body_len << "\r\n";
  header << "Connection: close\r\n\r\n";
  std::string h = header.str();
  send(sock, h.c_str(), h.length(), 0);
  send(sock, body, body_len, 0);
}

void CameraStreamServer::send_stats_response(int sock) {
  uint64_t capture_ts = latest_capture_ts_ms;
  uint64_t frame_id = 0;
  pthread_mutex_lock(&frame_mutex);
  frame_id = latest_frame_id;
  pthread_mutex_unlock(&frame_mutex);

  uint64_t server_ts = now_ms();
  double fps = ema_fps;
  std::ostringstream body;
  body << std::fixed << std::setprecision(2)
       << "{\"latestFrameId\":" << frame_id
       << ",\"latestCaptureTsMs\":" << capture_ts
       << ",\"serverTsMs\":" << server_ts << ",\"estimatedFps\":" << fps << "}";
  std::string json = body.str();
  send_response(sock, "application/json; charset=utf-8", json.c_str(),
                json.size());
}

void CameraStreamServer::send_mjpeg_stream(int sock) {
  const char *header =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: close\r\n\r\n";
  send(sock, header, strlen(header), 0);

  uint64_t last_frame_sent = 0;

  while (running) {
    std::vector<unsigned char> jpeg_copy;

    pthread_mutex_lock(&frame_mutex);
    while (running && latest_frame_id == last_frame_sent) {
      pthread_cond_wait(&frame_cond, &frame_mutex);
    }

    if (!running) {
      pthread_mutex_unlock(&frame_mutex);
      break;
    }

    if (current_jpeg.empty()) {
      pthread_mutex_unlock(&frame_mutex);
      continue;
    }

    jpeg_copy = current_jpeg;
    last_frame_sent = latest_frame_id;
    pthread_mutex_unlock(&frame_mutex);

    char boundary[256];
    snprintf(
        boundary, sizeof(boundary),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
        jpeg_copy.size());

    if (send(sock, boundary, strlen(boundary), 0) < 0)
      break;
    if (send(sock, jpeg_copy.data(), jpeg_copy.size(), 0) < 0)
      break;
    if (send(sock, "\r\n", 2, 0) < 0)
      break;
  }
}

void CameraStreamServer::update_yuyv_frame(const cv::Mat &yuyv) {
  if (yuyv.empty() || yuyv.type() != CV_8UC2)
    return;

  size_t sz = yuyv.total() * yuyv.elemSize();
  pthread_mutex_lock(&yuyv_mutex);
  yuyv_frame.assign(yuyv.ptr<uint8_t>(), yuyv.ptr<uint8_t>() + sz);
  yuyv_w = yuyv.cols;
  yuyv_h = yuyv.rows;
  ++yuyv_frame_id;
  pthread_cond_broadcast(&yuyv_cond);
  pthread_mutex_unlock(&yuyv_mutex);
}

void CameraStreamServer::send_debug_mjpeg_stream(int sock) {
  const char *header =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: close\r\n\r\n";
  send(sock, header, strlen(header), 0);

  uint64_t last_frame_sent = 0;

  while (running) {
    pthread_mutex_lock(&yuyv_mutex);
    while (running && yuyv_frame_id == last_frame_sent) {
      pthread_cond_wait(&yuyv_cond, &yuyv_mutex);
    }

    if (!running) {
      pthread_mutex_unlock(&yuyv_mutex);
      break;
    }

    if (yuyv_frame.empty()) {
      pthread_mutex_unlock(&yuyv_mutex);
      continue;
    }

    std::vector<uint8_t> raw = yuyv_frame;
    int w = yuyv_w, h = yuyv_h;
    last_frame_sent = yuyv_frame_id;
    pthread_mutex_unlock(&yuyv_mutex);

    std::vector<uint8_t> payload;
    payload.resize(4 + raw.size());
    payload[0] = (uint8_t)(w & 0xFF);
    payload[1] = (uint8_t)((w >> 8) & 0xFF);
    payload[2] = (uint8_t)(h & 0xFF);
    payload[3] = (uint8_t)((h >> 8) & 0xFF);
    memcpy(payload.data() + 4, raw.data(), raw.size());

    char boundary[256];
    snprintf(boundary, sizeof(boundary),
             "--frame\r\nContent-Type: application/octet-stream\r\nContent-Length: %zu\r\n\r\n",
             payload.size());

    if (send(sock, boundary, strlen(boundary), 0) < 0)
      break;
    if (send(sock, payload.data(), payload.size(), 0) < 0)
      break;
    if (send(sock, "\r\n", 2, 0) < 0)
      break;
  }
}

void CameraStreamServer::handle_snapshot_request(int sock,
                                                 const std::string &prefix) {
  pthread_mutex_lock(&original_frame_mutex);
  cv::Mat frame_copy = original_frame.clone();
  pthread_mutex_unlock(&original_frame_mutex);

  if (frame_copy.empty()) {
    const char *error_html = "<h1>Error</h1><p>没有可用的图像帧</p>";
    send_response(sock, "text/html; charset=utf-8", error_html,
                  strlen(error_html));
    return;
  }

  std::vector<unsigned char> image_buffer;
  std::vector<int> params;
  params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  params.push_back(3);

  if (!cv::imencode(".png", frame_copy, image_buffer, params)) {
    const char *error_html = "<h1>Error</h1><p>图像编码失败</p>";
    send_response(sock, "text/html; charset=utf-8", error_html,
                  strlen(error_html));
    std::cerr << "✗ 图像编码失败" << std::endl;
    return;
  }

  time_t now = time(NULL);
  struct tm tm_time;
  localtime_r(&now, &tm_time);
  char filename[256];
  snprintf(filename, sizeof(filename), "%s_%04d%02d%02d_%02d%02d%02d.png",
           prefix.c_str(), tm_time.tm_year + 1900, tm_time.tm_mon + 1,
           tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);

  std::ostringstream header;
  header << "HTTP/1.1 200 OK\r\n";
  header << "Content-Type: image/png\r\n";
  header << "Content-Length: " << image_buffer.size() << "\r\n";
  header << "Content-Disposition: attachment; filename=\"" << filename
         << "\"\r\n";
  header << "Cache-Control: no-cache\r\n";
  header << "Connection: close\r\n\r\n";

  std::string h = header.str();
  send(sock, h.c_str(), h.length(), 0);
  send(sock, image_buffer.data(), image_buffer.size(), 0);

  std::cout << "✓ 已发送原始无损图片到客户端: " << filename
            << " (大小: " << image_buffer.size() / 1024 << " KB, PNG无损)"
            << std::endl;
}

void CameraStreamServer::handle_client_request(int sock) {
  char buffer[4096];
  ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
  if (n <= 0) {
    close(sock);
    return;
  }
  buffer[n] = '\0';

  std::string request(buffer);
  size_t path_start = request.find(" ") + 1;
  size_t path_end = request.find(" ", path_start);
  std::string path = request.substr(path_start, path_end - path_start);

  if (path == "/" || path.find("/viewer") == 0 || path.find("/?") == 0) {
    send_response(sock, "text/html; charset=utf-8", viewer_html,
                  strlen(viewer_html));
  } else if (path == "/debug") {
    send_response(sock, "text/html; charset=utf-8", debug_html,
                  strlen(debug_html));
  } else if (path.find("/debug_stream") == 0) {
    send_debug_mjpeg_stream(sock);
  } else if (path.find("/stream") == 0) {
    send_mjpeg_stream(sock);
  } else if (path.find("/stats") == 0) {
    send_stats_response(sock);
  } else if (path.find("/snapshot") == 0) {
    std::string prefix = "snapshot";
    size_t query_pos = path.find("?prefix=");
    if (query_pos != std::string::npos) {
      size_t prefix_start = query_pos + 8;
      size_t prefix_end = path.find("&", prefix_start);
      if (prefix_end == std::string::npos) {
        prefix_end = path.length();
      }
      prefix = path.substr(prefix_start, prefix_end - prefix_start);

      size_t pos = 0;
      while ((pos = prefix.find("%20", pos)) != std::string::npos) {
        prefix.replace(pos, 3, " ");
        pos += 1;
      }

      bool valid = true;
      for (char c : prefix) {
        if (!isalnum(c) && c != '_' && c != '-') {
          valid = false;
          break;
        }
      }
      if (!valid || prefix.empty()) {
        prefix = "snapshot";
      }
    }

    handle_snapshot_request(sock, prefix);
  } else if (path.find("/record/start") == 0) {
    std::string prefix = "video";
    size_t query_pos = path.find("?prefix=");
    if (query_pos != std::string::npos) {
      size_t prefix_start = query_pos + 8;
      size_t prefix_end = path.find("&", prefix_start);
      if (prefix_end == std::string::npos) {
        prefix_end = path.length();
      }
      prefix = path.substr(prefix_start, prefix_end - prefix_start);

      size_t pos = 0;
      while ((pos = prefix.find("%20", pos)) != std::string::npos) {
        prefix.replace(pos, 3, " ");
        pos += 1;
      }

      bool valid = true;
      for (char c : prefix) {
        if (!isalnum(c) && c != '_' && c != '-') {
          valid = false;
          break;
        }
      }
      if (!valid || prefix.empty()) {
        prefix = "video";
      }
    }
    start_recording(prefix);
    std::ostringstream json;
    if (is_recording) {
      json << "{\"success\":true" << ",\"message\":\""
           << (use_jpeg_fallback ? "JPEG帧序列录制中" : "视频录制中") << ": "
           << video_filename << "\"" << ",\"filename\":\"" << video_filename
           << "\"" << ",\"fallback\":" << (use_jpeg_fallback ? "true" : "false")
           << "}";
    } else {
      json << "{\"success\":false"
           << ",\"message\":\"摄像头未就绪或编码器不可用\"}";
    }
    std::string body = json.str();
    send_response(sock, "application/json; charset=utf-8", body.c_str(),
                  body.size());
  } else if (path.find("/record/stop") == 0) {
    std::string saved_filename = video_filename;
    bool was_jpeg = use_jpeg_fallback;
    int saved_count = jpeg_frame_count;
    stop_recording();

    std::ostringstream json;
    json << "{\"success\":true";
    if (was_jpeg) {
      json << ",\"message\":\"已保存 " << saved_count << " 帧到 "
           << saved_filename << "\"" << ",\"frames\":" << saved_count;
    } else {
      json << ",\"message\":\"视频已保存: " << saved_filename << "\""
           << ",\"filename\":\"" << saved_filename << "\"";
    }
    json << "}";
    std::string body = json.str();
    send_response(sock, "application/json; charset=utf-8", body.c_str(),
                  body.size());
  } else if (path.find("/download") == 0) {
    std::string file_name;
    size_t query_pos = path.find("?file=");
    if (query_pos != std::string::npos) {
      size_t name_start = query_pos + 6;
      size_t name_end = path.find("&", name_start);
      if (name_end == std::string::npos)
        name_end = path.length();
      file_name = path.substr(name_start, name_end - name_start);
      size_t p = 0;
      while ((p = file_name.find("%20", p)) != std::string::npos) {
        file_name.replace(p, 3, " ");
        p += 1;
      }
    }
    if (file_name.empty()) {
      const char *err = "<h1>400 Bad Request</h1><p>缺少 file 参数</p>";
      send_response(sock, "text/html; charset=utf-8", err, strlen(err));
    } else {
      std::string full_path = "/tmp/" + file_name;
      std::ifstream file(full_path, std::ios::binary | std::ios::ate);
      if (!file) {
        std::ostringstream oss;
        oss << "<h1>404</h1><p>文件不存在: " << file_name << "</p>";
        std::string msg = oss.str();
        send_response(sock, "text/html; charset=utf-8", msg.c_str(),
                      msg.size());
      } else {
        long file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> buffer(file_size);
        file.read(buffer.data(), file_size);
        file.close();
        std::ostringstream header;
        header << "HTTP/1.1 200 OK\r\n";
        header << "Content-Type: application/octet-stream\r\n";
        header << "Content-Length: " << file_size << "\r\n";
        header << "Content-Disposition: attachment; filename=\"" << file_name
               << "\"\r\n";
        header << "Connection: close\r\n\r\n";
        std::string h = header.str();
        send(sock, h.c_str(), h.length(), 0);
        send(sock, buffer.data(), file_size, 0);
        std::cout << "[下载] 已发送文件: " << file_name << " ("
                  << (file_size / 1024.0) << " KB)" << std::endl;
      }
    }
  } else {
    const char *not_found = "<h1>404 Not Found</h1>";
    send_response(sock, "text/html", not_found, strlen(not_found));
  }

  close(sock);
}

void *CameraStreamServer::client_thread_func(void *arg) {
  int sock = *(int *)arg;
  delete (int *)arg;

  if (instance) {
    instance->handle_client_request(sock);
  } else {
    close(sock);
  }

  return NULL;
}

void *CameraStreamServer::server_thread_func(void *arg) {
  CameraStreamServer *server = static_cast<CameraStreamServer *>(arg);
  if (!server)
    return NULL;

  int server_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (server_sock < 0) {
    std::cerr << "创建socket失败" << std::endl;
    return NULL;
  }

  pthread_mutex_lock(&server->sock_mutex);
  server->server_sock_fd = server_sock;
  pthread_mutex_unlock(&server->sock_mutex);

  int opt = 1;
  setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(server->server_port);

  if (bind(server_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    std::cerr << "绑定端口失败" << std::endl;
    close(server_sock);
    return NULL;
  }

  if (listen(server_sock, 10) < 0) {
    std::cerr << "监听失败" << std::endl;
    close(server_sock);
    return NULL;
  }

  std::string local_ip = server->get_local_ip();

  std::cout << "\n======================================" << std::endl;
  std::cout << "📡 MJPEG摄像头图传服务器启动成功!" << std::endl;
  std::cout << "======================================" << std::endl;
  std::cout << "监听端口: " << server->server_port << std::endl;
  std::cout << "本机IP: " << local_ip << std::endl;
  std::cout << "请在浏览器访问: http://" << local_ip << ":"
            << server->server_port << std::endl;
  std::cout << "======================================\n" << std::endl;

  while (server->running) {
    struct sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);
    int client_sock =
        accept(server_sock, (struct sockaddr *)&client_addr, &len);

    if (client_sock < 0) {
      if (!server->running)
        break;
      continue;
    }

    int flag = 1;
    setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    int sndbuf = 65536;
    setsockopt(client_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    pthread_t tid;
    int *sock_ptr = new int(client_sock);
    pthread_create(&tid, NULL, client_thread_func, sock_ptr);
    pthread_detach(tid);
  }

  server->close_server_socket();
  return NULL;
}

void CameraStreamServer::signal_handler(int sig) {
  std::cout << "\n正在关闭服务器..." << std::endl;
  if (instance) {
    instance->stop_server();
  }
}

int CameraStreamServer::start_server(int port) {
  if (running) {
    std::cout << "服务器已经在运行中" << std::endl;
    return 0;
  }

  server_port = port;
  running = true;
  latest_frame_id = 0;
  current_jpeg.clear();

  instance = this;

  if (pthread_create(&server_thread_id, NULL, server_thread_func, this) != 0) {
    std::cerr << "创建服务器线程失败" << std::endl;
    running = false;
    return -1;
  }
  pthread_detach(server_thread_id);

  std::cout << "摄像头图传服务器启动中..." << std::endl;
  return 0;
}

void CameraStreamServer::update_frame(const cv::Mat &frame) {
  if (frame.empty())
    return;

  uint64_t capture_ts_ms = now_ms();

  pthread_mutex_lock(&original_frame_mutex);
  original_frame = frame.clone();
  pthread_mutex_unlock(&original_frame_mutex);

  static uint64_t last_capture_ts_local = 0;
  static double local_fps_estimate = 0.0;
  if (last_capture_ts_local != 0) {
    uint64_t delta = capture_ts_ms - last_capture_ts_local;
    if (delta > 0) {
      double instant_fps = 1000.0 / static_cast<double>(delta);
      if (local_fps_estimate <= 0.0) {
        local_fps_estimate = instant_fps;
      } else {
        local_fps_estimate = 0.85 * local_fps_estimate + 0.15 * instant_fps;
      }
      ema_fps = local_fps_estimate;
    }
  }
  last_capture_ts_local = capture_ts_ms;

  // 320x240 JPEG 流式传输
  std::vector<unsigned char> jpeg_buffer;
  std::vector<int> params;
  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(20);

  if (cv::imencode(".jpg", frame, jpeg_buffer, params)) {
    latest_capture_ts_ms = capture_ts_ms;
    pthread_mutex_lock(&frame_mutex);
    current_jpeg.swap(jpeg_buffer);
    ++latest_frame_id;
    pthread_cond_broadcast(&frame_cond);
    pthread_mutex_unlock(&frame_mutex);
  }

  if (is_recording) {
    record_frame(frame);
  }
}

void CameraStreamServer::start_recording(const std::string &prefix) {
  pthread_mutex_lock(&recording_mutex);

  if (is_recording) {
    pthread_mutex_unlock(&recording_mutex);
    std::cout << "[录制] 已经在录制中" << std::endl;
    return;
  }

  time_t now = time(NULL);
  struct tm tm_time;
  localtime_r(&now, &tm_time);
  char name_buf[256];
  snprintf(name_buf, sizeof(name_buf), "%s_%04d%02d%02d_%02d%02d%02d",
           prefix.c_str(), tm_time.tm_year + 1900, tm_time.tm_mon + 1,
           tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);

  std::string base_name = std::string(name_buf);
  std::string filepath = "/tmp/" + base_name + ".avi";

  // 尝试多种编码格式，写入测试帧验证是否真正可用
  int fourcc_list[] = {cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                       cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
                       cv::VideoWriter::fourcc('D', 'I', 'V', 'X'),
                       cv::VideoWriter::fourcc('F', 'L', 'V', '1'), -1};
  const char *fourcc_names[] = {"MJPG", "XVID", "DIVX", "FLV1", "AUTO"};

  cv::Mat test_frame(frame_height, frame_width, CV_8UC3, cv::Scalar(0, 0, 0));
  bool writer_ok = false;

  for (int i = 0; i < 5; i++) {
    video_writer.open(filepath, fourcc_list[i], frame_rate,
                      cv::Size(frame_width, frame_height), true);
    if (!video_writer.isOpened()) {
      std::cerr << "[录制] 编码 " << fourcc_names[i] << " 不可用" << std::endl;
      continue;
    }

    // 写入测试帧验证是否真正生效（避免 isOpened 为 true 但实际不写数据）
    video_writer.write(test_frame);
    video_writer.write(test_frame);
    video_writer.release();

    std::ifstream test_in(filepath, std::ios::binary | std::ios::ate);
    if (test_in && test_in.tellg() > 100) {
      test_in.close();
      // 重新打开用于实际录制
      video_writer.open(filepath, fourcc_list[i], frame_rate,
                        cv::Size(frame_width, frame_height), true);
      std::cout << "[录制] 使用编码: " << fourcc_names[i] << std::endl;
      writer_ok = true;
      break;
    }
    test_in.close();
    ::remove(filepath.c_str());
    std::cerr << "[录制] 编码 " << fourcc_names[i]
              << " isOpened 但写入无效，尝试下一个" << std::endl;
  }

  if (writer_ok) {
    use_jpeg_fallback = false;
    video_filename = base_name + ".avi";
    is_recording = true;
    pthread_mutex_unlock(&recording_mutex);
    std::cout << "[录制] ✅ 已开始录制视频: " << video_filename << std::endl;
    return;
  }

  // 所有编码器均不可用，回退到 JPEG 帧序列
  jpeg_dir = "/tmp/" + base_name + "_frames/";
  std::string mkdir_cmd = "mkdir -p " + jpeg_dir;
  if (system(mkdir_cmd.c_str()) != 0) {
    std::cerr << "[录制] 无法创建帧目录: " << jpeg_dir << std::endl;
    is_recording = false;
    use_jpeg_fallback = false;
    pthread_mutex_unlock(&recording_mutex);
    return;
  }

  use_jpeg_fallback = true;
  jpeg_frame_count = 0;
  video_filename = base_name + "_frames/";
  jpeg_base_name = base_name;
  is_recording = true;
  pthread_mutex_unlock(&recording_mutex);

  std::cout << "[录制] ⚠ 所有 AVI 编码器均不可用，回退到 JPEG 帧序列: "
            << base_name << "_frames/" << std::endl;
}

void CameraStreamServer::stop_recording(void) {
  pthread_mutex_lock(&recording_mutex);

  if (!is_recording) {
    pthread_mutex_unlock(&recording_mutex);
    std::cout << "[录制] 当前未在录制" << std::endl;
    return;
  }

  is_recording = false;

  if (!use_jpeg_fallback) {
    video_writer.release();
  }

  std::string saved_name = video_filename;
  bool was_jpeg = use_jpeg_fallback;
  int total_frames = jpeg_frame_count;

  video_filename.clear();
  use_jpeg_fallback = false;
  jpeg_frame_count = 0;
  jpeg_base_name.clear();

  pthread_mutex_unlock(&recording_mutex);

  if (was_jpeg) {
    std::cout << "[录制] ⏹ 停止录制，共保存 " << total_frames
              << " 帧到: " << saved_name << std::endl;

    // 使用 ffmpeg 将 JPEG 帧序列打包为 MP4
    std::string mp4_path = "/tmp/" + jpeg_base_name + ".mp4";
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
             "ffmpeg -y -framerate %d -i %sframe_%%06d.jpg -c:v libx264 "
             "-pix_fmt yuv420p %s 2>/dev/null",
             (int)frame_rate, saved_name.c_str(), mp4_path.c_str());
    int ret = system(cmd);
    if (ret == 0) {
      std::cout << "[录制] ✅ 已打包为 MP4: " << mp4_path << std::endl;
      // 用 MP4 文件名覆盖，浏览器下载时发送 MP4
      saved_name = jpeg_base_name + ".mp4";
      video_filename = saved_name;
    } else {
      // ffmpeg 不可用，打包 zip 作为替代
      std::string zip_path = "/tmp/" + jpeg_base_name + "_frames.zip";
      snprintf(cmd, sizeof(cmd), "cd /tmp && zip -rq %s %s",
               (jpeg_base_name + "_frames.zip").c_str(),
               (jpeg_base_name + "_frames").c_str());
      ret = system(cmd);
      if (ret == 0) {
        std::cout << "[录制] ⚠ ffmpeg 不可用，已打包为 ZIP: " << zip_path
                  << std::endl;
        std::cout << "[录制] 提示: 在电脑上解压后用 ffmpeg 合成视频: "
                  << "ffmpeg -framerate 30 -i frame_%06d.jpg -c:v libx264 "
                  << jpeg_base_name << ".mp4" << std::endl;
        saved_name = jpeg_base_name + "_frames.zip";
        video_filename = saved_name;
      } else {
        std::cout << "[录制] 提示: 在电脑上用 ffmpeg 合成视频: "
                  << "ffmpeg -framerate 30 -i frame_%06d.jpg -c:v libx264 "
                  << jpeg_base_name << ".mp4" << std::endl;
      }
    }
  } else {
    std::string full_path = "/tmp/" + saved_name;
    std::ifstream in(full_path, std::ios::binary | std::ios::ate);
    if (in) {
      long size = in.tellg();
      in.close();
      std::cout << "[录制] ⏹ 停止录制，视频已保存: " << saved_name << " ("
                << (size / 1024.0 / 1024.0) << " MB)" << std::endl;
    }
  }
}

void CameraStreamServer::record_frame(const cv::Mat &frame) {
  pthread_mutex_lock(&recording_mutex);
  if (!is_recording) {
    pthread_mutex_unlock(&recording_mutex);
    return;
  }

  if (use_jpeg_fallback) {
    char fname[512];
    snprintf(fname, sizeof(fname), "%s/frame_%06d.jpg", jpeg_dir.c_str(),
             jpeg_frame_count++);
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(90);
    cv::Mat resized;
    if (frame.cols != frame_width || frame.rows != frame_height) {
      cv::resize(frame, resized, cv::Size(frame_width, frame_height));
      cv::imwrite(fname, resized, params);
    } else {
      cv::imwrite(fname, frame, params);
    }
  } else if (video_writer.isOpened()) {
    if (frame.cols != frame_width || frame.rows != frame_height) {
      cv::Mat resized;
      cv::resize(frame, resized, cv::Size(frame_width, frame_height));
      video_writer.write(resized);
    } else {
      video_writer.write(frame);
    }
  }
  pthread_mutex_unlock(&recording_mutex);
}

void CameraStreamServer::stop_server(void) {
  if (!running)
    return;

  std::cout << "正在停止摄像头图传服务器..." << std::endl;
  running = false;
  close_server_socket();
  pthread_cond_broadcast(&frame_cond);

  if (instance == this) {
    instance = nullptr;
  }

  std::cout << "摄像头图传服务器已停止" << std::endl;
}

bool CameraStreamServer::is_running(void) { return running; }