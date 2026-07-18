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
<title>智能车视觉监控</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#f0f2f5;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;color:#333;min-height:100vh}
.header{background:#fff;border-bottom:1px solid #e0e0e0;padding:8px 16px;display:flex;align-items:center;gap:20px;flex-wrap:wrap;box-shadow:0 1px 3px rgba(0,0,0,0.06)}
.header .title{font-size:16px;font-weight:600;color:#1a73e8}
.header .meta{display:flex;gap:16px;font-size:12px;color:#666;flex-wrap:wrap}
.header .meta .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px}
.header .meta .dot.on{background:#34a853}
.header .meta .dot.off{background:#ea4335}
.header .meta .dot.rec{background:#ea4335;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.main{display:flex;gap:8px;padding:8px;height:calc(100vh - 52px)}
.panel{flex:1;background:#fff;border-radius:8px;overflow:hidden;display:flex;flex-direction:column;box-shadow:0 1px 3px rgba(0,0,0,0.08);min-width:0}
.panel-header{display:flex;align-items:center;justify-content:space-between;padding:6px 10px;border-bottom:1px solid #eee;background:#fafafa}
.panel-header .label{font-size:13px;font-weight:500;color:#555}
.canvas-wrap{flex:1;display:flex;align-items:flex-start;justify-content:center;overflow:auto;background:#f5f5f5;padding:4px}
.canvas-wrap img{display:block;image-rendering:pixelated}
.panel-ctrl{display:flex;gap:4px;padding:4px 6px;border-top:1px solid #eee;background:#fafafa;flex-wrap:wrap}
.btn{padding:5px 10px;border:1px solid #d0d0d0;border-radius:4px;cursor:pointer;font-size:11px;background:#fff;color:#444;transition:all .15s}
.btn:hover{background:#f0f0f0;border-color:#bbb}
.btn.rec{background:#fff;color:#d93025;border-color:#f5c6cb}
.btn.rec.active{background:#d93025;color:#fff;border-color:#d93025}
.btn.cap{background:#fff;color:#1a73e8;border-color:#c6dafc}
.btn.cap.active{background:#1a73e8;color:#fff;border-color:#1a73e8}
.btn.seq{background:#fff;color:#188038;border-color:#ceead6}
.btn.seq.active{background:#188038;color:#fff;border-color:#188038}
.btn:disabled{opacity:.4;cursor:not-allowed}
.toast{position:fixed;top:12px;right:12px;padding:10px 16px;border-radius:6px;color:#fff;font-size:13px;z-index:999;box-shadow:0 2px 8px rgba(0,0,0,0.15);animation:slideIn .3s}
.toast.ok{background:#34a853}
.toast.err{background:#ea4335}
@keyframes slideIn{from{transform:translateX(100px);opacity:0}to{transform:translateX(0);opacity:1}}
</style>
</head>
<body>
<div class="header">
<div class="title">🚗 智能车视觉监控</div>
<div class="meta">
<span><span class="dot on" id="dotMain"></span>全图 FPS:<b id="fpsMain">--</b></span>
<span><span class="dot on" id="dotRoi"></span>ROI FPS:<b id="fpsRoi">--</b></span>
<span>延迟:<b id="latency">--</b>ms</span>
<span id="recStatus" style="color:#d93025;display:none">⏺ 录制中</span>
</div>
</div>
<div class="main">
<div class="panel" id="panelMain">
<div class="panel-header"><span class="label">📷 全图 + 标注</span></div>
<div class="canvas-wrap"><img id="imgMain" src="" alt="全图" onmousemove="showCoords(event,'main',320,240)" onmouseout="hideCoords('main')"></div>
<div id="coordsMain" style="text-align:center;font-size:11px;color:#888;height:16px;line-height:16px">&nbsp;</div>
<div class="panel-ctrl">
<button class="btn rec" id="btnRecMain" onclick="toggleRec('main')">⏺ 录屏</button>
<button class="btn cap" onclick="snapshot('main')">📸 截图</button>
<button class="btn seq" id="btnSeqMain" onclick="toggleSeq('main')">🎞 连续截图</button>
</div>
</div>
<div class="panel" id="panelRoi">
<div class="panel-header"><span class="label">🔍 IPM ROI</span></div>
<div class="canvas-wrap"><img id="imgRoi" src="" alt="ROI"></div>
<div class="panel-ctrl">
<button class="btn rec" id="btnRecRoi" onclick="toggleRec('roi')">⏺ 录屏</button>
<button class="btn cap" onclick="snapshot('roi')">📸 截图</button>
<button class="btn seq" id="btnSeqRoi" onclick="toggleSeq('roi')">🎞 连续截图</button>
</div>
</div>
</div>
<script>
function $(id){return document.getElementById(id)}
function toast(msg,ok){var d=document.createElement('div');d.className='toast '+(ok?'ok':'err');d.textContent=msg;document.body.appendChild(d);setTimeout(function(){d.remove()},3000)}

function showCoords(e,canvas,origW,origH){
  var img=e.target;
  var rect=img.getBoundingClientRect();
  var x=e.clientX-rect.left, y=e.clientY-rect.top;
  var rx=Math.round(x/img.clientWidth*origW);
  var ry=Math.round(y/img.clientHeight*origH);
  rx=rx<0?0:(rx>=origW?origW-1:rx);
  ry=ry<0?0:(ry>=origH?origH-1:ry);
  var el=$('coords'+canvas.charAt(0).toUpperCase()+canvas.slice(1));
  if(el)el.textContent='X:'+rx+'  Y:'+ry+'  ('+origW+'x'+origH+')';
}
function hideCoords(canvas){
  var el=$('coords'+canvas.charAt(0).toUpperCase()+canvas.slice(1));
  if(el)el.textContent='\u00A0';
}

var state={main:{rec:false,seq:false,seqCount:0},roi:{rec:false,seq:false,seqCount:0}};

function toggleRec(canvas){
var s=state[canvas],id=canvas==='main'?'':'Roi';
var btn=$('btnRec'+(id?'Roi':'Main'));
var prefix=canvas+'_rec';
if(!s.rec){
fetch('/'+canvas+'/record/start?prefix='+encodeURIComponent(prefix))
.then(r=>r.json()).then(d=>{
if(d.success){s.rec=true;btn.classList.add('active');btn.textContent='⏹ 停止';$('recStatus').style.display='inline';toast('录制已开始','ok')}
else toast('录制失败:'+(d.message||''),'err')
}).catch(e=>toast('请求失败','err'));
}else{
fetch('/'+canvas+'/record/stop')
.then(r=>r.json()).then(d=>{
s.rec=false;btn.classList.remove('active');btn.textContent='⏺ 录屏';
if(!state.main.rec&&!state.roi.rec)$('recStatus').style.display='none';
toast('录制已保存','ok');
if(d.filename)setTimeout(function(){var a=document.createElement('a');a.href='/download?file='+encodeURIComponent(d.filename);a.download=d.filename;a.click()},500)
}).catch(e=>toast('请求失败','err'));
}
}

function snapshot(canvas){
var id=canvas==='main'?'':'Roi';
fetch('/'+canvas+'/snapshot')
.then(r=>r.blob()).then(blob=>{
var url=URL.createObjectURL(blob);
var a=document.createElement('a');
var ts=new Date().toISOString().replace(/[:.]/g,'-').slice(0,19);
a.href=url;a.download=canvas+'_'+ts+'.png';a.click();
URL.revokeObjectURL(url);toast('截图已保存','ok')
}).catch(e=>toast('截图失败','err'));
}

var seqIntervals={main:null,roi:null};

function toggleSeq(canvas){
var s=state[canvas],id=canvas==='main'?'':'Roi';
var btn=$('btnSeq'+(id?'Roi':'Main'));
if(!s.seq){
fetch('/'+canvas+'/seq/start')
.then(r=>r.json()).then(d=>{
if(d.success){
s.seq=true;s.seqCount=0;btn.classList.add('active');btn.textContent='⏹ 停止(0)';
toast('连续截图开始','ok');
}
})
.catch(e=>toast('请求失败','err'));
}else{
fetch('/'+canvas+'/seq/stop')
.then(r=>r.json()).then(d=>{
s.seq=false;btn.classList.remove('active');btn.textContent='🎞 连续截图';
if(seqIntervals[canvas]){clearInterval(seqIntervals[canvas]);seqIntervals[canvas]=null}
toast('已打包 '+d.count+' 张, 下载中...','ok');
if(d.filename)setTimeout(function(){var a=document.createElement('a');a.href='/download?file='+encodeURIComponent(d.filename);a.download=d.filename;a.click()},500)
})
.catch(e=>toast('请求失败','err'));
}
}

function pollSeqCount(){
['main','roi'].forEach(function(c){
if(!state[c].seq)return;
fetch('/'+c+'/seq/count').then(r=>r.json()).then(d=>{
state[c].seqCount=d.count||0;
var id=c==='main'?'':'Roi';
var btn=$('btnSeq'+(id?'Roi':'Main'));
if(btn)btn.textContent='⏹ 停止('+state[c].seqCount+')';
}).catch(function(){});
});
}
setInterval(pollSeqCount,800);

var imgMain=$('imgMain'),imgRoi=$('imgRoi');
function refreshStream(canvas,img){
var url='/'+canvas+'/stream?t='+Date.now();
fetch(url).then(r=>{
var ct=r.headers.get('content-type')||'';
if(ct.indexOf('multipart')>=0)return;
img.src=url
}).catch(function(){});
setTimeout(function(){refreshStream(canvas,img)},33);
}
function initMjpeg(canvas,img){
var url='/'+canvas+'/stream';
fetch(url).then(r=>r.body.getReader()).then(reader=>{
var buf=new Uint8Array(0),bd=[0xFF,0xD8];
function findSoi(a,f){for(var i=f;i<a.length-1;i++)if(a[i]==0xFF&&a[i+1]==0xD8)return i;return -1}
function pump(){reader.read().then(({done,value})=>{
if(done)return;
var t=new Uint8Array(buf.length+value.length);t.set(buf);t.set(value,buf.length);buf=t;
var s0=findSoi(buf,0);
while(s0>=0){
var s1=findSoi(buf,s0+2);
if(s1<0)break;
var jpeg=buf.slice(s0,s1);
var blob=new Blob([jpeg],{type:'image/jpeg'});
var oldSrc=img.src;
img.src=URL.createObjectURL(blob);
if(oldSrc&&oldSrc.startsWith('blob:'))URL.revokeObjectURL(oldSrc);
s0=s1;
}
if(buf.length>300000)buf=buf.slice(buf.length-150000);
pump();
})}
pump();
}).catch(function(){img.src='/'+canvas+'/stream?t='+Date.now();setTimeout(function(){initMjpeg(canvas,img)},1000)})
}
initMjpeg('main',imgMain);initMjpeg('roi',imgRoi);

async function updateStats(){
try{
var r=await fetch('/stats');if(!r.ok)return;
var d=await r.json();
$('fpsMain').textContent=(d.mainFps||0).toFixed(1)+' FPS';
$('fpsRoi').textContent=(d.roiFps||0).toFixed(1)+' FPS';
var lat=Number(d.latencyMs)||0;
$('latency').textContent=lat>0?lat+'':'--';
$('dotMain').className='dot '+(d.mainFps>0?'on':'off');
$('dotRoi').className='dot '+(d.roiFps>0?'on':'off');
}catch(e){}
}
setInterval(updateStats,1000);updateStats();

document.addEventListener('keydown',function(e){
if(e.target.tagName==='INPUT'||e.target.tagName==='TEXTAREA')return;
if(e.key==='1')toggleRec('main');else if(e.key==='2')snapshot('main');else if(e.key==='3')toggleSeq('main');
else if(e.key==='4')toggleRec('roi');else if(e.key==='5')snapshot('roi');else if(e.key==='6')toggleSeq('roi');
});
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
      latest_capture_ts_ms(0), ema_fps(0.0),
      roi_frame_id(0), roi_capture_ts_ms(0), roi_ema_fps(0.0),
      is_recording(false), frame_width(320), frame_height(240), frame_rate(30.0),
      use_jpeg_fallback(false), jpeg_frame_count(0),
      is_roi_recording(false), roi_use_jpeg_fallback(false), roi_jpeg_frame_count(0),
      is_capturing_seq(false), is_roi_capturing_seq(false),
      yuyv_w(0), yuyv_h(0), yuyv_frame_id(0) {
  pthread_mutex_init(&frame_mutex, NULL);
  pthread_cond_init(&frame_cond, NULL);
  pthread_mutex_init(&sock_mutex, NULL);
  pthread_mutex_init(&original_frame_mutex, NULL);
  pthread_mutex_init(&recording_mutex, NULL);
  pthread_mutex_init(&roi_frame_mutex, NULL);
  pthread_cond_init(&roi_frame_cond, NULL);
  pthread_mutex_init(&roi_recording_mutex, NULL);
  pthread_mutex_init(&seq_mutex, NULL);
  pthread_mutex_init(&roi_seq_mutex, NULL);
  pthread_mutex_init(&yuyv_mutex, NULL);
  pthread_cond_init(&yuyv_cond, NULL);
}

CameraStreamServer::~CameraStreamServer(void) {
  stop_server();
  if (is_recording) stop_recording(false);
  if (is_roi_recording) stop_recording(true);
  pthread_mutex_destroy(&frame_mutex);
  pthread_cond_destroy(&frame_cond);
  pthread_mutex_destroy(&sock_mutex);
  pthread_mutex_destroy(&original_frame_mutex);
  pthread_mutex_destroy(&recording_mutex);
  pthread_mutex_destroy(&roi_frame_mutex);
  pthread_cond_destroy(&roi_frame_cond);
  pthread_mutex_destroy(&roi_recording_mutex);
  pthread_mutex_destroy(&seq_mutex);
  pthread_mutex_destroy(&roi_seq_mutex);
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
  uint64_t cap_ts = latest_capture_ts_ms;
  uint64_t fid = 0, rfid = 0;
  pthread_mutex_lock(&frame_mutex); fid = latest_frame_id; pthread_mutex_unlock(&frame_mutex);
  pthread_mutex_lock(&roi_frame_mutex); rfid = roi_frame_id; pthread_mutex_unlock(&roi_frame_mutex);

  uint64_t srv_ts = now_ms();
  int64_t lat = (cap_ts > 0) ? static_cast<int64_t>(srv_ts - cap_ts) : -1;
  std::ostringstream body;
  body << std::fixed << std::setprecision(2)
       << "{\"mainFrameId\":" << fid << ",\"roiFrameId\":" << rfid
       << ",\"mainFps\":" << ema_fps << ",\"roiFps\":" << roi_ema_fps
       << ",\"latencyMs\":" << lat << ",\"serverTsMs\":" << srv_ts << "}";
  std::string json = body.str();
  send_response(sock, "application/json; charset=utf-8", json.c_str(), json.size());
}

void CameraStreamServer::send_mjpeg_stream(int sock, bool is_roi) {
  const char *header =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: close\r\n\r\n";
  send(sock, header, strlen(header), 0);
  pthread_mutex_t *pmut = is_roi ? &roi_frame_mutex : &frame_mutex;
  pthread_cond_t *pcond = is_roi ? &roi_frame_cond : &frame_cond;
  std::vector<unsigned char> *pjpeg = is_roi ? &roi_jpeg : &current_jpeg;
  uint64_t *pfid = is_roi ? &roi_frame_id : &latest_frame_id;

  uint64_t last_sent = 0;
  while (running) {
    std::vector<unsigned char> jpeg_copy;

   pthread_mutex_lock(pmut);
    while (running && *pfid == last_sent) { pthread_cond_wait(pcond, pmut); }
    if (!running) { pthread_mutex_unlock(pmut); break; }
    if (pjpeg->empty()) { pthread_mutex_unlock(pmut); continue; }
    jpeg_copy = *pjpeg;
    last_sent = *pfid;
    pthread_mutex_unlock(pmut);

    char boundary[256];
    snprintf(boundary, sizeof(boundary),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
        jpeg_copy.size());

    if (send(sock, boundary, strlen(boundary), 0) < 0) break;
    if (send(sock, jpeg_copy.data(), jpeg_copy.size(), 0) < 0) break;
    if (send(sock, "\r\n", 2, 0) < 0) break;
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
                                                 const std::string &prefix, bool is_roi) {
  pthread_mutex_lock(&original_frame_mutex);
  cv::Mat frame_copy = (is_roi ? roi_original_frame : original_frame).clone();
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

void CameraStreamServer::handle_panel_request(int sock, const std::string &path) {
  bool is_roi = (path.find("/roi/") == 0);
  std::string prefix = is_roi ? "roi" : "main";

  if (path.find("/stream") != std::string::npos) {
    send_mjpeg_stream(sock, is_roi);
  } else if (path.find("/snapshot") != std::string::npos) {
    handle_snapshot_request(sock, prefix + "_snap", is_roi);
  } else if (path.find("/record/start") != std::string::npos) {
    start_recording(prefix + "_rec", is_roi);
    std::ostringstream j; j<<"{\"success\":"<<(is_roi?(is_roi_recording?"true":"false"):(is_recording?"true":"false"))
        <<",\"filename\":\""<<(is_roi?roi_video_filename:video_filename)<<"\"}";
    std::string b=j.str(); send_response(sock,"application/json",b.c_str(),b.size());
  } else if (path.find("/record/stop") != std::string::npos) {
    std::string fn = is_roi ? roi_video_filename : video_filename;
    stop_recording(is_roi);
    std::ostringstream j; j<<"{\"success\":true,\"filename\":\""<<fn<<"\"}";
    std::string b=j.str(); send_response(sock,"application/json",b.c_str(),b.size());
  } else if (path.find("/seq/start") != std::string::npos) {
    pthread_mutex_t *m = is_roi ? &roi_seq_mutex : &seq_mutex;
    bool *f = is_roi ? &is_roi_capturing_seq : &is_capturing_seq;
    std::vector<std::vector<unsigned char>> *v = is_roi ? &roi_seq_frames : &seq_frames;
    pthread_mutex_lock(m); *f = true; v->clear(); pthread_mutex_unlock(m);
    send_response(sock, "application/json", "{\"success\":true}", 15);
  } else if (path.find("/seq/stop") != std::string::npos) {
    pthread_mutex_t *m = is_roi ? &roi_seq_mutex : &seq_mutex;
    bool *f = is_roi ? &is_roi_capturing_seq : &is_capturing_seq;
    std::vector<std::vector<unsigned char>> *v = is_roi ? &roi_seq_frames : &seq_frames;
    pthread_mutex_lock(m); *f = false; int cnt = (int)v->size(); std::string zip_fn;
    if (cnt > 0) {
      char tmp[256]; snprintf(tmp, sizeof(tmp), "/tmp/%s_seq_%llu", prefix.c_str(), (unsigned long long)now_ms());
      std::string dir(tmp); system(("mkdir -p " + dir).c_str());
      for (int i = 0; i < cnt; i++) {
        char fn[512]; snprintf(fn, sizeof(fn), "%s/%04d.jpg", dir.c_str(), i);
        FILE *fp = fopen(fn, "wb"); if(fp){ fwrite((*v)[i].data(),1,(*v)[i].size(),fp); fclose(fp); }
      }
      zip_fn = std::string(tmp) + ".zip";
      system(("cd /tmp && zip -j " + zip_fn + " " + dir + "/*.jpg 2>/dev/null").c_str());
      system(("rm -rf " + dir).c_str());
    }
    v->clear(); pthread_mutex_unlock(m);
    std::ostringstream j; j<<"{\"success\":true,\"count\":"<<cnt<<",\"filename\":\""<<zip_fn.substr(5)<<"\"}";
    std::string b=j.str(); send_response(sock,"application/json",b.c_str(),b.size());
  } else if (path.find("/seq/count") != std::string::npos) {
    pthread_mutex_t *m = is_roi ? &roi_seq_mutex : &seq_mutex;
    std::vector<std::vector<unsigned char>> *v = is_roi ? &roi_seq_frames : &seq_frames;
    pthread_mutex_lock(m); int c = (int)v->size(); pthread_mutex_unlock(m);
    std::ostringstream j; j<<"{\"count\":"<<c<<"}";
    std::string b=j.str(); send_response(sock,"application/json",b.c_str(),b.size());
  } else {
    send_response(sock, "text/html", "<h1>404</h1>", 13);
  }
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
  } else if (path.find("/roi/") == 0 || path.find("/main/") == 0) {
    handle_panel_request(sock, path);
  // ===== 旧版兼容路由 =====
  } else if (path.find("/stream") == 0) {
    send_mjpeg_stream(sock, false);
  } else if (path.find("/stats") == 0) {
    send_stats_response(sock);
  } else if (path.find("/snapshot") == 0) {
    handle_snapshot_request(sock, "snapshot", false);
  } else if (path.find("/record/start") == 0) {
    start_recording("video", false);
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
    stop_recording(false);

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

void CameraStreamServer::update_roi_frame(const cv::Mat &roi) {
  if (roi.empty()) return;

  uint64_t ts = now_ms();

  pthread_mutex_lock(&original_frame_mutex);
  roi_original_frame = roi.clone();
  pthread_mutex_unlock(&original_frame_mutex);

  static uint64_t last_ts = 0;
  static double fps_est = 0.0;
  if (last_ts != 0) {
    uint64_t d = ts - last_ts;
    if (d > 0) {
      double instant = 1000.0 / static_cast<double>(d);
      fps_est = fps_est <= 0.0 ? instant : 0.85 * fps_est + 0.15 * instant;
      roi_ema_fps = fps_est;
    }
  }
  last_ts = ts;

  std::vector<unsigned char> buf;
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 30};
  if (cv::imencode(".jpg", roi, buf, params)) {
    roi_capture_ts_ms = ts;
    pthread_mutex_lock(&roi_frame_mutex);
    roi_jpeg.swap(buf);
    ++roi_frame_id;
    pthread_cond_broadcast(&roi_frame_cond);
    pthread_mutex_unlock(&roi_frame_mutex);
  }

  if (is_roi_recording) {
    record_frame(roi, true);
  }

  pthread_mutex_lock(&roi_seq_mutex);
  if (is_roi_capturing_seq && !buf.empty()) {
    roi_seq_frames.push_back(buf);
  }
  pthread_mutex_unlock(&roi_seq_mutex);

  pthread_mutex_lock(&seq_mutex);
  if (is_capturing_seq) {
    std::vector<unsigned char> full_buf;
    cv::Mat display_snap;
    pthread_mutex_lock(&original_frame_mutex);
    display_snap = original_frame.clone();
    pthread_mutex_unlock(&original_frame_mutex);
    if (!display_snap.empty() && cv::imencode(".jpg", display_snap, full_buf, params)) {
      seq_frames.push_back(full_buf);
    }
  }
  pthread_mutex_unlock(&seq_mutex);
}

void CameraStreamServer::start_recording(const std::string &prefix, bool is_roi) {
  pthread_mutex_t *mut = is_roi ? &roi_recording_mutex : &recording_mutex;
  bool *rec_flag = is_roi ? &is_roi_recording : &is_recording;
  cv::VideoWriter *vw = is_roi ? &roi_video_writer : &video_writer;
  bool *use_fb = is_roi ? &roi_use_jpeg_fallback : &use_jpeg_fallback;
  std::string *jdir = is_roi ? &roi_jpeg_dir : &jpeg_dir;
  std::string *jbase = is_roi ? &roi_jpeg_base_name : &jpeg_base_name;
  int *jcnt = is_roi ? &roi_jpeg_frame_count : &jpeg_frame_count;
  std::string *fname = is_roi ? &roi_video_filename : &video_filename;
  int w = is_roi ? roi_original_frame.cols : frame_width;
  int h = is_roi ? roi_original_frame.rows : frame_height;
  const char *tag = is_roi ? "[ROI录制]" : "[录制]";

  if (w <= 0 || h <= 0) {
    std::cerr << tag << " 帧尺寸无效(" << w << "x" << h << ")，跳过录制" << std::endl;
    return;
  }

  pthread_mutex_lock(mut);
  if (*rec_flag) { pthread_mutex_unlock(mut); std::cout << tag << " 已经在录制中" << std::endl; return; }

  time_t now = time(NULL);
  struct tm tm_time; localtime_r(&now, &tm_time);
  char name_buf[256];
  snprintf(name_buf, sizeof(name_buf), "%s_%04d%02d%02d_%02d%02d%02d",
           prefix.c_str(), tm_time.tm_year + 1900, tm_time.tm_mon + 1,
           tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
  std::string base = std::string(name_buf), filepath = "/tmp/" + base + ".avi";

  int fourcc_list[] = {cv::VideoWriter::fourcc('M','J','P','G'),
                       cv::VideoWriter::fourcc('X','V','I','D'),
                       cv::VideoWriter::fourcc('D','I','V','X'),
                       cv::VideoWriter::fourcc('F','L','V','1'), -1};
  const char *fourcc_nm[] = {"MJPG","XVID","DIVX","FLV1","AUTO"};
  cv::Mat test(h, w, CV_8UC3, cv::Scalar(0,0,0));
  bool ok = false;
  for (int i = 0; i < 5; i++) {
    vw->open(filepath, fourcc_list[i], frame_rate, cv::Size(w, h), true);
    if (!vw->isOpened()) { std::cerr << tag << " 编码 " << fourcc_nm[i] << " 不可用" << std::endl; continue; }
    vw->write(test); vw->write(test); vw->release();
    std::ifstream in(filepath, std::ios::binary|std::ios::ate);
    if (in && in.tellg() > 100) { in.close();
      vw->open(filepath, fourcc_list[i], frame_rate, cv::Size(w, h), true);
      std::cout << tag << " 使用编码: " << fourcc_nm[i] << std::endl; ok = true; break; }
    in.close(); ::remove(filepath.c_str());
    std::cerr << tag << " 编码 " << fourcc_nm[i] << " 写入无效" << std::endl;
  }
  if (ok) { *use_fb = false; *fname = base + ".avi"; *rec_flag = true; pthread_mutex_unlock(mut);
    std::cout << tag << " ✅ 已开始: " << *fname << std::endl; return; }

  *jdir = "/tmp/" + base + "_frames/";
  std::string cmd = "mkdir -p " + *jdir;
  if (system(cmd.c_str()) != 0) { *rec_flag = false; *use_fb = false; pthread_mutex_unlock(mut);
    std::cerr << tag << " 无法创建帧目录" << std::endl; return; }
  *use_fb = true; *jcnt = 0; *fname = base + "_frames/"; *jbase = base; *rec_flag = true;
  pthread_mutex_unlock(mut);
  std::cout << tag << " ⚠ AVI 不可用，回退 JPEG: " << base << "_frames/" << std::endl;
}

void CameraStreamServer::stop_recording(bool is_roi) {
  pthread_mutex_t *mut = is_roi ? &roi_recording_mutex : &recording_mutex;
  bool *rec_flag = is_roi ? &is_roi_recording : &is_recording;
  cv::VideoWriter *vw = is_roi ? &roi_video_writer : &video_writer;
  bool *use_fb = is_roi ? &roi_use_jpeg_fallback : &use_jpeg_fallback;
  std::string *fname = is_roi ? &roi_video_filename : &video_filename;
  int *jcnt = is_roi ? &roi_jpeg_frame_count : &jpeg_frame_count;
  const char *tag = is_roi ? "[ROI录制]" : "[录制]";

  pthread_mutex_lock(mut);
  if (!*rec_flag) { pthread_mutex_unlock(mut); return; }
  *rec_flag = false;
  if (!*use_fb) vw->release();
  std::string saved = *fname;
  bool was_jpeg = *use_fb;
  int total = *jcnt;
  fname->clear(); *use_fb = false; *jcnt = 0;
  pthread_mutex_unlock(mut);

  if (was_jpeg) {
    std::cout << tag << " ⏹ 共 " << total << " 帧: " << saved << std::endl;
  } else {
    std::string fp = "/tmp/" + saved;
    std::ifstream in(fp, std::ios::binary | std::ios::ate);
    if (in) { long sz = in.tellg(); in.close();
      std::cout << tag << " ⏹ 已保存: " << saved << " (" << (sz/1024.0/1024.0) << " MB)" << std::endl; }
  }
}

void CameraStreamServer::record_frame(const cv::Mat &frame, bool is_roi) {
  if (frame.empty() || frame.cols <= 0 || frame.rows <= 0) return;
  if (!frame.data) return;

  pthread_mutex_t *mut = is_roi ? &roi_recording_mutex : &recording_mutex;
  bool *rec_flag = is_roi ? &is_roi_recording : &is_recording;
  cv::VideoWriter *vw = is_roi ? &roi_video_writer : &video_writer;
  bool *use_fb = is_roi ? &roi_use_jpeg_fallback : &use_jpeg_fallback;
  std::string *jdir = is_roi ? &roi_jpeg_dir : &jpeg_dir;
  int *jcnt = is_roi ? &roi_jpeg_frame_count : &jpeg_frame_count;

  pthread_mutex_lock(mut);
  if (!*rec_flag) { pthread_mutex_unlock(mut); return; }

  if (*use_fb) {
    char fn[512];
    snprintf(fn, sizeof(fn), "%s/frame_%06d.jpg", jdir->c_str(), (*jcnt)++);
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    cv::imwrite(fn, frame, params);
  } else if (vw->isOpened()) {
    vw->write(frame);
  }
  pthread_mutex_unlock(mut);
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