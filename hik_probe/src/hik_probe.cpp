// hik_probe.cpp —— 海康 USB3 相机 SDK 探针程序（不依赖 ROS）
//
// 目的：在虚拟相机和真机上各运行一次，自动完成 sdk_notes.md 中 U1/U2/U3/U5/U8/U10/U12/U15 等
//       需要实测的项目，把原始数据写成 CSV / 文本，供 analyze.py 分析、对比两台机器的结果。
//
// 安全性：
//   - 只读取和临时修改相机参数，不执行 UserSetSave，断电即恢复；
//   - 结束时默认把修改过的参数恢复成原值（可用 --no-restore 关闭）；
//   - 节点表导出只读取值，不会执行任何 Command 节点（DeviceReset 等）。
//
// 用法见 README.md。

#include "MvCameraControl.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <sys/utsname.h>
#include <unistd.h>

// ----------------------------------------------------------------------------
// 时间
// ----------------------------------------------------------------------------
static int64_t now_ns(clockid_t c) {
  timespec ts{};
  clock_gettime(c, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
struct Clocks { int64_t mono, raw, real; };
static Clocks clocks_now() {
  Clocks c;
  c.mono = now_ns(CLOCK_MONOTONIC);
  c.raw = now_ns(CLOCK_MONOTONIC_RAW);
  c.real = now_ns(CLOCK_REALTIME);
  return c;
}
static void sleep_until_mono(int64_t t_ns) {
  timespec ts{};
  ts.tv_sec = t_ns / 1000000000LL;
  ts.tv_nsec = t_ns % 1000000000LL;
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
}
static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// ----------------------------------------------------------------------------
// 日志
// ----------------------------------------------------------------------------
static FILE* g_log = nullptr;
static std::mutex g_log_mtx;
static void LOG(const char* fmt, ...) {
  char buf[2048];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  std::lock_guard<std::mutex> lk(g_log_mtx);
  fprintf(stdout, "%s\n", buf);
  fflush(stdout);
  if (g_log) { fprintf(g_log, "%s\n", buf); fflush(g_log); }
}
static std::string hexs(unsigned int v) { char b[16]; snprintf(b, sizeof(b), "0x%08X", v); return b; }
static std::string rets(int r) { return hexs(static_cast<unsigned int>(r)); }

// CSV：所有字段加引号，内部引号加倍
static std::string q(const std::string& s) {
  std::string o = "\"";
  for (char ch : s) { if (ch == '"') o += "\"\""; else o += ch; }
  return o + "\"";
}
static std::string fstr(double v) { char b[64]; snprintf(b, sizeof(b), "%.6f", v); return b; }
static std::string istr(long long v) { return std::to_string(v); }

// ----------------------------------------------------------------------------
// 参数
// ----------------------------------------------------------------------------
struct Options {
  std::string out_root = "./probe_out";
  std::string label = "";          // 例如 laptop_virtual / lab_real
  double fps = 60.0;
  int duration_s = 60;             // 双相机同时取流的时长
  double exposure_us = 5000.0;
  int static_s = 10;               // 静态对时测试时长（U1）
  double latch_hz = 20.0;          // 取流中对时频率（U2/U3/U12）
  int node_num = 30;               // SDK 缓存节点数
  int max_cams = 2;
  std::vector<std::string> ids;    // 指定 User ID 或序列号
  bool restore = true;
};

static void usage() {
  printf(
      "用法: hik_probe [选项]\n"
      "  --label NAME        本次运行的标签，如 laptop_virtual / lab_real（写进目录名）\n"
      "  --out DIR           输出根目录（默认 ./probe_out）\n"
      "  --ids A,B           只测试这些相机（按 User ID 或序列号匹配，逗号分隔）\n"
      "  --fps F             自由运行帧率（默认 60）\n"
      "  --duration S        双相机取流时长，秒（默认 60）\n"
      "  --exposure US       曝光时间，微秒（默认 5000）\n"
      "  --static S          静态对时测试时长，秒（默认 10）\n"
      "  --latch-hz F        取流中对时频率（默认 20）\n"
      "  --node-num N        SDK 图像缓存节点数（默认 30）\n"
      "  --max-cams N        最多测试几台相机（默认 2）\n"
      "  --no-restore        结束时不恢复相机参数\n");
}

static bool parse_args(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* n) -> const char* {
      if (i + 1 >= argc) { fprintf(stderr, "%s 缺少参数\n", n); exit(2); }
      return argv[++i];
    };
    if (a == "--label") o.label = need("--label");
    else if (a == "--out") o.out_root = need("--out");
    else if (a == "--fps") o.fps = atof(need("--fps"));
    else if (a == "--duration") o.duration_s = atoi(need("--duration"));
    else if (a == "--exposure") o.exposure_us = atof(need("--exposure"));
    else if (a == "--static") o.static_s = atoi(need("--static"));
    else if (a == "--latch-hz") o.latch_hz = atof(need("--latch-hz"));
    else if (a == "--node-num") o.node_num = atoi(need("--node-num"));
    else if (a == "--max-cams") o.max_cams = atoi(need("--max-cams"));
    else if (a == "--no-restore") o.restore = false;
    else if (a == "--ids") {
      std::string s = need("--ids"), cur;
      for (char ch : s) { if (ch == ',') { if (!cur.empty()) o.ids.push_back(cur); cur.clear(); } else cur += ch; }
      if (!cur.empty()) o.ids.push_back(cur);
    } else if (a == "-h" || a == "--help") { usage(); exit(0); }
    else { fprintf(stderr, "未知参数: %s\n", a.c_str()); usage(); return false; }
  }
  if (o.fps <= 0 || o.duration_s <= 0 || o.latch_hz <= 0 || o.node_num < 1) { fprintf(stderr, "参数取值不合法\n"); return false; }
  return true;
}

// ----------------------------------------------------------------------------
// 停止信号
// ----------------------------------------------------------------------------
static std::atomic<bool> g_abort{false};
static void on_sigint(int) { g_abort = true; }

// ----------------------------------------------------------------------------
// 记录结构
// ----------------------------------------------------------------------------
struct FrameRec {
  unsigned int frame_num, frame_counter, trigger_index, lost_packet, width, height, pixel_type, frame_len;
  uint64_t dev_ts;
  int64_t host_ts;     // SDK 的 nHostTimeStamp，单位未知（U5）
  Clocks recv;         // GetImageBuffer 返回后立即读取的三种系统时钟
  float exposure, gain;
};
struct LatchRec {
  std::string mode;    // latch / nolatch / invalidate / run
  int idx;
  Clocks t0;
  int64_t t1_mono;
  int ret_cmd, ret_read;
  int64_t dev_ts;
};
struct StatusRec {
  int64_t mono;
  unsigned int valid_images;
  int ret_valid;
  double resulting_fps, temperature;
  long long link_throughput, queue_blocks;
  unsigned int usb_recv_frames, usb_err_frames;
  int ret_match;
};

struct Camera {
  int index = 0;
  MV_CC_DEVICE_INFO info{};      // 按值拷贝（枚举结果会被下次枚举覆盖）
  std::string serial, user_id, model, tag;
  bool is_virtual = false;
  void* h = nullptr;
  bool opened = false, grabbing = false;
  std::mutex ctl;                // 控制类调用串行化
  std::atomic<bool> disconnected{false};
  std::atomic<size_t> nframes{0};          // 供主线程打印进度（避免读 vector 大小的数据竞争）
  std::atomic<unsigned int> stream_exc[16];
  std::vector<FrameRec> frames;
  std::vector<LatchRec> latch_run;
  std::vector<StatusRec> status;
  std::map<unsigned int, int> grab_errors;  // 返回码 -> 次数
  std::vector<std::string> config_rows;     // CSV 行
  struct Saved { std::string name; char type; double f; bool b; std::string s; bool ok; };
  std::vector<Saved> saved;                 // 用于恢复
  Camera() { for (auto& x : stream_exc) x = 0; }
};

// ----------------------------------------------------------------------------
// 工具
// ----------------------------------------------------------------------------
static std::string cstr64(const unsigned char* p) {
  size_t n = strnlen(reinterpret_cast<const char*>(p), INFO_MAX_BUFFER_SIZE);
  return std::string(reinterpret_cast<const char*>(p), n);
}
static std::string safe_name(const std::string& s) {
  std::string o;
  for (char ch : s) o += (isalnum(static_cast<unsigned char>(ch)) || ch == '-' || ch == '_') ? ch : '_';
  return o.empty() ? "cam" : o;
}
static bool mkdir_p(const std::string& path) {
  std::string cur;
  for (size_t i = 0; i < path.size(); ++i) {
    cur += path[i];
    if (path[i] == '/' || i + 1 == path.size()) {
      if (cur == "/" || cur == "./" || cur == ".") continue;
      if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }
  }
  return true;
}
static FILE* open_csv(const std::string& path, const std::string& header) {
  FILE* f = fopen(path.c_str(), "w");
  if (!f) { LOG("[错误] 无法写入 %s", path.c_str()); return nullptr; }
  fprintf(f, "%s\n", header.c_str());
  return f;
}
static std::string access_name(int m) {
  switch (m) {
    case AM_NI: return "NI(未实现)";
    case AM_NA: return "NA(不可用)";
    case AM_WO: return "WO";
    case AM_RO: return "RO";
    case AM_RW: return "RW";
    default: return "其他(" + std::to_string(m) + ")";
  }
}

// 读取任意类型节点为文本。type: I F E B S C；返回 SDK 返回码
static int read_node(void* h, const char* name, char type, std::string& val, std::string& range) {
  val.clear(); range.clear();
  int r = 0;
  switch (type) {
    case 'I': {
      MVCC_INTVALUE_EX v{}; r = MV_CC_GetIntValueEx(h, name, &v);
      if (r == MV_OK) { val = istr(v.nCurValue); range = "min=" + istr(v.nMin) + " max=" + istr(v.nMax) + " inc=" + istr(v.nInc); }
      break;
    }
    case 'F': {
      MVCC_FLOATVALUE v{}; r = MV_CC_GetFloatValue(h, name, &v);
      if (r == MV_OK) { val = fstr(v.fCurValue); range = "min=" + fstr(v.fMin) + " max=" + fstr(v.fMax); }
      break;
    }
    case 'E': {
      MVCC_ENUMVALUE_EX v{}; r = MV_CC_GetEnumValueEx(h, name, &v);
      if (r == MV_OK) {
        MVCC_ENUMENTRY e{}; e.nValue = v.nCurValue;
        std::string sym = (MV_CC_GetEnumEntrySymbolic(h, name, &e) == MV_OK) ? std::string(e.chSymbolic) : "?";
        val = sym + "(" + std::to_string(v.nCurValue) + ")";
        for (unsigned int k = 0; k < v.nSupportedNum && k < MV_MAX_ENUM_SYMBOLIC_NUM; ++k) {
          MVCC_ENUMENTRY e2{}; e2.nValue = v.nSupportValue[k];
          std::string s2 = (MV_CC_GetEnumEntrySymbolic(h, name, &e2) == MV_OK) ? std::string(e2.chSymbolic) : "?";
          range += (k ? "|" : "") + s2 + "=" + std::to_string(v.nSupportValue[k]);
        }
      }
      break;
    }
    case 'B': {
      bool b = false; r = MV_CC_GetBoolValue(h, name, &b);
      if (r == MV_OK) val = b ? "true" : "false";
      break;
    }
    case 'S': {
      MVCC_STRINGVALUE v{}; r = MV_CC_GetStringValue(h, name, &v);
      if (r == MV_OK) { val = std::string(v.chCurValue, strnlen(v.chCurValue, sizeof(v.chCurValue))); range = "maxlen=" + istr(v.nMaxLength); }
      break;
    }
    default: r = -1;  // Command：不读取、不执行
  }
  return r;
}

// ----------------------------------------------------------------------------
// 回调
// ----------------------------------------------------------------------------
static void __stdcall on_exception(unsigned int msg, void* user) {
  auto* c = static_cast<Camera*>(user);
  c->disconnected = true;
  LOG("[异常回调] 相机 %s 收到异常 %s", c->tag.c_str(), hexs(msg).c_str());
}
static void __stdcall on_stream_exception(MV_CC_STREAM_EXCEPTION_INFO* p, void* user) {
  auto* c = static_cast<Camera*>(user);
  unsigned int t = p ? static_cast<unsigned int>(p->enExceptionType) : 0;
  unsigned int idx = (t >= 0x4001 && t <= 0x400F) ? (t - 0x4000) : 0;
  c->stream_exc[idx]++;
}

// ----------------------------------------------------------------------------
// 节点清单（导出时只读，不执行 Command）
// ----------------------------------------------------------------------------
struct NodeSpec { const char* name; char type; };
static const NodeSpec kNodes[] = {
    {"DeviceModelName", 'S'}, {"DeviceSerialNumber", 'S'}, {"DeviceUserID", 'S'}, {"DeviceVendorName", 'S'},
    {"DeviceVersion", 'S'}, {"DeviceFirmwareVersion", 'S'}, {"DeviceScanType", 'E'},
    {"DeviceTimestamp", 'I'}, {"DeviceTimestampLatch", 'C'}, {"DeviceTimestampIncrement", 'I'},
    {"DeviceTimestampReset", 'C'}, {"TimestampReset", 'C'}, {"TimestampLatch", 'C'}, {"TimestampLatchValue", 'I'},
    {"GevTimestampTickFrequency", 'I'}, {"GevTimestampValue", 'I'}, {"GevTimestampControlLatch", 'C'}, {"GevTimestampControlReset", 'C'},
    {"DeviceUptime", 'I'}, {"DeviceTemperatureSelector", 'E'}, {"DeviceTemperature", 'F'}, {"DeviceReset", 'C'}, {"FindMe", 'C'},
    {"USBSpeedMode", 'E'}, {"U3VCurrentSpeed", 'I'}, {"DeviceConnectionSpeed", 'I'}, {"DeviceLinkSpeed", 'I'},
    {"DeviceMaxThroughput", 'I'}, {"DeviceLinkThroughputLimitMode", 'E'}, {"DeviceLinkThroughputLimit", 'I'},
    {"DeviceLinkCurrentThroughput", 'I'}, {"TransferQueueMaxBlockCount", 'I'}, {"TransferQueueCurrentBlockCount", 'I'},
    {"TransferQueueOverFlowCount", 'I'}, {"PayloadSize", 'I'},
    {"WidthMax", 'I'}, {"HeightMax", 'I'}, {"Width", 'I'}, {"Height", 'I'}, {"OffsetX", 'I'}, {"OffsetY", 'I'},
    {"ReverseX", 'B'}, {"ReverseY", 'B'}, {"PixelFormat", 'E'}, {"PixelSize", 'E'}, {"ADCBitDepth", 'E'},
    {"BinningHorizontal", 'E'}, {"BinningVertical", 'E'}, {"DecimationHorizontal", 'E'}, {"DecimationVertical", 'E'},
    {"TestPattern", 'E'}, {"FrameSpecInfoSelector", 'E'}, {"FrameSpecInfo", 'B'}, {"ChunkModeActive", 'B'},
    {"AcquisitionMode", 'E'}, {"AcquisitionBurstFrameCount", 'I'}, {"AcquisitionFrameRate", 'F'},
    {"AcquisitionFrameRateEnable", 'B'}, {"ResultingFrameRate", 'F'},
    {"TriggerSelector", 'E'}, {"TriggerMode", 'E'}, {"TriggerSource", 'E'}, {"TriggerActivation", 'E'},
    {"TriggerDelay", 'F'}, {"TriggerCacheEnable", 'B'}, {"TriggerSoftware", 'C'},
    {"ExposureMode", 'E'}, {"ExposureTimeMode", 'E'}, {"ExposureTime", 'F'}, {"ExposureAuto", 'E'},
    {"AutoExposureTimeLowerLimit", 'I'}, {"AutoExposureTimeUpperLimit", 'I'},
    {"Gain", 'F'}, {"GainAuto", 'E'}, {"AutoGainLowerLimit", 'F'}, {"AutoGainUpperLimit", 'F'},
    {"BalanceWhiteAuto", 'E'}, {"BalanceRatioSelector", 'E'}, {"BalanceRatio", 'I'},
    {"BlackLevel", 'F'}, {"BlackLevelEnable", 'B'}, {"DigitalShift", 'F'}, {"DigitalShiftEnable", 'B'},
    {"Gamma", 'F'}, {"GammaSelector", 'E'}, {"GammaEnable", 'B'}, {"Sharpness", 'I'}, {"SharpnessEnable", 'B'},
    {"DigitalNoiseReductionMode", 'E'}, {"ContrastRatio", 'I'}, {"ContrastRatioEnable", 'B'},
    {"LUTEnable", 'B'}, {"SequencerMode", 'E'},
    {"LineSelector", 'E'}, {"LineMode", 'E'}, {"LineSource", 'E'}, {"LineStatus", 'B'}, {"LineDebouncerTime", 'I'},
    {"EventSelector", 'E'}, {"EventNotification", 'E'},
    {"UserSetCurrent", 'I'}, {"UserSetSelector", 'E'}, {"UserSetDefault", 'E'},
};

static void dump_nodes(Camera& c, const std::string& path) {
  FILE* f = open_csv(path, "node,expected_type,access_mode_ret,access_mode,read_type,read_ret,value,range");
  if (!f) return;
  const char types[] = {'I', 'F', 'E', 'B', 'S'};
  for (const auto& n : kNodes) {
    enum MV_XML_AccessMode am = AM_NI;
    int ram = MV_XML_GetNodeAccessMode(c.h, n.name, &am);
    std::string am_s = (ram == MV_OK) ? access_name(static_cast<int>(am)) : "";
    std::string val, range, read_type;
    int rr = -1;
    if (n.type != 'C') {
      rr = read_node(c.h, n.name, n.type, val, range);
      read_type = std::string(1, n.type);
      if (rr != MV_OK) {  // 类型可能和预期不同（如 BlackLevel），依次尝试其他类型
        for (char t : types) {
          if (t == n.type) continue;
          std::string v2, r2;
          if (read_node(c.h, n.name, t, v2, r2) == MV_OK) { rr = MV_OK; val = v2; range = r2; read_type = std::string(1, t) + "(与预期不同)"; break; }
        }
      }
    } else {
      read_type = "C(不执行)";
    }
    fprintf(f, "%s,%s,%s,%s,%s,%s,%s,%s\n", q(n.name).c_str(), q(std::string(1, n.type)).c_str(), q(rets(ram)).c_str(),
            q(am_s).c_str(), q(read_type).c_str(), q(n.type == 'C' ? "" : rets(rr)).c_str(), q(val).c_str(), q(range).c_str());
  }
  fclose(f);
}

static void export_xml(Camera& c, const std::string& base) {
  std::vector<unsigned char> buf(64u * 1024u * 1024u);
  unsigned int len = 0;
  int r = MV_XML_GetGenICamXML(c.h, buf.data(), static_cast<unsigned int>(buf.size()), &len);
  if (r != MV_OK || len == 0) { LOG("  [XML] 导出失败 ret=%s len=%u", rets(r).c_str(), len); return; }
  bool zip = len >= 2 && buf[0] == 'P' && buf[1] == 'K';
  std::string path = base + (zip ? ".zip" : ".xml");
  FILE* f = fopen(path.c_str(), "wb");
  if (f) { fwrite(buf.data(), 1, len, f); fclose(f); LOG("  [XML] 已导出 %s（%u 字节）", path.c_str(), len); }
}

// 一次对时采样：t0 → Latch → 读 DeviceTimestamp → t1
static LatchRec latch_once(Camera& c, const std::string& mode, int idx) {
  LatchRec L{};
  L.mode = mode; L.idx = idx;
  MVCC_INTVALUE_EX v{};
  L.t0 = clocks_now();
  if (mode == "nolatch") L.ret_cmd = 0;
  else if (mode == "invalidate") L.ret_cmd = MV_CC_InvalidateNodes(c.h);
  else L.ret_cmd = MV_CC_SetCommandValue(c.h, "DeviceTimestampLatch");
  L.ret_read = MV_CC_GetIntValueEx(c.h, "DeviceTimestamp", &v);
  L.t1_mono = now_ns(CLOCK_MONOTONIC);
  L.dev_ts = (L.ret_read == MV_OK) ? v.nCurValue : -1;
  return L;
}
static std::string latch_row(const std::string& cam, const LatchRec& L) {
  return q(cam) + "," + q(L.mode) + "," + istr(L.idx) + "," + istr(L.t0.mono) + "," + istr(L.t1_mono) + "," + istr(L.t0.raw) + "," +
         istr(L.t0.real) + "," + istr(L.t1_mono - L.t0.mono) + "," + q(rets(L.ret_cmd)) + "," + q(rets(L.ret_read)) + "," + istr(L.dev_ts);
}
static const char* kLatchHeader = "cam,mode,idx,t0_mono_ns,t1_mono_ns,t0_raw_ns,t0_real_ns,rtt_ns,ret_cmd,ret_read,dev_ts";

// 配置记录
static void cfg(Camera& c, const std::string& phase, const std::string& node, const std::string& action, int ret, const std::string& readback) {
  c.config_rows.push_back(q(c.tag) + "," + q(phase) + "," + q(node) + "," + q(action) + "," + q(rets(ret)) + "," + q(readback));
  LOG("  [%s] %-28s %-22s ret=%s  读回=%s", phase.c_str(), node.c_str(), action.c_str(), rets(ret).c_str(), readback.c_str());
}
static std::string rb(Camera& c, const char* name, char t) {
  std::string v, r;
  int ret = read_node(c.h, name, t, v, r);
  return ret == MV_OK ? (v + (r.empty() ? "" : " [" + r + "]")) : ("读失败 " + rets(ret));
}
static void save_for_restore(Camera& c, const char* name, char t) {
  Camera::Saved s{name, t, 0, false, "", false};
  if (t == 'F') { MVCC_FLOATVALUE v{}; if (MV_CC_GetFloatValue(c.h, name, &v) == MV_OK) { s.f = v.fCurValue; s.ok = true; } }
  else if (t == 'B') { bool b; if (MV_CC_GetBoolValue(c.h, name, &b) == MV_OK) { s.b = b; s.ok = true; } }
  else if (t == 'E') {
    MVCC_ENUMVALUE_EX v{};
    if (MV_CC_GetEnumValueEx(c.h, name, &v) == MV_OK) {
      MVCC_ENUMENTRY e{}; e.nValue = v.nCurValue;
      if (MV_CC_GetEnumEntrySymbolic(c.h, name, &e) == MV_OK) { s.s = e.chSymbolic; s.ok = true; }
    }
  }
  c.saved.push_back(s);
}

// ----------------------------------------------------------------------------
// 线程
// ----------------------------------------------------------------------------
static std::atomic<bool> g_stop_grab{false};

static void grab_thread(Camera* c) {
  while (!g_stop_grab && !g_abort) {
    MV_FRAME_OUT fr;
    memset(&fr, 0, sizeof(fr));
    int r = MV_CC_GetImageBuffer(c->h, &fr, 200);
    Clocks rc = clocks_now();
    if (r == MV_OK) {
      const MV_FRAME_OUT_INFO_EX& in = fr.stFrameInfo;
      FrameRec f{};
      f.frame_num = in.nFrameNum; f.frame_counter = in.nFrameCounter; f.trigger_index = in.nTriggerIndex;
      f.lost_packet = in.nLostPacket;
      f.width = in.nExtendWidth ? in.nExtendWidth : in.nWidth;
      f.height = in.nExtendHeight ? in.nExtendHeight : in.nHeight;
      f.pixel_type = static_cast<unsigned int>(in.enPixelType);
      f.frame_len = in.nFrameLen;
      f.dev_ts = (static_cast<uint64_t>(in.nDevTimeStampHigh) << 32) | in.nDevTimeStampLow;
      f.host_ts = in.nHostTimeStamp;
      f.recv = rc;
      f.exposure = in.fExposureTime; f.gain = in.fGain;
      c->frames.push_back(f);
      c->nframes++;
      MV_CC_FreeImageBuffer(c->h, &fr);  // 只在成功取图后释放，与示例不同（见笔记 E 节）
    } else {
      c->grab_errors[static_cast<unsigned int>(r)]++;
    }
  }
}

static void control_thread(Camera* c, double latch_hz) {
  const int64_t period = static_cast<int64_t>(1e9 / latch_hz);
  int64_t next = now_ns(CLOCK_MONOTONIC);
  int64_t next_status = next;
  int idx = 0;
  while (!g_stop_grab && !g_abort) {
    {
      std::lock_guard<std::mutex> lk(c->ctl);
      c->latch_run.push_back(latch_once(*c, "run", idx++));
      int64_t t = now_ns(CLOCK_MONOTONIC);
      if (t >= next_status) {  // 每秒一次状态
        StatusRec s{};
        s.mono = t;
        s.ret_valid = MV_CC_GetValidImageNum(c->h, &s.valid_images);
        MVCC_FLOATVALUE fv{};
        s.resulting_fps = (MV_CC_GetFloatValue(c->h, "ResultingFrameRate", &fv) == MV_OK) ? fv.fCurValue : NAN;
        s.temperature = (MV_CC_GetFloatValue(c->h, "DeviceTemperature", &fv) == MV_OK) ? fv.fCurValue : NAN;
        MVCC_INTVALUE_EX iv{};
        s.link_throughput = (MV_CC_GetIntValueEx(c->h, "DeviceLinkCurrentThroughput", &iv) == MV_OK) ? iv.nCurValue : -1;
        s.queue_blocks = (MV_CC_GetIntValueEx(c->h, "TransferQueueCurrentBlockCount", &iv) == MV_OK) ? iv.nCurValue : -1;
        MV_MATCH_INFO_USB_DETECT usb{};
        MV_ALL_MATCH_INFO mi{};
        mi.nType = MV_MATCH_TYPE_USB_DETECT; mi.pInfo = &usb; mi.nInfoSize = sizeof(usb);
        s.ret_match = MV_CC_GetAllMatchInfo(c->h, &mi);
        s.usb_recv_frames = usb.nReceivedFrameCount; s.usb_err_frames = usb.nErrorFrameCount;
        c->status.push_back(s);
        next_status += 1000000000LL;
      }
    }
    next += period;
    sleep_until_mono(next);
  }
}

// ----------------------------------------------------------------------------
// 统计
// ----------------------------------------------------------------------------
static double percentile(std::vector<double> v, double p) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  double k = (v.size() - 1) * p;
  size_t i = static_cast<size_t>(k);
  double fr = k - i;
  return (i + 1 < v.size()) ? v[i] * (1 - fr) + v[i + 1] * fr : v[i];
}

// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------
int main(int argc, char** argv) {
  Options opt;
  if (!parse_args(argc, argv, opt)) return 2;
  signal(SIGINT, on_sigint);

  // 输出目录：<out>/<日期时间>_<主机名>_<标签>
  char host[128] = {0};
  gethostname(host, sizeof(host) - 1);
  time_t tt = time(nullptr);
  char tbuf[32];
  strftime(tbuf, sizeof(tbuf), "%Y%m%d_%H%M%S", localtime(&tt));
  std::string dir = opt.out_root + "/" + tbuf + "_" + safe_name(host) + (opt.label.empty() ? "" : "_" + safe_name(opt.label));
  if (!mkdir_p(dir)) { fprintf(stderr, "无法创建目录 %s\n", dir.c_str()); return 1; }
  g_log = fopen((dir + "/log.txt").c_str(), "w");
  LOG("输出目录：%s", dir.c_str());

  // ---- T0 环境 ----
  {
    FILE* f = fopen((dir + "/env.txt").c_str(), "w");
    utsname u{};
    uname(&u);
    unsigned int ver = MV_CC_GetSDKVersion();
    timespec res{};
    if (f) {
      fprintf(f, "hostname=%s\nlabel=%s\nuname=%s %s %s %s\n", host, opt.label.c_str(), u.sysname, u.release, u.version, u.machine);
      fprintf(f, "sdk_version=%s\n", hexs(ver).c_str());
      clock_getres(CLOCK_MONOTONIC, &res); fprintf(f, "res_CLOCK_MONOTONIC_ns=%ld\n", res.tv_nsec);
      clock_getres(CLOCK_REALTIME, &res); fprintf(f, "res_CLOCK_REALTIME_ns=%ld\n", res.tv_nsec);
      Clocks c = clocks_now();
      fprintf(f, "start_mono_ns=%lld\nstart_raw_ns=%lld\nstart_real_ns=%lld\n", (long long)c.mono, (long long)c.raw, (long long)c.real);
      fprintf(f, "options: fps=%.3f duration=%d exposure=%.1f static=%d latch_hz=%.1f node_num=%d max_cams=%d restore=%d\n",
              opt.fps, opt.duration_s, opt.exposure_us, opt.static_s, opt.latch_hz, opt.node_num, opt.max_cams, (int)opt.restore);
      const char* envs[] = {"LD_LIBRARY_PATH", "MVCAM_COMMON_RUNENV", "MVCAM_SDK_PATH"};
      for (auto e : envs) fprintf(f, "%s=%s\n", e, getenv(e) ? getenv(e) : "(未设置)");
      FILE* p = popen("lsusb -t 2>/dev/null", "r");
      if (p) { fprintf(f, "\n--- lsusb -t ---\n"); char line[512]; while (fgets(line, sizeof(line), p)) fputs(line, f); pclose(p); }
      fclose(f);
    }
    LOG("SDK 版本 %s", hexs(ver).c_str());
  }

  int r = MV_CC_Initialize();
  LOG("MV_CC_Initialize ret=%s", rets(r).c_str());
  if (r != MV_OK) return 1;

  // ---- T1 枚举（MV_USB_DEVICE 同时覆盖真机和虚拟相机，见笔记第 8 项）----
  std::vector<std::unique_ptr<Camera>> cams;
  {
    MV_CC_DEVICE_INFO_LIST list;
    memset(&list, 0, sizeof(list));
    r = MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    LOG("MV_CC_EnumDevices(MV_USB_DEVICE) ret=%s 数量=%u", rets(r).c_str(), list.nDeviceNum);
    FILE* f = open_csv(dir + "/devices.csv", "index,tlayer,model,serial,user_id,vendor,device_version,bcdUSB,is_virtual,selected");
    std::vector<MV_CC_DEVICE_INFO> copies;
    for (unsigned int i = 0; r == MV_OK && i < list.nDeviceNum; ++i) {
      if (list.pDeviceInfo[i]) copies.push_back(*list.pDeviceInfo[i]);  // 立即按值拷贝
    }
    for (size_t i = 0; i < copies.size(); ++i) {
      const auto& d = copies[i];
      const auto& u = d.SpecialInfo.stUsb3VInfo;  // nTLayerType 为 0x4 或 0x20 时都有效
      std::string serial = cstr64(u.chSerialNumber), uid = cstr64(u.chUserDefinedName), model = cstr64(u.chModelName);
      bool is_vir = serial.rfind("Vir", 0) == 0;
      bool want = opt.ids.empty() ? true
                                  : std::find_if(opt.ids.begin(), opt.ids.end(), [&](const std::string& s) { return s == serial || s == uid; }) != opt.ids.end();
      bool selected = want && static_cast<int>(cams.size()) < opt.max_cams;
      if (f) fprintf(f, "%zu,%s,%s,%s,%s,%s,%s,%s,%d,%d\n", i, q(hexs(d.nTLayerType)).c_str(), q(model).c_str(), q(serial).c_str(), q(uid).c_str(),
                     q(cstr64(u.chVendorName)).c_str(), q(cstr64(u.chDeviceVersion)).c_str(), q(hexs(u.nbcdUSB)).c_str(), is_vir, selected);
      LOG("  [%zu] 型号=%s 序列号=%s UserID=%s 传输层=%s %s%s", i, model.c_str(), serial.c_str(), uid.c_str(), hexs(d.nTLayerType).c_str(),
          is_vir ? "(虚拟)" : "(真机)", selected ? " ← 选中" : "");
      if (selected) {
        auto c = std::make_unique<Camera>();
        c->index = static_cast<int>(cams.size());
        c->info = d;
        c->serial = serial; c->user_id = uid; c->model = model; c->is_virtual = is_vir;
        c->tag = safe_name(uid.empty() ? serial : uid);
        for (auto& o : cams) if (o->tag == c->tag) c->tag += "_" + safe_name(serial);  // 防止 User ID 重名
        cams.push_back(std::move(c));
      }
    }
    if (f) fclose(f);
  }
  if (cams.empty()) {
    LOG("[错误] 没有可测试的相机。请检查连接、权限，或 MVS 客户端是否占用了相机。");
    MV_CC_Finalize();
    return 1;
  }

  // ---- 打开、导出、静态测试、配置 ----
  for (auto& cp : cams) {
    Camera& c = *cp;
    LOG("==== 相机 %s（%s %s）====", c.tag.c_str(), c.model.c_str(), c.serial.c_str());
    r = MV_CC_CreateHandle(&c.h, &c.info);
    LOG("  CreateHandle ret=%s", rets(r).c_str());
    if (r != MV_OK) { c.h = nullptr; continue; }
    r = MV_CC_OpenDevice(c.h);
    LOG("  OpenDevice ret=%s%s", rets(r).c_str(), r == static_cast<int>(0x80000203) ? "（相机被占用，先关闭 MVS 等程序）" : "");
    if (r != MV_OK) continue;
    c.opened = true;
    cfg(c, "callback", "RegisterExceptionCallBack", "register", MV_CC_RegisterExceptionCallBack(c.h, on_exception, &c), "");
    cfg(c, "callback", "RegisterStreamExceptionCallBack", "register", MV_CC_RegisterStreamExceptionCallBack(c.h, on_stream_exception, &c),
        c.is_virtual ? "虚拟相机预期不支持" : "");

    // T2 节点表、XML、参数文件（U10）
    std::string base = dir + "/" + c.tag;
    dump_nodes(c, base + "_nodes_before.csv");
    LOG("  节点表已导出 %s_nodes_before.csv", base.c_str());
    export_xml(c, base + "_genicam");
    { int rf = MV_CC_FeatureSave(c.h, (base + "_features.ini").c_str()); LOG("  FeatureSave ret=%s", rets(rf).c_str()); }

    // T3 静态时间戳测试（U1、U2）
    {
      FILE* f = open_csv(base + "_latch_static.csv", kLatchHeader);
      LOG("  [U2] 不 Latch 连续读 20 次");
      for (int i = 0; i < 20 && !g_abort; ++i) { auto L = latch_once(c, "nolatch", i); if (f) fprintf(f, "%s\n", latch_row(c.tag, L).c_str()); sleep_ms(10); }
      LOG("  [U2] InvalidateNodes 后读 20 次");
      for (int i = 0; i < 20 && !g_abort; ++i) { auto L = latch_once(c, "invalidate", i); if (f) fprintf(f, "%s\n", latch_row(c.tag, L).c_str()); sleep_ms(10); }
      LOG("  [U1] Latch+读，10 Hz，持续 %d 秒", opt.static_s);
      int n = opt.static_s * 10;
      int64_t next = now_ns(CLOCK_MONOTONIC);
      LatchRec first{}, last{};
      bool have_first = false;
      for (int i = 0; i < n && !g_abort; ++i) {
        auto L = latch_once(c, "latch", i);
        if (f) fprintf(f, "%s\n", latch_row(c.tag, L).c_str());
        if (L.ret_cmd == MV_OK && L.ret_read == MV_OK) { if (!have_first) { first = L; have_first = true; } last = L; }
        next += 100000000LL;
        sleep_until_mono(next);
      }
      if (f) fclose(f);
      if (have_first && last.t0.mono > first.t0.mono) {
        double dh = double((last.t0.mono + last.t1_mono) - (first.t0.mono + first.t1_mono)) / 2.0;
        double dd = double(last.dev_ts - first.dev_ts);
        LOG("  [U1] 设备差/主机差 = %.6f（≈1 说明单位是 ns；≈0 说明时间戳无效）", dd / dh);
      } else {
        LOG("  [U1] 没有成功的 Latch 采样（节点可能不存在，见 nodes 表）");
      }
    }

    // T4 配置自由运行（U8）
    const char* to_save[][2] = {{"TriggerMode", "E"}, {"AcquisitionMode", "E"}, {"ExposureAuto", "E"}, {"ExposureTime", "F"},
                                {"GainAuto", "E"}, {"AcquisitionFrameRateEnable", "B"}, {"AcquisitionFrameRate", "F"}};
    for (auto& s : to_save) save_for_restore(c, s[0], s[1][0]);
    cfg(c, "config", "TriggerMode", "Off", MV_CC_SetEnumValueByString(c.h, "TriggerMode", "Off"), rb(c, "TriggerMode", 'E'));
    cfg(c, "config", "AcquisitionMode", "Continuous", MV_CC_SetEnumValueByString(c.h, "AcquisitionMode", "Continuous"), rb(c, "AcquisitionMode", 'E'));
    cfg(c, "config", "ExposureAuto", "Off", MV_CC_SetEnumValueByString(c.h, "ExposureAuto", "Off"), rb(c, "ExposureAuto", 'E'));
    cfg(c, "config", "ExposureTime", fstr(opt.exposure_us), MV_CC_SetFloatValue(c.h, "ExposureTime", static_cast<float>(opt.exposure_us)), rb(c, "ExposureTime", 'F'));
    cfg(c, "config", "GainAuto", "Off", MV_CC_SetEnumValueByString(c.h, "GainAuto", "Off"), rb(c, "GainAuto", 'E'));
    cfg(c, "config", "AcquisitionFrameRateEnable", "true", MV_CC_SetBoolValue(c.h, "AcquisitionFrameRateEnable", true), rb(c, "AcquisitionFrameRateEnable", 'B'));
    cfg(c, "U8", "AcquisitionFrameRate", "30", MV_CC_SetFloatValue(c.h, "AcquisitionFrameRate", 30.0f), rb(c, "AcquisitionFrameRate", 'F'));
    cfg(c, "U8", "ResultingFrameRate", "读(设30后)", 0, rb(c, "ResultingFrameRate", 'F'));
    cfg(c, "config", "AcquisitionFrameRate", fstr(opt.fps), MV_CC_SetFloatValue(c.h, "AcquisitionFrameRate", static_cast<float>(opt.fps)), rb(c, "AcquisitionFrameRate", 'F'));
    cfg(c, "U8", "ResultingFrameRate", "读(设目标后)", 0, rb(c, "ResultingFrameRate", 'F'));
    cfg(c, "config", "PayloadSize", "读", 0, rb(c, "PayloadSize", 'I'));
    cfg(c, "config", "PixelFormat", "读", 0, rb(c, "PixelFormat", 'E'));
    cfg(c, "config", "SetImageNodeNum", std::to_string(opt.node_num), MV_CC_SetImageNodeNum(c.h, static_cast<unsigned int>(opt.node_num)), "");
    cfg(c, "config", "SetGrabStrategy", "OneByOne", MV_CC_SetGrabStrategy(c.h, MV_GrabStrategy_OneByOne), "");
  }

  // ---- T5 双相机同时取流（U2/U3/U5/U12/U15）----
  std::vector<Camera*> active;
  for (auto& cp : cams) if (cp->opened) active.push_back(cp.get());
  if (active.empty()) LOG("[错误] 没有成功打开的相机，跳过取流测试");
  for (auto* c : active) {
    c->frames.reserve(static_cast<size_t>(opt.fps * (opt.duration_s + 5)));
    int64_t t0 = now_ns(CLOCK_MONOTONIC);
    r = MV_CC_StartGrabbing(c->h);
    int64_t t1 = now_ns(CLOCK_MONOTONIC);
    cfg(*c, "grab", "StartGrabbing", "call", r, "耗时 " + fstr((t1 - t0) / 1e6) + " ms");
    c->grabbing = (r == MV_OK);
  }
  std::vector<std::thread> threads;
  g_stop_grab = false;
  for (auto* c : active) {
    if (!c->grabbing) continue;
    threads.emplace_back(grab_thread, c);
    threads.emplace_back(control_thread, c, opt.latch_hz);
  }
  if (!threads.empty()) {
    LOG("取流中，持续 %d 秒（Ctrl+C 可提前结束，仍会正常清理）", opt.duration_s);
    int64_t end = now_ns(CLOCK_MONOTONIC) + int64_t(opt.duration_s) * 1000000000LL;
    int64_t next_print = now_ns(CLOCK_MONOTONIC) + 5000000000LL;
    while (!g_abort && now_ns(CLOCK_MONOTONIC) < end) {
      sleep_ms(100);
      if (now_ns(CLOCK_MONOTONIC) >= next_print) {
        std::string s;
        for (auto* c : active) s += c->tag + ": " + std::to_string(c->nframes.load()) + " 帧  ";
        LOG("  %s", s.c_str());
        next_print += 5000000000LL;
      }
    }
    // 取流中写入测试（U10 锁定行为）：写回当前值，不改变实际设置
    for (auto* c : active) {
      if (!c->grabbing) continue;
      std::lock_guard<std::mutex> lk(c->ctl);
      MVCC_INTVALUE_EX iv{};
      if (MV_CC_GetIntValueEx(c->h, "Width", &iv) == MV_OK) cfg(*c, "lock_test", "Width", "写回当前值", MV_CC_SetIntValueEx(c->h, "Width", iv.nCurValue), "");
      if (MV_CC_GetIntValueEx(c->h, "OffsetX", &iv) == MV_OK) cfg(*c, "lock_test", "OffsetX", "写回当前值", MV_CC_SetIntValueEx(c->h, "OffsetX", iv.nCurValue), "");
      MVCC_ENUMVALUE_EX ev{};
      if (MV_CC_GetEnumValueEx(c->h, "PixelFormat", &ev) == MV_OK) cfg(*c, "lock_test", "PixelFormat", "写回当前值", MV_CC_SetEnumValue(c->h, "PixelFormat", ev.nCurValue), "");
      MVCC_FLOATVALUE fv{};
      if (MV_CC_GetFloatValue(c->h, "ExposureTime", &fv) == MV_OK) cfg(*c, "lock_test", "ExposureTime", "写回当前值", MV_CC_SetFloatValue(c->h, "ExposureTime", fv.fCurValue), "");
    }
  }
  // 停止：先让线程退出并 join，再 StopGrabbing（笔记第 11 项建议）
  g_stop_grab = true;
  for (auto& t : threads) t.join();
  for (auto* c : active) {
    if (!c->grabbing) continue;
    int64_t t0 = now_ns(CLOCK_MONOTONIC);
    r = MV_CC_StopGrabbing(c->h);
    int64_t t1 = now_ns(CLOCK_MONOTONIC);
    cfg(*c, "grab", "StopGrabbing", "call", r, "耗时 " + fstr((t1 - t0) / 1e6) + " ms");
    // U14（部分）：停流后时间戳是否连续
    LatchRec L = latch_once(*c, "latch", 0);
    cfg(*c, "U14", "DeviceTimestamp", "停流后 Latch", L.ret_cmd, istr(L.dev_ts));
    cfg(*c, "U14", "DeviceUptime", "读", 0, rb(*c, "DeviceUptime", 'I'));
  }

  // ---- 写出原始数据 ----
  for (auto* c : active) {
    std::string base = dir + "/" + c->tag;
    if (FILE* f = open_csv(base + "_frames.csv",
                           "cam,frame_num,frame_counter,trigger_index,lost_packet,width,height,pixel_type,frame_len,dev_ts,host_ts,recv_mono_ns,recv_raw_ns,recv_real_ns,exposure,gain")) {
      for (const auto& x : c->frames)
        fprintf(f, "%s,%u,%u,%u,%u,%u,%u,%s,%u,%llu,%lld,%lld,%lld,%lld,%.3f,%.3f\n", q(c->tag).c_str(), x.frame_num, x.frame_counter, x.trigger_index,
                x.lost_packet, x.width, x.height, q(hexs(x.pixel_type)).c_str(), x.frame_len, (unsigned long long)x.dev_ts, (long long)x.host_ts,
                (long long)x.recv.mono, (long long)x.recv.raw, (long long)x.recv.real, x.exposure, x.gain);
      fclose(f);
    }
    if (FILE* f = open_csv(base + "_latch_run.csv", kLatchHeader)) {
      for (const auto& L : c->latch_run) fprintf(f, "%s\n", latch_row(c->tag, L).c_str());
      fclose(f);
    }
    if (FILE* f = open_csv(base + "_status.csv", "cam,mono_ns,valid_images,ret_valid,resulting_fps,temperature,link_throughput_kb,queue_blocks,usb_recv_frames,usb_err_frames,ret_match")) {
      for (const auto& s : c->status)
        fprintf(f, "%s,%lld,%u,%s,%.3f,%.3f,%lld,%lld,%u,%u,%s\n", q(c->tag).c_str(), (long long)s.mono, s.valid_images, q(rets(s.ret_valid)).c_str(),
                s.resulting_fps, s.temperature, s.link_throughput, s.queue_blocks, s.usb_recv_frames, s.usb_err_frames, q(rets(s.ret_match)).c_str());
      fclose(f);
    }
  }

  // ---- 恢复参数、关闭 ----
  for (auto& cp : cams) {
    Camera& c = *cp;
    if (!c.opened) { if (c.h) MV_CC_DestroyHandle(c.h); continue; }
    dump_nodes(c, dir + "/" + c.tag + "_nodes_after.csv");
    if (opt.restore) {
      for (auto it = c.saved.rbegin(); it != c.saved.rend(); ++it) {
        if (!it->ok) continue;
        int rr = (it->type == 'F') ? MV_CC_SetFloatValue(c.h, it->name.c_str(), static_cast<float>(it->f))
               : (it->type == 'B') ? MV_CC_SetBoolValue(c.h, it->name.c_str(), it->b)
                                   : MV_CC_SetEnumValueByString(c.h, it->name.c_str(), it->s.c_str());
        cfg(c, "restore", it->name, "恢复原值", rr, rb(c, it->name.c_str(), it->type));
      }
    }
    MV_CC_RegisterExceptionCallBack(c.h, nullptr, nullptr);
    cfg(c, "close", "CloseDevice", "call", MV_CC_CloseDevice(c.h), "");
    cfg(c, "close", "DestroyHandle", "call", MV_CC_DestroyHandle(c.h), "");
    c.h = nullptr;
  }
  if (FILE* f = open_csv(dir + "/config.csv", "cam,phase,node,action,ret,readback")) {
    for (auto& cp : cams) for (auto& row : cp->config_rows) fprintf(f, "%s\n", row.c_str());
    fclose(f);
  }

  // ---- 简要汇总（详细分析用 analyze.py）----
  FILE* sf = fopen((dir + "/summary.txt").c_str(), "w");
  auto S = [&](const char* fmt, ...) {
    char buf[1024];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    LOG("%s", buf);
    if (sf) fprintf(sf, "%s\n", buf);
  };
  S("==== 汇总（%s）====", dir.c_str());
  for (auto* c : active) {
    S("相机 %s：%s %s %s", c->tag.c_str(), c->model.c_str(), c->serial.c_str(), c->is_virtual ? "(虚拟)" : "(真机)");
    size_t n = c->frames.size();
    S("  帧数 %zu；取图错误：%s", n, c->grab_errors.empty() ? "无" : "");
    for (auto& kv : c->grab_errors) S("    %s × %d%s", hexs(kv.first).c_str(), kv.second, kv.first == MV_E_NODATA ? "（超时无数据）" : "");
    if (n >= 2) {
      double secs = (c->frames.back().recv.mono - c->frames.front().recv.mono) / 1e9;
      size_t gaps = 0; bool ts_zero = true;
      std::vector<double> ddev;
      for (size_t i = 1; i < n; ++i) {
        if (c->frames[i].frame_num != c->frames[i - 1].frame_num + 1) gaps++;
        ddev.push_back(double(c->frames[i].dev_ts) - double(c->frames[i - 1].dev_ts));
      }
      for (auto& x : c->frames) if (x.dev_ts != 0) { ts_zero = false; break; }
      S("  按接收时刻计算的帧率 %.3f fps；帧号不连续 %zu 处", (n - 1) / secs, gaps);
      S("  帧设备时间戳：%s；相邻差中位数 %.0f（单位与设备时钟相同）", ts_zero ? "全部为 0（无效）" : "有值", percentile(ddev, 0.5));
    }
    std::vector<double> rtt;
    for (auto& L : c->latch_run) if (L.ret_cmd == MV_OK && L.ret_read == MV_OK) rtt.push_back((L.t1_mono - L.t0.mono) / 1e3);
    S("  取流中对时：成功 %zu / %zu；往返时间 us：最小 %.1f 中位 %.1f p99 %.1f", rtt.size(), c->latch_run.size(),
      rtt.empty() ? NAN : *std::min_element(rtt.begin(), rtt.end()), percentile(rtt, 0.5), percentile(rtt, 0.99));
    std::string se;
    for (int k = 1; k < 16; ++k) if (c->stream_exc[k]) se += hexs(0x4000 + k) + "×" + std::to_string(c->stream_exc[k].load()) + " ";
    S("  流异常：%s；掉线：%s", se.empty() ? "无" : se.c_str(), c->disconnected ? "是" : "否");
  }
  S("详细分析请运行：python3 analyze.py %s", dir.c_str());
  if (sf) fclose(sf);

  r = MV_CC_Finalize();
  LOG("MV_CC_Finalize ret=%s", rets(r).c_str());
  if (g_log) fclose(g_log);
  return 0;
}
