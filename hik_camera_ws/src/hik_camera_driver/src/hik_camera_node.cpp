// hik_camera_node.cpp —— 第 3 步：单相机 ROS2 组件节点
//
// 一个节点驱动一台相机。两台相机 = 同一个组件容器里加载两个本节点（第 4 步）。
//
// 话题（相对名称，加载时用命名空间区分左右，例如 /stereo/left/...）：
//   ~/image_raw   sensor_msgs/Image        header.stamp = 换算后的拍摄时刻（系统时间）
//   ~/camera_info sensor_msgs/CameraInfo   暂时只有尺寸和 frame_id（未标定）
//   ~/frame_meta  hik_camera_interfaces/FrameMeta
// 服务：
//   ~/start ~/stop ~/get_status            std_srvs/Trigger
//   ~/set_exposure ~/set_gain              hik_camera_interfaces/SetValue
//   ~/set_roi                              hik_camera_interfaces/SetRoi
// 参数：见 declareParams()；exposure_us、gain_db、fps 也可以用 ros2 param set 在线修改
//
// 兼容 ROS2 Humble（笔记本）和 Jazzy（实验室主机）：只使用两个版本通用的 rclcpp 接口。

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <memory>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <sys/stat.h>

#include "hik_camera_driver/hik_camera.hpp"
#include "hik_camera_interfaces/msg/frame_meta.hpp"
#include "hik_camera_interfaces/srv/set_roi.hpp"
#include "hik_camera_interfaces/srv/set_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace hik_camera_driver {

namespace {

// CLOCK_MONOTONIC → 系统时间（ROS 默认时钟）。每帧换算一次，两次读时钟相隔不到 1 µs。
int64_t monoToRealtimeNs(int64_t mono_ns) {
  timespec r{}, m{};
  clock_gettime(CLOCK_REALTIME, &r);
  clock_gettime(CLOCK_MONOTONIC, &m);
  const int64_t r_ns = int64_t(r.tv_sec) * 1000000000LL + r.tv_nsec;
  const int64_t m_ns = int64_t(m.tv_sec) * 1000000000LL + m.tv_nsec;
  return mono_ns + (r_ns - m_ns);
}

std::string nowString() {
  char b[32];
  time_t t = time(nullptr);
  strftime(b, sizeof b, "%Y%m%d_%H%M%S", localtime(&t));
  return b;
}

void mkdirs(const std::string& path) {
  std::string cur;
  for (std::size_t i = 0; i < path.size(); ++i) {
    cur += path[i];
    if (path[i] == '/' || i + 1 == path.size()) mkdir(cur.c_str(), 0755);
  }
}

// B 类参数的默认清单（两台相机共用）。曝光、增益、帧率、ROI 由单独的参数控制，不放在这里。
const std::vector<std::string> kDefaultSettings = {
    "AcquisitionMode:enum:Continuous",
    "TriggerSelector:enum:FrameBurstStart",
    "TriggerMode:enum:Off",
    "ExposureAuto:enum:Off",
    "GainAuto:enum:Off",
    "BalanceWhiteAuto:enum:Off:optional",
    "GammaEnable:bool:false:optional",
    "SharpnessEnable:bool:false:optional",
    "DigitalShiftEnable:bool:false:optional",
    "ContrastRatioEnable:bool:false:optional",
    "LUTEnable:bool:false:optional",
    "SequencerMode:enum:Off:optional",
};

}  // namespace

class HikCameraNode : public rclcpp::Node {
 public:
  explicit HikCameraNode(const rclcpp::NodeOptions& options) : rclcpp::Node("hik_camera", options) {
    declareParams();

    HikCameraOptions ho;
    ho.image_node_num = static_cast<unsigned int>(get_parameter("image_node_num").as_int());
    ho.sync_period_s = get_parameter("sync_period_s").as_double();
    cam_ = std::make_unique<HikCamera>(ho);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(get_parameter("qos_depth").as_int()))).reliable();
    pub_image_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
    pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", qos);
    pub_meta_ = create_publisher<hik_camera_interfaces::msg::FrameMeta>("~/frame_meta", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

    using Trigger = std_srvs::srv::Trigger;
    using SetValue = hik_camera_interfaces::srv::SetValue;
    using SetRoi = hik_camera_interfaces::srv::SetRoi;
    srv_start_ = create_service<Trigger>("~/start", [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
      res->success = startCamera(res->message);
    });
    srv_stop_ = create_service<Trigger>("~/stop", [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
      int r = cam_->stop();
      res->success = (r == 0);
      res->message = r == 0 ? "已停止" : errorString(r);
      RCLCPP_INFO(get_logger(), "stop: %s", res->message.c_str());
    });
    srv_status_ = create_service<Trigger>("~/get_status", [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
      res->success = cam_->isOpen();
      res->message = statusText();
    });
    srv_exposure_ = create_service<SetValue>("~/set_exposure", [this](const std::shared_ptr<SetValue::Request> req, std::shared_ptr<SetValue::Response> res) {
      res->success = applyFloat("ExposureTime", "exposure_us", req->value, res->actual, res->message);
    });
    srv_gain_ = create_service<SetValue>("~/set_gain", [this](const std::shared_ptr<SetValue::Request> req, std::shared_ptr<SetValue::Response> res) {
      res->success = applyFloat("Gain", "gain_db", req->value, res->actual, res->message);
    });
    srv_roi_ = create_service<SetRoi>("~/set_roi", [this](const std::shared_ptr<SetRoi::Request> req, std::shared_ptr<SetRoi::Response> res) {
      HikCamera::Roi roi{req->width, req->height, req->offset_x, req->offset_y};
      std::string detail;
      int r = cam_->setRoi(roi, &detail);
      HikCamera::Roi now;
      if (cam_->getRoi(now) == 0) { res->width = now.width; res->height = now.height; res->offset_x = now.offset_x; res->offset_y = now.offset_y; }
      res->success = (r == 0);
      res->message = r == 0 ? detail : errorString(r) + "：" + detail;
      RCLCPP_INFO(get_logger(), "set_roi: %s", res->message.c_str());
    });

    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) { return onParams(ps); });

    openAndConfigure();

    // 每秒记录一次累计帧数，用于计算“最近 10 秒的实测帧率”
    fps_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      std::lock_guard<std::mutex> lk(fps_mtx_);
      fps_hist_.emplace_back(std::chrono::steady_clock::now(), cam_->stats().frames);
      while (fps_hist_.size() > 11) fps_hist_.pop_front();
    });

    double sp = get_parameter("status_period_s").as_double();
    if (sp > 0) {
      status_timer_ = create_wall_timer(std::chrono::duration<double>(sp), [this] {
        if (cam_->isGrabbing() || cam_->disconnected()) RCLCPP_INFO(get_logger(), "%s", oneLineStatus().c_str());
      });
    }
    if (configured_ok_ && get_parameter("auto_start").as_bool()) {
      std::string msg;
      startCamera(msg);
    }
  }

  ~HikCameraNode() override {
    if (cam_) cam_->close();  // 停流、关闭相机（HikCamera 析构也会做，这里显式调用，日志顺序更清楚）
  }

 private:
  // ---------------------------------------------------------------- 参数
  void declareParams() {
    declare_parameter<std::string>("user_id", "");         // 按 Device User ID 打开（老师要求“相机 id 要配置”）
    declare_parameter<std::string>("serial", "");          // 可选：同时校验序列号
    declare_parameter<std::string>("frame_id", "camera");
    declare_parameter<double>("fps", 60.0);
    declare_parameter<double>("exposure_us", 5000.0);
    declare_parameter<double>("gain_db", 0.0);
    declare_parameter<int64_t>("roi_width", 0);            // 0 表示全幅
    declare_parameter<int64_t>("roi_height", 0);
    declare_parameter<int64_t>("roi_offset_x", 0);
    declare_parameter<int64_t>("roi_offset_y", 0);
    declare_parameter<std::vector<std::string>>("camera_settings", kDefaultSettings);
    declare_parameter<bool>("auto_start", false);          // false：等待 ~/start 服务（老师：“主机只发开始和结束命令”）
    declare_parameter<int64_t>("image_node_num", 30);
    declare_parameter<double>("sync_period_s", 1.0);
    declare_parameter<int64_t>("qos_depth", 5);
    declare_parameter<double>("status_period_s", 10.0);    // 定期打印一行状态；0 关闭
    declare_parameter<std::string>("report_dir", "");      // 启动报告保存目录；空 = ~/.ros/hik_camera/reports
  }

  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter>& ps) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    if (internal_update_) return result;  // 服务内部同步参数值时不再重复写相机
    for (const auto& p : ps) {
      const std::string& n = p.get_name();
      std::string msg;
      double actual = 0;
      if (n == "exposure_us" || n == "gain_db" || n == "fps") {
        if (!cam_ || !cam_->isOpen()) continue;  // 构造阶段（相机尚未打开）不处理
        const char* node = n == "exposure_us" ? "ExposureTime" : n == "gain_db" ? "Gain" : "AcquisitionFrameRate";
        int r = cam_->setFloat(node, p.as_double());
        if (r != 0) {
          result.successful = false;
          result.reason = std::string(node) + " 写入失败：" + errorString(r);
        } else {
          FloatInfo fi;
          if (cam_->getFloatInfo(node, fi) == 0) actual = fi.cur;
          RCLCPP_INFO(get_logger(), "参数 %s = %.3f（读回 %.3f）", n.c_str(), p.as_double(), actual);
        }
      } else if (n.rfind("roi_", 0) == 0 || n == "user_id" || n == "serial" || n == "camera_settings" || n == "image_node_num") {
        result.successful = false;
        result.reason = n + " 只在启动时生效（ROI 请用 ~/set_roi 服务）";
      }
      (void)msg;
    }
    return result;
  }

  // 服务和参数回调共用：写入 → 读回 → 同步 ROS 参数值
  bool applyFloat(const char* node, const char* param, double value, double& actual, std::string& message) {
    int r = cam_->setFloat(node, value);
    if (r != 0) {
      FloatInfo fi;
      std::string range = cam_->getFloatInfo(node, fi) == 0 ? "（范围 " + std::to_string(fi.min) + " – " + std::to_string(fi.max) + "）" : "";
      message = std::string(node) + " 写入失败：" + errorString(r) + range;
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
      return false;
    }
    FloatInfo fi;
    actual = cam_->getFloatInfo(node, fi) == 0 ? fi.cur : value;
    message = std::string(node) + " 已设置，读回 " + std::to_string(actual);
    internal_update_ = true;
    set_parameter(rclcpp::Parameter(param, actual));
    internal_update_ = false;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  // ---------------------------------------------------------------- 启动
  void openAndConfigure() {
    CameraSelector sel{get_parameter("user_id").as_string(), get_parameter("serial").as_string()};
    frame_id_ = get_parameter("frame_id").as_string();
    std::vector<std::string> report;
    auto R = [&](const std::string& s) { report.push_back(s); RCLCPP_INFO(get_logger(), "%s", s.c_str()); };

    int r = cam_->open(sel);
    if (r != 0) {
      RCLCPP_ERROR(get_logger(), "打开相机失败（%s）：%s", sel.describe().c_str(), cam_->lastError().c_str());
      configured_ok_ = false;
      return;
    }
    R("打开 " + cam_->device().describe() + "，名义 " + std::to_string(cam_->nominalNsPerTick()) + " ns/tick");

    // 1. B 类参数（顺序即写入顺序）
    std::vector<std::string> specs = get_parameter("camera_settings").as_string_array();
    // 2. 曝光、增益、帧率（A 类的初始值）
    char buf[96];
    std::snprintf(buf, sizeof buf, "ExposureTime:float:%.3f", get_parameter("exposure_us").as_double());
    specs.push_back(buf);
    std::snprintf(buf, sizeof buf, "Gain:float:%.3f", get_parameter("gain_db").as_double());
    specs.push_back(buf);
    specs.push_back("AcquisitionFrameRateEnable:bool:true");
    std::snprintf(buf, sizeof buf, "AcquisitionFrameRate:float:%.3f", get_parameter("fps").as_double());
    specs.push_back(buf);

    ParseResult parsed = parseParamSpecs(specs);
    for (auto& e : parsed.errors) R("配置错误（已忽略）：" + e.spec + " → " + e.message);
    for (auto& w : parsed.warnings) R("配置警告：" + w.spec + " → " + w.message);
    ApplyReport rep = applyParams(parsed.entries, *cam_);
    for (auto& line : rep.lines()) {
      report.push_back(line);
      if (line.rfind("OK", 0) == 0) RCLCPP_DEBUG(get_logger(), "%s", line.c_str());
      else RCLCPP_WARN(get_logger(), "%s", line.c_str());
    }
    R(rep.summary());
    configured_ok_ = rep.allRequiredOk();

    // 3. ROI（0 表示全幅）
    IntInfo wmax, hmax;
    if (cam_->getIntInfo("WidthMax", wmax) == 0 && cam_->getIntInfo("HeightMax", hmax) == 0) {
      HikCamera::Roi roi{get_parameter("roi_width").as_int(), get_parameter("roi_height").as_int(),
                         get_parameter("roi_offset_x").as_int(), get_parameter("roi_offset_y").as_int()};
      if (roi.width <= 0) roi.width = wmax.cur;
      if (roi.height <= 0) roi.height = hmax.cur;
      std::string detail;
      int rr = cam_->setRoi(roi, &detail);
      R(rr == 0 ? detail : "ROI 设置失败：" + errorString(rr) + " " + detail);
      if (rr != 0) configured_ok_ = false;
    }

    // 4. 启动检查
    EnumInfo pf;
    IntInfo w, h, payload;
    if (cam_->getEnumInfo("PixelFormat", pf) == 0 && cam_->getIntInfo("Width", w) == 0 && cam_->getIntInfo("Height", h) == 0) {
      auto fmt = lookupPixelFormat(pf.cur_value);
      std::string line = "检查：PixelFormat=" + pf.cur + " → " + (fmt ? fmt->ros_encoding : "不支持的格式") + "，" +
                         std::to_string(w.cur) + "x" + std::to_string(h.cur);
      if (fmt && cam_->getIntInfo("PayloadSize", payload) == 0) {
        std::size_t expect = frameBytes(*fmt, static_cast<uint32_t>(w.cur), static_cast<uint32_t>(h.cur));
        line += "，PayloadSize=" + std::to_string(payload.cur) + (static_cast<int64_t>(expect) == payload.cur ? " OK" : " 与预期 " + std::to_string(expect) + " 不一致");
      }
      if (!fmt) configured_ok_ = false;
      R(line);
    }
    FloatInfo res;
    if (cam_->getFloatInfo("ResultingFrameRate", res) == 0) R("检查：ResultingFrameRate=" + std::to_string(res.cur));
    EnumInfo usb;
    if (cam_->getEnumInfo("USBSpeedMode", usb) == 0) R("检查：USBSpeedMode=" + usb.cur);
    R(configured_ok_ ? "就绪，等待 ~/start（或 auto_start=true）" : "配置有错误，start 将被拒绝；请检查上面的 FAILED_REQUIRED / 错误信息");

    saveReport(report);
  }

  void saveReport(const std::vector<std::string>& lines) {
    std::string dir = get_parameter("report_dir").as_string();
    if (dir.empty()) {
      const char* home = getenv("HOME");
      dir = std::string(home ? home : "/tmp") + "/.ros/hik_camera/reports";
    }
    mkdirs(dir);
    std::string path = dir + "/" + nowString() + "_" + std::string(get_name()) + ".txt";
    std::ofstream f(path);
    if (!f) { RCLCPP_WARN(get_logger(), "无法写入启动报告 %s", path.c_str()); return; }
    f << "node=" << get_fully_qualified_name() << "\n";
    for (auto& l : lines) f << l << "\n";
    RCLCPP_INFO(get_logger(), "启动报告已保存：%s", path.c_str());
  }

  // ---------------------------------------------------------------- 取流
  bool startCamera(std::string& message) {
    if (!cam_->isOpen()) { message = "相机未打开：" + cam_->lastError(); RCLCPP_ERROR(get_logger(), "%s", message.c_str()); return false; }
    if (!configured_ok_) { message = "启动配置有错误，拒绝开流（见启动日志）"; RCLCPP_ERROR(get_logger(), "%s", message.c_str()); return false; }
    if (cam_->isGrabbing()) { message = "已经在取流"; return true; }
    int r = cam_->start([this](const Frame& f) { onFrame(f); });
    message = r == 0 ? "已开始取流" : "开流失败：" + errorString(r) + " " + cam_->lastError();
    if (r == 0) RCLCPP_INFO(get_logger(), "%s（%s）", message.c_str(), cam_->clockState().summary().c_str());
    else RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return r == 0;
  }

  // 在取图线程中执行：只做拷贝和发布
  void onFrame(const Frame& f) {
    if (!f.format) {
      unknown_format_frames_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "不支持的像素格式 %s，帧未发布", pixelFormatHex(f.pixel_type).c_str());
      return;
    }
    const std::size_t step = rowBytes(*f.format, f.width);
    const std::size_t need = step * f.height;
    if (f.size < need) {
      short_frames_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "帧长度 %zu 小于预期 %zu，帧未发布", f.size, need);
      return;
    }
    const rclcpp::Time stamp(monoToRealtimeNs(f.stamp_mono_ns), RCL_SYSTEM_TIME);

    auto img = std::make_unique<sensor_msgs::msg::Image>();
    img->header.stamp = stamp;
    img->header.frame_id = frame_id_;
    img->height = f.height;
    img->width = f.width;
    img->encoding = f.format->ros_encoding;
    img->is_bigendian = 0;
    img->step = static_cast<uint32_t>(step);
    img->data.assign(f.data, f.data + need);
    pub_image_->publish(std::move(img));  // unique_ptr：进程内通信时零拷贝

    auto info = std::make_unique<sensor_msgs::msg::CameraInfo>();
    info->header.stamp = stamp;
    info->header.frame_id = frame_id_;
    info->width = f.width;
    info->height = f.height;
    pub_info_->publish(std::move(info));

    auto meta = std::make_unique<hik_camera_interfaces::msg::FrameMeta>();
    meta->header.stamp = stamp;
    meta->header.frame_id = frame_id_;
    meta->time_source = f.time_source == TimeSource::kDevice              ? hik_camera_interfaces::msg::FrameMeta::TIME_DEVICE
                        : f.time_source == TimeSource::kDeviceProvisional ? hik_camera_interfaces::msg::FrameMeta::TIME_DEVICE_PROVISIONAL
                                                                          : hik_camera_interfaces::msg::FrameMeta::TIME_RECEIVE;
    meta->seq = f.seq;
    meta->frame_num = f.frame_num;
    meta->dev_ticks = f.dev_ticks;
    meta->stamp_mono_ns = f.stamp_mono_ns;
    meta->recv_mono_ns = f.recv_mono_ns;
    meta->lost_packet = f.lost_packet;
    meta->exposure_us = f.exposure_us;
    meta->gain_db = f.gain_db;
    pub_meta_->publish(std::move(meta));
  }

  // ---------------------------------------------------------------- 状态
  // 最近 10 秒的实测帧率（由每秒一次的帧数采样计算）
  double measuredFps() {
    std::lock_guard<std::mutex> lk(fps_mtx_);
    if (fps_hist_.size() < 2) return 0.0;
    const auto& a = fps_hist_.front();
    const auto& b = fps_hist_.back();
    const double dt = std::chrono::duration<double>(b.first - a.first).count();
    return (dt > 0 && b.second >= a.second) ? static_cast<double>(b.second - a.second) / dt : 0.0;
  }

  // 一键状态：所有数值都在调用时从相机实时读取（不是配置值）
  std::string statusText() {
    if (!cam_->isOpen()) return "相机未打开：" + cam_->lastError();
    auto st = cam_->stats();
    auto cs = cam_->clockState();
    auto rdF = [this](const char* n) { FloatInfo f; return cam_->getFloatInfo(n, f) == 0 ? f.cur : NAN; };
    auto rdI = [this](const char* n) { IntInfo i; return cam_->getIntInfo(n, i) == 0 ? i.cur : int64_t(-1); };
    EnumInfo pf;
    std::string pf_s = cam_->getEnumInfo("PixelFormat", pf) == 0 ? pf.cur : "?";
    auto fmt = lookupPixelFormat(pf.cur_value);
    const int64_t thr = rdI("DeviceLinkCurrentThroughput");
    const double temp = rdF("DeviceTemperature");
    char b[2048];
    std::snprintf(b, sizeof b,
        "== %s ==\n"
        "状态      %s | 掉线 %s | 启动配置 %s\n"
        "帧率      实测 %.2f fps（近 10 s）| 设定 %.2f | 相机 ResultingFrameRate %.2f\n"
        "帧数      本次取流累计 %llu 帧\n"
        "丢帧      帧号跳变 %llu | 丢包帧 %llu | 取图超时 %llu | 取图错误 %llu\n"
        "曝光      %.1f us | 增益 %.2f dB（相机当前值）\n"
        "ROI       %lldx%lld +%lld+%lld（最大 %lldx%lld）\n"
        "像素格式  %s -> %s\n"
        "温度      %s | 带宽 %s\n"
        "对时      %s %+.1f ppm rms %.0f us | 接收时间帧 %llu | 临时模型帧 %llu\n"
        "最近提示  %s",
        cam_->device().describe().c_str(),
        cam_->isGrabbing() ? "取流中" : "已停止", cam_->disconnected() ? "是" : "否", configured_ok_ ? "OK" : "有错误",
        measuredFps(), get_parameter("fps").as_double(), rdF("ResultingFrameRate"),
        (unsigned long long)st.frames,
        (unsigned long long)st.frame_num_gaps, (unsigned long long)st.lost_packet_frames, (unsigned long long)st.grab_timeouts,
        (unsigned long long)st.grab_errors,
        rdF("ExposureTime"), rdF("Gain"),
        (long long)rdI("Width"), (long long)rdI("Height"), (long long)rdI("OffsetX"), (long long)rdI("OffsetY"),
        (long long)rdI("WidthMax"), (long long)rdI("HeightMax"),
        pf_s.c_str(), fmt ? fmt->ros_encoding : "不支持",
        std::isnan(temp) ? "读取失败" : (std::to_string(temp).substr(0, std::to_string(temp).find('.') + 2) + " °C").c_str(),
        thr < 0 ? "读取失败" : (std::to_string(thr) + " KB/s（约 " + std::to_string(thr / 1024) + " MB/s）").c_str(),
        cs.ready ? (cs.provisional ? "临时" : "正常") : "未就绪", cs.rate_error_ppm, cs.residual_rms_ns / 1e3,
        (unsigned long long)st.receive_time_frames, (unsigned long long)st.provisional_frames,
        cam_->lastError().empty() ? "无" : cam_->lastError().c_str());
    return b;
  }

  std::string oneLineStatus() {
    auto st = cam_->stats();
    auto cs = cam_->clockState();
    uint64_t df = st.frames - last_frames_;
    last_frames_ = st.frames;
    char b[512];
    std::snprintf(b, sizeof b, "状态：%s 新增帧 %llu，帧号跳变 %llu，丢包帧 %llu，超时 %llu，错误 %llu，接收时间帧 %llu；对时 %s %+.1fppm rms=%.0fus%s",
                  cam_->isGrabbing() ? "取流中" : "已停止", (unsigned long long)df, (unsigned long long)st.frame_num_gaps,
                  (unsigned long long)st.lost_packet_frames, (unsigned long long)st.grab_timeouts, (unsigned long long)st.grab_errors,
                  (unsigned long long)st.receive_time_frames, cs.ready ? (cs.provisional ? "临时" : "正常") : "未就绪", cs.rate_error_ppm,
                  cs.residual_rms_ns / 1e3, cam_->disconnected() ? "；【已掉线】" : "");
    return b;
  }

  std::unique_ptr<HikCamera> cam_;
  std::string frame_id_;
  bool configured_ok_ = false;
  std::atomic<bool> internal_update_{false};
  std::atomic<uint64_t> unknown_format_frames_{0}, short_frames_{0};
  uint64_t last_frames_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  rclcpp::Publisher<hik_camera_interfaces::msg::FrameMeta>::SharedPtr pub_meta_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_stop_, srv_status_;
  rclcpp::Service<hik_camera_interfaces::srv::SetValue>::SharedPtr srv_exposure_, srv_gain_;
  rclcpp::Service<hik_camera_interfaces::srv::SetRoi>::SharedPtr srv_roi_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr status_timer_, fps_timer_;
  std::mutex fps_mtx_;
  std::deque<std::pair<std::chrono::steady_clock::time_point, uint64_t>> fps_hist_;
};

}  // namespace hik_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera_driver::HikCameraNode)
