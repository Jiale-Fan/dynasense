// dynasense_depth_image_to_pointcloud_node
//
// Back-projects depth images into organized point clouds with a pinhole model,
// one camera per configuration entry. Written for the real-robot bags that
// carry /depth_camera_*/depth/image_rect_raw but no camera_info, so the
// intrinsics come from the configuration (by default the Gazebo D435 model of
// anymal_d, see config/depth_cameras.yaml). A camera_info that IS published
// (live robot, or a bag that has it) overrides them.
//
// Inputs (per camera)
//   {ns}/depth/image_rect_raw   sensor_msgs/Image       16UC1 (mm) | mono16 (mm) | 32FC1 (m)
//   {ns}/depth/camera_info      sensor_msgs/CameraInfo  optional; a live one overrides the config
//
// Outputs (per camera)
//   {ns}/depth/points           sensor_msgs/PointCloud2  organized H x W, xyz float32,
//                               NaN = no return / outside [depth_min, depth_max], is_dense false,
//                               header (stamp, frame_id) copied from the image
//   {ns}/depth/camera_info      sensor_msgs/CameraInfo   synthesized from the configured intrinsics,
//                               same header, unless a live one is being received
//
// No cv_bridge: the image buffer is read through step/width/height directly.
// The math is that of depth_image_proc::convert: x = (u - cx) z / fx,
// y = (v - cy) z / fy, z = depth, in the image's optical frame (x right, y down,
// z forward). The face cameras' upside-down mount lives in TF, so nothing is
// flipped here.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/make_shared.hpp>
#include <ros/message_event.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace dynasense {
namespace {

constexpr const char* kLogPrefix = "[dynasense_depth2pc] ";

struct Intrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};

  bool valid() const { return fx > 0.0 && fy > 0.0 && width > 0 && height > 0; }

  bool operator==(const Intrinsics& other) const {
    return fx == other.fx && fy == other.fy && cx == other.cx && cy == other.cy && width == other.width &&
           height == other.height;
  }
};

struct GlobalParams {
  double depthMin{0.1};  // [m]
  double depthMax{3.0};  // [m]
  int decimation{1};
  double maxRate{0.0};  // [Hz] per camera, 0 = every frame
  bool publishCameraInfo{true};
  bool useLiveCameraInfo{true};
  bool processWithoutSubscribers{false};
  int imageQueueSize{2};
  bool tcpNoDelay{true};
  double depthUnitM{0.001};  // metres per 16-bit unit (REP 118: millimetres)
  std::string imageTopicTemplate{"{ns}/depth/image_rect_raw"};
  std::string cloudTopicTemplate{"{ns}/depth/points"};
  std::string cameraInfoTopicTemplate{"{ns}/depth/camera_info"};
};

struct CameraConfig {
  std::string name;
  std::string ns;
  std::string imageTopic;
  std::string cloudTopic;
  std::string cameraInfoTopic;
  std::string frameIdOverride;  // empty: keep the image header's frame_id
  Intrinsics intrinsics;
};

/// Replaces every "{ns}" in a topic template.
std::string substituteNamespace(std::string text, const std::string& ns) {
  const std::string token = "{ns}";
  std::size_t pos = text.find(token);
  while (pos != std::string::npos) {
    text.replace(pos, token.size(), ns);
    pos = text.find(token, pos + ns.size());
  }
  return text;
}

/// roslaunch types "2" as int and "2.0" as double; accept both.
bool asNumber(XmlRpc::XmlRpcValue& value, double& out) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    out = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    out = static_cast<int>(value);
    return true;
  }
  return false;
}

bool readNumber(XmlRpc::XmlRpcValue& parent, const std::string& key, double& out) {
  if (!parent.hasMember(key)) {
    return false;
  }
  if (!asNumber(parent[key], out)) {
    throw std::runtime_error("parameter '" + key + "' must be a number");
  }
  return true;
}

bool readString(XmlRpc::XmlRpcValue& parent, const std::string& key, std::string& out) {
  if (!parent.hasMember(key)) {
    return false;
  }
  if (parent[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error("parameter '" + key + "' must be a string");
  }
  out = static_cast<std::string>(parent[key]);
  return true;
}

/// Overwrites the members of `k` that `value` (a struct) provides.
void readIntrinsics(XmlRpc::XmlRpcValue& value, Intrinsics& k) {
  double d = 0.0;
  if (readNumber(value, "fx", d)) k.fx = d;
  if (readNumber(value, "fy", d)) k.fy = d;
  if (readNumber(value, "cx", d)) k.cx = d;
  if (readNumber(value, "cy", d)) k.cy = d;
  if (readNumber(value, "width", d)) k.width = static_cast<int>(std::lround(d));
  if (readNumber(value, "height", d)) k.height = static_cast<int>(std::lround(d));
}

bool hostIsBigEndian() {
  const uint16_t probe = 1;
  uint8_t first = 0;
  std::memcpy(&first, &probe, 1);
  return first == 0;
}

inline uint16_t load16(const uint8_t* p, bool swap) {
  uint16_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return swap ? static_cast<uint16_t>((v << 8) | (v >> 8)) : v;
}

inline float load32f(const uint8_t* p, bool swap) {
  uint32_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  if (swap) {
    v = ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8) | ((v & 0x00FF0000u) >> 8) | ((v & 0xFF000000u) >> 24);
  }
  float f = 0.0f;
  std::memcpy(&f, &v, sizeof(f));
  return f;
}

/// The same camera at another image size. Pixel centres map as
/// u' = (u + 0.5) * s - 0.5, hence the half-pixel terms.
Intrinsics rescaled(const Intrinsics& k, int width, int height) {
  const double sx = static_cast<double>(width) / k.width;
  const double sy = static_cast<double>(height) / k.height;
  Intrinsics r = k;
  r.width = width;
  r.height = height;
  r.fx = k.fx * sx;
  r.fy = k.fy * sy;
  r.cx = (k.cx + 0.5) * sx - 0.5;
  r.cy = (k.cy + 0.5) * sy - 0.5;
  return r;
}

}  // namespace

/// One depth camera: image in, cloud (+ camera_info) out.
///
/// All members except the live-camera_info hand-off are touched only from the
/// image callback. roscpp never runs the same callback concurrently (the
/// default allow_concurrent_callbacks = false), so they need no lock even with
/// a multi-threaded spinner; different cameras do run in parallel.
class DepthCamera {
 public:
  DepthCamera(ros::NodeHandle& nh, CameraConfig config, const GlobalParams& globals)
      : cfg_(std::move(config)), g_(globals) {
    cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>(cfg_.cloudTopic, 1);
    if (g_.publishCameraInfo) {
      infoPub_ = nh.advertise<sensor_msgs::CameraInfo>(cfg_.cameraInfoTopic, 1);
    }
    if (g_.useLiveCameraInfo) {
      infoSub_ = nh.subscribe(cfg_.cameraInfoTopic, 1, &DepthCamera::onCameraInfo, this);
    }
    ros::TransportHints hints;
    if (g_.tcpNoDelay) {
      hints = ros::TransportHints().tcpNoDelay();
    }
    imageSub_ = nh.subscribe(cfg_.imageTopic, static_cast<uint32_t>(g_.imageQueueSize), &DepthCamera::onImage, this,
                             hints);

    const Intrinsics& k = cfg_.intrinsics;
    ROS_INFO_STREAM(kLogPrefix << cfg_.name << ": " << cfg_.imageTopic << " -> " << cfg_.cloudTopic << "  fx=" << k.fx
                               << " fy=" << k.fy << " cx=" << k.cx << " cy=" << k.cy << " (" << k.width << "x"
                               << k.height << ")"
                               << (g_.publishCameraInfo ? ", camera_info -> " + cfg_.cameraInfoTopic : std::string()));
  }

 private:
  void onImage(const sensor_msgs::ImageConstPtr& msg) {
    if (!g_.processWithoutSubscribers && cloudPub_.getNumSubscribers() == 0 &&
        (!infoPub_ || infoPub_.getNumSubscribers() == 0)) {
      return;
    }
    if (g_.maxRate > 0.0 && !lastStamp_.isZero()) {
      // Header stamps, so the cap also holds under sim time. A negative step
      // means the bag looped or was rewound: publish.
      const double dt = (msg->header.stamp - lastStamp_).toSec();
      if (dt >= 0.0 && dt < 1.0 / g_.maxRate) {
        return;
      }
    }

    std::size_t bytesPerPixel = 0;
    bool isFloat = false;
    const std::string& enc = msg->encoding;
    if (enc == sensor_msgs::image_encodings::TYPE_16UC1 || enc == sensor_msgs::image_encodings::MONO16) {
      bytesPerPixel = 2;
    } else if (enc == sensor_msgs::image_encodings::TYPE_32FC1) {
      bytesPerPixel = 4;
      isFloat = true;
    } else {
      ROS_ERROR_STREAM_THROTTLE(5.0, kLogPrefix << cfg_.name << ": unsupported encoding '" << enc
                                                << "' (need 16UC1, mono16 or 32FC1)");
      return;
    }

    // step may exceed width * bytesPerPixel (row padding): rows are always
    // addressed through step, never through width.
    if (msg->width == 0 || msg->height == 0 || msg->step < msg->width * bytesPerPixel ||
        msg->data.size() < static_cast<std::size_t>(msg->step) * msg->height) {
      ROS_WARN_STREAM_THROTTLE(5.0, kLogPrefix << cfg_.name << ": malformed image (" << msg->width << "x"
                                               << msg->height << ", step " << msg->step << ", " << msg->data.size()
                                               << " bytes)");
      return;
    }

    if (!ensureTables(static_cast<int>(msg->width), static_cast<int>(msg->height))) {
      return;
    }

    auto cloud = boost::make_shared<sensor_msgs::PointCloud2>();
    cloud->header = msg->header;  // consumers look TF up by this stamp and frame
    if (!cfg_.frameIdOverride.empty()) {
      cloud->header.frame_id = cfg_.frameIdOverride;
    }
    if (cloud->header.frame_id.empty()) {
      ROS_WARN_STREAM_THROTTLE(5.0, kLogPrefix << cfg_.name
                                               << ": image has an empty frame_id; set frame_id in the config");
      return;
    }
    // Organized cloud. height/width must be set before the fields are added,
    // and PointCloud2Modifier::resize() must NOT be called: it forces height=1.
    cloud->height = static_cast<uint32_t>(tableH_);
    cloud->width = static_cast<uint32_t>(tableW_);
    cloud->is_bigendian = hostIsBigEndian();
    cloud->is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");  // x y z float32 + padding, point_step 16

    const bool swap = (msg->is_bigendian != 0) != hostIsBigEndian();
    if (isFloat) {
      backProject(*msg, bytesPerPixel, 1.0f, [swap](const uint8_t* p) { return load32f(p, swap); }, *cloud);
    } else {
      const float unit = static_cast<float>(g_.depthUnitM);
      backProject(
          *msg, bytesPerPixel, unit, [swap](const uint8_t* p) { return static_cast<float>(load16(p, swap)); },
          *cloud);
    }

    lastStamp_ = msg->header.stamp;
    cloudPub_.publish(cloud);
    if (infoPub_ && !liveInfoActive_) {
      infoPub_.publish(makeCameraInfo(cloud->header));
    }
  }

  template <typename LoadFn>
  void backProject(const sensor_msgs::Image& img, std::size_t bytesPerPixel, float unitScale, LoadFn load,
                   sensor_msgs::PointCloud2& cloud) const {
    const int dec = g_.decimation;
    const float zMin = static_cast<float>(g_.depthMin);
    const float zMax = static_cast<float>(g_.depthMax);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    const uint8_t* data = img.data.data();
    for (int j = 0; j < tableH_; ++j) {
      const uint8_t* row = data + static_cast<std::size_t>(j) * dec * img.step;
      const float yj = yTable_[j];
      for (int i = 0; i < tableW_; ++i, ++ix, ++iy, ++iz) {
        const float z = load(row + static_cast<std::size_t>(i) * dec * bytesPerPixel) * unitScale;
        if (!(z >= zMin && z <= zMax)) {  // also rejects 0 (no return), NaN and inf
          *ix = nan;
          *iy = nan;
          *iz = nan;
          continue;
        }
        *ix = xTable_[i] * z;
        *iy = yj * z;
        *iz = z;
      }
    }
  }

  /// (Re)builds the per-column and per-row back-projection tables when the
  /// image size changes or a live camera_info has arrived.
  bool ensureTables(int width, int height) {
    bool rebuild = (tableW_ == 0 || width != imgW_ || height != imgH_);
    Intrinsics k;
    {
      std::lock_guard<std::mutex> lock(infoMutex_);
      if (liveInfoPending_) {
        liveInfoPending_ = false;
        liveInfoActive_ = true;
        rebuild = true;
      }
      k = liveInfoActive_ ? liveInfo_ : cfg_.intrinsics;
    }
    if (!rebuild) {
      return true;
    }
    if (!k.valid()) {
      ROS_ERROR_STREAM_THROTTLE(5.0, kLogPrefix << cfg_.name << ": invalid intrinsics (fx=" << k.fx << " fy=" << k.fy
                                                << " " << k.width << "x" << k.height << ")");
      return false;
    }
    if (k.width != width || k.height != height) {
      if (!rescaleWarned_) {
        ROS_WARN_STREAM(kLogPrefix << cfg_.name << ": image is " << width << "x" << height
                                   << " but the intrinsics are for " << k.width << "x" << k.height
                                   << "; rescaling them");
        rescaleWarned_ = true;
      }
      k = rescaled(k, width, height);
    }

    active_ = k;
    imgW_ = width;
    imgH_ = height;
    const int dec = g_.decimation;
    tableW_ = (width + dec - 1) / dec;
    tableH_ = (height + dec - 1) / dec;
    xTable_.resize(static_cast<std::size_t>(tableW_));
    yTable_.resize(static_cast<std::size_t>(tableH_));
    for (int i = 0; i < tableW_; ++i) {
      xTable_[i] = static_cast<float>((static_cast<double>(i) * dec - k.cx) / k.fx);
    }
    for (int j = 0; j < tableH_; ++j) {
      yTable_[j] = static_cast<float>((static_cast<double>(j) * dec - k.cy) / k.fy);
    }
    return true;
  }

  /// A camera_info from someone else (the driver, or a bag that has it)
  /// replaces the configured intrinsics. Our own synthesized messages come
  /// back through this subscription too and are skipped by publisher name.
  void onCameraInfo(const ros::MessageEvent<sensor_msgs::CameraInfo const>& event) {
    if (event.getPublisherName() == ros::this_node::getName()) {
      return;
    }
    const sensor_msgs::CameraInfo& info = *event.getConstMessage();
    Intrinsics k;
    k.width = static_cast<int>(info.width);
    k.height = static_cast<int>(info.height);
    if (info.P[0] > 0.0) {
      k.fx = info.P[0];
      k.fy = info.P[5];
      k.cx = info.P[2];
      k.cy = info.P[6];
    } else {
      k.fx = info.K[0];
      k.fy = info.K[4];
      k.cx = info.K[2];
      k.cy = info.K[5];
    }
    if (!k.valid()) {
      ROS_WARN_STREAM_THROTTLE(10.0, kLogPrefix << cfg_.name << ": ignoring camera_info from "
                                                << event.getPublisherName() << " without valid intrinsics");
      return;
    }
    std::lock_guard<std::mutex> lock(infoMutex_);
    if (liveInfoSeen_ && k == liveInfo_) {
      return;  // the driver streams the same values at frame rate; nothing to rebuild
    }
    ROS_INFO_STREAM(kLogPrefix << cfg_.name << ": live camera_info from " << event.getPublisherName()
                               << " overrides the configured intrinsics (fx=" << k.fx << " fy=" << k.fy
                               << " cx=" << k.cx << " cy=" << k.cy << " " << k.width << "x" << k.height
                               << "); the synthesized camera_info stops");
    liveInfo_ = k;
    liveInfoSeen_ = true;
    liveInfoPending_ = true;
  }

  /// Describes the IMAGE (full resolution), not the decimated cloud.
  sensor_msgs::CameraInfo makeCameraInfo(const std_msgs::Header& header) const {
    sensor_msgs::CameraInfo info;
    info.header = header;
    info.width = static_cast<uint32_t>(imgW_);
    info.height = static_cast<uint32_t>(imgH_);
    info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info.D.assign(5, 0.0);
    const Intrinsics& k = active_;
    info.K = {{k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0}};
    info.R = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
    info.P = {{k.fx, 0.0, k.cx, 0.0, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0}};
    return info;
  }

  CameraConfig cfg_;
  const GlobalParams& g_;

  Intrinsics active_;  // configured, rescaled or live; what the tables were built from
  std::vector<float> xTable_;
  std::vector<float> yTable_;
  int tableW_{0};
  int tableH_{0};
  int imgW_{0};
  int imgH_{0};
  bool rescaleWarned_{false};
  ros::Time lastStamp_;

  std::mutex infoMutex_;
  Intrinsics liveInfo_;
  bool liveInfoSeen_{false};
  bool liveInfoPending_{false};
  bool liveInfoActive_{false};

  ros::Subscriber imageSub_;
  ros::Subscriber infoSub_;
  ros::Publisher cloudPub_;
  ros::Publisher infoPub_;
};

class DepthImageToPointCloudNode {
 public:
  DepthImageToPointCloudNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    loadGlobals(pnh);
    for (CameraConfig& cfg : loadCameras(pnh)) {
      cameras_.emplace_back(new DepthCamera(nh, std::move(cfg), globals_));
    }
    ROS_INFO_STREAM(kLogPrefix << cameras_.size() << " camera(s); depth range [" << globals_.depthMin << ", "
                               << globals_.depthMax << "] m, decimation " << globals_.decimation << ", max_rate "
                               << globals_.maxRate << " Hz (0 = every frame)");
  }

 private:
  void loadGlobals(ros::NodeHandle& pnh) {
    pnh.param("depth_min", globals_.depthMin, globals_.depthMin);
    pnh.param("depth_max", globals_.depthMax, globals_.depthMax);
    pnh.param("decimation", globals_.decimation, globals_.decimation);
    pnh.param("max_rate", globals_.maxRate, globals_.maxRate);
    pnh.param("publish_camera_info", globals_.publishCameraInfo, globals_.publishCameraInfo);
    pnh.param("use_live_camera_info", globals_.useLiveCameraInfo, globals_.useLiveCameraInfo);
    pnh.param("process_without_subscribers", globals_.processWithoutSubscribers,
              globals_.processWithoutSubscribers);
    pnh.param("image_queue_size", globals_.imageQueueSize, globals_.imageQueueSize);
    pnh.param("tcp_nodelay", globals_.tcpNoDelay, globals_.tcpNoDelay);
    pnh.param("depth_unit_m", globals_.depthUnitM, globals_.depthUnitM);
    pnh.param("image_topic_template", globals_.imageTopicTemplate, globals_.imageTopicTemplate);
    pnh.param("cloud_topic_template", globals_.cloudTopicTemplate, globals_.cloudTopicTemplate);
    pnh.param("camera_info_topic_template", globals_.cameraInfoTopicTemplate, globals_.cameraInfoTopicTemplate);

    globals_.decimation = std::max(1, globals_.decimation);
    globals_.imageQueueSize = std::max(1, globals_.imageQueueSize);
    if (!(globals_.depthMin >= 0.0 && globals_.depthMin < globals_.depthMax)) {
      throw std::runtime_error("depth_min must be >= 0 and smaller than depth_max");
    }
    if (!(globals_.depthUnitM > 0.0)) {
      throw std::runtime_error("depth_unit_m must be positive");
    }
    if (globals_.maxRate < 0.0) {
      globals_.maxRate = 0.0;
    }
  }

  std::vector<CameraConfig> loadCameras(ros::NodeHandle& pnh) {
    Intrinsics defaults;
    XmlRpc::XmlRpcValue defaultsValue;
    if (pnh.getParam("default_intrinsics", defaultsValue)) {
      if (defaultsValue.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("default_intrinsics must be a struct with fx fy cx cy width height");
      }
      readIntrinsics(defaultsValue, defaults);
    }

    XmlRpc::XmlRpcValue cams;
    if (!pnh.getParam("cameras", cams) || cams.getType() != XmlRpc::XmlRpcValue::TypeArray || cams.size() == 0) {
      throw std::runtime_error("'cameras' must be a non-empty list of {name, namespace, ...} entries");
    }

    std::vector<CameraConfig> configs;
    for (int i = 0; i < cams.size(); ++i) {
      XmlRpc::XmlRpcValue& entry = cams[i];
      const std::string where = "cameras[" + std::to_string(i) + "]";
      if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error(where + " must be a struct");
      }
      CameraConfig cfg;
      if (!readString(entry, "name", cfg.name) || cfg.name.empty()) {
        throw std::runtime_error(where + " has no 'name'");
      }
      if (!readString(entry, "namespace", cfg.ns)) {
        cfg.ns = "/depth_camera_" + cfg.name;
      }
      if (!readString(entry, "image_topic", cfg.imageTopic)) {
        cfg.imageTopic = substituteNamespace(globals_.imageTopicTemplate, cfg.ns);
      }
      if (!readString(entry, "cloud_topic", cfg.cloudTopic)) {
        cfg.cloudTopic = substituteNamespace(globals_.cloudTopicTemplate, cfg.ns);
      }
      if (!readString(entry, "camera_info_topic", cfg.cameraInfoTopic)) {
        cfg.cameraInfoTopic = substituteNamespace(globals_.cameraInfoTopicTemplate, cfg.ns);
      }
      readString(entry, "frame_id", cfg.frameIdOverride);
      if (cfg.imageTopic.empty() || cfg.cloudTopic.empty() || cfg.imageTopic == cfg.cloudTopic) {
        throw std::runtime_error(where + " (" + cfg.name + "): image_topic and cloud_topic must differ and be set");
      }
      cfg.intrinsics = defaults;
      readIntrinsics(entry, cfg.intrinsics);
      if (!cfg.intrinsics.valid()) {
        throw std::runtime_error(where + " (" + cfg.name +
                                 "): fx, fy, width and height must be positive (set them here or in "
                                 "default_intrinsics)");
      }
      configs.push_back(std::move(cfg));
    }
    return configs;
  }

  GlobalParams globals_;
  std::vector<std::unique_ptr<DepthCamera>> cameras_;
};

}  // namespace dynasense

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynasense_depth_image_to_pointcloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::unique_ptr<dynasense::DepthImageToPointCloudNode> node;
  try {
    node.reset(new dynasense::DepthImageToPointCloudNode(nh, pnh));
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("[dynasense_depth2pc] " << e.what());
    return 1;
  }

  // Cameras convert in parallel; roscpp never re-enters one camera's callback.
  const int threads = std::max(1, pnh.param("spinner_threads", 4));
  ros::AsyncSpinner spinner(static_cast<uint32_t>(threads));
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
