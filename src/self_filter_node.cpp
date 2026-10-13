// dynasense_self_filter_node
//
// Removes the robot's own body from point clouds and optionally clips them to a
// range band about the sensor, so that mapping (wavemap) or the BPS map never
// see the legs or the base as obstacles.
//
// The body is the set of <collision> primitives (box, cylinder, sphere) of the
// URDF held in a parameter, placed with TF at each cloud's header stamp. A
// point is dropped when its Euclidean distance to any primitive is at most
// body_margin + link_padding[link]; a point inside a primitive has distance 0.
// Meshes are not supported (anymal_d has none in <collision>).
//
// Inputs (per configured and enabled input)
//   <topic>                 sensor_msgs/PointCloud2 with float32 x y z (any layout)
//   TF: <cloud frame> <- every collision link, at the cloud stamp
//   URDF: parameter robot_description_param (default anymal_description)
//
// Outputs
//   <output_topic>          sensor_msgs/PointCloud2, unorganized xyz float32
//                           (point_step 16), same header stamp and frame_id;
//                           keep_organized writes NaN instead of dropping
//   ~body_markers           visualization_msgs/MarkerArray of the padded
//                           primitives in their link frames (when subscribed)
//
// Per-input statistics (kept/removed points, ms per cloud and per stage) are
// logged every stats_period seconds and summarised at shutdown.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <boost/make_shared.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace dynasense {
namespace {

constexpr const char* kLogPrefix = "[dynasense_self_filter] ";
using Clock = std::chrono::steady_clock;

/// Rotation + translation kept as Matrix3d/Vector3d, which Eigen does not
/// vectorise, so these structs can live in std::vector without aligned_allocator.
struct RigidTransform {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};

  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const { return R * p + t; }
  RigidTransform operator*(const RigidTransform& o) const { return {R * o.R, R * o.t + t}; }
  RigidTransform inverse() const { return {R.transpose(), -(R.transpose() * t)}; }
  static RigidTransform fromIsometry(const Eigen::Isometry3d& T) { return {T.linear(), T.translation()}; }
};

enum class Shape { kBox, kCylinder, kSphere };

struct Primitive {
  Shape type{Shape::kSphere};
  RigidTransform T_link_shape;  // <collision><origin> in the link frame
  Eigen::Vector3d half{Eigen::Vector3d::Zero()};  // box half extents
  double radius{0.0};                              // cylinder / sphere
  double halfLength{0.0};                          // cylinder, axis z
  double boundRadius{0.0};                         // circumscribed sphere about the shape origin
  std::string name;

  /// Euclidean distance from q (shape frame) to the solid; 0 inside.
  double distance(const Eigen::Vector3d& q) const {
    switch (type) {
      case Shape::kBox:
        return (q.cwiseAbs() - half).cwiseMax(0.0).norm();
      case Shape::kCylinder: {
        const double dr = std::max(0.0, std::hypot(q.x(), q.y()) - radius);
        const double dz = std::max(0.0, std::abs(q.z()) - halfLength);
        return std::hypot(dr, dz);
      }
      case Shape::kSphere:
        return std::max(0.0, q.norm() - radius);
    }
    return std::numeric_limits<double>::infinity();
  }

  const char* typeName() const {
    switch (type) {
      case Shape::kBox:
        return "box";
      case Shape::kCylinder:
        return "cylinder";
      case Shape::kSphere:
        return "sphere";
    }
    return "?";
  }
};

struct BodyLink {
  std::string frame;  // URDF link name == TF frame
  std::vector<Primitive> prims;
  double threshold{0.0};  // body_margin + link_padding[frame]
  Eigen::Vector3d boundCenter{Eigen::Vector3d::Zero()};
  double boundRadius{0.0};  // bounding sphere of all primitives, threshold included
};

/// A primitive placed in a cloud's frame for one callback.
struct PlacedPrimitive {
  const Primitive* prim;
  RigidTransform T_shape_cloud;
  double threshold;
};

struct PlacedLink {
  Eigen::Vector3d center;
  double radius2;
  std::vector<PlacedPrimitive> prims;
};

struct GlobalParams {
  std::string robotDescriptionParam{"anymal_description"};
  double bodyMargin{0.05};
  std::map<std::string, double> linkPadding;
  std::set<std::string> linksIgnore;
  double maxRange{0.8};  // [m] 0 = off
  double minRange{0.0};  // [m]
  bool keepOrganized{false};
  double tfTimeout{0.05};  // [s]
  bool tfFallbackLatest{false};
  bool publishEmpty{false};
  bool processWithoutSubscribers{false};
  int queueSize{2};
  bool tcpNoDelay{true};
  double markerRate{2.0};   // [Hz] 0 = off
  double statsPeriod{10.0};  // [s] 0 = off
  std::string outputTopicSuffix{"_self_filtered"};
};

struct InputConfig {
  std::string name;
  std::string topic;
  std::string outputTopic;
  bool enabled{true};
  double maxRange{-1.0};  // < 0: global value
  double minRange{-1.0};
};

struct Stats {
  uint64_t clouds{0};
  uint64_t tfDrops{0};
  uint64_t pointsIn{0};
  uint64_t kept{0};
  uint64_t nan{0};
  uint64_t range{0};
  uint64_t body{0};
  double tfMs{0.0};
  double filterMs{0.0};
  double publishMs{0.0};
  double totalMs{0.0};
  double maxTotalMs{0.0};
};

// ---------------------------------------------------------------------------
// Parameter helpers (same conventions as depth_image_to_pointcloud_node.cpp)

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

bool readBool(XmlRpc::XmlRpcValue& parent, const std::string& key, bool& out) {
  if (!parent.hasMember(key)) {
    return false;
  }
  if (parent[key].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    throw std::runtime_error("parameter '" + key + "' must be a boolean");
  }
  out = static_cast<bool>(parent[key]);
  return true;
}

/// Splits on whitespace, commas and semicolons.
std::vector<std::string> splitNames(const std::string& text) {
  std::vector<std::string> names;
  std::string current;
  for (const char c : text) {
    if (c == ' ' || c == ',' || c == ';' || c == '\t' || c == '\n') {
      if (!current.empty()) {
        names.push_back(current);
        current.clear();
      }
    } else {
      current.push_back(c);
    }
  }
  if (!current.empty()) {
    names.push_back(current);
  }
  return names;
}

bool hostIsBigEndian() {
  const uint16_t probe = 1;
  uint8_t first = 0;
  std::memcpy(&first, &probe, 1);
  return first == 0;
}

bool hasXyzFloat32(const sensor_msgs::PointCloud2& cloud) {
  int found = 0;
  for (const sensor_msgs::PointField& f : cloud.fields) {
    if ((f.name == "x" || f.name == "y" || f.name == "z") && f.datatype == sensor_msgs::PointField::FLOAT32 &&
        f.count == 1) {
      ++found;
    }
  }
  return found == 3;
}

RigidTransform poseFromUrdf(const urdf::Pose& p) {
  RigidTransform T;
  T.t = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  p.rotation.getQuaternion(qx, qy, qz, qw);
  T.R = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  return T;
}

geometry_msgs::Pose poseMsg(const RigidTransform& T) {
  geometry_msgs::Pose pose;
  pose.position.x = T.t.x();
  pose.position.y = T.t.y();
  pose.position.z = T.t.z();
  const Eigen::Quaterniond q(T.R);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

/// Collision primitives of every link of the URDF in parameter `param`.
std::vector<BodyLink> loadBody(const std::string& param, const GlobalParams& g, int& primitives, int& meshes) {
  urdf::Model model;
  if (!model.initParam(param)) {
    throw std::runtime_error("cannot load a URDF from parameter '" + param + "'");
  }
  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);

  std::vector<BodyLink> body;
  primitives = 0;
  meshes = 0;
  for (const urdf::LinkSharedPtr& link : links) {
    if (!link || g.linksIgnore.count(link->name) != 0) {
      continue;
    }
    BodyLink L;
    L.frame = link->name;
    for (std::size_t i = 0; i < link->collision_array.size(); ++i) {
      const urdf::CollisionSharedPtr& col = link->collision_array[i];
      if (!col || !col->geometry) {
        continue;
      }
      Primitive P;
      P.T_link_shape = poseFromUrdf(col->origin);
      P.name = link->name + "/collision[" + std::to_string(i) + "]";
      switch (col->geometry->type) {
        case urdf::Geometry::BOX: {
          const auto box = std::static_pointer_cast<urdf::Box>(col->geometry);
          P.type = Shape::kBox;
          P.half = 0.5 * Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z);
          P.boundRadius = P.half.norm();
          break;
        }
        case urdf::Geometry::CYLINDER: {
          const auto cyl = std::static_pointer_cast<urdf::Cylinder>(col->geometry);
          P.type = Shape::kCylinder;
          P.radius = cyl->radius;
          P.halfLength = 0.5 * cyl->length;
          P.boundRadius = std::hypot(P.radius, P.halfLength);
          break;
        }
        case urdf::Geometry::SPHERE: {
          const auto sph = std::static_pointer_cast<urdf::Sphere>(col->geometry);
          P.type = Shape::kSphere;
          P.radius = sph->radius;
          P.boundRadius = sph->radius;
          break;
        }
        default:
          ++meshes;
          ROS_WARN_STREAM(kLogPrefix << P.name << " is a mesh; meshes are not supported and it is skipped");
          continue;
      }
      L.prims.push_back(std::move(P));
    }
    if (L.prims.empty()) {
      continue;
    }
    const auto pad = g.linkPadding.find(L.frame);
    L.threshold = g.bodyMargin + (pad == g.linkPadding.end() ? 0.0 : pad->second);
    for (const Primitive& P : L.prims) {
      L.boundCenter += P.T_link_shape.t;
    }
    L.boundCenter /= static_cast<double>(L.prims.size());
    for (const Primitive& P : L.prims) {
      L.boundRadius = std::max(L.boundRadius, (P.T_link_shape.t - L.boundCenter).norm() + P.boundRadius);
    }
    L.boundRadius += L.threshold;
    primitives += static_cast<int>(L.prims.size());
    body.push_back(std::move(L));
  }
  for (const auto& kv : g.linkPadding) {
    const bool known = std::any_of(body.begin(), body.end(), [&](const BodyLink& L) { return L.frame == kv.first; });
    if (!known) {
      ROS_WARN_STREAM(kLogPrefix << "link_padding names '" << kv.first << "', which has no collision primitives");
    }
  }
  if (body.empty()) {
    throw std::runtime_error("the URDF in '" + param + "' has no box/cylinder/sphere collision geometry");
  }
  return body;
}

}  // namespace

/// One input cloud stream: subscribe, filter, publish, count.
class FilteredInput {
 public:
  FilteredInput(ros::NodeHandle& nh, InputConfig config, const GlobalParams& globals,
                const std::vector<BodyLink>& body, tf2_ros::Buffer& tfBuffer)
      : cfg_(std::move(config)), g_(globals), body_(body), tf_(tfBuffer) {
    maxRange_ = cfg_.maxRange >= 0.0 ? cfg_.maxRange : g_.maxRange;
    minRange_ = cfg_.minRange >= 0.0 ? cfg_.minRange : g_.minRange;
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(cfg_.outputTopic, 1);
    ros::TransportHints hints;
    if (g_.tcpNoDelay) {
      hints = ros::TransportHints().tcpNoDelay();
    }
    sub_ = nh.subscribe(cfg_.topic, static_cast<uint32_t>(g_.queueSize), &FilteredInput::onCloud, this, hints);
    ROS_INFO_STREAM(kLogPrefix << cfg_.name << ": " << cfg_.topic << " -> " << cfg_.outputTopic << "  range ("
                               << minRange_ << ", " << (maxRange_ > 0.0 ? std::to_string(maxRange_) : "inf") << "] m");
  }

  const InputConfig& config() const { return cfg_; }

  Stats snapshot() const {
    std::lock_guard<std::mutex> lock(statsMutex_);
    return stats_;
  }

 private:
  void onCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!g_.processWithoutSubscribers && pub_.getNumSubscribers() == 0) {
      return;
    }
    const std::size_t n = static_cast<std::size_t>(msg->width) * msg->height;
    if (n == 0) {
      return;
    }
    if (!hasXyzFloat32(*msg)) {
      ROS_WARN_STREAM_THROTTLE(5.0, kLogPrefix << cfg_.name << ": cloud without float32 x y z fields; dropped");
      return;
    }
    const auto t0 = Clock::now();

    std::vector<PlacedLink> placed;
    if (!placeBody(msg->header, placed)) {
      std::lock_guard<std::mutex> lock(statsMutex_);
      ++stats_.tfDrops;
      return;
    }
    const auto t1 = Clock::now();

    auto out = boost::make_shared<sensor_msgs::PointCloud2>();
    out->header = msg->header;  // downstream TF lookups use this stamp and frame
    out->is_bigendian = hostIsBigEndian();
    if (g_.keepOrganized) {
      out->height = msg->height;
      out->width = msg->width;
      out->is_dense = false;
    } else {
      out->height = 1;
      out->width = static_cast<uint32_t>(n);  // upper bound, shrunk below
      out->is_dense = true;
    }
    sensor_msgs::PointCloud2Modifier modifier(*out);
    modifier.setPointCloud2FieldsByString(1, "xyz");  // x y z float32 + padding, point_step 16

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    sensor_msgs::PointCloud2Iterator<float> ox(*out, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(*out, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(*out, "z");

    const double maxR2 = maxRange_ > 0.0 ? maxRange_ * maxRange_ : std::numeric_limits<double>::infinity();
    const double minR2 = minRange_ * minRange_;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    uint64_t kept = 0, nNan = 0, nRange = 0, nBody = 0;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix;
      const float y = *iy;
      const float z = *iz;
      bool keep = true;
      if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) {
        keep = false;
        ++nNan;
      } else {
        const Eigen::Vector3d p(x, y, z);
        const double r2 = p.squaredNorm();  // the sensor origin is the cloud frame origin
        if (r2 > maxR2 || r2 < minR2) {
          keep = false;
          ++nRange;
        } else if (nearBody(p, placed)) {
          keep = false;
          ++nBody;
        }
      }
      if (keep) {
        *ox = x;
        *oy = y;
        *oz = z;
        ++ox;
        ++oy;
        ++oz;
        ++kept;
      } else if (g_.keepOrganized) {
        *ox = nan;
        *oy = nan;
        *oz = nan;
        ++ox;
        ++oy;
        ++oz;
      }
    }
    if (!g_.keepOrganized) {
      modifier.resize(kept);  // height stays 1: width, row_step and data shrink to the kept points
    }
    const auto t2 = Clock::now();

    if (kept > 0 || g_.publishEmpty) {
      pub_.publish(out);
    }
    const auto t3 = Clock::now();

    const auto ms = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    std::lock_guard<std::mutex> lock(statsMutex_);
    ++stats_.clouds;
    stats_.pointsIn += n;
    stats_.kept += kept;
    stats_.nan += nNan;
    stats_.range += nRange;
    stats_.body += nBody;
    stats_.tfMs += ms(t0, t1);
    stats_.filterMs += ms(t1, t2);
    stats_.publishMs += ms(t2, t3);
    const double total = ms(t0, t3);
    stats_.totalMs += total;
    stats_.maxTotalMs = std::max(stats_.maxTotalMs, total);
  }

  static bool nearBody(const Eigen::Vector3d& p, const std::vector<PlacedLink>& links) {
    for (const PlacedLink& L : links) {
      if ((p - L.center).squaredNorm() > L.radius2) {
        continue;  // bounding-sphere reject
      }
      for (const PlacedPrimitive& P : L.prims) {
        if (P.prim->distance(P.T_shape_cloud * p) <= P.threshold) {
          return true;
        }
      }
    }
    return false;
  }

  /// Places every collision link in the cloud frame at the cloud stamp.
  bool placeBody(const std_msgs::Header& h, std::vector<PlacedLink>& placed) {
    placed.clear();
    placed.reserve(body_.size());
    for (const BodyLink& L : body_) {
      geometry_msgs::TransformStamped tf;
      try {
        // Waits only when the transform is not there yet (a cloud can precede
        // the TF sample at its own stamp by a millisecond or so).
        tf = tf_.lookupTransform(h.frame_id, L.frame, h.stamp, ros::Duration(g_.tfTimeout));
      } catch (const tf2::TransformException& ex) {
        if (!g_.tfFallbackLatest) {
          ROS_WARN_STREAM_THROTTLE(5.0, kLogPrefix << cfg_.name << ": no TF " << h.frame_id << " <- " << L.frame
                                                   << " at " << h.stamp << " (" << ex.what() << "); cloud dropped");
          return false;
        }
        try {
          tf = tf_.lookupTransform(h.frame_id, L.frame, ros::Time(0));
        } catch (const tf2::TransformException& ex2) {
          ROS_WARN_STREAM_THROTTLE(5.0, kLogPrefix << cfg_.name << ": no TF " << h.frame_id << " <- " << L.frame
                                                   << " at all (" << ex2.what() << "); cloud dropped");
          return false;
        }
      }
      const RigidTransform T_cloud_link = RigidTransform::fromIsometry(tf2::transformToEigen(tf));
      PlacedLink PL;
      PL.center = T_cloud_link * L.boundCenter;
      PL.radius2 = L.boundRadius * L.boundRadius;
      PL.prims.reserve(L.prims.size());
      for (const Primitive& P : L.prims) {
        PL.prims.push_back({&P, (T_cloud_link * P.T_link_shape).inverse(), L.threshold});
      }
      placed.push_back(std::move(PL));
    }
    return true;
  }

  InputConfig cfg_;
  const GlobalParams& g_;
  const std::vector<BodyLink>& body_;
  tf2_ros::Buffer& tf_;
  double maxRange_{0.0};
  double minRange_{0.0};

  ros::Subscriber sub_;
  ros::Publisher pub_;

  mutable std::mutex statsMutex_;
  Stats stats_;
};

class SelfFilterNode {
 public:
  SelfFilterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tfListener_(tfBuffer_) {
    loadGlobals(pnh);
    int primitives = 0, meshes = 0;
    body_ = loadBody(globals_.robotDescriptionParam, globals_, primitives, meshes);
    ROS_INFO_STREAM(kLogPrefix << body_.size() << " link(s) with " << primitives << " collision primitive(s) from '"
                               << globals_.robotDescriptionParam << "'" << (meshes ? " (meshes skipped)" : "")
                               << "; body_margin " << globals_.bodyMargin << " m, max_range "
                               << (globals_.maxRange > 0.0 ? std::to_string(globals_.maxRange) : "off")
                               << ", min_range " << globals_.minRange);
    for (const BodyLink& L : body_) {
      std::ostringstream shapes;
      for (const Primitive& P : L.prims) {
        shapes << " " << P.typeName();
      }
      ROS_INFO_STREAM(kLogPrefix << "  " << L.frame << ":" << shapes.str() << "  (threshold " << L.threshold
                                 << " m, bounding radius " << std::setprecision(3) << L.boundRadius << " m)");
    }

    int enabled = 0;
    for (InputConfig& cfg : loadInputs(pnh)) {
      if (!cfg.enabled) {
        ROS_INFO_STREAM(kLogPrefix << cfg.name << ": disabled");
        continue;
      }
      inputs_.emplace_back(new FilteredInput(nh, std::move(cfg), globals_, body_, tfBuffer_));
      ++enabled;
    }
    if (enabled == 0) {
      ROS_WARN_STREAM(kLogPrefix << "no input is enabled; the node idles");
    }

    if (globals_.markerRate > 0.0) {
      markerPub_ = pnh.advertise<visualization_msgs::MarkerArray>("body_markers", 1, /*latch=*/true);
      markerTimer_ = nh.createWallTimer(ros::WallDuration(1.0 / globals_.markerRate), &SelfFilterNode::onMarkerTimer,
                                        this);
    }
    if (globals_.statsPeriod > 0.0) {
      statsTimer_ = nh.createWallTimer(ros::WallDuration(globals_.statsPeriod), &SelfFilterNode::onStatsTimer, this);
    }
  }

  ~SelfFilterNode() {
    markerTimer_.stop();
    statsTimer_.stop();
    logStats(true);
  }

 private:
  void loadGlobals(ros::NodeHandle& pnh) {
    pnh.param("robot_description_param", globals_.robotDescriptionParam, globals_.robotDescriptionParam);
    pnh.param("body_margin", globals_.bodyMargin, globals_.bodyMargin);
    pnh.param("max_range", globals_.maxRange, globals_.maxRange);
    pnh.param("min_range", globals_.minRange, globals_.minRange);
    pnh.param("keep_organized", globals_.keepOrganized, globals_.keepOrganized);
    pnh.param("tf_timeout", globals_.tfTimeout, globals_.tfTimeout);
    pnh.param("tf_fallback_latest", globals_.tfFallbackLatest, globals_.tfFallbackLatest);
    pnh.param("publish_empty", globals_.publishEmpty, globals_.publishEmpty);
    pnh.param("process_without_subscribers", globals_.processWithoutSubscribers,
              globals_.processWithoutSubscribers);
    pnh.param("queue_size", globals_.queueSize, globals_.queueSize);
    pnh.param("tcp_nodelay", globals_.tcpNoDelay, globals_.tcpNoDelay);
    pnh.param("marker_rate", globals_.markerRate, globals_.markerRate);
    pnh.param("stats_period", globals_.statsPeriod, globals_.statsPeriod);
    pnh.param("output_topic_suffix", globals_.outputTopicSuffix, globals_.outputTopicSuffix);

    std::vector<std::string> ignore;
    pnh.param("links_ignore", ignore, ignore);
    globals_.linksIgnore.insert(ignore.begin(), ignore.end());

    XmlRpc::XmlRpcValue padding;
    if (pnh.getParam("link_padding", padding)) {
      if (padding.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("link_padding must be a map {link: metres}");
      }
      for (auto it = padding.begin(); it != padding.end(); ++it) {
        double value = 0.0;
        if (!asNumber(it->second, value)) {
          throw std::runtime_error("link_padding['" + it->first + "'] must be a number");
        }
        globals_.linkPadding[it->first] = value;
      }
    }

    if (globals_.bodyMargin < 0.0) {
      throw std::runtime_error("body_margin must be >= 0");
    }
    if (globals_.maxRange < 0.0) {
      globals_.maxRange = 0.0;
    }
    if (globals_.minRange < 0.0) {
      globals_.minRange = 0.0;
    }
    if (globals_.maxRange > 0.0 && globals_.minRange >= globals_.maxRange) {
      throw std::runtime_error("min_range must be smaller than max_range");
    }
    globals_.queueSize = std::max(1, globals_.queueSize);
    globals_.tfTimeout = std::max(0.0, globals_.tfTimeout);
  }

  std::vector<InputConfig> loadInputs(ros::NodeHandle& pnh) {
    XmlRpc::XmlRpcValue list;
    if (!pnh.getParam("inputs", list) || list.getType() != XmlRpc::XmlRpcValue::TypeArray || list.size() == 0) {
      throw std::runtime_error("'inputs' must be a non-empty list of {name, topic, ...} entries");
    }
    std::vector<InputConfig> inputs;
    for (int i = 0; i < list.size(); ++i) {
      XmlRpc::XmlRpcValue& entry = list[i];
      const std::string where = "inputs[" + std::to_string(i) + "]";
      if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error(where + " must be a struct");
      }
      InputConfig cfg;
      if (!readString(entry, "name", cfg.name) || cfg.name.empty()) {
        throw std::runtime_error(where + " has no 'name'");
      }
      if (!readString(entry, "topic", cfg.topic) || cfg.topic.empty()) {
        throw std::runtime_error(where + " (" + cfg.name + ") has no 'topic'");
      }
      if (!readString(entry, "output_topic", cfg.outputTopic)) {
        cfg.outputTopic = cfg.topic + globals_.outputTopicSuffix;
      }
      if (cfg.outputTopic.empty() || cfg.outputTopic == cfg.topic) {
        throw std::runtime_error(where + " (" + cfg.name + "): output_topic must differ from topic");
      }
      readBool(entry, "enabled", cfg.enabled);
      readNumber(entry, "max_range", cfg.maxRange);
      readNumber(entry, "min_range", cfg.minRange);
      for (const InputConfig& other : inputs) {
        if (other.name == cfg.name) {
          throw std::runtime_error("duplicate input name '" + cfg.name + "'");
        }
      }
      inputs.push_back(std::move(cfg));
    }

    // enabled_inputs (a name list) replaces the per-entry flags when present.
    std::string enabledText;
    if (pnh.getParam("enabled_inputs", enabledText)) {
      const std::vector<std::string> names = splitNames(enabledText);
      for (const std::string& name : names) {
        const bool known = std::any_of(inputs.begin(), inputs.end(), [&](const InputConfig& c) { return c.name == name; });
        if (!known) {
          throw std::runtime_error("enabled_inputs names unknown input '" + name + "'");
        }
      }
      for (InputConfig& cfg : inputs) {
        cfg.enabled = std::find(names.begin(), names.end(), cfg.name) != names.end();
      }
    }
    return inputs;
  }

  void onMarkerTimer(const ros::WallTimerEvent&) {
    if (markerPub_.getNumSubscribers() == 0) {
      return;
    }
    visualization_msgs::MarkerArray markers;
    int id = 0;
    for (const BodyLink& L : body_) {
      for (const Primitive& P : L.prims) {
        visualization_msgs::Marker m;
        m.header.frame_id = L.frame;
        m.header.stamp = ros::Time::now();
        m.ns = L.frame;
        m.id = id++;
        m.action = visualization_msgs::Marker::ADD;
        m.frame_locked = true;  // follows the link as the robot moves
        m.pose = poseMsg(P.T_link_shape);
        const double pad = L.threshold;
        switch (P.type) {
          case Shape::kBox:
            m.type = visualization_msgs::Marker::CUBE;
            m.scale.x = 2.0 * (P.half.x() + pad);
            m.scale.y = 2.0 * (P.half.y() + pad);
            m.scale.z = 2.0 * (P.half.z() + pad);
            break;
          case Shape::kCylinder:
            m.type = visualization_msgs::Marker::CYLINDER;
            m.scale.x = m.scale.y = 2.0 * (P.radius + pad);
            m.scale.z = 2.0 * (P.halfLength + pad);
            break;
          case Shape::kSphere:
            m.type = visualization_msgs::Marker::SPHERE;
            m.scale.x = m.scale.y = m.scale.z = 2.0 * (P.radius + pad);
            break;
        }
        m.color.r = 1.0f;
        m.color.g = 0.55f;
        m.color.b = 0.0f;
        m.color.a = 0.35f;
        markers.markers.push_back(m);
      }
    }
    markerPub_.publish(markers);
  }

  void onStatsTimer(const ros::WallTimerEvent&) { logStats(false); }

  void logStats(bool final) {
    Stats all;
    std::ostringstream table;
    table << std::fixed << std::setprecision(2);
    for (const auto& input : inputs_) {
      const Stats s = input->snapshot();
      if (s.clouds == 0 && s.tfDrops == 0) {
        table << "\n  " << std::setw(12) << std::left << input->config().name << " no clouds yet";
        continue;
      }
      const double perCloud = s.clouds ? s.totalMs / s.clouds : 0.0;
      const double usPerPoint = s.pointsIn ? 1000.0 * s.totalMs / s.pointsIn : 0.0;
      const double pct = s.pointsIn ? 100.0 / s.pointsIn : 0.0;
      table << "\n  " << std::setw(12) << std::left << input->config().name << std::right << std::setw(6) << s.clouds
            << " clouds" << (s.tfDrops ? " (" + std::to_string(s.tfDrops) + " TF drops)" : "") << ", "
            << (s.clouds ? s.pointsIn / s.clouds : 0) << " pts/cloud -> kept " << std::setw(5) << pct * s.kept
            << " % (nan " << pct * s.nan << ", range " << pct * s.range << ", body " << pct * s.body << ") | "
            << perCloud << " ms/cloud (tf " << (s.clouds ? s.tfMs / s.clouds : 0.0) << ", filter "
            << (s.clouds ? s.filterMs / s.clouds : 0.0) << ", publish " << (s.clouds ? s.publishMs / s.clouds : 0.0)
            << "), max " << s.maxTotalMs << " ms, " << std::setprecision(3) << usPerPoint << " us/pt"
            << std::setprecision(2);
      all.clouds += s.clouds;
      all.tfDrops += s.tfDrops;
      all.pointsIn += s.pointsIn;
      all.kept += s.kept;
      all.totalMs += s.totalMs;
      all.maxTotalMs = std::max(all.maxTotalMs, s.maxTotalMs);
    }
    if (all.clouds > 0) {
      table << "\n  " << std::setw(12) << std::left << "all" << std::right << std::setw(6) << all.clouds << " clouds, "
            << all.pointsIn << " pts -> kept " << (100.0 * all.kept / all.pointsIn) << " % | "
            << (all.totalMs / all.clouds) << " ms/cloud mean, " << all.maxTotalMs << " ms max, "
            << std::setprecision(3) << (1000.0 * all.totalMs / all.pointsIn) << " us/pt, " << std::setprecision(2)
            << (all.totalMs / 1000.0) << " s of filtering in total";
    }
    ROS_INFO_STREAM(kLogPrefix << (final ? "final statistics:" : "statistics:") << table.str());
  }

  GlobalParams globals_;
  std::vector<BodyLink> body_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  std::vector<std::unique_ptr<FilteredInput>> inputs_;
  ros::Publisher markerPub_;
  ros::WallTimer markerTimer_;
  ros::WallTimer statsTimer_;
};

}  // namespace dynasense

namespace {
std::atomic<bool> g_stopRequested{false};
void onStopSignal(int) { g_stopRequested = true; }
}  // namespace

int main(int argc, char** argv) {
  // Own SIGINT/SIGTERM handling: the final statistics must be logged before
  // ros::shutdown() tears rosconsole down.
  ros::init(argc, argv, "dynasense_self_filter_node", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, onStopSignal);
  std::signal(SIGTERM, onStopSignal);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::unique_ptr<dynasense::SelfFilterNode> node;
  try {
    node.reset(new dynasense::SelfFilterNode(nh, pnh));
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("[dynasense_self_filter] " << e.what());
    return 1;
  }

  // Inputs filter in parallel; roscpp never re-enters one input's callback.
  const int threads = std::max(1, pnh.param("spinner_threads", 4));
  ros::AsyncSpinner spinner(static_cast<uint32_t>(threads));
  spinner.start();
  while (ros::ok() && !g_stopRequested) {
    ros::WallDuration(0.1).sleep();
  }
  spinner.stop();
  node.reset();  // logs the final statistics while rosconsole is still up
  std::fflush(stdout);
  ros::shutdown();
  return 0;
}
