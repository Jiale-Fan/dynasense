// dynasense_bps_node
//
// Maintains the contact-history BPS map for the ContactBPS locomotion policy
// and publishes the 102-dimensional direction vector the policy consumes as
// observation block [49,151).
//
// Inputs
//   /contacts/{LF,LH,RF,RH}/{thigh,shank}   gazebo_msgs/ContactsState
//   /depth_camera_*/point_cloud_self_filtered + lidar  sensor_msgs/PointCloud2
//   TF: odom <- base, *_THIGH, *_SHANK, *_FOOT
//
// Outputs
//   /dynasense/bps_state     dynasense/BpsState        (policy rate)
//   /dynasense/bps_markers   visualization_msgs/MarkerArray (debug)
//   service /dynasense/bps/reset  std_srvs/Empty
//
// See DS_LOCORESET_DEPTHHEIGHTSCAN_CONTACTBPS_DEPLOYMENT_HANDOVER.md.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <gazebo_msgs/ContactsState.h>

#include "dynasense/BpsMap.hpp"
#include "dynasense/BpsState.h"

namespace dynasense {
namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

/// Yaw-only rotation extracted from a full rotation matrix.
Eigen::Matrix3d yawOnly(const Eigen::Matrix3d& R) {
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

/// Matplotlib "inferno" colormap for t in [0,1]: near-black at 0, light yellow
/// at 1. 17 control points lifted from matplotlib's 256-entry table and
/// linearly interpolated; worst per-channel error against the full table is
/// 6/255, which is not visible in a marker.
Eigen::Vector3f inferno(double t) {
  static const float kTable[17][3] = {
      {0.001462f, 0.000466f, 0.013866f}, {0.041966f, 0.028000f, 0.140584f}, {0.128488f, 0.047323f, 0.289642f},
      {0.237010f, 0.036581f, 0.395612f}, {0.339929f, 0.061759f, 0.429200f}, {0.439263f, 0.098629f, 0.431733f},
      {0.538581f, 0.133906f, 0.415702f}, {0.637450f, 0.170368f, 0.382220f}, {0.732796f, 0.214332f, 0.332053f},
      {0.819548f, 0.272810f, 0.268507f}, {0.891827f, 0.349998f, 0.196523f}, {0.945123f, 0.444778f, 0.118766f},
      {0.977425f, 0.552622f, 0.038020f}, {0.987932f, 0.669158f, 0.059628f}, {0.975924f, 0.790939f, 0.197307f},
      {0.948615f, 0.910906f, 0.396250f}, {0.988362f, 0.998364f, 0.644924f}};
  constexpr int kLast = 16;

  t = std::min(std::max(t, 0.0), 1.0);
  const double x = t * kLast;
  const int lo = std::min(static_cast<int>(x), kLast);
  const int hi = std::min(lo + 1, kLast);
  const float f = static_cast<float>(x - lo);

  return Eigen::Vector3f(kTable[lo][0] * (1.0f - f) + kTable[hi][0] * f,
                         kTable[lo][1] * (1.0f - f) + kTable[hi][1] * f,
                         kTable[lo][2] * (1.0f - f) + kTable[hi][2] * f);
}

/// One depth camera or lidar, described by the ray pattern the policy was
/// trained on. Incoming clouds are binned onto this pattern (nearest hit per
/// cell) so the BPS map is fed at the trained ray density rather than at the
/// full sensor resolution.
struct RaySource {
  std::string name;
  std::string topic;
  int cols{24};
  int rows{16};
  double hMin{0.0};  ///< [rad]
  double hMax{0.0};
  double vMin{0.0};
  double vMax{0.0};
  /// True for ROS optical frames (x right, y down, z forward); false for body
  /// frames (x forward, y left, z up).
  bool opticalFrame{true};
  double maxRange{1.0};

  ros::Subscriber sub;
};

}  // namespace

class BpsNode {
 public:
  BpsNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), tfListener_(tfBuffer_) {
    loadParameters();

    map_.configure(mapConfig_);
    queryUnitDirections_.assign(static_cast<std::size_t>(kNumQueryPoints), Eigen::Vector3d::UnitZ());

    statePub_ = nh_.advertise<dynasense::BpsState>(statePublishTopic_, 1);
    if (publishMarkers_) {
      markerPub_ = nh_.advertise<visualization_msgs::MarkerArray>(markerTopic_, 1);
    }
    resetServer_ = nh_.advertiseService(resetService_, &BpsNode::onReset, this);

    subscribeContacts();
    subscribeRaySources();

    timer_ = nh_.createTimer(ros::Duration(1.0 / updateRate_), &BpsNode::onTimer, this);

    ROS_INFO_STREAM("[dynasense_bps] up: " << contactSubs_.size() << " contact topics, " << raySources_.size()
                                           << " ray sources, publishing " << statePublishTopic_ << " at "
                                           << updateRate_ << " Hz");

    if (!mapConfig_.contactEnabled && !mapConfig_.rayEnabled) {
      ROS_WARN("[dynasense_bps] BOTH voxel-activation paths are disabled: the BPS map will stay "
               "permanently empty, so all 102 BPS observation values are exactly zero and the "
               "policy is BLIND TO OBSTACLES. Set enable_contact_voxels / enable_ray_voxels in "
               "config/bps.yaml to re-enable.");
    }
  }

 private:
  // -------------------------------------------------------------------------
  // Setup
  // -------------------------------------------------------------------------

  void loadParameters() {
    pnh_.param<std::string>("odom_frame", odomFrame_, "odom");
    pnh_.param<std::string>("base_frame", baseFrame_, "base");
    pnh_.param<std::string>("state_topic", statePublishTopic_, "/dynasense/bps_state");
    pnh_.param<std::string>("marker_topic", markerTopic_, "/dynasense/bps_markers");
    pnh_.param<std::string>("reset_service", resetService_, "/dynasense/bps/reset");
    pnh_.param<bool>("publish_markers", publishMarkers_, true);
    pnh_.param<bool>("publish_voxel_cubes", publishVoxelCubes_, true);
    pnh_.param<bool>("publish_query_arrows", publishQueryArrows_, true);
    pnh_.param<double>("arrow_length", arrowLength_, 0.3);
    int markerMaxCount = 20000;
    pnh_.param<int>("marker_max_count", markerMaxCount, markerMaxCount);
    markerMaxCount_ = static_cast<std::size_t>(std::max(0, markerMaxCount));
    pnh_.param<double>("update_rate", updateRate_, 50.0);
    pnh_.param<double>("tf_timeout", tfTimeout_, 0.02);
    pnh_.param<std::string>("robot_model_name", robotModelName_, "anymal");

    // The two voxel-activation paths. A disabled path is not subscribed at all
    // and its LRU buffer is given capacity 0.
    pnh_.param<bool>("enable_contact_voxels", mapConfig_.contactEnabled, true);
    pnh_.param<bool>("enable_ray_voxels", mapConfig_.rayEnabled, true);

    pnh_.param<double>("contact_force_threshold", contactForceThreshold_, 1.0);
    pnh_.param<bool>("contact_rising_edge_only", contactRisingEdgeOnly_, true);
    pnh_.param<std::string>("contact_topic_format", contactTopicFormat_, "/contacts/{leg}/{body}");

    // Which frame gazebo_ros_bumper's contact_positions are actually in.
    //   "odom"   -- verbatim, no transform. The bumper reports Gazebo WORLD
    //               coordinates, and the Gazebo world coincides with odom in
    //               this sim (the ground-clearance plugin relies on the same
    //               thing: it publishes raw world hit points labelled odom).
    //   "header" -- treat them as local to msg->header.frame_id and transform
    //               odom <- that frame. Correct only if the bumper honours its
    //               <frameName>, which it is not trusted to do.
    // Anything else falls back to "odom" with a warning.
    pnh_.param<std::string>("contact_position_frame", contactPositionFrame_, "odom");
    if (contactPositionFrame_ != "odom" && contactPositionFrame_ != "header") {
      ROS_WARN_STREAM("[dynasense_bps] contact_position_frame '" << contactPositionFrame_
                                                                 << "' unknown; using 'odom'");
      contactPositionFrame_ = "odom";
    }
    pnh_.param<int>("contact_frame_debug_count", contactFrameDebugCount_, 10);

    double voxelSize = 0.05;
    double maxQueryDistance = 0.5;
    int contactCapacity = 200;
    int rayCapacity = 500;
    pnh_.param<double>("voxel_size", voxelSize, voxelSize);
    pnh_.param<double>("max_query_distance", maxQueryDistance, maxQueryDistance);
    pnh_.param<int>("contact_buffer_capacity", contactCapacity, contactCapacity);
    pnh_.param<int>("ray_buffer_capacity", rayCapacity, rayCapacity);
    pnh_.param<double>("ground_z", mapConfig_.groundZ, 0.0);
    pnh_.param<double>("min_height_above_ground", mapConfig_.minHeightAboveGround, 0.05);
    pnh_.param<double>("max_height_above_ground", mapConfig_.maxHeightAboveGround, 1.8);
    pnh_.param<double>("max_horizontal_range", mapConfig_.maxHorizontalRange, 4.0);

    mapConfig_.voxelSize = voxelSize;
    mapConfig_.maxQueryDistance = maxQueryDistance;
    mapConfig_.contactCapacity = static_cast<std::size_t>(std::max(0, contactCapacity));
    mapConfig_.rayCapacity = static_cast<std::size_t>(std::max(0, rayCapacity));
  }

  static std::string substitute(std::string format, const std::string& leg, const std::string& body) {
    const auto replace = [&format](const std::string& token, const std::string& value) {
      for (std::size_t pos = format.find(token); pos != std::string::npos; pos = format.find(token, pos)) {
        format.replace(pos, token.size(), value);
        pos += value.size();
      }
    };
    replace("{leg}", leg);
    replace("{body}", body);
    return format;
  }

  void subscribeContacts() {
    if (!mapConfig_.contactEnabled) {
      ROS_WARN("[dynasense_bps] physical-contact voxel activation DISABLED "
               "(enable_contact_voxels: false); not subscribing to any /contacts topic");
      return;
    }

    // {body} values substituted into contact_topic_format. One Gazebo contact
    // sensor per (leg, body); each keeps its own rising-edge state, so a thigh
    // and a foot touch in the same instant both register.
    std::vector<std::string> bodies;
    pnh_.param<std::vector<std::string>>("contact_bodies", bodies, {"thigh", "shank", "foot"});
    if (bodies.empty()) {
      ROS_WARN("[dynasense_bps] contact_bodies is empty; no contact topic will be subscribed");
    }
    for (const char* leg : queryLegNames()) {
      for (const std::string& body : bodies) {
        const std::string topic = substitute(contactTopicFormat_, leg, body);
        const std::size_t index = contactState_.size();
        contactState_.push_back(false);
        contactTopics_.push_back(topic);
        contactSubs_.push_back(nh_.subscribe<gazebo_msgs::ContactsState>(
            topic, 1, boost::bind(&BpsNode::onContacts, this, _1, index)));
        ROS_INFO_STREAM("[dynasense_bps] contact topic: " << topic);
      }
    }
  }

  void subscribeRaySources() {
    if (!mapConfig_.rayEnabled) {
      ROS_WARN("[dynasense_bps] external-ray voxel activation DISABLED "
               "(enable_ray_voxels: false); not subscribing to any depth or lidar cloud");
      return;
    }

    // Six base-mounted depth cameras, each binned onto the trained 24 x 16
    // pinhole pattern (FOV 86.62 x 59.73 deg, max range 1.0 m).
    std::vector<std::string> cameraTopics;
    pnh_.param<std::vector<std::string>>(
        "depth_cloud_topics", cameraTopics,
        {"/depth_camera_front_upper/point_cloud_self_filtered", "/depth_camera_front_lower/point_cloud_self_filtered",
         "/depth_camera_rear_upper/point_cloud_self_filtered", "/depth_camera_rear_lower/point_cloud_self_filtered",
         "/depth_camera_left/point_cloud_self_filtered", "/depth_camera_right/point_cloud_self_filtered"});

    int camCols = 24;
    int camRows = 16;
    double camHFovDeg = 86.62;
    double camVFovDeg = 59.73;
    double rayMaxRange = 1.0;
    bool camOptical = true;
    pnh_.param<int>("camera_pattern_cols", camCols, camCols);
    pnh_.param<int>("camera_pattern_rows", camRows, camRows);
    pnh_.param<double>("camera_hfov_deg", camHFovDeg, camHFovDeg);
    pnh_.param<double>("camera_vfov_deg", camVFovDeg, camVFovDeg);
    pnh_.param<double>("ray_max_range", rayMaxRange, rayMaxRange);
    pnh_.param<bool>("camera_optical_frame", camOptical, camOptical);

    for (const std::string& topic : cameraTopics) {
      RaySource source;
      source.name = topic;
      source.topic = topic;
      source.cols = camCols;
      source.rows = camRows;
      source.hMin = -0.5 * camHFovDeg * kDeg2Rad;
      source.hMax = 0.5 * camHFovDeg * kDeg2Rad;
      source.vMin = -0.5 * camVFovDeg * kDeg2Rad;
      source.vMax = 0.5 * camVFovDeg * kDeg2Rad;
      source.opticalFrame = camOptical;
      source.maxRange = rayMaxRange;
      raySources_.push_back(source);
    }

    // Lidar: 5 channels, vertical FOV [0,15] deg, horizontal FOV [-160,160] deg
    // at 30 deg resolution. The simulated Velodyne is a different beam layout,
    // so this bins whatever arrives onto the trained grid.
    bool useLidar = true;
    std::string lidarTopic = "/lidar/point_cloud";
    int lidarRows = 5;
    double lidarHMinDeg = -160.0;
    double lidarHMaxDeg = 160.0;
    double lidarVMinDeg = 0.0;
    double lidarVMaxDeg = 15.0;
    double lidarResDeg = 30.0;
    pnh_.param<bool>("use_lidar", useLidar, useLidar);
    pnh_.param<std::string>("lidar_topic", lidarTopic, lidarTopic);
    pnh_.param<int>("lidar_channels", lidarRows, lidarRows);
    pnh_.param<double>("lidar_hfov_min_deg", lidarHMinDeg, lidarHMinDeg);
    pnh_.param<double>("lidar_hfov_max_deg", lidarHMaxDeg, lidarHMaxDeg);
    pnh_.param<double>("lidar_vfov_min_deg", lidarVMinDeg, lidarVMinDeg);
    pnh_.param<double>("lidar_vfov_max_deg", lidarVMaxDeg, lidarVMaxDeg);
    pnh_.param<double>("lidar_horizontal_resolution_deg", lidarResDeg, lidarResDeg);

    if (useLidar) {
      RaySource source;
      source.name = lidarTopic;
      source.topic = lidarTopic;
      source.rows = std::max(1, lidarRows);
      source.cols = std::max(1, static_cast<int>(std::lround((lidarHMaxDeg - lidarHMinDeg) / lidarResDeg)) + 1);
      source.hMin = lidarHMinDeg * kDeg2Rad;
      source.hMax = lidarHMaxDeg * kDeg2Rad;
      source.vMin = lidarVMinDeg * kDeg2Rad;
      source.vMax = lidarVMaxDeg * kDeg2Rad;
      source.opticalFrame = false;
      source.maxRange = rayMaxRange;
      raySources_.push_back(source);
    }

    for (std::size_t i = 0; i < raySources_.size(); ++i) {
      raySources_[i].sub = nh_.subscribe<sensor_msgs::PointCloud2>(
          raySources_[i].topic, 1, boost::bind(&BpsNode::onPointCloud, this, _1, i));
      ROS_INFO_STREAM("[dynasense_bps] ray source: " << raySources_[i].topic << " (" << raySources_[i].cols << "x"
                                                     << raySources_[i].rows << ")");
    }
  }

  // -------------------------------------------------------------------------
  // TF helper
  // -------------------------------------------------------------------------

  bool lookup(const std::string& targetFrame, const std::string& sourceFrame, const ros::Time& stamp,
              Eigen::Isometry3d& transform) {
    try {
      const geometry_msgs::TransformStamped tf =
          tfBuffer_.lookupTransform(targetFrame, sourceFrame, stamp, ros::Duration(tfTimeout_));
      transform = tf2::transformToEigen(tf);
      return true;
    } catch (const tf2::TransformException& ex) {
      // Fall back to the latest available transform; in simulation the sensor
      // stamps can lead the TF tree slightly.
      try {
        const geometry_msgs::TransformStamped tf =
            tfBuffer_.lookupTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(tfTimeout_));
        transform = tf2::transformToEigen(tf);
        return true;
      } catch (const tf2::TransformException& ex2) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[dynasense_bps] TF " << targetFrame << " <- " << sourceFrame << ": "
                                                            << ex2.what());
        return false;
      }
    }
  }

  // -------------------------------------------------------------------------
  // Contact input
  // -------------------------------------------------------------------------

  void onContacts(const gazebo_msgs::ContactsState::ConstPtr& msg, std::size_t index) {
    // Strongest obstacle contact patch on this body, ignoring self-collisions.
    double bestForce = 0.0;
    const geometry_msgs::Vector3* bestPosition = nullptr;

    for (const auto& state : msg->states) {
      if (state.contact_positions.empty()) {
        continue;
      }
      if (isSelfCollision(state.collision1_name) && isSelfCollision(state.collision2_name)) {
        continue;
      }

      const auto& f = state.total_wrench.force;
      const double force = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
      if (force <= bestForce) {
        continue;
      }

      // Within the patch, take the point carrying the largest wrench.
      std::size_t bestPoint = 0;
      double bestPointForce = -1.0;
      for (std::size_t i = 0; i < state.contact_positions.size(); ++i) {
        double pointForce = force;
        if (i < state.wrenches.size()) {
          const auto& w = state.wrenches[i].force;
          pointForce = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        }
        if (pointForce > bestPointForce) {
          bestPointForce = pointForce;
          bestPoint = i;
        }
      }

      bestForce = force;
      bestPosition = &state.contact_positions[bestPoint];
    }

    const bool active = bestForce > contactForceThreshold_;
    const bool wasActive = contactState_[index];
    contactState_[index] = active;

    // Both operands are lvalues, so this binds without copying a string on
    // every one of the ~2400 contact messages per second.
    static const std::string kNoLabel;
    const std::string& label = index < contactTopics_.size() ? contactTopics_[index] : kNoLabel;

    if (!active || bestPosition == nullptr) {
      // A patch arrived but carried too little force. Silent until now, which
      // is indistinguishable from no contact at all.
      if (!msg->states.empty() && bestPosition != nullptr) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[dynasense_bps] " << label << ": contact below force threshold ("
                                                         << bestForce << " N <= " << contactForceThreshold_ << " N)");
      }
      return;
    }
    if (contactRisingEdgeOnly_ && wasActive) {
      // Held contact. Only the rising edge registers a voxel, so a touch whose
      // rising edge was rejected (e.g. by the height gate) produces nothing at
      // all for as long as it is held -- release and touch again to retry.
      ROS_INFO_STREAM_THROTTLE(5.0, "[dynasense_bps] " << label
                                                       << ": contact held; rising-edge only, no new voxel");
      return;
    }

    // gazebo_ros_bumper's contact_positions are Gazebo WORLD coordinates even
    // though the header carries <frameName>. See contact_position_frame.
    const Eigen::Vector3d raw(bestPosition->x, bestPosition->y, bestPosition->z);

    Eigen::Isometry3d odomFromSensor = Eigen::Isometry3d::Identity();
    bool haveSensorTf = false;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != odomFrame_) {
      haveSensorTf = lookup(odomFrame_, msg->header.frame_id, msg->header.stamp, odomFromSensor);
    } else {
      haveSensorTf = true;  // already odom
    }

    // Print both interpretations for the first few contacts so the frame can be
    // settled by eye: whichever column lands on the robot is the right one.
    if (contactFrameDebugCount_ > 0) {
      --contactFrameDebugCount_;
      const Eigen::Vector3d asHeader = odomFromSensor * raw;
      ROS_INFO_STREAM("[dynasense_bps] contact frame check ("
                      << msg->header.frame_id << ", using '" << contactPositionFrame_ << "'): raw/as-odom ["
                      << raw.x() << ", " << raw.y() << ", " << raw.z() << "]  as-header ["
                      << asHeader.x() << ", " << asHeader.y() << ", " << asHeader.z() << "]"
                      << (haveSensorTf ? "" : "  (header TF unavailable)"));
    }

    Eigen::Vector3d world;
    if (contactPositionFrame_ == "header") {
      if (!haveSensorTf) {
        return;
      }
      world = odomFromSensor * raw;
    } else {
      world = raw;
    }

    bool inserted = false;
    {
      std::lock_guard<std::mutex> lock(mapMutex_);
      inserted = map_.addContactPoint(world);
    }

    // Every gate in this path used to be silent, which made a dropped contact
    // indistinguishable from a contact that never happened. Say which it was.
    if (inserted) {
      ROS_INFO_STREAM_THROTTLE(1.0, "[dynasense_bps] voxel from " << label << " at odom z=" << world.z()
                                                                  << " (force " << bestForce << " N)");
    } else {
      const double lo = mapConfig_.groundZ + mapConfig_.minHeightAboveGround;
      const double hi = mapConfig_.groundZ + mapConfig_.maxHeightAboveGround;
      ROS_WARN_STREAM_THROTTLE(1.0, "[dynasense_bps] contact from " << label << " REJECTED: odom ["
                                                                    << world.x() << ", " << world.y() << ", "
                                                                    << world.z() << "], accepted height band is ["
                                                                    << lo << ", " << hi << ") and horizontal range is "
                                                                    << mapConfig_.maxHorizontalRange << " m from base");
    }
  }

  bool isSelfCollision(const std::string& collisionName) const {
    return collisionName.rfind(robotModelName_ + "::", 0) == 0;
  }

  // -------------------------------------------------------------------------
  // External-ray input
  // -------------------------------------------------------------------------

  void onPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, std::size_t sourceIndex) {
    const RaySource& source = raySources_[sourceIndex];

    const int cells = source.cols * source.rows;
    if (cells <= 0) {
      return;
    }

    // Bin every point onto the trained ray grid, keeping the nearest hit per
    // cell. One arithmetic pass; no allocation beyond the two grids.
    std::vector<float> nearestRange(static_cast<std::size_t>(cells), std::numeric_limits<float>::max());
    std::vector<Eigen::Vector3f> nearestPoint(static_cast<std::size_t>(cells), Eigen::Vector3f::Zero());

    const double hSpan = source.hMax - source.hMin;
    const double vSpan = source.vMax - source.vMin;
    if (hSpan <= 0.0 || vSpan <= 0.0) {
      return;
    }

    // The iterators throw if the cloud lacks x/y/z, which would otherwise take
    // the node down on a malformed message.
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> itXPtr, itYPtr, itZPtr;
    try {
      itXPtr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x"));
      itYPtr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "y"));
      itZPtr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "z"));
    } catch (const std::runtime_error& ex) {
      ROS_WARN_STREAM_THROTTLE(5.0, "[dynasense_bps] " << source.topic << ": " << ex.what());
      return;
    }
    sensor_msgs::PointCloud2ConstIterator<float>& itX = *itXPtr;
    sensor_msgs::PointCloud2ConstIterator<float>& itY = *itYPtr;
    sensor_msgs::PointCloud2ConstIterator<float>& itZ = *itZPtr;

    bool any = false;
    for (; itX != itX.end(); ++itX, ++itY, ++itZ) {
      const float x = *itX;
      const float y = *itY;
      const float z = *itZ;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const double range = std::sqrt(static_cast<double>(x) * x + static_cast<double>(y) * y +
                                     static_cast<double>(z) * z);
      if (range <= 1e-3 || range > source.maxRange) {
        continue;  // Obstacle hits only, within the trained 1.0 m horizon.
      }

      double h = 0.0;
      double v = 0.0;
      if (source.opticalFrame) {
        // Optical frame: x right, y down, z forward.
        h = std::atan2(static_cast<double>(x), static_cast<double>(z));
        v = std::atan2(static_cast<double>(-y), std::hypot(static_cast<double>(x), static_cast<double>(z)));
      } else {
        // Body frame: x forward, y left, z up.
        h = std::atan2(static_cast<double>(y), static_cast<double>(x));
        v = std::atan2(static_cast<double>(z), std::hypot(static_cast<double>(x), static_cast<double>(y)));
      }
      if (h < source.hMin || h > source.hMax || v < source.vMin || v > source.vMax) {
        continue;
      }

      int col = static_cast<int>((h - source.hMin) / hSpan * source.cols);
      int row = static_cast<int>((v - source.vMin) / vSpan * source.rows);
      col = std::min(std::max(col, 0), source.cols - 1);
      row = std::min(std::max(row, 0), source.rows - 1);

      const std::size_t cell = static_cast<std::size_t>(row) * source.cols + col;
      if (static_cast<float>(range) < nearestRange[cell]) {
        nearestRange[cell] = static_cast<float>(range);
        nearestPoint[cell] = Eigen::Vector3f(x, y, z);
        any = true;
      }
    }

    if (!any) {
      return;
    }

    Eigen::Isometry3d odomFromSensor = Eigen::Isometry3d::Identity();
    if (!msg->header.frame_id.empty() && msg->header.frame_id != odomFrame_) {
      if (!lookup(odomFrame_, msg->header.frame_id, msg->header.stamp, odomFromSensor)) {
        return;
      }
    }

    std::lock_guard<std::mutex> lock(mapMutex_);
    for (std::size_t cell = 0; cell < nearestRange.size(); ++cell) {
      if (nearestRange[cell] == std::numeric_limits<float>::max()) {
        continue;
      }
      map_.addRayPoint(odomFromSensor * nearestPoint[cell].cast<double>());
    }
  }

  // -------------------------------------------------------------------------
  // Query and publish
  // -------------------------------------------------------------------------

  void onTimer(const ros::TimerEvent&) {
    const ros::WallTime tickStart = ros::WallTime::now();
    const ros::Time stamp = ros::Time::now();

    // Query the body poses at ros::Time(0), i.e. the LATEST transform in the
    // buffer, not at `stamp`.
    //
    // This matters a lot. `stamp` is now(), which is always slightly ahead of
    // the newest TF, so tf2 would block for the full tf_timeout on every one of
    // the 13 lookups below before throwing and falling back. At 20 ms x 13 that
    // is 260 ms per tick - the node would run at ~4 Hz instead of 50 Hz, and
    // since the observation is published from this same callback, the POLICY
    // would see 4 Hz exteroception, not just the markers.
    //
    // The latest available transform is exactly what we want here anyway: this
    // is a "where is the robot right now" query, not a lookup against a sensor
    // capture time.
    const ros::Time queryTime(0);

    Eigen::Isometry3d odomFromBase = Eigen::Isometry3d::Identity();
    if (!lookup(odomFrame_, baseFrame_, queryTime, odomFromBase)) {
      return;
    }

    // Body poses for the 34 query points: base plus THIGH/SHANK/FOOT per leg.
    std::array<Eigen::Isometry3d, 4> odomFromThigh;
    std::array<Eigen::Isometry3d, 4> odomFromShank;
    std::array<Eigen::Isometry3d, 4> odomFromFoot;
    for (int leg = 0; leg < 4; ++leg) {
      const std::string legName = queryLegNames()[leg];
      if (!lookup(odomFrame_, legName + "_THIGH", queryTime, odomFromThigh[leg]) ||
          !lookup(odomFrame_, legName + "_SHANK", queryTime, odomFromShank[leg]) ||
          !lookup(odomFrame_, legName + "_FOOT", queryTime, odomFromFoot[leg])) {
        return;
      }
    }

    // The BPS directions are expressed in the yaw-only base frame, unlike the
    // proprioceptive part of the observation which uses the full base frame.
    const Eigen::Matrix3d R_yawBase_odom = yawOnly(odomFromBase.rotation()).transpose();

    dynasense::BpsState msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = baseFrame_;

    std::lock_guard<std::mutex> lock(mapMutex_);
    map_.setBasePosition(odomFromBase.translation());

    const std::vector<QueryPointSpec>& specs = queryPointSpecs();
    for (std::size_t i = 0; i < specs.size(); ++i) {
      const QueryPointSpec& spec = specs[i];

      const Eigen::Isometry3d* bodyPose = &odomFromBase;
      switch (spec.body) {
        case QueryBody::Base:
          bodyPose = &odomFromBase;
          break;
        case QueryBody::Thigh:
          bodyPose = &odomFromThigh[spec.leg];
          break;
        case QueryBody::Shank:
          bodyPose = &odomFromShank[spec.leg];
          break;
        case QueryBody::Foot:
          bodyPose = &odomFromFoot[spec.leg];
          break;
      }

      const Eigen::Vector3d pointOdom = (*bodyPose) * spec.local;
      const BpsQueryResult result = map_.query(pointOdom);
      const Eigen::Vector3d directionBase = R_yawBase_odom * result.directionWorld;

      // Kept for the arrow view: the WORLD-frame unit gradient, without the
      // gain. The actor value below is a different quantity - same direction,
      // but rotated into the yaw-only base frame and scaled by the gain.
      queryUnitDirections_[i] = result.unitDirectionWorld;

      msg.directions[3 * i + 0] = static_cast<float>(directionBase.x());
      msg.directions[3 * i + 1] = static_cast<float>(directionBase.y());
      msg.directions[3 * i + 2] = static_cast<float>(directionBase.z());
      msg.distances[i] = static_cast<float>(result.distance);
      msg.query_points[i].x = pointOdom.x();
      msg.query_points[i].y = pointOdom.y();
      msg.query_points[i].z = pointOdom.z();
    }

    msg.num_contact_voxels = static_cast<uint32_t>(map_.numContactVoxels());
    msg.num_ray_voxels = static_cast<uint32_t>(map_.numRayVoxels());
    statePub_.publish(msg);

    // Standing reminder: a disabled activation path is easy to leave on by
    // accident and silently blinds the policy.
    if (!mapConfig_.contactEnabled || !mapConfig_.rayEnabled) {
      ROS_WARN_STREAM_THROTTLE(10.0, "[dynasense_bps] voxel activation disabled (contacts: "
                                         << (mapConfig_.contactEnabled ? "on" : "OFF")
                                         << ", rays: " << (mapConfig_.rayEnabled ? "on" : "OFF")
                                         << "); BPS observation is degraded by design");
    }

    if (publishMarkers_ && markerPub_.getNumSubscribers() > 0) {
      publishMarkers(msg, stamp);
    }

    // Overrun guard. This callback carries the observation, so if it cannot keep
    // up, the policy silently gets stale exteroception - exactly the failure a
    // blocking TF lookup used to cause here. Cheap enough to leave in.
    const double elapsed = (ros::WallTime::now() - tickStart).toSec();
    if (updateRate_ > 0.0 && elapsed > 1.0 / updateRate_) {
      ROS_WARN_STREAM_THROTTLE(2.0, "[dynasense_bps] tick took " << elapsed * 1e3 << " ms, over the "
                                                                 << 1e3 / updateRate_ << " ms budget; the BPS "
                                                                    "observation is running slower than "
                                                                 << updateRate_ << " Hz");
    }
  }

  // The two debug-only views from the Play task. Both read the same current BPS
  // state that built the observation; neither affects inference.
  void publishMarkers(const dynasense::BpsState& state, const ros::Time& stamp) {
    visualization_msgs::MarkerArray array;
    if (publishVoxelCubes_) {
      appendVoxelCubes(array, stamp);
    }
    if (publishQueryArrows_) {
      appendQueryArrows(array, state, stamp);
    }
    if (!array.markers.empty()) {
      markerPub_.publish(array);
    }
  }

  //! View 1: activated-voxel cubes.
  //!
  //! Every valid voxel in the UNION of the contact and external-ray LRU buffers,
  //! drawn at its world-frame voxel centre as a voxel-sized cube. "Activated"
  //! means retained as a known obstacle voxel - it is NOT the distance-dependent
  //! gain the actor observation uses. Contact- and ray-activated voxels are
  //! deliberately not distinguished by colour. Hidden entirely when the map is
  //! empty.
  void appendVoxelCubes(visualization_msgs::MarkerArray& array, const ros::Time& stamp) {
    std::vector<Eigen::Vector3d> centres;
    map_.bufferedVoxelCentresUnion(centres);

    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = odomFrame_;
    marker.ns = "bps_activated_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;

    if (centres.empty()) {
      // No valid voxel: hide the cube view rather than publishing an empty list,
      // so a previously drawn set does not linger.
      marker.action = visualization_msgs::Marker::DELETE;
      array.markers.push_back(marker);
      return;
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = map_.config().voxelSize;
    marker.color.r = 0.9f;
    marker.color.g = 0.2f;
    marker.color.b = 0.1f;
    marker.color.a = 0.6f;

    const std::size_t stride = displayStride(centres.size());
    marker.points.reserve((centres.size() + stride - 1) / stride);
    for (std::size_t i = 0; i < centres.size(); i += stride) {
      geometry_msgs::Point p;
      p.x = centres[i].x();
      p.y = centres[i].y();
      p.z = centres[i].z();
      marker.points.push_back(p);
    }
    array.markers.push_back(marker);
  }

  //! View 2: query-point arrows.
  //!
  //! One FIXED-LENGTH arrow at each of the 34 moving body-local query points,
  //! pointing along the world-frame unit gradient away from the nearest retained
  //! cube. Length encodes nothing; the inferno colour encodes clamped distance
  //! abs(d)/max_query_distance, so d = 0 is the dark end and d >= 0.5 m the
  //! bright/yellow end.
  //!
  //! This is NOT the actor value. Observation [49,151) uses the same direction
  //! rotated into the yaw-only base frame and multiplied by the gain. On an empty
  //! map every arrow is bright and its direction is a meaningless artifact
  //! (see BpsMap::query), while all 102 actor values are exactly zero.
  void appendQueryArrows(visualization_msgs::MarkerArray& array, const dynasense::BpsState& state,
                         const ros::Time& stamp) {
    const std::size_t count = state.query_points.size();
    const std::size_t stride = displayStride(count);

    for (std::size_t i = 0; i < count; ++i) {
      visualization_msgs::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = odomFrame_;
      marker.ns = "bps_query_arrows";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::Marker::ARROW;

      if (i % stride != 0) {
        // Dropped by display subsampling: delete rather than leave it stale.
        marker.action = visualization_msgs::Marker::DELETE;
        array.markers.push_back(marker);
        continue;
      }

      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      // ARROW with two points: scale is (shaft diameter, head diameter, head length).
      marker.scale.x = 0.1 * arrowLength_;
      marker.scale.y = 0.2 * arrowLength_;
      marker.scale.z = 0.3 * arrowLength_;

      const Eigen::Vector3d& dir = queryUnitDirections_[i];
      geometry_msgs::Point tail = state.query_points[i];
      geometry_msgs::Point tip;
      tip.x = tail.x + arrowLength_ * dir.x();
      tip.y = tail.y + arrowLength_ * dir.y();
      tip.z = tail.z + arrowLength_ * dir.z();
      marker.points.push_back(tail);
      marker.points.push_back(tip);

      const double t = std::min(std::max(std::abs(static_cast<double>(state.distances[i])) /
                                             map_.config().maxQueryDistance,
                                         0.0),
                                1.0);
      const Eigen::Vector3f rgb = inferno(t);
      marker.color.r = rgb.x();
      marker.color.g = rgb.y();
      marker.color.b = rgb.z();
      marker.color.a = 1.0f;

      array.markers.push_back(marker);
    }
  }

  //! Display-only stride so a huge map does not flood RViz. Never changes what
  //! the actor sees.
  std::size_t displayStride(std::size_t count) const {
    if (markerMaxCount_ == 0 || count <= markerMaxCount_) {
      return 1;
    }
    return (count + markerMaxCount_ - 1) / markerMaxCount_;
  }

  bool onReset(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    std::lock_guard<std::mutex> lock(mapMutex_);
    map_.clear();
    std::fill(contactState_.begin(), contactState_.end(), false);
    ROS_INFO("[dynasense_bps] buffers cleared");
    return true;
  }

  // -------------------------------------------------------------------------

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  ros::Publisher statePub_;
  ros::Publisher markerPub_;
  ros::ServiceServer resetServer_;
  ros::Timer timer_;

  std::vector<ros::Subscriber> contactSubs_;
  std::vector<bool> contactState_;
  std::vector<RaySource> raySources_;

  mutable std::mutex mapMutex_;
  BpsMap map_;
  BpsMapConfig mapConfig_{};

  //! World-frame unit gradients from the last query, for the arrow view only.
  //! Sized in the constructor.
  std::vector<Eigen::Vector3d> queryUnitDirections_;

  std::string odomFrame_;
  std::string baseFrame_;
  std::string statePublishTopic_;
  std::string markerTopic_;
  std::string resetService_;
  std::string robotModelName_;
  std::string contactTopicFormat_;
  bool publishMarkers_{true};
  bool publishVoxelCubes_{true};
  bool publishQueryArrows_{true};
  double arrowLength_{0.3};
  std::size_t markerMaxCount_{20000};
  bool contactRisingEdgeOnly_{true};
  std::vector<std::string> contactTopics_;
  std::string contactPositionFrame_{"odom"};
  int contactFrameDebugCount_{10};
  double updateRate_{50.0};
  double tfTimeout_{0.02};
  double contactForceThreshold_{1.0};
};

}  // namespace dynasense

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynasense_bps_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  dynasense::BpsNode node(nh, pnh);

  // Sensor callbacks and the query timer run concurrently; the map is mutexed.
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
