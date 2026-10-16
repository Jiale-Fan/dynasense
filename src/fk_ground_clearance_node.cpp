// dynasense_fk_ground_clearance_node
//
// Hardware stand-in for the Gazebo ground-clearance plugin: the same 17
// "gravity rays" (dynasense/GroundClearance, order as in the message), but the
// clearance of each ray origin is its height above a FLAT floor at floor_z,
//
//   clearance_i = clamp(z_i - floor_z, 0, max_range),
//
// with z_i the height of the ray origin in `frame` from TF, i.e. forward
// kinematics of the measured joint angles on top of the state estimator's
// odom -> base. Obstacles below the robot (steps, boxes) are NOT seen; the
// training rays would report the distance to their top instead.
//
// Ray origins: base, then per leg (LF, LH, RF, RH) the HIP, THIGH, SHANK and
// FOOT frame origins, exactly the points the plugin casts from.
//
// Params (private): floor_z [m, in frame] (required unless floor_z_topic),
// floor_z_topic (std_msgs/Float64, overrides), frame (odom), base_frame (base),
// rate (50 Hz), max_range (2.0 m), topic (/dynasense/ground_clearance),
// marker_topic (/dynasense/ground_clearance_markers, red spheres at the floor
// points below each origin, only when subscribed).

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "dynasense/GroundClearance.h"

namespace {

constexpr const char* kLogPrefix = "[dynasense_fk_ground_clearance] ";

std::vector<std::string> rayFrames(const std::string& baseFrame) {
  std::vector<std::string> frames{baseFrame};
  for (const char* leg : {"LF", "LH", "RF", "RH"}) {
    for (const char* link : {"_HIP", "_THIGH", "_SHANK", "_FOOT"}) {
      frames.push_back(std::string(leg) + link);
    }
  }
  return frames;
}

class FkGroundClearance {
 public:
  FkGroundClearance(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tfListener_(tfBuffer_) {
    std::string topic = "/dynasense/ground_clearance";
    std::string markerTopic = "/dynasense/ground_clearance_markers";
    std::string floorTopic;
    double rate = 50.0;
    pnh.param("floor_z", floorZ_, std::numeric_limits<double>::quiet_NaN());
    pnh.param("floor_z_topic", floorTopic, floorTopic);
    pnh.param("frame", frame_, std::string("odom"));
    pnh.param("base_frame", baseFrame_, std::string("base"));
    pnh.param("rate", rate, rate);
    pnh.param("max_range", maxRange_, 2.0);
    pnh.param("topic", topic, topic);
    pnh.param("marker_topic", markerTopic, markerTopic);
    if (!std::isfinite(floorZ_) && floorTopic.empty()) {
      throw std::runtime_error("set floor_z or floor_z_topic");
    }
    if (!(rate > 0.0) || !(maxRange_ > 0.0)) {
      throw std::runtime_error("rate and max_range must be > 0");
    }
    frames_ = rayFrames(baseFrame_);
    pub_ = nh.advertise<dynasense::GroundClearance>(topic, 1);
    markerPub_ = nh.advertise<visualization_msgs::Marker>(markerTopic, 1);
    if (!floorTopic.empty()) {
      floorSub_ = nh.subscribe(floorTopic, 1, &FkGroundClearance::onFloor, this);
    }
    timer_ = nh.createTimer(ros::Duration(1.0 / rate), &FkGroundClearance::onTimer, this);
    ROS_INFO_STREAM(kLogPrefix << "17 rays (" << baseFrame_ << ", HIP/THIGH/SHANK/FOOT per leg) above a flat floor at "
                               << (std::isfinite(floorZ_) ? std::to_string(floorZ_) : "<" + floorTopic + ">")
                               << " m in " << frame_ << " -> " << topic << " at " << rate << " Hz, max range "
                               << maxRange_ << " m");
  }

 private:
  void onFloor(const std_msgs::Float64::ConstPtr& msg) {
    if (!std::isfinite(msg->data)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(floorZ_) || std::abs(msg->data - floorZ_) > 1e-6) {
      ROS_INFO_STREAM(kLogPrefix << "floor height " << msg->data << " m");
    }
    floorZ_ = msg->data;
  }

  void onTimer(const ros::TimerEvent&) {
    double floor = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      floor = floorZ_;
    }
    if (!std::isfinite(floor)) {
      ROS_INFO_THROTTLE(10.0, "%swaiting for the floor height", kLogPrefix);
      return;
    }

    dynasense::GroundClearance msg;
    msg.header.frame_id = frame_;
    msg.max_range = static_cast<float>(maxRange_);
    std::array<geometry_msgs::Point, 17> hits{};
    std::size_t numHits = 0;
    ros::Time stamp;
    for (std::size_t i = 0; i < frames_.size() && i < msg.clearance.size(); ++i) {
      geometry_msgs::TransformStamped tf;
      try {
        // Latest transform: "where is the robot now", as in the BPS node.
        tf = tfBuffer_.lookupTransform(frame_, frames_[i], ros::Time(0));
      } catch (const tf2::TransformException& ex) {
        ROS_WARN_STREAM_THROTTLE(5.0, kLogPrefix << "no TF " << frame_ << " <- " << frames_[i] << ": " << ex.what()
                                                 << "; not publishing");
        return;
      }
      if (i == 0) stamp = tf.header.stamp;
      const auto& t = tf.transform.translation;
      const double clearance = std::min(std::max(t.z - floor, 0.0), maxRange_);
      msg.clearance[i] = static_cast<float>(clearance);
      if (clearance < maxRange_) {
        hits[numHits].x = t.x;
        hits[numHits].y = t.y;
        hits[numHits].z = floor;  // the floor point straight below the ray origin
        ++numHits;
      }
    }
    msg.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    pub_.publish(msg);

    if (markerPub_.getNumSubscribers() > 0) {
      visualization_msgs::Marker marker;
      marker.header = msg.header;
      marker.ns = "ground_clearance_hits";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      if (numHits == 0) {
        marker.action = visualization_msgs::Marker::DELETE;
      } else {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
        marker.color.r = 1.0f;
        marker.color.a = 1.0f;
        marker.points.assign(hits.begin(), hits.begin() + static_cast<std::ptrdiff_t>(numHits));
      }
      markerPub_.publish(marker);
    }
  }

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  ros::Publisher pub_;
  ros::Publisher markerPub_;
  ros::Subscriber floorSub_;
  ros::Timer timer_;
  std::vector<std::string> frames_;
  std::string frame_{"odom"};
  std::string baseFrame_{"base"};
  double maxRange_{2.0};
  std::mutex mutex_;
  double floorZ_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynasense_fk_ground_clearance");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try {
    FkGroundClearance node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM(kLogPrefix << e.what());
    return 1;
  }
  return 0;
}
