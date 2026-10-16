// dynasense_wavemap_bps_bridge
//
// Turns the live wavemap occupancy map into the voxel set the BPS node uses as
// its external-ray voxels (ray_voxel_source: map_voxels in config/bps.yaml).
//
// Inputs
//   map_topic (/wavemap/map)  wavemap_msgs/Map, incremental; merged into a local
//                             copy (blocks the server dropped are erased too).
//                             <map_topic>_request_full is called once the link
//                             is up, so the copy starts complete.
//   TF map_frame <- base_frame (latest)
//   floor_z_topic (optional, std_msgs/Float64, latched): the floor height in the
//                             map frame, e.g. from scripts/start_bps_hardware.sh.
//                             Until it arrives nothing is published.
// Output
//   output_topic (/dynasense/wavemap_voxels)  sensor_msgs/PointCloud2 at `rate`:
//     centres of the occupied min_cell_width cells within `radius` of the base in
//     xy and with centre z in [z_min, z_max) (ABSOLUTE, map frame; by default
//     floor_z + z_min_above_floor .. floor_z + z_max_above_floor), nearest
//     first, at most max_voxels; fields x y z intensity (= log-odds); frame =
//     map frame; stamp = the base TF stamp. Empty clouds are published too, so
//     the BPS buffer empties when nothing is near.
//
// Statistics every stats_period and at shutdown. floor_z (when finite) is
// checked against the lowest foot frame: an absolute z band only works when it
// matches the floor height in odom, which is not 0 on ANYmal and drifts.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "wavemap_voxel_extraction.hpp"

namespace dynasense {
namespace {

constexpr const char* kLogPrefix = "[dynasense_wavemap_bps_bridge] ";

struct BridgeStats {
  std::size_t extractions{0};
  double sumExtractMs{0.0};
  double maxExtractMs{0.0};
  std::size_t sumCells{0};
  std::size_t sumKept{0};
  std::size_t maxKept{0};
  std::size_t truncatedTicks{0};
  std::size_t tfFailures{0};
  std::size_t mapMessages{0};
  double sumMergeMs{0.0};
  double maxMergeMs{0.0};
  double sumMessageKb{0.0};
  double maxMapAgeS{0.0};
};

double median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  return values[mid];
}

}  // namespace

class WavemapBpsBridge {
 public:
  WavemapBpsBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : tfListener_(tfBuffer_), startWall_(ros::WallTime::now()) {
    std::string mapTopic = "/wavemap/map";
    std::string requestService;
    std::string outputTopic = "/dynasense/wavemap_voxels";
    int queueSize = 50;
    double rate = 20.0;
    double statsPeriod = 10.0;
    double radius = params_.radius;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::string floorTopic;
    double threshold = params_.occupancyThreshold;
    int maxVoxels = static_cast<int>(params_.maxVoxels);
    pnh.param("map_topic", mapTopic, mapTopic);
    pnh.param("map_request_service", requestService, requestService);
    pnh.param("map_queue_size", queueSize, queueSize);
    pnh.param("output_topic", outputTopic, outputTopic);
    pnh.param("base_frame", baseFrame_, std::string("base"));
    pnh.param("rate", rate, rate);
    pnh.param("radius", radius, radius);
    // Band: absolute z_min/z_max when both are given, otherwise relative to the floor.
    pnh.param("z_min", absZMin_, nan);
    pnh.param("z_max", absZMax_, nan);
    pnh.param("z_min_above_floor", zMinAboveFloor_, 0.05);
    pnh.param("z_max_above_floor", zMaxAboveFloor_, 1.8);
    pnh.param("floor_z_topic", floorTopic, floorTopic);
    pnh.param("occupancy_threshold", threshold, threshold);
    pnh.param("max_voxels", maxVoxels, maxVoxels);
    pnh.param("map_timeout", mapTimeout_, 1.0);
    pnh.param("stats_period", statsPeriod, statsPeriod);
    pnh.param("floor_z", floorZ_, nan);
    pnh.param("floor_check_tolerance", floorTolerance_, 0.05);
    pnh.param<std::vector<std::string>>("foot_frames", footFrames_, {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"});
    if (!(radius > 0.0)) throw std::runtime_error("radius must be > 0");
    if (!(rate > 0.0)) throw std::runtime_error("rate must be > 0");
    absoluteBand_ = std::isfinite(absZMin_) && std::isfinite(absZMax_);
    if (absoluteBand_ && !(absZMax_ > absZMin_)) throw std::runtime_error("z_max must be > z_min");
    if (!absoluteBand_ && !(zMaxAboveFloor_ > zMinAboveFloor_)) {
      throw std::runtime_error("z_max_above_floor must be > z_min_above_floor");
    }
    if (!absoluteBand_ && !std::isfinite(floorZ_) && floorTopic.empty()) {
      throw std::runtime_error("set z_min and z_max, or floor_z, or floor_z_topic");
    }
    params_.radius = radius;
    params_.occupancyThreshold = static_cast<float>(threshold);
    params_.maxVoxels = static_cast<std::size_t>(std::max(0, maxVoxels));
    if (requestService.empty()) {
      requestService = mapTopic + "_request_full";
    }
    requestService_ = requestService;

    pub_ = nh.advertise<sensor_msgs::PointCloud2>(outputTopic, 1);
    if (!floorTopic.empty()) {
      floorSub_ = nh.subscribe(floorTopic, 1, &WavemapBpsBridge::onFloor, this);
    }
    mapSub_ = nh.subscribe(mapTopic, static_cast<uint32_t>(std::max(1, queueSize)), &WavemapBpsBridge::onMap, this,
                           ros::TransportHints().tcpNoDelay());
    requestClient_ = nh.serviceClient<std_srvs::Empty>(requestService_);
    timer_ = nh.createTimer(ros::Duration(1.0 / rate), &WavemapBpsBridge::onTimer, this);
    if (statsPeriod > 0.0) {
      statsTimer_ = nh.createWallTimer(ros::WallDuration(statsPeriod), &WavemapBpsBridge::onStatsTimer, this);
    }
    requestThread_ = std::thread(&WavemapBpsBridge::requestLoop, this);

    std::ostringstream band;
    if (absoluteBand_) {
      band << "centre z in [" << absZMin_ << ", " << absZMax_ << ") absolute";
    } else {
      band << "centre z in [floor + " << zMinAboveFloor_ << ", floor + " << zMaxAboveFloor_ << "), floor "
           << (std::isfinite(floorZ_) ? std::to_string(floorZ_) : std::string("from ") + floorTopic);
    }
    ROS_INFO_STREAM(kLogPrefix << mapTopic << " -> " << outputTopic << " at " << rate << " Hz: occupied cells within "
                               << radius << " m (xy) of " << baseFrame_ << ", " << band.str() << ", log-odds > "
                               << threshold << ", at most "
                               << (params_.maxVoxels ? std::to_string(params_.maxVoxels) : std::string("all"))
                               << " voxels nearest first");
  }

  ~WavemapBpsBridge() {
    stop_ = true;
    if (requestThread_.joinable()) {
      requestThread_.join();
    }
    logStats(true);
  }

 private:
  void onMap(const wavemap_msgs::Map::ConstPtr& msg) {
    const ros::WallTime start = ros::WallTime::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!mapFrame_.empty() && msg->header.frame_id != mapFrame_) {
      ROS_WARN_STREAM(kLogPrefix << "map frame changed from '" << mapFrame_ << "' to '" << msg->header.frame_id
                                 << "'; dropping the local copy");
      map_.reset();
    }
    if (!wavemap::convert::rosMsgToMap(*msg, map_) || !map_) {
      ROS_WARN_THROTTLE(5.0, "%scould not convert a map message", kLogPrefix);
      return;
    }
    if (!std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(map_)) {
      ROS_ERROR_THROTTLE(5.0, "%sthe map is not a hashed_wavelet_octree; set map/type in the wavemap config",
                         kLogPrefix);
    }
    mapFrame_ = msg->header.frame_id;
    lastMapReceipt_ = ros::Time::now();
    const double mergeMs = (ros::WallTime::now() - start).toSec() * 1e3;
    ++stats_.mapMessages;
    stats_.sumMergeMs += mergeMs;
    stats_.maxMergeMs = std::max(stats_.maxMergeMs, mergeMs);
    stats_.sumMessageKb += static_cast<double>(ros::serialization::serializationLength(*msg)) / 1024.0;
  }

  void onFloor(const std_msgs::Float64::ConstPtr& msg) {
    if (!std::isfinite(msg->data)) {
      ROS_WARN_STREAM(kLogPrefix << "ignoring a non-finite floor height");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(floorZ_) || std::abs(msg->data - floorZ_) > 1e-6) {
      ROS_INFO_STREAM(kLogPrefix << "floor height " << msg->data << " m"
                                 << (absoluteBand_ ? " (floor check only; the band is absolute)" : ""));
    }
    floorZ_ = msg->data;
  }

  void onTimer(const ros::TimerEvent&) {
    std::string frame;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_) {
        ROS_INFO_THROTTLE(10.0, "%swaiting for the first map message", kLogPrefix);
        return;
      }
      if (absoluteBand_) {
        params_.zMin = absZMin_;
        params_.zMax = absZMax_;
      } else if (std::isfinite(floorZ_)) {
        params_.zMin = floorZ_ + zMinAboveFloor_;
        params_.zMax = floorZ_ + zMaxAboveFloor_;
      } else {
        ROS_INFO_THROTTLE(10.0, "%swaiting for the floor height (publishes nothing until it is known)", kLogPrefix);
        return;
      }
      frame = mapFrame_;
    }

    geometry_msgs::TransformStamped tf;
    try {
      tf = tfBuffer_.lookupTransform(frame, baseFrame_, ros::Time(0));
    } catch (const tf2::TransformException& ex) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.tfFailures;
      ROS_WARN_STREAM_THROTTLE(5.0, kLogPrefix << "no TF " << frame << " <- " << baseFrame_ << ": " << ex.what());
      return;
    }
    const Eigen::Vector3d base(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);

    ExtractionStats extraction;
    double mapAge = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto* octree = dynamic_cast<const wavemap::HashedWaveletOctree*>(map_.get());
      if (!octree) {
        return;
      }
      const ros::WallTime start = ros::WallTime::now();
      extractOccupiedVoxels(*octree, base, params_, voxels_, &extraction);
      const double ms = (ros::WallTime::now() - start).toSec() * 1e3;
      numBlocks_ = octree->getHashMap().size();
      mapAge = (ros::Time::now() - lastMapReceipt_).toSec();
      ++stats_.extractions;
      stats_.sumExtractMs += ms;
      stats_.maxExtractMs = std::max(stats_.maxExtractMs, ms);
      stats_.sumCells += extraction.cellsVisited;
      stats_.sumKept += voxels_.size();
      stats_.maxKept = std::max(stats_.maxKept, voxels_.size());
      stats_.truncatedTicks += extraction.truncated > 0 ? 1u : 0u;
      stats_.maxMapAgeS = std::max(stats_.maxMapAgeS, mapAge);
      lastMapAgeS_ = mapAge;
      publish(frame, tf.header.stamp);
    }

    if (mapAge > mapTimeout_) {
      ROS_WARN_STREAM_THROTTLE(2.0, kLogPrefix << "no map message for " << mapAge
                                               << " s; publishing voxels from a stale map");
    }

    if ((++tick_ % 10) == 0) {
      sampleLowestFoot(frame);
    }
  }

  void publish(const std::string& frame, const ros::Time& stamp) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(voxels_.size());
    if (!voxels_.empty()) {
      sensor_msgs::PointCloud2Iterator<float> itX(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> itY(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> itZ(cloud, "z");
      sensor_msgs::PointCloud2Iterator<float> itI(cloud, "intensity");
      for (const ExtractedVoxel& v : voxels_) {
        *itX = v.centre.x();
        *itY = v.centre.y();
        *itZ = v.centre.z();
        *itI = v.logOdds;
        ++itX;
        ++itY;
        ++itZ;
        ++itI;
      }
    }
    cloud.is_dense = true;
    pub_.publish(cloud);
  }

  void sampleLowestFoot(const std::string& frame) {
    double lowest = std::numeric_limits<double>::infinity();
    for (const std::string& foot : footFrames_) {
      try {
        lowest = std::min(lowest, tfBuffer_.lookupTransform(frame, foot, ros::Time(0)).transform.translation.z);
      } catch (const tf2::TransformException&) {
        return;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    footZ_.push_back(lowest);
    if (footZ_.size() > 100) {
      footZ_.pop_front();
    }
  }

  void requestLoop() {
    bool needed = true;
    while (!stop_ && ros::ok()) {
      if (mapSub_.getNumPublishers() == 0) {
        needed = true;  // no link (yet, or the server restarted): resync when it comes back
      } else if (needed && requestClient_.waitForExistence(ros::Duration(1.0))) {
        std_srvs::Empty srv;
        if (requestClient_.call(srv)) {
          needed = false;
          ROS_INFO_STREAM(kLogPrefix << "requested the full map from " << requestService_);
        }
      }
      ros::WallDuration(0.5).sleep();
    }
  }

  void onStatsTimer(const ros::WallTimerEvent&) { logStats(false); }

  void logStats(bool final) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double elapsed = std::max(1e-3, (ros::WallTime::now() - startWall_).toSec());
    const BridgeStats& s = stats_;
    std::ostringstream line;
    line << std::fixed << std::setprecision(2);
    if (s.extractions == 0) {
      line << "no extraction yet (map messages: " << s.mapMessages << ", TF failures: " << s.tfFailures << ")";
    } else {
      const double n = static_cast<double>(s.extractions);
      line << s.extractions << " extractions (" << n / elapsed << " Hz), " << s.sumExtractMs / n << " ms mean, "
           << s.maxExtractMs << " ms max, " << static_cast<double>(s.sumCells) / n << " cells/extraction, voxels "
           << static_cast<double>(s.sumKept) / n << " mean / " << s.maxKept << " max, " << s.truncatedTicks
           << " capped | map: " << s.mapMessages << " msgs (" << static_cast<double>(s.mapMessages) / elapsed
           << " Hz), "
           << (s.mapMessages ? s.sumMergeMs / static_cast<double>(s.mapMessages) : 0.0) << " ms merge mean, "
           << s.maxMergeMs << " ms max, "
           << (s.mapMessages ? s.sumMessageKb / static_cast<double>(s.mapMessages) : 0.0) << " KB/msg, "
           << numBlocks_ << " blocks, age " << lastMapAgeS_ << " s now / " << s.maxMapAgeS << " s max | "
           << s.tfFailures << " TF failures";
    }
    std::vector<double> feet(footZ_.begin(), footZ_.end());
    const double footMedian = median(feet);
    if (std::isfinite(footMedian)) {
      line << " | lowest foot z median " << std::setprecision(3) << footMedian << " (floor_z " << floorZ_ << ")";
    }
    if (!absoluteBand_ || std::isfinite(floorZ_)) {
      line << " | band [" << std::setprecision(3) << params_.zMin << ", " << params_.zMax << ")";
    }
    ROS_INFO_STREAM(kLogPrefix << (final ? "final statistics: " : "statistics: ") << line.str());
    if (std::isfinite(floorZ_) && std::isfinite(footMedian) && std::abs(footMedian - floorZ_) > floorTolerance_) {
      ROS_WARN_STREAM(kLogPrefix << "floor_z " << floorZ_ << " differs from the lowest foot (" << footMedian
                                 << ") by more than " << floorTolerance_
                                 << " m: the absolute z band is misplaced. Relaunch with floor_z:=" << std::setprecision(2)
                                 << footMedian << " and set the BPS node's ground_z to the same value.");
    }
  }

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  ros::Publisher pub_;
  ros::Subscriber mapSub_;
  ros::Subscriber floorSub_;
  ros::ServiceClient requestClient_;
  ros::Timer timer_;
  ros::WallTimer statsTimer_;
  std::thread requestThread_;
  std::atomic<bool> stop_{false};

  VoxelExtractionParams params_;
  std::string baseFrame_{"base"};
  std::string requestService_;
  double mapTimeout_{1.0};
  double absZMin_{std::numeric_limits<double>::quiet_NaN()};
  double absZMax_{std::numeric_limits<double>::quiet_NaN()};
  double zMinAboveFloor_{0.05};
  double zMaxAboveFloor_{1.8};
  bool absoluteBand_{false};
  double floorTolerance_{0.05};
  std::vector<std::string> footFrames_;

  std::mutex mutex_;  // guards everything below
  double floorZ_{std::numeric_limits<double>::quiet_NaN()};
  wavemap::MapBase::Ptr map_;
  std::string mapFrame_;
  ros::Time lastMapReceipt_;
  std::vector<ExtractedVoxel> voxels_;
  BridgeStats stats_;
  std::size_t numBlocks_{0};
  double lastMapAgeS_{0.0};
  std::deque<double> footZ_;
  std::size_t tick_{0};
  const ros::WallTime startWall_;
};

}  // namespace dynasense

namespace {
std::atomic<bool> g_stopRequested{false};
void onStopSignal(int) { g_stopRequested = true; }
}  // namespace

int main(int argc, char** argv) {
  // Own SIGINT/SIGTERM handling: the final statistics must be logged before
  // ros::shutdown() tears rosconsole down.
  ros::init(argc, argv, "dynasense_wavemap_bps_bridge", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, onStopSignal);
  std::signal(SIGTERM, onStopSignal);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::unique_ptr<dynasense::WavemapBpsBridge> node;
  try {
    node.reset(new dynasense::WavemapBpsBridge(nh, pnh));
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("[dynasense_wavemap_bps_bridge] " << e.what());
    return 1;
  }

  ros::AsyncSpinner spinner(2);  // map merges and extractions can overlap; the mutex serialises map access
  spinner.start();
  while (ros::ok() && !g_stopRequested) {
    ros::WallDuration(0.1).sleep();
  }
  spinner.stop();
  node.reset();
  std::fflush(stdout);
  ros::shutdown();
  return 0;
}
