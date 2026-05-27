// Custom Gazebo ray-sensor plugin that publishes a flattened 4x4 ToF grid
// as a single LaserScan message. Designed for tiny multi-zone ToF sensors.

#include <cmath>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace dynasense
{
class FootToFRayPlugin : public gazebo::SensorPlugin
{
public:
  FootToFRayPlugin() = default;
  ~FootToFRayPlugin() override
  {
    if (ros_node_)
    {
      ros_node_->shutdown();
    }
  }

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    // Require a ray sensor (not gpu_ray).
    ray_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
    if (!ray_sensor_)
    {
      gzerr << "[FootToFRayPlugin] Requires a RaySensor parent.\n";
      return;
    }

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "dynasense_foot_tof_ray_plugin",
                ros::init_options::NoSigintHandler);
    }

    // Optional SDF params.
    std::string robot_ns = "/";
    if (sdf->HasElement("robotNamespace"))
    {
      robot_ns = sdf->Get<std::string>("robotNamespace");
    }

    topic_name_ = "/foot_tof/snapshot_gazebo_source";
    if (sdf->HasElement("topicName"))
    {
      topic_name_ = sdf->Get<std::string>("topicName");
    }

    frame_name_ = ray_sensor_->ParentName();
    if (sdf->HasElement("frameName"))
    {
      frame_name_ = sdf->Get<std::string>("frameName");
    }

    ros_node_.reset(new ros::NodeHandle(robot_ns));
    pub_ = ros_node_->advertise<sensor_msgs::LaserScan>(topic_name_, 1);

    // Activate the sensor and connect update callback.
    ray_sensor_->SetActive(true);
    update_connection_ = ray_sensor_->ConnectUpdated(
        std::bind(&FootToFRayPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    // Fetch all ranges (flattened). For a 4x4 grid, this should be size 16.
    std::vector<double> ranges;
    ray_sensor_->Ranges(ranges);
    if (ranges.empty())
    {
      return;
    }

    const unsigned int h_count = ray_sensor_->RayCount();
    const unsigned int v_count = ray_sensor_->VerticalRayCount();
    const double angle_min = ray_sensor_->AngleMin().Radian();
    const double angle_max = ray_sensor_->AngleMax().Radian();
    const double range_min = ray_sensor_->RangeMin();
    const double range_max = ray_sensor_->RangeMax();

    sensor_msgs::LaserScan msg;
    msg.header.seq = seq_++;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_name_;

    // LaserScan is 1-D; we flatten the 2-D grid in Gazebo order.
    // The order is the same as RaySensor::Ranges() output.
    msg.angle_min = angle_min;
    msg.angle_max = angle_max;
    msg.angle_increment = (h_count > 1) ? (angle_max - angle_min) / (h_count - 1) : 0.0;
    msg.time_increment = 0.0;
    msg.scan_time = (ray_sensor_->UpdateRate() > 0.0) ? (1.0 / ray_sensor_->UpdateRate()) : 0.0;
    msg.range_min = range_min;
    msg.range_max = range_max;

    msg.ranges.reserve(ranges.size());
    for (double r : ranges)
    {
      if (!std::isfinite(r))
      {
        ROS_WARN_THROTTLE(1.0, "[FootToFRayPlugin] Non-finite range detected, clamping to max.");
        r = range_max;
      }
      else if (r < 0.0)
      {
        ROS_WARN_THROTTLE(1.0, "[FootToFRayPlugin] Negative range detected (%.6f), clamping to 0.", r);
        r = 0.0;
      }
      msg.ranges.push_back(static_cast<float>(r));
    }

    // We do not publish per-ray intensities for ToF (leave empty).
    pub_.publish(msg);
  }

  gazebo::sensors::RaySensorPtr ray_sensor_;
  gazebo::event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher pub_;

  std::string topic_name_;
  std::string frame_name_;
  uint32_t seq_{0};
};

GZ_REGISTER_SENSOR_PLUGIN(FootToFRayPlugin)
}  // namespace dynasense
