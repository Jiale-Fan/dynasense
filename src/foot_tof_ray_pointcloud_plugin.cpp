// Custom Gazebo ray-sensor plugin that publishes a 4x4 ToF grid
// as a PointCloud2 message. Each ray becomes one point in the cloud.

#include <cmath>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace dynasense
{
class FootToFRayPointCloudPlugin : public gazebo::SensorPlugin
{
public:
  FootToFRayPointCloudPlugin() = default;
  ~FootToFRayPointCloudPlugin() override
  {
    if (ros_node_)
    {
      ros_node_->shutdown();
    }
  }

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    ray_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
    if (!ray_sensor_)
    {
      gzerr << "[FootToFRayPointCloudPlugin] Requires a RaySensor parent.\n";
      return;
    }

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "dynasense_foot_tof_pointcloud_plugin",
                ros::init_options::NoSigintHandler);
    }

    std::string robot_ns = "/";
    if (sdf->HasElement("robotNamespace"))
    {
      robot_ns = sdf->Get<std::string>("robotNamespace");
    }

    topic_name_ = "/foot_tof/pointcloud";
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
    pub_ = ros_node_->advertise<sensor_msgs::PointCloud2>(topic_name_, 1);

    ray_sensor_->SetActive(true);
    update_connection_ = ray_sensor_->ConnectUpdated(
        std::bind(&FootToFRayPointCloudPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    std::vector<double> ranges;
    ray_sensor_->Ranges(ranges);
    if (ranges.empty())
    {
      return;
    }

    const int h_count = ray_sensor_->RayCount();
    const int v_count = ray_sensor_->VerticalRayCount();
    const double h_min = ray_sensor_->AngleMin().Radian();
    const double h_max = ray_sensor_->AngleMax().Radian();
    const double v_min = ray_sensor_->VerticalAngleMin().Radian();
    const double v_max = ray_sensor_->VerticalAngleMax().Radian();

    const double h_inc = (h_count > 1) ? (h_max - h_min) / (h_count - 1) : 0.0;
    const double v_inc = (v_count > 1) ? (v_max - v_min) / (v_count - 1) : 0.0;

    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_name_;
    msg.height = static_cast<uint32_t>(v_count);
    msg.width = static_cast<uint32_t>(h_count);
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(static_cast<size_t>(h_count * v_count));

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    // Flattened order: horizontal index varies fastest, vertical slowest.
    for (int v = 0; v < v_count; ++v)
    {
      const double v_angle = v_min + v * v_inc;
      const double cv = std::cos(v_angle);
      const double sv = std::sin(v_angle);
      for (int h = 0; h < h_count; ++h, ++iter_x, ++iter_y, ++iter_z)
      {
        const int idx = v * h_count + h;
        const double r = ranges[idx];
        if (!std::isfinite(r) || r <= 0.0)
        {
          *iter_x = std::numeric_limits<float>::quiet_NaN();
          *iter_y = std::numeric_limits<float>::quiet_NaN();
          *iter_z = std::numeric_limits<float>::quiet_NaN();
          continue;
        }
        const double h_angle = h_min + h * h_inc;
        const double ch = std::cos(h_angle);
        const double sh = std::sin(h_angle);
        *iter_x = static_cast<float>(r * cv * ch);
        *iter_y = static_cast<float>(r * cv * sh);
        *iter_z = static_cast<float>(r * sv);
      }
    }

    pub_.publish(msg);
  }

  gazebo::sensors::RaySensorPtr ray_sensor_;
  gazebo::event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher pub_;

  std::string topic_name_;
  std::string frame_name_;
};

GZ_REGISTER_SENSOR_PLUGIN(FootToFRayPointCloudPlugin)
}  // namespace dynasense
