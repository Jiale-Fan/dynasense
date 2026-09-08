// Gazebo model plugin publishing the downward ground clearance of a fixed set
// of link-attached points.
//
// This produces observation block [151,168) of the ContactBPS policy: one ray
// per link origin, cast along normalised world gravity against the terrain,
// clipped to [0, maxRange], with misses reporting maxRange.
//
// Rays are declared in SDF so the ray set stays with the robot description:
//
//   <plugin name="dynasense_ground_clearance"
//           filename="libdynasense_ground_clearance_plugin.so">
//     <topicName>/dynasense/ground_clearance</topicName>
//     <updateRate>50</updateRate>
//     <maxRange>2.0</maxRange>
//     <robotModel>anymal</robotModel>       <!-- hits on this model are ignored -->
//     <groundModel></groundModel>           <!-- optional: accept hits on this model only -->
//     <ray><link>base</link><offset>0 0 0</offset></ray>
//     ...
//   </plugin>
//
// A ray whose <link> does not exist in the SDF model is reported as a miss and
// warned about once. This matters because URDF fixed-joint lumping removes
// links such as LF_FOOT; those rays are declared against the surviving link
// (LF_SHANK) plus a constant offset.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

#include <dynasense/GroundClearance.h>

namespace dynasense
{
class GroundClearancePlugin : public gazebo::ModelPlugin
{
public:
  GroundClearancePlugin() = default;
  ~GroundClearancePlugin() override
  {
    update_connection_.reset();
    if (ros_node_)
    {
      ros_node_->shutdown();
    }
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    if (!world_)
    {
      gzerr << "[GroundClearancePlugin] No world.\n";
      return;
    }

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "dynasense_ground_clearance_plugin",
                ros::init_options::NoSigintHandler);
    }

    std::string robot_ns = "/";
    if (sdf->HasElement("robotNamespace"))
    {
      robot_ns = sdf->Get<std::string>("robotNamespace");
    }
    topic_name_ = sdf->HasElement("topicName") ? sdf->Get<std::string>("topicName")
                                               : std::string("/dynasense/ground_clearance");
    frame_name_ = sdf->HasElement("frameName") ? sdf->Get<std::string>("frameName") : std::string("odom");
    max_range_ = sdf->HasElement("maxRange") ? sdf->Get<double>("maxRange") : 2.0;
    update_rate_ = sdf->HasElement("updateRate") ? sdf->Get<double>("updateRate") : 50.0;
    robot_model_ = sdf->HasElement("robotModel") ? sdf->Get<std::string>("robotModel") : model_->GetName();
    ground_model_ = sdf->HasElement("groundModel") ? sdf->Get<std::string>("groundModel") : std::string();

    // Debug visualisation of the ray hits.
    publish_markers_ = sdf->HasElement("publishMarkers") ? sdf->Get<bool>("publishMarkers") : true;
    marker_topic_ = sdf->HasElement("markerTopic") ? sdf->Get<std::string>("markerTopic")
                                                   : std::string("/dynasense/ground_clearance_markers");
    marker_diameter_ = sdf->HasElement("markerDiameter") ? sdf->Get<double>("markerDiameter") : 0.02;

    // A ray origin can sit INSIDE the robot's own collision geometry (this is
    // the case for the base, SHANK and FOOT rays). RayShape only reports the
    // closest intersection, so such a hit has to be stepped over rather than
    // treated as a miss. See CastRay().
    skip_step_ = sdf->HasElement("skipStep") ? sdf->Get<double>("skipStep") : 0.01;
    max_skips_ = sdf->HasElement("maxSkips") ? sdf->Get<int>("maxSkips") : 16;

    if (!ParseRays(sdf))
    {
      gzerr << "[GroundClearancePlugin] No <ray> elements declared; plugin disabled.\n";
      return;
    }

    // One reusable ray shape; ODE resolves the intersection on demand.
    gazebo::physics::PhysicsEnginePtr physics = world_->Physics();
    if (!physics)
    {
      gzerr << "[GroundClearancePlugin] No physics engine.\n";
      return;
    }
    ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        physics->CreateShape("ray", gazebo::physics::CollisionPtr()));
    if (!ray_)
    {
      gzerr << "[GroundClearancePlugin] Could not create a ray shape.\n";
      return;
    }

    ros_node_.reset(new ros::NodeHandle(robot_ns));
    pub_ = ros_node_->advertise<dynasense::GroundClearance>(topic_name_, 1);
    if (publish_markers_)
    {
      marker_pub_ = ros_node_->advertise<visualization_msgs::Marker>(marker_topic_, 1);
    }

    update_period_ = (update_rate_ > 0.0) ? 1.0 / update_rate_ : 0.0;
    last_update_ = world_->SimTime();

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GroundClearancePlugin::OnUpdate, this));

    gzmsg << "[GroundClearancePlugin] " << rays_.size() << " rays -> " << topic_name_ << " at " << update_rate_
          << " Hz\n";
  }

private:
  struct RaySpec
  {
    std::string link_name;
    ignition::math::Vector3d offset{0, 0, 0};
    gazebo::physics::LinkPtr link;  ///< resolved lazily; null means "always a miss"
    bool warned{false};
  };

  bool ParseRays(const sdf::ElementPtr &sdf)
  {
    if (!sdf->HasElement("ray"))
    {
      return false;
    }
    for (sdf::ElementPtr e = sdf->GetElement("ray"); e; e = e->GetNextElement("ray"))
    {
      RaySpec spec;
      spec.link_name = e->HasElement("link") ? e->Get<std::string>("link") : std::string();
      if (e->HasElement("offset"))
      {
        spec.offset = e->Get<ignition::math::Vector3d>("offset");
      }
      rays_.push_back(spec);
    }
    return !rays_.empty();
  }

  void OnUpdate()
  {
    const gazebo::common::Time now = world_->SimTime();
    if (update_period_ > 0.0 && (now - last_update_).Double() < update_period_)
    {
      return;
    }
    last_update_ = now;

    dynasense::GroundClearance msg;
    msg.header.seq = seq_++;
    msg.header.stamp.sec = now.sec;
    msg.header.stamp.nsec = now.nsec;
    msg.header.frame_id = frame_name_;
    msg.max_range = static_cast<float>(max_range_);

    const std::size_t count = std::min(rays_.size(), msg.clearance.size());
    for (std::size_t i = 0; i < msg.clearance.size(); ++i)
    {
      msg.clearance[i] = static_cast<float>(max_range_);
    }

    hit_points_.clear();
    for (std::size_t i = 0; i < count; ++i)
    {
      bool hit = false;
      ignition::math::Vector3d hit_point;
      msg.clearance[i] = static_cast<float>(CastRay(rays_[i], hit, hit_point));
      if (hit)
      {
        hit_points_.push_back(hit_point);
      }
    }

    pub_.publish(msg);

    if (publish_markers_ && marker_pub_.getNumSubscribers() > 0)
    {
      PublishHitMarkers(msg.header);
    }
  }

  /// Debug view: one small red sphere at each ray's ground intersection.
  /// Rays that miss contribute no sphere. Hidden entirely when nothing is hit.
  void PublishHitMarkers(const std_msgs::Header &header)
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "ground_clearance_hits";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;

    if (hit_points_.empty())
    {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub_.publish(marker);
      return;
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = marker_diameter_;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.points.reserve(hit_points_.size());
    for (const ignition::math::Vector3d &p : hit_points_)
    {
      geometry_msgs::Point point;
      point.x = p.X();
      point.y = p.Y();
      point.z = p.Z();
      marker.points.push_back(point);
    }

    marker_pub_.publish(marker);
  }

  /// Cast one ray straight down and return the clearance. On a hit, `hit_point`
  /// receives the world-frame intersection.
  ///
  /// Visual meshes are irrelevant here: Gazebo ray queries collide against
  /// COLLISION geometry only. The robot's own collision geometry, however, is
  /// very much in the way - the base ray origin sits inside the base collision
  /// box, each SHANK origin sits inside its knee cylinder, and each FOOT origin
  /// sits on the foot ball. RayShape reports only the CLOSEST intersection, so
  /// those self-hits have to be stepped over. Treating one as a miss would
  /// report max_range for 9 of the 17 rays.
  double CastRay(RaySpec &spec, bool &hit, ignition::math::Vector3d &hit_point)
  {
    hit = false;

    if (!spec.link)
    {
      spec.link = model_->GetLink(spec.link_name);
      if (!spec.link)
      {
        if (!spec.warned)
        {
          spec.warned = true;
          gzwarn << "[GroundClearancePlugin] Link '" << spec.link_name
                 << "' not found in the SDF model (fixed-joint lumping?); reporting misses.\n";
        }
        return max_range_;
      }
    }

    const ignition::math::Pose3d link_pose = spec.link->WorldPose();
    const ignition::math::Vector3d origin = link_pose.CoordPositionAdd(spec.offset);
    // Gravity is (0,0,-9.81) in this world; cast along its normalised direction.
    const ignition::math::Vector3d end(origin.X(), origin.Y(), origin.Z() - max_range_);

    double travelled = 0.0;
    for (int attempt = 0; attempt <= max_skips_; ++attempt)
    {
      if (travelled >= max_range_)
      {
        return max_range_;
      }

      const ignition::math::Vector3d start(origin.X(), origin.Y(), origin.Z() - travelled);

      double distance = max_range_;
      std::string entity;
      ray_->SetPoints(start, end);
      ray_->GetIntersection(distance, entity);

      if (entity.empty() || !std::isfinite(distance))
      {
        return max_range_;  // Nothing left along the ray.
      }

      const double absolute = travelled + distance;
      if (absolute >= max_range_)
      {
        return max_range_;
      }

      const bool is_self = IsRobot(entity);
      const bool wrong_model = !ground_model_.empty() && entity.rfind(ground_model_, 0) != 0;
      if (is_self || wrong_model)
      {
        // Step past this surface and keep looking for the ground. skip_step_
        // guarantees forward progress even when the query keeps reporting a
        // zero-distance hit from inside a geom.
        travelled = absolute + skip_step_;
        continue;
      }

      hit = true;
      hit_point.Set(origin.X(), origin.Y(), origin.Z() - absolute);
      return std::min(std::max(absolute, 0.0), max_range_);
    }

    if (!spec.warned)
    {
      spec.warned = true;
      gzwarn << "[GroundClearancePlugin] Ray from '" << spec.link_name
             << "' exhausted maxSkips without clearing the robot's own collision geometry; "
                "reporting a miss. Raise <maxSkips> or <skipStep>.\n";
    }
    return max_range_;
  }

  bool IsRobot(const std::string &entity) const
  {
    return entity.rfind(robot_model_ + "::", 0) == 0 || entity == robot_model_;
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::RayShapePtr ray_;
  gazebo::event::ConnectionPtr update_connection_;

  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher pub_;
  ros::Publisher marker_pub_;

  std::vector<RaySpec> rays_;
  std::vector<ignition::math::Vector3d> hit_points_;
  std::string topic_name_;
  std::string frame_name_;
  std::string robot_model_;
  std::string ground_model_;
  std::string marker_topic_;
  bool publish_markers_{true};
  double marker_diameter_{0.02};
  double skip_step_{0.01};
  int max_skips_{16};
  double max_range_{2.0};
  double update_rate_{50.0};
  double update_period_{0.02};
  gazebo::common::Time last_update_;
  uint32_t seq_{0};
};

GZ_REGISTER_MODEL_PLUGIN(GroundClearancePlugin)
}  // namespace dynasense
