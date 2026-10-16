# dynasense

Virtual Gazebo sensors and the contact-history BPS map for the Dynasense
locomotion policies on ANYmal-D.

## Contents

```
dynasense/
├── include/dynasense/BpsMap.hpp        # voxel LRU buffers + BPS query (no ROS, no Gazebo)
├── src/bps_map.cpp
├── src/bps_node.cpp                    # dynasense_bps_node
├── src/ground_clearance_plugin.cpp     # Gazebo model plugin, 17 downward rays
├── src/depth_image_to_pointcloud_node.cpp  # dynasense_depth_image_to_pointcloud_node (bag replay)
├── src/self_filter_node.cpp            # dynasense_self_filter_node: robot body + range filter for clouds
├── src/wavemap_bps_bridge_node.cpp     # dynasense_wavemap_bps_bridge: wavemap cells near the base -> BPS voxels
├── src/fk_ground_clearance_node.cpp    # dynasense_fk_ground_clearance_node: hardware gravity rays above a flat floor
├── src/wavemap_voxel_extraction.{hpp,cpp}  # occupied-cell extraction (C++17, wavemap; optional target)
├── msg/{BpsState,GroundClearance}.msg
├── config/bps.yaml
├── config/depth_cameras.yaml           # intrinsics + topics for the depth converter
├── config/self_filter.yaml             # self filter: margin, range band, inputs
├── config/wavemap_replay.yaml          # wavemap server template ($(arg ...) substituted by the launch file)
├── config/wavemap_live.yaml            # same for the robot: four face cameras + lidar, fast map publishing
├── config/depth_cameras_live.yaml      # converter overlay for the robot (four cameras, driver camera_info)
├── config/depth_cameras_sim.yaml       # converter overlay for Gazebo (simulated camera topics)
├── config/bps_live.yaml                # BPS overlay for the robot (odom, ray voxels from the bridge)
├── launch/sim_contact_bps.launch       # the whole ContactBPS stack
├── launch/sim_bps_sensors_only.launch  # same, minus the learned controller
├── launch/depth_image_to_pointcloud.launch  # depth converter only
├── launch/replay_depth_clouds.launch   # bag + TF + depth converter + RViz
├── launch/replay_wavemap.launch        # bag + depth converter + self filter + wavemap + RViz
├── launch/live_wavemap.launch          # robot: depth converter + self filter + wavemap + BPS bridge
├── launch/bps_hardware.launch          # robot: floor topic + BPS node + FK ground clearance
├── scripts/tf_static_aggregator.py     # re-latches the union of /tf_static during bag replay
├── scripts/check_wavemap_voxels.py     # checks the bridge output; z histogram to find the floor
├── scripts/estimate_floor_z.py         # floor height from the four foot soles (TF / FK)
├── scripts/start_bps_hardware.sh      # estimate the floor, then bps_hardware.launch
└── foot_tof/                           # unrelated: foot ToF ray plugins + preprocessor
```

## ContactBPS pipeline

Produces the exteroceptive two thirds of the observation for
`rl_controller::RlController`, per
`DS_LOCORESET_DEPTHHEIGHTSCAN_CONTACTBPS_DEPLOYMENT_HANDOVER.md`.

```
gzserver
  libgazebo_ros_bumper.so x12 (declared in the ANYmal-D leg xacros)
      -> /contacts/{LF,LH,RF,RH}/{thigh,shank,foot}   gazebo_msgs/ContactsState @200 Hz
  libdynasense_ground_clearance_plugin.so
      -> /dynasense/ground_clearance             dynasense/GroundClearance @50 Hz
      -> /dynasense/ground_clearance_markers     visualization_msgs/Marker (debug)
  6 depth cameras @50 Hz + Velodyne @50 Hz
      -> /depth_camera_*/point_cloud_self_filtered, /lidar/point_cloud

dynasense_bps_node @50 Hz
  contacts + clouds + TF -> voxelise -> 200-entry contact LRU / 500-entry ray LRU
                         -> 34 query points -> exterior-box SDF
      -> /dynasense/bps_state    dynasense/BpsState
      -> /dynasense/bps_markers  visualization_msgs/MarkerArray (debug)
      service /dynasense/bps/reset (std_srvs/Empty)
```

The 102-value BPS reading is recomputed and published at 50 Hz, matching the
policy step; the markers are published from the same callback, so they run at the
same rate. The map *contents* update asynchronously in each sensor callback —
contacts up to 200 Hz (rising edges only), depth clouds and lidar at 50 Hz — so a
voxel is retained the moment it is seen rather than on the next tick.

> **TF query time matters here.** The 13 body-pose lookups in `onTimer` use
> `ros::Time(0)` (latest available), *not* `ros::Time::now()`. Asking for `now()`
> is always slightly ahead of the newest TF, so tf2 blocks for the full
> `tf_timeout` on every lookup before falling back — 13 × 20 ms = 260 ms per
> tick, i.e. **~4 Hz instead of 50 Hz**. Because the observation is published
> from the same callback, that starves the policy, not just the display. The node
> now warns whenever a tick overruns its budget.

If the cube view still seems to trail the robot, that is the LRU *history*
working as intended rather than a rate problem: a voxel stays buffered until it
is evicted, so the cube cloud keeps everything seen over the last 200/500
insertions. `max_horizontal_range` gates *insertion* only — it does not remove
voxels once the robot has walked away from them.

### Coordinate frames

`map_frame: world` keeps the voxel history in Gazebo's static world frame.
The stock ROS Noetic bumper publishes world-coordinate contact points even
when its header names a robot link. `contact_position_frame: world` handles
that explicitly; points are transformed if a different map frame is configured.
Do not select `header` for the stock bumper or assume estimator `odom` equals
Gazebo `world`.

Depth/lidar hits and robot query poses are transformed into the same map frame.
Voxel and query-arrow markers, plus `BpsState.header` and `query_points`, name
that frame. The policy's 102 `directions` values remain in the yaw-only base
frame. Ground-clearance hit markers also use `world`; scalar clearances are
unchanged. RViz can use `odom` as its fixed frame because TF places the world
mesh and world markers together.

For older configuration files, `odom_frame` is a fallback name for `map_frame`;
the old `contact_position_frame: odom` value means world-coordinate bumper
points and is accepted with a warning. `ground_z` is measured in the chosen
map frame. Restart the BPS node after changing frames so old voxels are cleared.

The frame regression test needs ROS but no simulator. After rebuilding, run
`rostest dynasense frame_alignment.test`. It feeds matching synthetic contact
and camera hits through a translated/rotated world-to-odom TF and checks their
voxel alignment, query/marker frames, and yaw-only policy directions.

### Run

```bash
catkin build dynasense rl_controller dynasense_worlds
source devel/setup.bash
roslaunch dynasense sim_contact_bps.launch
# terrain sunk below the origin? tell the BPS map where the ground is:
roslaunch dynasense sim_contact_bps.launch ground_z:=-1.0
```

#### Sensor bring-up without the policy

To debug the BPS sensor on its own, use the sibling launch file. It brings up
the identical sensor set but never registers `rl_controller::RlController`, so
no `.onnx` is loaded and no policy writes joint targets:

```bash
roslaunch dynasense sim_bps_sensors_only.launch
# separate full-window RViz instead of the embedded rqt widget:
roslaunch dynasense sim_bps_sensors_only.launch standalone_rviz:=true
```

The robot is driven by the stock ANYbotics/RSL controllers, so joystick
motion_1 (`walk_learning_perceptive`) still walks it around while you watch the
BPS cubes and clearance spheres. The difference is one feature-toggle value:
`nvidia_rl_controller: sensors_only` (in `anymal_d_rsl/config/rsl_bps_sensors.yaml`)
instead of `enabled`. Both values load the same
`load_anymal_description.custom_parameters` block, which is what keeps the
sensor stream you debug here identical to the one the policy will see.

### Switching the activation paths off

The two voxel-activation paths are independently switchable in
[`config/bps.yaml`](config/bps.yaml):

```yaml
enable_contact_voxels: false    # physical THIGH/SHANK contacts
enable_ray_voxels: false        # external depth-camera and lidar hits
```

A disabled path is not subscribed at all and its LRU buffer is given capacity 0,
so nothing is retained and nothing reaches a query — an exact off switch, not a
downstream filter.

Both ship enabled. Setting both to `false` makes the map empty and the policy
blind to obstacles; use this only to debug behaviour without BPS input.

The node says so loudly: a startup `WARN` when both are off, plus a `WARN` every
10 s while either is off. The node still publishes `/dynasense/bps_state` at the
normal rate (all zeros), so the controller's readiness check still passes and the
policy runs — it just has no obstacle information.

Visual signature of an empty map: **no cubes at all**, and all 34 arrows bright
yellow pointing in arbitrary directions. `num_contact_voxels` and
`num_ray_voxels` in `BpsState` stay at 0.

### The two halves of the map

- **Physical contacts** — the strongest non-self contact patch on each THIGH and
  SHANK, force `> 1 N`, **rising edge only**, voxelised into a 200-entry LRU.
- **External rays** (`ray_voxel_source: sensors`, the default; with
  `map_voxels` they come from a wavemap map instead, see "Live wavemap → BPS")
  — the six self-filtered depth clouds and the lidar cloud
  (body-filtered by a `dynasense_self_filter_node` instance that
  `sim_contact_bps.launch` runs for the lidar only, `lidar_self_filter:=false`
  feeds the raw `/lidar/point_cloud`),
  binned onto the trained ray grids (`24x16` at FOV `86.62 x 59.73` deg per
  camera; 5 channels over vFOV `[-15,15]` deg at 30 deg horizontal resolution for
  the lidar), nearest hit per cell, range `<= 1.0 m`, into a 500-entry LRU.

Both buffers deduplicate and evict least-recently-touched voxels. Querying a
point returns the exterior-box SDF gradient of the nearest buffered voxel across
**both** buffers, attenuated by distance, in the **yaw-only** base frame.

### Debug visualisation

`/dynasense/bps_markers` carries the two debug-only views from the Play task.
Both read the same current BPS state that built the observation; neither affects
inference. Markers are only assembled when something is subscribed.

**`bps_activated_voxels`** — every valid voxel in the **union** of the two LRU
buffers, drawn at its world-frame voxel centre as a 5 cm cube, red-orange
(`0.9, 0.2, 0.1`) at `0.6` opacity. Contact- and ray-activated voxels are
deliberately **not** distinguished by colour. "Activated" means retained as a
known obstacle voxel — it is *not* the distance-dependent `gain` the actor sees.
With no valid voxel the view is hidden (`DELETE`, so a previous set does not
linger).

**`bps_query_arrows`** — one **fixed-length** 0.3 m arrow at each of the 34
moving body-local query points, pointing along the world-frame unit gradient
away from the nearest retained cube. Inferno colour encodes clamped
`abs(d)/0.5`: `d=0` is the dark end, `d >= 0.5 m` the bright/yellow end. **Arrow
length encodes nothing** — not distance, not actor-input magnitude.

The arrow is therefore *not* the actor value:

| | direction | frame | magnitude |
|---|---|---|---|
| arrow view | unit gradient | **world** | fixed 0.3 m; colour = distance |
| observation `[49,151)` | same gradient | **yaw-only base** | `× gain` |

On an empty map every arrow is bright and its **direction is meaningless**. This
reproduces a training-time artifact: the Play visualisation does not mask invalid
buffer slots, so it takes the gradient against the zero-initialised slot 0 and
only falls back to `+z` when that is degenerate. The actor is unaffected — the
gain is zero, so all 102 BPS values are exactly zero. See `BpsMap::query`.

Above `marker_max_count` (20,000) cubes or arrows, markers are stride-subsampled
**for display only**; the actor input is never subsampled.

### `ground_z`

The handover treats ground and obstacles as separate terrain prims.
`dynasense_worlds` ships a single terrain mesh, so the split is a flat constant:
voxels whose centre is less than `min_height_above_ground` (0.05 m) above
`ground_z` are treated as ground and dropped. **Set `ground_z` to the height of
the walkable surface in the loaded world** — it is the one parameter you must get
right per terrain.

The grid itself is a global 5 cm lattice in `map_frame` (`world` by default) rather than the training
sensor's `160x160x40` per-environment grid, which has no deployment equivalent.
The training extent is reproduced by the height band and by
`max_horizontal_range` (4 m from the base). Both LRU buffers are hard-capped, so
memory is bounded regardless.

### Contact sensors

Declared in `anymal_d_rsl/assets/urdf/leg_eflesh/{thigh,shank,foot}/*_macro.urdf.xacro`
with the stock `libgazebo_ros_bumper.so`. Two details matter:

- URDF fixed-joint lumping folds `<LEG>_thigh_fixed` into `<LEG>_THIGH` and
  mangles collision names, so the sensors bind explicit collision names with
  their fixed-joint lump prefixes and index suffixes. The foot sensor binds
  only the shin tube, excluding the foot ball.
- `<frameName>` names a surviving link (`<LEG>_THIGH`, `<LEG>_SHANK`), but the
  stock bumper still publishes world-coordinate patch positions. The
  older `/contacts/<LEG>/knee_cylinder` sensor names `<LEG>_shank_fixed`, which
  does not survive; it is left as-is because `knee_eflesh_ros` consumes it.

Verify the generated SDF after any xacro change:

```bash
rosrun xacro xacro $(rospack find anymal_d_rsl)/assets/urdf/anymal_d_rsl.urdf.xacro \
  enable_ground_clearance_rays:=true > /tmp/r.urdf
gz sdf -p /tmp/r.urdf | grep -n '<collision name\|<sensor\|<plugin'
```

### Ground-clearance plugin

17 `RayShape` queries per update, one per link origin, along world `-z`, clipped
to `[0, maxRange]`, misses reporting `maxRange`. Rays are declared in SDF from
`anymal_d_rsl/assets/urdf/ground_clearance.gazebo.xacro`, which is included by
the top-level xacro when `enable_ground_clearance_rays:=true` (the
`nvidia_rl_controller` feature toggle sets this).

Only links that survive fixed-joint lumping can be addressed by name.
`<LEG>_FOOT` does not, so the foot rays use `<LEG>_SHANK` plus the constant
SHANK→FOOT offset; keep those constants in step with
`leg_eflesh/foot/foot_3_6_3_macro.urdf.xacro` if the foot version changes.

**Ray queries see collision geometry, never visual meshes** — a ray origin inside
a visual mesh is not blocked by it. The robot's own *collision* geometry is a
different matter, and it is genuinely in the way: the `base` origin sits inside
the base collision box (`0.894 × 0.1615 × 0.256` at `z=0.016`, so the box spans
`z ∈ [-0.112, 0.144]`), each `<LEG>_SHANK` origin sits inside its knee cylinder
(`r=0.071`, centred exactly on that origin), and each `<LEG>_FOOT` origin sits on
the foot ball sphere (`r=0.033`, centre `0.033` away). That is **9 of the 17
rays**.

`RayShape` reports only the *closest* intersection, so those self-hits are
**stepped over**: on a hit belonging to `<robotModel>` (or, when `<groundModel>`
is set, to any other model) the ray is re-cast from `skipStep` (1 cm) past that
surface, up to `maxSkips` (16) times. Rejecting the ray instead would report
`maxRange` for all nine — feet reading 2.0 m while standing on the floor.

`<groundModel>` restricts hits to one terrain model and is where a future
ground/obstacle mesh split plugs in.

### Debug view

`/dynasense/ground_clearance_markers` (`visualization_msgs/Marker`, RViz display
**Ground Clearance Hits**) draws a 2 cm red sphere at each ray's ground
intersection. Rays that miss contribute no sphere; the view is hidden entirely
when nothing is hit. Only assembled when something is subscribed.

This is the cheapest check on the ray set, and the one part of the observation
that has no other visual verification. On flat ground expect 17 spheres in a
tidy pattern under the robot — four under the feet, four under the shanks, and so
on. A sphere in the wrong place means a bad `<offset>`; a *missing* sphere means
that ray is reporting `maxRange`, which for a foot on the floor is wrong and
points at the self-hit skipping (raise `<maxSkips>` or `<skipStep>`).

## Verification order

1. `gz sdf -p` shows the eight contact sensors and the clearance plugin.
2. `rostopic hz /contacts/LF/thigh` ~200 Hz; drive into a box and confirm
   `contact_positions` is non-empty.
3. `rostopic echo /dynasense/ground_clearance` on flat terrain: the four
   `*_FOOT` entries ~0, `base` ~0.5 m; walking off an edge pushes entries toward
   `max_range`.
4. **RViz → Contact BPS Map**: confirm the 34 arrow tails sit on the robot's
   links with correct left/right mirroring *before* trusting any policy output.
   The query points come from `anymal_d.usd` body frames and per-leg mirroring is
   the most likely place for a silent frame error. On open flat ground expect all
   34 arrows bright yellow with arbitrary directions (see above) — that is
   correct, not a bug. Walking up to an obstacle should turn the nearby arrows
   dark and point them away from the red-orange cubes.
5. `rostopic echo /dynasense/bps_state` — `num_ray_voxels` should climb as the
   robot approaches an obstacle and stay at 0 on open flat ground.

## Depth image → point cloud (bag replay)

`dynasense_depth_image_to_pointcloud_node` back-projects the six RealSense
depth images into organized `sensor_msgs/PointCloud2` clouds. It exists because
the real-robot bags (e.g. `~/Documents/rosbags/23sept/jiale_dynasense_files`)
carry `/depth_camera_*/depth/image_rect_raw` but **no `camera_info` and no
point clouds**, so neither the stock `depth_image_proc` pipeline nor the BPS
node can consume them.

```
/depth_camera_<side>/depth/image_rect_raw    sensor_msgs/Image  16UC1 mm (also mono16, 32FC1 m), 424x240 @60 Hz
  -> /depth_camera_<side>/depth/points        sensor_msgs/PointCloud2: organized 240x424, xyz float32,
                                              NaN = no return / outside [depth_min, depth_max], is_dense false,
                                              header (stamp, frame_id) copied from the image
  -> /depth_camera_<side>/depth/camera_info   synthesized sensor_msgs/CameraInfo, same header
```

Files: `src/depth_image_to_pointcloud_node.cpp`, `config/depth_cameras.yaml`,
`launch/depth_image_to_pointcloud.launch` (converter only),
`launch/replay_depth_clouds.launch` (bag + TF + converter + RViz),
`scripts/tf_static_aggregator.py`, `test/depth_image_to_pointcloud.test`.

### Intrinsics

No calibration is stored offline, so `config/depth_cameras.yaml` uses the
Gazebo D435 model from
`anymal_d/urdf/base/sensors/depth_camera/realsense_d435_{face,side}_3_6_0.urdf.xacro`:
`horizontal_fov 1.50098` rad at 424x240, `Cx = W/2 - 0.5`, `Cy = H/2 - 0.5`,
`focalLength 0` (Gazebo computes it):

    fx = fy = W / (2 tan(hfov/2)) = 424 / (2 tan(0.75049)) = 227.343 px,  cx = 211.5,  cy = 119.5,  no distortion

These are *simulation* values; a real D435 at this resolution is roughly
fx ≈ 223, fy ≈ 217, so expect a few percent lateral error at the image border.
To use the robot's calibration, run
`rostopic echo -n1 /depth_camera_<side>/depth/camera_info` on the robot and
paste `P[0] P[5] P[2] P[6]` as `fx fy cx cy` into that camera's entry. When a
`camera_info` topic *is* being published (live robot, or a bag that has it),
the node adopts it automatically and stops publishing its synthesized one.

The upside-down mounting of the face cameras is in TF
(`*_camera -> *_camera_parent` is a 180° roll), so the images are **not**
flipped before back-projection.

### Run

```bash
catkin build dynasense
source devel/setup.bash
# bag replay with RViz (group "Dynasense Depth Clouds" in anymal_d_rsl/config/ui/config_depth_lidar.rviz):
roslaunch dynasense replay_depth_clouds.launch bag:=/path/to/2026-09-23-18-30-12.bag
roslaunch dynasense replay_depth_clouds.launch bag:=... start:=20 rate:=0.5 loop:=true
roslaunch dynasense replay_depth_clouds.launch bag:=... max_rate:=0 decimation:=2   # every frame, 212x120 points
# converter only (live robot, or next to anymal_d_rsl/bin/replay.sh):
roslaunch dynasense depth_image_to_pointcloud.launch
roslaunch dynasense depth_image_to_pointcloud.launch cloud_topic_template:={ns}/depth/color/points  # stock raw name
```

`replay_depth_clouds.launch` sets `use_sim_time`, plays the bag with `--clock`
and `--delay 5`, loads `anymal_description` (the RSL xacro with
`bagfile_is_played:=true`, for the RobotModel display only; joint TFs and the
calibrated camera chain come from the bag) and caps the converter at 15 Hz per
camera (6 × 60 Hz × 1.6 MB would be ~590 MB/s into RViz).

> **`/tf_static` and rosbag play.** The bag holds nine separately latched
> `/tf_static` messages (anymal_tf_publisher, six RealSense nodelet managers,
> compslam). `rosbag play` merges them into one publisher, and a latched
> publisher only re-sends its *last* message, so anything that subscribes after
> the first milliseconds of playback (a slow RViz start, `tf_echo`, a restarted
> node) never sees the depth-camera chains and the clouds cannot be placed in
> `odom`. `scripts/tf_static_aggregator.py` reads the statics from the bag
> file, merges everything it sees on `/tf_static`, and re-latches the union.
> The replay launch file starts it; with your own `rosbag play`, run
> `rosrun dynasense tf_static_aggregator.py _bags:=/path/to.bag`.

### Parameters (`config/depth_cameras.yaml`)

| parameter | default | meaning |
|---|---|---|
| `depth_min`, `depth_max` | 0.1, 3.0 m | Gazebo near clip / cloud cutoff; pixels outside become NaN |
| `decimation` | 1 | pixel stride; the cloud is ⌈W/d⌉×⌈H/d⌉, sampled at u = d·i with the original intrinsics |
| `max_rate` | 0 | per-camera Hz cap by header stamp (0 = every frame) |
| `publish_camera_info` | true | synthesized CameraInfo next to each cloud (enables `rviz/DepthCloud`, `depth_image_proc`) |
| `use_live_camera_info` | true | adopt a `camera_info` published by someone else |
| `process_without_subscribers` | false | convert even when nobody listens |
| `image_queue_size`, `tcp_nodelay`, `spinner_threads` | 2, true, 4 | |
| `depth_unit_m` | 0.001 | metres per 16-bit unit (REP 118) |
| `*_topic_template` | `{ns}/depth/image_rect_raw`, `{ns}/depth/points`, `{ns}/depth/camera_info` | `{ns}` = the camera's `namespace` |
| `default_intrinsics` | Gazebo values above | used by entries without their own |
| `cameras[]` | six entries | `name` (required), `namespace`, optional `image_topic`, `cloud_topic`, `camera_info_topic`, `frame_id`, `fx fy cx cy width height` |

### Verify

```bash
rostest dynasense depth_image_to_pointcloud.test            # synthetic images: layout, padding, NaNs, decimation, live camera_info
rostopic hz /depth_camera_front_lower/depth/points -w 30     # 15 Hz (replay default) / 60 Hz (max_rate:=0)
rostopic echo -n1 --noarr /depth_camera_front_lower/depth/points   # 240x424, x y z, point_step 16, frame *_depth_optical_frame
rostopic echo -n1 /tf_static | grep -c child_frame_id        # ~105 with the aggregator, 1 without
rosrun tf tf_echo base depth_camera_front_lower_depth_optical_frame   # t ≈ [0.368 -0.025 -0.060], looking ~60° down
```

In RViz (fixed frame `odom`) the depth-cloud floor must lie on the
`/lidar/point_cloud` floor (the lidar needs no intrinsics, so it is the
reference), and the front-lower returns from the LF foot must sit on the
rendered LF shank, not RF — a mirrored back-projection would swap them.

### Caveats

- The clouds are **raw**: unlike `/depth_camera_*/point_cloud_self_filtered`,
  the robot's own legs are still in them. Pointing `dynasense_bps_node` at them
  (`depth_cloud_topics`) registers the legs as obstacles; use the
  `*_self_filtered` clouds of `dynasense_self_filter_node` (next section) instead.
- Real-robot bags have no `world` frame; the BPS node would need
  `map_frame: odom` there.
- Do not add the `replay.yaml` static transform
  (`*_camera_parent -> *_depth_optical_frame`) on top of these bags: the bag's
  `/tf_static` already has it, and a second parent breaks the TF tree.

## Self filter + wavemap replay (bag)

`launch/replay_wavemap.launch` replays a real-robot bag and builds a wavemap
voxel map from the depth cameras and/or the lidar while the robot's own body is
removed from the clouds:

```
replay_depth_clouds.launch  (sim time, anymal_description, tf_static aggregator, depth converter, rosbag play)
  /depth_camera_<side>/depth/points (organized xyz)   /lidar/point_cloud   /tf /tf_static
dynasense_self_filter_node                            URDF collision primitives + TF(cloud frame <- link @ stamp)
  drop: NaN | range > max_range (0 = off) | range < min_range | dist(collision) <= body_margin + link_padding[link]
  -> /depth_camera_<side>/depth/points_self_filtered, /lidar/point_cloud_self_filtered   (unorganized xyz, same header)
  -> /self_filter/body_markers                                                          (padded primitives, RViz)
wavemap_ros/ros_server  (config/wavemap_replay.yaml)  -> /wavemap/map, /wavemap/obstacle_pointcloud
rviz -d anymal_d_rsl/config/ui/config_wavemap.rviz -f odom
```

### Run

```bash
catkin build dynasense
source devel/setup.bash
roslaunch dynasense replay_wavemap.launch                     # 2026-09-23-18-25-44.bag, all 6 cameras + lidar, 0.8 m, 5 cm voxels
roslaunch dynasense replay_wavemap.launch depth_cameras:="front_lower rear_lower" use_lidar:=false
roslaunch dynasense replay_wavemap.launch max_range:=0 min_cell_width:=0.03 body_margin:=0.08
roslaunch dynasense replay_wavemap.launch start:=15 duration:=60 rate:=0.5
roslaunch dynasense replay_wavemap.launch rviz:=false quit_when_done:=true    # headless
rosservice call /wavemap/map_request_full                     # RViz started after mapping began: fetch the whole map
```

| arg | default | meaning |
|---|---|---|
| `bag`, `rate`, `start`, `duration`, `delay`, `pause`, `loop`, `quit_when_done` | 18-25-44 bag, 1.0, 0, "", 5 s, false, false, false | rosbag play |
| `depth_cameras` | all six | space/comma separated camera names; `""` = none |
| `use_lidar` | true | |
| `body_margin` | 0.05 m | distance to the collision primitives below which points are removed |
| `max_range` | 0.8 m | far clip about the sensor origin, `0` = off (wavemap then integrates to 5 m) |
| `min_cell_width` | 0.05 m | wavemap voxel size |
| `wavemap_max_range` | = `max_range` | integrator far limit; set it separately (e.g. `max_range:=0 wavemap_max_range:=0.8`) to keep far returns for free-space carving |
| `depth_min_range`, `lidar_min_range` | 0.2 m, 0.5 m | integrator near limits; must exceed 6 x the range sigma (derived and capped by the launch file) |
| `world_frame` | odom | wavemap map frame (the bag has no `world`) |
| `depth_rate`, `decimation` | 15 Hz, 1 | depth converter cap and pixel stride |
| `depth_config` | `config/depth_cameras.yaml` | converter YAML (cameras, topics, intrinsics); `fast_stereo_ros` passes its stereo variant |
| `depth_fx`, `depth_fy`, `depth_cx`, `depth_cy` | Gazebo D435 values | pinhole model wavemap reprojects the depth clouds with; must match the converter's intrinsics (real values: `infra1/camera_info`) |
| `wavemap_threads`, `depth_time_offset`, `lidar_time_offset` | 8, 0, 0.05 s | wavemap tuning |
| `rviz`, `rviz_config` | true, `config_wavemap.rviz` | |

### Self filter (`dynasense_self_filter_node`, `config/self_filter.yaml`)

- The body is every `<collision>` box, cylinder and sphere of the URDF in
  `robot_description_param` (`anymal_description`: 42 primitives on 22 links,
  no meshes). Each link is placed with TF at the cloud's header stamp; a point
  is dropped when its exact Euclidean distance to any primitive is at most
  `body_margin + link_padding[link]` (0 inside). A per-link bounding sphere
  rejects far points cheaply.
- `body_margin` is measured from the **collision** surface. The visual shells
  protrude up to ~4.2 cm beyond the collision shapes on `base` and `lidar` and
  1-2 cm on the legs; add `link_padding: {base: 0.04, lidar: 0.04}` if the
  visual surface should count. The foot sphere plus margin also removes a
  ~8 cm disc of floor under each foot.
- Range band: `min_range < |p| <= max_range` about the sensor origin (the cloud
  frame origin); `max_range: 0` disables the far clip.
- TF: if a link transform is not available within `tf_timeout` the cloud is
  dropped (never published unfiltered); `tf_fallback_latest: true` uses the
  latest transform instead.
- Output: unorganized `x y z` float32, point_step 16, same header stamp and
  `frame_id` (`keep_organized: true` keeps the layout and writes NaN). Empty
  clouds are not published unless `publish_empty: true`.
- Sensor selection: `inputs[].enabled`, or the `enabled_inputs` name list which
  replaces those flags (the launch file builds it from `depth_cameras` and
  `use_lidar`). Disabled inputs are not subscribed, so the depth converter also
  stops converting those cameras.
- `~body_markers`: the padded primitives as a MarkerArray in their link frames
  (padded boxes are drawn with sharp corners; the true hull has rounded edges).
- Statistics: every `stats_period` seconds one line per input with clouds,
  points per cloud, kept / NaN / range / body percentages and ms per cloud
  split into TF, filter and publish time; a summary table is printed at
  shutdown.

### wavemap

- `config/wavemap_replay.yaml` is loaded with `subst_value="true"`, so the
  launch args fill `world_frame`, `min_cell_width`, `wavemap_max_range`,
  `depth_min_range`/`lidar_min_range`, `depth_range_sigma`/`lidar_range_sigma`
  (= max(cell/2, 1 cm / 3 cm), capped below min_range/6), `wavemap_threads` and
  the time offsets.
- **Occupied voxels right in front of a sensor** are a wavemap artefact, not
  filter output: a pixel without a return (NaN, a self-filtered body point, a
  point clipped by `max_range`) keeps range 0 in wavemap's range image, and the
  continuous-ray model then raises the occupancy of cells closer than
  6 x `range_sigma` on every frame. Only the integrator's `min_range` excludes
  them, hence `min_range > 6 x range_sigma` (the launch file enforces it).
  With the old defaults (min_range 0.1 m, sigma 0.025 m) this produced a shell
  of voxels at 0.10-0.15 m from every camera. Six `pinhole_camera_projector`
  integrators use the converter's intrinsics (keep them identical to
  `config/depth_cameras.yaml`, wavemap reprojects the points); the VLP-16 uses a
  `spherical_projector` with 16 x 1800 cells. Map type `hashed_wavelet_octree`
  (the RViz plugin renders surfaces only for that type).
- All seven inputs are always listed; a deselected sensor never publishes.
- The map is published incrementally (`/wavemap/map`, changed blocks only,
  every 0.5 s): start RViz before mapping begins (`delay:=8` on a slow machine)
  or call `/wavemap/map_request_full`. `/wavemap/obstacle_pointcloud` carries
  the occupied cells as a PointCloud2.
- Not covered: per-point lidar motion compensation (wavemap 2.0.1 undistorts
  only Ouster clouds), so a scan smears by up to ~5 cm at 0.5 m/s; the camera
  intrinsics are the Gazebo values (see the depth section).
- **RViz shows the map display with a red cross and no properties (no Topic
  field):** RViz could not load `wavemap_rviz_plugin/WavemapMap` because the
  shell that started it has a `ROS_PACKAGE_PATH` from before the wavemap
  packages were built (the display placeholder then has no properties). Check
  with `rospack plugins --attrib=plugin rviz | grep wavemap` in that shell; if
  it prints nothing, `source ~/catkin_ws/devel/setup.bash` again in that
  terminal (or open a new one) and restart RViz. The error is also logged in
  `~/.ros/log/latest/rosout.log` as "PluginlibFactory: The plugin for class
  'wavemap_rviz_plugin/WavemapMap' failed to load".

### Stereo depth instead of the on-chip depth

`anymal_rsl_utils/fast_stereo_ros` runs Fast-FoundationStereo (TensorRT) on the
infra1/infra2 pairs and publishes `/depth_camera_<side>/stereo/image_rect_raw`
(16UC1) plus `camera_info`. Its `replay_stereo_wavemap.launch` includes
`replay_wavemap.launch` with `depth_config:=.../depth_cameras_stereo.yaml`, so the
converter reads the stereo depth while the cloud topics, the self filter and this
wavemap config stay as they are; pass `depth_fx:=... depth_cy:=...` from the
recording's camera_info so wavemap reprojects with the right model.

### Verify

```bash
rostest dynasense self_filter.test                     # synthetic box/cylinder/sphere robot
rosnode info /self_filter                              # 7 subscriptions, 7 publications + body_markers
rostopic hz /depth_camera_front_lower/depth/points_self_filtered /lidar/point_cloud_self_filtered
rostopic echo -n1 --noarr /depth_camera_front_lower/depth/points_self_filtered   # height 1, x y z, point_step 16
rostopic echo -n1 --noarr /wavemap/map                 # frame odom
rostopic echo -n1 --noarr /wavemap/obstacle_pointcloud # width grows while the robot walks
```

## Live wavemap → BPS (real robot)

`launch/live_wavemap.launch` runs the mapping pipeline on the robot's live
topics and turns the map into the BPS node's external-ray voxels:

```
/depth_camera_{front,rear}_{upper,lower}/depth/image_rect_raw   (driver, + driver camera_info)
  -> dynasense_depth_to_cloud (converter, 15 Hz)  -> /depth_camera_<side>/depth/points
/lidar/point_cloud
  -> self_filter (5 cm body margin, 0.8 m clip)   -> *_wavemap_filtered
  -> wavemap (config/wavemap_live.yaml, odom)     -> /wavemap/map (incremental, every 0.05 s)
  -> wavemap_bps_bridge                           -> /dynasense/wavemap_voxels
       occupied 5 cm cell centres within bridge_radius (xy) of base,
       centre z in [bridge_z_min, bridge_z_max) ABSOLUTE in odom, nearest first
  -> dynasense_bps_node (ray_voxel_source: map_voxels): each message REPLACES the ray buffer
```

It sets no `/use_sim_time`, plays no bag, does not reload `anymal_description`
and does not re-latch static TF; the robot stack provides all of that. It does
**not** start the BPS node.

### Run (real robot, on the LPC)

```bash
# 1. mapping + bridge; the bridge waits for the floor height
roslaunch dynasense live_wavemap.launch depth_fx:=... depth_fy:=... depth_cx:=... depth_cy:=...
# 2. robot standing still on flat ground: estimate the floor, start the BPS side
rosrun dynasense start_bps_hardware.sh
# 3. switch motion_control_manager to parkour
```

`start_bps_hardware.sh` runs `scripts/estimate_floor_z.py` (sole heights of the
four feet from TF, i.e. forward kinematics on top of the estimator's odom pose;
median over 2 s; refuses when the soles differ by more than 3 cm) and then
`launch/bps_hardware.launch floor_z:=<estimate>`, which starts:

- a latched `/dynasense/floor_z` (std_msgs/Float64): the bridge's z band becomes
  `[floor + 0.05, floor + 1.8)`;
- `dynasense_bps_node` with `config/bps.yaml` + `config/bps_live.yaml`
  (`map_frame: odom`, ray voxels from the bridge) and `ground_z` = floor;
- `dynasense_fk_ground_clearance_node`: the 17 gravity rays of the policy
  observation (`/dynasense/ground_clearance`, same message as the Gazebo plugin)
  as the heights of base, HIP, THIGH, SHANK and FOOT above the flat floor,
  clipped to [0, 2] m. Obstacles under the robot are not seen.

Options: `start_bps_hardware.sh --floor-z -0.29` skips the estimate, `--duration`,
`--max-spread`, `--frame` go to the estimator, anything after `--` to the launch
file (e.g. `-- ground_clearance:=false`). Take `depth_fx…cy` from
`rostopic echo -n1 /depth_camera_front_lower/depth/camera_info` (K). In Gazebo
the plugin provides the clearance and `sim_contact_bps.launch` the BPS node, so
none of this is needed there.

| Arg | Default | Meaning |
|---|---|---|
| `floor_z` | empty | floor height in `world_frame`; empty: 0 with `sim:=true`, otherwise read from `floor_z_topic` (`/dynasense/floor_z`, published by `start_bps_hardware.sh`). NOT 0 in odom on ANYmal D (about −0.29 m in the 2026-09-23 bag). The bridge compares it with the lowest foot and warns when it is off by more than 5 cm |
| `bridge_radius` | 1.0 m | horizontal radius around `base` (the policy's ray horizon) |
| `bridge_z_min_above_floor`, `bridge_z_max_above_floor` | 0.05, 1.8 m | z band on cell centres relative to the floor (mirrors the BPS height band) |
| `bridge_z_min`, `bridge_z_max` | empty | absolute band; overrides the floor-relative one when both are set |
| `bridge_max_voxels` | 2000 | nearest-first cap, equal to `ray_buffer_capacity` |
| `bridge_rate` | 20 Hz | extraction and publishing rate |
| `bridge_occupancy_threshold` | 0.0 | log-odds above which a cell counts as occupied |
| `publish_map_period` | 0.05 s | incremental `/wavemap/map` period (bridge latency) |
| `input_retry_period` | 0.02 s | how often wavemap integrates queued clouds |
| `tree_height` | 6 | block size `min_cell_width·2^tree_height`; 5 halves the bytes per publish |
| `wavemap_threads` | 2 | the LPC also runs locomotion |
| `launch_prefix` | `nice -n 10` | process priority of all four nodes |
| `sim` | false | Gazebo: simulated camera topics, `world` frame |
| `filtered_suffix` | `_wavemap_filtered` | self filter output suffix, read by wavemap |
| `depth_cameras`, `use_lidar`, `body_margin`, `max_range`, `min_cell_width`, `depth_fx…cy`, `depth_rate`, … | as in `replay_wavemap.launch` | |

The BPS side is configured in `config/bps.yaml`: `ray_voxel_source`
(`sensors` | `map_voxels`), `map_voxel_topic`, `map_voxel_timeout` (warn after
0.5 s without a message, keep the last voxels unless
`map_voxel_clear_on_timeout`), `map_voxels_apply_gates` (false: the bridge band
governs, `ground_z` then only matters for contact voxels). `BpsState` is
unchanged, so the controller needs no rebuild; `num_ray_voxels` reports the
wavemap voxel count.

The bridge logs every `stats_period` and at shutdown: extraction time, cells
visited, voxels kept, map message rate/size/merge time, map age, TF failures and
the lowest foot height against `floor_z`.

### Gazebo

```bash
roslaunch dynasense sim_contact_bps.launch bps_ray_source:=map_voxels   # BPS ray voxels from the bridge
roslaunch dynasense live_wavemap.launch sim:=true                       # second terminal
rviz -d $(rospack find anymal_d_rsl)/config/ui/config_depth_lidar.rviz  # group "Wavemap (live_wavemap.launch)"
```

`sim:=true` reads the simulated cameras' `aligned_depth_to_color/image_raw`
(`config/depth_cameras_sim.yaml`), builds the map in the Gazebo `world` frame
(the BPS node's `map_frame` in simulation) and keeps `floor_z` at 0, the height
of the contact_bps terrain. Do not pass the robot's odom value (−0.29) there. The
live filter publishes `*_wavemap_filtered`, so it does not collide with the
`/lidar/point_cloud_self_filtered` of `sim_contact_bps.launch`. Without
`bps_ray_source:=map_voxels` the BPS node keeps binning the sensor clouds and the
wavemap is only displayed. Contacts and ground clearance still come from Gazebo.

### Check

```bash
rosrun dynasense check_wavemap_voxels.py _radius:=1.0 _z_min:=-0.24 _z_max:=1.51
# floor: a wide band and a histogram (start a second bridge with _z_min:=-1.5 _z_max:=1.5)
rosrun dynasense check_wavemap_voxels.py _topic:=<wide topic> _histogram:=true _duration:=20
```

Bag test of the live launch on a PC (fresh master): set `use_sim_time`, load
`anymal_description` from the xacro, run `scripts/tf_static_aggregator.py
_bags:=<bag>`, start the launch and the BPS node as above, then `rosbag play
--clock` the four face cameras' `depth/image_rect_raw`, `/lidar/point_cloud`,
`/tf` and `/tf_static`.

### Caveats

- **Latency.** An obstacle reaches the BPS 60–250 ms after it is seen
  (integration, map publish, bridge tick), against 20–40 ms for the direct ray
  path. The bridge's map age shows it.
- **One wavemap thread.** Integration, map publishing and the obstacle cloud
  share one thread; if it falls behind (wavemap logs "Dropping cloud", the map
  age grows), use `tree_height:=5`, `publish_map_period:=0.1` or `depth_rate:=10`.
- **Map growth.** wavemap 2.0.1's `crop_map` looks up the body pose at exactly
  the current time and fails on live TF, so the map is not cropped; reset it
  between runs (`rosservice call /wavemap/reset_map`).
- **Absolute z band vs odom drift.** Re-measure `floor_z` each session; leave a
  margin on `bridge_z_min` on uneven ground.
- **Ghost cells.** The 0.8 m clip also stops free-space carving beyond 0.8 m, so
  noise inside 0.8 m is never cleared; compare with
  `max_range:=0 wavemap_max_range:=0.8`.
- **Build.** The bridge is compiled with `-march=native` (wavemap's flags); build
  it on the computer that runs it. It is skipped when wavemap is not in the
  workspace.
- **Training mismatch.** The policy was trained on a 1 m ray-hit LRU, not on a
  persistent dense occupancy map.
- Contact voxels have no source on hardware (they come from Gazebo contact
  topics). The FK ground clearance assumes a flat floor at the estimated height.

## foot_tof

Unrelated earlier work: two Gazebo ray sensor plugins and a preprocessor node
for the multi-zone foot ToF sensors. `foot_tof/bin/tof_preprocessor.py` and
`foot_tof/launch/sim_foot_tof.launch` reference `rl_controller` messages and a
launch file that exist only on the `dev/foot_tof` branch of
`nvidia_rl_controller`, so they do not run against this branch.
