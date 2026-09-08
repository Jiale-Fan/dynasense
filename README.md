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
├── msg/{BpsState,GroundClearance}.msg
├── config/bps.yaml
├── launch/sim_contact_bps.launch       # the whole ContactBPS stack
├── launch/sim_bps_sensors_only.launch  # same, minus the learned controller
└── foot_tof/                           # unrelated: foot ToF ray plugins + preprocessor
```

## ContactBPS pipeline

Produces the exteroceptive two thirds of the observation for
`rl_controller::RlController`, per
`DS_LOCORESET_DEPTHHEIGHTSCAN_CONTACTBPS_DEPLOYMENT_HANDOVER.md`.

```
gzserver
  libgazebo_ros_bumper.so x8 (declared in the ANYmal-D leg xacros)
      -> /contacts/{LF,LH,RF,RH}/{thigh,shank}   gazebo_msgs/ContactsState @200 Hz
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

> **Both currently ship OFF.** The map stays permanently empty, so all 102 BPS
> observation values are exactly zero and the policy is **blind to obstacles**.
> This is a deliberate debugging configuration for checking behaviour without
> meaningful BPS input. Set both back to `true` for normal operation.

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
- **External rays** — the six self-filtered depth clouds and the lidar cloud,
  binned onto the trained ray grids (`24x16` at FOV `86.62 x 59.73` deg per
  camera; 5 channels over vFOV `[0,15]` deg at 30 deg horizontal resolution for
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

The grid itself is a global 5 cm lattice in `odom` rather than the training
sensor's `160x160x40` per-environment grid, which has no deployment equivalent.
The training extent is reproduced by the height band and by
`max_horizontal_range` (4 m from the base). Both LRU buffers are hard-capped, so
memory is bounded regardless.

### Contact sensors

Declared in `anymal_d_rsl/assets/urdf/leg_eflesh/{thigh,shank}/*_macro.urdf.xacro`
with the stock `libgazebo_ros_bumper.so`. Two details matter:

- URDF fixed-joint lumping folds `<LEG>_thigh_fixed` into `<LEG>_THIGH` and
  mangles collision names, so the sensors bind with
  `<collision>__default__</collision>` (every collision of the lumped link).
- `<frameName>` is the link that **survives** lumping (`<LEG>_THIGH`,
  `<LEG>_SHANK`), so patch positions land in a frame that exists in TF. The
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

## foot_tof

Unrelated earlier work: two Gazebo ray sensor plugins and a preprocessor node
for the multi-zone foot ToF sensors. `foot_tof/bin/tof_preprocessor.py` and
`foot_tof/launch/sim_foot_tof.launch` reference `rl_controller` messages and a
launch file that exist only on the `dev/foot_tof` branch of
`nvidia_rl_controller`, so they do not run against this branch.
