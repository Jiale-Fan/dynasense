# Downward clearance from each link origin along world gravity, in metres.
#
# Produced by the dynasense ground-clearance Gazebo model plugin. One ray per
# link origin, cast along normalised world gravity against the terrain, clipped
# to [0, max_range]; a ray that hits nothing reports max_range.
#
# Order (17 entries) matches the policy observation block [151,168):
#   base,
#   LF_HIP, LF_THIGH, LF_SHANK, LF_FOOT,
#   LH_HIP, LH_THIGH, LH_SHANK, LH_FOOT,
#   RF_HIP, RF_THIGH, RF_SHANK, RF_FOOT,
#   RH_HIP, RH_THIGH, RH_SHANK, RH_FOOT

std_msgs/Header header

float32[17] clearance

# Range at which a ray is declared a miss (metres). Misses report this value.
float32 max_range
