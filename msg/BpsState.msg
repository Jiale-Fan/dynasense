# Basis-point-set (BPS) encoding of the contact-history voxel map.
#
# Produced by dynasense_bps_node at the policy rate. Feeds policy observation
# block [49,151) verbatim: 34 query points, each a unit-ish direction toward the
# nearest buffered obstacle voxel, expressed in the YAW-ONLY base frame and
# attenuated by distance. Values lie in [-1, 1].
#
# Query-point order (34 points, flattened xyz):
#   base:     6 points
#   LF, LH, RF, RH (in that order), each 7 points:
#             THIGH 3, SHANK 1, FOOT 3

std_msgs/Header header          # stamp = query time; frame_id = yaw-only base frame

float32[102] directions         # 34 x [x, y, z]

# --- debug / visualisation, not consumed by the policy ---

float32[34] distances                   # exterior-box SDF distance per query point, capped at 0.5 m
geometry_msgs/Point[34] query_points    # the 34 query points in the odom frame
uint32 num_contact_voxels               # occupancy of the physical-contact LRU buffer
uint32 num_ray_voxels                   # occupancy of the external-ray LRU buffer
