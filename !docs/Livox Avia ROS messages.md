The format is defined by two ROS message types vendored in the repo at include/livox_ros_driver/.

CustomMsg (livox_ros_driver/CustomMsg)

```
Header header        # std_msgs/Header (seq, stamp, frame_id)
uint64 timebase      # Time of first point (nanoseconds)
uint32 point_num     # Number of points in this frame
uint8  lidar_id      # LiDAR device ID
uint8[3] rsvd        # Reserved
CustomPoint[] points # Array of point data
```

CustomPoint (livox_ros_driver/CustomPoint) — 16 bytes each

```
uint32 offset_time   # Offset from timebase (nanoseconds)
float32 x            # X coordinate (meters)
float32 y            # Y coordinate (meters)
float32 z            # Z coordinate (meters)
uint8  reflectivity  # 0–255
uint8  tag           # Livox point tag (e.g. 0=normal, 1=noise)
uint8  line          # Laser line/scanner number
```

So each CustomMsg contains a ROS Header for the frame timestamp, a timebase giving the absolute time of the first point in ns, and an array of CustomPoint structs each with its own offset_time (ns from timebase), XYZ coordinates as float32, and three uint8 metadata fields. This is the Livox-proprietary format, distinct from sensor_msgs/PointCloud2.