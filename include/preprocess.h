/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Header guard to prevent multiple inclusions of this file
#ifndef PREPROCESS_H_
// Define the header guard macro
#define PREPROCESS_H_

// Include common shared types: point types, StatesGroup, enums, and macros
#include "common_lib.h"
// Include Livox custom ROS message definition
#include <livox_ros_driver/CustomMsg.h>
// Include PCL-to-ROS conversion utilities
#include <pcl_conversions/pcl_conversions.h>

// Use the standard C++ namespace
using namespace std;

// Macro to check if a value is valid (not an extreme outlier)
#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

// Enumeration of LiDAR point feature classifications
enum LiDARFeature
{
  // Normal point, no special feature
  Nor,
  // Point that possibly belongs to a planar surface
  Poss_Plane,
  // Point confirmed to belong to a planar surface
  Real_Plane,
  // Point at an edge jump discontinuity
  Edge_Jump,
  // Point on an edge of a planar surface
  Edge_Plane,
  // Wire-like point (thin structure)
  Wire,
  // Point with zero or invalid range
  ZeroPoint
};
// Enumeration for forward/backward direction relative to a point
enum Surround
{
  // Previous point in the scan order
  Prev,
  // Next point in the scan order
  Next
};
// Enumeration for edge jump classification types
enum E_jump
{
  // Normal edge, no jump
  Nr_nor,
  // Zero-range point causing a jump
  Nr_zero,
  // 180-degree jump (opposite direction)
  Nr_180,
  // Infinite-range point causing a jump
  Nr_inf,
  // Blind spot region causing a jump
  Nr_blind
};

// Structure holding per-point geometric properties for feature extraction
struct orgtype
{
  // Range (distance) of the point from the sensor origin
  double range;
  // Distance to the line connecting neighboring points
  double dista;
  // Horizontal and vertical angles of the point
  double angle[2];
  // Intersection angle between adjacent line segments
  double intersect;
  // Edge jump type for previous and next neighbors
  E_jump edj[2];
  // Classified LiDAR feature type for this point
  LiDARFeature ftype;
  // Default constructor initializes all fields to safe defaults
  orgtype()
  {
    // Initialize range to zero
    range = 0;
    // Set both edge jump classifications to normal
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    // Set feature type to normal
    ftype = Nor;
    // Initialize intersect angle to 2 (valid default)
    intersect = 2;
  }
};

// Velodyne/VLP-16 point struct with intensity, time offset, and ring index.
/*** Velodyne ***/
namespace velodyne_ros
{
// Velodyne point structure with 4D position, intensity, time offset, and ring index
struct EIGEN_ALIGN16 Point
{
  // Macro to add x, y, z, and padding fields for 4D position
  PCL_ADD_POINT4D;
  // Laser return intensity
  float intensity;
  // Time offset from the start of the scan (seconds)
  float time;
  // Laser ring (scan line) index
  std::uint16_t ring;
  // Macro to enforce proper memory alignment for the struct
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
// Register the Velodyne point struct with PCL for ROS serialization
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))
/****************/

// Ouster OS1/OS2 point struct with intensity, timestamp, reflectivity, ring, and ambient.
/*** Ouster ***/
namespace ouster_ros
{
// Ouster point structure with 4D position, intensity, timestamp, reflectivity, ring, ambient, and range
struct EIGEN_ALIGN16 Point
{
  // Macro to add x, y, z, and padding fields for 4D position
  PCL_ADD_POINT4D;
  // Laser return intensity
  float intensity;
  // Timestamp in nanoseconds (or microseconds depending on firmware)
  std::uint32_t t;
  // Reflectivity measurement (signal strength normalized)
  std::uint16_t reflectivity;
  // Laser ring (scan line) index
  uint8_t ring;
  // Ambient light level reading
  std::uint16_t ambient;
  // Raw range measurement in millimeters
  std::uint32_t range;
  // Macro to enforce proper memory alignment for the struct
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
// Register the Ouster point struct with PCL for ROS serialization
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
/****************/

// Hesai XT32 point struct with intensity, timestamp, and ring.
/*** Hesai_XT32 ***/
namespace xt32_ros
{
// Hesai XT32 point structure with 4D position, intensity, timestamp, and ring index
struct EIGEN_ALIGN16 Point
{
  // Macro to add x, y, z, and padding fields for 4D position
  PCL_ADD_POINT4D;
  // Laser return intensity
  float intensity;
  // Timestamp of the point in seconds
  double timestamp;
  // Laser ring (scan line) index
  std::uint16_t ring;
  // Macro to enforce proper memory alignment for the struct
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
// Register the Hesai XT32 point struct with PCL for ROS serialization
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

// Hesai Pandar128 point struct with intensity, timestamp, and ring.
/*** Hesai_Pandar128 ***/
namespace Pandar128_ros
{
// Hesai Pandar128 point structure with 4D position, intensity, timestamp, and ring index
struct EIGEN_ALIGN16 Point
{
  // Macro to add x, y, z, and padding fields for 4D position
  PCL_ADD_POINT4D;
  // Laser return intensity (8-bit unsigned)
  uint8_t intensity;
  // Timestamp of the point in seconds
  double timestamp;
  // Laser ring (scan line) index
  uint16_t ring;
  // Macro to enforce proper memory alignment for the struct
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace Pandar128_ros
// Register the Hesai Pandar128 point struct with PCL for ROS serialization
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

// Robosense Airy point struct with intensity, timestamp, and ring.
/*** Robosense_Airy ***/
namespace robosense_ros
{
// Robosense Airy point structure with 4D position, intensity, timestamp, and ring index
struct EIGEN_ALIGN16 Point
{
  // Macro to add x, y, z, and padding fields for 4D position
  PCL_ADD_POINT4D;
  // Laser return intensity
  float intensity;
  // Timestamp of the point in seconds
  double timestamp;
  // Laser ring (scan line) index
  uint16_t ring;
  // Macro to enforce proper memory alignment for the struct
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace robosense_ros
// Register the Robosense Airy point struct with PCL for ROS serialization
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

// Main LiDAR preprocessing class: handles point cloud feature extraction and LiDAR-specific parsing
class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Default constructor
  Preprocess();
  // Destructor
  ~Preprocess();

  // Process a Livox custom ROS message and output a processed point cloud
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  // Process a standard ROS PointCloud2 message and output a processed point cloud
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  // Configure the preprocessor: feature extraction flag, LiDAR type, blind distance, and filter count
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  // Full-resolution point cloud, corner features, and surface features after processing
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  // Ring-separated point buffers for multi-line LiDARs (up to 128 lines)
  PointCloudXYZI pl_buff[128]; // maximum 128 line lidar
  // Per-point geometric types for each scan line (up to 128 lines)
  vector<orgtype> typess[128]; // maximum 128 line lidar
  // LiDAR sensor type identifier, point subsampling filter count, and number of scan lines
  int lidar_type, point_filter_num, N_SCANS;
  
  // Blind region distance (points closer than this are rejected) and its squared value
  double blind, blind_sqr;
  // Flags for enabling feature extraction and whether the sensor provides per-point time offsets
  bool feature_enabled, given_offset_time;
  // ROS publishers for full cloud, surface features, and corner features
  ros::Publisher pub_full, pub_surf, pub_corn;

private:
  // Handler for Livox Avia custom message format
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  // Handler for Ouster OS1-64 standard PointCloud2 messages
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Handler for Velodyne standard PointCloud2 messages
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Handler for Hesai XT32 standard PointCloud2 messages
  void xt32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Handler for Hesai Pandar128 standard PointCloud2 messages
  void Pandar128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Handler for Robosense Airy standard PointCloud2 messages
  void robosense_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Handler for Intel L515 LiDAR standard PointCloud2 messages
  void l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Extract edge and surface features from the processed point cloud
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  // Publish a point cloud to the appropriate ROS topic
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  // Determine if a sequence of points forms a planar surface and compute the direction
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  // Check if a small group of points lies on a plane
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  // Determine if an edge jump exists between consecutive points
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  // Number of consecutive points in a group for plane/edge detection
  int group_size;
  // Distance thresholds A and B for plane fitting, and the infinity bound
  double disA, disB, inf_bound;
  // Ratio limits for accepting a mid-point as within the plane: max-mid, mid-min, max-min
  double limit_maxmid, limit_midmin, limit_maxmin;
  // Point-to-line ratio threshold for edge detection
  double p2l_ratio;
  // Upward and downward edge jump limits in meters
  double jump_up_limit, jump_down_limit;
  // Cosine of 160 degrees for wide-angle jump detection
  double cos160;
  // Edge feature parameters A and B for curvature-based edge scoring
  double edgea, edgeb;
  // Small plane intersection angle threshold and ratio threshold
  double smallp_intersect, smallp_ratio;
  // Velocity components for motion compensation in x, y, z
  double vx, vy, vz;
};
// Type alias for a shared pointer to a Preprocess instance
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_