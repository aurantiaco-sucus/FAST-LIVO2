// Begin include guard to prevent multiple inclusions of this header.
#ifndef TYPES_H
// Define the include guard symbol for the types header.
#define TYPES_H

// Include Eigen core linear algebra functionality for matrix and vector types.
#include <Eigen/Eigen>
// Include PCL point cloud container class.
#include <pcl/point_cloud.h>
// Include PCL point type definitions (PointXYZI, PointXYZRGB, etc.).
#include <pcl/point_types.h>

// Alias for a PCL point with XYZ, intensity, and normal information.
typedef pcl::PointXYZINormal PointType;
// Alias for a PCL point with XYZ and RGB color information.
typedef pcl::PointXYZRGB PointTypeRGB;
// Alias for a PCL point with XYZ, RGB, and alpha channel information.
typedef pcl::PointXYZRGBA PointTypeRGBA;
// Alias for a point cloud of PointType (XYZINormal) points.
typedef pcl::PointCloud<PointType> PointCloudXYZI;
// Alias for an STL vector of PointType with Eigen-aligned allocator.
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
// Alias for a point cloud of RGB-colored points.
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
// Alias for a point cloud of RGBA-colored points.
typedef pcl::PointCloud<PointTypeRGBA> PointCloudXYZRGBA;

// Alias for a 2D single-precision floating-point vector.
typedef Eigen::Vector2f V2F;
// Alias for a 2D double-precision floating-point vector.
typedef Eigen::Vector2d V2D;
// Alias for a 3D double-precision floating-point vector.
typedef Eigen::Vector3d V3D;
// Alias for a 3x3 double-precision matrix.
typedef Eigen::Matrix3d M3D;
// Alias for a 3D single-precision floating-point vector.
typedef Eigen::Vector3f V3F;
// Alias for a 3x3 single-precision matrix.
typedef Eigen::Matrix3f M3F;

// Macro to declare a double-precision matrix with compile-time rows (a) and columns (b).
#define MD(a, b) Eigen::Matrix<double, (a), (b)>
// Macro to declare a double-precision column vector with compile-time dimension (a).
#define VD(a) Eigen::Matrix<double, (a), 1>
// Macro to declare a single-precision matrix with compile-time rows (a) and columns (b).
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
// Macro to declare a single-precision column vector with compile-time dimension (a).
#define VF(a) Eigen::Matrix<float, (a), 1>

// Structure holding preintegrated LiDAR-relative states at an IMU measurement time.
struct Pose6D
{
  /*** the preintegrated Lidar states at the time of IMU measurements in a frame ***/
  // Time offset of this IMU measurement relative to the first LiDAR point in the frame.
  double offset_time; // the offset time of IMU measurement w.r.t the first lidar point
  // Preintegrated total linear acceleration at the LiDAR origin in the global frame.
  double acc[3];      // the preintegrated total acceleration (global frame) at the Lidar origin
  // Unbiased angular velocity at the LiDAR origin in the body frame.
  double gyr[3];      // the unbiased angular velocity (body frame) at the Lidar origin
  // Preintegrated velocity at the LiDAR origin in the global frame.
  double vel[3];      // the preintegrated velocity (global frame) at the Lidar origin
  // Preintegrated position at the LiDAR origin in the global frame.
  double pos[3];      // the preintegrated position (global frame) at the Lidar origin
  // Preintegrated rotation matrix at the LiDAR origin (row-major, 9 elements).
  double rot[9];      // the preintegrated rotation (global frame) at the Lidar origin
};

// End of the types header include guard.
#endif
