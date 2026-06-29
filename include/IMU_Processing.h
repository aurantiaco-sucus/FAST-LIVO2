/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Header guard to prevent multiple inclusions of this file
#ifndef IMU_PROCESSING_H
// Define the header guard macro
#define IMU_PROCESSING_H

// Include Eigen core for matrix and vector operations
#include <Eigen/Eigen>
// Include common shared types: StatesGroup, MeasureGroup, LidarMeasureGroup, etc.
#include "common_lib.h"
// Include condition_variable for thread synchronization
#include <condition_variable>
// Include nav_msgs/Odometry for publishing IMU-propagated odometry
#include <nav_msgs/Odometry.h>
// Include SO(3) math utilities for exponential/logarithmic maps
#include <utils/so3_math.h>
// Include fstream for logging IMU data to a file
#include <fstream>
// Comparison function for sorting points by their time offset (stored in curvature)
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

// Handles IMU initialization, forward propagation, state covariance prediction, and
// LiDAR point cloud undistortion using IMU measurements between scan times.
/// *************IMU Process and undistortion
class ImuProcess
{
public:
  // Macro to enforce proper memory alignment for classes containing Eigen members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Default constructor
  ImuProcess();
  // Destructor
  ~ImuProcess();

  // Reset the IMU processor to its initial state
  void Reset();
  // Reset the IMU processor with a specific start timestamp and last IMU message
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  // Set the LiDAR-to-IMU extrinsic calibration (translation and rotation)
  void set_extrinsic(const V3D &transl, const M3D &rot);
  // Set only the translation part of the LiDAR-to-IMU extrinsic calibration
  void set_extrinsic(const V3D &transl);
  // Set the extrinsic calibration from a 4x4 homogeneous transformation matrix
  void set_extrinsic(const MD(4, 4) & T);
  // Set the scaling factor for gyroscope measurement covariance
  void set_gyr_cov_scale(const V3D &scaler);
  // Set the scaling factor for accelerometer measurement covariance
  void set_acc_cov_scale(const V3D &scaler);
  // Set the covariance for gyroscope bias random walk
  void set_gyr_bias_cov(const V3D &b_g);
  // Set the covariance for accelerometer bias random walk
  void set_acc_bias_cov(const V3D &b_a);
  // Set the covariance for inverse exposure time random walk
  void set_inv_expo_cov(const double &inv_expo);
  // Set the number of IMU frames required for initialization
  void set_imu_init_frame_num(const int &num);
  // Disable use of IMU measurements
  void disable_imu();
  // Disable gravity vector estimation during IMU initialization
  void disable_gravity_est();
  // Disable gyroscope and accelerometer bias estimation
  void disable_bias_est();
  // Disable exposure time estimation
  void disable_exposure_est();
  // Main processing function: IMU propagation, undistortion, and state update
  void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  // Undistort a LiDAR point cloud using interpolated IMU poses
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  // Output file stream for logging IMU data
  ofstream fout_imu;
  // Mean accelerometer norm computed during IMU initialization
  double IMU_mean_acc_norm;
  // Gyroscope readings with bias removed
  V3D unbiased_gyr;

  // Covariance of accelerometer measurements
  V3D cov_acc;
  // Covariance of gyroscope measurements
  V3D cov_gyr;
  // Covariance of gyroscope bias random walk
  V3D cov_bias_gyr;
  // Covariance of accelerometer bias random walk
  V3D cov_bias_acc;
  // Covariance of inverse exposure time random walk
  double cov_inv_expo;
  // Timestamp of the first LiDAR scan
  double first_lidar_time;
  // Flag indicating that IMU time initialization is complete
  bool imu_time_init = false;
  // Flag indicating that IMU initialization is required (first scans)
  bool imu_need_init = true;
  // 3x3 identity matrix for use in computations
  M3D Eye3d;
  // 3x1 zero vector for use in computations
  V3D Zero3d;
  // Type identifier of the LiDAR sensor (Livox, Velodyne, Ouster, etc.)
  int lidar_type;

private:
  // Perform IMU initialization by static alignment: estimate gravity and biases
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
  // Forward propagate the state when no IMU measurements are available (constant velocity model)
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  // Point cloud waiting for processing in the next step
  PointCloudXYZI pcl_wait_proc;
  // Pointer to the most recent IMU message
  sensor_msgs::ImuConstPtr last_imu;
  // Pointer to the current undistorted LiDAR point cloud
  PointCloudXYZI::Ptr cur_pcl_un_;
  // Vector of IMU poses for interpolation during undistortion
  vector<Pose6D> IMUpose;
  // Rotation matrix from LiDAR frame to IMU frame
  M3D Lid_rot_to_IMU;
  // Translation vector from LiDAR frame to IMU frame
  V3D Lid_offset_to_IMU;
  // Mean accelerometer value during the current initialization window
  V3D mean_acc;
  // Mean gyroscope value during the current initialization window
  V3D mean_gyr;
  // Angular velocity from the previous IMU step
  V3D angvel_last;
  // Acceleration from the previous IMU step (compensated for gravity)
  V3D acc_s_last;
  // Timestamp of the end of the last propagation interval
  double last_prop_end_time;
  // Timestamp of the last LiDAR scan end time
  double time_last_scan;
  // Current initialization iteration count and maximum initialization frames
  int init_iter_num = 1, MAX_INI_COUNT = 20;
  // Flag indicating the first frame of data
  bool b_first_frame = true;
  // Flag enabling IMU processing
  bool imu_en = true;
  // Flag enabling gravity vector estimation
  bool gravity_est_en = true;
  // Flag enabling gyroscope and accelerometer bias estimation
  bool ba_bg_est_en = true;
  // Flag enabling camera exposure time estimation
  bool exposure_estimate_en = true;
};
// Type alias for a shared pointer to an ImuProcess instance
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif