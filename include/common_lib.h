/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Begin include guard to prevent multiple inclusions of this header.
#ifndef COMMON_LIB_H
// Define the include guard symbol for the common library header.
#define COMMON_LIB_H

// Include SO(3) exponential and logarithmic map utilities for rotation operations.
#include <utils/so3_math.h>
// Include project-wide type aliases (V3D, M3D, PointCloudXYZI, etc.).
#include <utils/types.h>
// Include ANSI terminal color macros for debug console output.
#include <utils/color.h>
// Include OpenCV headers for image processing functionality.
#include <opencv2/opencv.hpp>
// Include ROS IMU sensor message type for handling gyroscope and accelerometer data.
#include <sensor_msgs/Imu.h>
// Include Sophus SE(3) Lie group algebra for 3D rigid body transformations.
#include <sophus/se3.h>
// Include ROS transform broadcaster for publishing coordinate frame transforms.
#include <tf/transform_broadcaster.h>

// Import all standard C++ library names into the global namespace.
using namespace std;
// Import all Eigen linear algebra names into the global namespace.
using namespace Eigen;
// Import all Sophus Lie group names into the global namespace.
using namespace Sophus;

// Macro to print the current source file name and line number to stdout for debugging.
#define print_line std::cout << __FILE__ << ", " << __LINE__ << std::endl;
// Standard gravitational acceleration magnitude in m/s^2 (Guangdong/China region).
#define G_m_s2 (9.81)   // Gravaty const in GuangDong/China
// Dimensionality of the joint EKF state vector (SO(3) contributes 3 of the 19 dimensions).
#define DIM_STATE (19)  // Dimension of states (Let Dim(SO(3)) = 3)
// Initial scalar value placed on the diagonal of the state covariance matrix.
#define INIT_COV (0.01)
// Capacity constant for large preallocated buffers (500 entries).
#define SIZE_LARGE (500)
// Capacity constant for small preallocated buffers (100 entries).
#define SIZE_SMALL (100)
// Macro to expand a 3-element C array into three comma-separated scalar arguments.
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
// Macro to expand a 9-element C array into nine comma-separated scalar arguments.
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
// Macro to build an absolute file path under the project ROOT_DIR/Log/ directory for debug output.
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

// Supported LiDAR sensor types for point cloud processing.
enum LID_TYPE
{
  // Livox Avia solid-state LiDAR sensor.
  AVIA = 1,
  // Velodyne VLP-16 16-channel mechanical spinning LiDAR.
  VELO16 = 2,
  // Ouster OS-64 64-channel mechanical spinning LiDAR.
  OUST64 = 3,
  // Intel RealSense L515 solid-state LiDAR sensor.
  L515 = 4,
  // Livox XT32 solid-state LiDAR sensor.
  XT32 = 5,
  // Huawei/Pandar 128-channel mechanical spinning LiDAR.
  PANDAR128 = 6,
  // RoboSense mechanical spinning LiDAR (general/unspecified model).
  ROBOSENSE = 7
};
// Operating modes: only LiDAR odometry, LiDAR-inertial odometry, or full LIVO.
enum SLAM_MODE
{
  // Pure LiDAR-only odometry mode (no IMU, no visual).
  ONLY_LO = 0,
  // LiDAR-inertial odometry mode (IMU aided, visual disabled).
  ONLY_LIO = 1,
  // Full LiDAR-inertial-visual odometry mode.
  LIVO = 2
};
// EKF initialization and update stage: waiting, visual update, LiDAR update, or LiDAR-only.
enum EKF_STATE
{
  // EKF has not yet been initialized and is waiting for measurements.
  WAIT = 0,
  // Visual-inertial odometry EKF update step is active.
  VIO = 1,
  // LiDAR-inertial odometry EKF update step is active.
  LIO = 2,
  // LiDAR-only odometry EKF update step (no IMU aiding).
  LO = 3
};

// A synchronized set of IMU messages and an optional camera image within a processing interval.
struct MeasureGroup
{
  // Timestamp of the visual (camera) measurement.
  double vio_time;
  // Timestamp of the LiDAR-inertial measurement.
  double lio_time;
  // FIFO queue of IMU messages accumulated between consecutive LiDAR scans.
  deque<sensor_msgs::Imu::ConstPtr> imu;
  // Grayscale camera image associated with this measurement group, or empty if unavailable.
  cv::Mat img;
  // Default constructor initializing both timestamps to zero.
  MeasureGroup()
  {
    // Initialize the visual timestamp to zero.
    vio_time = 0.0;
    // Initialize the LiDAR-inertial timestamp to zero.
    lio_time = 0.0;
  };
};

// A complete LiDAR-inertial-visual measurement bundle spanning one processing cycle.
struct LidarMeasureGroup
{
  // Start timestamp of the current LiDAR scan frame.
  double lidar_frame_beg_time;
  // End timestamp of the current LiDAR scan frame.
  double lidar_frame_end_time;
  // Timestamp of the most recent LiDAR-inertial odometry state update.
  double last_lio_update_time;
  // Raw incoming LiDAR point cloud for the current scan frame.
  PointCloudXYZI::Ptr lidar;
  // Processed point cloud (edge/plane features) for the current LiDAR frame.
  PointCloudXYZI::Ptr pcl_proc_cur;
  // Processed point cloud (edge/plane features) for the upcoming LiDAR frame.
  PointCloudXYZI::Ptr pcl_proc_next;
  // Deque of intra-frame IMU-visual measurement groups associated with this LiDAR frame.
  deque<struct MeasureGroup> measures;
  // Current EKF processing stage indicator (WAIT, VIO, LIO, or LO).
  EKF_STATE lio_vio_flg;
  // Monotonically increasing sequential index of the current LiDAR scan.
  int lidar_scan_index_now;

  // Default constructor initializing all members to safe default values.
  LidarMeasureGroup()
  {
    // Set the frame begin time to negative zero (just below the first valid measurement).
    lidar_frame_beg_time = -0.0;
    // Set the frame end time to zero.
    lidar_frame_end_time = 0.0;
    // Mark that no LIO state update has occurred yet.
    last_lio_update_time = -1.0;
    // Initialize the EKF stage to WAIT (not yet initialized).
    lio_vio_flg = WAIT;
    // Allocate an empty point cloud for the raw LiDAR data member.
    this->lidar.reset(new PointCloudXYZI());
    // Allocate an empty point cloud for the current frame processed features.
    this->pcl_proc_cur.reset(new PointCloudXYZI());
    // Allocate an empty point cloud for the next frame processed features.
    this->pcl_proc_next.reset(new PointCloudXYZI());
    // Clear the deque of intra-frame measurement groups.
    this->measures.clear();
    // Start the LiDAR scan indexing at zero.
    lidar_scan_index_now = 0;
    // Confirm that no LIO update has occurred (duplicate assignment for safety).
    last_lio_update_time = -1.0;
  };
};

// A 3D point tracked through body, IMU, and world frames with full uncertainty propagation.
typedef struct pointWithVar
{
  // 3D point coordinates in the LiDAR body frame.
  Eigen::Vector3d point_b;     // point in the lidar body frame
  // 3D point coordinates in the IMU body frame (after extrinsic calibration).
  Eigen::Vector3d point_i;     // point in the imu body frame
  // 3D point coordinates in the global world frame.
  Eigen::Vector3d point_w;     // point in the world frame
  // Point covariance with the state covariance component removed.
  Eigen::Matrix3d var_nostate; // the var removed the state covarience
  // Point covariance expressed in the body frame.
  Eigen::Matrix3d body_var;
  // Full point covariance matrix in the world frame.
  Eigen::Matrix3d var;
  // Skew-symmetric cross-product matrix of the point (for Jacobian computation).
  Eigen::Matrix3d point_crossmat;
  // Surface normal vector at this point (for plane-based residuals).
  Eigen::Vector3d normal;
  // Default constructor zero-initializing all members.
  pointWithVar()
  {
    // Zero the state-decoupled covariance matrix.
    var_nostate = Eigen::Matrix3d::Zero();
    // Zero the full covariance matrix.
    var = Eigen::Matrix3d::Zero();
    // Zero the body-frame covariance matrix.
    body_var = Eigen::Matrix3d::Zero();
    // Zero the skew-symmetric cross-product matrix.
    point_crossmat = Eigen::Matrix3d::Zero();
    // Zero the body-frame point coordinates.
    point_b = Eigen::Vector3d::Zero();
    // Zero the IMU-frame point coordinates.
    point_i = Eigen::Vector3d::Zero();
    // Zero the world-frame point coordinates.
    point_w = Eigen::Vector3d::Zero();
    // Zero the surface normal vector.
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;


// 19-dimensional EKF state: rotation, position, inverse exposure time, velocity, biases, gravity.
struct StatesGroup
{
  // Default constructor initializing the EKF state to identity rotations and zero vectors.
  StatesGroup()
  {
    // Initialize the attitude rotation matrix to identity (no rotation).
    this->rot_end = M3D::Identity();
    // Initialize the position to the origin (zero vector).
    this->pos_end = V3D::Zero();
    // Initialize the velocity to zero.
    this->vel_end = V3D::Zero();
    // Initialize the gyroscope bias to zero.
    this->bias_g = V3D::Zero();
    // Initialize the accelerometer bias to zero.
    this->bias_a = V3D::Zero();
    // Initialize the gravity vector to zero (will be estimated during initialization).
    this->gravity = V3D::Zero();
    // Set the inverse exposure time to a default positive value.
    this->inv_expo_time = 1.0;
    // Initialize the full state covariance matrix as identity scaled by INIT_COV.
    this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
    // Set a very small initial covariance for the inverse exposure time state.
    this->cov(6, 6) = 0.00001;
    // Set a very small initial covariance block for the bias and gravity states.
    this->cov.block<9, 9>(10, 10) = MD(9, 9)::Identity() * 0.00001;
  };

  // Copy constructor: deep-copy all state members from an existing StatesGroup.
  StatesGroup(const StatesGroup &b)
  {
    // Copy the attitude rotation matrix from the source.
    this->rot_end = b.rot_end;
    // Copy the position from the source.
    this->pos_end = b.pos_end;
    // Copy the velocity from the source.
    this->vel_end = b.vel_end;
    // Copy the gyroscope bias from the source.
    this->bias_g = b.bias_g;
    // Copy the accelerometer bias from the source.
    this->bias_a = b.bias_a;
    // Copy the gravity vector from the source.
    this->gravity = b.gravity;
    // Copy the inverse exposure time from the source.
    this->inv_expo_time = b.inv_expo_time;
    // Copy the full state covariance matrix from the source.
    this->cov = b.cov;
  };

  // Assignment operator: copy all state members from an existing StatesGroup.
  StatesGroup &operator=(const StatesGroup &b)
  {
    // Assign the attitude rotation matrix from the source.
    this->rot_end = b.rot_end;
    // Assign the position from the source.
    this->pos_end = b.pos_end;
    // Assign the velocity from the source.
    this->vel_end = b.vel_end;
    // Assign the gyroscope bias from the source.
    this->bias_g = b.bias_g;
    // Assign the accelerometer bias from the source.
    this->bias_a = b.bias_a;
    // Assign the gravity vector from the source.
    this->gravity = b.gravity;
    // Assign the inverse exposure time from the source.
    this->inv_expo_time = b.inv_expo_time;
    // Assign the full state covariance matrix from the source.
    this->cov = b.cov;
    // Return a reference to this object to support chained assignment.
    return *this;
  };

  // Addition operator: compose a state increment onto the current EKF state on SO(3) x R^16.
  StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    // Create a temporary StatesGroup to hold the resulting state.
    StatesGroup a;
    // Update the rotation by applying the incremental rotation via the SO(3) exponential map.
    a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    // Update the position by adding the position increment.
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    // Update the inverse exposure time by adding its increment.
    a.inv_expo_time = this->inv_expo_time + state_add(6, 0);
    // Update the velocity by adding the velocity increment.
    a.vel_end = this->vel_end + state_add.block<3, 1>(7, 0);
    // Update the gyroscope bias by adding the bias increment.
    a.bias_g = this->bias_g + state_add.block<3, 1>(10, 0);
    // Update the accelerometer bias by adding the bias increment.
    a.bias_a = this->bias_a + state_add.block<3, 1>(13, 0);
    // Update the gravity vector by adding the gravity increment.
    a.gravity = this->gravity + state_add.block<3, 1>(16, 0);
    // Copy the current state covariance to the result (covariance is not updated by addition).
    a.cov = this->cov;
    // Return the resulting composite state.
    return a;
  };

  // Compound addition operator: apply a state increment in-place.
  StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    // Compose the incremental rotation onto the current attitude via the SO(3) exponential map.
    this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    // Add the position increment to the current position.
    this->pos_end += state_add.block<3, 1>(3, 0);
    // Add the inverse exposure time increment.
    this->inv_expo_time += state_add(6, 0);
    // Add the velocity increment to the current velocity.
    this->vel_end += state_add.block<3, 1>(7, 0);
    // Add the gyroscope bias increment.
    this->bias_g += state_add.block<3, 1>(10, 0);
    // Add the accelerometer bias increment.
    this->bias_a += state_add.block<3, 1>(13, 0);
    // Add the gravity increment.
    this->gravity += state_add.block<3, 1>(16, 0);
    // Return a reference to this object for chaining.
    return *this;
  };

  // Subtraction operator: compute the state difference vector between two StatesGroup instances.
  Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b)
  {
    // Declare a DIM_STATE-dimensional vector to hold the difference.
    Matrix<double, DIM_STATE, 1> a;
    // Compute the relative rotation matrix from b to this.
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    // Extract the rotation difference via the SO(3) logarithmic map.
    a.block<3, 1>(0, 0) = Log(rotd);
    // Compute the position difference.
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    // Compute the inverse exposure time difference.
    a(6, 0) = this->inv_expo_time - b.inv_expo_time;
    // Compute the velocity difference.
    a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;
    // Compute the gyroscope bias difference.
    a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
    // Compute the accelerometer bias difference.
    a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
    // Compute the gravity vector difference.
    a.block<3, 1>(16, 0) = this->gravity - b.gravity;
    // Return the full state difference vector.
    return a;
  };

  // Reset the pose-related states (rotation, position, velocity) to identity and zero.
  void resetpose()
  {
    // Reset the attitude rotation matrix to identity (no rotation).
    this->rot_end = M3D::Identity();
    // Reset the position to the world origin.
    this->pos_end = V3D::Zero();
    // Reset the velocity to zero.
    this->vel_end = V3D::Zero();
  }

  // The estimated attitude (rotation matrix) at the end of the LiDAR scan (world frame).
  M3D rot_end;                              // the estimated attitude (rotation matrix) at the end lidar point
  // The estimated position at the end of the LiDAR scan (world frame).
  V3D pos_end;                              // the estimated position at the end lidar point (world frame)
  // The estimated velocity at the end of the LiDAR scan (world frame).
  V3D vel_end;                              // the estimated velocity at the end lidar point (world frame)
  // The estimated inverse exposure time for the camera (dimensionless scale).
  double inv_expo_time;                     // the estimated inverse exposure time (no scale)
  // The estimated gyroscope bias vector (body frame).
  V3D bias_g;                               // gyroscope bias
  // The estimated accelerometer bias vector (body frame).
  V3D bias_a;                               // accelerator bias
  // The estimated gravity vector (world frame).
  V3D gravity;                              // the estimated gravity acceleration
  // The full 19x19 EKF state covariance matrix.
  Matrix<double, DIM_STATE, DIM_STATE> cov; // states covariance
};

// Populate a Pose6D struct from acceleration, gyroscope, velocity, position, and rotation.
template <typename T>
// Build a Pose6D struct from the provided time, acceleration, gyroscope, velocity, position, and rotation matrix.
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p,
                const Matrix<T, 3, 3> &R)
{
  // Create a local Pose6D structure to hold the output.
  Pose6D rot_kp;
  // Assign the time offset to the Pose6D structure.
  rot_kp.offset_time = t;
  // Loop over the three spatial dimensions (x, y, z).
  for (int i = 0; i < 3; i++)
  {
    // Copy the i-th acceleration component from the Eigen vector to the C array.
    rot_kp.acc[i] = a(i);
    // Copy the i-th gyroscope component from the Eigen vector to the C array.
    rot_kp.gyr[i] = g(i);
    // Copy the i-th velocity component from the Eigen vector to the C array.
    rot_kp.vel[i] = v(i);
    // Copy the i-th position component from the Eigen vector to the C array.
    rot_kp.pos[i] = p(i);
    // Loop over the three columns of the rotation matrix.
    for (int j = 0; j < 3; j++)
      // Copy the (i, j) rotation matrix element into the flat C array in row-major order.
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  // Map<M3D>(rot_kp.rot, 3,3) = R;
  // Return the populated Pose6D structure, using move semantics for efficiency.
  return move(rot_kp);
}

// End of the common library header include guard.
#endif
