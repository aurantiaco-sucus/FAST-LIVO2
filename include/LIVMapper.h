/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Header guard to prevent multiple inclusions of this file
#ifndef LIV_MAPPER_H
// Define the header guard macro
#define LIV_MAPPER_H

// Include IMU processing header for initialization, forward propagation, and undistortion
#include "IMU_Processing.h"
// Include visual-inertial odometry manager header
#include "vio.h"
// Include LiDAR point cloud preprocessing header for feature extraction
#include "preprocess.h"
// Include cv_bridge for converting between ROS Image messages and OpenCV images
#include <cv_bridge/cv_bridge.h>
// Include image_transport for publishing and subscribing to ROS image topics
#include <image_transport/image_transport.h>
// Include nav_msgs/Path message type for publishing the estimated trajectory
#include <nav_msgs/Path.h>
// Include vikit camera loader for loading camera calibration from ROS parameters
#include <vikit/camera_loader.h>

// Central orchestrator for the FAST-LIVO2 system. Manages sensor data ingestion,
// synchronization, IMU propagation, LiDAR-inertial odometry, visual-inertial odometry,
// state estimation, map building, and ROS publishing.
class LIVMapper
{
public:
  // Constructor: initializes the mapper with a ROS node handle
  LIVMapper(ros::NodeHandle &nh);
  // Destructor: cleans up resources
  ~LIVMapper();
  // Create all ROS subscribers and publishers for sensor topics and output topics
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  // Initialize processing components (preprocessor, IMU processor, voxel map, VIO manager)
  void initializeComponents();
  // Open log files for recording odometry, visual positions, and point statistics
  void initializeFiles();
  // Main loop: synchronizes measurements and runs state estimation and mapping
  void run();
  // Perform gravity alignment using IMU acceleration measurements
  void gravityAlignment();
  // Handle initialization of the first camera frame
  void handleFirstFrame();
  // Core estimator: runs LIO and VIO updates on synchronized measurements
  void stateEstimationAndMapping();
  // Execute visual-inertial odometry update (feature tracking, EKF update)
  void handleVIO();
  // Execute LiDAR-inertial odometry update (voxel scan-to-map matching)
  void handleLIO();
  // Save the accumulated point cloud map to a PCD file
  void savePCD();
  // Process the buffered IMU measurements
  void processImu();
  
  // Synchronize incoming LiDAR, IMU, and image measurements into a single package
  bool sync_packages(LidarMeasureGroup &meas);
  // Perform one step of IMU state propagation using gyroscope and accelerometer readings
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  // Timer callback that periodically propagates the IMU state
  void imu_prop_callback(const ros::TimerEvent &e);
  // Apply a rigid rotation and translation to transform a point cloud
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  // Transform a single point from the body frame to the world frame
  void pointBodyToWorld(const PointType &pi, PointType &po);
  // Transform a point from the LiDAR frame to the IMU frame using the extrinsic calibration
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  // Transform a colored point from the body frame to the world frame
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  // Callback for standard ROS PointCloud2 messages (Velodyne, Ouster, etc.)
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Callback for Livox custom ROS messages
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  // Callback for IMU sensor messages
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  // Callback for camera image messages
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  // Publish the RGB image with tracked visual features overlaid
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  // Publish the full-resolution LiDAR point cloud transformed to the world frame
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  // Publish the visual sub-map point cloud
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  // Publish the point cloud colored by plane-fitting residual
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  // Publish the estimated odometry as an Odometry message
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  // Publish the current pose to MAVROS for drone control
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  // Publish the estimated trajectory as a Path message
  void publish_path(const ros::Publisher pubPath);
  // Read all configuration parameters from the ROS parameter server
  void readParameters(ros::NodeHandle &nh);
  // Set the position and orientation of a PoseStamped or Odometry message from the current state
  template <typename T> void set_posestamp(T &out);
  // Template version of pointBodyToWorld for generic scalar types (float/double)
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  // Template version of pointBodyToWorld that returns the transformed point
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  // Extract a cv::Mat image from a ROS Image message (handles various encodings)
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  // Mutexes for protecting the sensor data buffers and the IMU propagation buffer
  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  // Condition variable for signaling when new sensor data is available
  std::condition_variable sig_buffer;

  // Current SLAM operating mode (ONLY_LO, ONLY_LIO, or LIVO)
  SLAM_MODE slam_mode_;
  // Hash map from voxel grid coordinates to octree nodes for the LIO map
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  // Root directory for saving log files and PCD maps
  string root_dir;
  // ROS topic names for LiDAR, IMU, sequence name, and camera image
  string lid_topic, imu_topic, seq_name, img_topic;
  // Translation from LiDAR frame to IMU frame (extrinsic calibration)
  V3D extT;
  // Rotation from LiDAR frame to IMU frame (extrinsic calibration)
  M3D extR;

  // Number of downsampled surface feature points and maximum ICP iterations
  int feats_down_size = 0, max_iterations = 0;

  // Mean residual from the last LIO scan-to-map optimization
  double res_mean_last = 0.05;
  // Covariance values for gyroscope, accelerometer, and inverse exposure bias
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  // Distance threshold for initializing visual points from RGB LiDAR returns
  double blind_rgb_points = 0.0;
  // Timestamps of the most recently processed LiDAR scan, IMU message, and image frame
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  // Minimum filter size for downsampling surface points
  double filter_size_surf_min = 0;
  // Voxel filter leaf size for the global point cloud map
  double filter_size_pcd = 0;
  // Timestamp of the first LiDAR scan received
  double _first_lidar_time = 0.0;
  // Timing statistics: data association time, solver time, constant Hessian construction time
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  // Boolean flags for map initialization, PCD saving, image saving, effect point publishing, pose output, ROS driver fix, and HILTI mode
  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  // Image save interval (frames) and PCD save interval (frames) and PCD save type (0=RGB, 1=intensity)
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  // Number of past scans to publish at once
  int pub_scan_num = 1;

  // IMU-propagated state and the latest EKF state after update
  StatesGroup imu_propagate, latest_ekf_state;

  // Flags indicating new IMU data arrival, state update completion, IMU propagation enabled, and first EKF completion
  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  // Deque of IMU messages used for state propagation between LiDAR scans
  deque<sensor_msgs::Imu> prop_imu_buffer;
  // The most recently received IMU message
  sensor_msgs::Imu newest_imu;
  // Timestamp of the most recent EKF state update
  double latest_ekf_time;
  // Odometry message for publishing the IMU-propagated state
  nav_msgs::Odometry imu_prop_odom;
  // Publisher for the IMU-propagated odometry
  ros::Publisher pubImuPropOdom;
  // Time offset between the system clock and IMU timestamps
  double imu_time_offset = 0.0;
  // Time offset between the system clock and LiDAR timestamps
  double lidar_time_offset = 0.0;

  // Flags controlling gravity alignment: enabled and finished status
  bool gravity_align_en = false, gravity_align_finished = false;

  // Flag indicating a jump in sensor timestamps during synchronization
  bool sync_jump_flag = false;

  // Flags for LiDAR push status, IMU enabled, gravity estimation enabled, reset flag, and bias estimation enabled
  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  // Flag for enabling dense (full-resolution) point cloud map generation
  bool dense_map_en = false;
  // Image enable flag (0=disabled, 1=enabled) and number of frames for IMU initialization
  int img_en = 1, imu_int_frame = 3;
  // Flag for computing surface normals on the point cloud
  bool normal_en = true;
  // Flag for estimating camera exposure time from IMU data
  bool exposure_estimate_en = false;
  // Initial exposure time value for the camera
  double exposure_time_init = 0.0;
  // Flag for using inverse compositional formulation in visual alignment
  bool inverse_composition_en = false;
  // Flag for enabling ray casting for visibility checks
  bool raycast_en = false;
  // LiDAR enable flag (0=disabled, 1=enabled)
  int lidar_en = 1;
  // Flag indicating the first frame has been processed
  bool is_first_frame = false;
  // Grid and patch parameters: grid cell size, patch size, grid dimensions, and pyramid level count
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  // Outlier rejection threshold for visual feature matching
  double outlier_threshold;
  // Time spent plotting or visualization
  double plot_time;
  // Frame counter for logging and save intervals
  int frame_cnt;
  // Time offset between the system clock and image timestamps
  double img_time_offset = 0.0;
  // Deque of raw LiDAR point cloud shared pointers for buffering
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  // Deque of LiDAR header timestamps for synchronization
  deque<double> lid_header_time_buffer;
  // Deque of buffered IMU messages as shared pointers
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  // Deque of buffered OpenCV images
  deque<cv::Mat> img_buffer;
  // Deque of buffered image timestamps
  deque<double> img_time_buffer;
  // Vector of point-with-variance structures for LIO residual computation
  vector<pointWithVar> _pv_list;
  // LiDAR-to-IMU extrinsic translation as a vector of doubles
  vector<double> extrinT;
  // LiDAR-to-IMU extrinsic rotation as a vector of doubles
  vector<double> extrinR;
  // Camera-to-IMU extrinsic translation as a vector of doubles
  vector<double> cameraextrinT;
  // Camera-to-IMU extrinsic rotation as a vector of doubles
  vector<double> cameraextrinR;
  // Covariance for visual feature point observations
  double IMG_POINT_COV;

  // Point cloud for publishing the visual sub-map
  PointCloudXYZI::Ptr visual_sub_map;
  // Point cloud of undistorted LiDAR features
  PointCloudXYZI::Ptr feats_undistort;
  // Downsampled surface features in the body frame
  PointCloudXYZI::Ptr feats_down_body;
  // Downsampled surface features transformed to the world frame
  PointCloudXYZI::Ptr feats_down_world;
  // World-frame point cloud waiting to be published
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  // Body-frame point cloud waiting to be published
  PointCloudXYZI::Ptr pcl_wait_pub;
  // RGB point cloud waiting to be saved to a PCD file
  PointCloudXYZRGB::Ptr pcl_wait_save;
  // Intensity point cloud waiting to be saved to a PCD file
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  // Output file streams for logging pre-integration data, odometry, visual positions, LiDAR positions, and point statistics
  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

  // PCL voxel grid filter for downsampling surface points
  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  // Current Euler angles (roll, pitch, yaw) of the estimated pose
  V3D euler_cur;

  // Measurement group containing the latest synchronized LiDAR data
  LidarMeasureGroup LidarMeasures;
  // Current EKF state (position, velocity, orientation, biases)
  StatesGroup _state;
  // Propagated state from IMU forward propagation
  StatesGroup  state_propagat;

  // Path message for accumulating and publishing the trajectory
  nav_msgs::Path path;
  // Odometry message for publishing the latest estimated pose
  nav_msgs::Odometry odomAftMapped;
  // Quaternion for the latest estimated orientation
  geometry_msgs::Quaternion geoQuat;
  // PoseStamped message for the current body pose in the trajectory path
  geometry_msgs::PoseStamped msg_body_pose;

  // Shared pointer to the LiDAR point cloud preprocessor
  PreprocessPtr p_pre;
  // Shared pointer to the IMU processor for initialization and forward propagation
  ImuProcessPtr p_imu;
  // Shared pointer to the voxel octree map manager for LIO
  VoxelMapManagerPtr voxelmap_manager;
  // Shared pointer to the visual-inertial odometry manager
  VIOManagerPtr vio_manager;

  // Publisher for plane visualization markers
  ros::Publisher plane_pub;
  // Publisher for voxel visualization markers
  ros::Publisher voxel_pub;
  // Subscriber for the LiDAR point cloud topic
  ros::Subscriber sub_pcl;
  // Subscriber for the IMU topic
  ros::Subscriber sub_imu;
  // Subscriber for the camera image topic
  ros::Subscriber sub_img;
  // Publisher for the full-resolution world-frame point cloud
  ros::Publisher pubLaserCloudFullRes;
  // Publisher for surface normal visualization
  ros::Publisher pubNormal;
  // Publisher for the visual sub-map point cloud
  ros::Publisher pubSubVisualMap;
  // Publisher for the residual-colored effect point cloud
  ros::Publisher pubLaserCloudEffect;
  // Publisher for the entire LiDAR map point cloud
  ros::Publisher pubLaserCloudMap;
  // Publisher for the odometry after mapping
  ros::Publisher pubOdomAftMapped;
  // Publisher for the estimated trajectory path
  ros::Publisher pubPath;
  // Publisher for dynamic objects point cloud
  ros::Publisher pubLaserCloudDyn;
  // Publisher for dynamic objects point cloud with removed outliers
  ros::Publisher pubLaserCloudDynRmed;
  // Publisher for debugging the dynamic objects point cloud
  ros::Publisher pubLaserCloudDynDbg;
  // Publisher for the RGB image with visual feature overlay
  image_transport::Publisher pubImage;
  // Publisher for sending the pose to MAVROS
  ros::Publisher mavros_pose_publisher;
  // Timer for periodic IMU state propagation
  ros::Timer imu_prop_timer;

  // Number of processed frames
  int frame_num = 0;
  // Average total processing time per frame
  double aver_time_consu = 0;
  // Average ICP matching time per frame
  double aver_time_icp = 0;
  // Average time for map incremental update per frame
  double aver_time_map_inre = 0;
  // Flag for enabling COLMAP output format
  bool colmap_output_en = false;
};
#endif