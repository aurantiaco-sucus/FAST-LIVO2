/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Include the LIVMapper class header
#include "LIVMapper.h"

// Constructor: initializes extrinsic parameters, preprocessing, IMU processing,
// voxel map, VIO manager, and supporting data structures
LIVMapper::LIVMapper(ros::NodeHandle &nh)
    // Initialize lidar-to-IMU extrinsic translation to zero
    : extT(0, 0, 0),
      // Initialize lidar-to-IMU extrinsic rotation to identity
      extR(M3D::Identity())
{
  // Allocate a 3-element vector for extrinsic translation parameters
  extrinT.assign(3, 0.0);
  // Allocate a 9-element vector for extrinsic rotation parameters
  extrinR.assign(9, 0.0);
  // Allocate a 3-element vector for camera extrinsic translation
  cameraextrinT.assign(3, 0.0);
  // Allocate a 9-element vector for camera extrinsic rotation
  cameraextrinR.assign(9, 0.0);

  // Create the LiDAR point cloud preprocessor
  p_pre.reset(new Preprocess());
  // Create the IMU preintegration and processing module
  p_imu.reset(new ImuProcess());

  // Load all configurable parameters from the ROS parameter server
  readParameters(nh);
  // Create a voxel map configuration struct
  VoxelMapConfig voxel_config;
  // Load voxel map-specific settings from the ROS parameter server
  loadVoxelConfig(nh, voxel_config);

  // Allocate point cloud for the visual sub-map
  visual_sub_map.reset(new PointCloudXYZI());
  // Allocate point cloud for undistorted LiDAR features
  feats_undistort.reset(new PointCloudXYZI());
  // Allocate point cloud for downsampled body-frame features
  feats_down_body.reset(new PointCloudXYZI());
  // Allocate point cloud for downsampled world-frame features
  feats_down_world.reset(new PointCloudXYZI());
  // Allocate point cloud for world-frame points waiting to be published
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  // Allocate point cloud for points waiting to be published (non-RGB)
  pcl_wait_pub.reset(new PointCloudXYZI());
  // Allocate point cloud for RGB-colored points to be saved to disk
  pcl_wait_save.reset(new PointCloudXYZRGB());
  // Allocate point cloud for intensity-only points to be saved to disk
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  // Create the voxel map manager with the loaded configuration and map reference
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  // Create the visual-inertial odometry manager
  vio_manager.reset(new VIOManager());
  // Store the project root directory path
  root_dir = ROOT_DIR;
  // Initialize output files for logging and PCD saving
  initializeFiles();
  // Initialize all processing components with loaded parameters
  initializeComponents();
  // Stamp the path message with the current ROS time
  path.header.stamp = ros::Time::now();
  // Set the path message frame ID to the odometry reference frame
  path.header.frame_id = "camera_init";
}

// Destructor: default, no custom cleanup needed
LIVMapper::~LIVMapper() {}

// Load all configurable parameters from the ROS parameter server into member variables
void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  // Read the LiDAR topic name (default: /livox/lidar)
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  // Read the IMU topic name (default: /livox/imu)
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  // Read the ROS driver bug fix flag
  nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
  // Read the image enable flag (1 = enabled)
  nh.param<int>("common/img_en", img_en, 1);
  // Read the LiDAR enable flag (1 = enabled)
  nh.param<int>("common/lidar_en", lidar_en, 1);
  // Read the image topic name
  nh.param<string>("common/img_topic", img_topic, "/left_camera/image");

  // Read the normal VIO optimization flag
  nh.param<bool>("vio/normal_en", normal_en, true);
  // Read the inverse compositional VIO flag
  nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
  // Read the maximum number of VIO iterations
  nh.param<int>("vio/max_iterations", max_iterations, 5);
  // Read the image point covariance value
  nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 100);
  // Read the raycast enable flag for visual initialization
  nh.param<bool>("vio/raycast_en", raycast_en, false);
  // Read the exposure time estimation enable flag
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  // Read the inverse exposure time covariance
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
  // Read the grid size for feature distribution
  nh.param<int>("vio/grid_size", grid_size, 5);
  // Read the grid height (number of vertical cells)
  nh.param<int>("vio/grid_n_height", grid_n_height, 17);
  // Read the number of pyramid levels for patch alignment
  nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  // Read the image patch size for tracking
  nh.param<int>("vio/patch_size", patch_size, 8);
  // Read the outlier rejection threshold
  nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);

  // Read the initial exposure time offset
  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  // Read the image timestamp offset
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  // Read the IMU timestamp offset
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  // Read the LiDAR timestamp offset
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  // Read the IMU-rate odometry propagation enable flag
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  // Read the gravity alignment enable flag
  nh.param<bool>("uav/gravity_align_en", gravity_align_en, false);

  // Read the sequence name for evo evaluation output
  nh.param<string>("evo/seq_name", seq_name, "01");
  // Read the pose output enable flag for evo
  nh.param<bool>("evo/pose_output_en", pose_output_en, false);
  // Read the gyroscope measurement covariance
  nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
  // Read the accelerometer measurement covariance
  nh.param<double>("imu/acc_cov", acc_cov, 1.0);
  // Read the number of IMU initialization frames
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  // Read the IMU enable flag
  nh.param<bool>("imu/imu_en", imu_en, false);
  // Read the gravity vector estimation enable flag
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  // Read the accelerometer/gyroscope bias estimation enable flag
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);

  // Read the blind region range (points closer than this are ignored)
  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  // Read the surface voxel filter leaf size
  nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  // Read the HILTI dataset compatibility flag
  nh.param<bool>("preprocess/hilti_en", hilti_en, false);
  // Read the LiDAR sensor type (e.g., AVIA, VELO16)
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  // Read the number of LiDAR scan lines
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  // Read the point filter stride (keep every Nth point)
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  // Read the feature extraction enable flag
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  // Read the PCD save interval (negative = save only at end)
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  // Read the PCD save enable flag
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  // Read the PCD save type (0 = world frame, 1 = body frame)
  nh.param<int>("pcd_save/type", pcd_save_type, 0);
  // Read the image save enable flag
  nh.param<bool>("image_save/img_save_en", img_save_en, false);
  // Read the image save interval
  nh.param<int>("image_save/interval", img_save_interval, 1);

  // Read the COLMAP output enable flag for SfM compatibility
  nh.param<bool>("pcd_save/colmap_output_en", colmap_output_en, false);
  // Read the downsampling filter size for PCD output
  nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
  // Read the extrinsic translation vector (LiDAR to IMU)
  nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  // Read the extrinsic rotation matrix (LiDAR to IMU)
  nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
  // Read the camera extrinsic translation (camera to LiDAR)
  nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
  // Read the camera extrinsic rotation (camera to LiDAR)
  nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());
  // Read the debug plot timestamp threshold
  nh.param<double>("debug/plot_time", plot_time, -10);
  // Read the debug frame count threshold
  nh.param<int>("debug/frame_cnt", frame_cnt, 6);

  // Read the blind distance for RGB point publishing
  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
  // Read the number of scans to accumulate before publishing
  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  // Read the effective point publishing enable flag
  nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
  // Read the dense map publishing enable flag
  nh.param<bool>("publish/dense_map_en", dense_map_en, false);

  // Precompute the squared blind distance for fast comparison
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

// Initialize all processing components using the parameters loaded from the ROS server
void LIVMapper::initializeComponents() 
{
  // Set the leaf size of the surface downsampling voxel filter
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  // Load the extrinsic translation vector from the parameter array into the Eigen type
  extT << VEC_FROM_ARRAY(extrinT);
  // Load the extrinsic rotation matrix from the parameter array into the Eigen type
  extR << MAT_FROM_ARRAY(extrinR);

  // Set the voxel map manager's extrinsic translation from loaded parameters
  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  // Set the voxel map manager's extrinsic rotation from loaded parameters
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  // Load the camera model from the ROS namespace; throw if not found
  if (!vk::camera_loader::loadFromRosNs("laserMapping", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  // Pass the feature grid size to the VIO manager
  vio_manager->grid_size = grid_size;
  // Pass the patch size for image alignment to the VIO manager
  vio_manager->patch_size = patch_size;
  // Pass the outlier rejection threshold to the VIO manager
  vio_manager->outlier_threshold = outlier_threshold;
  // Set the IMU-to-LiDAR extrinsic calibration in the VIO manager
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  // Set the LiDAR-to-camera extrinsic calibration in the VIO manager
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  // Provide a pointer to the main EKF state for VIO updates
  vio_manager->state = &_state;
  // Provide a pointer to the propagated state for VIO use
  vio_manager->state_propagat = &state_propagat;
  // Set the maximum VIO optimization iterations
  vio_manager->max_iterations = max_iterations;
  // Set the image point measurement covariance
  vio_manager->img_point_cov = IMG_POINT_COV;
  // Enable or disable the normal VIO formulation
  vio_manager->normal_en = normal_en;
  // Enable or disable the inverse compositional VIO formulation
  vio_manager->inverse_composition_en = inverse_composition_en;
  // Enable or disable raycasting for visual initialization
  vio_manager->raycast_en = raycast_en;
  // Set the grid width (number of horizontal cells) for feature distribution
  vio_manager->grid_n_width = grid_n_width;
  // Set the grid height (number of vertical cells) for feature distribution
  vio_manager->grid_n_height = grid_n_height;
  // Set the number of pyramid levels for multi-resolution patch alignment
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  // Enable or disable exposure time estimation in VIO
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  // Enable or disable COLMAP-compatible output mode
  vio_manager->colmap_output_en = colmap_output_en;
  // Finalize VIO initialization (allocate internal structures)
  vio_manager->initializeVIO();

  // Set the IMU-to-LiDAR extrinsic in the IMU processor
  p_imu->set_extrinsic(extT, extR);
  // Set the gyroscope noise covariance scale factor
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  // Set the accelerometer noise covariance scale factor
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  // Set the inverse exposure time process noise covariance
  p_imu->set_inv_expo_cov(inv_expo_cov);
  // Set the gyroscope bias random walk covariance
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  // Set the accelerometer bias random walk covariance
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  // Set the number of IMU frames used for static initialization
  p_imu->set_imu_init_frame_num(imu_int_frame);

  // Disable IMU processing if IMU is not enabled in config
  if (!imu_en) p_imu->disable_imu();
  // Disable gravity estimation if not enabled in config
  if (!gravity_est_en) p_imu->disable_gravity_est();
  // Disable bias estimation if not enabled in config
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  // Disable exposure time estimation if not enabled in config
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  // Determine the SLAM mode: LIVO (full), ONLY_LIO, or ONLY_LO
  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

// Initialize file logging and PCD output directories
void LIVMapper::initializeFiles() 
{
  // If both PCD save and COLMAP output are enabled, run the COLMAP setup script
  if (pcd_save_en && colmap_output_en)
  {
      // Construct the path to the COLMAP output shell script
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      // Build the chmod command to make the script executable
      std::string chmodCommand = "chmod +x " + folderPath;
      
      // Execute the chmod command to set executable permissions
      int chmodRet = system(chmodCommand.c_str());  
      // Check if chmod succeeded
      if (chmodRet != 0) {
          // Print error if permission setting failed
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      // Execute the COLMAP output setup script
      int executionRet = system(folderPath.c_str());
      // Check if script execution succeeded
      if (executionRet != 0) {
          // Print error if script execution failed
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  // Open points3D.txt for COLMAP output if enabled
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  // Open lidar_poses.txt for writing LiDAR trajectory if PCD saving is enabled
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  // Open image_poses.txt for writing camera trajectory if image saving is enabled
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  // Open mat_pre.txt for recording pre-update state estimates
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  // Open mat_out.txt for recording post-update state estimates
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

// Set up all ROS subscribers and publishers for the SLAM system
void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it) 
{
  // Subscribe to LiDAR data: use Livox callback for AVIA, standard callback otherwise
  sub_pcl = p_pre->lidar_type == AVIA ? 
            nh.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this): 
            nh.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
  // Subscribe to the IMU data topic
  sub_imu = nh.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);
  // Subscribe to the image data topic
  sub_img = nh.subscribe(img_topic, 200000, &LIVMapper::img_cbk, this);
  
  // Publisher for the full-resolution registered point cloud
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  // Publisher for surface normal visualization markers
  pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);
  // Publisher for the visual sub-map point cloud before VIO update
  pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 100);
  // Publisher for the effective (inlier) point cloud after LIO matching
  pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  // Publisher for the global LiDAR map
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);
  // Publisher for the odometry after mapping (world-frame pose)
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  // Publisher for the full pose trajectory path
  pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
  // Publisher for plane normal markers (planner visualization)
  plane_pub = nh.advertise<visualization_msgs::Marker>("/planner_normal", 1);
  // Publisher for voxel grid visualization markers
  voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);
  // Publisher for dynamic object point cloud
  pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100);
  // Publisher for point cloud after dynamic object removal
  pubLaserCloudDynRmed = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_removed", 100);
  // Publisher for dynamic object debug history
  pubLaserCloudDynDbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 100);
  // Publisher for MAVROS vision pose (PX4 bridge)
  mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  // Publisher for the RGB image with overlaid features
  pubImage = it.advertise("/rgb_img", 1);
  // Publisher for IMU-only propagated odometry (high-rate)
  pubImuPropOdom = nh.advertise<nav_msgs::Odometry>("/LIVO2/imu_propagate", 10000);
  // Timer for IMU propagation callback at 250 Hz (4 ms period)
  imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);
  // Publisher for plane markers from the voxel map
  voxelmap_manager->voxel_map_pub_= nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
}

// Handle the very first LiDAR frame: record the initial timestamp
void LIVMapper::handleFirstFrame() 
{
  // Only execute on the very first frame
  if (!is_first_frame)
  {
    // Record the timestamp of the first LiDAR update
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    // Pass the first LiDAR time to the IMU processor for logging
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    // Set the flag to indicate the first frame has been handled
    is_first_frame = true;
    // Log that the first LiDAR frame has been received
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

// Perform gravity alignment to rotate the state into the world frame (z-up)
void LIVMapper::gravityAlignment() 
{
  // Run only if IMU is initialized and gravity alignment hasn't been done yet
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    // Log the start of gravity alignment
    std::cout << "Gravity Alignment Starts" << std::endl;
    // Define the target gravity direction (z-up in ENU convention)
    V3D ez(0, 0, -1), gz(_state.gravity);
    // Compute the rotation that aligns the estimated gravity to the world z-axis
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    // Convert the quaternion to a rotation matrix
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    // Apply the gravity alignment rotation to the current position
    _state.pos_end = G_R_I0 * _state.pos_end;
    // Apply the gravity alignment rotation to the current attitude
    _state.rot_end = G_R_I0 * _state.rot_end;
    // Apply the gravity alignment rotation to the current velocity
    _state.vel_end = G_R_I0 * _state.vel_end;
    // Rotate the gravity vector to the aligned frame
    _state.gravity = G_R_I0 * _state.gravity;
    // Mark gravity alignment as complete
    gravity_align_finished = true;
    // Log the completion of gravity alignment
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

// Process IMU measurements: forward propagation and LiDAR point undistortion
void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  // Run IMU forward propagation and undistort LiDAR points using IMU data
  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  // Perform gravity alignment if enabled in the configuration
  if (gravity_align_en) gravityAlignment();

  // Store the propagated state for later use in VIO/LIO updates
  state_propagat = _state;
  // Pass the current state to the voxel map manager
  voxelmap_manager->state_ = _state;
  // Pass the undistorted features to the voxel map manager
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

// Route to the appropriate estimation pipeline based on the current data mode
void LIVMapper::stateEstimationAndMapping() 
{
  // Select between VIO and LIO/LO processing based on the measurement flag
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      // Visual-inertial odometry update (camera + IMU)
      handleVIO();
      break;
    case LIO:
    case LO:
      // LiDAR-inertial or LiDAR-only odometry update
      handleLIO();
      break;
  }
}

// Handle the Visual-Inertial Odometry update step
void LIVMapper::handleVIO() 
{
  // Convert the current rotation matrix to Euler angles for logging
  euler_cur = RotMtoEuler(_state.rot_end);
  // Log the pre-update state estimate to the pre-file
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  // Check if there are any world-frame points available for visual tracking
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    // Log that no points are available and skip VIO processing
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  // Log the number of raw features available for the visual update
  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  // Set the debug plotting flag based on the current time relative to the plot_time threshold
  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    // Enable plotting when within the debug time window
    vio_manager->plot_flag = true;
  } 
  else 
  {
    // Disable plotting outside the debug time window
    vio_manager->plot_flag = false;
  }

  // Run the VIO frame processing: patch alignment, EKF update, map management
  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  // If IMU-rate odometry propagation is enabled, update the latest EKF state
  if (imu_prop_enable) 
  {
    // Mark that EKF has finished at least once
    ekf_finish_once = true;
    // Store the latest EKF state for high-rate propagation
    latest_ekf_state = _state;
    // Record the timestamp of the latest EKF update
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    // Signal that the state has been updated for the IMU propagation callback
    state_update_flg = true;
  }

  // Publish the world-frame registered point cloud with RGB colors
  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  // Publish the RGB image with tracked features overlaid
  publish_img_rgb(pubImage, vio_manager);

  // Recompute Euler angles from the updated state for post-update logging
  euler_cur = RotMtoEuler(_state.rot_end);
  // Log the post-update state estimate to the out-file
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

// Handle the LiDAR-Inertial Odometry update step
void LIVMapper::handleLIO() 
{    
  // Convert the current rotation to Euler angles for pre-update logging
  euler_cur = RotMtoEuler(_state.rot_end);
  // Log the pre-update state to the pre-file
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  // Check if there are undistorted LiDAR points to process
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    // Log that no LiDAR points are available and skip LIO processing
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  // Record the start time of the LIO processing pipeline
  double t0 = omp_get_wtime();

  // Apply the voxel downsampling filter to the undistorted features
  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  // Record the time after downsampling
  double t_down = omp_get_wtime();

  // Store the number of downsampled body-frame points
  feats_down_size = feats_down_body->points.size();
  // Pass the downsampled body-frame cloud to the voxel map manager
  voxelmap_manager->feats_down_body_ = feats_down_body;
  // Transform the downsampled points from body frame to world frame
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  // Pass the world-frame downsampled cloud to the voxel map manager
  voxelmap_manager->feats_down_world_ = feats_down_world;
  // Pass the downsampled size to the voxel map manager
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  // Build the initial voxel map on the very first LiDAR frame
  if (!lidar_map_inited) 
  {
    // Mark the LiDAR map as initialized
    lidar_map_inited = true;
    // Build the voxel map from the initial point cloud
    voxelmap_manager->BuildVoxelMap();
  }

  // Record the time after map initialization / data preparation
  double t1 = omp_get_wtime();

  // Run the LiDAR EKF state estimation (ICP-based point-to-plane)
  voxelmap_manager->StateEstimation(state_propagat);
  // Update the local state with the estimated state from the voxel map
  _state = voxelmap_manager->state_;
  // Store the point-to-plane list from the voxel map update
  _pv_list = voxelmap_manager->pv_list_;

  // Record the time after state estimation (ICP)
  double t2 = omp_get_wtime();

  // If IMU-rate propagation is enabled, record the latest EKF update
  if (imu_prop_enable) 
  {
    // Mark that EKF has completed at least once
    ekf_finish_once = true;
    // Store the latest EKF state for IMU propagation
    latest_ekf_state = _state;
    // Record the timestamp of this EKF update
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    // Signal the IMU propagation callback that a new state is available
    state_update_flg = true;
  }

  // If pose output for evo evaluation is enabled, write the pose to a file
  if (pose_output_en) 
  {
    // Static flag for first-time file open
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    // Open the file for writing on the first call, appending thereafter
    if (!pos_opend) 
    {
      // Create/open the sequence result file for writing
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      // Mark that the file has been opened
      pos_opend = true;
      // Check if the file opened successfully
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      // Open the sequence result file for appending
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      // Check if the file opened successfully
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    }
    // Declare a transformation matrix (unused placeholder)
    Eigen::Matrix4d outT;
    // Extract the quaternion from the current rotation matrix
    Eigen::Quaterniond q(_state.rot_end);
    // Set fixed-point notation for consistent output formatting
    evoFile << std::fixed;
    // Write the timestamp, position, and orientation to the evo file
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  // Convert the updated rotation to Euler angles for publishing
  euler_cur = RotMtoEuler(_state.rot_end);
  // Create a ROS quaternion message from the Euler angles
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  // Publish the odometry message
  publish_odometry(pubOdomAftMapped);

  // Record the time after odometry publishing
  double t3 = omp_get_wtime();

  // Create a point cloud for the world-frame transformed LiDAR points
  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  // Transform the downsampled body-frame points to the world frame
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  // Iterate over all transformed points to compute per-point covariance
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    // Set the world-frame position in the point-to-plane list
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    // Get the cross-product matrix for this point (used in covariance propagation)
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    // Get the body-frame covariance for this point
    M3D var = voxelmap_manager->body_cov_list_[i];
    // Propagate the covariance from the body frame to the world frame using the state covariance
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    // Store the propagated world-frame covariance
    voxelmap_manager->pv_list_[i].var = var;
  }
  // Update the voxel map with the registered point-to-plane associations
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  // Log the voxel map update
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  // Refresh the local point-to-plane list from the voxel map manager
  _pv_list = voxelmap_manager->pv_list_;
  
  // Record the time after voxel map update
  double t4 = omp_get_wtime();

  // If map sliding window is enabled, slide the map to keep only recent regions
  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  // Select the full-resolution or downsampled cloud based on dense_map_en config
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  // Get the size of the selected point cloud
  int size = laserCloudFullRes->points.size();
  // Allocate a world-frame point cloud with the same size
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  // Transform every point from the body frame to the world frame
  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  // Store the world-frame cloud for publishing
  *pcl_w_wait_pub = *laserCloudWorld;

  // Publish the registered world-frame point cloud
  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  // Publish effective (inlier) points if enabled
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  // Publish the voxel/plane map markers if enabled in config
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  // Publish the camera trajectory path
  publish_path(pubPath);
  // Publish the pose to MAVROS for UAV control
  publish_mavros(mavros_pose_publisher);

  // Increment the frame counter for averaging
  frame_num++;
  // Update the running average of total processing time
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);

  // Print a formatted table of LIO timing statistics
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  // Convert the final rotation to Euler angles for post-update logging
  euler_cur = RotMtoEuler(_state.rot_end);
  // Log the post-update state to the out-file
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

// Save the accumulated point clouds to PCD files at the end of the run
void LIVMapper::savePCD() 
{
  // Save only if PCD saving is enabled, there are points, and interval is negative (end-of-run save)
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    // Define file paths for raw and downsampled point clouds
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    // Create a PCD file writer
    pcl::PCDWriter pcd_writer;

    // If images are enabled, we have RGB point clouds
    if (img_en)
    {
      // Create a cloud for downsampled RGB points
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      // Create a voxel grid filter for downsampling
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      // Set the input cloud for the voxel filter
      voxel_filter.setInputCloud(pcl_wait_save);
      // Set the leaf size for the voxel filter
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      // Apply the voxel filter to produce the downsampled cloud
      voxel_filter.filter(*downsampled_cloud);
  
      // Save the raw (non-downsampled) RGB point cloud
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      // Log the save location and point count
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      // Save the downsampled RGB point cloud
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      // Log the save location and downsampled point count
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      // If COLMAP output is enabled, write the points3D.txt file
      if(colmap_output_en)
      {
        // Write the header for the COLMAP points3D file
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        // Iterate over all points in the downsampled cloud
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            // Get a reference to the current point
            const auto& point = downsampled_cloud->points[i];
            // Write the point ID, position, RGB color, and zero error to the COLMAP file
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      // Save the raw intensity-only point cloud if no image data
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      // Log the save location and point count
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

// Main SLAM processing loop: synchronizes data, processes IMU, and runs estimation
void LIVMapper::run() 
{
  // Set the spin rate to 5000 Hz for minimal latency in callback processing
  ros::Rate rate(5000);
  // Continue looping until ROS shuts down
  while (ros::ok()) 
  {
    // Process all pending ROS callbacks
    ros::spinOnce();
    // Synchronize LiDAR, IMU, and image data into a coherent measurement group
    if (!sync_packages(LidarMeasures)) 
    {
      // If not enough data is available, sleep and retry
      rate.sleep();
      continue;
    }
    // Handle the first LiDAR frame timestamp recording
    handleFirstFrame();

    // Process IMU data: forward propagation and LiDAR point undistortion
    processImu();

    // Run the appropriate state estimation and mapping (LIO or VIO)
    stateEstimationAndMapping();
  }
  // Save the accumulated point cloud data after the loop ends
  savePCD();
}

// Perform a single IMU propagation step using the given acceleration and angular velocity
void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  // Get the mean acceleration norm used for scale correction
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  // Correct the accelerometer reading by scale and subtract the bias
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  // Subtract the gyroscope bias from the angular velocity reading
  angvel_avr -= imu_prop_state.bias_g;

  // Compute the incremental rotation using the exponential map
  M3D Exp_f = Exp(angvel_avr, dt);
  // Propagate the IMU attitude by the incremental rotation
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  // Compute the specific acceleration in the global frame
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  // Propagate the position using the current velocity and acceleration (constant acceleration model)
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  // Propagate the velocity using the acceleration
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

// Timer callback for high-rate IMU-only odometry propagation (250 Hz)
void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{
  // Skip propagation if IMU not initialized, no new IMU data, or EKF never finished
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  // Lock the IMU propagation buffer mutex
  mtx_buffer_imu_prop.lock();
  // Reset the new IMU flag to match IMU data rate
  new_imu = false; // 控制propagate频率和IMU频率一致
  // Only proceed if IMU propagation is enabled and the buffer is not empty
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    // Static variable to track the last IMU time relative to the LiDAR end time
    static double last_t_from_lidar_end_time = 0;
    // If the EKF state was just updated by a LIO/VIO step, re-initialize propagation
    if (state_update_flg)
    {
      // Reset the propagation state to the latest EKF estimate
      imu_propagate = latest_ekf_state;
      // Drop all IMU packets that arrived before the latest EKF update time
      while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      // Reset the relative time tracker
      last_t_from_lidar_end_time = 0;
      // Process all remaining IMU packets up to the current time
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        // Compute the time of this IMU packet relative to the latest EKF update
        double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
        // Compute the time step between consecutive IMU packets
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // Extract the linear acceleration from the IMU packet
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        // Extract the angular velocity from the IMU packet
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        // Perform a single IMU propagation step
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        // Update the relative time tracker
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      // Clear the state update flag since propagation has been resynchronized
      state_update_flg = false;
    }
    else
    {
      // Extract the acceleration from the newest IMU packet
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      // Extract the angular velocity from the newest IMU packet
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      // Compute the time of the newest IMU packet relative to the latest EKF update
      double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
      // Compute the time step since the last propagation
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      // Perform a single IMU propagation step
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      // Update the relative time tracker
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    // Extract the propagated position and velocity
    V3D posi, vel_i;
    // Declare a quaternion for the orientation
    Eigen::Quaterniond q;
    // Get the propagated position
    posi = imu_propagate.pos_end;
    // Get the propagated velocity
    vel_i = imu_propagate.vel_end;
    // Convert the propagated rotation matrix to a quaternion
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    // Set the frame ID for the IMU propagation odometry message
    imu_prop_odom.header.frame_id = "world";
    // Set the timestamp to the newest IMU measurement time
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    // Set the propagated position in the odometry message
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    // Set the propagated orientation (quaternion) in the odometry message
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    // Set the propagated linear velocity in the odometry twist field
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    // Publish the IMU-propagated odometry message
    pubImuPropOdom.publish(imu_prop_odom);
  }
  // Unlock the IMU propagation buffer mutex
  mtx_buffer_imu_prop.unlock();
}

// Transform a point cloud from the body frame to the world frame using given rotation and translation
void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  // Clear the output cloud by swapping with an empty one
  PointCloudXYZI().swap(*trans_cloud);
  // Reserve memory for the output cloud to match the input size
  trans_cloud->reserve(input_cloud->size());
  // Iterate over every point in the input cloud
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    // Get the current point from the input cloud
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    // Extract the 3D position from the point
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    // Apply the LiDAR-to-IMU extrinsic, then the world rotation and translation
    p = (rot * (extR * p + extT) + t);
    // Create a new point with the transformed coordinates
    PointType pi;
    // Set the transformed x coordinate
    pi.x = p(0);
    // Set the transformed y coordinate
    pi.y = p(1);
    // Set the transformed z coordinate
    pi.z = p(2);
    // Preserve the original intensity value
    pi.intensity = p_c.intensity;
    // Add the transformed point to the output cloud
    trans_cloud->points.push_back(pi);
  }
}

// Transform a single LiDAR point from the body frame to the world frame (overloaded)
void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  // Extract the body-frame point position
  V3D p_body(pi.x, pi.y, pi.z);
  // Apply the extrinsic and state transformation to get the world-frame position
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  // Set the output point's x coordinate
  po.x = p_global(0);
  // Set the output point's y coordinate
  po.y = p_global(1);
  // Set the output point's z coordinate
  po.z = p_global(2);
  // Preserve the original intensity value
  po.intensity = pi.intensity;
}

// Template specialization: transform an Eigen vector from body to world frame
template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  // Extract the body-frame position from the input vector
  V3D p_body(pi[0], pi[1], pi[2]);
  // Apply the extrinsic and state transformation to get the world-frame position
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  // Set the output vector's x coordinate
  po[0] = p_global(0);
  // Set the output vector's y coordinate
  po[1] = p_global(1);
  // Set the output vector's z coordinate
  po[2] = p_global(2);
}

// Template specialization: transform and return an Eigen vector from body to world frame
template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  // Extract the body-frame position from the input vector
  V3D p(pi[0], pi[1], pi[2]);
  // Apply the extrinsic and state transformation to get the world-frame position
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  // Construct and return the output vector
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

// Transform a LiDAR point from the body frame to the world frame using pointer arguments
void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  // Extract the body-frame position from the input point
  V3D p_body(pi->x, pi->y, pi->z);
  // Apply the extrinsic and state transformation to get the world-frame position
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  // Set the output point's x coordinate
  po->x = p_global(0);
  // Set the output point's y coordinate
  po->y = p_global(1);
  // Set the output point's z coordinate
  po->z = p_global(2);
  // Preserve the original intensity value
  po->intensity = pi->intensity;
}

// Transform a LiDAR point from the body frame to the IMU frame (extrinsic only, no state)
void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  // Extract the LiDAR body-frame position
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  // Apply the extrinsic calibration to get the IMU-frame position
  V3D p_body_imu(extR * p_body_lidar + extT);

  // Set the output point's x coordinate
  po->x = p_body_imu(0);
  // Set the output point's y coordinate
  po->y = p_body_imu(1);
  // Set the output point's z coordinate
  po->z = p_body_imu(2);
  // Preserve the original intensity value
  po->intensity = pi->intensity;
}

// Callback for standard (non-Livox) LiDAR point cloud messages
void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Skip processing if LiDAR is disabled
  if (!lidar_en) return;
  // Lock the shared data buffer mutex
  mtx_buffer.lock();

  // Compute the corrected header time with the configured offset
  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // Detect and handle timestamp loop-back (bag loop) by clearing the buffer
  if (cur_head_time < last_timestamp_lidar)
  {
    // Log the loop-back event
    ROS_ERROR("lidar loop back, clear buffer");
    // Clear all buffered LiDAR data
    lid_raw_data_buffer.clear();
  }
  // cout<<"got feature"<<endl;
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  // Create a new point cloud and run preprocessing (feature extraction, filtering)
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  // Push the processed point cloud into the raw data buffer
  lid_raw_data_buffer.push_back(ptr);
  // Record the header timestamp for this LiDAR frame
  lid_header_time_buffer.push_back(cur_head_time);
  // Update the last LiDAR timestamp
  last_timestamp_lidar = cur_head_time;

  // Unlock the shared buffer mutex
  mtx_buffer.unlock();
  // Notify any waiting threads that new data is available
  sig_buffer.notify_all();
}

// Callback for Livox-format LiDAR point cloud messages (CustomMsg)
void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  // Skip processing if LiDAR is disabled
  if (!lidar_en) return;
  // Lock the shared data buffer mutex
  mtx_buffer.lock();
  // Deep-copy the incoming message to a mutable pointer
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  // Detect if the IMU and LiDAR timestamps have drifted apart by more than 1 second
  if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
  {
    // Compute and report the time difference between IMU and LiDAR
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
    // Print the detected hardware time lag (assuming a fixed 100 ms offset)
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  // Extract the header timestamp from the message
  double cur_head_time = msg->header.stamp.toSec();
  // Log the received LiDAR timestamp
  ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  // Detect and handle timestamp loop-back by clearing the buffer
  if (cur_head_time < last_timestamp_lidar)
  {
    // Log the loop-back event
    ROS_ERROR("lidar loop back, clear buffer");
    // Clear all buffered LiDAR data
    lid_raw_data_buffer.clear();
  }
  // Create a new point cloud and run preprocessing
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  // Validate that the processed point cloud is not null or empty
  if (!ptr || ptr->empty()) {
    // Log an error for empty point cloud
    ROS_ERROR("Received an empty point cloud");
    // Unlock the mutex before returning
    mtx_buffer.unlock();
    return;
  }

  // Push the processed point cloud into the raw data buffer
  lid_raw_data_buffer.push_back(ptr);
  // Record the header timestamp for this LiDAR frame
  lid_header_time_buffer.push_back(cur_head_time);
  // Update the last LiDAR timestamp
  last_timestamp_lidar = cur_head_time;

  // Unlock the shared buffer mutex
  mtx_buffer.unlock();
  // Notify any waiting threads that new data is available
  sig_buffer.notify_all();
}

// Callback for incoming IMU sensor messages
void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  // Skip processing if IMU is disabled
  if (!imu_en) return;

  // Ignore IMU messages received before the first LiDAR frame
  if (last_timestamp_lidar < 0.0) return;
  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  // Deep-copy the incoming IMU message
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  // Apply the IMU timestamp offset to the message header
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
  // Extract the corrected timestamp
  double timestamp = msg->header.stamp.toSec();

  // Warn if the IMU and LiDAR timestamps differ by more than 0.5 seconds
  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    // Log the synchronization warning
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  // If the ROS driver bug fix is enabled, adjust the IMU timestamp to match the LiDAR
  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  // Update the message header with the final corrected timestamp
  msg->header.stamp = ros::Time().fromSec(timestamp);

  // Lock the shared data buffer mutex
  mtx_buffer.lock();

  // Detect and handle IMU timestamp loop-back
  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    // Unlock before returning
    mtx_buffer.unlock();
    // Notify any waiting threads
    sig_buffer.notify_all();
    // Log the loop-back error
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // Update the last IMU timestamp
  last_timestamp_imu = timestamp;

  // Push the IMU message into the main IMU buffer
  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  // Unlock the shared buffer mutex
  mtx_buffer.unlock();
  // If IMU propagation is enabled, also buffer the message for high-rate odometry
  if (imu_prop_enable)
  {
    // Lock the IMU propagation buffer mutex
    mtx_buffer_imu_prop.lock();
    // Add the IMU message to the propagation buffer (only if IMU is initialized)
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    // Store the newest IMU message for the propagation callback
    newest_imu = *msg;
    // Signal that new IMU data is available for propagation
    new_imu = true;
    // Unlock the propagation buffer mutex
    mtx_buffer_imu_prop.unlock();
  }
  // Notify any waiting threads that new IMU data is available
  sig_buffer.notify_all();
}

// Convert a ROS image message to an OpenCV Mat (BGR8 format)
cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Declare the output OpenCV image matrix
  cv::Mat img;
  // Convert the ROS image to an OpenCV BGR image
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  // Return the converted image
  return img;
}

// Callback for incoming camera image messages
void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  // Skip processing if image input is disabled
  if (!img_en) return;
  // Deep-copy the incoming image message
  sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  // For HILTI 2022 datasets (40 Hz), only process every 4th frame to reduce to 10 Hz
  if (hilti_en)
  {
    // Static counter to track frames
    static int frame_counter = 0;
    // Skip 3 out of every 4 frames
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  // Apply the image timestamp offset to the message header
  double msg_header_time = msg->header.stamp.toSec() + img_time_offset;
  // Reject duplicate images with the same timestamp
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  // Log the received image timestamp
  ROS_INFO("Get image, its header time: %.6f", msg_header_time);
  // Ignore images received before the first LiDAR frame
  if (last_timestamp_lidar < 0) return;

  // Detect and handle image timestamp loop-back
  if (msg_header_time < last_timestamp_img)
  {
    // Log the loop-back error
    ROS_ERROR("image loop back. \n");
    return;
  }

  // Lock the shared data buffer mutex
  mtx_buffer.lock();

  // Store the corrected image timestamp
  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  // Reject images that are too close together (less than 20 ms apart)
  if (img_time_correct - last_timestamp_img < 0.02)
  {
    // Warn about the potential timestamp jump
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    // Unlock the mutex before returning
    mtx_buffer.unlock();
    // Notify waiting threads
    sig_buffer.notify_all();
    return;
  }

  // Convert the ROS image message to an OpenCV Mat
  cv::Mat img_cur = getImageFromMsg(msg);
  // Push the image into the image buffer for processing
  img_buffer.push_back(img_cur);
  // Record the corrected image timestamp
  img_time_buffer.push_back(img_time_correct);

  // Update the last image timestamp
  last_timestamp_img = img_time_correct;
  // ROS_INFO("Correct Image time: %.6f", img_time_correct);
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  // Unlock the shared buffer mutex
  mtx_buffer.unlock();
  // Notify any waiting threads that new image data is available
  sig_buffer.notify_all();
}

// Synchronize LiDAR, IMU, and camera data into a coherent measurement group based on the SLAM mode
bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  // Return false if LiDAR is enabled but no raw data is available
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  // Return false if images are enabled but no image data is available
  if (img_buffer.empty() && img_en) return false;
  // Return false if IMU is enabled but no IMU data is available
  if (imu_buffer.empty() && imu_en) return false;

  // Select the synchronization strategy based on the current SLAM mode
  switch (slam_mode_)
  {
  // LIO-only mode (LiDAR + IMU, no camera)
  case ONLY_LIO:
  {
    // Initialize the last LIO update time on the very first call
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    // If this is a new LiDAR scan, initialize the measurement structure
    if (!lidar_pushed)
    {
      // Push the first LiDAR topic into the measurement
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      // Reject scans with only one point (insufficient data)
      if (meas.lidar->points.size() <= 1) return false;

      // Set the beginning time of this LiDAR frame from the header
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_frame_beg_time
      // Calculate the end time of the LiDAR scan using the last point's timestamp offset
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      // Set the current processing cloud to the raw LiDAR data
      meas.pcl_proc_cur = meas.lidar;
      // Mark that the LiDAR data has been pushed for this scan
      lidar_pushed = true; // flag
    }

    // Wait for IMU data that spans the entire LiDAR scan for complete propagation
    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      // Not enough IMU data yet, need to wait
      return false;
    }

    // Create a measurement group to hold the synchronized IMU data for this LiDAR scan
    struct MeasureGroup m; // standard method to keep imu message.

    // Clear any previous IMU data in the group
    m.imu.clear();
    // Set the LIO processing time to the end of the LiDAR scan
    m.lio_time = meas.lidar_frame_end_time;
    // Lock the shared buffer mutex
    mtx_buffer.lock();
    // Extract all IMU messages that fall within this LiDAR scan
    while (!imu_buffer.empty())
    {
      // Stop when IMU timestamps exceed the LiDAR scan end time
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
      // Add the IMU message to the measurement group
      m.imu.push_back(imu_buffer.front());
      // Remove the IMU message from the buffer
      imu_buffer.pop_front();
    }
    // Remove the processed LiDAR data from the buffers
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    // Unlock the shared buffer mutex
    mtx_buffer.unlock();
    // Notify waiting threads that data has been consumed
    sig_buffer.notify_all();

    // Set the processing flag to LIO (LiDAR-Inertial Odometry)
    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    // Add the measurement group to the synchronized measurements
    meas.measures.push_back(m);
    // Reset the LiDAR push flag for the next scan
    lidar_pushed = false; // sync one whole lidar scan.
    // Signal that a complete synchronized package is ready
    return true;

    break;
  }

  // LIVO mode (LiDAR + IMU + Camera)
  case LIVO:
  {
    // Save the previous processing flag to determine the next step
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // Select the synchronization path based on the previous state
    switch (last_lio_vio_flg)
    {
    // Initial state or after a VIO update: prepare for the next LIO step
    case WAIT:
    case VIO:
    {
      // Compute the image capture time with the exposure time offset
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      // Initialize the last LIO update time on the very first call
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();

      // Compute the newest LiDAR time (last header + last point offset)
      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      // Get the newest IMU timestamp
      double imu_newest_time = imu_buffer.back()->header.stamp.toSec();

      // Reject images that are older than the last LIO update (already processed)
      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        // Discard the stale image frame
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        // Log the discarded frame
        ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      // Wait if the image capture time exceeds the newest available LiDAR or IMU data
      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // Not enough data yet to synchronize this image
        return false;
      }

      // Create a measurement group for the LIO step
      struct MeasureGroup m;

      // Clear any previous IMU data in the group
      m.imu.clear();
      // Set the LIO time to the image capture time (LIO and VIO share the same timestamp)
      m.lio_time = img_capture_time;
      // Lock the shared buffer mutex
      mtx_buffer.lock();
      // Extract IMU messages up to the LIO processing time
      while (!imu_buffer.empty())
      {
        // Stop when IMU timestamps exceed the LIO time
        if (imu_buffer.front()->header.stamp.toSec() > m.lio_time) break;

        // Only include IMU messages that are newer than the last LIO update
        if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        // Remove the processed IMU message from the buffer
        imu_buffer.pop_front();
      }
      // Unlock the shared buffer mutex
      mtx_buffer.unlock();
      // Notify waiting threads
      sig_buffer.notify_all();

      // Swap the next processing cloud to the current one for incremental accumulation
      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      // Clear the next processing cloud
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      // Count the number of LiDAR frames in the buffer
      int lid_frame_num = lid_raw_data_buffer.size();
      // Estimate the maximum size needed for the current and next clouds
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      // Reserve memory for the current processing cloud
      meas.pcl_proc_cur->reserve(max_size);
      // Reserve memory for the next processing cloud
      meas.pcl_proc_next->reserve(max_size);

      // Process all LiDAR frames up to the image capture time
      while (!lid_raw_data_buffer.empty())
      {
        // Stop if the next frame's header exceeds the image capture time
        if (lid_header_time_buffer.front() > img_capture_time) break;
        // Get the points from the current LiDAR frame
        auto pcl(lid_raw_data_buffer.front()->points);
        // Get the header time of the current LiDAR frame
        double frame_header_time(lid_header_time_buffer.front());
        // Compute the maximum time offset (in ms) for points belonging to the current LIO window
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        // Iterate over all points in the LiDAR frame
        for (int i = 0; i < pcl.size(); i++)
        {
          // Get a mutable copy of the current point
          auto pt = pcl[i];
          // Check if the point's timestamp offset falls within the current LIO window
          if (pcl[i].curvature < max_offs_time_ms)
          {
            // Adjust the curvature timestamp relative to the last LIO update time
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            // Add to the current processing cloud
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            // Adjust the curvature timestamp relative to the new LIO time
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            // Add to the next processing cloud for the subsequent LIO update
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        // Remove the processed LiDAR frame from the buffer
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      // Add the measurement group to the synchronized measurements
      meas.measures.push_back(m);
      // Set the processing flag to LIO for the upcoming step
      meas.lio_vio_flg = LIO;
      // Signal that a complete synchronized package is ready
      return true;
    }

    // After a LIO update: prepare for the VIO step
    case LIO:
    {
      // Compute the image capture time with the exposure time offset
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      // Set the processing flag to VIO for the upcoming step
      meas.lio_vio_flg = VIO;
      // Clear the previous measurements for the new VIO step
      meas.measures.clear();
      // Get the timestamp of the next IMU message (unused in the current code)
      double imu_time = imu_buffer.front()->header.stamp.toSec();

      // Create a measurement group for the VIO step
      struct MeasureGroup m;
      // Set the VIO time to the image capture time
      m.vio_time = img_capture_time;
      // Set the LIO time to the last LIO update time for reference
      m.lio_time = meas.last_lio_update_time;
      // Attach the front image from the buffer to this VIO measurement
      m.img = img_buffer.front();
      // Lock the shared buffer mutex
      mtx_buffer.lock();
      // Remove the processed image from the buffers
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      // Unlock the shared buffer mutex
      mtx_buffer.unlock();
      // Notify waiting threads
      sig_buffer.notify_all();
      // Add the measurement group to the synchronized measurements
      meas.measures.push_back(m);
      // Reset the LiDAR push flag (the next LIO step will refresh the frame end time)
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // Signal that a complete synchronized package is ready
      return true;
    }

    // Unexpected state: return false
    default:
    {
      return false;
    }
    }
    break;
  }

  // LO-only mode (LiDAR only, no IMU, no camera)
  case ONLY_LO:
  {
    // If this is a new LiDAR scan, initialize the measurement
    if (!lidar_pushed) 
    { 
      // Return false if no LiDAR data is available
      if (lid_raw_data_buffer.empty())  return false;
      // Push the first LiDAR topic into the measurement
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      // Set the beginning time from the LiDAR header
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      // Calculate the end time using the last point's timestamp offset
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      // Mark that the LiDAR data has been pushed
      lidar_pushed = true;             
    }
    // Create an empty measurement group (no IMU or image data)
    struct MeasureGroup m; // standard method to keep imu message.
    // Set the LIO processing time to the end of the LiDAR scan
    m.lio_time = meas.lidar_frame_end_time;
    // Lock the shared buffer mutex
    mtx_buffer.lock();
    // Remove the processed LiDAR data from the buffer
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    // Unlock the shared buffer mutex
    mtx_buffer.unlock();
    // Notify waiting threads
    sig_buffer.notify_all();
    // Reset the LiDAR push flag for the next scan
    lidar_pushed = false; // sync one whole lidar scan.
    // Set the processing flag to LO (LiDAR-only Odometry)
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    // Add the measurement group to the synchronized measurements
    meas.measures.push_back(m);
    // Signal that a complete synchronized package is ready
    return true;
    break;
  }

  // Unknown SLAM mode: log error and return false
  default:
  {
    // Log the invalid SLAM mode
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  // Fallback error if the switch statement falls through
  ROS_ERROR("out sync");
}

// Publish the current image with tracked features overlaid (RGB visualization)
void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  // Get the RGB image copy from the VIO manager (includes drawn features)
  cv::Mat img_rgb = vio_manager->img_cp;
  // Create a CvImage message for publishing
  cv_bridge::CvImage out_msg;
  // Set the timestamp to the current ROS time
  out_msg.header.stamp = ros::Time::now();
  // Set the image encoding to BGR8
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  // Attach the RGB image to the message
  out_msg.image = img_rgb;
  // Publish the image message
  pubImage.publish(out_msg.toImageMsg());
}

// Publishes the registered world-frame point cloud with RGB color from the
// camera image, and optionally saves PCD files to disk.
// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  // If there are no points to publish, return immediately
  if (pcl_w_wait_pub->empty()) return;
  // Create an RGB-colored point cloud for publishing
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  // Static counter for scan accumulation before publishing
  static int pub_num = 1;
  // Increment the publication counter
  pub_num++;

  // In VIO mode, project LiDAR points into the camera image for RGB coloring
  if (LidarMeasures.lio_vio_flg == VIO)
  {
    // Accumulate the new world-frame points to the wait buffer
    *pcl_wait_pub += *pcl_w_wait_pub;
    // Only publish after accumulating enough scans (pub_scan_num threshold)
    if(pub_num >= pub_scan_num)
    {
      // Reset the publication counter
      pub_num = 1;
      // Get the number of accumulated points
      size_t size = pcl_wait_pub->points.size();
      // Reserve memory for the RGB cloud
      laserCloudWorldRGB->reserve(size);
      // Get the current RGB image from the VIO manager for color lookup
      cv::Mat img_rgb = vio_manager->img_rgb;
      // Iterate over all accumulated points for RGB color assignment
      for (size_t i = 0; i < size; i++)
      {
        // Create a new RGB point
        PointTypeRGB pointRGB;
        // Set the point's position from the accumulated cloud
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        // Get the world-frame position for projection
        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        // Project the world point into the camera frustum; skip if behind the camera
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        // Project the world point to 2D pixel coordinates
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        // Check if the pixel is within the image frame (with a 3-pixel border)
        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          // Get the interpolated pixel color from the RGB image
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          // Assign the red channel (BGR image: pixel[2] is R)
          pointRGB.r = pixel[2];
          // Assign the green channel
          pointRGB.g = pixel[1];
          // Assign the blue channel
          pointRGB.b = pixel[0];
          // Only add the point if it is beyond the blind RGB distance threshold
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  // Create a ROS PointCloud2 message for publishing
  sensor_msgs::PointCloud2 laserCloudmsg;
  // In LIVO mode during VIO step, publish the RGB-colored point cloud
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  // In LIO or LO mode, publish the intensity-only world-frame point cloud
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  // Set the timestamp to the current ROS time
  laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
  // Set the frame ID to the reference odometry frame
  laserCloudmsg.header.frame_id = "camera_init";
  // Publish the point cloud
  pubLaserCloudFullRes.publish(laserCloudmsg);

  // Determine the update time for file naming (based on VIO or LIO time)
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    // Use the VIO update time
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    // Use the LIO update time
    update_time = LidarMeasures.measures.back().lio_time;
  }
  // Format the update time as a string for file naming
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  // If PCD saving is enabled, accumulate and periodically write point clouds
  if (pcd_save_en)
  {
    // Static counter to track scans between saves
    static int scan_wait_num = 0;

    // Select the save method based on the configured PCD save type
    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        // In LIVO mode, accumulate RGB world-frame points for saving
        if (slam_mode_ == LIVO)
        {
          // Accumulate RGB-colored world-frame points
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          // Accumulate intensity-only world-frame points
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        // Increment the scan counter for LIO/LO updates
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          // Get the number of undistorted points
          int size = feats_undistort->points.size();
          // Allocate a body-frame point cloud
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          // Transform all points from LiDAR body frame to IMU frame
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          // Accumulate the body-frame points
          *pcl_wait_save_intensity += *laserCloudBody;
          // Increment the scan counter
          scan_wait_num++;
          // Log the accumulated point count
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        // Set the save interval to 1 for body-frame mode (save every scan)
        pcd_save_interval = 1;
        
        break;

      default:
        // Default behavior: save every scan
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    // Check if it is time to write the accumulated points to disk
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      // Build the output file path with the timestamp
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      // Create a PCD file writer
      pcl::PCDWriter pcd_writer;

      // Log the save location
      cout << "current scan saved to " << all_points_dir << endl;
      // Write RGB points if available
      if (pcl_wait_save->points.size() > 0)
      {
        // Write the RGB point cloud to disk
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        // Clear the save buffer
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      // Write intensity points if available
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        // Write the intensity point cloud to disk
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        // Clear the save buffer
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      // Reset the scan wait counter
      scan_wait_num = 0;
    }
    
    // Append the LiDAR pose to the trajectory file for LIO/LO updates
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      // Extract the quaternion from the current rotation
      Eigen::Quaterniond q(_state.rot_end);
      // Set fixed-point notation
      fout_lidar_pos << std::fixed << std::setprecision(6);
      // Write the timestamp, position, and orientation to the pose file
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  // If image saving is enabled, periodically save camera images with poses
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    // Static counter to track images between saves
    static int img_wait_num = 0;
    // Increment the image counter
    img_wait_num++;

    // Check if it is time to save an image
    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      // Save the RGB image to disk with the timestamp as filename
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      // Extract the quaternion from the current rotation
      Eigen::Quaterniond q(_state.rot_end);
      // Set fixed-point notation
      fout_visual_pos << std::fixed << std::setprecision(6);
      // Write the VIO timestamp, position, and orientation to the visual pose file
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      // Reset the image wait counter
      img_wait_num = 0;
    }
  }

  // Clear the wait buffer for RGB points if any were added
  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  // Clear the world-frame wait buffer after VIO processing
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

// Publish the visual sub-map point cloud (before VIO update)
void LIVMapper::publish_visual_sub_map(const ros::Publisher &pubSubVisualMap)
{
  // Create a shared pointer to the visual sub-map cloud
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  // Get the number of points; return early if empty
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  // Create a new cloud for publishing (copy of the sub-map)
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  // Copy the sub-map into the publish cloud
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  // Always publish (the condition is constant true)
  if (1)
  {
    // Create a ROS PointCloud2 message
    sensor_msgs::PointCloud2 laserCloudmsg;
    // Convert the PCL cloud to a ROS message
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    // Set the timestamp to the current ROS time
    laserCloudmsg.header.stamp = ros::Time::now();
    // Set the frame ID to the reference odometry frame
    laserCloudmsg.header.frame_id = "camera_init";
    // Publish the visual sub-map
    pubSubVisualMap.publish(laserCloudmsg);
  }
}

// Publish the effective (inlier) world-frame points from LIO point-to-plane matching
void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  // Get the number of effective points
  int effect_feat_num = ptpl_list.size();
  // Allocate a point cloud with the same number of points
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  // Iterate over all effective points and copy their world-frame positions
  for (int i = 0; i < effect_feat_num; i++)
  {
    // Set the x coordinate from the point-to-plane list
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    // Set the y coordinate from the point-to-plane list
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    // Set the z coordinate from the point-to-plane list
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  // Create a ROS PointCloud2 message
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  // Convert the PCL cloud to a ROS message
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  // Set the timestamp to the current ROS time
  laserCloudFullRes3.header.stamp = ros::Time::now();
  // Set the frame ID to the reference odometry frame
  laserCloudFullRes3.header.frame_id = "camera_init";
  // Publish the effective point cloud
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

// Template helper: set position and orientation fields of a ROS pose message from the current state
template <typename T> void LIVMapper::set_posestamp(T &out)
{
  // Set the position x from the estimated state
  out.position.x = _state.pos_end(0);
  // Set the position y from the estimated state
  out.position.y = _state.pos_end(1);
  // Set the position z from the estimated state
  out.position.z = _state.pos_end(2);
  // Set the orientation x from the pre-computed quaternion
  out.orientation.x = geoQuat.x;
  // Set the orientation y from the pre-computed quaternion
  out.orientation.y = geoQuat.y;
  // Set the orientation z from the pre-computed quaternion
  out.orientation.z = geoQuat.z;
  // Set the orientation w from the pre-computed quaternion
  out.orientation.w = geoQuat.w;
}

// Publish the current odometry estimate and broadcast the TF transform
void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  // Set the header frame ID to the reference odometry frame
  odomAftMapped.header.frame_id = "camera_init";
  // Set the child frame ID to the mapped frame
  odomAftMapped.child_frame_id = "aft_mapped";
  // Set the timestamp to the current ROS time
  odomAftMapped.header.stamp = ros::Time::now(); //.ros::Time()fromSec(last_timestamp_lidar);
  // Fill in the pose from the current state estimate
  set_posestamp(odomAftMapped.pose.pose);

  // Create a static TF broadcaster for the odometry transform
  static tf::TransformBroadcaster br;
  // Create a TF transform object
  tf::Transform transform;
  // Create a TF quaternion
  tf::Quaternion q;
  // Set the translation from the estimated position
  transform.setOrigin(tf::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  // Set the quaternion components from the pre-computed geoQuat
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  // Set the rotation from the quaternion
  transform.setRotation(q);
  // Broadcast the transform from "camera_init" to "aft_mapped"
  br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped") );
  // Publish the odometry message
  pubOdomAftMapped.publish(odomAftMapped);
}

// Publish the current pose to MAVROS for UAV vision-based navigation
void LIVMapper::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
  // Set the timestamp to the current ROS time
  msg_body_pose.header.stamp = ros::Time::now();
  // Set the header frame ID to the reference odometry frame
  msg_body_pose.header.frame_id = "camera_init";
  // Fill in the pose from the current state estimate
  set_posestamp(msg_body_pose.pose);
  // Publish the pose to the MAVROS topic
  mavros_pose_publisher.publish(msg_body_pose);
}

// Append the current pose to the trajectory path and publish it
void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  // Fill in the pose from the current state estimate
  set_posestamp(msg_body_pose.pose);
  // Set the timestamp to the current ROS time
  msg_body_pose.header.stamp = ros::Time::now();
  // Set the header frame ID to the reference odometry frame
  msg_body_pose.header.frame_id = "camera_init";
  // Append the current pose to the path trajectory
  path.poses.push_back(msg_body_pose);
  // Publish the complete path
  pubPath.publish(path);
}