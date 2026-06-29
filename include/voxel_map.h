/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Header guard to prevent multiple inclusions of this file.
#ifndef VOXEL_MAP_H_
// Define the header guard macro.
#define VOXEL_MAP_H_

// Include shared type definitions (StatesGroup, pointWithVar, MeasureGroup, etc.).
#include "common_lib.h"
// Include Eigen dense linear algebra library for matrix/vector operations.
#include <Eigen/Dense>
// Include file stream for logging debug information to disk.
#include <fstream>
// Include standard C math functions.
#include <math.h>
// Include mutex for thread-safe access to the voxel map.
#include <mutex>
// Include OpenMP header for parallel loop execution.
#include <omp.h>
// Include PCL I/O utilities for point cloud copying and field manipulation.
#include <pcl/common/io.h>
// Include ROS node handle and publisher/subscriber infrastructure.
#include <ros/ros.h>
// Include std::thread for multi-threading support.
#include <thread>
// Include POSIX API for usleep and other system calls.
#include <unistd.h>
// Include unordered_map for hash-based voxel storage.
#include <unordered_map>
// Include visualization marker message type for RViz publishing.
#include <visualization_msgs/Marker.h>
// Include array visualization marker message type for batch publishing.
#include <visualization_msgs/MarkerArray.h>

// Prime multiplier for the voxel location hash function.
#define VOXELMAP_HASH_P 116101
// Maximum hash value to bound the hash output range.
#define VOXELMAP_MAX_N 10000000000

// Global counter for generating unique voxel plane identifiers.
static int voxel_plane_id = 0;

// Configuration structure for voxel map parameters.
typedef struct VoxelMapConfig
{
  // Maximum side length of an individual voxel in meters.
  double max_voxel_size_;
  // Maximum number of octree subdivision layers.
  int max_layer_;
  // Maximum number of iterations for plane refinement optimization.
  int max_iterations_;
  // Initial point-count thresholds per octree layer before subdivision.
  std::vector<int> layer_init_num_;
  // Maximum number of points stored in a single leaf voxel.
  int max_points_num_;
  // Threshold on eigenvalue ratio for planar surface detection.
  double planner_threshold_;
  // Expected angular error from beam divergence for LiDAR points.
  double beam_err_;
  // Expected depth measurement error for LiDAR points.
  double dept_err_;
  // Multiplier for sigma-based outlier rejection threshold.
  double sigma_num_;
  // Flag to enable publishing plane visualization markers.
  bool is_pub_plane_map_;

  // config of local map sliding
  double sliding_thresh;
  // Enable flag for the map sliding window mechanism.
  bool map_sliding_en;
  // Half size of the local map sliding window in voxel units.
  int half_map_size;
// Typedef alias for the VoxelMapConfig struct.
} VoxelMapConfig;

// Structure holding the result of a point-to-plane association for one LiDAR point.
typedef struct PointToPlane
{
  // 3D coordinates of the point in the body frame.
  Eigen::Vector3d point_b_;
  // 3D coordinates of the point in the world frame.
  Eigen::Vector3d point_w_;
  // Surface normal vector of the associated plane.
  Eigen::Vector3d normal_;
  // Center point of the associated plane.
  Eigen::Vector3d center_;
  // 6x6 covariance matrix of the plane parameter estimates.
  Eigen::Matrix<double, 6, 6> plane_var_;
  // 3x3 covariance of the point measurement in the body frame.
  M3D body_cov_;
  // Octree layer depth at which this plane was found.
  int layer_;
  // Plane offset distance: dot(normal, center).
  double d_;
  // Smallest eigenvalue of the plane covariance matrix.
  double eigen_value_;
  // Whether this point-to-plane association is geometrically valid.
  bool is_valid_;
  // Signed distance from the point to the associated plane surface.
  float dis_to_plane_;
// Typedef alias for the PointToPlane struct.
} PointToPlane;

// Structure representing a planar surface fitted within a single voxel.
typedef struct VoxelPlane
{
  // Center position of the plane in world coordinates.
  Eigen::Vector3d center_;
  // Unit normal vector of the plane (perpendicular to surface).
  Eigen::Vector3d normal_;
  // Binormal direction on the plane surface.
  Eigen::Vector3d y_normal_;
  // Tangent direction on the plane surface.
  Eigen::Vector3d x_normal_;
  // 3x3 covariance matrix of the points in this voxel.
  Eigen::Matrix3d covariance_;
  // 6x6 covariance matrix of the plane parameter estimates.
  Eigen::Matrix<double, 6, 6> plane_var_;
  // Estimated radius of the plane patch within the voxel.
  float radius_ = 0;
  // Minimum eigenvalue from PCA (smallest spread direction).
  float min_eigen_value_ = 1;
  // Middle eigenvalue from PCA.
  float mid_eigen_value_ = 1;
  // Maximum eigenvalue from PCA.
  float max_eigen_value_ = 1;
  // Plane offset: dot(normal, center).
  float d_ = 0;
  // Number of points used to estimate this plane.
  int points_size_ = 0;
  // Whether the points in this voxel form a valid planar surface.
  bool is_plane_ = false;
  // Whether the plane has been initialized with points.
  bool is_init_ = false;
  // Unique identifier assigned from the global voxel_plane_id counter.
  int id_ = 0;
  // Whether the plane estimate has been updated in the current frame.
  bool is_update_ = false;
  // Default constructor: zero-initializes all matrix and vector members.
  VoxelPlane()
  {
    // Initialize plane parameter covariance to zero matrix.
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    // Initialize point covariance to zero matrix.
    covariance_ = Eigen::Matrix3d::Zero();
    // Initialize center to origin.
    center_ = Eigen::Vector3d::Zero();
    // Initialize normal to zero vector.
    normal_ = Eigen::Vector3d::Zero();
  }
// Typedef alias for the VoxelPlane struct.
} VoxelPlane;

// 3D integer coordinates used as a key in the voxel hash map.
class VOXEL_LOCATION
{
public:
  // Integer voxel grid coordinates along the X, Y, and Z axes.
  int64_t x, y, z;

  // Constructor with default zero-initialized voxel coordinates.
  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  // Equality operator: two locations are equal if all coordinates match.
  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
// Closing brace for VOXEL_LOCATION class.
};

// Hash functor specialization for VOXEL_LOCATION to enable use as unordered_map key.
// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

// Down-sampled point structure storing only position, intensity, and accumulation count.
struct DS_POINT
{
  // 3D position coordinates (x, y, z).
  float xyz[3];
  // Intensity or reflectivity value of the point.
  float intensity;
  // Number of raw points accumulated into this down-sampled point.
  int count = 0;
// Closing brace for DS_POINT struct.
};

// Compute the measurement covariance of a LiDAR point in the body frame from range and angular uncertainties.
void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

// Octree structure for adaptive spatial subdivision of the voxel map.
class VoxelOctoTree
{

public:
  // Default constructor.
  VoxelOctoTree() = default;
  // Temporary buffer of points currently being inserted into this node.
  std::vector<pointWithVar> temp_points_;
  // Pointer to the VoxelPlane fitted from points in this voxel.
  VoxelPlane *plane_ptr_;
  // Current depth layer of this octree node (0 = root).
  int layer_;
  // Node state: 0 = leaf (no children), 1 = has children (subdivided).
  int octo_state_; // 0 is end of tree, 1 is not
  // Pointers to the eight child octants (nullptr if not subdivided).
  VoxelOctoTree *leaves_[8];
  // Center coordinates of this voxel in the world frame (x, y, z).
  double voxel_center_[3]; // x, y, z
  // Per-layer initial point count thresholds before subdivision.
  std::vector<int> layer_init_num_;
  // Half the side length of this voxel (quarter of the parent).
  float quater_length_;
  // Threshold on eigenvalue ratio to classify points as a plane.
  float planer_threshold_;
  // Minimum number of points required before subdividing this node.
  int points_size_threshold_;
  // Number of new points that trigger a plane re-estimation.
  int update_size_threshold_;
  // Maximum number of points stored in this leaf voxel.
  int max_points_num_;
  // Maximum allowed octree depth (global limit).
  int max_layer_;
  // Count of new points added since the last plane update.
  int new_points_;
  // Whether the octree subdivision has been performed.
  bool init_octo_;
  // Whether this node should be updated with new measurements.
  bool update_enable_;

  // Parameterized constructor: sets depth limits, thresholds, and zero-initializes state.
  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    // Clear any temporary points from the buffer.
    temp_points_.clear();
    // Initially this node is a leaf (no children).
    octo_state_ = 0;
    // No new points have been added yet.
    new_points_ = 0;
    // Default threshold for triggering plane re-estimation.
    update_size_threshold_ = 5;
    // Octree has not been subdivided yet.
    init_octo_ = false;
    // Update is enabled by default.
    update_enable_ = true;
    // Initialize all child pointers to null.
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    // Allocate a new VoxelPlane for this node.
    plane_ptr_ = new VoxelPlane;
  }

  // Destructor: recursively deletes child octants and the associated plane.
  ~VoxelOctoTree()
  {
    // Delete all eight child octrees.
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    // Delete the plane associated with this voxel.
    delete plane_ptr_;
  }
  // Initialize a plane from a set of points using PCA-based plane fitting.
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
  // Initialize the octree by subdividing until each leaf meets the threshold.
  void init_octo_tree();
  // Subdivide this node into eight children by splitting along each axis.
  void cut_octo_tree();
  // Update the octree by inserting a new point and optionally re-estimating the plane.
  void UpdateOctoTree(const pointWithVar &pv);

  // Traverse the octree to find the leaf voxel containing the given world point.
  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
  // Insert a point into the octree, creating child nodes as needed.
  VoxelOctoTree *Insert(const pointWithVar &pv);
// Closing brace for VoxelOctoTree class.
};

// Load voxel map configuration parameters from the ROS parameter server.
void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

// Main manager class for the LiDAR voxel map: handles state estimation, map building, and residual computation.
class VoxelMapManager
{
public:
  // Default constructor.
  VoxelMapManager() = default;
  // Configuration parameters governing voxel map behavior.
  VoxelMapConfig config_setting_;
  // Frame counter incremented with each processed LiDAR scan.
  int current_frame_id_ = 0;
  // ROS publisher for visualization of the voxel plane map.
  ros::Publisher voxel_map_pub_;
  // Hash map from integer voxel coordinates to their octree structures.
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;

  // Undistorted LiDAR point cloud from the current scan.
  PointCloudXYZI::Ptr feats_undistort_;
  // Down-sampled point cloud in the body frame.
  PointCloudXYZI::Ptr feats_down_body_;
  // Down-sampled point cloud transformed to the world frame.
  PointCloudXYZI::Ptr feats_down_world_;

  // Rotation matrix from IMU to LiDAR (extrinsic calibration).
  M3D extR_;
  V3D extT_;
  // Timing statistics for residual building and EKF update steps.
  float build_residual_time, ekf_time;
  // Running average of residual building time.
  float ave_build_residual_time = 0.0;
  // Running average of EKF update time.
  float ave_ekf_time = 0.0;
  // Total number of LiDAR scans processed.
  int scan_count = 0;
  // Current EKF state estimate (pose, velocity, biases, gravity).
  StatesGroup state_;
  // Position from the previous frame, used for motion compensation.
  V3D position_last_;

  // Position of the sensor when the last map sliding operation occurred.
  V3D last_slide_position = {0,0,0};

  // Orientation quaternion for RViz visualization messages.
  geometry_msgs::Quaternion geoQuat_;

  // Number of points in the down-sampled feature cloud.
  int feats_down_size_;
  // Number of effective features used after outlier rejection.
  int effct_feat_num_;
  // List of cross-product matrices for Jacobian computation.
  std::vector<M3D> cross_mat_list_;
  // List of body-frame covariance matrices for each feature point.
  std::vector<M3D> body_cov_list_;
  // List of point-with-variance structures for EKF update.
  std::vector<pointWithVar> pv_list_;
  // List of point-to-plane association results for residual computation.
  std::vector<PointToPlane> ptpl_list_;

  // Parameterized constructor: stores config and voxel map reference, and initializes point clouds.
  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    // Reset frame counter.
    current_frame_id_ = 0;
    // Allocate undistorted feature point cloud.
    feats_undistort_.reset(new PointCloudXYZI());
    // Allocate body-frame down-sampled point cloud.
    feats_down_body_.reset(new PointCloudXYZI());
    // Allocate world-frame down-sampled point cloud.
    feats_down_world_.reset(new PointCloudXYZI());
  };

  // Perform EKF state estimation using the propagated state and current LiDAR measurements.
  void StateEstimation(StatesGroup &state_propagat);
  // Transform a point cloud from one coordinate frame to another using rotation and translation.
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  // Build the voxel map from the current set of undistorted LiDAR features.
  void BuildVoxelMap();
  // Map a 3D world point to an RGB color based on its voxel coordinates.
  V3F RGBFromVoxel(const V3D &input_point);

  // Update the voxel map by inserting new LiDAR points.
  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

  // Build the full list of point-to-plane residuals using OpenMP parallelization.
  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  // Compute a single point-to-plane residual and its associated probability weight.
  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  // Publish the voxel plane map as RViz visualization markers.
  void pubVoxelMap();

  // Slide the local map window when the sensor moves beyond a threshold distance.
  void mapSliding();
  // Clear voxel memory outside the specified bounding box for the local sliding map.
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );

private:
  // Collect all updated planes from the octree for visualization publishing.
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  // Publish a single plane as a visualization marker with specified color and transparency.
  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  // Compute a geometry_msgs Quaternion from three orthonormal basis vectors.
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  // Map a scalar value to a jet colormap RGB triplet for visualization.
  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
// Closing brace for VoxelMapManager class.
};
// Shared pointer type alias for VoxelMapManager.
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

// End of header guard.
#endif // VOXEL_MAP_H_