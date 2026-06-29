/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Header guard to prevent multiple inclusions of this file.
#ifndef VIO_H_
// Define the header guard macro.
#define VIO_H_

// Include voxel map types for visual-LiDAR association (VoxelOctoTree, pointWithVar).
#include "voxel_map.h"
// Include feature data structure (Feature, VisualPoint).
#include "feature.h"
// Include OpenCV image processing utilities (C interface).
#include <opencv2/imgproc/imgproc_c.h>
// Include PCL voxel grid filter for down-sampling point clouds.
#include <pcl/filters/voxel_grid.h>
// Include std::set for ordered unique element storage.
#include <set>
// Include vikit math utility functions (e.g., skew-symmetric, normalization).
#include <vikit/math_utils.h>
// Include vikit robust cost functions (e.g., Huber, Tukey).
#include <vikit/robust_cost.h>
// Include vikit vision functions for image processing.
#include <vikit/vision.h>
// Include vikit pinhole camera model for projection and unprojection.
#include <vikit/pinhole_camera.h>

// Holds the retrieved visual map points projected into the current frame together with
// their warped patches, photometric errors, and search levels for EKF update.
struct SubSparseMap
{
  // Photometric errors after projection with the propagated (prior) pose.
  vector<float> propa_errors;
  // Photometric errors after the EKF update step.
  vector<float> errors;
  // Warped image patches warped from the reference frame to the current frame.
  vector<vector<float>> warp_patch;
  // Image pyramid search levels at which each point is tracked.
  vector<int> search_levels;
  // Pointers to the retrieved visual map points for the current frame.
  vector<VisualPoint *> voxel_points;
  // Inverse exposure times for each retrieved point's reference frame.
  vector<double> inv_expo_list;
  // New LiDAR points with covariance to be added to the visual map.
  vector<pointWithVar> add_from_voxel_map;

  // Constructor: pre-allocate memory for all vectors to reduce reallocation overhead.
  SubSparseMap()
  {
    // Pre-allocate space for propagated errors.
    propa_errors.reserve(SIZE_LARGE);
    // Pre-allocate space for updated errors.
    errors.reserve(SIZE_LARGE);
    // Pre-allocate space for warped patches.
    warp_patch.reserve(SIZE_LARGE);
    // Pre-allocate space for search levels.
    search_levels.reserve(SIZE_LARGE);
    // Pre-allocate space for voxel point pointers.
    voxel_points.reserve(SIZE_LARGE);
    // Pre-allocate space for inverse exposure times.
    inv_expo_list.reserve(SIZE_LARGE);
    // Pre-allocate smaller space for points to add from the voxel map.
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  // Clear all stored data to prepare for the next frame.
  void reset()
  {
    // Clear propagated error vector.
    propa_errors.clear();
    // Clear updated error vector.
    errors.clear();
    // Clear warped patch vector.
    warp_patch.clear();
    // Clear search level vector.
    search_levels.clear();
    // Clear voxel point pointer vector.
    voxel_points.clear();
    // Clear inverse exposure time list.
    inv_expo_list.clear();
    // Clear the list of points to add from the voxel map.
    add_from_voxel_map.clear();
  }
// Closing brace for SubSparseMap struct.
};

// Stores the affine warp matrix and search level for a reference feature, used to
// warp the reference patch into the current frame during alignment.
class Warp
{
public:
  // Affine warp matrix mapping from the reference image to the current image.
  Matrix2d A_cur_ref;
  // Pyramid search level at which this warp is valid.
  int search_level;
  // Constructor: stores the search level and warp matrix.
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  // Destructor (trivial).
  ~Warp() {}
// Closing brace for Warp class.
};

// A voxel bucket containing pointers to VisualPoint objects, used for spatial
// hashing of the visual map for efficient retrieval.
class VOXEL_POINTS
{
public:
  // Vector of visual point pointers stored in this voxel bucket.
  std::vector<VisualPoint *> voxel_points;
  // Number of points in this voxel bucket.
  int count;
  // Constructor: initializes the point count.
  VOXEL_POINTS(int num) : count(num) {}
  // Destructor: deletes all VisualPoint objects owned by this voxel.
  ~VOXEL_POINTS() 
  { 
    // Iterate through all visual points and delete them.
    for (VisualPoint* vp : voxel_points) 
    {
      // Only delete non-null pointers and set them to null to prevent double deletion.
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
// Closing brace for VOXEL_POINTS class.
};

// Manages visual-inertial odometry: frame processing, feature retrieval, patch warp,
// photometric error computation, EKF update (direct image alignment), visual map
// point generation, and keyframe management.
class VIOManager
{
public:
  // Size of the spatial grid used for feature distribution.
  int grid_size;
  // Pointer to the abstract camera model for projection/unprojection.
  vk::AbstractCamera *cam;
  // Pointer to the pinhole camera model (derived from AbstractCamera).
  vk::PinholeCamera *pinhole_cam;
  // Pointer to the current EKF state estimate.
  StatesGroup *state;
  // Pointer to the propagated (prior) EKF state before the visual update.
  StatesGroup *state_propagat;
  // Rotation matrices and Jacobians for the LiDAR-IMU-camera extrinsics and state.
  M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  // Translation vectors for LiDAR-IMU-camera extrinsics.
  V3D Pli, Pci, Pcl, Pcw;
  // Grid indices for each cell in the feature distribution grid.
  vector<int> grid_num;
  // Map index lookup for mapping grid cells to feature indices.
  vector<int> map_index;
  // Flags indicating whether a point lies near the image border.
  vector<int> border_flag;
  // Flags indicating whether a map point should be updated in the current frame.
  vector<int> update_flag;
  // Distances from the camera to projected visual map points.
  vector<float> map_dist;
  // Scan-line values for efficient patch extraction.
  vector<float> scan_value;
  // Buffer for holding image patch data during warp operations.
  vector<float> patch_buffer;
  // Flags for algorithmic options: normal equations, inverse compositional, exposure estimation, ray casting, ref patch caching.
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  // Enable flags for NCC-based matching and Colmap output dumping.
  bool ncc_en = false, colmap_output_en = false;

  // Image dimensions and grid subdivision parameters.
  int width, height, grid_n_width, grid_n_height, length;
  // Factor by which the input image is resized before processing.
  double image_resize_factor;
  // Camera intrinsic parameters: focal lengths and principal point.
  double fx, fy, cx, cy;
  // Patch parameters: pyramid levels, patch dimensions, border size, and warp length.
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  // Maximum EKF iterations and total number of tracked visual points.
  int max_iterations, total_points;

  // Measurement noise covariance for image points, outlier rejection threshold, and NCC matching threshold.
  double img_point_cov, outlier_threshold, ncc_thre;
  
  // Pointer to the current sparse visual submap for the frame.
  SubSparseMap *visual_submap;
  // Sample points along camera rays for LiDAR-visual depth association.
  std::vector<std::vector<V3D>> rays_with_sample_points;

  // Timing accumulators for Jacobian computation and EKF update steps.
  double compute_jacobian_time, update_ekf_time;
  // Running average of total VIO processing time per frame.
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  // Number of camera frames processed since initialization.
  int frame_count = 0;
  // Flag to enable/disable debug plotting of tracked features.
  bool plot_flag;

  // EKF gain matrix and Hessian approximation (H^T * H) for the visual update.
  Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  // Kalman gain matrix and pseudo-inverse of the sub-sampled Jacobian.
  MatrixXd K, H_sub_inv;

  // Output file streams for camera pose logging and Colmap data export.
  ofstream fout_camera, fout_colmap;
  // Hash map from voxel coordinates to visual point buckets.
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;
  // Hash map from voxel coordinates to point counts for the current frame subset.
  unordered_map<VOXEL_LOCATION, int> sub_feat_map; 
  // Cache of precomputed affine warp matrices indexed by feature ID.
  unordered_map<int, Warp *> warp_map;
  // Visual points retrieved from the map for the current frame.
  vector<VisualPoint *> retrieve_voxel_points;
  // New visual points to be appended to the map from LiDAR-depth associations.
  vector<pointWithVar> append_voxel_points;
  // Current camera frame being processed.
  FramePtr new_frame_;
  // OpenCV image containers: grayscale copy, RGB version, and test image.
  cv::Mat img_cp, img_rgb, img_test;

  // Enumeration of cell types in the feature distribution grid.
  enum CellType
  {
    // Cell contains a visual map point.
    TYPE_MAP = 1,
    // Cell contains a LiDAR point cloud point.
    TYPE_POINTCLOUD,
    // Cell type has not been determined yet.
    TYPE_UNKNOWN
  };

  // Constructor: initializes VIO parameters and allocates resources.
  VIOManager();
  // Destructor: cleans up visual map, warp cache, and other resources.
  ~VIOManager();
  // Update the EKF state using the inverse compositional formulation at a given pyramid level.
  void updateStateInverse(cv::Mat img, int level);
  // Update the EKF state using the forward compositional formulation at a given pyramid level.
  void updateState(cv::Mat img, int level);
  // Main frame processing pipeline: associates visual and LiDAR data, updates EKF.
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  // Retrieve visual map points that project into the current frame from the sparse voxel map.
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  // Create new visual map points from LiDAR depth measurements and image features.
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);
  // Set the extrinsic calibration between the IMU and LiDAR sensor.
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);
  // Set the extrinsic calibration between the LiDAR and camera.
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  // Initialize the VIO system: set up camera, grid, and map structures.
  void initializeVIO();
  // Extract an image patch from the specified pyramid level at the given pixel coordinates.
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);
  // Compute the 2x3 Jacobian of the camera projection function at a 3D point.
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  // Compute the full photometric Jacobian and perform the EKF update.
  void computeJacobianAndUpdateEKF(cv::Mat img);
  // Reset the feature distribution grid for a new frame.
  void resetGrid();
  // Update the positions and descriptors of visual map points based on the current frame.
  void updateVisualMapPoints(cv::Mat img);
  // Compute the affine warp matrix from the reference frame to the current frame based on depth.
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3 &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
  // Compute the affine warp matrix using a homography for known surface normals.
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3 &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);
  // Warp a reference image patch to the current frame using an affine transformation.
  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  // Insert a new visual point into the voxel-based visual map.
  void insertPointIntoVoxelMap(VisualPoint *pt_new);
  // Visualize currently tracked visual points on the image (debug).
  void plotTrackedPoints();
  // Update the stored camera frame state with the latest EKF estimate.
  void updateFrameState(StatesGroup state);
  // Project reference patches into the current frame using the plane map for depth.
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  // Update the reference patches of visual map points after a successful EKF update.
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  // Precompute reference patches at the given pyramid level for all converged points.
  void precomputeReferencePatches(int level);
  // Export camera poses and image data to Colmap-compatible format.
  void dumpDataForColmap();
  // Compute the Normalized Cross-Correlation between two image patches.
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  // Determine the best pyramid search level from the affine warp matrix.
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  // Get the bilinearly interpolated pixel value from the image at sub-pixel coordinates.
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
// Closing brace for VIOManager class.
};
// Shared pointer type alias for VIOManager.
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

// End of header guard.
#endif // VIO_H_