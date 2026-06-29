/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Include the voxel map header defining VoxelOctoTree and VoxelMapManager classes
#include "voxel_map.h"

// Computes the 3x3 measurement covariance of a LiDAR point in the body frame,
// modeled as a combination of range error and angular (beam) uncertainty.
void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov)
{
// Prevent division by zero by setting a small positive z value
  if (pb[2] == 0) pb[2] = 0.0001;
// Compute the range (distance from origin to the point)
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
// Compute range variance as the square of range increment
  float range_var = range_inc * range_inc;
// Initialize a 2x2 matrix for angular direction variance
  Eigen::Matrix2d direction_var;
// Populate direction variance with angular uncertainty converted from degrees to radians
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
// Create a copy of the point vector for direction computation
  Eigen::Vector3d direction(pb);
// Normalize the direction vector to unit length
  direction.normalize();
// Initialize the skew-symmetric matrix of the direction vector
  Eigen::Matrix3d direction_hat;
// Form the skew-symmetric cross-product matrix from direction components
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
// Construct an arbitrary basis vector orthogonal to the direction
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
// Normalize the first basis vector
  base_vector1.normalize();
// Compute the second basis vector via cross product with direction
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
// Normalize the second basis vector
  base_vector2.normalize();
// Initialize a 3x2 matrix to hold the two basis vectors as columns
  Eigen::Matrix<double, 3, 2> N;
// Populate N with the two orthonormal basis vectors
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
// Compute the Jacobian mapping angular uncertainty to Cartesian space
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
// Assemble the full 3x3 covariance: range variance along direction plus angular variance in the tangent plane
  cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

// Loads voxel map configuration (max layer, voxel size, plane thresholds, local
// map sliding parameters) from the ROS parameter server.
void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config)
{
// Read flag for publishing plane map visualization
  nh.param<bool>("publish/pub_plane_en", voxel_config.is_pub_plane_map_, false);
  
// Read maximum voxel octree depth
  nh.param<int>("lio/max_layer", voxel_config.max_layer_, 1);
// Read maximum voxel size at the coarsest layer
  nh.param<double>("lio/voxel_size", voxel_config.max_voxel_size_, 0.5);
// Read minimum eigenvalue threshold for planarity detection
  nh.param<double>("lio/min_eigen_value", voxel_config.planner_threshold_, 0.01);
// Read sigma multiplier for Mahalanobis distance gating
  nh.param<double>("lio/sigma_num", voxel_config.sigma_num_, 3);
// Read beam angular error (degrees) for point covariance
  nh.param<double>("lio/beam_err", voxel_config.beam_err_, 0.02);
// Read depth (range) error for point covariance
  nh.param<double>("lio/dept_err", voxel_config.dept_err_, 0.05);
// Read per-layer initialization point count thresholds
  nh.param<vector<int>>("lio/layer_init_num", voxel_config.layer_init_num_, vector<int>{5,5,5,5,5});
// Read maximum points per voxel before freezing updates
  nh.param<int>("lio/max_points_num", voxel_config.max_points_num_, 50);
// Read maximum EKF iteration count
  nh.param<int>("lio/max_iterations", voxel_config.max_iterations_, 5);

// Read flag enabling sliding window map
  nh.param<bool>("local_map/map_sliding_en", voxel_config.map_sliding_en, false);
// Read half-size of the local map bounding box (in voxels)
  nh.param<int>("local_map/half_map_size", voxel_config.half_map_size, 100);
// Read distance threshold to trigger a map slide
  nh.param<double>("local_map/sliding_thresh", voxel_config.sliding_thresh, 8);
}

// Computes plane parameters (center, normal, eigenvalues, covariance, uncertainty)
// from a set of points via eigendecomposition. Marks the plane as valid if the
// smallest eigenvalue is below the planarity threshold.
void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
// Zero out the 6x6 plane parameter covariance matrix
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
// Zero out the 3x3 point covariance accumulator
  plane->covariance_ = Eigen::Matrix3d::Zero();
// Zero out the plane center accumulator
  plane->center_ = Eigen::Vector3d::Zero();
// Zero out the plane normal vector
  plane->normal_ = Eigen::Vector3d::Zero();
// Store the number of points in this voxel
  plane->points_size_ = points.size();
// Initialize radius to zero
  plane->radius_ = 0;
// Accumulate point covariance and center over all points
  for (auto pv : points)
  {
// Accumulate outer product of point position
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
// Accumulate point position sum for center computation
    plane->center_ += pv.point_w;
  }
// Compute mean center by dividing by point count
  plane->center_ = plane->center_ / plane->points_size_;
// Compute the sample covariance matrix of point positions
  plane->covariance_ = plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();
// Perform eigendecomposition of the covariance matrix
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
// Extract eigenvectors from the decomposition result
  Eigen::Matrix3cd evecs = es.eigenvectors();
// Extract eigenvalues from the decomposition result
  Eigen::Vector3cd evals = es.eigenvalues();
// Declare a real-valued eigenvalue vector
  Eigen::Vector3d evalsReal;
// Convert complex eigenvalues to real values
  evalsReal = evals.real();
// Declare indices for minimum and maximum eigenvalues
  Eigen::Matrix3f::Index evalsMin, evalsMax;
// Find the index of the smallest eigenvalue
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
// Find the index of the largest eigenvalue
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
// Compute the index of the middle eigenvalue (0+1+2 = 3)
  int evalsMid = 3 - evalsMin - evalsMax;
// Extract the eigenvector corresponding to the smallest eigenvalue
  Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
// Extract the eigenvector corresponding to the middle eigenvalue
  Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
// Extract the eigenvector corresponding to the largest eigenvalue
  Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
// Initialize the Jacobian of the center estimate w.r.t. point positions
  Eigen::Matrix3d J_Q;
// Populate J_Q as (1/N) * I_3x3
  J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_;
  // && evalsReal(evalsMid) > 0.05
  //&& evalsReal(evalsMid) > 0.01
// Check if the smallest eigenvalue is below the planarity threshold
  if (evalsReal(evalsMin) < planer_threshold_)
  {
// Loop over all points to compute the plane parameter Jacobian
    for (int i = 0; i < points.size(); i++)
    {
// Initialize the 6x3 Jacobian matrix (plane normal + center w.r.t. point position)
      Eigen::Matrix<double, 6, 3> J;
// Initialize the 3x3 auxiliary matrix F for derivative computation
      Eigen::Matrix3d F;
// Loop over the three eigenvector dimensions
      for (int m = 0; m < 3; m++)
      {
// Skip the minimum eigenvalue dimension (plane normal direction)
        if (m != (int)evalsMin)
        {
// Compute the m-th row of F via eigenvalue perturbation formula
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].point_w - plane->center_).transpose() / ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
              (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
// Assign the computed row to the auxiliary matrix F
          F.row(m) = F_m;
        }
// For the minimum eigenvalue dimension, set the row to zero
        else
        {
// Initialize a zero row vector
          Eigen::Matrix<double, 1, 3> F_m;
// Populate with zeros
          F_m << 0, 0, 0;
// Assign the zero row to the auxiliary matrix F
          F.row(m) = F_m;
        }
      }
// Compute the upper 3x3 block of J (normal part) via eigenvectors times F
      J.block<3, 3>(0, 0) = evecs.real() * F;
// Compute the lower 3x3 block of J (center part) as the identity scaled by 1/N
      J.block<3, 3>(3, 0) = J_Q;
// Accumulate the plane parameter covariance via error propagation (J * cov_point * J^T)
      plane->plane_var_ += J * points[i].var * J.transpose();
    }

// Set the plane normal from the smallest eigenvector (minimum variance direction)
    plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
// Set the secondary (y) direction from the middle eigenvector
    plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
// Set the primary (x) direction from the largest eigenvector
    plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
// Store the minimum eigenvalue
    plane->min_eigen_value_ = evalsReal(evalsMin);
// Store the middle eigenvalue
    plane->mid_eigen_value_ = evalsReal(evalsMid);
// Store the maximum eigenvalue
    plane->max_eigen_value_ = evalsReal(evalsMax);
// Compute plane radius as the square root of the maximum eigenvalue
    plane->radius_ = sqrt(evalsReal(evalsMax));
// Compute the plane offset d = -normal . center
    plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) + plane->normal_(2) * plane->center_(2));
// Mark this voxel as containing a valid plane
    plane->is_plane_ = true;
// Mark the plane as updated for publishing
    plane->is_update_ = true;
// Assign a unique ID to the plane on first initialization
    if (!plane->is_init_)
    {
// Assign the global voxel plane ID
      plane->id_ = voxel_plane_id;
// Increment the global ID counter
      voxel_plane_id++;
// Mark the plane as initialized
      plane->is_init_ = true;
    }
  }
  else
  {
// Mark the plane as updated (for non-planar voxels)
    plane->is_update_ = true;
// Mark the voxel as non-planar
    plane->is_plane_ = false;
  }
}

// Initializes the octree at this node: if enough points exist, attempts plane
// fitting. On success the node terminates; otherwise it subdivides into eight children.
void VoxelOctoTree::init_octo_tree()
{
// Check if accumulated points exceed the initialization threshold
  if (temp_points_.size() > points_size_threshold_)
  {
// Attempt plane fitting on the accumulated points
    init_plane(temp_points_, plane_ptr_);
// If plane fitting succeeded, mark the node as planar and finalize
    if (plane_ptr_->is_plane_ == true)
    {
// Set octree state to 0 (planar/terminal node)
      octo_state_ = 0;
      // new added
// If the point count exceeds the maximum, freeze the node and clear storage
      if (temp_points_.size() > max_points_num_)
      {
// Disable further updates to this node
        update_enable_ = false;
// Deallocate the point buffer by swapping with an empty vector
        std::vector<pointWithVar>().swap(temp_points_);
// Reset the new point counter
        new_points_ = 0;
      }
    }
    else
    {
// Set octree state to 1 (non-planar, requires subdivision)
      octo_state_ = 1;
// Recursively split into eight child octants
      cut_octo_tree();
    }
// Mark the octree as initialized
    init_octo_ = true;
// Reset the new point counter
    new_points_ = 0;
  }
}

// Recursively distributes points into eight child octants based on their position
// relative to the voxel center, then attempts plane fitting at each child.
void VoxelOctoTree::cut_octo_tree()
{
// If maximum depth reached, mark as planar terminal and return
  if (layer_ >= max_layer_)
  {
// Set octree state to 0 (terminal node at max depth)
    octo_state_ = 0;
    return;
  }
// Iterate over all points to distribute them into child octants
  for (size_t i = 0; i < temp_points_.size(); i++)
  {
// Binary encoding of the child octant index (0-7) based on position relative to center
    int xyz[3] = {0, 0, 0};
// Set x-bit: 1 if point x coordinate is greater than voxel center x
    if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
// Set y-bit: 1 if point y coordinate is greater than voxel center y
    if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
// Set z-bit: 1 if point z coordinate is greater than voxel center z
    if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
// Compute child index as a 3-bit binary number (4*x + 2*y + z)
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
// If the child octant does not exist yet, create it
    if (leaves_[leafnum] == nullptr)
    {
// Allocate a new VoxelOctoTree for this child with incremented layer
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
// Propagate the layer initialization counts to the child
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
// Set the child voxel center x coordinate (shifted by quarter length in direction)
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
// Set the child voxel center y coordinate
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
// Set the child voxel center z coordinate
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
// Halve the quarter length for the child
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
    }
// Copy the point into the child's temporary buffer
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
// Increment the child's new point counter
    leaves_[leafnum]->new_points_++;
  }
// After distribution, attempt plane fitting at each child that has enough points
  for (uint i = 0; i < 8; i++)
  {
// Check if the child octant exists
    if (leaves_[i] != nullptr)
    {
// Check if the child has enough points to attempt plane fitting
      if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_)
      {
// Attempt plane fitting on the child's points
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
// If the child is planar, mark it as terminal
        if (leaves_[i]->plane_ptr_->is_plane_)
        {
// Set octree state to 0 (planar terminal)
          leaves_[i]->octo_state_ = 0;
          // new added
// If the child exceeds max points, freeze its updates
          if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_)
          {
// Disable further point insertion into this child
            leaves_[i]->update_enable_ = false;
// Clear the child's point buffer to free memory
            std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
// Reset the new point counter
            new_points_ = 0;
          }
        }
        else
        {
// Mark the child as non-planar requiring further subdivision
          leaves_[i]->octo_state_ = 1;
// Recursively subdivide the child
          leaves_[i]->cut_octo_tree();
        }
// Mark the child as initialized
        leaves_[i]->init_octo_ = true;
// Reset the child's new point counter
        leaves_[i]->new_points_ = 0;
      }
    }
  }
}

// Incrementally updates the octree with a new point. If the node is initialized
// and planar, adds to its point buffer and re-fits periodically. Otherwise
// propagates the point into the appropriate child octant.
void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv)
{
// If the octree node has not been initialized yet
  if (!init_octo_)
  {
// Increment the new point counter
    new_points_++;
// Append the point to the temporary buffer
    temp_points_.push_back(pv);
// If enough points accumulated, initialize the octree
    if (temp_points_.size() > points_size_threshold_) { init_octo_tree(); }
  }
  else
  {
// If the node is planar, attempt to add the point directly
    if (plane_ptr_->is_plane_)
    {
// Check if updates are still enabled for this node
      if (update_enable_)
      {
// Increment the new point counter
        new_points_++;
// Append the point to the temporary buffer
        temp_points_.push_back(pv);
// If enough new points accumulated, re-fit the plane
        if (new_points_ > update_size_threshold_)
        {
// Recompute plane parameters with the updated point set
          init_plane(temp_points_, plane_ptr_);
// Reset the new point counter after re-fitting
          new_points_ = 0;
        }
// If the buffer is full, freeze the node
        if (temp_points_.size() >= max_points_num_)
        {
// Disable further updates
          update_enable_ = false;
// Deallocate the point buffer
          std::vector<pointWithVar>().swap(temp_points_);
// Reset the new point counter
          new_points_ = 0;
        }
      }
    }
    else
    {
// If the node is non-planar and below max depth, propagate the point to children
      if (layer_ < max_layer_)
      {
// Encode the child octant index based on point position relative to center
        int xyz[3] = {0, 0, 0};
// Determine x-bit of the child index
        if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
// Determine y-bit of the child index
        if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
// Determine z-bit of the child index
        if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
// Compute the 0-7 child index from the three bits
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
// If the child exists, recurse into it
        if (leaves_[leafnum] != nullptr) { leaves_[leafnum]->UpdateOctoTree(pv); }
        else
        {
// Otherwise, create a new child octree node
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
// Propagate the layer initialization counts to the child
          leaves_[leafnum]->layer_init_num_ = layer_init_num_;
// Set the child voxel center x coordinate
          leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
// Set the child voxel center y coordinate
          leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
// Set the child voxel center z coordinate
          leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
// Halve the quarter length for the child
          leaves_[leafnum]->quater_length_ = quater_length_ / 2;
// Insert the point into the newly created child
          leaves_[leafnum]->UpdateOctoTree(pv);
        }
      }
      else
      {
// If at max depth and updates enabled, buffer the point at this leaf
        if (update_enable_)
        {
// Increment the new point counter
          new_points_++;
// Append the point to the temporary buffer
          temp_points_.push_back(pv);
// If enough new points accumulated, re-fit the plane
          if (new_points_ > update_size_threshold_)
          {
// Recompute plane parameters
            init_plane(temp_points_, plane_ptr_);
// Reset the new point counter
            new_points_ = 0;
          }
// If the buffer is full, freeze this leaf
          if (temp_points_.size() > max_points_num_)
          {
// Disable further updates
            update_enable_ = false;
// Deallocate the point buffer
            std::vector<pointWithVar>().swap(temp_points_);
// Reset the new point counter
            new_points_ = 0;
          }
        }
      }
    }
  }
}

// Traverses the octree to find the deepest node containing the given world point.
// Returns the current node when a plane is found or max depth is reached.
VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw)
{
// If uninitialized, planar, or at max depth, return this node as the correspondence
  if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

// Encode the child index based on the query point position
  int xyz[3] = {0, 0, 0};
// Determine x-bit of the child index
  xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
// Determine y-bit of the child index
  xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
// Determine z-bit of the child index
  xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
// Compute the 0-7 child index
  int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

  // printf("leafnum: %d. \n", leafnum);

// Recurse into the child if it exists, otherwise return this node
  return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

// Inserts a point into the octree. If the node is non-planar and within depth
// limit, recurses into the appropriate child; otherwise appends to this node's buffer.
VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv)
{
// If uninitialized, planar, or at max depth, buffer the point at this node
  if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) || (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_)))
  {
// Increment the new point counter
    new_points_++;
// Append the point to the temporary buffer
    temp_points_.push_back(pv);
// Return this node as the insertion target
    return this;
  }

// If non-planar and below max depth, propagate to the appropriate child
  if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_))
  {
// Encode the child index from point position
    int xyz[3] = {0, 0, 0};
// Determine x-bit
    xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
// Determine y-bit
    xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
// Determine z-bit
    xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
// Compute child index
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
// Recurse into existing child or create a new one
    if (leaves_[leafnum] != nullptr) { return leaves_[leafnum]->Insert(pv); }
    else
    {
// Create a new child octree node
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
// Propagate layer initialization counts
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
// Set child center x
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
// Set child center y
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
// Set child center z
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
// Set child quarter length
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
// Insert the point into the new child
      return leaves_[leafnum]->Insert(pv);
    }
  }
// Return null if insertion path is invalid (should not be reached)
  return nullptr;
}

// Core iterative EKF update: computes point-to-plane residuals, builds the
// measurement Jacobian, runs iterated Kalman updates, checks convergence, and
// updates the state covariance.
void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{
// Clear the list of cross-product matrices from the previous iteration
  cross_mat_list_.clear();
// Reserve space for cross-product matrices matching the number of downsampled features
  cross_mat_list_.reserve(feats_down_size_);
// Clear the list of body-frame point covariances
  body_cov_list_.clear();
// Reserve space for body covariances
  body_cov_list_.reserve(feats_down_size_);

  // build_residual_time = 0.0;
  // ekf_time = 0.0;
  // double t0 = omp_get_wtime();

// Loop over all downsampled LiDAR points in the body frame
  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
// Extract the 3D point coordinates from the PCL point
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
// Prevent division by zero by setting a small positive z value
    if (point_this[2] == 0) { point_this[2] = 0.001; }
// Declare the 3x3 covariance matrix
    M3D var;
// Compute the body-frame measurement covariance for this point
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
// Store the body-frame covariance
    body_cov_list_.push_back(var);
// Transform the point from LiDAR frame to IMU/body frame using extrinsic calibration
    point_this = extR_ * point_this + extT_;
// Declare the cross-product matrix for the transformed point
    M3D point_crossmat;
// Compute the skew-symmetric cross-product matrix of the transformed point
    point_crossmat << SKEW_SYM_MATRX(point_this);
// Store the cross-product matrix for Jacobian computation
    cross_mat_list_.push_back(point_crossmat);
  }

// Clear and reallocate the world-frame point-with-variance list
  vector<pointWithVar>().swap(pv_list_);
// Resize the point list to match the number of downsampled features
  pv_list_.resize(feats_down_size_);

// Declare EKF intermediate matrices: gain matrix G, H^T*H, and identity
  int rematch_num = 0;
// Zero the gain matrix G
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
// Zero the H^T*H accumulator
  G.setZero();
// Set the identity matrix
  H_T_H.setZero();
  I_STATE.setIdentity();

// Initialize EKF flags for convergence and stop conditions
  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
// Begin the iterated EKF loop
  for (int iterCount = 0; iterCount < config_setting_.max_iterations_; iterCount++)
  {
// Accumulate total point-to-plane residual for convergence monitoring
    double total_residual = 0.0;
// Allocate a point cloud for world-frame transformed lidar points
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
// Transform all downsampled body-frame points to world frame using current state estimate
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
// Extract the rotation covariance sub-block from the state covariance
    M3D rot_var = state_.cov.block<3, 3>(0, 0);
// Extract the translation covariance sub-block from the state covariance
    M3D t_var = state_.cov.block<3, 3>(3, 3);
// Loop over all points to compute their world-frame positions and propagated covariances
    for (size_t i = 0; i < feats_down_body_->size(); i++)
    {
// Reference the current point-with-variance entry
      pointWithVar &pv = pv_list_[i];
// Store the body-frame coordinates of the point
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
// Store the world-frame coordinates of the point
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;

// Retrieve the body-frame covariance for this point
      M3D cov = body_cov_list_[i];
// Retrieve the cross-product matrix for this point
      M3D point_crossmat = cross_mat_list_[i];
// Propagate covariance from body frame to world frame, including rotation and translation uncertainty
      cov = state_.rot_end * cov * state_.rot_end.transpose() + (-point_crossmat) * rot_var * (-point_crossmat.transpose()) + t_var;
// Store the propagated world-frame covariance
      pv.var = cov;
// Store the original body-frame covariance
      pv.body_var = body_cov_list_[i];
    }
// Clear the point-to-plane residual list for this iteration
    ptpl_list_.clear();

    // double t1 = omp_get_wtime();

// Build the point-to-plane residual list in parallel using OpenMP
    BuildResidualListOMP(pv_list_, ptpl_list_);

    // build_residual_time += omp_get_wtime() - t1;

// Accumulate the absolute residuals for monitoring convergence
    for (int i = 0; i < ptpl_list_.size(); i++)
    {
// Add the absolute distance to the total residual
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
// Record the number of effective features (points with valid plane associations)
    effct_feat_num_ = ptpl_list_.size();
// Log feature statistics: raw count, downsampled count, effective count, average residual
    cout << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_ 
         << " effective feature num: " << effct_feat_num_ << " average residual: " << total_residual / effct_feat_num_ << endl;

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
// Declare the measurement Jacobian matrix H (size: effective features x 6)
    MatrixXd Hsub(effct_feat_num_, 6);
// Declare the transposed and inverse-variance weighted Jacobian H^T * R^{-1}
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_);
// Declare the inverse measurement variance vector R^{-1}
    VectorXd R_inv(effct_feat_num_);
// Declare the measurement innovation vector (negative residuals)
    VectorXd meas_vec(effct_feat_num_);
// Zero the measurement vector
    meas_vec.setZero();
// Loop over all effective features to fill the Jacobian and innovation
    for (int i = 0; i < effct_feat_num_; i++)
    {
// Reference the current point-to-plane residual entry
      auto &ptpl = ptpl_list_[i];
// Extract the body-frame point coordinates
      V3D point_this(ptpl.point_b_);
// Transform the point from LiDAR to IMU/body frame using extrinsic calibration
      point_this = extR_ * point_this + extT_;
// Extract a copy of the body-frame point
      V3D point_body(ptpl.point_b_);
// Declare the cross-product matrix
      M3D point_crossmat;
// Compute the skew-symmetric cross-product matrix of the extrinsically-transformed point
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/

// Compute the world-frame point position from the propagated state
      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end;
// Declare the Jacobian of the plane normal-distance constraint w.r.t. pose
      Eigen::Matrix<double, 1, 6> J_nq;
// Compute the position Jacobian block: vector from plane center to point
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
// Compute the orientation Jacobian block: negative plane normal
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;

// Compute the total measurement covariance combining rotation and extrinsic uncertainty
      M3D var;
// Propagate body covariance through rotation and extrinsic calibration
      var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose();

// Compute the predicted residual variance from plane uncertainty
      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();

// Compute the inverse innovation covariance (R^{-1}) with regularization
      R_inv(i) = 1.0 / (0.001 + sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      /*** calculate the Measuremnt Jacobian matrix H ***/
// Compute the oriented cross-product for the Jacobian row: A = (p x) * R^T * n
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
// Fill the H matrix row: [A^T, n^T] (6 elements)
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
// Fill the H^T * R^{-1} matrix column with weighted Jacobian entries
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
// Set the measurement innovation as the negative point-to-plane distance
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
    }
// Reset the EKF stop flag for this iteration
    EKF_stop_flg = false;
// Reset the convergence flag
    flg_EKF_converged = false;
    /*** Iterative Kalman Filter Update ***/
// Declare the Kalman gain matrix
    MatrixXd K(DIM_STATE, effct_feat_num_);
    // auto &&Hsub_T = Hsub.transpose();
// Compute the weighted innovation vector H^T * R^{-1} * z
    auto &&HTz = Hsub_T_R_inv * meas_vec;
    // fout_dbg<<"HTz: "<<HTz<<endl;
// Compute the H^T * R^{-1} * H block for the 6-DOF pose subspace
    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
// Compute the full-state Kalman gain: K_1 = (H^T*H + P^{-1})^{-1}
    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H.block<DIM_STATE, DIM_STATE>(0, 0) + state_.cov.block<DIM_STATE, DIM_STATE>(0, 0).inverse()).inverse();
// Compute the gain matrix G = K_1 * H^T*H (for covariance update)
    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
// Compute the state prediction error vector (propagated minus current state)
    auto vec = state_propagat - state_;
// Compute the state update solution: K_1 * H^T*R^{-1}*z + (I - G) * delta_x
    VD(DIM_STATE)
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
// Declare min indices (unused placeholder)
    int minRow, minCol;
// Apply the computed state correction
    state_ += solution;
// Extract the rotation update (3-DOF)
    auto rot_add = solution.block<3, 1>(0, 0);
// Extract the translation update (3-DOF)
    auto t_add = solution.block<3, 1>(3, 0);
// Check convergence: if both rotation and translation increments are small
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }
// Compute the Euler angles from the updated rotation matrix for publishing
    V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0);

    /*** Rematch Judgement ***/

// If converged or near max iterations, perform rematching to refine associations
    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2)))) { rematch_num++; }

    /*** Convergence Judgements and Covariance Update ***/
// If EKF should stop (converged or max iterations reached)
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1)))
    {
      /*** Covariance Update ***/
      // _state.cov = (I_STATE - G) * _state.cov;
// Update the state covariance using the Joseph form: (I - G) * P
      state_.cov.block<DIM_STATE, DIM_STATE>(0, 0) =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) - G.block<DIM_STATE, DIM_STATE>(0, 0)) * state_.cov.block<DIM_STATE, DIM_STATE>(0, 0);
      // total_distance += (_state.pos_end - position_last).norm();
// Store the current position for sliding window and logging
      position_last_ = state_.pos_end;
// Compute the quaternion from Euler angles for ROS message publishing
      geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
// Set the EKF stop flag to true
      EKF_stop_flg = true;
    }
// Break if EKF stop is requested
    if (EKF_stop_flg) break;
  }
}

// Transforms a LiDAR point cloud from body frame to a target frame using the
// given rotation and translation, including extrinsic calibration.
void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
// Clear the output cloud by swapping with an empty cloud
  pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
// Pre-allocate memory for the transformed cloud
  trans_cloud->reserve(input_cloud->size());
// Loop over all input points
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
// Access the current input point
    pcl::PointXYZINormal p_c = input_cloud->points[i];
// Convert the PCL point to an Eigen 3D vector
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
// Apply extrinsic rotation, extrinsic translation, then state rotation and translation
    p = (rot * (extR_ * p + extT_) + t);
// Create a new PCL point for the output cloud
    pcl::PointXYZI pi;
// Set the x coordinate
    pi.x = p(0);
// Set the y coordinate
    pi.y = p(1);
// Set the z coordinate
    pi.z = p(2);
// Copy the intensity value from the input point
    pi.intensity = p_c.intensity;
// Append the transformed point to the output cloud
    trans_cloud->points.push_back(pi);
  }
}

// Builds the initial voxel map from the first LiDAR scan: discretizes world-frame
// points into voxel keys, inserts them into VoxelOctoTree objects, and initializes
// the octrees by attempting plane fitting at each voxel.
void VoxelMapManager::BuildVoxelMap()
{
// Read the voxel size from configuration
  float voxel_size = config_setting_.max_voxel_size_;
// Read the planarity threshold from configuration
  float planer_threshold = config_setting_.planner_threshold_;
// Read the maximum octree depth from configuration
  int max_layer = config_setting_.max_layer_;
// Read the maximum points per voxel from configuration
  int max_points_num = config_setting_.max_points_num_;
// Read the per-layer initialization point thresholds from configuration
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;

// Create an empty list for world-frame points with variance
  std::vector<pointWithVar> input_points;

// Iterate over all downsampled world-frame points
  for (size_t i = 0; i < feats_down_world_->size(); i++)
  {
// Create a point-with-variance structure
    pointWithVar pv;
// Set the world-frame coordinates of the point
    pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
// Retrieve the corresponding body-frame point coordinates
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
// Declare the 3x3 measurement covariance matrix
    M3D var;
// Compute the body-frame point covariance using range and beam errors
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
// Declare the cross-product matrix for the body-frame point
    M3D point_crossmat;
// Compute the skew-symmetric cross-product matrix
    point_crossmat << SKEW_SYM_MATRX(point_this);
// Propagate the body covariance to world frame considering rotation and translation uncertainty
    var = (state_.rot_end * extR_) * var * (state_.rot_end * extR_).transpose() +
// Add the translation uncertainty contribution
          (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + state_.cov.block<3, 3>(3, 3);
// Store the propagated world-frame covariance
    pv.var = var;
// Append the point to the input list
    input_points.push_back(pv);
  }

// Get the total number of input points
  uint plsize = input_points.size();
// Iterate over all input points to assign them to voxels
  for (uint i = 0; i < plsize; i++)
  {
// Access the current point
    const pointWithVar p_v = input_points[i];
// Declare the discretized voxel coordinates
    float loc_xyz[3];
// Loop over x, y, z dimensions
    for (int j = 0; j < 3; j++)
    {
// Compute the discrete voxel coordinate by dividing by voxel size
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
// Adjust negative coordinates to ensure consistent floor behavior
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
// Create the 3D voxel location key as a tuple of integers
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
// Look up the voxel in the hash map
    auto iter = voxel_map_.find(position);
// If the voxel already exists, add the point to it
    if (iter != voxel_map_.end())
    {
// Append the point to the existing voxel's point buffer
      voxel_map_[position]->temp_points_.push_back(p_v);
// Increment the new point counter for the existing voxel
      voxel_map_[position]->new_points_++;
    }
// Otherwise, create a new voxel octree for this location
    else
    {
// Allocate a new VoxelOctoTree at layer 0 with configured parameters
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
// Insert the new octree into the voxel map
      voxel_map_[position] = octo_tree;
// Set the voxel quarter length as one quarter of the voxel size
      voxel_map_[position]->quater_length_ = voxel_size / 4;
// Compute the voxel center x coordinate from its discretized position
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
// Compute the voxel center y coordinate
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
// Compute the voxel center z coordinate
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
// Append the point to the new voxel
      voxel_map_[position]->temp_points_.push_back(p_v);
// Increment the new point counter
      voxel_map_[position]->new_points_++;
// Store the layer initialization count vector for this voxel
      voxel_map_[position]->layer_init_num_ = layer_init_num;
    }
  }
// After all points are inserted, initialize each voxel's octree
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter)
  {
// Initialize the octree (attempt plane fitting or subdivide)
    iter->second->init_octo_tree();
  }
}

// Returns a pseudo-color for a voxel based on its 3D grid coordinate, used for
// visualization of the voxel structure.
V3F VoxelMapManager::RGBFromVoxel(const V3D &input_point)
{
// Declare the discretized voxel coordinates
  int64_t loc_xyz[3];
// Loop over dimensions to compute voxel index
  for (int j = 0; j < 3; j++)
  {
// Floor the point coordinate divided by voxel size to get the integer voxel index
    loc_xyz[j] = floor(input_point[j] / config_setting_.max_voxel_size_);
  }

// Create the voxel location key
  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
// Compute a hash from the sum of the voxel coordinates
  int64_t ind = loc_xyz[0] + loc_xyz[1] + loc_xyz[2];
// Map the hash to one of three color channels (0, 1, 2)
  uint k((ind + 100000) % 3);
// Create an RGB vector with only one channel active based on the hash
  V3F RGB((k == 0) * 255.0, (k == 1) * 255.0, (k == 2) * 255.0);
  // cout<<"RGB: "<<RGB.transpose()<<endl;
  return RGB;
}

// Updates the voxel map with new points from the current scan. Each point is
// assigned to its voxel by world coordinate hashing, and the corresponding octree
// is incrementally updated.
void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
// Read voxel size from configuration
  float voxel_size = config_setting_.max_voxel_size_;
// Read planarity threshold from configuration
  float planer_threshold = config_setting_.planner_threshold_;
// Read maximum octree depth from configuration
  int max_layer = config_setting_.max_layer_;
// Read maximum points per voxel from configuration
  int max_points_num = config_setting_.max_points_num_;
// Read per-layer initialization point thresholds from configuration
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;
// Get the number of input points
  uint plsize = input_points.size();
// Loop over all input points
  for (uint i = 0; i < plsize; i++)
  {
// Access the current input point
    const pointWithVar p_v = input_points[i];
// Declare the discretized voxel coordinates
    float loc_xyz[3];
// Loop over x, y, z dimensions
    for (int j = 0; j < 3; j++)
    {
// Compute the discrete voxel coordinate
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
// Adjust negative coordinates for consistent floor behavior
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
// Create the voxel location key
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
// Look up the voxel in the hash map
    auto iter = voxel_map_.find(position);
// If the voxel exists, insert the point into its octree
    if (iter != voxel_map_.end()) { voxel_map_[position]->UpdateOctoTree(p_v); }
    else
    {
// Otherwise, create a new voxel octree
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
// Insert the new octree into the voxel map
      voxel_map_[position] = octo_tree;
// Store the layer initialization counts
      voxel_map_[position]->layer_init_num_ = layer_init_num;
// Set the voxel quarter length
      voxel_map_[position]->quater_length_ = voxel_size / 4;
// Compute the voxel center x coordinate
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
// Compute the voxel center y coordinate
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
// Compute the voxel center z coordinate
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
// Insert the point into the new voxel's octree
      voxel_map_[position]->UpdateOctoTree(p_v);
    }
  }
}

// For each downsampled LiDAR point, finds the corresponding voxel octree,
// computes the point-to-plane residual, and collects successful associations into
// a list for the EKF update. Uses OpenMP for parallel processing.
void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
// Read the maximum octree depth from configuration
  int max_layer = config_setting_.max_layer_;
// Read the voxel size from configuration
  double voxel_size = config_setting_.max_voxel_size_;
// Read the sigma multiplier for Mahalanobis gating from configuration
  double sigma_num = config_setting_.sigma_num_;
// Create a mutex for thread-safe access to shared data
  std::mutex mylock;
// Clear the output point-to-plane list
  ptpl_list.clear();
// Pre-allocate the full list of point-to-plane results for all points
  std::vector<PointToPlane> all_ptpl_list(pv_list.size());
// Pre-allocate the flag array indicating successful associations
  std::vector<bool> useful_ptpl(pv_list.size());
// Pre-allocate the index array for parallel processing
  std::vector<size_t> index(pv_list.size());
// Initialize the index array and useful flags
  for (size_t i = 0; i < index.size(); ++i)
  {
// Set the index value
    index[i] = i;
// Initialize the useful flag to false
    useful_ptpl[i] = false;
  }
// Enable OpenMP parallel processing if the MP_EN macro is defined
  #ifdef MP_EN
// Set the number of OpenMP threads
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
  #endif
// Parallel loop over all downsampled points
  for (int i = 0; i < index.size(); i++)
  {
// Reference the current point-with-variance entry
    pointWithVar &pv = pv_list[i];
// Declare the discretized voxel coordinates
    float loc_xyz[3];
// Loop over dimensions
    for (int j = 0; j < 3; j++)
    {
// Compute the discrete voxel coordinate
      loc_xyz[j] = pv.point_w[j] / voxel_size;
// Adjust negative coordinates for consistent floor behavior
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
// Create the voxel location key
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
// Look up the voxel in the hash map
    auto iter = voxel_map_.find(position);
// If the voxel exists, attempt to build a point-to-plane residual
    if (iter != voxel_map_.end())
    {
// Get the octree for this voxel
      VoxelOctoTree *current_octo = iter->second;
// Create a point-to-plane residual structure for this point
      PointToPlane single_ptpl;
// Flag indicating whether a valid plane association was found
      bool is_sucess = false;
// Probability of the best plane association (for multi-hypothesis selection)
      double prob = 0;
// Recursively search the octree for the best plane association
      build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl);
// If no association found, check the neighboring voxel for boundary points
      if (!is_sucess)
      {
// Start with the current voxel position
        VOXEL_LOCATION near_position = position;
// If the point is outside the voxel center + quarter length in x, shift to the neighboring voxel
        if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) { near_position.x = near_position.x + 1; }
// If the point is below the voxel center - quarter length in x, shift to the neighboring voxel
        else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) { near_position.x = near_position.x - 1; }
// If outside in y, shift to neighboring voxel
        if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) { near_position.y = near_position.y + 1; }
// If below in y, shift to neighboring voxel
        else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) { near_position.y = near_position.y - 1; }
// If outside in z, shift to neighboring voxel
        if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) { near_position.z = near_position.z + 1; }
// If below in z, shift to neighboring voxel
        else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) { near_position.z = near_position.z - 1; }
// Look up the neighboring voxel
        auto iter_near = voxel_map_.find(near_position);
// If the neighboring voxel exists, attempt plane association there
        if (iter_near != voxel_map_.end()) { build_single_residual(pv, iter_near->second, 0, is_sucess, prob, single_ptpl); }
      }
// If a valid association was found, store the result
      if (is_sucess)
      {
// Lock the mutex for thread-safe write
        mylock.lock();
// Mark this point as having a valid association
        useful_ptpl[i] = true;
// Store the point-to-plane data
        all_ptpl_list[i] = single_ptpl;
// Unlock the mutex
        mylock.unlock();
      }
// Otherwise, mark as invalid
      else
      {
// Lock the mutex
        mylock.lock();
// Mark this point as having no valid association
        useful_ptpl[i] = false;
// Unlock the mutex
        mylock.unlock();
      }
    }
  }
// Collect all valid associations into the output list
  for (size_t i = 0; i < useful_ptpl.size(); i++)
  {
// If valid, append to the output list
    if (useful_ptpl[i]) { ptpl_list.push_back(all_ptpl_list[i]); }
  }
}

// Recursively computes the point-to-plane residual for a single point against the
// planes in an octree branch. Selects the candidate with the highest probability
// (Gaussian likelihood) within a Mahalanobis distance threshold.
void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess,
                                            double &prob, PointToPlane &single_ptpl)
{
// Read the maximum octree depth from configuration
  int max_layer = config_setting_.max_layer_;
// Read the sigma multiplier for Mahalanobis gating from configuration
  double sigma_num = config_setting_.sigma_num_;

// Set the radius multiplier for plane proximity checking
  double radius_k = 3;
// Extract the world-frame point position
  Eigen::Vector3d p_w = pv.point_w;
// If the current octree node has a valid plane
  if (current_octo->plane_ptr_->is_plane_)
  {
// Reference the plane object for convenience
    VoxelPlane &plane = *current_octo->plane_ptr_;
// Compute the vector from plane center to the query point
    Eigen::Vector3d p_world_to_center = p_w - plane.center_;
// Compute the absolute point-to-plane distance using the plane equation
    float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);
// Compute the squared distance from point to plane center
    float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                          (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
// Compute the tangential distance from point to plane center (in the plane)
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

// Check if the point is within the plane radius (times the radius multiplier)
    if (range_dis <= radius_k * plane.radius_)
    {
// Declare the Jacobian of the point-to-plane distance w.r.t. plane parameters
      Eigen::Matrix<double, 1, 6> J_nq;
// Position component of the Jacobian: vector from center to point
      J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
// Normal component of the Jacobian: negative normal direction
      J_nq.block<1, 3>(0, 3) = -plane.normal_;
// Compute the predicted variance of the residual from plane parameter uncertainty
      double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
// Add the point's own measurement uncertainty projected onto the normal direction
      sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
// Check Mahalanobis distance: reject if residual exceeds sigma_num * sigma
      if (dis_to_plane < sigma_num * sqrt(sigma_l))
      {
// Mark the association as successful
        is_sucess = true;
// Compute the Gaussian likelihood of this association
        double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
// If this probability is higher than the previous best, update the selection
        if (this_prob > prob)
// Store the new best probability
        {
// Store the world-frame normal
          prob = this_prob;
// Copy body-frame covariance to the residual structure
          pv.normal = plane.normal_;
// Copy body-frame point coordinates
          single_ptpl.body_cov_ = pv.body_var;
// Copy world-frame point coordinates
          single_ptpl.point_b_ = pv.point_b;
// Copy plane parameter covariance
          single_ptpl.point_w_ = pv.point_w;
// Copy plane normal
          single_ptpl.plane_var_ = plane.plane_var_;
// Copy plane center
          single_ptpl.normal_ = plane.normal_;
// Copy plane offset d
          single_ptpl.center_ = plane.center_;
// Copy the current octree layer for diagnostics
          single_ptpl.d_ = plane.d_;
// Compute the signed point-to-plane distance and store it
          single_ptpl.layer_ = current_layer;
          single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        }
        return;
      }
      else
      {
        // is_sucess = false;
        return;
      }
    }
    else
    {
      // is_sucess = false;
      return;
    }
  }
  else
// If this node has no plane, recurse into its children
  {
// Check if maximum depth has not been reached
    if (current_layer < max_layer)
    {
// Loop over all eight child octants
      for (size_t leafnum = 0; leafnum < 8; leafnum++)
      {
// Check if the child octant exists
        if (current_octo->leaves_[leafnum] != nullptr)
        {

// Reference the child octree node
          VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
// Recursively search the child for a plane association
          build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
        }
      }
      return;
    }
    else { return; }
  }
}

// Publishes all voxel planes as a ROS MarkerArray for RViz visualization, with
// color mapping based on plane covariance trace.
void VoxelMapManager::pubVoxelMap()
{
// Set the maximum trace value for color normalization
  double max_trace = 0.25;
// Set the power exponent for color mapping
  double pow_num = 0.2;
// Create a rate limiter at 500 Hz
  ros::Rate loop(500);
// Set the alpha transparency for plane visualization
  float use_alpha = 0.8;
// Create a marker array message for RViz
  visualization_msgs::MarkerArray voxel_plane;
// Reserve space for up to 1 million markers
  voxel_plane.markers.reserve(1000000);
// Create a list to collect all planes for publishing
  std::vector<VoxelPlane> pub_plane_list;
// Iterate over all voxels in the map
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
  {
// Recursively collect updated planes from this voxel's octree
    GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
  }
// Iterate over all collected planes
  for (size_t i = 0; i < pub_plane_list.size(); i++)
  {
// Extract the diagonal of the position covariance block (3x3)
    V3D plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
// Compute the trace of the plane position covariance
    double trace = plane_cov.sum();
// Clamp the trace to the maximum value for color normalization
    if (trace >= max_trace) { trace = max_trace; }
// Normalize the trace to the [0, 1] range
    trace = trace * (1.0 / max_trace);
// Apply power scaling for enhanced color contrast
    trace = pow(trace, pow_num);
// Declare RGB color channels
    uint8_t r, g, b;
// Map the normalized trace value to a jet colormap color
    mapJet(trace, 0, 1, r, g, b);
// Convert RGB from 8-bit to floating point in [0, 1]
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
// Declare the alpha value for this plane
    double alpha;
// Set alpha to the user-specified value if the plane is valid
    if (pub_plane_list[i].is_plane_) { alpha = use_alpha; }
// Set alpha to zero (invisible) for non-planar voxels
    else { alpha = 0; }
// Publish the single plane marker
    pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
  }
// Publish the complete marker array to ROS
  voxel_map_pub_.publish(voxel_plane);
// Sleep to maintain the configured publishing rate
  loop.sleep();
}

// Recursively collects all planes that have been updated since the last publish,
// up to the specified voxel layer depth.
void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list)
{
// If the current layer exceeds the max publish layer, stop recursion
  if (current_octo->layer_ > pub_max_voxel_layer) { return; }
// If the plane has been updated since last publish, add it to the list
  if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
// If the node can have children (below max layer)
  if (current_octo->layer_ < current_octo->max_layer_)
  {
// Only recurse if the node is non-planar (has children)
    if (!current_octo->plane_ptr_->is_plane_)
    {
// Loop over all eight child octants
      for (size_t i = 0; i < 8; i++)
      {
// Recursively collect planes from valid children
        if (current_octo->leaves_[i] != nullptr) { GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list); }
      }
    }
  }
  return;
}

// Converts a VoxelPlane into a CYLINDER marker and adds it to the marker array
// for RViz visualization.
void VoxelMapManager::pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane,
                                     const float alpha, const Eigen::Vector3d rgb)
{
// Create a new Marker message for this plane
  visualization_msgs::Marker plane;
// Set the frame ID to the global initialization frame
  plane.header.frame_id = "camera_init";
// Set the timestamp to current ROS time
  plane.header.stamp = ros::Time();
// Set the namespace for the marker
  plane.ns = plane_ns;
// Set the unique ID for this marker
  plane.id = single_plane.id_;
// Set the marker type to CYLINDER for plane visualization
  plane.type = visualization_msgs::Marker::CYLINDER;
// Set the action to ADD (create or update)
  plane.action = visualization_msgs::Marker::ADD;
// Set the x position of the marker center
  plane.pose.position.x = single_plane.center_[0];
// Set the y position of the marker center
  plane.pose.position.y = single_plane.center_[1];
// Set the z position of the marker center
  plane.pose.position.z = single_plane.center_[2];
// Declare a quaternion for marker orientation
  geometry_msgs::Quaternion q;
// Compute quaternion from the three orthonormal plane axes
  CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
// Assign the orientation to the marker
  plane.pose.orientation = q;
// Set the marker scale along the x-axis (proportional to max eigenvalue)
  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_);
// Set the marker scale along the y-axis (proportional to mid eigenvalue)
  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
// Set the marker scale along the z-axis (proportional to min eigenvalue, plane normal)
  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
// Set the alpha (transparency) of the marker
  plane.color.a = alpha;
// Set the red color component
  plane.color.r = rgb(0);
// Set the green color component
  plane.color.g = rgb(1);
// Set the blue color component
  plane.color.b = rgb(2);
// Set the marker lifetime to forever
  plane.lifetime = ros::Duration();
// Append the marker to the marker array
  plane_pub.markers.push_back(plane);
}

// Computes a quaternion from three orthonormal basis vectors, used to orient
// the cylinder marker with the plane normal direction.
void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                                      geometry_msgs::Quaternion &q)
{
// Create a 3x3 rotation matrix from the three basis vectors
  Eigen::Matrix3d rot;
// Populate the rotation matrix rows from the x, y, z axis vectors
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
// Transpose to get the world-to-local rotation matrix
  Eigen::Matrix3d rotation = rot.transpose();
// Convert the rotation matrix to a quaternion
  Eigen::Quaterniond eq(rotation);
// Set the quaternion w component
  q.w = eq.w();
// Set the quaternion x component
  q.x = eq.x();
// Set the quaternion y component
  q.y = eq.y();
// Set the quaternion z component
  q.z = eq.z();
}

// Maps a scalar value to an RGB color using the jet colormap, used for
// visualizing plane uncertainty in RViz.
void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b)
{
// Initialize all channels to white
  r = 255;
// Initialize green channel
  g = 255;
// Initialize blue channel
  b = 255;

// Clamp the value to the minimum bound
  if (v < vmin) { v = vmin; }

// Clamp the value to the maximum bound
  if (v > vmax) { v = vmax; }

// Declare floating-point color components
  double dr, dg, db;

// First segment: blue ramps up from 0.504 to 1.0, red and green stay at 0
  if (v < 0.1242)
  {
// Compute blue channel ramp
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
// Red and green remain zero
    dg = dr = 0.;
  }
// Second segment: blue stays at 1, green ramps up, red stays at 0
  else if (v < 0.3747)
  {
// Blue at maximum
    db = 1.;
// Red at zero
    dr = 0.;
// Compute green channel ramp
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  }
// Third segment: blue ramps down, green stays at 1, red ramps up
  else if (v < 0.6253)
  {
// Compute blue channel ramp down
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
// Green at maximum
    dg = 1.;
// Compute red channel ramp up
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  }
// Fourth segment: blue at 0, red stays at 1, green ramps down
  else if (v < 0.8758)
  {
// Blue at zero
    db = 0.;
// Red at maximum
    dr = 1.;
// Compute green channel ramp down
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  }
// Final segment: blue at 0, green at 0, red ramps down from 1 to 0.504
  else
  {
// Blue at zero
    db = 0.;
// Green at zero
    dg = 0.;
// Compute red channel ramp down
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

// Convert floating-point red to 8-bit
  r = (uint8_t)(255 * dr);
// Convert floating-point green to 8-bit
  g = (uint8_t)(255 * dg);
// Convert floating-point blue to 8-bit
  b = (uint8_t)(255 * db);
}

// Sliding window map management: removes voxels outside a half-map-size bounding
// box around the current position, triggered when the robot moves beyond a threshold.
void VoxelMapManager::mapSliding()
{
// Check if the robot has moved far enough to trigger a slide
  if((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh)
  {
// Log the distance since last slide
    std::cout<<RED<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
// Return without sliding if below threshold
    return;
  }

  //get global id now
// Update the last slide position to the current position
  last_slide_position = position_last_;
// Record the start time for performance measurement
  double t_sliding_start = omp_get_wtime();
// Declare the discretized position coordinates
  float loc_xyz[3];
// Loop over dimensions
  for (int j = 0; j < 3; j++)
  {
// Compute the discrete voxel coordinate of the current position
    loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
// Adjust negative coordinates for consistent floor behavior
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
// Delete all voxels outside the half-map-size bounding box around the current position
  clearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size, (int64_t)loc_xyz[0] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[1] + config_setting_.half_map_size, (int64_t)loc_xyz[1] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[2] + config_setting_.half_map_size, (int64_t)loc_xyz[2] - config_setting_.half_map_size);
// Record the end time for performance measurement
  double t_sliding_end = omp_get_wtime();
// Log the time taken for map sliding
  std::cout<<RED<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
  return;
}

// Deletes all root-level voxels whose discrete coordinates fall outside the
// specified axis-aligned bounding box, freeing memory for the sliding map.
void VoxelMapManager::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
// Counter for the number of deleted voxels
  int delete_voxel_cout = 0;
  // double delete_time = 0;
  // double last_delete_time = 0;
// Iterate over all voxels in the map
  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); )
  {
// Get the voxel location key
    const VOXEL_LOCATION& loc = it->first;
// Determine if the voxel lies outside the axis-aligned bounding box
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
// If the voxel is outside the bounding box, delete it
    if (should_remove){
      // last_delete_time = omp_get_wtime();
// Free the octree memory
      delete it->second;
// Erase the voxel from the map and advance the iterator
      it = voxel_map_.erase(it);
      // delete_time += omp_get_wtime() - last_delete_time;
// Increment the deletion counter
      delete_voxel_cout++;
// Otherwise, advance the iterator normally
    } else {
      ++it;
    }
  }
// Log the number of deleted voxels
  std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<RESET<<"\n";
  // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}