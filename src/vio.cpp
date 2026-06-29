/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "vio.h"

// Constructor: default initialization, resources are allocated in initializeVIO.
VIOManager::VIOManager()
{
  // downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
}

// Destructor: releases the visual submap, warp map entries, and feature map entries.
VIOManager::~VIOManager()
{
  // Deletes the visual submap pointer.
  delete visual_submap;
  // Iterates over the warp map and deletes each entry.
  for (auto& pair : warp_map) delete pair.second;
  // Clears the warp map container.
  warp_map.clear();
  // Iterates over the feature map and deletes each entry.
  for (auto& pair : feat_map) delete pair.second;
  // Clears the feature map container.
  feat_map.clear();
}

// Sets the extrinsic calibration from IMU to LiDAR (rotation and translation).
void VIOManager::setImuToLidarExtrinsic(const V3D &transl, const M3D &rot)
{
  // Computes LiDAR position in IMU frame.
  Pli = -rot.transpose() * transl;
  // Computes LiDAR rotation in IMU frame.
  Rli = rot.transpose();
}

// Sets the extrinsic calibration from LiDAR to camera (rotation and translation).
void VIOManager::setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P)
{
  // Converts rotation vector to rotation matrix.
  Rcl << MAT_FROM_ARRAY(R);
  // Converts translation vector.
  Pcl << VEC_FROM_ARRAY(P);
}

// Initializes VIO: computes compound extrinsics, sets up grid parameters,
// precomputes raycasting sample points, opens colmap output files, and resizes
// grid/retrieval buffers.
void VIOManager::initializeVIO()
{
  // Allocates the visual submap structure.
  visual_submap = new SubSparseMap;

  // Retrieves camera intrinsic parameters.
  fx = cam->fx();
  fy = cam->fy();
  cx = cam->cx();
  cy = cam->cy();
  // Gets the image resize factor from the camera.
  image_resize_factor = cam->scale();

  // Prints the camera intrinsics for debugging.
  printf("intrinsic: %.6lf, %.6lf, %.6lf, %.6lf\n", fx, fy, cx, cy);

  // Retrieves image dimensions from the camera.
  width = cam->width();
  height = cam->height();

  // Prints the image dimensions and resize factor.
  printf("width: %d, height: %d, scale: %f\n", width, height, image_resize_factor);
  // Computes the rotation from IMU to camera frame.
  Rci = Rcl * Rli;
  // Computes the translation from IMU to camera frame.
  Pci = Rcl * Pli + Pcl;

  // Temporary variables for Jacobian precomputation.
  V3D Pic;
  M3D tmp;
  // Precomputes Jacobian for rotation update.
  Jdphi_dR = Rci;
  // Computes camera position in IMU frame.
  Pic = -Rci.transpose() * Pci;
  // Creates skew-symmetric matrix for position.
  tmp << SKEW_SYM_MATRX(Pic);
  // Precomputes Jacobian for position update.
  Jdp_dR = -Rci * tmp;

  // Computes the grid dimensions for the image.
  if (grid_size > 10)
  {
    // Computes grid width from the specified grid size.
    grid_n_width = ceil(static_cast<double>(width / grid_size));
    // Computes grid height from the specified grid size.
    grid_n_height = ceil(static_cast<double>(height / grid_size));
  }
  else
  {
    // Computes grid size from the specified grid dimensions.
    grid_size = static_cast<int>(height / grid_n_height);
    // Recomputes grid height from the adjusted grid size.
    grid_n_height = ceil(static_cast<double>(height / grid_size));
    // Recomputes grid width from the adjusted grid size.
    grid_n_width = ceil(static_cast<double>(width / grid_size));
  }
  // Total number of grid cells.
  length = grid_n_width * grid_n_height;

  // Precomputes raycasting sample points if enabled.
  if(raycast_en)
  {
    // Resizes the border flag vector.
    border_flag.resize(length, 0);

    // Releases old ray data and reserves space.
    std::vector<std::vector<V3D>>().swap(rays_with_sample_points);
    rays_with_sample_points.reserve(length);
    // Prints the grid configuration.
    printf("grid_size: %d, grid_n_height: %d, grid_n_width: %d, length: %d\n", grid_size, grid_n_height, grid_n_width, length);

    // Raycasting sample parameters.
    float d_min = 0.1;
    float d_max = 3.0;
    float step = 0.2;
    // Iterates over each grid row.
    for (int grid_row = 1; grid_row <= grid_n_height; grid_row++)
    {
      // Iterates over each grid column.
      for (int grid_col = 1; grid_col <= grid_n_width; grid_col++)
      {
        // Container for sample points in this grid cell.
        std::vector<V3D> SamplePointsEachGrid;
        // Computes the linear index of this grid cell.
        int index = (grid_row - 1) * grid_n_width + grid_col - 1;

        // Marks border cells.
        if (grid_row == 1 || grid_col == 1 || grid_row == grid_n_height || grid_col == grid_n_width) border_flag[index] = 1;

        // Computes the pixel center of this grid cell.
        int u = grid_size / 2 + (grid_col - 1) * grid_size;
        int v = grid_size / 2 + (grid_row - 1) * grid_size;
        // Samples points along the ray at different depths.
        for (float d_temp = d_min; d_temp <= d_max; d_temp += step)
        {
          // 3D sample point along the ray.
          V3D xyz;
          // Back-projects pixel to 3D ray direction.
          xyz = cam->cam2world(u, v);
          // Scales the ray to the current depth.
          xyz *= d_temp / xyz[2];
          // Adds the sample point to the grid cell list.
          SamplePointsEachGrid.push_back(xyz);
        }
        // Stores the sample points for this grid cell.
        rays_with_sample_points.push_back(SamplePointsEachGrid);
      }
    }
  }

  if(colmap_output_en)
  {
    pinhole_cam = dynamic_cast<vk::PinholeCamera*>(cam);
    fout_colmap.open(DEBUG_FILE_DIR("Colmap/sparse/0/images.txt"), ios::out);
    fout_colmap << "# Image list with two lines of data per image:\n";
    fout_colmap << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n";
    fout_colmap << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
    fout_camera.open(DEBUG_FILE_DIR("Colmap/sparse/0/cameras.txt"), ios::out);
    fout_camera << "# Camera list with one line of data per camera:\n";
    fout_camera << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
    fout_camera << "1 PINHOLE " << width << " " << height << " "
        << std::fixed << std::setprecision(6)  // 控制浮点数精度为10位
        << fx << " " << fy << " "
        << cx << " " << cy << std::endl;
    fout_camera.close();
  }
  // Resizes the grid cell type buffer.
  grid_num.resize(length);
  // Resizes the map point index buffer.
  map_index.resize(length);
  // Resizes the map point distance buffer.
  map_dist.resize(length);
  // Resizes the update flag buffer.
  update_flag.resize(length);
  // Resizes the scan value buffer.
  scan_value.resize(length);

  // Computes patch pixel count.
  patch_size_total = patch_size * patch_size;
  // Computes half patch size.
  patch_size_half = static_cast<int>(patch_size / 2);
  // Resizes the patch buffer.
  patch_buffer.resize(patch_size_total);
  // Computes warped patch length across pyramid levels.
  warp_len = patch_size_total * patch_pyrimid_level;
  // Computes the image border margin for patches.
  border = (patch_size_half + 1) * (1 << patch_pyrimid_level);

  // Reserves space for retrieval points per grid cell.
  retrieve_voxel_points.reserve(length);
  // Reserves space for append points per grid cell.
  append_voxel_points.reserve(length);

  // Clears the sub-feature map.
  sub_feat_map.clear();
}

// Resets all grid-level buffers (type, index, distance, update flag, scan value)
// and clears retrieval/append point vectors for a new frame.
void VIOManager::resetGrid()
{
  // Resets all grid cells to unknown type.
  fill(grid_num.begin(), grid_num.end(), TYPE_UNKNOWN);
  // Resets all map indices to zero.
  fill(map_index.begin(), map_index.end(), 0);
  // Resets all map distances to a large value.
  fill(map_dist.begin(), map_dist.end(), 10000.0f);
  // Resets all update flags to zero.
  fill(update_flag.begin(), update_flag.end(), 0);
  // Resets all scan values to zero.
  fill(scan_value.begin(), scan_value.end(), 0.0f);

  // Clears the retrieval points per grid cell.
  retrieve_voxel_points.clear();
  retrieve_voxel_points.resize(length);

  // Clears the append points per grid cell.
  append_voxel_points.clear();
  append_voxel_points.resize(length);

  // Resets the total point counter.
  total_points = 0;
}

// Computes the 2x3 Jacobian of the camera projection function with respect to
// a 3D point in camera coordinates.
void VIOManager::computeProjectionJacobian(V3D p, MD(2, 3) & J)
{
  // X coordinate in camera frame.
  const double x = p[0];
  // Y coordinate in camera frame.
  const double y = p[1];
  // Inverse of depth.
  const double z_inv = 1. / p[2];
  // Squared inverse depth.
  const double z_inv_2 = z_inv * z_inv;
  // Jacobian entry for du/dx.
  J(0, 0) = fx * z_inv;
  // Jacobian entry for du/dy.
  J(0, 1) = 0.0;
  // Jacobian entry for du/dz.
  J(0, 2) = -fx * x * z_inv_2;
  // Jacobian entry for dv/dx.
  J(1, 0) = 0.0;
  // Jacobian entry for dv/dy.
  J(1, 1) = fy * z_inv;
  // Jacobian entry for dv/dz.
  J(1, 2) = -fy * y * z_inv_2;
}

// Extracts a bilinearly interpolated image patch at the given pixel coordinate
// and pyramid level, storing the result in patch_tmp.
void VIOManager::getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level)
{
  // Reference pixel u coordinate.
  const float u_ref = pc[0];
  // Reference pixel v coordinate.
  const float v_ref = pc[1];
  // Scale factor for the current pyramid level.
  const int scale = (1 << level);
  // Integer u coordinate at the current scale.
  const int u_ref_i = floorf(pc[0] / scale) * scale;
  // Integer v coordinate at the current scale.
  const int v_ref_i = floorf(pc[1] / scale) * scale;
  // Subpixel offset in u direction.
  const float subpix_u_ref = (u_ref - u_ref_i) / scale;
  // Subpixel offset in v direction.
  const float subpix_v_ref = (v_ref - v_ref_i) / scale;
  // Bilinear interpolation weight for top-left neighbor.
  const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
  // Bilinear interpolation weight for top-right neighbor.
  const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
  // Bilinear interpolation weight for bottom-left neighbor.
  const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
  // Bilinear interpolation weight for bottom-right neighbor.
  const float w_ref_br = subpix_u_ref * subpix_v_ref;
  // Iterates over patch rows.
  for (int x = 0; x < patch_size; x++)
  {
    // Pointer to the start of the row in the image.
    uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i - patch_size_half * scale + x * scale) * width + (u_ref_i - patch_size_half * scale);
    // Iterates over patch columns.
    for (int y = 0; y < patch_size; y++, img_ptr += scale)
    {
      // Bilinear interpolation of the pixel value.
      patch_tmp[patch_size_total * level + x * patch_size + y] =
          w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] + w_ref_bl * img_ptr[scale * width] + w_ref_br * img_ptr[scale * width + scale];
    }
  }
}

// Inserts a newly created VisualPoint into the visual feature map using voxel
// hashing (0.5 m voxel size).
void VIOManager::insertPointIntoVoxelMap(VisualPoint *pt_new)
{
  // Point position in world coordinates.
  V3D pt_w(pt_new->pos_[0], pt_new->pos_[1], pt_new->pos_[2]);
  // Voxel size for spatial hashing.
  double voxel_size = 0.5;
  // Voxel grid coordinates.
  float loc_xyz[3];
  // Computes voxel coordinates for each axis.
  for (int j = 0; j < 3; j++)
  {
    // Computes the voxel index.
    loc_xyz[j] = pt_w[j] / voxel_size;
    // Adjusts negative indices.
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // Creates the voxel location key.
  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  // Looks up the voxel in the feature map.
  auto iter = feat_map.find(position);
  // If the voxel already exists.
  if (iter != feat_map.end())
  {
    // Appends the point to the existing voxel.
    iter->second->voxel_points.push_back(pt_new);
    // Increments the point count.
    iter->second->count++;
  }
  // If the voxel does not exist yet.
  else
  {
    // Creates a new voxel point container.
    VOXEL_POINTS *ot = new VOXEL_POINTS(0);
    // Adds the point to the new voxel.
    ot->voxel_points.push_back(pt_new);
    // Inserts the voxel into the feature map.
    feat_map[position] = ot;
  }
}

// Computes the affine warp matrix using a homography induced by a planar surface
// with known normal, enabling photometric alignment between reference and current views.
void VIOManager::getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref, const V3D &xyz_ref, const V3D &normal_ref,
                                                  const SE3 &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref)
{
  // create homography matrix
  // Translation from current to reference frame.
  const V3D t = T_cur_ref.inverse().translation();
  // Homography matrix from the plane-induced transformation.
  const Eigen::Matrix3d H_cur_ref =
      T_cur_ref.rotation_matrix() * (normal_ref.dot(xyz_ref) * Eigen::Matrix3d::Identity() - t * normal_ref.transpose());
  // Compute affine warp matrix A_ref_cur using homography projection
  // Patch half-size for derivative computation.
  const int kHalfPatchSize = 4;
  // Ray direction at u-offset in reference frame.
  V3D f_du_ref(cam.cam2world(px_ref + Eigen::Vector2d(kHalfPatchSize, 0) * (1 << level_ref)));
  // Ray direction at v-offset in reference frame.
  V3D f_dv_ref(cam.cam2world(px_ref + Eigen::Vector2d(0, kHalfPatchSize) * (1 << level_ref)));
  //   f_du_ref = f_du_ref/f_du_ref[2];
  //   f_dv_ref = f_dv_ref/f_dv_ref[2];
  // Warped reference point in current frame.
  const V3D f_cur(H_cur_ref * xyz_ref);
  // Warped u-offset point in current frame.
  const V3D f_du_cur = H_cur_ref * f_du_ref;
  // Warped v-offset point in current frame.
  const V3D f_dv_cur = H_cur_ref * f_dv_ref;
  // Projected pixel of reference point.
  V2D px_cur(cam.world2cam(f_cur));
  // Projected pixel of u-offset point.
  V2D px_du_cur(cam.world2cam(f_du_cur));
  // Projected pixel of v-offset point.
  V2D px_dv_cur(cam.world2cam(f_dv_cur));
  // First column of affine warp matrix.
  A_cur_ref.col(0) = (px_du_cur - px_cur) / kHalfPatchSize;
  // Second column of affine warp matrix.
  A_cur_ref.col(1) = (px_dv_cur - px_cur) / kHalfPatchSize;
}

// Computes the affine warp matrix from a reference view to the current view
// using depth and the relative SE3 transformation.
void VIOManager::getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref,
                                        const SE3 &T_cur_ref, const int level_ref, const int pyramid_level, const int halfpatch_size,
                                        Matrix2d &A_cur_ref)
{
  // Compute affine warp matrix A_ref_cur
  // Computes the 3D reference point from ray and depth.
  const Vector3d xyz_ref(f_ref * depth_ref);
  // Ray direction at u-offset in reference frame.
  Vector3d xyz_du_ref(cam.cam2world(px_ref + Vector2d(halfpatch_size, 0) * (1 << level_ref) * (1 << pyramid_level)));
  // Ray direction at v-offset in reference frame.
  Vector3d xyz_dv_ref(cam.cam2world(px_ref + Vector2d(0, halfpatch_size) * (1 << level_ref) * (1 << pyramid_level)));
  // Scales u-offset ray to the reference depth.
  xyz_du_ref *= xyz_ref[2] / xyz_du_ref[2];
  // Scales v-offset ray to the reference depth.
  xyz_dv_ref *= xyz_ref[2] / xyz_dv_ref[2];
  // Projected pixel of reference point in current frame.
  const Vector2d px_cur(cam.world2cam(T_cur_ref * (xyz_ref)));
  // Projected pixel of u-offset in current frame.
  const Vector2d px_du(cam.world2cam(T_cur_ref * (xyz_du_ref)));
  // Projected pixel of v-offset in current frame.
  const Vector2d px_dv(cam.world2cam(T_cur_ref * (xyz_dv_ref)));
  // First column of affine warp matrix.
  A_cur_ref.col(0) = (px_du - px_cur) / halfpatch_size;
  // Second column of affine warp matrix.
  A_cur_ref.col(1) = (px_dv - px_cur) / halfpatch_size;
}

// Warps the reference image patch into the current frame using the affine
// transformation A_cur_ref and bilinear interpolation.
void VIOManager::warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                               const int pyramid_level, const int halfpatch_size, float *patch)
{
  // Full patch size (twice the half-size).
  const int patch_size = halfpatch_size * 2;
  // Inverse affine transformation for reverse warping.
  const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  // Checks if the warp matrix is valid.
  if (isnan(A_ref_cur(0, 0)))
  {
    // Reports NaN warp (likely no translation).
    printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
    return;
  }

  // Pointer to the output patch buffer.
  float *patch_ptr = patch;
  // Iterates over patch rows.
  for (int y = 0; y < patch_size; ++y)
  {
    // Iterates over patch columns.
    for (int x = 0; x < patch_size; ++x) //, ++patch_ptr)
    {
      // Pixel offset from patch center.
      Vector2f px_patch(x - halfpatch_size, y - halfpatch_size);
      // Applies search level scaling.
      px_patch *= (1 << search_level);
      // Applies pyramid level scaling.
      px_patch *= (1 << pyramid_level);
      // Warped pixel coordinate in reference image.
      const Vector2f px(A_ref_cur * px_patch + px_ref.cast<float>());
      // Checks if the warped pixel is within image bounds.
      if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 || px[1] >= img_ref.rows - 1)
        // Sets out-of-bounds pixels to zero.
        patch_ptr[patch_size_total * pyramid_level + y * patch_size + x] = 0;
      else
        // Bilinearly interpolates the pixel value.
        patch_ptr[patch_size_total * pyramid_level + y * patch_size + x] = (float)vk::interpolateMat_8u(img_ref, px[0], px[1]);
    }
  }
}

// Selects the coarsest pyramid search level based on the determinant of the
// affine warp matrix, ensuring the patch area change stays below a threshold.
int VIOManager::getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level)
{
  // Compute patch level in other image
  // Initial search level starts at zero.
  int search_level = 0;
  // Determinant of the affine warp matrix (area scaling).
  double D = A_cur_ref.determinant();
  // Increases search level while the area change is above threshold.
  while (D > 3.0 && search_level < max_level)
  {
    // Moves to the next coarser level.
    search_level += 1;
    // Each coarser level reduces area by factor 4.
    D *= 0.25;
  }
  // Returns the selected search level.
  return search_level;
}

// Computes the normalized cross-correlation (NCC) between a reference patch and
// a current patch for outlier rejection.
double VIOManager::calculateNCC(float *ref_patch, float *cur_patch, int patch_size)
{
  // Sum of reference patch intensities.
  double sum_ref = std::accumulate(ref_patch, ref_patch + patch_size, 0.0);
  // Mean of reference patch intensities.
  double mean_ref = sum_ref / patch_size;

  // Sum of current patch intensities.
  double sum_cur = std::accumulate(cur_patch, cur_patch + patch_size, 0.0);
  // Mean of current patch intensities.
  double mean_curr = sum_cur / patch_size;

  // Accumulators for NCC computation.
  double numerator = 0, demoniator1 = 0, demoniator2 = 0;
  // Iterates over all pixels in the patch.
  for (int i = 0; i < patch_size; i++)
  {
    // Cross product of mean-subtracted values.
    double n = (ref_patch[i] - mean_ref) * (cur_patch[i] - mean_curr);
    // Accumulates the numerator.
    numerator += n;
    // Accumulates reference variance.
    demoniator1 += (ref_patch[i] - mean_ref) * (ref_patch[i] - mean_ref);
    // Accumulates current variance.
    demoniator2 += (cur_patch[i] - mean_curr) * (cur_patch[i] - mean_curr);
  }
  // Returns the NCC score with epsilon for numerical stability.
  return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);
}

// Projects LiDAR points into the current frame to build a depth map, then
// retrieves visual map points whose projections fall within view. Optionally
// performs raycasting to discover occluded voxels.
void VIOManager::retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
{
  if (feat_map.size() <= 0) return;
  double ts0 = omp_get_wtime();

  visual_submap->reset();

  sub_feat_map.clear();

  float voxel_size = 0.5;

  if (!normal_en) warp_map.clear();

  cv::Mat depth_img = cv::Mat::zeros(height, width, CV_32FC1);
  float *it = (float *)depth_img.data;

  int loc_xyz[3];

  for (int i = 0; i < pg.size(); i++)
  {
    V3D pt_w = pg[i].point_w;

    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = floor(pt_w[j] / voxel_size);
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);

    auto iter = sub_feat_map.find(position);
    if (iter == sub_feat_map.end()) { sub_feat_map[position] = 0; }
    else { iter->second = 0; }

    V3D pt_c(new_frame_->w2f(pt_w));

    if (pt_c[2] > 0)
    {
      V2D px;
      px = new_frame_->cam_->world2cam(pt_c);

      if (new_frame_->cam_->isInFrame(px.cast<int>(), border))
      {
        float depth = pt_c[2];
        int col = int(px[0]);
        int row = int(px[1]);
        it[width * row + col] = depth;
      }
    }
  }

  vector<VOXEL_LOCATION> DeleteKeyList;

  for (auto &iter : sub_feat_map)
  {
    VOXEL_LOCATION position = iter.first;

    auto corre_voxel = feat_map.find(position);

    if (corre_voxel != feat_map.end())
    {
      bool voxel_in_fov = false;
      std::vector<VisualPoint *> &voxel_points = corre_voxel->second->voxel_points;
      int voxel_num = voxel_points.size();

      for (int i = 0; i < voxel_num; i++)
      {
        VisualPoint *pt = voxel_points[i];
        if (pt == nullptr) continue;
        if (pt->obs_.size() == 0) continue;

        V3D norm_vec(new_frame_->T_f_w_.rotation_matrix() * pt->normal_);
        V3D dir(new_frame_->T_f_w_ * pt->pos_);
        if (dir[2] < 0) continue;

        V2D pc(new_frame_->w2c(pt->pos_));
        if (new_frame_->cam_->isInFrame(pc.cast<int>(), border))
        {
          voxel_in_fov = true;
          int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);
          grid_num[index] = TYPE_MAP;
          Vector3d obs_vec(new_frame_->pos() - pt->pos_);
          float cur_dist = obs_vec.norm();
          if (cur_dist <= map_dist[index])
          {
            map_dist[index] = cur_dist;
            retrieve_voxel_points[index] = pt;
          }
        }
      }
      if (!voxel_in_fov) { DeleteKeyList.push_back(position); }
    }
  }

  // RayCasting Module
  if (raycast_en)
  {
    for (int i = 0; i < length; i++)
    {
      if (grid_num[i] == TYPE_MAP || border_flag[i] == 1) continue;

      for (const auto &it : rays_with_sample_points[i])
      {
        V3D sample_point_w = new_frame_->f2w(it);
        for (int j = 0; j < 3; j++)
        {
          loc_xyz[j] = floor(sample_point_w[j] / voxel_size);
          if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
        }

        VOXEL_LOCATION sample_pos(loc_xyz[0], loc_xyz[1], loc_xyz[2]);

        auto corre_sub_feat_map = sub_feat_map.find(sample_pos);
        if (corre_sub_feat_map != sub_feat_map.end()) break;

        auto corre_feat_map = feat_map.find(sample_pos);
        if (corre_feat_map != feat_map.end())
        {
          bool voxel_in_fov = false;

          std::vector<VisualPoint *> &voxel_points = corre_feat_map->second->voxel_points;
          int voxel_num = voxel_points.size();
          if (voxel_num == 0) continue;

          for (int j = 0; j < voxel_num; j++)
          {
            VisualPoint *pt = voxel_points[j];

            if (pt == nullptr) continue;
            if (pt->obs_.size() == 0) continue;

            V3D norm_vec(new_frame_->T_f_w_.rotation_matrix() * pt->normal_);
            V3D dir(new_frame_->T_f_w_ * pt->pos_);
            if (dir[2] < 0) continue;
            dir.normalize();

            V2D pc(new_frame_->w2c(pt->pos_));

            if (new_frame_->cam_->isInFrame(pc.cast<int>(), border))
            {
              voxel_in_fov = true;
              int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);
              grid_num[index] = TYPE_MAP;
              Vector3d obs_vec(new_frame_->pos() - pt->pos_);

              float cur_dist = obs_vec.norm();

              if (cur_dist <= map_dist[index])
              {
                map_dist[index] = cur_dist;
                retrieve_voxel_points[index] = pt;
              }
            }
          }

          if (voxel_in_fov) sub_feat_map[sample_pos] = 0;
          break;
        }
        else
        {
          VOXEL_LOCATION sample_pos(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
          auto iter = plane_map.find(sample_pos);
          if (iter != plane_map.end())
          {
            VoxelOctoTree *current_octo;
            current_octo = iter->second->find_correspond(sample_point_w);
            if (current_octo->plane_ptr_->is_plane_)
            {
              pointWithVar plane_center;
              VoxelPlane &plane = *current_octo->plane_ptr_;
              plane_center.point_w = plane.center_;
              plane_center.normal = plane.normal_;
              visual_submap->add_from_voxel_map.push_back(plane_center);
              break;
            }
          }
        }
      }
    }
  }

  for (auto &key : DeleteKeyList)
  {
    sub_feat_map.erase(key);
  }

  for (int i = 0; i < length; i++)
  {
    if (grid_num[i] == TYPE_MAP)
    {
      VisualPoint *pt = retrieve_voxel_points[i];

      V2D pc(new_frame_->w2c(pt->pos_));

      // Converts point to camera frame for depth check.
      V3D pt_cam(new_frame_->w2f(pt->pos_));
      // Flag for depth discontinuity detection.
      bool depth_continous = false;
      // Checks depth consistency within the patch neighborhood.
      for (int u = -patch_size_half; u <= patch_size_half; u++)
      {
        for (int v = -patch_size_half; v <= patch_size_half; v++)
        {
          // Skips the center pixel.
          if (u == 0 && v == 0) continue;

          // Reads the LiDAR depth at the neighbor pixel.
          float depth = it[width * (v + int(pc[1])) + u + int(pc[0])];

          // Skips if no depth data.
          if (depth == 0.) continue;

          // Computes depth difference.
          double delta_dist = abs(pt_cam[2] - depth);

          // Flags depth discontinuity if difference exceeds threshold.
          if (delta_dist > 0.5)
          {
            depth_continous = true;
            break;
          }
        }
        if (depth_continous) break;
      }
      // Skips points with depth discontinuities.
      if (depth_continous) continue;

      // Reference feature for photometric matching.
      Feature *ref_ftr;
      // Buffer for the warped reference patch.
      std::vector<float> patch_wrap(warp_len);

      // Search level and affine warp matrix.
      int search_level;
      Matrix2d A_cur_ref_zero;

      // Skips if normal is not initialized.
      if (!pt->is_normal_initialized_) continue;

      // Selects reference patch using normal-based or view-based method.
      if (normal_en)
      {
        // Minimum photometric error across candidate patches.
        float phtometric_errors_min = std::numeric_limits<float>::max();

        // If only one observation exists, use it directly.
        if (pt->obs_.size() == 1)
        {
          ref_ftr = *pt->obs_.begin();
          pt->ref_patch = ref_ftr;
          pt->has_ref_patch_ = true;
        }
        // Otherwise, selects the patch with minimum cross-reprojection error.
        else if (!pt->has_ref_patch_)
        {
          for (auto it = pt->obs_.begin(), ite = pt->obs_.end(); it != ite; ++it)
          {
            Feature *ref_patch_temp = *it;
            float *patch_temp = ref_patch_temp->patch_;
            float phtometric_errors = 0.0;
            int count = 0;
            for (auto itm = pt->obs_.begin(), itme = pt->obs_.end(); itm != itme; ++itm)
            {
              if ((*itm)->id_ == ref_patch_temp->id_) continue;
              float *patch_cache = (*itm)->patch_;

              for (int ind = 0; ind < patch_size_total; ind++)
              {
                phtometric_errors += (patch_temp[ind] - patch_cache[ind]) * (patch_temp[ind] - patch_cache[ind]);
              }
              count++;
            }
            phtometric_errors = phtometric_errors / count;
            if (phtometric_errors < phtometric_errors_min)
            {
              phtometric_errors_min = phtometric_errors;
              ref_ftr = ref_patch_temp;
            }
          }
          pt->ref_patch = ref_ftr;
          pt->has_ref_patch_ = true;
        }
        else { ref_ftr = pt->ref_patch; }
      }
      // Uses view-angle-based closest observation selection.
      else
      {
        if (!pt->getCloseViewObs(new_frame_->pos(), ref_ftr, pc)) continue;
      }

      // Computes affine warp using normal-based homography.
      if (normal_en)
      {
        // Normal vector rotated into reference frame.
        V3D norm_vec = (ref_ftr->T_f_w_.rotation_matrix() * pt->normal_).normalized();
        
        // Point position in reference frame.
        V3D pf(ref_ftr->T_f_w_ * pt->pos_);

        // Relative pose from reference to current frame.
        SE3 T_cur_ref = new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse();

        // Computes the affine warp via homography.
        getWarpMatrixAffineHomography(*cam, ref_ftr->px_, pf, norm_vec, T_cur_ref, 0, A_cur_ref_zero);

        // Determines the best pyramid search level.
        search_level = getBestSearchLevel(A_cur_ref_zero, 2);
      }
      // Uses cached or depth-based affine warp computation.
      else
      {
        auto iter_warp = warp_map.find(ref_ftr->id_);
        if (iter_warp != warp_map.end())
        {
          // Uses cached warp parameters.
          search_level = iter_warp->second->search_level;
          A_cur_ref_zero = iter_warp->second->A_cur_ref;
        }
        else
        {
          // Computes affine warp from depth and relative pose.
          getWarpMatrixAffine(*cam, ref_ftr->px_, ref_ftr->f_, (ref_ftr->pos() - pt->pos_).norm(), new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse(),
                              ref_ftr->level_, 0, patch_size_half, A_cur_ref_zero);

          // Determines the best pyramid search level.
          search_level = getBestSearchLevel(A_cur_ref_zero, 2);

          // Caches the warp parameters for reuse.
          Warp *ot = new Warp(search_level, A_cur_ref_zero);
          warp_map[ref_ftr->id_] = ot;
        }
      }

      // Warps reference patch across pyramid levels.
      for (int pyramid_level = 0; pyramid_level <= patch_pyrimid_level - 1; pyramid_level++)
      {
        warpAffine(A_cur_ref_zero, ref_ftr->img_, ref_ftr->px_, ref_ftr->level_, search_level, pyramid_level, patch_size_half, patch_wrap.data());
      }

      // Extracts the image patch at the current pixel location.
      getImagePatch(img, pc, patch_buffer.data(), 0);

      // Accumulates photometric error with exposure compensation.
      float error = 0.0;
      for (int ind = 0; ind < patch_size_total; ind++)
      {
        error += (ref_ftr->inv_expo_time_ * patch_wrap[ind] - state->inv_expo_time * patch_buffer[ind]) *
                 (ref_ftr->inv_expo_time_ * patch_wrap[ind] - state->inv_expo_time * patch_buffer[ind]);
      }

      // Optional NCC-based outlier rejection.
      if (ncc_en)
      {
        double ncc = calculateNCC(patch_wrap.data(), patch_buffer.data(), patch_size_total);
        if (ncc < ncc_thre)
        {
          continue;
        }
      }

      // Rejects outliers based on photometric error threshold.
      if (error > outlier_threshold * patch_size_total) continue;

      // Adds the point and its error to the visual submap.
      visual_submap->voxel_points.push_back(pt);
      visual_submap->propa_errors.push_back(error);
      visual_submap->search_levels.push_back(search_level);
      visual_submap->errors.push_back(error);
      visual_submap->warp_patch.push_back(patch_wrap);
      visual_submap->inv_expo_list.push_back(ref_ftr->inv_expo_time_);
    }
  }
  // Total number of retrieved visual points.
  total_points = visual_submap->voxel_points.size();
  printf("[ VIO ] Retrieve %d points from visual sparse map\n", total_points);
}

// Runs the photometric EKF update across all pyramid levels from coarse to fine.
// Switches between inverse-compositional and forward-additive update strategies.
void VIOManager::computeJacobianAndUpdateEKF(cv::Mat img)
{
  // Returns early if no visual points are available.
  if (total_points == 0) return;
  
  // Resets timing accumulators.
  compute_jacobian_time = update_ekf_time = 0.0;

  // Iterates pyramid levels from coarse to fine.
  for (int level = patch_pyrimid_level - 1; level >= 0; level--)
  {
    // Selects update strategy based on configuration.
    if (inverse_composition_en)
    {
      // Resets precomputed reference patch cache.
      has_ref_patch_cache = false;
      // Inverse-compositional EKF update.
      updateStateInverse(img, level);
    }
    else
      // Forward-additive EKF update.
      updateState(img, level);
  }
  // Applies the covariance update.
  state->cov -= G * state->cov;
  // Updates the frame state from the EKF state.
  updateFrameState(*state);
}

// Creates new VisualPoint objects from LiDAR points with valid normals that
// project into the current frame but are not already covered by map points.
void VIOManager::generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg)
{
  // Requires at least 10 points to proceed.
  if (pg.size() <= 10) return;

  // double t0 = omp_get_wtime();
  // Iterates over all LiDAR points with uncertainty.
  for (int i = 0; i < pg.size(); i++)
  {
    // Skips points without valid normals.
    if (pg[i].normal == V3D(0, 0, 0)) continue;

    // Transforms point to world coordinates.
    V3D pt = pg[i].point_w;
    // Projects to current frame pixel coordinates.
    V2D pc(new_frame_->w2c(pt));

    // Checks if the point is within the image frame (with border margin).
    if (new_frame_->cam_->isInFrame(pc.cast<int>(), border)) // 20px is the patch size in the matcher
    {
      // Computes the grid cell index.
      int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);

      // Only considers cells not already occupied by map points.
      if (grid_num[index] != TYPE_MAP)
      {
        // Computes the Shi-Tomasi corner score.
        float cur_value = vk::shiTomasiScore(img, pc[0], pc[1]);
        // if (cur_value < 5) continue;
        // Retains the best score per grid cell.
        if (cur_value > scan_value[index])
        {
          // Stores the best point for this cell.
          scan_value[index] = cur_value;
          append_voxel_points[index] = pg[i];
          grid_num[index] = TYPE_POINTCLOUD;
        }
      }
    }
  }

  // Also processes plane centers from the voxel map.
  for (int j = 0; j < visual_submap->add_from_voxel_map.size(); j++)
  {
    V3D pt = visual_submap->add_from_voxel_map[j].point_w;
    V2D pc(new_frame_->w2c(pt));

    if (new_frame_->cam_->isInFrame(pc.cast<int>(), border)) // 20px is the patch size in the matcher
    {
      int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);

      if (grid_num[index] != TYPE_MAP)
      {
        float cur_value = vk::shiTomasiScore(img, pc[0], pc[1]);
        if (cur_value > scan_value[index])
        {
          scan_value[index] = cur_value;
          append_voxel_points[index] = visual_submap->add_from_voxel_map[j];
          grid_num[index] = TYPE_POINTCLOUD;
        }
      }
    }
  }

  // Creates new VisualPoints for the best point in each grid cell.
  int add = 0;
  for (int i = 0; i < length; i++)
  {
    if (grid_num[i] == TYPE_POINTCLOUD) // && (scan_value[i]>=50))
    {
      pointWithVar pt_var = append_voxel_points[i];
      V3D pt = pt_var.point_w;

      // Computes the viewing direction relative to the normal.
      V3D norm_vec(new_frame_->T_f_w_.rotation_matrix() * pt_var.normal);
      V3D dir(new_frame_->T_f_w_ * pt);
      dir.normalize();
      double cos_theta = dir.dot(norm_vec);
      V2D pc(new_frame_->w2c(pt));

      // Extracts the image patch at this pixel.
      float *patch = new float[patch_size_total];
      getImagePatch(img, pc, patch, 0);

      // Creates a new visual point.
      VisualPoint *pt_new = new VisualPoint(pt);

      // Creates the first feature observation for this point.
      Vector3d f = cam->cam2world(pc);
      Feature *ftr_new = new Feature(pt_new, patch, pc, f, new_frame_->T_f_w_, 0);
      ftr_new->img_ = img;
      ftr_new->id_ = new_frame_->id_;
      ftr_new->inv_expo_time_ = state->inv_expo_time;

      // Adds the feature reference to the point.
      pt_new->addFrameRef(ftr_new);
      // Stores the point covariance.
      pt_new->covariance_ = pt_var.var;
      pt_new->is_normal_initialized_ = true;

      // Ensures the normal points toward the camera.
      if (cos_theta < 0) { pt_new->normal_ = -pt_var.normal; }
      else { pt_new->normal_ = pt_var.normal; }
      
      pt_new->previous_normal_ = pt_new->normal_;

      // Inserts the point into the visual map voxel structure.
      insertPointIntoVoxelMap(pt_new);
      add += 1;
    }
  }
  printf("[ VIO ] Append %d new visual map points\n", add);
}

// Adds new feature observations to existing visual points if the viewpoint has
// changed sufficiently (distance, angle, or pixel displacement threshold).
void VIOManager::updateVisualMapPoints(cv::Mat img)
{
  // Returns early if no points are available.
  if (total_points == 0) return;

  // Counter for updated points.
  int update_num = 0;
  // Current frame pose.
  SE3 pose_cur = new_frame_->T_f_w_;
  // Iterates over all visual points in the submap.
  for (int i = 0; i < total_points; i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];
    // Skips null points.
    if (pt == nullptr) continue;
    // For converged points, removes non-reference features.
    if (pt->is_converged_)
    { 
      pt->deleteNonRefPatchFeatures();
      continue;
    }

    // Projects point to current frame.
    V2D pc(new_frame_->w2c(pt->pos_));
    bool add_flag = false;
    
    // Extracts the image patch at this pixel.
    float *patch_temp = new float[patch_size_total];
    getImagePatch(img, pc, patch_temp, 0);
    // TODO: condition: distance and view_angle
    // Step 1: time
    // Gets the most recent feature observation.
    Feature *last_feature = pt->obs_.back();

    // Step 2: delta_pose
    // Computes the relative pose change.
    SE3 pose_ref = last_feature->T_f_w_;
    SE3 delta_pose = pose_ref * pose_cur.inverse();
    // Translation magnitude.
    double delta_p = delta_pose.translation().norm();
    // Rotation angle magnitude.
    double delta_theta = (delta_pose.rotation_matrix().trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (delta_pose.rotation_matrix().trace() - 1));
    // Adds observation if translation or rotation exceeds threshold.
    if (delta_p > 0.5 || delta_theta > 0.3) add_flag = true; // 0.5 || 0.3

    // Step 3: pixel distance
    // Computes pixel distance from last observation.
    Vector2d last_px = last_feature->px_;
    double pixel_dist = (pc - last_px).norm();
    // Adds observation if pixel displacement is large.
    if (pixel_dist > 40) add_flag = true;

    // Maintain the size of 3D point observation features.
    // Limits the number of stored observations per point.
    if (pt->obs_.size() >= 30)
    {
      Feature *ref_ftr;
      pt->findMinScoreFeature(new_frame_->pos(), ref_ftr);
      pt->deleteFeatureRef(ref_ftr);
    }
    // Creates a new feature observation if conditions are met.
    if (add_flag)
    {
      update_num += 1;
      update_flag[i] = 1;
      // Back-projects pixel to ray direction.
      Vector3d f = cam->cam2world(pc);
      // Creates the new feature observation.
      Feature *ftr_new = new Feature(pt, patch_temp, pc, f, new_frame_->T_f_w_, visual_submap->search_levels[i]);
      ftr_new->img_ = img;
      ftr_new->id_ = new_frame_->id_;
      ftr_new->inv_expo_time_ = state->inv_expo_time;
      // Adds the observation to the visual point.
      pt->addFrameRef(ftr_new);
    }
  }
  printf("[ VIO ] Update %d points in visual submap\n", update_num);
}

// Refines the surface normal estimate of each visual point using the LiDAR plane
// map, selects the best reference patch by NCC + view-angle score, and marks
// converged points.
void VIOManager::updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
{
  // Returns early if no points are available.
  if (total_points == 0) return;

  // Iterates over all visual points in the submap.
  for (int i = 0; i < visual_submap->voxel_points.size(); i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];

    // Skips points without initialized normals.
    if (!pt->is_normal_initialized_) continue;
    // Skips already converged points.
    if (pt->is_converged_) continue;
    // Requires at least 5 observations.
    if (pt->obs_.size() <= 5) continue;
    // Only processes points that were updated this frame.
    if (update_flag[i] == 0) continue;

    // Looks up the LiDAR plane map for normal refinement.
    const V3D &p_w = pt->pos_;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_w[j] / 0.5;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = plane_map.find(position);
    // If a plane voxel is found, refines the normal.
    if (iter != plane_map.end())
    {
      VoxelOctoTree *current_octo;
      current_octo = iter->second->find_correspond(p_w);
      if (current_octo->plane_ptr_->is_plane_)
      {
        VoxelPlane &plane = *current_octo->plane_ptr_;
        // Distance from point to plane.
        float dis_to_plane = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        float dis_to_plane_abs = fabs(dis_to_plane);
        // Squared distance from point to plane center.
        float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
                              (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) + (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);
        // Checks if point is within the plane extent.
        if (range_dis <= 3 * plane.radius_)
        {
          // Jacobian for plane uncertainty computation.
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
          J_nq.block<1, 3>(0, 3) = -plane.normal_;
          // Plane uncertainty propagated through Jacobian.
          double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
          // Adds point covariance along normal direction.
          sigma_l += plane.normal_.transpose() * pt->covariance_ * plane.normal_;

          // Chi-squared test: checks if point is consistent with the plane.
          if (dis_to_plane_abs < 3 * sqrt(sigma_l))
          {
            // Updates the normal direction.
            if (pt->previous_normal_.dot(plane.normal_) < 0) { pt->normal_ = -plane.normal_; }
            else { pt->normal_ = plane.normal_; }

            // Computes the normal update magnitude.
            double normal_update = (pt->normal_ - pt->previous_normal_).norm();

            pt->previous_normal_ = pt->normal_;

            // Marks point as converged if normal is stable.
            if (normal_update < 0.0001 && pt->obs_.size() > 10)
            {
              pt->is_converged_ = true;
              // visual_converged_point.push_back(pt);
            }
          }
        }
      }
    }

    // Selects the best reference patch based on NCC score and view angle.
    float score_max = -1000.;
    for (auto it = pt->obs_.begin(), ite = pt->obs_.end(); it != ite; ++it)
    {
      Feature *ref_patch_temp = *it;
      float *patch_temp = ref_patch_temp->patch_;
      // NCC accumulators.
      float NCC_up = 0.0;
      float NCC_down1 = 0.0;
      float NCC_down2 = 0.0;
      float NCC = 0.0;
      float score = 0.0;
      int count = 0;

      // Computes the viewing angle relative to the surface normal.
      V3D pf = ref_patch_temp->T_f_w_ * pt->pos_;
      V3D norm_vec = ref_patch_temp->T_f_w_.rotation_matrix() * pt->normal_;
      pf.normalize();
      double cos_angle = pf.dot(norm_vec);
      // if(fabs(cos_angle) < 0.86) continue; // 20 degree

      // Computes reference patch mean if not cached.
      float ref_mean;
      if (abs(ref_patch_temp->mean_) < 1e-6)
      {
        float ref_sum = std::accumulate(patch_temp, patch_temp + patch_size_total, 0.0);
        ref_mean = ref_sum / patch_size_total;
        ref_patch_temp->mean_ = ref_mean;
      }

      // Computes NCC between this feature and all other observations.
      for (auto itm = pt->obs_.begin(), itme = pt->obs_.end(); itm != itme; ++itm)
      {
        if ((*itm)->id_ == ref_patch_temp->id_) continue;
        float *patch_cache = (*itm)->patch_;

        float other_mean;
        if (abs((*itm)->mean_) < 1e-6)
        {
          float other_sum = std::accumulate(patch_cache, patch_cache + patch_size_total, 0.0);
          other_mean = other_sum / patch_size_total;
          (*itm)->mean_ = other_mean;
        }

        for (int ind = 0; ind < patch_size_total; ind++)
        {
          NCC_up += (patch_temp[ind] - ref_mean) * (patch_cache[ind] - other_mean);
          NCC_down1 += (patch_temp[ind] - ref_mean) * (patch_temp[ind] - ref_mean);
          NCC_down2 += (patch_cache[ind] - other_mean) * (patch_cache[ind] - other_mean);
        }
        NCC += fabs(NCC_up / sqrt(NCC_down1 * NCC_down2));
        count++;
      }

      // Computes average NCC across all comparisons.
      NCC = NCC / count;

      // Combined score: NCC plus view-angle cosine.
      score = NCC + cos_angle;

      ref_patch_temp->score_ = score;

      // Selects the feature with the highest score as reference.
      if (score > score_max)
      {
        score_max = score;
        pt->ref_patch = ref_patch_temp;
        pt->has_ref_patch_ = true;
      }
    }

  }
}

// Debug visualization: projects reference patches into the current frame and
// saves side-by-side comparison images with warp and photometric error overlays.
void VIOManager::projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
{
  if (total_points == 0) return;
  // if(new_frame_->id_ != 2) return; //124

  int patch_size = 25;
  string dir = string(ROOT_DIR) + "Log/ref_cur_combine/";

  cv::Mat result = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat result_normal = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat result_dense = cv::Mat::zeros(height, width, CV_8UC1);

  cv::Mat img_photometric_error = new_frame_->img_.clone();

  uchar *it = (uchar *)result.data;
  uchar *it_normal = (uchar *)result_normal.data;
  uchar *it_dense = (uchar *)result_dense.data;

  struct pixel_member
  {
    Vector2f pixel_pos;
    uint8_t pixel_value;
  };

  int num = 0;
  for (int i = 0; i < visual_submap->voxel_points.size(); i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];

    if (pt->is_normal_initialized_)
    {
      Feature *ref_ftr;
      ref_ftr = pt->ref_patch;
      // Feature* ref_ftr;
      V2D pc(new_frame_->w2c(pt->pos_));
      V2D pc_prior(new_frame_->w2c_prior(pt->pos_));

      V3D norm_vec(ref_ftr->T_f_w_.rotation_matrix() * pt->normal_);
      V3D pf(ref_ftr->T_f_w_ * pt->pos_);

      if (pf.dot(norm_vec) < 0) norm_vec = -norm_vec;

      // norm_vec << norm_vec(1), norm_vec(0), norm_vec(2);
      cv::Mat img_cur = new_frame_->img_;
      cv::Mat img_ref = ref_ftr->img_;

      SE3 T_cur_ref = new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse();
      Matrix2d A_cur_ref;
      getWarpMatrixAffineHomography(*cam, ref_ftr->px_, pf, norm_vec, T_cur_ref, 0, A_cur_ref);

      // const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
      int search_level = getBestSearchLevel(A_cur_ref.inverse(), 2);

      double D = A_cur_ref.determinant();
      if (D > 3) continue;

      num++;

      cv::Mat ref_cur_combine_temp;
      int radius = 20;
      cv::hconcat(img_cur, img_ref, ref_cur_combine_temp);
      cv::cvtColor(ref_cur_combine_temp, ref_cur_combine_temp, CV_GRAY2BGR);

      getImagePatch(img_cur, pc, patch_buffer.data(), 0);

      float error_est = 0.0;
      float error_gt = 0.0;

      for (int ind = 0; ind < patch_size_total; ind++)
      {
        error_est += (ref_ftr->inv_expo_time_ * visual_submap->warp_patch[i][ind] - state->inv_expo_time * patch_buffer[ind]) *
                     (ref_ftr->inv_expo_time_ * visual_submap->warp_patch[i][ind] - state->inv_expo_time * patch_buffer[ind]);
      }
      std::string ref_est = "ref_est " + std::to_string(1.0 / ref_ftr->inv_expo_time_);
      std::string cur_est = "cur_est " + std::to_string(1.0 / state->inv_expo_time);
      std::string cur_propa = "cur_gt " + std::to_string(error_gt);
      std::string cur_optimize = "cur_est " + std::to_string(error_est);

      cv::putText(ref_cur_combine_temp, ref_est, cv::Point2f(ref_ftr->px_[0] + img_cur.cols - 40, ref_ftr->px_[1] + 40), cv::FONT_HERSHEY_COMPLEX, 0.4,
                  cv::Scalar(0, 255, 0), 1, 8, 0);

      cv::putText(ref_cur_combine_temp, cur_est, cv::Point2f(pc[0] - 40, pc[1] + 40), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, 8, 0);
      cv::putText(ref_cur_combine_temp, cur_propa, cv::Point2f(pc[0] - 40, pc[1] + 60), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 0, 255), 1, 8,
                  0);
      cv::putText(ref_cur_combine_temp, cur_optimize, cv::Point2f(pc[0] - 40, pc[1] + 80), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, 8,
                  0);

      cv::rectangle(ref_cur_combine_temp, cv::Point2f(ref_ftr->px_[0] + img_cur.cols - radius, ref_ftr->px_[1] - radius),
                    cv::Point2f(ref_ftr->px_[0] + img_cur.cols + radius, ref_ftr->px_[1] + radius), cv::Scalar(0, 0, 255), 1);
      cv::rectangle(ref_cur_combine_temp, cv::Point2f(pc[0] - radius, pc[1] - radius), cv::Point2f(pc[0] + radius, pc[1] + radius),
                    cv::Scalar(0, 255, 0), 1);
      cv::rectangle(ref_cur_combine_temp, cv::Point2f(pc_prior[0] - radius, pc_prior[1] - radius),
                    cv::Point2f(pc_prior[0] + radius, pc_prior[1] + radius), cv::Scalar(255, 255, 255), 1);
      cv::circle(ref_cur_combine_temp, cv::Point2f(ref_ftr->px_[0] + img_cur.cols, ref_ftr->px_[1]), 1, cv::Scalar(0, 0, 255), -1, 8);
      cv::circle(ref_cur_combine_temp, cv::Point2f(pc[0], pc[1]), 1, cv::Scalar(0, 255, 0), -1, 8);
      cv::circle(ref_cur_combine_temp, cv::Point2f(pc_prior[0], pc_prior[1]), 1, cv::Scalar(255, 255, 255), -1, 8);
      cv::imwrite(dir + std::to_string(new_frame_->id_) + "_" + std::to_string(ref_ftr->id_) + "_" + std::to_string(num) + ".png",
                  ref_cur_combine_temp);

      std::vector<std::vector<pixel_member>> pixel_warp_matrix;

      for (int y = 0; y < patch_size; ++y)
      {
        vector<pixel_member> pixel_warp_vec;
        for (int x = 0; x < patch_size; ++x) //, ++patch_ptr)
        {
          Vector2f px_patch(x - patch_size / 2, y - patch_size / 2);
          px_patch *= (1 << search_level);
          const Vector2f px_ref(px_patch + ref_ftr->px_.cast<float>());
          uint8_t pixel_value = (uint8_t)vk::interpolateMat_8u(img_ref, px_ref[0], px_ref[1]);

          const Vector2f px(A_cur_ref.cast<float>() * px_patch + pc.cast<float>());
          if (px[0] < 0 || px[1] < 0 || px[0] >= img_cur.cols - 1 || px[1] >= img_cur.rows - 1)
            continue;
          else
          {
            pixel_member pixel_warp;
            pixel_warp.pixel_pos << px[0], px[1];
            pixel_warp.pixel_value = pixel_value;
            pixel_warp_vec.push_back(pixel_warp);
          }
        }
        pixel_warp_matrix.push_back(pixel_warp_vec);
      }

      float x_min = 1000;
      float y_min = 1000;
      float x_max = 0;
      float y_max = 0;

      for (int i = 0; i < pixel_warp_matrix.size(); i++)
      {
        vector<pixel_member> pixel_warp_row = pixel_warp_matrix[i];
        for (int j = 0; j < pixel_warp_row.size(); j++)
        {
          float x_temp = pixel_warp_row[j].pixel_pos[0];
          float y_temp = pixel_warp_row[j].pixel_pos[1];
          if (x_temp < x_min) x_min = x_temp;
          if (y_temp < y_min) y_min = y_temp;
          if (x_temp > x_max) x_max = x_temp;
          if (y_temp > y_max) y_max = y_temp;
        }
      }
      int x_min_i = floor(x_min);
      int y_min_i = floor(y_min);
      int x_max_i = ceil(x_max);
      int y_max_i = ceil(y_max);
      Matrix2f A_cur_ref_Inv = A_cur_ref.inverse().cast<float>();
      for (int i = x_min_i; i < x_max_i; i++)
      {
        for (int j = y_min_i; j < y_max_i; j++)
        {
          Eigen::Vector2f pc_temp(i, j);
          Vector2f px_patch = A_cur_ref_Inv * (pc_temp - pc.cast<float>());
          if (px_patch[0] > (-patch_size / 2 * (1 << search_level)) && px_patch[0] < (patch_size / 2 * (1 << search_level)) &&
              px_patch[1] > (-patch_size / 2 * (1 << search_level)) && px_patch[1] < (patch_size / 2 * (1 << search_level)))
          {
            const Vector2f px_ref(px_patch + ref_ftr->px_.cast<float>());
            uint8_t pixel_value = (uint8_t)vk::interpolateMat_8u(img_ref, px_ref[0], px_ref[1]);
            it_normal[width * j + i] = pixel_value;
          }
        }
      }
    }
  }
  for (int i = 0; i < visual_submap->voxel_points.size(); i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];

    if (!pt->is_normal_initialized_) continue;

    Feature *ref_ftr;
    V2D pc(new_frame_->w2c(pt->pos_));
    ref_ftr = pt->ref_patch;

    Matrix2d A_cur_ref;
    getWarpMatrixAffine(*cam, ref_ftr->px_, ref_ftr->f_, (ref_ftr->pos() - pt->pos_).norm(), new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse(), 0, 0,
                        patch_size_half, A_cur_ref);
    int search_level = getBestSearchLevel(A_cur_ref.inverse(), 2);
    double D = A_cur_ref.determinant();
    if (D > 3) continue;

    cv::Mat img_cur = new_frame_->img_;
    cv::Mat img_ref = ref_ftr->img_;
    for (int y = 0; y < patch_size; ++y)
    {
      for (int x = 0; x < patch_size; ++x) //, ++patch_ptr)
      {
        Vector2f px_patch(x - patch_size / 2, y - patch_size / 2);
        px_patch *= (1 << search_level);
        const Vector2f px_ref(px_patch + ref_ftr->px_.cast<float>());
        uint8_t pixel_value = (uint8_t)vk::interpolateMat_8u(img_ref, px_ref[0], px_ref[1]);

        const Vector2f px(A_cur_ref.cast<float>() * px_patch + pc.cast<float>());
        if (px[0] < 0 || px[1] < 0 || px[0] >= img_cur.cols - 1 || px[1] >= img_cur.rows - 1)
          continue;
        else
        {
          int col = int(px[0]);
          int row = int(px[1]);
          it[width * row + col] = pixel_value;
        }
      }
    }
  }
  cv::Mat ref_cur_combine;
  cv::Mat ref_cur_combine_normal;
  cv::Mat ref_cur_combine_error;

  cv::hconcat(result, new_frame_->img_, ref_cur_combine);
  cv::hconcat(result_normal, new_frame_->img_, ref_cur_combine_normal);

  cv::cvtColor(ref_cur_combine, ref_cur_combine, CV_GRAY2BGR);
  cv::cvtColor(ref_cur_combine_normal, ref_cur_combine_normal, CV_GRAY2BGR);
  cv::absdiff(img_photometric_error, result_normal, img_photometric_error);
  cv::hconcat(img_photometric_error, new_frame_->img_, ref_cur_combine_error);

  cv::imwrite(dir + std::to_string(new_frame_->id_) + "_0_" + ".png", ref_cur_combine);
  cv::imwrite(dir + std::to_string(new_frame_->id_) + +"_0_" +
                  "photometric"
                  ".png",
              ref_cur_combine_error);
  cv::imwrite(dir + std::to_string(new_frame_->id_) + "_0_" + "normal" + ".png", ref_cur_combine_normal);
}

// Precomputes the image gradient Jacobians for all retrieved visual points in
// the reference frame. Used by the inverse-compositional update strategy.
void VIOManager::precomputeReferencePatches(int level)
{
  // Start timing.
  double t1 = omp_get_wtime();
  // Returns early if no visual points are available.
  if (total_points == 0) return;
  // Image gradient Jacobian (1x2).
  MD(1, 2) Jimg;
  // Projection Jacobian (2x3).
  MD(2, 3) Jdpi;
  // Rotation, position, and combined Jacobians.
  MD(1, 3) Jdphi, Jdp, JdR, Jdt;

  // Total measurement dimension.
  const int H_DIM = total_points * patch_size_total;

  // Resizes and zeroes the precomputed Jacobian matrix.
  H_sub_inv.resize(H_DIM, 6);
  H_sub_inv.setZero();
  // Skew-symmetric matrix of world position.
  M3D p_w_hat;

  // Iterates over all visual points.
  for (int i = 0; i < total_points; i++)
  {
    // Scale factor for this pyramid level.
    const int scale = (1 << level);

    // Current visual point and its reference patch image.
    VisualPoint *pt = visual_submap->voxel_points[i];
    cv::Mat img = pt->ref_patch->img_;

    // Skips null points.
    if (pt == nullptr) continue;

    // Computes depth of point from reference camera.
    double depth((pt->pos_ - pt->ref_patch->pos()).norm());
    // 3D point in reference frame.
    V3D pf = pt->ref_patch->f_ * depth;
    // Pixel coordinate of reference feature.
    V2D pc = pt->ref_patch->px_;
    // Rotation from world to reference frame.
    M3D R_ref_w = pt->ref_patch->T_f_w_.rotation_matrix();

    // Computes projection Jacobian.
    computeProjectionJacobian(pf, Jdpi);
    // Skew-symmetric of world position for rotational Jacobian.
    p_w_hat << SKEW_SYM_MATRX(pt->pos_);

    // Bilinear interpolation parameters.
    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int u_ref_i = floorf(pc[0] / scale) * scale;
    const int v_ref_i = floorf(pc[1] / scale) * scale;
    const float subpix_u_ref = (u_ref - u_ref_i) / scale;
    const float subpix_v_ref = (v_ref - v_ref_i) / scale;
    const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
    const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;

    // Iterates over patch rows.
    for (int x = 0; x < patch_size; x++)
    {
      // Pointer to the start of the row in the image.
      uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i + x * scale - patch_size_half * scale) * width + u_ref_i - patch_size_half * scale;
      // Iterates over patch columns.
      for (int y = 0; y < patch_size; ++y, img_ptr += scale)
      {
        // Horizontal image gradient via central differences.
        float du =
            0.5f *
            ((w_ref_tl * img_ptr[scale] + w_ref_tr * img_ptr[scale * 2] + w_ref_bl * img_ptr[scale * width + scale] +
              w_ref_br * img_ptr[scale * width + scale * 2]) -
             (w_ref_tl * img_ptr[-scale] + w_ref_tr * img_ptr[0] + w_ref_bl * img_ptr[scale * width - scale] + w_ref_br * img_ptr[scale * width]));
        // Vertical image gradient via central differences.
        float dv =
            0.5f *
            ((w_ref_tl * img_ptr[scale * width] + w_ref_tr * img_ptr[scale + scale * width] + w_ref_bl * img_ptr[width * scale * 2] +
              w_ref_br * img_ptr[width * scale * 2 + scale]) -
             (w_ref_tl * img_ptr[-scale * width] + w_ref_tr * img_ptr[-scale * width + scale] + w_ref_bl * img_ptr[0] + w_ref_br * img_ptr[scale]));

        // Assembles the image gradient vector.
        Jimg << du, dv;
        // Scales by the inverse level scale.
        Jimg = Jimg * (1.0 / scale);

        // Rotational Jacobian: dI/dR.
        JdR = Jimg * Jdpi * R_ref_w * p_w_hat;
        // Translational Jacobian: dI/dt.
        Jdt = -Jimg * Jdpi * R_ref_w;

        // Stores precomputed Jacobian row.
        H_sub_inv.block<1, 6>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt;
      }
    }
  }
  // Marks the precomputed cache as valid.
  has_ref_patch_cache = true;
}

// Inverse-compositional EKF update: warps the reference patch by the current
// state estimate, computes photometric residuals, and iteratively updates the
// EKF state.
void VIOManager::updateStateInverse(cv::Mat img, int level)
{
  // Returns early if no visual points are available.
  if (total_points == 0) return;
  // Saves the current state for rollback.
  StatesGroup old_state = (*state);
  // Pixel coordinate in current frame.
  V2D pc;
  // Jacobian matrices for the photometric error.
  MD(1, 2) Jimg;
  MD(2, 3) Jdpi;
  MD(1, 3) Jdphi, Jdp, JdR, Jdt;
  // Residual vector and Jacobian matrix.
  VectorXd z;
  MatrixXd H_sub;
  // Convergence flags.
  bool EKF_end = false;
  float last_error = std::numeric_limits<float>::max();
  // Timing accumulators.
  compute_jacobian_time = update_ekf_time = 0.0;
  // Skew-symmetric matrix of world position.
  M3D P_wi_hat;
  bool z_init = true;
  // Measurement dimension.
  const int H_DIM = total_points * patch_size_total;

  // Allocates residual vector.
  z.resize(H_DIM);
  z.setZero();

  // Allocates Jacobian matrix (6 DoF: rotation + translation).
  H_sub.resize(H_DIM, 6);
  H_sub.setZero();

  // Iterative Gauss-Newton optimization.
  for (int iteration = 0; iteration < max_iterations; iteration++)
  {
    double t1 = omp_get_wtime();
    double count_outlier = 0;
    // Precomputes reference patch Jacobians if not cached.
    if (has_ref_patch_cache == false) precomputeReferencePatches(level);
    int n_meas = 0;
    float error = 0.0;
    // Current rotation and position estimates.
    M3D Rwi(state->rot_end);
    V3D Pwi(state->pos_end);
    P_wi_hat << SKEW_SYM_MATRX(Pwi);
    // Camera-to-world rotation.
    Rcw = Rci * Rwi.transpose();
    // Camera-to-world translation.
    Pcw = -Rci * Rwi.transpose() * Pwi + Pci;

    M3D p_hat;

    // Computes residuals for each visual point.
    for (int i = 0; i < total_points; i++)
    {
      float patch_error = 0.0;

      // Current pyramid scale.
      const int scale = (1 << level);

      VisualPoint *pt = visual_submap->voxel_points[i];

      // Skips null points.
      if (pt == nullptr) continue;

      // Projects point into current camera frame.
      V3D pf = Rcw * pt->pos_ + Pcw;
      pc = cam->world2cam(pf);

      // Bilinear interpolation parameters.
      const float u_ref = pc[0];
      const float v_ref = pc[1];
      const int u_ref_i = floorf(pc[0] / scale) * scale;
      const int v_ref_i = floorf(pc[1] / scale) * scale;
      const float subpix_u_ref = (u_ref - u_ref_i) / scale;
      const float subpix_v_ref = (v_ref - v_ref_i) / scale;
      const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
      const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
      const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
      const float w_ref_br = subpix_u_ref * subpix_v_ref;

      // Pre-warped reference patch.
      vector<float> P = visual_submap->warp_patch[i];
      // Iterates over patch pixels.
      for (int x = 0; x < patch_size; x++)
      {
        // Pointer to the start of the image row.
        uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i + x * scale - patch_size_half * scale) * width + u_ref_i - patch_size_half * scale;
        for (int y = 0; y < patch_size; ++y, img_ptr += scale)
        {
          // Photometric residual: bilinearly interpolated image minus warped reference.
          double res = w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] + w_ref_bl * img_ptr[scale * width] +
                       w_ref_br * img_ptr[scale * width + scale] - P[patch_size_total * level + x * patch_size + y];
          z(i * patch_size_total + x * patch_size + y) = res;
          patch_error += res * res;
          // Retrieves precomputed Jacobians.
          MD(1, 3) J_dR = H_sub_inv.block<1, 3>(i * patch_size_total + x * patch_size + y, 0);
          MD(1, 3) J_dt = H_sub_inv.block<1, 3>(i * patch_size_total + x * patch_size + y, 3);
          // Warps Jacobians to current state.
          JdR = J_dR * Rwi + J_dt * P_wi_hat * Rwi;
          Jdt = J_dt * Rwi;
          // Stores the Jacobian row.
          H_sub.block<1, 6>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt;
          n_meas++;
        }
      }
      visual_submap->errors[i] = patch_error;
      error += patch_error;
    }

    // Mean photometric error.
    error = error / n_meas;

    compute_jacobian_time += omp_get_wtime() - t1;

    double t3 = omp_get_wtime();

    // Accepts the update if error decreased.
    if (error <= last_error)
    {
      // Saves state for potential rollback.
      old_state = (*state);
      last_error = error;

      // EKF update: computes Kalman gain and state correction.
      auto &&H_sub_T = H_sub.transpose();
      H_T_H.setZero();
      G.setZero();
      H_T_H.block<6, 6>(0, 0) = H_sub_T * H_sub;
      MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state->cov / img_point_cov).inverse()).inverse();
      auto &&HTz = H_sub_T * z;
      auto vec = (*state_propagat) - (*state);
      G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
      auto solution = -K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
      (*state) += solution;
      // Rotation and translation update magnitudes.
      auto &&rot_add = solution.block<3, 1>(0, 0);
      auto &&t_add = solution.block<3, 1>(3, 0);

      // Checks convergence: small rotation and translation increments.
      if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f)) { EKF_end = true; }
    }
    // Rejects update and terminates if error increased.
    else
    {
      (*state) = old_state;
      EKF_end = true;
    }

    update_ekf_time += omp_get_wtime() - t3;

    if (iteration == max_iterations || EKF_end) break; 
  }
}

// Forward-additive EKF update: warps the reference patch into the current frame,
// computes photometric residuals and image Jacobians, and iteratively updates the
// EKF state with optional exposure-time estimation.
void VIOManager::updateState(cv::Mat img, int level)
{
  // Returns early if no visual points are available.
  if (total_points == 0) return;
  // Saves the current state for rollback.
  StatesGroup old_state = (*state);

  // Residual vector and Jacobian matrix.
  VectorXd z;
  MatrixXd H_sub;
  // Convergence flags.
  bool EKF_end = false;
  float last_error = std::numeric_limits<float>::max();

  // Measurement dimension.
  const int H_DIM = total_points * patch_size_total;
  // Allocates residual vector.
  z.resize(H_DIM);
  z.setZero();
  // Allocates Jacobian matrix (7 DoF: rotation + translation + exposure).
  H_sub.resize(H_DIM, 7);
  H_sub.setZero();

  // Iterative Gauss-Newton optimization.
  for (int iteration = 0; iteration < max_iterations; iteration++)
  {
    double t1 = omp_get_wtime();

    // Current rotation, position, and derived quantities.
    M3D Rwi(state->rot_end);
    V3D Pwi(state->pos_end);
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci * Rwi.transpose() * Pwi + Pci;
    Jdp_dt = Rci * Rwi.transpose();
    
    float error = 0.0;
    int n_meas = 0;
  
    // Optional OpenMP parallelization.
    #ifdef MP_EN
      omp_set_num_threads(MP_PROC_NUM);
      #pragma omp parallel for reduction(+:error, n_meas)
    #endif
    // Computes residuals for each visual point.
    for (int i = 0; i < total_points; i++)
    {
      // Jacobian matrices.
      MD(1, 2) Jimg;
      MD(2, 3) Jdpi;
      MD(1, 3) Jdphi, Jdp, JdR, Jdt;

      // Accumulated patch error.
      float patch_error = 0.0;
      // Search level and combined pyramid level.
      int search_level = visual_submap->search_levels[i];
      int pyramid_level = level + search_level;
      // Scale factor and its inverse.
      int scale = (1 << pyramid_level);
      float inv_scale = 1.0f / scale;

      // Current visual point.
      VisualPoint *pt = visual_submap->voxel_points[i];

      // Skips null points.
      if (pt == nullptr) continue;

      // Projects point into camera frame.
      V3D pf = Rcw * pt->pos_ + Pcw;
      V2D pc = cam->world2cam(pf);

      // Computes projection Jacobian.
      computeProjectionJacobian(pf, Jdpi);
      M3D p_hat;
      p_hat << SKEW_SYM_MATRX(pf);

      // Bilinear interpolation parameters.
      float u_ref = pc[0];
      float v_ref = pc[1];
      int u_ref_i = floorf(pc[0] / scale) * scale;
      int v_ref_i = floorf(pc[1] / scale) * scale;
      float subpix_u_ref = (u_ref - u_ref_i) / scale;
      float subpix_v_ref = (v_ref - v_ref_i) / scale;
      float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
      float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
      float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
      float w_ref_br = subpix_u_ref * subpix_v_ref;

      // Pre-warped reference patch and inverse exposure time.
      vector<float> P = visual_submap->warp_patch[i];
      double inv_ref_expo = visual_submap->inv_expo_list[i];

      // Iterates over patch pixels.
      for (int x = 0; x < patch_size; x++)
      {
        uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i + x * scale - patch_size_half * scale) * width + u_ref_i - patch_size_half * scale;
        for (int y = 0; y < patch_size; ++y, img_ptr += scale)
        {
          // Horizontal image gradient.
          float du =
              0.5f *
              ((w_ref_tl * img_ptr[scale] + w_ref_tr * img_ptr[scale * 2] + w_ref_bl * img_ptr[scale * width + scale] +
                w_ref_br * img_ptr[scale * width + scale * 2]) -
               (w_ref_tl * img_ptr[-scale] + w_ref_tr * img_ptr[0] + w_ref_bl * img_ptr[scale * width - scale] + w_ref_br * img_ptr[scale * width]));
          // Vertical image gradient.
          float dv =
              0.5f *
              ((w_ref_tl * img_ptr[scale * width] + w_ref_tr * img_ptr[scale + scale * width] + w_ref_bl * img_ptr[width * scale * 2] +
                w_ref_br * img_ptr[width * scale * 2 + scale]) -
               (w_ref_tl * img_ptr[-scale * width] + w_ref_tr * img_ptr[-scale * width + scale] + w_ref_bl * img_ptr[0] + w_ref_br * img_ptr[scale]));

          // Assembles the image Jacobian with exposure compensation.
          Jimg << du, dv;
          Jimg = Jimg * state->inv_expo_time;
          Jimg = Jimg * inv_scale;
          // Rotational and translational Jacobians.
          Jdphi = Jimg * Jdpi * p_hat;
          Jdp = -Jimg * Jdpi;
          // Combined Jacobians in the global frame.
          JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
          Jdt = Jdp * Jdp_dt;

          // Current pixel intensity.
          double cur_value =
              w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] + w_ref_bl * img_ptr[scale * width] + w_ref_br * img_ptr[scale * width + scale];
          // Photometric residual with exposure correction.
          double res = state->inv_expo_time * cur_value - inv_ref_expo * P[patch_size_total * level + x * patch_size + y];

          // Stores the residual.
          z(i * patch_size_total + x * patch_size + y) = res;

          patch_error += res * res;
          n_meas += 1;
          
          // Stores Jacobian with optional exposure dimension.
          if (exposure_estimate_en) { H_sub.block<1, 7>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt, cur_value; }
          else { H_sub.block<1, 6>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt; }
        }
      }
      visual_submap->errors[i] = patch_error;
      error += patch_error;
    }

    // Mean photometric error.
    error = error / n_meas;
    
    compute_jacobian_time += omp_get_wtime() - t1;

    double t3 = omp_get_wtime();

    // Accepts the update if error decreased.
    if (error <= last_error)
    {
      old_state = (*state);
      last_error = error;

      // EKF update with 7-DoF state (6 DoF pose + exposure).
      auto &&H_sub_T = H_sub.transpose();
      H_T_H.setZero();
      G.setZero();
      H_T_H.block<7, 7>(0, 0) = H_sub_T * H_sub;
      MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state->cov / img_point_cov).inverse()).inverse();
      auto &&HTz = H_sub_T * z;
      auto vec = (*state_propagat) - (*state);
      G.block<DIM_STATE, 7>(0, 0) = K_1.block<DIM_STATE, 7>(0, 0) * H_T_H.block<7, 7>(0, 0);
      MD(DIM_STATE, 1)
      solution = -K_1.block<DIM_STATE, 7>(0, 0) * HTz + vec - G.block<DIM_STATE, 7>(0, 0) * vec.block<7, 1>(0, 0);

      // Applies the state update.
      (*state) += solution;
      auto &&rot_add = solution.block<3, 1>(0, 0);
      auto &&t_add = solution.block<3, 1>(3, 0);

      auto &&expo_add = solution.block<1, 1>(6, 0);
      // Checks convergence.
      if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f))  EKF_end = true;
    }
    else
    {
      // Rolls back if error increased.
      (*state) = old_state;
      EKF_end = true;
    }

    update_ekf_time += omp_get_wtime() - t3;

    if (iteration == max_iterations || EKF_end) break;
  }
}

// Updates the new frame's pose from the current EKF state.
void VIOManager::updateFrameState(StatesGroup state)
{
  // Extracts rotation from the EKF state.
  M3D Rwi(state.rot_end);
  // Extracts position from the EKF state.
  V3D Pwi(state.pos_end);
  // Computes the camera-to-world rotation.
  Rcw = Rci * Rwi.transpose();
  // Computes the camera-to-world translation.
  Pcw = -Rci * Rwi.transpose() * Pwi + Pci;
  // Sets the new frame pose as an SE3 transformation.
  new_frame_->T_f_w_ = SE3(Rcw, Pcw);
}

// Draws circles on the debug image showing tracked visual points: green if the
// photometric error decreased after EKF update, blue otherwise.
void VIOManager::plotTrackedPoints()
{
  // Total number of visual points in the submap.
  int total_points = visual_submap->voxel_points.size();
  // Returns early if no points are available.
  if (total_points == 0) return;
  // Iterates over all visual points.
  for (int i = 0; i < total_points; i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];
    // Projects point to pixel coordinates in the current frame.
    V2D pc(new_frame_->w2c(pt->pos_));

    // Green circle if error decreased (tracking improved).
    if (visual_submap->errors[i] <= visual_submap->propa_errors[i])
    {
      // inlier_count++;
      cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 7, cv::Scalar(0, 255, 0), -1, 8); // Green Sparse Align tracked
    }
    // Blue circle if error increased.
    else
    {
      cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 7, cv::Scalar(255, 0, 0), -1, 8); // Blue Sparse Align tracked
    }
  }
}

// Returns the bilinearly interpolated BGR pixel value from a 3-channel image.
V3F VIOManager::getInterpolatedPixel(cv::Mat img, V2D pc)
{
  // Pixel u coordinate.
  const float u_ref = pc[0];
  // Pixel v coordinate.
  const float v_ref = pc[1];
  // Integer u coordinate.
  const int u_ref_i = floorf(pc[0]);
  // Integer v coordinate.
  const int v_ref_i = floorf(pc[1]);
  // Subpixel offset in u.
  const float subpix_u_ref = (u_ref - u_ref_i);
  // Subpixel offset in v.
  const float subpix_v_ref = (v_ref - v_ref_i);
  // Bilinear interpolation weights.
  const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
  const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
  const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
  const float w_ref_br = subpix_u_ref * subpix_v_ref;
  // Pointer to the BGR pixel data.
  uint8_t *img_ptr = (uint8_t *)img.data + ((v_ref_i)*width + (u_ref_i)) * 3;
  // Bilinearly interpolated blue channel.
  float B = w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[0 + 3] + w_ref_bl * img_ptr[width * 3] + w_ref_br * img_ptr[width * 3 + 0 + 3];
  // Bilinearly interpolated green channel.
  float G = w_ref_tl * img_ptr[1] + w_ref_tr * img_ptr[1 + 3] + w_ref_bl * img_ptr[1 + width * 3] + w_ref_br * img_ptr[width * 3 + 1 + 3];
  // Bilinearly interpolated red channel.
  float R = w_ref_tl * img_ptr[2] + w_ref_tr * img_ptr[2 + 3] + w_ref_bl * img_ptr[2 + width * 3] + w_ref_br * img_ptr[width * 3 + 2 + 3];
  // Returns the interpolated BGR pixel.
  V3F pixel(B, G, R);
  return pixel;
}

// Saves the current frame as an undistorted image and writes its camera pose
// to the colmap-format images.txt file for offline SfM.
void VIOManager::dumpDataForColmap()
{
  // Static counter for image naming.
  static int cnt = 1;
  // Formats the counter as a zero-padded string.
  std::ostringstream ss;
  ss << std::setw(5) << std::setfill('0') << cnt;
  std::string cnt_str = ss.str();
  // Output image path.
  std::string image_path = std::string(ROOT_DIR) + "Log/Colmap/images/" + cnt_str + ".png";
  
  // Undistorts the RGB image.
  cv::Mat img_rgb_undistort;
  pinhole_cam->undistortImage(img_rgb, img_rgb_undistort);
  // Saves the undistorted image.
  cv::imwrite(image_path, img_rgb_undistort);
  
  // Extracts pose as quaternion and translation.
  Eigen::Quaterniond q(new_frame_->T_f_w_.rotation_matrix());
  Eigen::Vector3d t = new_frame_->T_f_w_.translation();
  // Writes the image pose to the colmap images file.
  fout_colmap << cnt << " "
            << std::fixed << std::setprecision(6)  // 保证浮点数精度为6位
            << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
            << t.x() << " " << t.y() << " " << t.z() << " "
            << 1 << " "  // CAMERA_ID (假设相机ID为1)
            << cnt_str << ".png" << std::endl;
  fout_colmap << "0.0 0.0 -1" << std::endl;
  cnt++;
}

// Main VIO frame processing pipeline: resizes image, creates frame, retrieves
// visual map points, runs EKF update, generates new map points, updates existing
// ones, refines normals, and optionally saves colmap data.
void VIOManager::processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time)
{
  // Resizes image if dimensions do not match expected camera size.
  if (width != img.cols || height != img.rows)
  {
    if (img.empty()) printf("[ VIO ] Empty Image!\n");
    cv::resize(img, img, cv::Size(img.cols * image_resize_factor, img.rows * image_resize_factor), 0, 0, CV_INTER_LINEAR);
  }
  // Stores the RGB and debug copies of the image.
  img_rgb = img.clone();
  img_cp = img.clone();
  // img_test = img.clone();

  // Converts color image to grayscale if needed.
  if (img.channels() == 3) cv::cvtColor(img, img, CV_BGR2GRAY);

  // Creates the new frame with camera model and image.
  new_frame_.reset(new Frame(cam, img));
  // Updates the frame pose from the current EKF state.
  updateFrameState(*state);
  
  // Resets grid buffers for the new frame.
  resetGrid();

  // Timing for VIO pipeline stages.
  double t1 = omp_get_wtime();

  // Retrieves visual map points and computes depth map.
  retrieveFromVisualSparseMap(img, pg, feat_map);

  double t2 = omp_get_wtime();

  // Runs photometric EKF update.
  computeJacobianAndUpdateEKF(img);

  double t3 = omp_get_wtime();

  // Creates new visual map points from LiDAR points.
  generateVisualMapPoints(img, pg);

  double t4 = omp_get_wtime();
  
  // Visualizes tracked points on the debug image.
  plotTrackedPoints();

  // Saves debug visualization if enabled.
  if (plot_flag) projectPatchFromRefToCur(feat_map);

  double t5 = omp_get_wtime();

  // Updates existing visual map points with new observations.
  updateVisualMapPoints(img);

  double t6 = omp_get_wtime();

  // Refines normals and reference patches using the LiDAR plane map.
  updateReferencePatch(feat_map);

  double t7 = omp_get_wtime();
  
  // Saves colmap data if enabled.
  if(colmap_output_en)  dumpDataForColmap();

  // Updates frame counter and running average timing.
  frame_count++;
  // Computes running average (excluding visualization time).
  ave_total = ave_total * (frame_count - 1) / frame_count + (t7 - t1 - (t5 - t4)) / frame_count;
  
  // Prints the VIO timing table.
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         VIO Time                            |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // Prints the sparse map size.
  printf("\033[1;34m| %-29s | %-27zu |\033[0m\n", "Sparse Map Size", feat_map.size());
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // Prints individual stage timings.
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "retrieveFromVisualSparseMap", t2 - t1);
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "computeJacobianAndUpdateEKF", t3 - t2);
  printf("\033[1;32m| %-27s   | %-27lf |\033[0m\n", "-> computeJacobian", compute_jacobian_time);
  printf("\033[1;32m| %-27s   | %-27lf |\033[0m\n", "-> updateEKF", update_ekf_time);
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "generateVisualMapPoints", t4 - t3);
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "updateVisualMapPoints", t6 - t5);
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "updateReferencePatch", t7 - t6);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // Prints total timing.
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "Current Total Time", t7 - t1 - (t5 - t4));
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "Average Total Time", ave_total);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

}