/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Include the preprocessor header defining the Preprocess class and type aliases.
#include "preprocess.h"

// Return code for non-planar classification result.
#define RETURN0 0x00
// Return code mask combining both return states.
#define RETURN0AND1 0x10

// Constructor: sets default LiDAR feature extraction parameters and thresholds.
Preprocess::Preprocess() : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  // Set the infinity bound for range checking.
  inf_bound = 10;
  // Set the number of scanner rings/lines.
  N_SCANS = 6;
  // Set the group size for plane judgment window.
  group_size = 8;
  // Set distance coefficient A for adaptive grouping threshold (first assignment).
  disA = 0.01;
  // Overwrite distance coefficient A (second assignment, possibly a typo).
  disA = 0.1; // B?
  // Set the point-to-line ratio threshold for plane acceptance.
  p2l_ratio = 225;
  // Set the maximum ratio limit between max and mid distances.
  limit_maxmid = 6.25;
  // Set the maximum ratio limit between mid and min distances.
  limit_midmin = 6.25;
  // Set the maximum ratio limit between max and min distances.
  limit_maxmin = 3.24;
  // Set the jump-up angle threshold in degrees before cosine conversion.
  jump_up_limit = 170.0;
  // Set the jump-down angle threshold in degrees before cosine conversion.
  jump_down_limit = 8.0;
  // Set the cosine-of-160-degrees value placeholder before conversion.
  cos160 = 160.0;
  // Set edge distance multiplier for jump-edge validation.
  edgea = 2;
  // Set edge distance offset for jump-edge validation.
  edgeb = 0.1;
  // Set the small-plane intersection angle threshold in degrees before conversion.
  smallp_intersect = 172.5;
  // Set the small-plane distance ratio threshold.
  smallp_ratio = 1.2;
  // Initialize the given-offset-time flag to false.
  given_offset_time = false;

  // Convert jump-up limit from degrees to cosine value.
  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  // Convert jump-down limit from degrees to cosine value.
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  // Convert cos160 from degrees to cosine value.
  cos160 = cos(cos160 / 180 * M_PI);
  // Convert small-plane intersect angle from degrees to cosine value.
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

// Destructor (currently no cleanup needed).
Preprocess::~Preprocess() {}

// Configures the preprocessor with feature extraction flag, LiDAR type, blind
// range, and point filtering interval.
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  // Enable or disable feature extraction.
  feature_enabled = feat_en;
  // Set the active LiDAR sensor type enum.
  lidar_type = lid_type;
  // Set the minimum blind range (squared) for point filtering.
  blind = bld;
  // Set the point decimation interval (keep every Nth point).
  point_filter_num = pfilt_num;
}

// Processes a Livox custom-format message: extracts features or filters points.
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  // Delegate to the Livox Avia-specific handler.
  avia_handler(msg);
  // Assign the output cloud from the internal surface-point buffer.
  *pcl_out = pl_surf;
}

// Routes a standard ROS PointCloud2 to the appropriate sensor-specific handler
// based on the configured lidar type.
void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  // Dispatch to the correct handler based on the configured LiDAR type.
  switch (lidar_type)
  {
  // Ouster OS1-64 sensor.
  case OUST64:
    oust64_handler(msg);
    break;

  // Velodyne VLP-16 sensor.
  case VELO16:
    velodyne_handler(msg);
    break;

  // Intel RealSense L515 sensor.
  case L515:
    l515_handler(msg);
    break;

  // Hesai XT32 sensor.
  case XT32:
    xt32_handler(msg);
    break;

  // Hesai Pandar128 sensor.
  case PANDAR128:
    Pandar128_handler(msg);
    break;

  // Robosense Airy sensor.
  case ROBOSENSE:
    robosense_handler(msg);
    break;

  // Unsupported LiDAR type — print an error.
  default:
    printf("Error LiDAR Type: %d \n", lidar_type);
    break;
  }
  // Assign the processed surface points to the output cloud.
  *pcl_out = pl_surf;
}

// Handles Livox Avia LiDAR data: converts to unified format, optionally extracts
// edge/plane features, and filters points by blind range and decimation.
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  // Clear the surface point cloud buffer.
  pl_surf.clear();
  // Clear the corner/edge point cloud buffer.
  pl_corn.clear();
  // Clear the full point cloud buffer.
  pl_full.clear();
  // Record the start time of preprocessing.
  double t1 = omp_get_wtime();
  // Get the total number of points in the Livox message.
  int plsize = msg->point_num;
  // Log the input point count for debugging.
  printf("[ Preprocess ] Input point number: %d \n", plsize);
  // printf("point_filter_num: %d\n", point_filter_num);

  // Pre-allocate memory for corner points.
  pl_corn.reserve(plsize);
  // Pre-allocate memory for surface points.
  pl_surf.reserve(plsize);
  // Resize the full point array to match the input size.
  pl_full.resize(plsize);

  // Initialize per-ring point buffers for each scan line.
  for (int i = 0; i < N_SCANS; i++)
  {
    // Clear the ring buffer for scan line i.
    pl_buff[i].clear();
    // Pre-allocate memory for the ring buffer.
    pl_buff[i].reserve(plsize);
  }
  // Initialize the valid point counter to zero.
  uint valid_num = 0;

  // Branch based on whether feature extraction is enabled.
  if (feature_enabled)
  {
    // Iterate through all points starting from index 1 (skip first for pairwise check).
    for (uint i = 1; i < plsize; i++)
    {
      // Keep only points on valid scan lines with the correct tag (bit 0x10).
      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10))
      {
        // Copy X coordinate from the Livox message.
        pl_full[i].x = msg->points[i].x;
        // Copy Y coordinate from the Livox message.
        pl_full[i].y = msg->points[i].y;
        // Copy Z coordinate from the Livox message.
        pl_full[i].z = msg->points[i].z;
        // Store reflectivity as intensity.
        pl_full[i].intensity = msg->points[i].reflectivity;
        // Store offset time in seconds in the curvature field (used as timestamp).
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points

        // Flag to track if this point is spatially distinct from the previous.
        bool is_new = false;
        // Check if the point differs from the previous point in position.
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
            (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
        {
          // Add the point to its corresponding scan-line ring buffer.
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    // Static counter for averaging feature extraction timing across frames.
    static int count = 0;
    // Static accumulator for total feature extraction time.
    static double time = 0.0;
    // Increment the frame counter.
    count++;
    // Record the start time of the feature extraction phase.
    double t0 = omp_get_wtime();
    // Process each scan line independently.
    for (int j = 0; j < N_SCANS; j++)
    {
      // Skip scan lines with too few points (fewer than or equal to 5).
      if (pl_buff[j].size() <= 5) continue;
      // Get a reference to the current ring buffer.
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      // Update plsize to the current ring buffer size.
      plsize = pl.size();
      // Get a reference to the type annotation array for this ring.
      vector<orgtype> &types = typess[j];
      // Clear previous type annotations.
      types.clear();
      // Resize type array to match point count.
      types.resize(plsize);
      // Decrement because we compare pairs (i, i+1), so last index has no neighbor.
      plsize--;
      // Compute range and inter-point distance for each consecutive pair.
      for (uint i = 0; i < plsize; i++)
      {
        // Compute squared horizontal range from origin.
        types[i].range = pl[i].x * pl[i].x + pl[i].y * pl[i].y;
        // Compute X difference to the next point.
        vx = pl[i].x - pl[i + 1].x;
        // Compute Y difference to the next point.
        vy = pl[i].y - pl[i + 1].y;
        // Compute Z difference to the next point.
        vz = pl[i].z - pl[i + 1].z;
        // Store squared Euclidean distance between consecutive points.
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      // Compute squared horizontal range for the last point (no neighbor).
      types[plsize].range = pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y;
      // Run the feature classification on this scan line.
      give_feature(pl, types);
      // pl_surf += pl;
    }
    // Accumulate the feature extraction duration into the static total.
    time += omp_get_wtime() - t0;
    // Print the average feature extraction time per frame.
    printf("Feature extraction time: %lf \n", time / count);
  }
  // Non-feature path: simple filtering and decimation only.
  else
  {
    // Iterate over all points in the message.
    for (uint i = 0; i < plsize; i++)
    {
      // Accept points from valid scan lines.
      if ((msg->points[i].line < N_SCANS)) // && ((msg->points[i].tag & 0x30) == 0x10))
      {
        // Increment the valid point counter.
        valid_num++;

        // Copy X coordinate from the Livox message.
        pl_full[i].x = msg->points[i].x;
        // Copy Y coordinate from the Livox message.
        pl_full[i].y = msg->points[i].y;
        // Copy Z coordinate from the Livox message.
        pl_full[i].z = msg->points[i].z;
        // Store reflectivity as intensity.
        pl_full[i].intensity = msg->points[i].reflectivity;
        // Store offset time in seconds in the curvature field.
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points

        // For the first point, clamp negative curvature to zero.
        if (i == 0)
          pl_full[i].curvature = fabs(pl_full[i].curvature) < 1.0 ? pl_full[i].curvature : 0.0;
        // For subsequent points, ensure monotonic time with a fallback increment.
        else
        {
          pl_full[i].curvature = fabs(pl_full[i].curvature - pl_full[i - 1].curvature) < 1.0
                                     ? pl_full[i].curvature
                                     : pl_full[i - 1].curvature + 0.004166667f; // float(100/24000)
        }

        // Apply point decimation filter (keep every Nth valid point).
        if (valid_num % point_filter_num == 0)
        {
          // Check that the point is outside the blind zone.
          if (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z >= blind_sqr)
          {
            // Add the point to the output surface cloud.
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
  // Log the number of output surface points.
  printf("[ Preprocess ] Output point number: %zu \n", pl_surf.points.size());
}

// Handles Intel L515 LiDAR data: converts to unified format with RGB stored in
// normal fields and filters points.
void Preprocess::l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Clear the surface point cloud buffer.
  pl_surf.clear();
  // Clear the corner point cloud buffer.
  pl_corn.clear();
  // Clear the full point cloud buffer.
  pl_full.clear();
  // Temporary point cloud holding the original RGB data.
  pcl::PointCloud<pcl::PointXYZRGB> pl_orig;
  // Convert the ROS message to a PCL point cloud.
  pcl::fromROSMsg(*msg, pl_orig);
  // Get the total number of points.
  int plsize = pl_orig.size();
  // Pre-allocate memory for corner points.
  pl_corn.reserve(plsize);
  // Pre-allocate memory for surface points.
  pl_surf.reserve(plsize);

  // Extract the ROS message timestamp.
  double time_stamp = msg->header.stamp.toSec();
  // Iterate over the original points.
  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    // Apply decimation — skip points that do not match the filter interval.
    if (i % point_filter_num != 0) continue;

    // Compute squared Euclidean distance from origin.
    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    // Skip points inside the blind zone.
    if (range < blind_sqr) continue;

    // Temporary 3D vector (unused placeholder for potential future use).
    Eigen::Vector3d pt_vec;
    // Temporary point to hold converted data.
    PointType added_pt;
    // Copy X coordinate.
    added_pt.x = pl_orig.points[i].x;
    // Copy Y coordinate.
    added_pt.y = pl_orig.points[i].y;
    // Copy Z coordinate.
    added_pt.z = pl_orig.points[i].z;
    // Store red channel value in normal_x.
    added_pt.normal_x = pl_orig.points[i].r;
    // Store green channel value in normal_y.
    added_pt.normal_y = pl_orig.points[i].g;
    // Store blue channel value in normal_z.
    added_pt.normal_z = pl_orig.points[i].b;

    // Set curvature to zero (no time information available for L515).
    added_pt.curvature = 0.0;
    // Append the converted point to the surface cloud.
    pl_surf.points.push_back(added_pt);
  }

  // Log the original point count for debugging.
  cout << "pl size:: " << pl_orig.points.size() << endl;
}

// Handles Ouster OS1-64 LiDAR data: extracts features by ring or filters points
// with yaw-angle-based timestamps.
void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Clear the surface point cloud buffer.
  pl_surf.clear();
  // Clear the corner point cloud buffer.
  pl_corn.clear();
  // Clear the full point cloud buffer.
  pl_full.clear();
  // Temporary point cloud in Ouster native format.
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  // Convert the ROS message to a PCL point cloud.
  pcl::fromROSMsg(*msg, pl_orig);
  // Get the total number of points.
  int plsize = pl_orig.size();
  // Pre-allocate memory for corner points.
  pl_corn.reserve(plsize);
  // Pre-allocate memory for surface points.
  pl_surf.reserve(plsize);
  // Branch based on whether feature extraction is enabled.
  if (feature_enabled)
  {
    // Initialize per-ring point buffers for each scan line.
    for (int i = 0; i < N_SCANS; i++)
    {
      // Clear the ring buffer for scan line i.
      pl_buff[i].clear();
      // Pre-allocate memory for the ring buffer.
      pl_buff[i].reserve(plsize);
    }

    // Iterate over all points and assign them to ring buffers.
    for (uint i = 0; i < plsize; i++)
    {
      // Compute squared distance from origin.
      double range =
          pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      // Skip points inside the blind zone.
      if (range < blind_sqr) continue;
      // Temporary 3D vector (unused placeholder for potential future use).
      Eigen::Vector3d pt_vec;
      // Temporary point for conversion.
      PointType added_pt;
      // Copy X coordinate.
      added_pt.x = pl_orig.points[i].x;
      // Copy Y coordinate.
      added_pt.y = pl_orig.points[i].y;
      // Copy Z coordinate.
      added_pt.z = pl_orig.points[i].z;
      // Copy intensity value.
      added_pt.intensity = pl_orig.points[i].intensity;
      // Clear normal_x field.
      added_pt.normal_x = 0;
      // Clear normal_y field.
      added_pt.normal_y = 0;
      // Clear normal_z field.
      added_pt.normal_z = 0;
      // Compute the yaw angle in degrees.
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      // Wrap yaw angle to [-180, 180) range.
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      // Wrap yaw angle from below -180 back into range.
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      // Store the embedded timestamp (microseconds converted to seconds) as curvature.
      added_pt.curvature = pl_orig.points[i].t / 1e6;
      // Add the point to its corresponding ring buffer if the ring ID is valid.
      if (pl_orig.points[i].ring < N_SCANS) { pl_buff[pl_orig.points[i].ring].push_back(added_pt); }
    }

    // Classify features independently for each scan ring.
    for (int j = 0; j < N_SCANS; j++)
    {
      // Get a reference to the current ring buffer.
      PointCloudXYZI &pl = pl_buff[j];
      // Get the number of points in this ring.
      int linesize = pl.size();
      // Get a reference to the type annotation array.
      vector<orgtype> &types = typess[j];
      // Clear previous type annotations.
      types.clear();
      // Resize the type array to match point count.
      types.resize(linesize);
      // Decrement because we compare pairs (i, i+1), so last index has no neighbor.
      linesize--;
      // Compute range and inter-point distance for each consecutive pair.
      for (uint i = 0; i < linesize; i++)
      {
        // Compute horizontal range from origin (Euclidean distance in XY plane).
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        // Compute X difference to the next point.
        vx = pl[i].x - pl[i + 1].x;
        // Compute Y difference to the next point.
        vy = pl[i].y - pl[i + 1].y;
        // Compute Z difference to the next point.
        vz = pl[i].z - pl[i + 1].z;
        // Store squared Euclidean distance between consecutive points.
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      // Compute horizontal range for the last point (has no neighbor).
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      // Run the feature classification on this scan line.
      give_feature(pl, types);
    }
  }
  // Non-feature path: simple filtering and timestamp-based sorting.
  else
  {
    // Extract the message timestamp.
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    // Iterate over all original points.
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      // Apply decimation — skip points that do not match the filter interval.
      if (i % point_filter_num != 0) continue;

      // Compute squared distance from origin.
      double range =
          pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

      // Skip points inside the blind zone.
      if (range < blind_sqr) continue;

      // Temporary 3D vector (unused placeholder for potential future use).
      Eigen::Vector3d pt_vec;
      // Temporary point for conversion.
      PointType added_pt;
      // Copy X coordinate.
      added_pt.x = pl_orig.points[i].x;
      // Copy Y coordinate.
      added_pt.y = pl_orig.points[i].y;
      // Copy Z coordinate.
      added_pt.z = pl_orig.points[i].z;
      // Copy intensity value.
      added_pt.intensity = pl_orig.points[i].intensity;
      // Clear normal_x field.
      added_pt.normal_x = 0;
      // Clear normal_y field.
      added_pt.normal_y = 0;
      // Clear normal_z field.
      added_pt.normal_z = 0;
      // Compute the yaw angle in degrees.
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      // Wrap yaw angle to [-180, 180) range.
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      // Wrap yaw angle from below -180 back into range.
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      // Store the embedded timestamp (microseconds converted to seconds) as curvature.
      added_pt.curvature = pl_orig.points[i].t / 1e6;

      // cout<<added_pt.curvature<<endl;

      // Append the converted point to the surface cloud.
      pl_surf.points.push_back(added_pt);
    }
    // Sort surface points by their curvature (timestamp) in ascending order.
    std::sort(pl_surf.points.begin(), pl_surf.points.end(), [](const PointType &a, const PointType &b) {
      return a.curvature < b.curvature;
    });
  }
}

// Maximum number of scan lines supported by the ring-based processing logic.
#define MAX_LINE_NUM 64

// Handles Velodyne/VLP-16 LiDAR data: computes per-point offset times from yaw
// angle or embedded timestamps, extracts features, and filters points.
void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Clear the surface point cloud buffer.
  pl_surf.clear();
  // Clear the corner point cloud buffer.
  pl_corn.clear();
  // Clear the full point cloud buffer.
  pl_full.clear();

  // Temporary point cloud in Velodyne native format.
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  // Convert the ROS message to a PCL point cloud.
  pcl::fromROSMsg(*msg, pl_orig);
  // Get the total number of points.
  int plsize = pl_orig.points.size();
  // Return early if there are no points.
  if (plsize == 0) return;
  // Pre-allocate memory for the surface point cloud.
  pl_surf.reserve(plsize);

  // Flags to track first-point status per scan line.
  bool is_first[MAX_LINE_NUM];
  // Yaw angle of the first point in each scan line.
  double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
  // Scanning angular velocity in degrees per millisecond.
  double omega_l = 3.61;                 // scan angular velocity
  // Yaw angle of the last processed point per scan line.
  float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
  // Curvature (time offset) of the last processed point per scan line.
  float time_last[MAX_LINE_NUM] = {0.0}; // last offset time

  // Check if the sensor provides embedded timestamps (positive time on last point).
  if (pl_orig.points[plsize - 1].time > 0) { given_offset_time = true; }
  // Otherwise, compute offset times from yaw angles.
  else
  {
    given_offset_time = false;
    // Initialize all first-point flags to true.
    memset(is_first, true, sizeof(is_first));
    // Compute the yaw angle of the first point in degrees.
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    // Initialize yaw_end to yaw_first for the full-scan search.
    double yaw_end = yaw_first;
    // Record the ring of the first point for matching the last point on the same ring.
    int layer_first = pl_orig.points[0].ring;
    // Search backward for the last point on the same ring.
    for (uint i = plsize - 1; i > 0; i--)
    {
      // Found the last point on the first ring.
      if (pl_orig.points[i].ring == layer_first)
      {
        // Compute its yaw angle to determine the full rotation extent.
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  // Branch based on whether feature extraction is enabled.
  if (feature_enabled)
  {
    // Initialize per-ring point buffers for each scan line.
    for (int i = 0; i < N_SCANS; i++)
    {
      // Clear the ring buffer for scan line i.
      pl_buff[i].clear();
      // Pre-allocate memory for the ring buffer.
      pl_buff[i].reserve(plsize);
    }

    // Iterate over all points.
    for (int i = 0; i < plsize; i++)
    {
      // Temporary point for conversion.
      PointType added_pt;
      // Clear normal_x field.
      added_pt.normal_x = 0;
      // Clear normal_y field.
      added_pt.normal_y = 0;
      // Clear normal_z field.
      added_pt.normal_z = 0;
      // Get the ring (scan line) of the current point.
      int layer = pl_orig.points[i].ring;
      // Skip points on invalid scan lines (beyond configured N_SCANS).
      if (layer >= N_SCANS) continue;
      // Copy X coordinate.
      added_pt.x = pl_orig.points[i].x;
      // Copy Y coordinate.
      added_pt.y = pl_orig.points[i].y;
      // Copy Z coordinate.
      added_pt.z = pl_orig.points[i].z;
      // Copy intensity value.
      added_pt.intensity = pl_orig.points[i].intensity;
      // Store the embedded time (milliseconds) as curvature.
      added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

      // If the sensor does not provide embedded timestamps, compute from yaw.
      if (!given_offset_time)
      {
        // Compute the yaw angle in degrees.
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        // If this is the first point on this scan line, record its yaw and skip.
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          // Record the yaw of the first point on this line.
          yaw_fp[layer] = yaw_angle;
          // Mark the first point as processed (clear the flag).
          is_first[layer] = false;
          // Set curvature to zero (start of the scan line).
          added_pt.curvature = 0.0;
          // Record the yaw of this point for continuity checks.
          yaw_last[layer] = yaw_angle;
          // Record the curvature of this point for monotonicity checks.
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // Compute offset time from yaw difference relative to the first point.
        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        // Handle the wrap-around case when yaw crossed from 360 back to 0.
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        // Ensure monotonic increasing curvature by adding a full rotation period if needed.
        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        // Update the last yaw value for this layer.
        yaw_last[layer] = yaw_angle;
        // Update the last time value for this layer.
        time_last[layer] = added_pt.curvature;
      }

      // Add the point to its corresponding ring buffer.
      pl_buff[layer].points.push_back(added_pt);
    }

    // Classify features independently for each scan ring.
    for (int j = 0; j < N_SCANS; j++)
    {
      // Get a reference to the current ring buffer.
      PointCloudXYZI &pl = pl_buff[j];
      // Get the number of points in this ring.
      int linesize = pl.size();
      // Skip rings with fewer than 2 points (insufficient for feature extraction).
      if (linesize < 2) continue;
      // Get a reference to the type annotation array.
      vector<orgtype> &types = typess[j];
      // Clear previous type annotations.
      types.clear();
      // Resize the type array to match point count.
      types.resize(linesize);
      // Decrement because we compare pairs (i, i+1), so last index has no neighbor.
      linesize--;
      // Compute range and inter-point distance for each consecutive pair.
      for (uint i = 0; i < linesize; i++)
      {
        // Compute horizontal range from origin (Euclidean distance in XY plane).
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        // Compute X difference to the next point.
        vx = pl[i].x - pl[i + 1].x;
        // Compute Y difference to the next point.
        vy = pl[i].y - pl[i + 1].y;
        // Compute Z difference to the next point.
        vz = pl[i].z - pl[i + 1].z;
        // Store squared Euclidean distance between consecutive points.
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      // Compute horizontal range for the last point (has no neighbor).
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      // Run the feature classification on this scan line.
      give_feature(pl, types);
    }
  }
  // Non-feature path: simple filtering and yaw-based time computation.
  else
  {
    // Iterate over all points.
    for (int i = 0; i < plsize; i++)
    {
      // Temporary point for conversion.
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      // Clear normal_x field.
      added_pt.normal_x = 0;
      // Clear normal_y field.
      added_pt.normal_y = 0;
      // Clear normal_z field.
      added_pt.normal_z = 0;
      // Copy X coordinate.
      added_pt.x = pl_orig.points[i].x;
      // Copy Y coordinate.
      added_pt.y = pl_orig.points[i].y;
      // Copy Z coordinate.
      added_pt.z = pl_orig.points[i].z;
      // Copy intensity value.
      added_pt.intensity = pl_orig.points[i].intensity;
      // Store the embedded time (milliseconds) as curvature.
      added_pt.curvature = pl_orig.points[i].time / 1000.0;

      // If the sensor does not provide embedded timestamps, compute from yaw.
      if (!given_offset_time)
      {
        // Get the ring (scan line) of the current point.
        int layer = pl_orig.points[i].ring;
        // Compute the yaw angle in degrees.
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        // If this is the first point on this scan line, record its yaw and skip.
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          // Record the yaw of the first point on this line.
          yaw_fp[layer] = yaw_angle;
          // Mark the first point as processed (clear the flag).
          is_first[layer] = false;
          // Set curvature to zero (start of the scan line).
          added_pt.curvature = 0.0;
          // Record the yaw of this point for continuity checks.
          yaw_last[layer] = yaw_angle;
          // Record the curvature of this point for monotonicity checks.
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        // Compute offset time from yaw difference relative to the first point.
        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        // Handle the wrap-around case when yaw crossed from 360 back to 0.
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        // Ensure monotonic increasing curvature by adding a full rotation period if needed.
        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        // added_pt.curvature = pl_orig.points[i].t;

        // Update the last yaw value for this layer.
        yaw_last[layer] = yaw_angle;
        // Update the last time value for this layer.
        time_last[layer] = added_pt.curvature;
      }

      // Apply point decimation filter (keep every Nth point).
      if (i % point_filter_num == 0)
      {
        // Check that the point is outside the blind zone.
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          // Add the point to the output surface cloud.
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

// Handles Hesai Pandar128 LiDAR data: converts to unified format, computes
// per-point offset times, and sorts the output by time.
void Preprocess::Pandar128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Clear the surface point cloud buffer.
  pl_surf.clear();

  // Temporary point cloud in Pandar128 native format.
  pcl::PointCloud<Pandar128_ros::Point> pl_orig;
  // Convert the ROS message to a PCL point cloud.
  pcl::fromROSMsg(*msg, pl_orig);
  // Get the total number of points.
  int plsize = pl_orig.points.size();
  // Pre-allocate memory for the surface point cloud.
  pl_surf.reserve(plsize);

  // Record the timestamp of the first point as the reference (time zero).
  double time_head = pl_orig.points[0].timestamp;
  // Iterate over all points.
  for (int i = 0; i < plsize; i++)
  {
    // Temporary point for conversion.
    PointType added_pt;

    // Clear normal_x field.
    added_pt.normal_x = 0;
    // Clear normal_y field.
    added_pt.normal_y = 0;
    // Clear normal_z field.
    added_pt.normal_z = 0;
    // Copy X coordinate.
    added_pt.x = pl_orig.points[i].x;
    // Copy Y coordinate.
    added_pt.y = pl_orig.points[i].y;
    // Copy Z coordinate.
    added_pt.z = pl_orig.points[i].z;
    // Normalize intensity from [0, 255] to [0.0, 1.0].
    added_pt.intensity = static_cast<float>(pl_orig.points[i].intensity) / 255.0f;
    // Compute offset time in milliseconds relative to the first point.
    added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;

    // Apply point decimation filter (keep every Nth point).
    if (i % point_filter_num == 0)
    {
      // Check that the point is outside the blind zone.
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
      {
        // Add the point to the output surface cloud.
        pl_surf.points.push_back(added_pt);
        // printf("time mode: %d time: %d \n", given_offset_time,
        // pl_orig.points[i].t);
      }
    }
  }

  // define a lambda function for the comparison
  // Lambda comparator for sorting points by curvature (timestamp).
  auto comparePoints = [](const PointType& a, const PointType& b) -> bool
  {
    // Return true if point a has an earlier timestamp than point b.
    return a.curvature < b.curvature;
  };
  
  // sort the points using the comparison function
  // Sort the output points by curvature (timestamp) for temporal consistency.
  std::sort(pl_surf.points.begin(), pl_surf.points.end(), comparePoints);
}

// Handles Hesai XT32 LiDAR data: computes yaw-based offset times, extracts
// features by ring, and filters points.
void Preprocess::xt32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Clear the surface point cloud buffer.
  pl_surf.clear();
  // Clear the corner point cloud buffer.
  pl_corn.clear();
  // Clear the full point cloud buffer.
  pl_full.clear();

  // Temporary point cloud in XT32 native format.
  pcl::PointCloud<xt32_ros::Point> pl_orig;
  // Convert the ROS message to a PCL point cloud.
  pcl::fromROSMsg(*msg, pl_orig);
  // Get the total number of points.
  int plsize = pl_orig.points.size();
  // Pre-allocate memory for the surface point cloud.
  pl_surf.reserve(plsize);

  // Flags to track first-point status per scan line.
  bool is_first[MAX_LINE_NUM];
  // Yaw angle of the first point in each scan line.
  double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
  // Scanning angular velocity in degrees per millisecond.
  double omega_l = 3.61;                 // scan angular velocity
  // Yaw angle of the last processed point per scan line.
  float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
  // Curvature (time offset) of the last processed point per scan line.
  float time_last[MAX_LINE_NUM] = {0.0}; // last offset time

  // Check if the sensor provides embedded timestamps (positive timestamp on last point).
  if (pl_orig.points[plsize - 1].timestamp > 0) { given_offset_time = true; }
  // Otherwise, compute offset times from yaw angles.
  else
  {
    given_offset_time = false;
    // Initialize all first-point flags to true.
    memset(is_first, true, sizeof(is_first));
    // Compute the yaw angle of the first point in degrees.
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    // Initialize yaw_end to yaw_first for the full-scan search.
    double yaw_end = yaw_first;
    // Record the ring of the first point for matching the last point on the same ring.
    int layer_first = pl_orig.points[0].ring;
    // Search backward for the last point on the same ring.
    for (uint i = plsize - 1; i > 0; i--)
    {
      // Found the last point on the first ring.
      if (pl_orig.points[i].ring == layer_first)
      {
        // Compute its yaw angle to determine the full rotation extent.
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  // Reference timestamp from the first point (used in non-feature path).
  double time_head = pl_orig.points[0].timestamp;

  // Branch based on whether feature extraction is enabled.
  if (feature_enabled)
  {
    // Initialize per-ring point buffers for each scan line.
    for (int i = 0; i < N_SCANS; i++)
    {
      // Clear the ring buffer for scan line i.
      pl_buff[i].clear();
      // Pre-allocate memory for the ring buffer.
      pl_buff[i].reserve(plsize);
    }

    // Iterate over all points.
    for (int i = 0; i < plsize; i++)
    {
      // Temporary point for conversion.
      PointType added_pt;
      // Clear normal_x field.
      added_pt.normal_x = 0;
      // Clear normal_y field.
      added_pt.normal_y = 0;
      // Clear normal_z field.
      added_pt.normal_z = 0;
      // Get the ring (scan line) of the current point.
      int layer = pl_orig.points[i].ring;
      // Skip points on invalid scan lines (beyond configured N_SCANS).
      if (layer >= N_SCANS) continue;
      // Copy X coordinate.
      added_pt.x = pl_orig.points[i].x;
      // Copy Y coordinate.
      added_pt.y = pl_orig.points[i].y;
      // Copy Z coordinate.
      added_pt.z = pl_orig.points[i].z;
      // Copy intensity value.
      added_pt.intensity = pl_orig.points[i].intensity;
      // Store the embedded timestamp (milliseconds) as curvature.
      added_pt.curvature = pl_orig.points[i].timestamp / 1000.0; // units: ms

      // If the sensor does not provide embedded timestamps, compute from yaw.
      if (!given_offset_time)
      {
        // Compute the yaw angle in degrees.
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        // If this is the first point on this scan line, record its yaw and skip.
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          // Record the yaw of the first point on this line.
          yaw_fp[layer] = yaw_angle;
          // Mark the first point as processed (clear the flag).
          is_first[layer] = false;
          // Set curvature to zero (start of the scan line).
          added_pt.curvature = 0.0;
          // Record the yaw of this point for continuity checks.
          yaw_last[layer] = yaw_angle;
          // Record the curvature of this point for monotonicity checks.
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // Compute offset time from yaw difference relative to the first point.
        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        // Handle the wrap-around case when yaw crossed from 360 back to 0.
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        // Ensure monotonic increasing curvature by adding a full rotation period if needed.
        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        // Update the last yaw value for this layer.
        yaw_last[layer] = yaw_angle;
        // Update the last time value for this layer.
        time_last[layer] = added_pt.curvature;
      }

      // Add the point to its corresponding ring buffer.
      pl_buff[layer].points.push_back(added_pt);
    }

    // Classify features independently for each scan ring.
    for (int j = 0; j < N_SCANS; j++)
    {
      // Get a reference to the current ring buffer.
      PointCloudXYZI &pl = pl_buff[j];
      // Get the number of points in this ring.
      int linesize = pl.size();
      // Skip rings with fewer than 2 points (insufficient for feature extraction).
      if (linesize < 2) continue;
      // Get a reference to the type annotation array.
      vector<orgtype> &types = typess[j];
      // Clear previous type annotations.
      types.clear();
      // Resize the type array to match point count.
      types.resize(linesize);
      // Decrement because we compare pairs (i, i+1), so last index has no neighbor.
      linesize--;
      // Compute range and inter-point distance for each consecutive pair.
      for (uint i = 0; i < linesize; i++)
      {
        // Compute horizontal range from origin (Euclidean distance in XY plane).
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        // Compute X difference to the next point.
        vx = pl[i].x - pl[i + 1].x;
        // Compute Y difference to the next point.
        vy = pl[i].y - pl[i + 1].y;
        // Compute Z difference to the next point.
        vz = pl[i].z - pl[i + 1].z;
        // Store squared Euclidean distance between consecutive points.
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      // Compute horizontal range for the last point (has no neighbor).
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      // Run the feature classification on this scan line.
      give_feature(pl, types);
    }
  }
  // Non-feature path: simple filtering and timestamp-based time computation.
  else
  {
    // Iterate over all points.
    for (int i = 0; i < plsize; i++)
    {
      // Temporary point for conversion.
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      // Clear normal_x field.
      added_pt.normal_x = 0;
      // Clear normal_y field.
      added_pt.normal_y = 0;
      // Clear normal_z field.
      added_pt.normal_z = 0;
      // Copy X coordinate.
      added_pt.x = pl_orig.points[i].x;
      // Copy Y coordinate.
      added_pt.y = pl_orig.points[i].y;
      // Copy Z coordinate.
      added_pt.z = pl_orig.points[i].z;
      // Copy intensity value.
      added_pt.intensity = pl_orig.points[i].intensity;
      // Compute offset time in milliseconds relative to the first point.
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;

      // Apply point decimation filter (keep every Nth point).
      if (i % point_filter_num == 0)
      {
        // Check that the point is outside the blind zone.
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          // Add the point to the output surface cloud.
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

// Handles Robosense Airy LiDAR data: converts to unified format, computes
// per-point offset times, validates points, and sorts by time.
void Preprocess::robosense_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Clear the surface point cloud buffer.
  pl_surf.clear();

  // Temporary point cloud in Robosense native format.
  pcl::PointCloud<robosense_ros::Point> pl_orig;
  // Convert the ROS message to a PCL point cloud.
  pcl::fromROSMsg(*msg, pl_orig);
  // Get the total number of points.
  int plsize = pl_orig.size();
  // Pre-allocate memory for the surface point cloud.
  pl_surf.reserve(plsize);

  // Record the timestamp of the first point as the reference (time zero).
  double time_head = pl_orig.points[0].timestamp;
  // Iterate over all points.
  for (int i = 0; i < plsize; ++i)
  {
    // Apply decimation — skip points that do not match the filter interval.
    if (i % point_filter_num != 0) continue;

    // Reference to the current point for cleaner access.
    const auto& pt = pl_orig.points[i];
    // Extract X, Y, Z coordinates into local variables for repeated use.
    const double x = pt.x, y = pt.y, z = pt.z;
    // Compute squared Euclidean distance from origin.
    const double dist_sqr = x * x + y * y + z * z;
    // Validate the point: must be outside the blind zone and have finite coordinates.
    const bool is_valid = (dist_sqr >= blind_sqr) && !std::isnan(x) && !std::isnan(y) && !std::isnan(z);
    // Skip invalid points.
    if (!is_valid) continue;

    // Temporary point for conversion.
    PointType added_pt;
    // Clear normal_x field.
    added_pt.normal_x = 0;
    // Clear normal_y field.
    added_pt.normal_y = 0;
    // Clear normal_z field.
    added_pt.normal_z = 0;
    // Copy X coordinate.
    added_pt.x = pt.x;
    // Copy Y coordinate.
    added_pt.y = pt.y;
    // Copy Z coordinate.
    added_pt.z = pt.z;
    // Copy intensity value.
    added_pt.intensity = pt.intensity;
    // Compute offset time in milliseconds relative to the first point.
    added_pt.curvature = (pt.timestamp - time_head) * 1000.0;
    // Append the converted point to the surface cloud.
    pl_surf.points.push_back(added_pt);
  }
  // Sort surface points by their curvature (timestamp) in ascending order.
  std::sort(pl_surf.points.begin(), pl_surf.points.end(), [](const PointType &a, const PointType &b) {
    return a.curvature < b.curvature;
  });
}

// Classifies each point in a scan line as plane, edge, or wire based on local
// geometric properties (range, distance, angles, planarity checks).
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  // Get the total number of points in the scan line.
  int plsize = pl.size();
  // Secondary size variable for loop bounds throughout the function.
  int plsize2;
  // Return early if there are no points.
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  // Starting index for processing (skip points inside the blind zone).
  uint head = 0;

  // Advance head past any points that fall inside the blind zone.
  while (types[head].range < blind_sqr)
  {
    head++;
  }

  // Surf
  // Compute the upper bound for the main plane-judgment loop (leave room for group_size).
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  // Current direction vector for the plane being evaluated.
  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  // Direction vector from the previous plane evaluation.
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  // Index of the next point after the current group (output from plane_judge).
  uint i_nex = 0, i2;
  // Index of the last starting point for a plane segment.
  uint last_i = 0;
  // Index of the last next-point for a plane segment.
  uint last_i_nex = 0;
  // State flag from the last evaluated group (1 = planar, 0 = non-planar).
  int last_state = 0;
  // Result code from plane_judge (1 = plane, 0 = not plane, 2 = blind zone).
  int plane_type;

  // Main loop: evaluate each point in the scan line for planarity.
  for (uint i = head; i < plsize2; i++)
  {
    // Skip points inside the blind zone.
    if (types[i].range < blind_sqr) { continue; }

    // Save the current index before plane judgment overwrites i.
    i2 = i;

    // Evaluate whether the points starting from i form a planar surface.
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    // If the group was classified as planar (return value 1).
    if (plane_type == 1)
    {
      // Label each point in the planar segment with the appropriate type.
      for (uint j = i; j <= i_nex; j++)
      {
        // Interior points are labeled as Real_Plane; endpoints are Poss_Plane.
        if (j != i && j != i_nex) { types[j].ftype = Real_Plane; }
        else { types[j].ftype = Poss_Plane; }
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      // If the previous group was also planar, check direction continuity.
      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        // Compute the dot product between the previous and current plane directions.
        double mod = last_direct.transpose() * curr_direct;
        // Large direction change (angle near 90 degrees) indicates an edge-plane boundary.
        if (mod > -0.707 && mod < 0.707) { types[i].ftype = Edge_Plane; }
        // Small direction change keeps the point classified as Real_Plane.
        else { types[i].ftype = Real_Plane; }
      }

      // Advance i to the end of this planar group (the loop increment will add 1).
      i = i_nex - 1;
      // Record that the last evaluated state was planar.
      last_state = 1;
    }
    // Non-planar group: plane_judge returned 0 or 2.
    else // if(plane_type == 2)
    {
      // Advance i to the next candidate start index.
      i = i_nex;
      // Record that the last evaluated state was non-planar.
      last_state = 0;
    }

    // Update the tracking indices for the next iteration.
    last_i = i2;
    last_i_nex = i_nex;
    // Update the direction vector for continuity checking.
    last_direct = curr_direct;
  }

  // Compute the upper bound for the edge-judgment loop (need at least 3 points from end).
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  // Edge detection pass: check for jump edges and wire-like features.
  for (uint i = head + 3; i < plsize2; i++)
  {
    // Skip blind-zone points and points already classified as planar (Real_Plane or higher).
    if (types[i].range < blind_sqr || types[i].ftype >= Real_Plane) { continue; }

    // Skip points where either adjacent inter-point distance is negligibly small.
    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) { continue; }

    // Vector from origin to the current point.
    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    // Array for vectors to the previous (index 0) and next (index 1) neighbors.
    Eigen::Vector3d vecs[2];

    // Evaluate both the previous (j=0) and next (j=1) neighbors.
    for (int j = 0; j < 2; j++)
    {
      // Offset direction: -1 for previous neighbor, +1 for next neighbor.
      int m = -1;
      if (j == 1) { m = 1; }

      // If the neighbor point is inside the blind zone, mark the edge direction accordingly.
      if (types[i + m].range < blind_sqr)
      {
        // Mark as infinity edge if the current point is far, otherwise as blind.
        if (types[i].range > inf_bound) { types[i].edj[j] = Nr_inf; }
        else { types[i].edj[j] = Nr_blind; }
        continue;
      }

      // Compute the vector from the current point to the neighbor point.
      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      // Subtract the current point position to get the relative displacement.
      vecs[j] = vecs[j] - vec_a;

      // Compute the cosine of the angle between the point vector and the neighbor vector.
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      // Classify the edge direction based on the cosine threshold.
      if (types[i].angle[j] < jump_up_limit) { types[i].edj[j] = Nr_180; }
      else if (types[i].angle[j] > jump_down_limit) { types[i].edj[j] = Nr_zero; }
    }

    // Compute the cosine of the angle between the two neighbor vectors (previous and next).
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    // Check for a jump edge at the previous-neighbor side.
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      // Verify the edge with the geometric edge-jump judge.
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Prev)) { types[i].ftype = Edge_Jump; }
      }
    }
    // Check for a jump edge at the next-neighbor side.
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next)) { types[i].ftype = Edge_Jump; }
      }
    }
    // Check for an infinity-marked edge on the previous side.
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      // Verify the edge with the geometric edge-jump judge toward the previous side.
      if (edge_jump_judge(pl, types, i, Prev)) { types[i].ftype = Edge_Jump; }
    }
    // Check for an infinity-marked edge on the next side.
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      // Verify the edge with the geometric edge-jump judge toward the next side.
      if (edge_jump_judge(pl, types, i, Next)) { types[i].ftype = Edge_Jump; }
    }
    // If both edge directions are marked abnormal (non-normal), classify as Wire.
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      // Only upgrade to Wire if the current type is still Nor (normal).
      if (types[i].ftype == Nor) { types[i].ftype = Wire; }
    }
  }

  // Compute the upper bound for the small-plane refinement loop (exclude last point).
  plsize2 = plsize - 1;
  // Ratio variable for comparing adjacent inter-point distances.
  double ratio;
  // Refinement pass: promote isolated planar-like points to Real_Plane.
  for (uint i = head + 1; i < plsize2; i++)
  {
    // Skip points near the blind zone or with neighbors in the blind zone.
    if (types[i].range < blind_sqr || types[i - 1].range < blind_sqr || types[i + 1].range < blind_sqr) { continue; }

    // Skip points with negligibly small inter-point distances (likely duplicate points).
    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) { continue; }

    // Only process points currently classified as normal (Nor).
    if (types[i].ftype == Nor)
    {
      // Compute the ratio between the larger and smaller adjacent distances.
      if (types[i - 1].dista > types[i].dista) { ratio = types[i - 1].dista / types[i].dista; }
      else { ratio = types[i].dista / types[i - 1].dista; }

      // Check if the point meets the small-plane criteria (tight angle and similar distances).
      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        // Promote the previous normal point to Real_Plane if it is still Nor.
        if (types[i - 1].ftype == Nor) { types[i - 1].ftype = Real_Plane; }
        // Promote the next normal point to Real_Plane if it is still Nor.
        if (types[i + 1].ftype == Nor) { types[i + 1].ftype = Real_Plane; }
        // Promote the current point to Real_Plane.
        types[i].ftype = Real_Plane;
      }
    }
  }

  // Index tracking the start of the current surface segment (-1 means no active segment).
  int last_surface = -1;
  // Final pass: collect surface and edge points into the output clouds.
  for (uint j = head; j < plsize; j++)
  {
    // If the point is classified as a surface point (possible or real plane).
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      // Mark the start of a new surface segment if not already tracking one.
      if (last_surface == -1) { last_surface = j; }

      // Decimate: only output every point_filter_num-th surface point.
      if (j == uint(last_surface + point_filter_num - 1))
      {
        // Create a new point to add to the surface cloud.
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.curvature = pl[j].curvature;
        // Add the decimated surface point to the output.
        pl_surf.push_back(ap);

        // Reset the surface segment tracker (single point segment consumed).
        last_surface = -1;
      }
    }
    // Non-surface point (edge, wire, or normal/unclassified).
    else
    {
      // Collect edge-type points (Edge_Jump or Edge_Plane) into the corner cloud.
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) { pl_corn.push_back(pl[j]); }
      // If we were tracking a surface segment, finalize it by averaging.
      if (last_surface != -1)
      {
        // Accumulate all points in the surface segment.
        PointType ap;
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.curvature += pl[k].curvature;
        }
        // Compute the centroid (mean) of the surface segment.
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        // Add the averaged surface point to the output cloud.
        pl_surf.push_back(ap);
      }
      // Reset the surface segment tracker.
      last_surface = -1;
    }
  }
}

// Publishes a point cloud as a ROS PointCloud2 message with the livox frame.
void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  // Set the point cloud height to 1 (unorganized point cloud).
  pl.height = 1;
  // Set the point cloud width to the total number of points.
  pl.width = pl.size();
  // Output ROS PointCloud2 message object.
  sensor_msgs::PointCloud2 output;
  // Convert the PCL point cloud to a ROS message.
  pcl::toROSMsg(pl, output);
  // Set the frame ID for the output message to "livox".
  output.header.frame_id = "livox";
  // Set the timestamp for the output message.
  output.header.stamp = ct;
}

// Judges whether a group of consecutive points forms a planar surface by
// checking curvature, distance ratios, and local linearity.
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  // Compute the adaptive group distance threshold based on the current point's range.
  double group_dis = disA * types[i_cur].range + disB;
  // Square the threshold for direct comparison with squared Euclidean distances.
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  // Temporary variable for the direct distance between first and last points in the group.
  double two_dis;
  // Container for storing all inter-point distances within the group.
  vector<double> disarr;
  // Pre-allocate space for up to 20 distance values.
  disarr.reserve(20);

  // Collect the initial group_size inter-point distances into the array.
  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    // If any point in the initial group is inside the blind zone, abort with code 2.
    if (types[i_nex].range < blind_sqr)
    {
      curr_direct.setZero();
      return 2;
    }
    // Record the inter-point distance for the current point.
    disarr.push_back(types[i_nex].dista);
  }

  // Expand the group outward while the chord distance (first to last) stays under the threshold.
  for (;;)
  {
    // Safety check: prevent index out of bounds on the point cloud.
    if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    // If the candidate expansion point is inside the blind zone, abort with code 2.
    if (types[i_nex].range < blind_sqr)
    {
      curr_direct.setZero();
      return 2;
    }
    // Compute the vector from the first point in the group to the candidate expansion point.
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    // Compute the squared chord distance from first to candidate.
    two_dis = vx * vx + vy * vy + vz * vz;
    // Stop expanding if the chord distance exceeds the adaptive group threshold.
    if (two_dis >= group_dis) { break; }
    // Record the inter-point distance and continue expanding.
    disarr.push_back(types[i_nex].dista);
    // Advance to the next candidate point.
    i_nex++;
  }

  // Maximum perpendicular distance (width) of intermediate points from the chord.
  double leng_wid = 0;
  // Arrays for intermediate vectors and cross product results.
  double v1[3], v2[3];
  // Compute the perpendicular deviation of each intermediate point from the chord line.
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    // Safety check: prevent index out of bounds.
    if ((j >= pl.size()) || (i_cur >= pl.size())) break;
    // Vector from the first point in the group to the intermediate point.
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    // Cross product of v1 and the group chord vector (vx, vy, vz).
    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    // Squared magnitude of the cross product (proportional to perpendicular distance squared).
    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    // Keep the maximum perpendicular distance value.
    if (lw > leng_wid) { leng_wid = lw; }
  }

  // If the perpendicular deviation is too large relative to chord length, reject as non-planar.
  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  // Sort the inter-point distances in descending order.
  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  // If the second-smallest distance is effectively zero, the points are too close (degenerate).
  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  // For Livox Avia, apply the stricter max/mid and mid/min distance ratio checks.
  if (lidar_type == AVIA)
  {
    // Ratio of the largest to median distance.
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    // Ratio of the median to second-smallest distance.
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    // Reject if either ratio exceeds its configured threshold (indicating non-uniform spacing).
    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  // For all other LiDAR types, use the simpler max/min ratio check.
  else
  {
    // Ratio of the largest to second-smallest distance.
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    // Reject if the ratio exceeds the configured threshold.
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  // Set the direction vector to the chord vector (from first to last point in the group).
  curr_direct << vx, vy, vz;
  // Normalize the direction vector to unit length.
  curr_direct.normalize();
  // Return 1 to indicate that the group is classified as a planar surface.
  return 1;
}

// Determines whether a range discontinuity between consecutive points is caused
// by a true edge in the scene (as opposed to noise or occlusion).
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  // If checking the previous direction (Prev = 0).
  if (nor_dir == 0)
  {
    // If either of the two preceding points is inside the blind zone, reject the edge.
    if (types[i - 1].range < blind_sqr || types[i - 2].range < blind_sqr) { return false; }
  }
  // If checking the next direction (Next = 1).
  else if (nor_dir == 1)
  {
    // If either of the two following points is inside the blind zone, reject the edge.
    if (types[i + 1].range < blind_sqr || types[i + 2].range < blind_sqr) { return false; }
  }
  // Retrieve the inter-point distances at the edge-adjacent positions.
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  // Temporary variable for swapping.
  double d;

  // Ensure d1 holds the larger of the two distances.
  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  // Convert squared distances to linear distances.
  d1 = sqrt(d1);
  d2 = sqrt(d2);

  // Reject if the distance ratio exceeds the edgea multiplier or the absolute difference exceeds edgeb.
  if (d1 > edgea * d2 || (d1 - d2) > edgeb) { return false; }

  // Accept the discontinuity as a genuine edge.
  return true;
}