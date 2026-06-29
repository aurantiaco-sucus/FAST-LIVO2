/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Include the IMU processing header with the ImuProcess class declaration and dependencies.
#include "IMU_Processing.h"

// Constructor: initializes noise covariances, IMU offsets, and default state.
ImuProcess::ImuProcess() : Eye3d(M3D::Identity()),
                           Zero3d(0, 0, 0), b_first_frame(true), imu_need_init(true)
{
  // Set the initialization iteration counter to 1.
  init_iter_num = 1;
  // Set the accelerometer measurement noise covariance.
  cov_acc = V3D(0.1, 0.1, 0.1);
  // Set the gyroscope measurement noise covariance.
  cov_gyr = V3D(0.1, 0.1, 0.1);
  // Set the gyroscope bias random walk covariance.
  cov_bias_gyr = V3D(0.1, 0.1, 0.1);
  // Set the accelerometer bias random walk covariance.
  cov_bias_acc = V3D(0.1, 0.1, 0.1);
  // Set the inverse exposure time covariance for rolling-shutter estimation.
  cov_inv_expo = 0.2;
  // Initialize the mean accelerometer reading to the default gravity-aligned vector.
  mean_acc = V3D(0, 0, -1.0);
  // Initialize the mean gyroscope reading to zero.
  mean_gyr = V3D(0, 0, 0);
  // Initialize the last angular velocity to zero.
  angvel_last = Zero3d;
  // Initialize the last specific acceleration to zero.
  acc_s_last = Zero3d;
  // Initialize the LiDAR-to-IMU translation offset to zero.
  Lid_offset_to_IMU = Zero3d;
  // Initialize the LiDAR-to-IMU rotation matrix to identity.
  Lid_rot_to_IMU = Eye3d;
  // Allocate and store a default last IMU message.
  last_imu.reset(new sensor_msgs::Imu());
  // Allocate and store an empty current undistorted point cloud.
  cur_pcl_un_.reset(new PointCloudXYZI());
}
// End of ImuProcess constructor.

// Empty destructor for the ImuProcess class.
ImuProcess::~ImuProcess() {}

// Resets the IMU processor to its initial state for re-initialization.
void ImuProcess::Reset()
{
  // Log a warning that the IMU processor is being reset.
  ROS_WARN("Reset ImuProcess");
  // Reset the mean accelerometer reading to the default gravity direction.
  mean_acc = V3D(0, 0, -1.0);
  // Reset the mean gyroscope reading to zero.
  mean_gyr = V3D(0, 0, 0);
  // Reset the last angular velocity to zero.
  angvel_last = Zero3d;
  // Mark the IMU as needing re-initialization.
  imu_need_init = true;
  // Reset the initialization iteration counter to 1.
  init_iter_num = 1;
  // Clear the stored IMU pose trajectory buffer.
  IMUpose.clear();
  // Re-allocate a default last IMU message.
  last_imu.reset(new sensor_msgs::Imu());
  // Re-allocate an empty current undistorted point cloud.
  cur_pcl_un_.reset(new PointCloudXYZI());
}
// End of Reset.

// Disables IMU processing: skips propagation and initialization steps.
void ImuProcess::disable_imu()
{
  // Print a message indicating that IMU processing has been disabled.
  cout << "IMU Disabled !!!!!" << endl;
  // Set the IMU enable flag to false.
  imu_en = false;
  // Mark that IMU initialization is no longer required.
  imu_need_init = false;
}
// End of disable_imu.

// Disables online gravity vector estimation in the EKF prediction.
void ImuProcess::disable_gravity_est()
{
  // Print a message indicating that gravity estimation has been disabled.
  cout << "Online Gravity Estimation Disabled !!!!!" << endl;
  // Set the gravity estimation enable flag to false.
  gravity_est_en = false;
}
// End of disable_gravity_est.

// Disables online gyroscope and accelerometer bias estimation.
void ImuProcess::disable_bias_est()
{
  // Print a message indicating that bias estimation has been disabled.
  cout << "Bias Estimation Disabled !!!!!" << endl;
  // Set the bias estimation enable flag to false.
  ba_bg_est_en = false;
}
// End of disable_bias_est.

// Disables online exposure-time estimation for rolling-shutter camera.
void ImuProcess::disable_exposure_est()
{
  // Print a message indicating that time offset estimation has been disabled.
  cout << "Online Time Offset Estimation Disabled !!!!!" << endl;
  // Set the exposure estimation enable flag to false.
  exposure_estimate_en = false;
}
// End of disable_exposure_est.

// Set the LiDAR-to-IMU extrinsic calibration from a 4x4 homogeneous transformation matrix.
void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  // Extract the translation component from the 4x4 extrinsic matrix.
  Lid_offset_to_IMU = T.block<3, 1>(0, 3);
  // Extract the rotation component from the 4x4 extrinsic matrix.
  Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}
// End of set_extrinsic (4x4 matrix overload).

// Set only the translation part of the LiDAR-to-IMU extrinsic calibration.
void ImuProcess::set_extrinsic(const V3D &transl)
{
  // Set the LiDAR-to-IMU translation from the provided vector.
  Lid_offset_to_IMU = transl;
  // Set the LiDAR-to-IMU rotation to identity (no rotation).
  Lid_rot_to_IMU.setIdentity();
}
// End of set_extrinsic (translation-only overload).

// Set both the translation and rotation components of the LiDAR-to-IMU extrinsic.
void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  // Set the LiDAR-to-IMU translation from the provided vector.
  Lid_offset_to_IMU = transl;
  // Set the LiDAR-to-IMU rotation from the provided rotation matrix.
  Lid_rot_to_IMU = rot;
}
// End of set_extrinsic (translation and rotation overload).

// Set the gyroscope measurement noise covariance vector.
void ImuProcess::set_gyr_cov_scale(const V3D &scaler) { cov_gyr = scaler; }

// Set the accelerometer measurement noise covariance vector.
void ImuProcess::set_acc_cov_scale(const V3D &scaler) { cov_acc = scaler; }

// Set the gyroscope bias random walk covariance vector.
void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

// Set the inverse exposure time covariance for rolling-shutter estimation.
void ImuProcess::set_inv_expo_cov(const double &inv_expo) { cov_inv_expo = inv_expo; }

// Set the accelerometer bias random walk covariance vector.
void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

// Set the maximum number of IMU frames used during initialization.
void ImuProcess::set_imu_init_frame_num(const int &num) { MAX_INI_COUNT = num; }

// Initializes the IMU: accumulates gyro/accel measurements, estimates mean
// gravity direction, and sets initial biases to zero. Repeats over multiple
// frames until MAX_INI_COUNT is reached.
void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  // Declare vectors for current accelerometer and gyroscope readings.
  V3D cur_acc, cur_gyr;

  // Check if this is the very first frame of data.
  if (b_first_frame)
  {
    // Reset all IMU processor state to defaults.
    Reset();
    // Initialize the iteration counter to 1.
    N = 1;
    // Mark that the first frame has been processed.
    b_first_frame = false;
    // Get the first IMU accelerometer reading.
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    // Get the first IMU gyroscope reading.
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    // Store the first accelerometer reading as the initial mean.
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    // Store the first gyroscope reading as the initial mean.
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  // Iterate over all IMU measurements in the current group.
  for (const auto &imu : meas.imu)
  {
    // Get the accelerometer reading from the current IMU message.
    const auto &imu_acc = imu->linear_acceleration;
    // Get the gyroscope reading from the current IMU message.
    const auto &gyr_acc = imu->angular_velocity;
    // Copy the accelerometer reading into the Eigen vector.
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    // Copy the gyroscope reading into the Eigen vector.
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    // Accumulate the running mean of accelerometer readings.
    mean_acc += (cur_acc - mean_acc) / N;
    // Accumulate the running mean of gyroscope readings.
    mean_gyr += (cur_gyr - mean_gyr) / N;

    // Increment the sample count.
    N++;
  }
  // Compute the norm of the mean accelerometer vector.
  IMU_mean_acc_norm = mean_acc.norm();
  // Set the gravity vector from the normalized mean acceleration scaled by G_m_s2.
  state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;
  // Initialize the rotation end state to identity.
  state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  // Initialize the gyroscope bias to zero.
  state_inout.bias_g = Zero3d; // mean_gyr;

  // Store the last IMU message for the next propagation step.
  last_imu = meas.imu.back();
}
// End of IMU_init.

// Forward propagates the state and covariance using a constant-velocity model
// when IMU data is unavailable. Also un-distorts the LiDAR scan.
void ImuProcess::Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  // Copy the input LiDAR point cloud to the output point cloud.
  pcl_out = *(meas.lidar);
  /*** sort point clouds by offset time ***/
  // Get the beginning timestamp of the current LiDAR frame.
  const double &pcl_beg_time = meas.lidar_frame_beg_time;
  // Sort the point cloud points by their offset timestamp in ascending order.
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // Compute the end timestamp of the LiDAR scan.
  const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  // Store the last LIO update time as the end of the scan.
  meas.last_lio_update_time = pcl_end_time;
  // Extract the offset time of the last point relative to the scan start.
  const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);

  // Declare the state transition matrix and process noise covariance.
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  // Initialize the time step variable.
  double dt = 0;

  // Check if this is the very first frame of data.
  if (b_first_frame)
  {
    // Use a default time step of 0.1 seconds for the first frame.
    dt = 0.1;
    // Mark that the first frame has been processed.
    b_first_frame = false;
  }
  else { dt = pcl_beg_time - time_last_scan; }

  // Record the current scan start time for the next propagation step.
  time_last_scan = pcl_beg_time;
  // Compute the exponential map of the gyroscope bias over the time step.
  M3D Exp_f = Exp(state_inout.bias_g, dt);

  // Set the state transition matrix to identity.
  F_x.setIdentity();
  // Set the process noise covariance to zero.
  cov_w.setZero();

  // Set the rotation sub-block: rotation propagated backward by -dt.
  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  // Set the gyroscope-bias-to-rotation coupling sub-block.
  F_x.block<3, 3>(0, 10) = Eye3d * dt;
  // Set the velocity-to-position integration sub-block.
  F_x.block<3, 3>(3, 7) = Eye3d * dt;

  // Set the gyroscope process noise covariance block.
  cov_w.block<3, 3>(10, 10).diagonal() = cov_gyr * dt * dt; // for omega in constant model
  // Set the accelerometer process noise covariance block.
  cov_w.block<3, 3>(7, 7).diagonal() = cov_acc * dt * dt; // for velocity in constant model
  // Propagate the state covariance matrix using the linearized model.
  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
  // Propagate the rotation end state.
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  // Propagate the position end state using the constant velocity assumption.
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;

  // Skip point undistortion for L515 LiDAR (solid-state, no rolling shutter).
  if (lidar_type != L515)
  {
    // Start iterating from the last point in the cloud.
    auto it_pcl = pcl_out.points.end() - 1;
    // Declare the per-point time offset from the scan end.
    double dt_j = 0.0;
    // Iterate backward through all points in the cloud.
    for(; it_pcl != pcl_out.points.begin(); it_pcl--)
    {
        // Compute the offset time of the current point from the scan end.
        dt_j= pcl_end_offset_time - it_pcl->curvature/double(1000);
        // Compute the rotation from the point's time to the scan end time.
        M3D R_jk(Exp(state_inout.bias_g, - dt_j));
        // Extract the original point position.
        V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
        // Using rotation and translation to un-distort points
        V3D p_jk;
        // Compute the motion compensation translation from velocity.
        p_jk = - state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;
  
        // Apply rotation and translation to undistort the point position.
        V3D P_compensate =  R_jk * P_j + p_jk;
  
        /// save Undistorted points and their rotation
        // Store the undistorted x-coordinate back into the point cloud.
        it_pcl->x = P_compensate(0);
        // Store the undistorted y-coordinate back into the point cloud.
        it_pcl->y = P_compensate(1);
        // Store the undistorted z-coordinate back into the point cloud.
        it_pcl->z = P_compensate(2);
    }
  }
}
// End of Forward_without_imu.


// Forward-propagates the EKF state through each IMU measurement, accumulating
// poses along the trajectory. Then backward-propagates each LiDAR point to
// remove motion distortion using the interpolated IMU poses.
void ImuProcess::UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  // Record the start time of the undistortion for performance measurement.
  double t0 = omp_get_wtime();
  // Clear the output point cloud.
  pcl_out.clear();
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  // Get the measurement group from the LiDAR measure group.
  MeasureGroup &meas = lidar_meas.measures.back();
  // cout<<"meas.imu.size: "<<meas.imu.size()<<endl;
  // Get the vector of IMU measurements for this frame.
  auto v_imu = meas.imu;
  // Prepend the last IMU message from the previous frame for continuity.
  v_imu.push_front(last_imu);
  // Get the timestamp of the first IMU message in the buffer.
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  // Get the timestamp of the last IMU message in the buffer.
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  // Get the timestamp at which the last propagation ended.
  const double prop_beg_time = last_prop_end_time;

  // Determine the propagation end time based on whether this is a LIO or VIO update.
  const double prop_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;

  // Check if the current update is a LIO update.
  if (lidar_meas.lio_vio_flg == LIO)
  {
    // Resize the processing point cloud to match the current processed cloud.
    pcl_wait_proc.resize(lidar_meas.pcl_proc_cur->points.size());
    // Copy the current processed LiDAR cloud into the waiting buffer.
    pcl_wait_proc = *(lidar_meas.pcl_proc_cur);
    // Initialize the LiDAR scan point index to 0.
    lidar_meas.lidar_scan_index_now = 0;
    // Push the initial pose (at offset 0) into the IMU pose buffer.
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
  }

  /*** forward propagation at each imu point ***/
  // Initialize IMU state variables from the input state.
  V3D acc_imu(acc_s_last), angvel_avr(angvel_last), acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  // cout << "[ IMU ] input state: " << state_inout.vel_end.transpose() << " " << state_inout.pos_end.transpose() << endl;
  // Initialize the rotation matrix from the input state.
  M3D R_imu(state_inout.rot_end);
  // Declare the state transition and process noise covariance matrices.
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  // Declare the time step and accumulated time variables.
  double dt, dt_all = 0.0;
  // Declare the offset time variable.
  double offs_t;
  // double imu_time;
  // Declare the exposure time parameter.
  double tau;
  // Check if the IMU time parameters have been initialized.
  if (!imu_time_init)
  {
    // Use a default tau value of 1.0.
    tau = 1.0;
    // Mark the IMU time initialization as complete.
    imu_time_init = true;
  }
  else
  {
    // Use the inverse exposure time from the state.
    tau = state_inout.inv_expo_time;
  }
  
  // Switch on the update type (LIO or VIO).
  switch (lidar_meas.lio_vio_flg)
  {
  // Handle both LIO and VIO propagation cases.
  case LIO:
  case VIO:
    // Reset the time step accumulator.
    dt = 0;
    // Iterate over consecutive IMU measurement pairs.
    for (int i = 0; i < v_imu.size() - 1; i++)
    {
      // Get the earlier IMU message in the pair.
      auto head = v_imu[i];
      // Get the later IMU message in the pair.
      auto tail = v_imu[i + 1];

      // Skip this pair if both messages are before the propagation start.
      if (tail->header.stamp.toSec() < prop_beg_time) continue;

      // Compute the average angular velocity between the two IMU readings.
      angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x), 0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
          0.5 * (head->angular_velocity.z + tail->angular_velocity.z);

      // Compute the average linear acceleration between the two IMU readings.
      acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x), 0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
          0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

      // #ifdef DEBUG_PRINT
      fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;
      // #endif

      // Remove the gyroscope bias from the average angular velocity.
      angvel_avr -= state_inout.bias_g;
      // Scale the acceleration to m/s^2 and remove the accelerometer bias.
      acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

      // Check if the head IMU is before the propagation start time.
      if (head->header.stamp.toSec() < prop_beg_time)
      {
        // printf("00 \n");
        // Compute the time step from the last propagation end to the tail.
        dt = tail->header.stamp.toSec() - last_prop_end_time;
        // Compute the offset time from propagation start to the tail.
        offs_t = tail->header.stamp.toSec() - prop_beg_time;
      }
      // Check if this is not the last IMU pair.
      else if (i != v_imu.size() - 2)
      {
        // printf("11 \n");
        // Compute the time step as the full interval between the two IMU messages.
        dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        // Compute the offset time from propagation start to the tail.
        offs_t = tail->header.stamp.toSec() - prop_beg_time;
      }
      // This is the last IMU pair, partial step to propagation end.
      else
      {
        // printf("22 \n");
        // Compute the partial time step to the propagation end.
        dt = prop_end_time - head->header.stamp.toSec();
        // Compute the offset time from propagation start to propagation end.
        offs_t = prop_end_time - prop_beg_time;
      }

      // Accumulate the total time step.
      dt_all += dt;
      // printf("[ LIO Propagation ] dt: %lf \n", dt);

      /* covariance propagation */
      // Declare the skew-symmetric matrix of the average acceleration.
      M3D acc_avr_skew;
      // Compute the exponential map of the average angular velocity over dt.
      M3D Exp_f = Exp(angvel_avr, dt);
      // Build the skew-symmetric matrix from the acceleration vector.
      acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

      // Set the state transition matrix to identity.
      F_x.setIdentity();
      // Set the process noise covariance to zero.
      cov_w.setZero();

      // Set the rotation sub-block in the state transition matrix.
      F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
      // Set the gyroscope-bias-to-rotation coupling if bias estimation is enabled.
      if (ba_bg_est_en) F_x.block<3, 3>(0, 10) = -Eye3d * dt;
      // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
      // Set the velocity-to-position integration sub-block.
      F_x.block<3, 3>(3, 7) = Eye3d * dt;
      // Set the rotation-to-velocity coupling via the gravity-skew term.
      F_x.block<3, 3>(7, 0) = -R_imu * acc_avr_skew * dt;
      // Set the accelerometer-bias-to-velocity coupling if bias estimation is enabled.
      if (ba_bg_est_en) F_x.block<3, 3>(7, 13) = -R_imu * dt;
      // Set the gravity-to-velocity coupling if gravity estimation is enabled.
      if (gravity_est_en) F_x.block<3, 3>(7, 16) = Eye3d * dt;

      // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);
      // F_x(6,6) = 0.25 * 2 * CV_PI * 0.5 * cos(2 * CV_PI * 0.5 * imu_time) * (-tau*tau); F_x(18,18) = 0.00001;
      // Set the inverse exposure time process noise if exposure estimation is enabled.
      if (exposure_estimate_en) cov_w(6, 6) = cov_inv_expo * dt * dt;
      // Set the gyroscope process noise covariance block.
      cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
      // Set the accelerometer process noise covariance block, rotated into the global frame.
      cov_w.block<3, 3>(7, 7) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
      // Set the gyroscope bias random walk process noise covariance block.
      cov_w.block<3, 3>(10, 10).diagonal() = cov_bias_gyr * dt * dt; // bias gyro covariance
      // Set the accelerometer bias random walk process noise covariance block.
      cov_w.block<3, 3>(13, 13).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance

      // Propagate the state covariance matrix.
      state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

      /* propogation of IMU attitude */
      // Update the rotation by the exponential map of the angular velocity.
      R_imu = R_imu * Exp_f;

      /* Specific acceleration (global frame) of IMU */
      // Compute the specific acceleration in the global frame including gravity.
      acc_imu = R_imu * acc_avr + state_inout.gravity;

      /* propogation of IMU */
      // Propagate the position using the current velocity and acceleration.
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

      /* velocity of IMU */
      // Propagate the velocity using the specific acceleration.
      vel_imu = vel_imu + acc_imu * dt;

      /* save the poses at each IMU measurements */
      // Store the angular velocity for the next iteration.
      angvel_last = angvel_avr;
      // Store the specific acceleration for the next iteration.
      acc_s_last = acc_imu;

      // Record the current IMU pose (offset, acceleration, gyro, velocity, position, rotation).
      IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }

    // Store the last LIO update time as the propagation end time.
    lidar_meas.last_lio_update_time = prop_end_time;
    break;
  }

  // Update the state's velocity from the forward propagation result.
  state_inout.vel_end = vel_imu;
  // Update the state's rotation from the forward propagation result.
  state_inout.rot_end = R_imu;
  // Update the state's position from the forward propagation result.
  state_inout.pos_end = pos_imu;
  // Update the inverse exposure time parameter in the state.
  state_inout.inv_expo_time = tau;

  // Store the last IMU message for the next propagation step.
  last_imu = v_imu.back();
  // Store the current propagation end time for the next step.
  last_prop_end_time = prop_end_time;

  // Record the end time of the forward propagation phase.
  double t1 = omp_get_wtime();

  // Return early if there are no points to undistort.
  if (pcl_wait_proc.points.size() < 1) return;

  /*** undistort each lidar point (backward propagation), ONLY working for LIO
   * update ***/
  // Check if the current update is a LIO update (only LIO points need undistortion).
  if (lidar_meas.lio_vio_flg == LIO)
  {
    // Start iterating from the last point in the waiting cloud.
    auto it_pcl = pcl_wait_proc.points.end() - 1;
    // Precompute the rotation from the end frame to the LiDAR frame.
    M3D extR_Ri(Lid_rot_to_IMU.transpose() * state_inout.rot_end.transpose());
    // Precompute the translation from the end frame to the LiDAR frame.
    V3D exrR_extT(Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU);
    // Iterate backward through the stored IMU poses.
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      // Get the earlier IMU pose in the pair.
      auto head = it_kp - 1;
      // Get the later IMU pose in the pair.
      auto tail = it_kp;
      // Reconstruct the rotation matrix from the stored IMU pose.
      R_imu << MAT_FROM_ARRAY(head->rot);
      // Reconstruct the acceleration from the stored IMU pose.
      acc_imu << VEC_FROM_ARRAY(head->acc);
      // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
      // Reconstruct the velocity from the stored IMU pose.
      vel_imu << VEC_FROM_ARRAY(head->vel);
      // Reconstruct the position from the stored IMU pose.
      pos_imu << VEC_FROM_ARRAY(head->pos);
      // Reconstruct the angular velocity from the stored IMU pose.
      angvel_avr << VEC_FROM_ARRAY(head->gyr);

      // Iterate over points whose offset time falls within the current IMU interval.
      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
      {
        // Compute the time offset of the point from the head IMU timestamp.
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* Transform to the 'end' frame */
        // Compute the rotation from the point's time to the IMU head time.
        M3D R_i(R_imu * Exp(angvel_avr, dt));
        // Compute the translation from the point's position to the scan-end position.
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - state_inout.pos_end);

        // Extract the original point coordinates.
        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        // Apply the full undistortion transform (LiDAR-to-IMU, motion compensation, IMU-to-end).
        V3D P_compensate = (extR_Ri * (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) - exrR_extT);

        /// save Undistorted points and their rotation
        // Store the undistorted x-coordinate back into the point cloud.
        it_pcl->x = P_compensate(0);
        // Store the undistorted y-coordinate back into the point cloud.
        it_pcl->y = P_compensate(1);
        // Store the undistorted z-coordinate back into the point cloud.
        it_pcl->z = P_compensate(2);

        // Stop if we have reached the first point.
        if (it_pcl == pcl_wait_proc.points.begin()) break;
      }
    }
    // Assign the undistorted point cloud to the output.
    pcl_out = pcl_wait_proc;
    // Clear the waiting point cloud buffer.
    pcl_wait_proc.clear();
    // Clear the IMU pose buffer for the next frame.
    IMUpose.clear();
  }
}
// End of UndistortPcl.

// Main IMU processing entry: performs initialization if needed, then calls
// point cloud undistortion with the current state.
void ImuProcess::Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
  // Declare performance timing variables.
  double t1, t2, t3;
  // Record the start time of the processing.
  t1 = omp_get_wtime();
  // Assert that the LiDAR measurement pointer is not null.
  ROS_ASSERT(lidar_meas.lidar != nullptr);
  // Check if IMU processing is enabled.
  if (!imu_en)
  {
    // Use the fallback forward-propagation without IMU data.
    Forward_without_imu(lidar_meas, stat, *cur_pcl_un_);
    // Return after the IMU-less propagation.
    return;
  }

  // Extract the measurement group from the LiDAR measure group.
  MeasureGroup meas = lidar_meas.measures.back();

  // Check if IMU initialization is still needed.
  if (imu_need_init)
  {
    // Determine the point cloud end time based on the update type.
    double pcl_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;

    // Return early if there are no IMU measurements available.
    if (meas.imu.empty()) { return; };
    /// The very first lidar frame
    // Perform IMU initialization with the current measurements.
    IMU_init(meas, stat, init_iter_num);

    // Keep the initialization flag true until enough frames are accumulated.
    imu_need_init = true;

    // Store the last IMU message for the next initialization frame.
    last_imu = meas.imu.back();

    // Check if enough initialization frames have been accumulated.
    if (init_iter_num > MAX_INI_COUNT)
    {
      // Mark initialization as complete.
      imu_need_init = false;
      // Log the final gravity estimate and covariances.
      ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: "
               "%.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f \n",
               stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm(), cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1],
               cov_gyr[2]);
      // Log the bias covariances.
      ROS_INFO("IMU Initials: ba covarience: %.8f %.8f %.8f; bg covarience: "
               "%.8f %.8f %.8f",
               cov_bias_acc[0], cov_bias_acc[1], cov_bias_acc[2], cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2]);
      // Open the IMU log file for writing debug data.
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
    }

    // Return without performing undistortion during initialization.
    return;
  }

  // Perform full IMU propagation and LiDAR point cloud undistortion.
  UndistortPcl(lidar_meas, stat, *cur_pcl_un_);
}
