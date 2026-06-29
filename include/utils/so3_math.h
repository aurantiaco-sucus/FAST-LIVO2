// Begin include guard to prevent multiple inclusions of this header.
#ifndef SO3_MATH_H
// Define the include guard symbol for the SO(3) math header.
#define SO3_MATH_H

// Include Eigen core matrix and vector types for linear algebra operations.
#include <Eigen/Core>
// Include standard C math functions (sin, cos, acos, sqrt, atan2, etc.).
#include <math.h>

// Macro to initialize a 3x3 skew-symmetric matrix from a 3-element vector v.
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

// Compute the SO(3) exponential map (rotation matrix) from an rvalue angle-axis vector.
template <typename T> Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
  // Compute the Euclidean norm (rotation angle magnitude) of the angle-axis vector.
  T ang_norm = ang.norm();
  // Create a 3x3 identity matrix as the base for the Rodrigues formula.
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  // Only apply the Rodrigues formula if the rotation angle is above a small threshold.
  if (ang_norm > 0.0000001)
  {
    // Normalize the angle-axis vector to obtain the unit rotation axis.
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    // Declare the skew-symmetric matrix of the rotation axis.
    Eigen::Matrix<T, 3, 3> K;
    // Initialize K as the skew-symmetric matrix of the unit rotation axis.
    K << SKEW_SYM_MATRX(r_axis);
    /// Roderigous Tranformation
    // Apply the Rodrigues rotation formula to compute the rotation matrix.
    return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }
  // Return the identity matrix if the angle is effectively zero.
  else { return Eye3; }
}

// Compute the SO(3) exponential map scaled by a time increment dt (angular velocity integration).
template <typename T, typename Ts> Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
  // Compute the norm of the angular velocity vector.
  T ang_vel_norm = ang_vel.norm();
  // Create a 3x3 identity matrix for the zero-rotation case.
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  // Only compute the rotation if the angular velocity magnitude is above the threshold.
  if (ang_vel_norm > 0.0000001)
  {
    // Obtain the unit rotation axis from the angular velocity vector.
    Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    // Declare the skew-symmetric matrix of the rotation axis.
    Eigen::Matrix<T, 3, 3> K;
    // Fill K with the skew-symmetric entries derived from the unit axis.
    K << SKEW_SYM_MATRX(r_axis);
    // Compute the total rotation angle as angular velocity magnitude times the time step.
    T r_ang = ang_vel_norm * dt;
    /// Roderigous Tranformation
    // Use the Rodrigues formula with the integrated angle to obtain the rotation matrix.
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }
  // Return identity if the angular velocity is negligible.
  else { return Eye3; }
}

// Compute the SO(3) exponential map from three individual scalar components (v1, v2, v3).
template <typename T> Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
  // Compute the Euclidean norm from the three scalar components.
  T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  // Create a 3x3 identity matrix for the zero-angle case.
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  // Only apply the Rodrigues formula if the norm exceeds the small threshold.
  if (norm > 0.00001)
  {
    // Normalize each component to obtain the unit rotation axis array.
    T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
    // Declare the skew-symmetric matrix of the unit rotation axis.
    Eigen::Matrix<T, 3, 3> K;
    // Initialize K from the normalized axis components.
    K << SKEW_SYM_MATRX(r_ang);
    /// Roderigous Tranformation
    // Apply the Rodrigues rotation formula using the computed norm as the angle.
    return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
  }
  // Return identity matrix if the rotation angle is effectively zero.
  else { return Eye3; }
}

/* Logrithm of a Rotation Matrix */
// Compute the SO(3) logarithmic map (angle-axis vector) from a rotation matrix.
template <typename T> Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
  // Compute the rotation angle theta from the matrix trace, handling near-identity cases.
  T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  // Extract the unscaled skew-symmetric components to form the rotation vector direction.
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  // Return the scaled rotation vector; use small-angle approximation for near-zero angles.
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

// Convert a 3x3 rotation matrix to Euler angles (ZYX convention: yaw, pitch, roll).
template <typename T> Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
  // Compute the sine of the pitch angle from the first column of the rotation matrix.
  T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  // Flag indicating whether the pitch angle is near +/-90 degrees (gimbal lock).
  bool singular = sy < 1e-6;
  // Declare storage for the three Euler angles.
  T x, y, z;
  // Use the standard ZYX Euler angle extraction if not at a singularity.
  if (!singular)
  {
    // Extract the roll angle from the rotation matrix entries.
    x = atan2(rot(2, 1), rot(2, 2));
    // Extract the pitch angle from the rotation matrix entries.
    y = atan2(-rot(2, 0), sy);
    // Extract the yaw angle from the rotation matrix entries.
    z = atan2(rot(1, 0), rot(0, 0));
  }
  // Handle the gimbal lock case where pitch is near +/-90 degrees.
  else
  {
    // Set roll using an alternative formula valid at the singularity.
    x = atan2(-rot(1, 2), rot(1, 1));
    // Pitch is still extracted via the same formula.
    y = atan2(-rot(2, 0), sy);
    // Set yaw to zero at the singularity.
    z = 0;
  }
  // Pack the three Euler angles into a 3-element vector.
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  // Return the Euler angle vector (roll, pitch, yaw).
  return ang;
}

// End of the SO(3) math header include guard.
#endif
