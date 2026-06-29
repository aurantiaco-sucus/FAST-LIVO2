/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Header guard to prevent multiple inclusions of this file.
#ifndef LIVO_POINT_H_
// Define the header guard macro.
#define LIVO_POINT_H_

// Include boost noncopyable to prevent copying of VisualPoint objects.
#include <boost/noncopyable.hpp>
// Include shared type definitions (Vector3d, Matrix3d, etc.).
#include "common_lib.h"
// Include frame definitions (Feature forward declaration, Frame type aliases).
#include "frame.h"

// Forward declaration of Feature (defined in feature.h).
class Feature;

/// A visual map point on the surface of the scene.
class VisualPoint : boost::noncopyable
{
// Public member variables and methods.
public:
  // Macro to enforce proper Eigen alignment for classes with Eigen member types.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 3D position of the point in the world coordinate frame.
  Vector3d pos_;                //!< 3d pos of the point in the world coordinate frame.
  // Estimated surface normal direction at the point.
  Vector3d normal_;             //!< Surface normal at point.
  // Inverse covariance matrix (information matrix) of the normal estimate.
  Matrix3d normal_information_; //!< Inverse covariance matrix of normal estimation.
  // Surface normal estimate from the previous update step.
  Vector3d previous_normal_;    //!< Last updated normal vector.
  // List of features (reference patches) that observe this 3D point.
  list<Feature *> obs_;         //!< Reference patches which observe the point.
  // 3x3 covariance matrix of the point position estimate.
  Eigen::Matrix3d covariance_;  //!< Covariance of the point.
  // Flag indicating whether the point position has converged.
  bool is_converged_;           //!< True if the point is converged.
  // Flag indicating whether the surface normal has been initialized.
  bool is_normal_initialized_;  //!< True if the normal is initialized.
  // Flag indicating whether this point has a valid reference patch for tracking.
  bool has_ref_patch_;          //!< True if the point has a reference patch.
  // Pointer to the reference patch feature used for photometric alignment.
  Feature *ref_patch;           //!< Reference patch of the point.

  // Constructor: creates a visual point at the given 3D position.
  VisualPoint(const Vector3d &pos);
  // Destructor: removes itself from all observing features.
  ~VisualPoint();
  // Find the feature with the lowest score among observations of this point.
  void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;
  // Delete all features that are not the selected reference patch.
  void deleteNonRefPatchFeatures();
  // Remove a specific feature from this point's observation list.
  void deleteFeatureRef(Feature *ftr);
  // Add a feature as a new observation reference to this point.
  void addFrameRef(Feature *ftr);
  // Find an observation of this point taken from a similar viewpoint, for reference patch selection.
  bool getCloseViewObs(const Vector3d &pos, Feature *&obs, const Vector2d &cur_px) const;
// Closing brace for VisualPoint class.
};

// End of header guard.
#endif // LIVO_POINT_H_
