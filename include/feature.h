/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Header guard to prevent multiple inclusions of this file.
#ifndef LIVO_FEATURE_H_
// Define the header guard macro.
#define LIVO_FEATURE_H_

// Include VisualPoint for the 3D point associated with this feature.
#include "visual_point.h"

// Represents a visual patch feature extracted at a pixel location in an image frame.
// A salient image region that is tracked across frames.
struct Feature
{
  // Macro to enforce proper Eigen alignment for structures with Eigen member types.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Enumeration of feature types detected in the image.
  enum FeatureType
  {
    // Corner feature (e.g., from FAST corner detector).
    CORNER,
    // Edgelet feature (edge-based feature).
    EDGELET
  };
  // Unique identifier for this feature (used for debugging and tracking).
  int id_;
  // Type of this feature: CORNER or EDGELET.
  FeatureType type_;     //!< Type can be corner or edgelet.
  // Image in which this feature was extracted.
  cv::Mat img_;          //!< Image associated with the patch feature
  // Pixel coordinates of this feature on the base image pyramid level (level 0).
  Vector2d px_;          //!< Coordinates in pixels on pyramid level 0.
  // Unit-bearing vector (ray direction) from the camera center through the feature.
  Vector3d f_;           //!< Unit-bearing vector of the patch feature.
  // Image pyramid level at which this feature was detected and extracted.
  int level_;            //!< Image pyramid level where patch feature was extracted.
  // Pointer to the associated 3D visual point in the map.
  VisualPoint *point_;   //!< Pointer to 3D point which corresponds to the patch feature.
  // Dominant gradient direction (used for edgelet features).
  Vector2d grad_;        //!< Dominant gradient direction for edglets, normalized.
  // Camera pose at the time this feature was extracted (world to frame).
  SE3 T_f_w_;            //!< Pose of the frame where the patch feature was extracted.
  // Pointer to the raw pixel data of the extracted image patch.
  float *patch_;         //!< Pointer to the image patch data.
  // Quality score for this feature (used for feature selection and culling).
  float score_;          //!< Score of the patch feature.
  // Mean intensity value of the image patch, used for intensity normalization.
  float mean_;           //!< Mean intensity of the image patch feature, used for normalization.
  // Inverse exposure time for photometric calibration of the source image.
  double inv_expo_time_; //!< Inverse exposure time of the image where the patch feature was extracted.
  
  // Constructor: initializes a feature with its associated point, patch, pixel, bearing, pose, and pyramid level.
  Feature(VisualPoint *_point, float *_patch, const Vector2d &_px, const Vector3d &_f, const SE3 &_T_f_w, int _level)
      : type_(CORNER), px_(_px), f_(_f), T_f_w_(_T_f_w), mean_(0), score_(0), level_(_level), patch_(_patch), point_(_point)
  {
    // Constructor body is empty; all initialization is done via the initializer list.
  }

  // Get the camera position in the world frame at the time this feature was extracted.
  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }
  
  // Destructor: deallocates the image patch memory.
  ~Feature()
  {
    // ROS_WARN("The feature %d has been destructed.", id_);
    // Release the heap-allocated patch data.
    delete[] patch_;
  }
// Closing brace for Feature struct.
};

// End of header guard.
#endif // LIVO_FEATURE_H_
