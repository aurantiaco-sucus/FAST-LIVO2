/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Header guard to prevent multiple inclusions of this file.
#ifndef LIVO_FRAME_H_
// Define the header guard macro.
#define LIVO_FRAME_H_

// Include boost noncopyable to prevent copying of Frame objects.
#include <boost/noncopyable.hpp>
// Include abstract camera interface for projection functions.
#include <vikit/abstract_camera.h>

// Forward declaration of VisualPoint (defined in visual_point.h).
class VisualPoint;
// Forward declaration of Feature (defined in feature.h).
struct Feature;

// Type alias for a list of Feature pointers (observed features in a frame).
typedef list<Feature *> Features;
// Type alias for a vector of OpenCV images forming an image pyramid.
typedef vector<cv::Mat> ImgPyr;

// Stores a camera image together with extracted features and the estimated camera pose.
/// A frame saves the image, the associated features and the estimated pose.
class Frame : boost::noncopyable
{
// Public member variables and methods.
public:
  // Macro to enforce proper Eigen alignment for classes with Eigen member types.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Static counter for assigning unique frame IDs across all frames.
  static int frame_counter_; //!< Counts the number of created frames. Used to set the unique id.
  // Unique identifier for this frame instance.
  int id_;                   //!< Unique id of the frame.
  // Pointer to the camera model used for projection and unprojection.
  vk::AbstractCamera *cam_;  //!< Camera model.
  // Transform from world frame to this frame's coordinate system.
  SE3 T_f_w_;                //!< Transform (f)rame from (w)orld.
  // Transform from world to frame using the IMU propagated pose (prior).
  SE3 T_f_w_prior_;          //!< Transform (f)rame from (w)orld provided by the IMU prior.
  // Raw image data associated with this frame.
  cv::Mat img_;              //!< Image of the frame.
  // List of visual features (tracked patches) extracted from this frame.
  Features fts_;             //!< List of features in the image.

  // Constructor: creates a frame from a camera model and image.
  Frame(vk::AbstractCamera *cam, const cv::Mat &img);
  // Destructor: cleans up features and associated data.
  ~Frame();

  /// Initialize new frame and create image pyramid.
  void initFrame(const cv::Mat &img);

  /// Return number of point observations.
  inline size_t nObs() const { return fts_.size(); }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
  inline Vector2d w2c(const Vector3d &xyz_w) const { return cam_->world2cam(T_f_w_ * xyz_w); }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c) using the IMU prior pose.
  inline Vector2d w2c_prior(const Vector3d &xyz_w) const { return cam_->world2cam(T_f_w_prior_ * xyz_w); }
  
  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline Vector3d c2f(const Vector2d &px) const { return cam_->cam2world(px[0], px[1]); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

  /// Transforms point coordinates in world-frame (w) to camera-frams (f).
  inline Vector3d w2f(const Vector3d &xyz_w) const { return T_f_w_ * xyz_w; }

  /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
  inline Vector3d f2w(const Vector3d &f) const { return T_f_w_.inverse() * f; }

  /// Projects Point from unit sphere (f) in camera pixels (c).
  inline Vector2d f2c(const Vector3d &f) const { return cam_->world2cam(f); }

  /// Return the pose of the frame in the (w)orld coordinate frame.
  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }
// Closing brace for Frame class.
};

// Unique pointer alias for automatic Frame memory management.
typedef std::unique_ptr<Frame> FramePtr;

// Helper utilities for frame-level operations such as image pyramid construction.
/// Some helper functions for the frame object.
namespace frame_utils
{

// Builds a multi-scale image pyramid using half-sample downsampling.
/// Creates an image pyramid of half-sampled images.
void createImgPyramid(const cv::Mat &img_level_0, int n_levels, ImgPyr &pyr);

// Closing brace for frame_utils namespace.
} // namespace frame_utils

// End of header guard.
#endif // LIVO_FRAME_H_
