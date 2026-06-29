/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Include Boost.Bind for function object binding and callback utilities
#include <boost/bind.hpp>
// Include feature descriptor types and utilities
#include "feature.h"
// Include Frame class declaration and interface
#include "frame.h"
// Include visual map point class for 3D point management
#include "visual_point.h"
// Include standard runtime exception handling
#include <stdexcept>
// Include vikit mathematical utility functions
#include <vikit/math_utils.h>
// Include vikit performance monitoring utilities
#include <vikit/performance_monitor.h>
// Include vikit vision processing functions (e.g., halfSample)
#include <vikit/vision.h>

// Initialize the static frame counter to zero; each new frame increments it
int Frame::frame_counter_ = 0;

// Constructor: assigns a unique frame ID, stores the camera model, and
// initializes the frame from the provided image.
Frame::Frame(vk::AbstractCamera *cam, const cv::Mat &img)
    // Assign a unique incrementing frame ID from the static counter
    : id_(frame_counter_++), 
      // Store the camera model pointer for projection and geometry operations
      cam_(cam)
{
  // Validate the input image and store it as the frame's image data
  initFrame(img);
}

// Destructor: deletes all features associated with this frame.
Frame::~Frame()
{
  // Iterate over all features and deallocate each one to prevent memory leaks
  std::for_each(fts_.begin(), fts_.end(), [&](Feature *i) { delete i; });
}

// Validates that the image matches the camera model dimensions and is grayscale,
// then stores it as the frame image.
void Frame::initFrame(const cv::Mat &img)
{
  // Reject empty images with a runtime error
  if (img.empty()) { throw std::runtime_error("Frame: provided image is empty"); }

  // Validate that the image dimensions match the camera model's expected resolution
  if (img.cols != cam_->width() || img.rows != cam_->height())
  {
    // Throw an error if the image size does not match the camera model
    throw std::runtime_error("Frame: provided image has not the same size as the camera model");
  }

  // Ensure the image is single-channel grayscale (CV_8UC1)
  if (img.type() != CV_8UC1) { throw std::runtime_error("Frame: provided image is not grayscale"); }

  // Store the validated image in the frame member variable
  img_ = img;
}

// Helper utilities for frame-level operations such as image pyramid construction.
/// Utility functions for the Frame class
namespace frame_utils
{

// Builds a multi-scale image pyramid using half-sample downsampling.
void createImgPyramid(const cv::Mat &img_level_0, int n_levels, ImgPyr &pyr)
{
  // Allocate storage for the specified number of pyramid levels
  pyr.resize(n_levels);
  // Set the base level (level 0) to the original full-resolution image
  pyr[0] = img_level_0;
  // Build each subsequent level by halving the previous level's dimensions
  for (int i = 1; i < n_levels; ++i)
  {
    // Allocate a half-resolution image buffer for the current pyramid level
    pyr[i] = cv::Mat(pyr[i - 1].rows / 2, pyr[i - 1].cols / 2, CV_8U);
    // Downsample the previous level by a factor of 2 using half-sample interpolation
    vk::halfSample(pyr[i - 1], pyr[i]);
  }
}

} // namespace frame_utils
