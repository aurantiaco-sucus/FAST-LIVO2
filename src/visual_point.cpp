/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// Include VisualPoint class declaration
#include "visual_point.h"
// Include Feature class for managing visual feature observations
#include "feature.h"
// Include standard runtime exception handling
#include <stdexcept>
// Include vikit mathematical utility functions
#include <vikit/math_utils.h>

// Constructor: initializes a 3D visual map point at the given world position
// with default normal and convergence flags.
VisualPoint::VisualPoint(const Vector3d &pos)
    // Initialize the 3D position with the given world coordinates and reset normals to zero
    : pos_(pos), previous_normal_(Vector3d::Zero()), normal_(Vector3d::Zero()),
      // Set convergence, normal initialization, and reference patch flags to their default false state
      is_converged_(false), is_normal_initialized_(false), has_ref_patch_(false)
{
}

// Destructor: deletes all feature observations and clears the observation list.
VisualPoint::~VisualPoint() 
{
  // Iterate through all feature observations stored in the deque and deallocate each one
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    // Deallocate the feature observation object to free memory
    delete(*it);
  }
  // Clear the observation list after deleting all entries
  obs_.clear();
  // Nullify the reference patch pointer since all observations are gone
  ref_patch = nullptr;
}

// Adds a feature observation to the front of the observation list.
void VisualPoint::addFrameRef(Feature *ftr)
{
  // Insert the new feature observation at the front of the deque for efficient access
  obs_.push_front(ftr);
}

// Removes a specific feature observation from the list and deletes it. Clears
// the reference patch if it matches the removed feature.
void VisualPoint::deleteFeatureRef(Feature *ftr)
{
  // Check if the feature to delete is the current reference patch
  if (ref_patch == ftr)
  {
    // Clear the reference patch pointer since its feature is being removed
    ref_patch = nullptr;
    // Mark that this point no longer has a valid reference patch
    has_ref_patch_ = false;
  }
  // Search through all observations in the deque for the matching feature pointer
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    // Check if this observation entry matches the feature to delete
    if ((*it) == ftr)
    {
      // Deallocate the matching feature observation object
      delete((*it));
      // Remove the observation entry from the deque
      obs_.erase(it);
      // Exit immediately to avoid using invalidated iterators
      return;
    }
  }
}

// Finds the observation whose viewing direction is closest to the current frame
// position, returning false if the best match has a cosine angle below 0.5.
bool VisualPoint::getCloseViewObs(const Vector3d &framepos, Feature *&ftr, const Vector2d &cur_px) const
{
  // TODO: get frame with same point of view AND same pyramid level!
  // Return false if there are no observations to search through
  if (obs_.size() <= 0) return false;

  // Compute the direction vector from the 3D point to the query frame's position
  Vector3d obs_dir(framepos - pos_);
  // Normalize the observation direction to unit length for angle comparison
  obs_dir.normalize();
  // Initialize the iterator to the first observation for best-match tracking
  auto min_it = obs_.begin();
  // Initialize the minimum cosine angle to zero (dot product lower bound)
  double min_cos_angle = 0;
  // Iterate over all observations to find the one with the closest viewing angle
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    // Compute the direction from the 3D point to the frame where this feature was observed
    Vector3d dir((*it)->T_f_w_.inverse().translation() - pos_);
    // Normalize the feature's observation direction
    dir.normalize();
    // Compute the cosine of the angle between the query direction and this observation's direction
    double cos_angle = obs_dir.dot(dir);
    // Update the best match if this observation has a smaller angular difference (larger cosine)
    if (cos_angle > min_cos_angle)
    {
      // Record the higher cosine value representing a smaller viewing angle
      min_cos_angle = cos_angle;
      // Update the iterator to point to the best matching observation so far
      min_it = it;
    }
  }
  // Set the output feature pointer to the best matching observation
  ftr = *min_it;

  // Reject the match if the viewing angle exceeds 60 degrees (cosine less than 0.5)
  if (min_cos_angle < 0.5) // assume that observations larger than 60° are useless 0.5
  {
    // Signal that no suitable close-view observation was found
    return false;
  }

  // Signal that a suitable close-view observation was found successfully
  return true;
}

// Finds the feature observation with the lowest NCC+angle score, used when
// the observation list exceeds the size limit.
void VisualPoint::findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const
{
  // Initialize the iterator to the first observation for minimum-score tracking
  auto min_it = obs_.begin();
  // Initialize the minimum score to the maximum possible float value for comparison
  float min_score = std::numeric_limits<float>::max();

  // Iterate over all observations to find the one with the lowest NCC+angle score
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    // Check if this observation has a lower score than the current minimum
    if ((*it)->score_ < min_score)
    {
      // Update the recorded minimum score
      min_score = (*it)->score_;
      // Update the iterator to point to the lowest-scoring observation so far
      min_it = it;
    }
  }
  // Set the output feature pointer to the lowest-scoring observation
  ftr = *min_it;
}

// Deletes all feature observations that are not the reference patch, keeping
// only the reference patch for converged points.
void VisualPoint::deleteNonRefPatchFeatures()
{
  // Iterate through observations using manual iterator control to safely erase non-reference features
  for (auto it = obs_.begin(); it != obs_.end();)
  {
    // Check if this feature is NOT the reference patch and should be removed
    if (*it != ref_patch)
    {
      // Deallocate the non-reference feature observation
      delete *it;
      // Erase the entry from the deque and obtain the next valid iterator
      it = obs_.erase(it);
    }
    else
    {
      // Keep the reference patch feature and advance to the next observation
      ++it;
    }
  }
}