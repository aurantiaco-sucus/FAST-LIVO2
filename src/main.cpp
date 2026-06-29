// Include the main SLAM orchestrator class header
#include "LIVMapper.h"

// Entry point: initializes ROS node, loads camera model, creates the LIVMapper
// orchestrator, and enters the main processing loop.
int main(int argc, char **argv)
{
  // Initialize the ROS node with the name "laserMapping"
  ros::init(argc, argv, "laserMapping");
  // Create a ROS node handle for parameter and communication interfaces
  ros::NodeHandle nh;
  // Create an image transport instance for subscribing/publishing image topics
  image_transport::ImageTransport it(nh);
  // Instantiate the main SLAM pipeline orchestrator
  LIVMapper mapper(nh); 
  // Set up all ROS subscribers, publishers, and image transport
  mapper.initializeSubscribersAndPublishers(nh, it);
  // Enter the main SLAM processing loop (blocking)
  mapper.run();
  // Exit successfully after the SLAM loop finishes
  return 0;
}