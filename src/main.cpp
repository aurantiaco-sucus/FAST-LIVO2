#include "LIVMapper.h"

// Entry point: initializes ROS node, loads camera model, creates the LIVMapper
// orchestrator, and enters the main processing loop.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh); 
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}