#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "I GOT SOMETHING" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_projector");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("image_points_couples", 1000, chatterCallback);
  ros::spin();

  return 0;
}
