#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gugudan_publisher");
  ros::NodeHandle n;
  ros::Publisher gugudan_pub = n.advertise<std_msgs::String>("gugudan", 1000);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    for (int i = 1; i <= 9; i++)
    {
      for (int j = 1; j <= 9; j++)
      {
		ss << "\n";
        ss << i << " * " << j << " = " << (i * j) << "\n";
      }
      ss << "\n";
    }
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    gugudan_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
