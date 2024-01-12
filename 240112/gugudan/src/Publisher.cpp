#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gugudan_publisher");
  ros::NodeHandle n;
  ros::Publisher gugudan_pub = n.advertise<std_msgs::Int32>("gugudan", 1000);
  ros::Rate loop_rate(1);

  std_msgs::Int32 msg;

  for (int i = 1; i <= 9; i++)
  {
    for (int j = 1; j <= 9; j++)
    {
      msg.data = i * j;
      gugudan_pub.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
