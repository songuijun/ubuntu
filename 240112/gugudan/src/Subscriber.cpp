#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This callback function is called when a message is received on the 'gugudan' topic.
 */
void gugudanCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received gugudan: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gugudan_listener");
  ros::NodeHandle n;

  /**
   * The subscribe() function is used to tell ROS that you want to receive messages
   * on a given topic. This invokes a call to the ROS master node, which keeps a
   * registry of publishers and subscribers.
   *
   * The second parameter to subscribe() is the size of the message queue. If messages
   * are arriving faster than they are being processed, this is the number of messages
   * that will be buffered up before beginning to throw away the oldest ones.
   *
   * The third parameter is the callback function that will be called when a message
   * is received on the 'gugudan' topic.
   */
  ros::Subscriber sub = n.subscribe("gugudan", 1000, gugudanCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks. All callbacks will be called
   * from within this thread (the main one). ros::spin() will exit when Ctrl-C is
   * pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
