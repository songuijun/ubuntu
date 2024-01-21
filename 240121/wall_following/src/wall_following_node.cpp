#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_sonar = 0.0;
double front_right_sonar = 0.0;
double front_left_sonar = 0.0;
double error_old = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
    front_sonar = msg->range;
    ROS_INFO("Front_sonar: %.2lf", front_sonar);
}

void Front_Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
    front_right_sonar = msg->range;
    ROS_INFO("Front_Right_sonar: %.2lf", front_right_sonar);
}

void Front_Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
    front_left_sonar = msg->range;
    ROS_INFO("Front_Left_sonar: %.2lf", front_left_sonar);
}

void PID_Wall_Following(geometry_msgs::Twist& cmd_vel, double Kp, double Ki, double Kd)
{
    double error = front_left_sonar - front_right_sonar;
    double error_d = error - error_old;
    double error_sum = 0.0;
    error_sum += error;

    double steering_control = Kp * error + Ki * error_sum + Kd * error_d;

    if (front_sonar <= 1.0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        cmd_vel.linear.x = 0.65;
        cmd_vel.angular.z = steering_control;
    }

    error_old = error;
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "wall_following");
    ros::NodeHandle n;

    ros::Subscriber front_sonar_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);
    ros::Subscriber left_sonar_sub = n.subscribe("/range_front_left", 1000, Front_Left_Sonar_Callback);
    ros::Subscriber right_sonar_sub = n.subscribe("/range_front_right", 1000, Front_Right_Sonar_Callback);

    ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);

    double wall_Kp = 0.3;
    double wall_Ki = 0.0;
    double wall_Kd = 1.2;

    while (ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        PID_Wall_Following(cmd_vel, wall_Kp, wall_Ki, wall_Kd);
        sonar_cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
