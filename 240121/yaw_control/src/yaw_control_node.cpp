#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

int count = 0;
double roll, pitch, yaw;
double error_old = 0.0;
double target_heading_yaw = 89.9;

double Kp = 0.0015;
double Ki = 0.0;
double Kd = 0.3;

double yaw_deg;
double error = target_heading_yaw - yaw_deg;
double error_d = error - error_old;
double error_sum = error;

double Steering_Angle = Kp * error + Ki * error_sum + Kd * error_d;


double Constrain_Yaw(double yaw_deg)
{
    if (yaw_deg > 360
    {
        yaw_deg = yaw_deg - 360;
    }
    else if (yaw_deg < 0)
    {
        yaw_deg = yaw_deg + 360;
    }
    return yaw_deg;
}

void target_yaw_control(geometry_msgs::Twist &cmd_vel)
{
    double yaw_deg = RAD2DEG(yaw);

    yaw_deg = Constrain_Yaw(yaw_deg);

    if (fabs(error) < 0.5)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        cmd_vel.linear.x = 1.0;
        cmd_vel.angular.z = Steering_Angle;
    }
    error_old = error; 
}

void imu1Callback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);      
        m.getRPY(roll, pitch, yaw);

    double yaw_deg = Constrain_Yaw(RAD2DEG(yaw));

    printf("%f\n", yaw_deg);
}

int main(int argc, char **argv)
{	
    geometry_msgs::Twist cmd_vel;

    ros::init(argc, argv, "yaw_control");
    ros::NodeHandle n;
    ros::Subscriber yaw_control_sub = n.subscribe("/imu", 1000, imu1Callback); 
    ros::Publisher yaw_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);
    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
        target_yaw_control(cmd_vel);
        yaw_cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
