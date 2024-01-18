#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define TSL1401CL_SIZE 320

#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];

float kp = 0.2;
float ki = 0.0;
float kd = 0.1;

float error   = 0;
float error_d = 0;
float error_old = 0;

double Steering_angle = 0;

float Line_Center = 144;
float OFFSET = 0;

void threshold(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    double THRESHOLD = 0.01;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (tsl1401cl_data[i] > THRESHOLD)
        {
            LineSensor_threshold_Data[i] = 255;
        }
        else
        {
            LineSensor_threshold_Data[i] = 0;
        }
    }
}

int find_line_center()
{
    int line_center_result = 0;
    int lineMass = 0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        lineMass += LineSensor_threshold_Data[i];
        line_center_result += LineSensor_threshold_Data[i] * i;
    }

    line_center_result = (lineMass != 0) ? line_center_result / lineMass : 0;
    if (lineMass == 0)
    {
        return line_center_result;
    }
    return line_center_result;
}

void lane_control(geometry_msgs::Twist &cmd_vel, double line_center)
{
    if (line_center == 0.0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        error = Line_Center - line_center + OFFSET;
        error_d = error - error_old;
        Steering_angle = kp * error + kd * error_d + ki * 0;

        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = Steering_angle / 130;

        error_old = error;
    }
}

int main(int argc, char **argv)
{
    int count = 0;

    geometry_msgs::Twist cmd_vel;
    ros::init(argc, argv, "line_control");
    ros::NodeHandle nh;

    ros::Subscriber tsl1401cl_sub = nh.subscribe("/tsl1401cl", 10, threshold);
    ros::Publisher tst1401cl_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);
    while (ros::ok())
    {
        printf("Threshold Data: \n");

        for (int i = 0; i < TSL1401CL_SIZE; i++)
        {
            printf("%d ", LineSensor_threshold_Data[i]);
        }
        printf("\n");

        double line_center = find_line_center();
        printf("Line Center: %f\n", line_center);

        lane_control(cmd_vel, line_center);

        tst1401cl_cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
