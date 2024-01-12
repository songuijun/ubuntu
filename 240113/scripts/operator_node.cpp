#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

float x_value;
float y_value;
std::string operator_value;

void xCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_value = msg->data;
}

void operatorCallback(const std_msgs::String::ConstPtr& msg)
{
    operator_value = msg->data;
}

void yCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_value = msg->data;

    float result;
    if (operator_value == "+") {
        result = x_value + y_value;
        printf("%.2f + %.2f = %.2f\n", x_value, y_value, result);
    } else if (operator_value == "-") {
        result = x_value - y_value;
        printf("%.2f - %.2f = %.2f\n", x_value, y_value, result);
    } else if (operator_value == "*") {
        result = x_value * y_value;
        printf("%.2f * %.2f = %.2f\n", x_value, y_value, result);
    } else if (operator_value == "/") {
        if (y_value != 0) {
            result = x_value / y_value;
            printf("%.2f / %.2f = %.2f\n", x_value, y_value, result);
        } else {
            printf("Error: Division by zero\n");
        }
    } else {
        printf("Error: Invalid operator\n");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber x_sub = nh.subscribe("/float32/x", 1, xCallback);
    ros::Subscriber operator_sub = nh.subscribe("/string/operator", 1, operatorCallback);
    ros::Subscriber y_sub = nh.subscribe("/float32/y", 1, yCallback);

    ros::spin();

    return 0;
}
