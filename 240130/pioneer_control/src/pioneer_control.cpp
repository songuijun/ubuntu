#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x
int mission_flag = 0;

#define TSL1401CL_SIZE 320
#define THRESHOLD 0.1
#define Line_Center 147
#define OFFSET 13

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];

double front = 0.0;
double left = 0.0;
double right = 0.0;

double error_wall_old = 0.0;
double error_lane_old = 0.0;

double roll, pitch, yaw;
double error_yaw_old = 0.0;


geometry_msgs::Twist cmd_vel;

void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
    for (int i = 0; i < tsl1401cl_size; i++)
    {
        if (tsl1401cl_data[i] > threshold)
        {
            ThresholdData[i] = 255;
        }
        else
        {
            ThresholdData[i] = 0;
        }
    }
}


double find_line_center()
{
    double centroid = 0.0;
    double sum = 0.0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        sum += LineSensor_threshold_Data[i];
        centroid += LineSensor_threshold_Data[i] * i;
    }

    if (sum == 0)
    {
        sum = 1;
    }

    centroid = centroid / sum;

    return centroid;
}

void tsl1401cl_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
        printf("%d", LineSensor_threshold_Data[i]);
    }
    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
    
    double centroid = find_line_center();
}

geometry_msgs::Twist lane_control(double Kp_lane, double Ki_lane, double Kd_lane )
{
    geometry_msgs::Twist cmd_vel;

    double lineCenter = find_line_center();

    double error_lane = Line_Center - lineCenter + OFFSET;
    double error_lane_d = error_lane - error_lane_old;
    double error_lane_s = 0.0;

    error_lane_s += error_lane;

    double steering_angle = Kp_lane * error_lane + Ki_lane * error_lane_s + Kd_lane * error_lane_d;

    cmd_vel.linear.x = 0.55;
    cmd_vel.angular.z = steering_angle;

    error_lane_old = error_lane;

    return cmd_vel;
}

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    front = msg->range;
    //printf("Front: %.2f\n", front);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    right = msg->range;
    //printf("Right: %.2f\n\n", right);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    left = msg->range;
    //printf("Left:  %.2f\n", left);
}


double yaw_degree(double yaw_deg)
{
    if (yaw_deg > 360)
    {
        yaw_deg = yaw_deg - 360;
    }
    else if (yaw_deg < 0)
    {
        yaw_deg = yaw_deg + 360;
    }
    return yaw_deg;
}

geometry_msgs::Twist yaw_control(double Kp_yaw, double Ki_yaw, double Kd_yaw, double target_heading_yaw)
{
    geometry_msgs::Twist cmd_vel;

    double yaw_deg = RAD2DEG(yaw);
    yaw_deg = yaw_degree(yaw_deg);

    double error_yaw = target_heading_yaw - yaw_deg;

    if (error_yaw > 180)
    {
        error_yaw = error_yaw - 360;
    }
    else if (error_yaw < -180)
    {
        error_yaw = error_yaw + 360;
    }

    double error_yaw_s = 0.0;
    double error_yaw_d = error_yaw - error_yaw_old;

    error_yaw_s += error_yaw;

    double Steering_Angle = Kp_yaw * error_yaw + Ki_yaw * error_yaw_s + Kd_yaw * error_yaw_d;

    cmd_vel.linear.x = 1.0;
    cmd_vel.angular.z = Steering_Angle;

    if (fabs(error_yaw) < 1.0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        mission_flag++;
    }

    error_yaw_old = error_yaw;

    return cmd_vel;
}

void print_lane()
{
    //printf("Threshold Data: \n");


}

geometry_msgs::Twist wall_following(double Kp_wall, double Ki_wall, double Kd_wall)
{
    geometry_msgs::Twist cmd_vel;

    double error_wall = left - right;
    double error_wall_d = error_wall - error_wall_old;
    double error_wall_s = 0.0;
    error_wall_s += error_wall;

    double steering_control = Kp_wall * error_wall + Ki_wall * error_wall_s + Kd_wall * error_wall_d;


    cmd_vel.linear.x = 0.3;
    cmd_vel.angular.z = steering_control;


    error_wall_old = error_wall;

    return cmd_vel;
}



void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    double yaw_deg = yaw_degree(RAD2DEG(yaw));

    //printf("%.2f\n", yaw_deg);
}

int main(int argc, char **argv)
{
    int count = 0;
    

    ros::init(argc, argv, "pioneer_control");
    ros::NodeHandle nh;
    
    ros::Subscriber front_sonar_sub = nh.subscribe("/range_front", 1000, Front_Sonar_Callback);
    ros::Subscriber left_sonar_sub = nh.subscribe("/range_front_left", 1000, Left_Sonar_Callback);
    ros::Subscriber right_sonar_sub = nh.subscribe("/range_front_right", 1000, Right_Sonar_Callback);
	ros::Subscriber tsl1401cl_sub = nh.subscribe("/tsl1401cl", 10, tsl1401cl_Callback);
    ros::Subscriber yaw_control_sub = nh.subscribe("/imu", 1000, imu1Callback);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);

    geometry_msgs::Twist cmd_vel;

    double Kp_lane1 = 0.0015;
    double Ki_lane1 = 0.0;
    double Kd_lane1 = 0.005;

	double Kp_lane2 = 0.0021;
    double Ki_lane2 = 0.0;
    double Kd_lane2 = 0.003;

    double Kp_wall = 0.42;
    double Ki_wall = 0.0;
    double Kd_wall = 0.3;

 	double Kp_yaw1 = 0.02;
	double Ki_yaw1 = 0.0;
	double Kd_yaw1 = 0.2;
    
    double Kp_yaw2 = 0.021;
	double Ki_yaw2 = 0.0;
	double Kd_yaw2 = 0.2;
    
	double target_yaw_degree = 0.0;
    
    while (ros::ok())
    {
        tsl1401cl_Callback;
        find_line_center();

        switch (mission_flag)
        {
        case 0:
            if (find_line_center() == 0.0)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                mission_flag++;
            }
            break;

        case 1:
            if (find_line_center() != 0.0)
            {
                cmd_vel = lane_control(Kp_lane1, Ki_lane1, Kd_lane1);
            }
            if (find_line_center() == 0.0)
            {
                mission_flag++;
            }
            break;

        case 2:
            if (front > 1.0)
            {
                cmd_vel.linear.x = 0.2;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
				
				target_yaw_degree = 265.0;
                mission_flag++;
            }
            break;

        case 3:
            cmd_vel = yaw_control(Kp_yaw1, Ki_yaw1, Kd_yaw1, target_yaw_degree); 
            break;

        case 4:
            cmd_vel = wall_following(Kp_wall, Ki_wall, Kd_wall);
            if (front < 0.86)
            {
                target_yaw_degree = 185.0;
                mission_flag++;
            }
            break;
        case 5:
            cmd_vel = yaw_control(Kp_yaw2, Ki_yaw2, Kd_yaw2, target_yaw_degree); 
            break;

       case 6:
       
       for (int i = 0; i < TSL1401CL_SIZE; i++)
		{
            if(LineSensor_threshold_Data[i] != 255)
            {
				cmd_vel = lane_control(Kp_lane2, Ki_lane2, Kd_lane2);
			}
               
          else
           {
			   	cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
           }
           
		}
    	break; 
		
	}
        cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
