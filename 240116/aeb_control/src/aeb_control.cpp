#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
//sonar sensor를 위해 float 32 값을 받는 헤더 선언

double front_sonar = 0.0; // 전방 초음파 센서의 거리 값을 저장하는 전역 변수

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)// 전방 초음파 센서의 데이터를 받아오는 콜백 함수
{
    // 수신된 초음파 센서의 값을 front_sonar 변수에 저장하고 출력
    ROS_INFO("Sonar Range: [%f]", front_sonar);
    front_sonar = msg->range;
}

int main(int argc, char **argv)
{
    // 루프 반복 횟수를 저장하는 변수
    int count = 0;

    // 로봇의 이동 명령을 저장하는 Twist 메시지
    geometry_msgs::Twist cmd_vel;

    // ROS 노드 초기화
    ros::init(argc, argv, "aeb_control_sonar");
    ros::NodeHandle n;

    // 전방 초음파 센서의 데이터를 받아오기 위한 서브스크라이버 설정
    ros::Subscriber front_sonar_range_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);

    // 로봇의 이동 명령을 발행하기 위한 퍼블리셔 설정
    ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    // 루프 주기 설정
    ros::Rate loop_rate(30.0);

    // ROS가 실행 중일 때까지 루프 실행
    while (ros::ok())
    {
        // 전방 초음파 센서의 거리에 따라 이동 명령 설정
        if (front_sonar < 1.0)
        {
            cmd_vel.linear.x = 0.0; // sonar의 값이 1보다 작으면 정지
        }
        else
        {
            cmd_vel.linear.x = 0.2; // sonar의 값이 1보다 클 때, 전진
        }

        // 이동 명령을 발행
        sonar_cmd_vel_pub.publish(cmd_vel);

        // 콜백 함수 실행 및 루프 주기 동안 대기
        ros::spinOnce();
        loop_rate.sleep();

        // 루프 반복 횟수 증가
        ++count;
    }

    // 프로그램 종료
    return 0;
}
