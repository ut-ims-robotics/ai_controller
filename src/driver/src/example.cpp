#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
geometry_msgs::Twist robot_vel;
double forward_moving [6] = {0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
double backward_moving [6] = {-0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
double left_moving [6] = {0.0, 0.2, 0.0, 0.0, 0.0, 0.0};
double right_moving [6] = {0.0, -0.2, 0.0, 0.0, 0.0, 0.0};
double* list_moving [4] = {forward_moving, right_moving, backward_moving, left_moving};
double angle [2] = {M_PI_4, -M_PI_4};
void square(ros::Publisher trajectory_pub) {
    for (int i = 0; i <= 3; i++) {
        ros::Time start_time = ros::Time::now();
        ros::Duration three_seconds(3.0);
        while(ros::Time::now() - start_time < three_seconds) {
            robot_vel.linear.x = list_moving[i][0];
            robot_vel.linear.y = list_moving[i][1];
            robot_vel.linear.z = list_moving[i][2];
            robot_vel.angular.x = list_moving[i][3];
            robot_vel.angular.y = list_moving[i][4];
            robot_vel.angular.z = list_moving[i][5];
            trajectory_pub.publish(robot_vel);
        }
    }
}
void n_h_square(ros::Publisher trajectory_pub) {
    for (int i = 0; i <= 3; i++) {
        ros::Time start_time = ros::Time::now();
        ros::Duration two_seconds(2.0);
        ros::Duration three_seconds(3.0);
        while(ros::Time::now() - start_time < three_seconds) {
            robot_vel.linear.x = 0.2;
            robot_vel.angular.z = 0;
            trajectory_pub.publish(robot_vel);
        }
        ros::Duration(1).sleep();
        start_time = ros::Time::now();
        while(ros::Time::now() - start_time < two_seconds) {
            robot_vel.linear.x = 0;
            robot_vel.angular.z = 0.623;
            trajectory_pub.publish(robot_vel);
        }
        ros::Duration(1).sleep();
    }
}
void circle(ros::Publisher trajectory_pub) {
    ros::Time start_time = ros::Time::now();
    ros::Duration seven_seconds(7.0);
    while(ros::Time::now() - start_time < seven_seconds) {
        robot_vel.linear.x = 0.2;
        robot_vel.angular.z = M_PI_4;
        trajectory_pub.publish(robot_vel);
    }
}
void eight_circle(ros::Publisher trajectory_pub) {
    for (int i = 0; i <= 1; i++) {
        ros::Time start_time = ros::Time::now();
        ros::Duration seven_seconds(7.5);
        while(ros::Time::now() - start_time < seven_seconds) {
            robot_vel.linear.x = 0.2;
            robot_vel.angular.z = angle[i];
            trajectory_pub.publish(robot_vel);
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_commander");
    ros::NodeHandle n;
    ros::Publisher trajectory_pub = 
        n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        
        n_h_square(trajectory_pub);

        trajectory_pub.publish(robot_vel);
        loop_rate.sleep();
    }

    return 0;
}