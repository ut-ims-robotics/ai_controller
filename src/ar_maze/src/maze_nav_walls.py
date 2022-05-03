#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from robotont_laserscan_to_distance.msg import LaserScanSplit

TIME_IF_CENTER = 1.0
TIME_IF_SIDE = 0.5
DETECTING_SPEED_ROTATION = 0.5
marker = False
twist_msg = Twist()
side_rotation = [1, -1]
calc_time_dur = 0
calc_time_dur_obstacle = 0
moving = True
rotation = False
obstacle = False
obstacle_center = False
obstacle_left = False
obstacle_right = False

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def callback(data):
    global start_time, marker, calc_time_dur, calc_time_dur_obstacle, moving, rotation, moving_start_time, rotation_start_time, obstacle, obstacle_right, obstacle_center, obstacle_left
    if not marker:
        if data.center_min < 0.4 or data.left_min < 0.3 or data.right_min < 0.3 or obstacle:
            moving = True
            calc_time_dur = 0
            rotation = False
            if data.center_min < 0.4 or obstacle_center and not obstacle_left and not obstacle_right:
                if calc_time_dur_obstacle == 0:
                    start_time = rospy.get_time()
                    obstacle_center = True
                    obstacle = True
                if (rospy.get_time() - start_time) < TIME_IF_CENTER:
                    calc_time_dur_obstacle = 1
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = side_rotation[0] * DETECTING_SPEED_ROTATION
                    rospy.loginfo(twist_msg)
                    cmd_vel_pub.publish(twist_msg)
                    print(rospy.get_time() - start_time, "Rotation because of center obstacle")
                elif (rospy.get_time() - start_time) >= TIME_IF_CENTER:
                    calc_time_dur_obstacle = 0
                    obstacle_center = False
                    obstacle = False
            elif data.left_min < 0.3 or obstacle_left and not obstacle_right:
                if calc_time_dur_obstacle == 0:
                    start_time = rospy.get_time()
                    obstacle_left = True
                    obstacle = True
                if (rospy.get_time() - start_time) < TIME_IF_SIDE:
                    calc_time_dur_obstacle = 1
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = side_rotation[1] * DETECTING_SPEED_ROTATION
                    rospy.loginfo(twist_msg)
                    cmd_vel_pub.publish(twist_msg)
                    print(rospy.get_time() - start_time, "Rotation because of left obstacle")
                elif (rospy.get_time() - start_time) >= TIME_IF_SIDE:
                    calc_time_dur_obstacle = 0
                    obstacle_left = False
                    obstacle = False
            elif data.right_min < 0.3 or obstacle_right:
                if calc_time_dur_obstacle == 0:
                    start_time = rospy.get_time()
                    obstacle_right = True
                    obstacle = True
                if (rospy.get_time() - start_time) < TIME_IF_SIDE:
                    calc_time_dur_obstacle = 1
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = side_rotation[0] * DETECTING_SPEED_ROTATION
                    rospy.loginfo(twist_msg)
                    cmd_vel_pub.publish(twist_msg)
                    print(rospy.get_time() - start_time, "Rotation because of right obstacle")
                elif (rospy.get_time() - start_time) >= TIME_IF_SIDE:
                    calc_time_dur_obstacle = 0
                    obstacle_right = False
                    obstacle = False
        else:
            if moving:
                if calc_time_dur == 0: 
                    moving_start_time = rospy.get_time()
                if (rospy.get_time() - moving_start_time) < 3.0:
                    calc_time_dur = 1
                    twist_msg.linear.x = 1.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = 0.0
                    rospy.loginfo(twist_msg)
                    cmd_vel_pub.publish(twist_msg)
                    print("Center: ", data.center_min)
                    print("Left: ", data.left_min)
                    print("Right: ", data.right_min)
                    print("Moving to find tag")
                elif (rospy.get_time() - moving_start_time) >= 3.0:
                    calc_time_dur = 0
                    moving = False
                    rotation = True
            elif rotation:
                if calc_time_dur == 0:
                    rotation_start_time = rospy.get_time()
                if (rospy.get_time() - rotation_start_time) < 6.4//DETECTING_SPEED_ROTATION:
                    calc_time_dur = 1
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = DETECTING_SPEED_ROTATION
                    rospy.loginfo(twist_msg)
                    cmd_vel_pub.publish(twist_msg)
                    print("Rotation to find tag")
                elif (rospy.get_time() - rotation_start_time) >= 6.4//DETECTING_SPEED_ROTATION:
                    calc_time_dur = 0
                    moving = True
                    rotation = False

def ar_listener():

    rospy.init_node('maze_nav_walls', anonymous=True)
    rospy.Subscriber("scan_to_distance", LaserScanSplit, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        ar_listener()
    except rospy.ROSInterruptException:
        pass