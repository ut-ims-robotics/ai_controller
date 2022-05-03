#!/usr/bin/python3

import rospy
import tf
import numpy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from robotont_laserscan_to_distance.msg import LaserScanSplit
from math import atan2, sqrt

MAX_LIN_VEL = 0.05
MAX_ANG_VEL = 0.6

GOAL_TOLERANCE = 0.05
ANGLE_TOLERANCE = 0.1
GOAL_DIST_FROM_MARKER = 0.5
DETECTING_SPEED_ROTATION = 0.5

last_heartbeat = 0
k=0
param = 0
twist_msg = Twist()
finish = False
distance_achieved = False

DETECTING_SPEED_ROTATION = 0.5
side_rotation = [1, -1]
calc_time_dur = 0
moving = True
rotation = False
ar_present = True

class Server:
    def __init__(self):
        self.tag_data = None
        self.laser_data = None

    def tag_callback(self, msg):
        # "Store" message received.
        self.tag_data = msg
        print(self.tag_data)

    def laser_callback(self, msg):
        # "Store" the message received.
        self.laser_data = msg



# def callback(data):
#         global last_heartbeat, marker_ids, k, param, twist_msg, cmd_vel_pub, finish, distance_achieved, ar_present
#         #print(len(data.markers))
#         #if not ar_present:
#             #laser()
#         if len(data.markers)==0 and k<len(marker_ids):
#             #last_heartbeat = rospy.get_time()
#             ar_present = True
#             rotation()
#         elif len(data.markers)!=0 and k<len(marker_ids):
#             ar_present = True
#             param = 0
#             print("Markers are not empty")
#             for marker in data.markers:
#                 # Check the marker id
#                 if marker.id != int(marker_ids[k]):
#                         #last_heartbeat = rospy.get_time()
#                         rotation()
#                         continue
#                 rospy.loginfo(rospy.get_caller_id() + " I heard %s", marker)
#                 marker_pos = (
#                         marker.pose.pose.position.x,
#                         marker.pose.pose.position.y,
#                         marker.pose.pose.position.z,
#                         0)
#                 marker_ori = (
#                         marker.pose.pose.orientation.x,
#                         marker.pose.pose.orientation.y,
#                         marker.pose.pose.orientation.z,
#                         marker.pose.pose.orientation.w)

#                 # Start with a unit vector pointing forward
#                 goal_dir = numpy.array([0, 0, 1, 0])

#                 # Rotation matrix that represents to marker orientation
#                 rot_mat = tf.transformations.quaternion_matrix(marker_ori)

#                 # Rotate the unit vec so it points to the direction of the marker
#                 goal_dir = rot_mat.dot(goal_dir)
#                 #print("Goal direction: ", goal_dir)
#                 goal_pos = marker_pos + goal_dir * GOAL_DIST_FROM_MARKER
#                 # Find planar distance to the goal
#                 dist_to_goal = numpy.linalg.norm(goal_pos[0:2])
#                 dist_to_goal_x = numpy.linalg.norm(goal_pos[0])
#                 dist_to_marker = numpy.linalg.norm(marker_pos[0:2])
#                 #twist_msg = Twist()
#                 angle = atan2(marker_pos[1],marker_pos[0])
#                 rospy.logdebug("ANGLE: %s", angle)
#                 rospy.logdebug("DISTTOMARKER: %s", dist_to_marker)
#                 rospy.logdebug("GoalDir: %s", goal_dir)
#                 rospy.logdebug("DISTTOGOAL: %s", dist_to_goal)
#                 rospy.logdebug("DISTTOGOALX: %s", dist_to_goal_x)
#                 rospy.logdebug("\nPOS: %s", marker_pos)
#                 rospy.logdebug("\nGOAL: %s", goal_pos)

#                 if dist_to_goal_x > GOAL_TOLERANCE and not distance_achieved:
#                         print("Calibrating distance")
#                         print(dist_to_goal_x)
#                         twist_msg.linear.x = min(max(goal_pos[0],-MAX_LIN_VEL),MAX_LIN_VEL)
#                         twist_msg.linear.y = 0
#                         twist_msg.angular.z = 4*min(max(angle,-MAX_ANG_VEL),MAX_ANG_VEL)
#                         print("Updating cmd vel %s", twist_msg)
#                         cmd_vel_pub.publish(twist_msg)
#                         if dist_to_goal_x <= GOAL_TOLERANCE:
#                                 distance_achieved = True
#                 elif abs(angle) > ANGLE_TOLERANCE or dist_to_goal > GOAL_TOLERANCE:
#                         print("Calibrating angle")
#                         twist_msg.linear.x = min(max(goal_pos[0],-MAX_LIN_VEL),MAX_LIN_VEL)
#                         twist_msg.linear.y = min(max(goal_pos[1],-MAX_LIN_VEL),MAX_LIN_VEL)
#                         twist_msg.angular.z = 4*min(max(angle,-MAX_ANG_VEL),MAX_ANG_VEL)
#                         print("Updating cmd vel %s", twist_msg)
#                         cmd_vel_pub.publish(twist_msg)
#                 else:
#                         k=k+1
#                         distance_achieved = False
#         elif k>=len(marker_ids):
#             ar_present = True
#             finish = True
#             #print(finish)
# def rotation():
#         global last_heartbeat, param, ar_present
#         #print(param)
#         if param==0:
#                 param = 1
#                 last_heartbeat = rospy.get_time()
#         print("Next marker:", marker_ids[k])
#         print(last_heartbeat)
#         print(6.4//DETECTING_SPEED_ROTATION)
#         print(rospy.get_time() - last_heartbeat)
#         if (rospy.get_time() - last_heartbeat) >= 6.4//DETECTING_SPEED_ROTATION:
#                 #print("Further actions...in process")
#                 ar_present = False
#         else:
#                 twist_msg.linear.x = 0.0
#                 twist_msg.linear.y = 0.0
#                 twist_msg.angular.z = DETECTING_SPEED_ROTATION
#                 rospy.loginfo(twist_msg)
#                 #last_heartbeat = rospy.get_time()
#                 cmd_vel_pub.publish(twist_msg)
#                 print("Rotating")

# def laser(data_scan):
#     global ar_present, calc_time_dur, moving, rotation, moving_start_time, rotation_start_time
#     if not ar_present:
#         print("I am inside callback_laser() in marker=False")
#         if data_scan.center_min < 0.4 or data_scan.left_min < 0.3 or data_scan.right_min < 0.3:
#             moving = True
#             calc_time_dur = 0
#             rotation = False
#             if data_scan.center_min < 0.4:
#                 twist_msg.linear.x = 0.0
#                 twist_msg.linear.y = 0.0
#                 twist_msg.angular.z = side_rotation[0] * DETECTING_SPEED_ROTATION
#                 rospy.loginfo(twist_msg)
#                 cmd_vel_pub.publish(twist_msg)
#                 print("Rotation because of center obstacle")
#             elif data_scan.left_min < 0.3:
#                 twist_msg.linear.x = 0.0
#                 twist_msg.linear.y = 0.0
#                 twist_msg.angular.z = side_rotation[1] * DETECTING_SPEED_ROTATION
#                 rospy.loginfo(twist_msg)
#                 cmd_vel_pub.publish(twist_msg)
#                 print("Rotation because of left obstacle")
#             elif data_scan.right_min < 0.3:
#                 twist_msg.linear.x = 0.0
#                 twist_msg.linear.y = 0.0
#                 twist_msg.angular.z = side_rotation[0] * DETECTING_SPEED_ROTATION
#                 rospy.loginfo(twist_msg)
#                 cmd_vel_pub.publish(twist_msg)
#                 print("Rotation because of right obstacle")
#         else:
#             if moving:
#                 if calc_time_dur == 0: 
#                     moving_start_time = rospy.get_time()
#                 if (rospy.get_time() - moving_start_time) < 3.0:
#                     calc_time_dur = 1
#                     twist_msg.linear.x = 1.0
#                     twist_msg.linear.y = 0.0
#                     twist_msg.angular.z = 0.0
#                     rospy.loginfo(twist_msg)
#                     cmd_vel_pub.publish(twist_msg)
#                     print("Center: ", data_scan.center_min)
#                     print("Left: ", data_scan.left_min)
#                     print("Right: ", data_scan.right_min)
#                     print("Moving to find tag")
#                 elif (rospy.get_time() - moving_start_time) >= 3.0:
#                     calc_time_dur = 0
#                     moving = False
#                     rotation = True
#             elif rotation:
#                 if calc_time_dur == 0:
#                     rotation_start_time = rospy.get_time()
#                 if (rospy.get_time() - rotation_start_time) < 6.4//DETECTING_SPEED_ROTATION:
#                     calc_time_dur = 1
#                     twist_msg.linear.x = 0.0
#                     twist_msg.linear.y = 0.0
#                     twist_msg.angular.z = DETECTING_SPEED_ROTATION
#                     rospy.loginfo(twist_msg)
#                     cmd_vel_pub.publish(twist_msg)
#                     print("Rotation to find tag")
#                 elif (rospy.get_time() - rotation_start_time) >= 6.4//DETECTING_SPEED_ROTATION:
#                     calc_time_dur = 0
#                     moving = True
#                     rotation = False

def ar_demo():
        global marker_ids
        # Initialize this ROS node
        rospy.init_node('main_project', anonymous=True)
        # get target marker id
        marker_ids = rospy.get_param('~marker_ids').split(",")

        #initialize heartbeat
        #global last_heartbeat
        #last_heartbeat = rospy.get_time()

        #global ar_present
        #ar_present = True

        # Create publisher for command velocity
        global cmd_vel_pub
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        #Register heartbeat timer
        #t = rospy.Timer(rospy.Duration(0.1), timer_callback)

        # Set up subscriber for /ar_pose_marker
        rospy.loginfo("Subscribing to ar_pose_marker and to scan_to_distance")
        
        server = Server()

        #rospy.Subscriber('/orientation', Float64MultiArray , server.orientation_callback)
        #rospy.Subscriber('/velocity', Float64MultiArray, server.velocity_callback)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, server.tag_callback)
        rospy.Subscriber("scan_to_distance", LaserScanSplit, server.laser_callback)


        rospy.spin()

if __name__ == '__main__':
    try:
        ar_demo()
    except rospy.ROSInterruptException:
        pass