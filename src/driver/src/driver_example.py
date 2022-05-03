#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist 

robot_vel = Twist()

def move():
    pub = rospy.Publisher('test_move', Twist, queue_size=1)
    #robot_vel.linear.x = 1.0
    robot_vel.angular.z = 1.0

    now = rospy.Time.now()
    rate = rospy.Rate(10)

    while rospy.Time.now() < now + rospy.Duration.from_sec(1):
        pub.publish(robot_vel)
        rate.sleep() 

if __name__ == '__main__':
    try:
        rospy.init_node('driver_example', anonymous=True)
        move()
    except rospy.ROSInterruptException:
        pass


