#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist


def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmdvel', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist =Twist()
        twist.angular.z=0
        twist.linear.x=0.1
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")



