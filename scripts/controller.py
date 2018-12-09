#!/usr/bin/env python
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

def main():
    pub = rospy.Publisher('/robot/goal_end_pose', Pose)
    rospy.init_node('sawyer_controller', anonymous=True)
    r = rospy.Rate(1)
    msg = Pose()
    msg.position.x =
    msg.position.y =
    msg.position.z =

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
