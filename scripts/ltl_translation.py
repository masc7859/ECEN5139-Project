#!/usr/bin/env python
import rospy
from multiprocessing import process
from std_msgs.msg import String

def main():
    pub = rospy.Publisher('robot_commands', String, queue_size=20)
    rospy.init_node('ltl_translator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    ltlInput = rospy.get_param('/ltl_translation/ltl_input')

    while not rospy.is_shutdown():
        rospy.get_time()
        rospy.loginfo(ltlInput)
        pub.publish(ltlInput)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
