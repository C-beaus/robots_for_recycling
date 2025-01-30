#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Bool

# Replace this with the correct topic for your deadman switch
DEADMAN_SWITCH_TOPIC = '/franka_panda/deadman_switch'

def wait_for_deadman_switch():
    rospy.init_node('deadman_switch_wait', anonymous=True)

    # Subscriber to the deadman switch status
    def callback(msg):
        if msg.data:
            rospy.loginfo("Deadman switch is held. Proceeding with the launch.")
            return True
        else:
            return False

    rospy.Subscriber(DEADMAN_SWITCH_TOPIC, Bool, callback)
    rospy.loginfo("Waiting for deadman switch to be held...")
    
    # Wait until the deadman switch is held
    while not rospy.is_shutdown():
        if callback():
            break
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        wait_for_deadman_switch()
    except rospy.ROSInterruptException:
        pass
