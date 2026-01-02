#!/usr/bin/env python3
import rospy
import rostest
import sys
from vision_object_tracking.msg import ObjectDetection

received = False

def callback(msg):
    global received
    rospy.loginfo('Test received detection with confidence: %f', msg.confidence)
    if msg.confidence >= rospy.get_param('~conf_threshold', 0.1):
        received = True


if __name__ == '__main__':
    rospy.init_node('test_detection_node', anonymous=True)
    conf = rospy.get_param('~conf_threshold', 0.1)
    timeout = rospy.get_param('~timeout', 15.0)

    rospy.Subscriber('/object_position_detection', ObjectDetection, callback)

    start = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if received:
            rospy.loginfo('Detection test passed')
            sys.exit(0)
        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logerr('Detection test timed out after %f seconds', timeout)
            sys.exit(1)
        rate.sleep()
