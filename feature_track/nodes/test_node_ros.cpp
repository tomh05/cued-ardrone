import roslib
import rospy
from cv_bridge import CvBridge

def test_run():
    print 'testing'

def test_ros_node_server():
    rospy.init_node('test_node_ros')
    s = rospy.Service('test_node_ros', TestRun, test_run)


if ___name___ == "___main___":
        test_ros_node_server()


