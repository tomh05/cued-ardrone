#!/usr/bin/env python

import roslib; roslib.load_manifest('dynamics')
import rospy
from ardrone_autonomy.srv import CamSelect


if __name__ == '__main__':
	rospy.init_node('setdronecamera', anonymous=True)
	camera = rospy.get_param('~camera', 'Invalid default')
	camselectclient = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
	if camera == 'front':
		camselectclient(0)
	elif camera == 'bottom':
		camselectclient(1)
	else:
		rospy.logerr("Camera param string invalid! (%s). Choose from 'front' or 'bottom'.", camera)
	rospy.spin()
