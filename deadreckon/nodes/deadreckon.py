#!/usr/bin/env python  
import roslib
roslib.load_manifest('deadreckon')
import rospy
import ardrone_autonomy.msg
import tf
import math

def navdataCallback(d):
    print d.rotX
    br = tf.TransformBroadcaster() #create broadcaster
    br.sendTransform((0,0,0),
                     tf.transformations.quaternion_from_euler(d.rotX * math.pi / 180, d.rotY * math.pi / 180, d.rotZ * math.pi / 180),
                     rospy.Time.now(),
                     "ardrone_base_link",
                     "world")

if __name__ == '__main__':
    rospy.init_node('deadreckon_broadcaster')
    #turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,navdataCallback)
    rospy.spin()
