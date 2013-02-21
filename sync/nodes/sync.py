#!/bin/sh
''''exec python -u -- "$0" ${1+"$@"} # '''

#=================================== Sync =====================================
# 
# This node sets navdata and image header stamps to actual measurement time
#
#==============================================================================
#
# ardrone_driver publishes navdata and image_raw with header stamp
# The navdata header corresponds to the time ardrone_driver receives navdata
# The image_raw header corresponds to the time ardrone_driver sends image_raw
#
# navdata.tm is the drone time in milliseconds
# image_raw does not have a drone time (it is stripped in the parrot pipeline)
# 
# navdata_video_stream provides info about parrot encoding including drone
# times.
#
# This node first syncronises the navdata_video_stream frame timings and the 
# image_raw frames. It uses this to establish a drone time to ROS time offset
# and republishes the navdata and image_raw feeds with header stamps that are
# in the same time frame and reflect measurement time.
#
#==============================================================================
#
# As there is essentially no documentation on this, much of this is speculative
#
# The navdata_video_stream is sent at navdata frequency (200Hz) and reports the
# video encoder state. Of particular interest is that it reports the number of 
# the current frame being encoded and the drone time of this state. Video 
# frames are encoded/sent at 30Hz so there are several encoder state reports 
# per frame.
#
# We can place bounds on the processing time per frame (essentially the time
# between starting encoding and receiving the image) as between 0 and 1/30 secs
# If encoding took and longer than this the send buffer would underflow and the
# encode buffer would overflow, whereas we continously and reliably see frames
# at 30Hz.
#
# We also assume that encoding occurs necessarily before we receive a frame (a)
# However, the ordering of encoding msgs and received frames is not constant.
# Encoding msgs sometimes will get several frames ahead of image frames, and be
# followed by several image frames that catch up the count.
#
# Using these assumptions a syncing strategy was creating as the following:
#   Initialise the lock as between the encoding frame and the image frame where
#   the image time is closest too, but strictly not exceeding the encode time
#
#   Monitor the encode and image frames. If an image frame ever occurs before
#   the (apparently) corresponding encoder frame, shift the lock to stop this
#
#   Eventually a lock should be reached such that img frames never precede 
#   encode frames - Condition (a) is satisfied. By starting from the minimum
#   offset, and increasing only when the lock is proven incorrect we guarantee
#   that the lock is the actual one; any frame shift exceeding this would still
#   satisfy (a) but be incorrect
# 
#   Network unreliability means image frames are not always received (Decoding
#   failed for I frame). This will mean the image frame count is out by 1.
#   There is no direct way to detect this beyond the image frame to frame time
#   significantly exceeding the average
#
#==============================================================================
#
#   There are several potential flaws to the assumptions made:
#
# Assumption 1: An image frame cannot precede the encoding frame.
#   
#   Possible Alternative:
#
#   Theoretically an image frame may be able to be received before the encode
#   message as the encode messages are updated at the drones internal 200Hz
#   cycle. If encoding time was arbitrarily small (less than 1/200) the image
#   frame could be received before the encode frame was updated as images are
#   sent when encoded, whereas the encode frames are sent at constant rate.
#
#   Effect:
#
#   This will cause the drone to track too far(overcompensate for encoder time)
#   There is no provision to compensate for this, but it does not appear to
#   occur
#
# Assumption 2: Encode to receive time is less than frame to frame period
#
#   Possible Alternative:
#
#   The rate of images outputted by the onboard pipeline must be 30Hz to stop
#   over/underflow. However, the lead time (the time an image spends in the 
#   pipeline) may be arbitrarily long. A reasonable example is if the network
#   transmission is handled by the main processor, but the video encoding is 
#   carried out by a dedicated encoding chip. In this case, a frame could spend
#   1/30 secs in the encoding stage then 1/30 secs in the transmission stage.
#   Images would still enter and leave the pipeline at 30Hz, but more than one
#   image would be in the pipeline at once being processed in parallel.
#
#   Effect:
#
#   If the lead time is greater than the assumed 1/30 bound we will be under
#   correcting for the lead time. The output time will still however be closer
#   to the actual time than before. Also if the time is unstable (specifically
#   sometimes less than 1/30) the node will correctly track regardless of the
#   incorrect assumption as it is only used to lock not track.
#
# Assumption 3: The frame number is the number of frames put into the encoder
#
#   Possible Alternative:
#
#   The actual meaning of the frame count is undocumented. It could equally be
#   the number of frames that have been encoded.
#
#   Effect:
#
#   Sync will only correct for the post encoding lead time, so under corrects.
#   This is still better than un-corrected and in this situation there is no 
#   way to recover the pre-encode time unless there is some hitherto unknown
#   output from the drone. 
#
#==============================================================================
#
#   Is this worth it?
#   There are three likely cases:
#
# Case 1: Both image encoding time and image & navdata network times are small
#         (or matched)
#   
#   In this case the ROS stamps are ~correct. There is no point doing this.
#
# Case 2: Encode time is small but image network time exceeds navdata net time
#
#   ROS stamps are out of sync. This is worthwhile
#
# Case 3: Encode time is not small, but image & network times are small (or
#         matched)
#
#   ROS stamps are out of sync. This is worthwhile provided the frame numbers
#   are pre-encode (See assumption 3)
#
# Case 4: Encode time is not small and image net time exceeds navdata net time
#
#   ROS stamps are out of sync. This is worthwhile
#
# Test results:
#   Correction is typically ~5-6 ms. Given we can only possibly sync to 5 ms
#   resolution (1/200Hz), this is not even credible. This is NOT worthwhile
#
#==============================================================================



import roslib
roslib.load_manifest('sync')
import rospy
import ardrone_autonomy.msg
import math
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
import sys




class Syncer:
    def __init__(self):
        self.locked = False
        
        self.locked_stamp = None
        
        self.ros_time_offset = None
        self.frame_offset = 0
        self.frame_prev = -1
        self.frame_count = 0
        
        self.time_buffer = []
        self.image_buffer = []
        
        self.dnav = 0
        self.dimage = 0
        
        print "Unlocked"
        self.connect()
        
        self.corrections = []
        
        
    def connect(self):
        rospy.Subscriber('/ardrone/navdata',ardrone_autonomy.msg.Navdata,self.on_got_navdata)
        rospy.Subscriber('/ardrone/navdata_video_stream',ardrone_autonomy.msg.navdata_video_stream,self.on_got_navdata_video_stream)
        rospy.Subscriber('/ardrone/front/image_raw',Image, self.on_got_image)
        self.image_pub = rospy.Publisher('/sync/image_raw', Image)
        self.nav_pub = rospy.Publisher('/sync/navdata', ardrone_autonomy.msg.Navdata)
        
    def on_got_image(self, d):
        self.frame_count = self.frame_count + 1
        #print "img: ", self.frame_count
        
        if not self.locked:
            
            if len(self.time_buffer) < 10:
                return
            i = d.header
            fc = self.frame_count
            
            best = None
            self.bests = []
            
            for index, t in enumerate(self.time_buffer):
                if t[0].header.stamp.to_sec() <= i.stamp.to_sec():
                    best = (t[0], i, index)
                    self.bests.append(best)
                elif best != None:
                    self.frame_offset = best[0].frame_number + self.frame_count - fc
                    self.frame_count = 0
                    self.ros_time_offset = best[0].header.stamp.to_sec()-best[0].drone_time
                    self.dnav = best[0].frame_number - self.frame_offset
                    self.locked = True
                    print "Locked"
                    self.best_index = index
                    self.update_time_buffer()
                    return
        else:
            print "Delta: ", self.dnav - self.frame_count
            if self.dnav < self.frame_count:
                # We have received an image frame before the encoder frame
                # This means the tracking is wrong
                # We need to shift tracking by (at least) one
                self.track(self.dnav - self.frame_count)
                return
            if len(self.time_buffer) > 0:
                # In general the time buffer should only contain one entry
                # so this operation is not overly costly
                for i, t in enumerate(self.time_buffer):
                    if self.frame_count == t[1]:
                        # If the image does not match the first entry in the 
                        # buffer, we must have missed an image frame. The
                        # method of tracking cannot overtrack, therefore
                        # we must have missed a frame (likely due to network
                        # issues). As such we do not need to track, but to 
                        # correct the frame count
                        self.corrections.append(np.around((t[0].header.stamp.to_sec() - (self.ros_time_offset+t[0].drone_time))*1000,1))
                        print "Correction: ", np.mean(self.corrections)
                        self.publish_image(d, t[0].drone_time)
                        if (i != 0):
                            print "WARNING: Suspected image drop"
                        del self.time_buffer[0:i+1]
                        self.frame_count = self.frame_count + i
                        return
            # We have received an image frame without an entry in the buffer
            # However, we have recorded receiving the encoder frame
            # It should by all rights be in the buffer
            # This state will only occur with 6+ missed encoder frames or 
            # serious desync
            print "ERROR: Buffer is empty but should not be"
    
    def update_time_buffer(self):
        # Ensure self.dnav is up-to-date
        print "Checking time buffer"
        slice_index = 0
        for i, t in enumerate(self.time_buffer):
            self.dnav = t[0].frame_number-self.frame_offset
            self.time_buffer[i] = (t[0], self.dnav)
            if self.dnav == self.frame_count:
                slice_index = i
        del self.time_buffer[0:slice_index+1]
        print "Done"
        print "-----------------------------"
        
    def unlock(self):
        self.locked = False
        self.time_buffer = []
        return
    
    def track(self, offset):
        # Shifts the tracking position
        self.best_index = self.best_index + offset
        if self.best_index < 0:
            print "Unlocked < - Locked"
            self.unlock()
            return
        elif self.best_index >= len(self.bests):
            print "Unlocked < - Locked"
            self.unlock()
            return
        print "Tracking: ", offset
        self.frame_offset = self.frame_offset + offset
        self.ros_time_offset = self.bests[self.best_index][0].header.stamp.to_sec()-self.bests[self.best_index][0].drone_time
        self.dnav = self.dnav - offset
    
    def on_got_navdata_video_stream(self, d):
        if self.frame_prev != d.frame_number:
            self.frame_prev = d.frame_number
            new_dnav = d.frame_number-self.frame_offset
            if new_dnav - self.dnav > 1:
                print "Serious frame loss"
                # Correcting like this will either:
                # 1) Correct for dropped image frames
                # 2) Move image frames ahead of encode frames. This will cause
                #    a complete desync which will be detected. The sync will
                #    release the lock and attempt to resynchronise.
                self.frame_count = self.frame_count + new_dnav - self.dnav -1
            self.dnav = new_dnav
            self.time_buffer.append((d, self.dnav))
            #print "enc: ", self.dnav
        
    def on_got_navdata(self, d):
        if self.locked:
            d.header.stamp = rospy.Time.from_sec(self.ros_time_offset+d.tm*0.000001)
            self.nav_pub.publish(d)
        
    def publish_image(self, image, tm):
        image.header.stamp = rospy.Time.from_sec(self.ros_time_offset+tm)
        self.image_pub.publish(image)
        


if __name__ == '__main__':
    
    rospy.init_node('sync')
    
    print "\r\n"
    print "============================== Sync ==============================="
    print " Wait until stable on Double Locked state"
    print "==================================================================="
    print "\r\n"
    s = Syncer() # Initialise class to preserve vars
    rospy.spin()
