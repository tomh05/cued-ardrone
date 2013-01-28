; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude StampedFrames.msg.html

(cl:defclass <StampedFrames> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (frame1
    :reader frame1
    :initarg :frame1
    :type custom_msgs-msg:StampedMatchesWithImage
    :initform (cl:make-instance 'custom_msgs-msg:StampedMatchesWithImage))
   (frame2
    :reader frame2
    :initarg :frame2
    :type custom_msgs-msg:StampedMatchesWithImage
    :initform (cl:make-instance 'custom_msgs-msg:StampedMatchesWithImage)))
)

(cl:defclass StampedFrames (<StampedFrames>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StampedFrames>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StampedFrames)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<StampedFrames> is deprecated: use custom_msgs-msg:StampedFrames instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StampedFrames>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header-val is deprecated.  Use custom_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'frame1-val :lambda-list '(m))
(cl:defmethod frame1-val ((m <StampedFrames>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:frame1-val is deprecated.  Use custom_msgs-msg:frame1 instead.")
  (frame1 m))

(cl:ensure-generic-function 'frame2-val :lambda-list '(m))
(cl:defmethod frame2-val ((m <StampedFrames>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:frame2-val is deprecated.  Use custom_msgs-msg:frame2 instead.")
  (frame2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StampedFrames>) ostream)
  "Serializes a message object of type '<StampedFrames>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'frame1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'frame2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StampedFrames>) istream)
  "Deserializes a message object of type '<StampedFrames>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'frame1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'frame2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StampedFrames>)))
  "Returns string type for a message object of type '<StampedFrames>"
  "custom_msgs/StampedFrames")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StampedFrames)))
  "Returns string type for a message object of type 'StampedFrames"
  "custom_msgs/StampedFrames")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StampedFrames>)))
  "Returns md5sum for a message object of type '<StampedFrames>"
  "ee218eba4fa8a4f54760afca804eb3b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StampedFrames)))
  "Returns md5sum for a message object of type 'StampedFrames"
  "ee218eba4fa8a4f54760afca804eb3b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StampedFrames>)))
  "Returns full string definition for message of type '<StampedFrames>"
  (cl:format cl:nil "Header header~%custom_msgs/StampedMatchesWithImage frame1~%custom_msgs/StampedMatchesWithImage frame2~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: custom_msgs/StampedMatchesWithImage~%Header header~%float32[] pts~%sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StampedFrames)))
  "Returns full string definition for message of type 'StampedFrames"
  (cl:format cl:nil "Header header~%custom_msgs/StampedMatchesWithImage frame1~%custom_msgs/StampedMatchesWithImage frame2~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: custom_msgs/StampedMatchesWithImage~%Header header~%float32[] pts~%sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StampedFrames>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'frame1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'frame2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StampedFrames>))
  "Converts a ROS message object to a list"
  (cl:list 'StampedFrames
    (cl:cons ':header (header msg))
    (cl:cons ':frame1 (frame1 msg))
    (cl:cons ':frame2 (frame2 msg))
))
