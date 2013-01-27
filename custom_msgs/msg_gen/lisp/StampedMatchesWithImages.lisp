; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude StampedMatchesWithImages.msg.html

(cl:defclass <StampedMatchesWithImages> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pts1
    :reader pts1
    :initarg :pts1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (pts2
    :reader pts2
    :initarg :pts2
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (image1
    :reader image1
    :initarg :image1
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (image2
    :reader image2
    :initarg :image2
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass StampedMatchesWithImages (<StampedMatchesWithImages>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StampedMatchesWithImages>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StampedMatchesWithImages)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<StampedMatchesWithImages> is deprecated: use custom_msgs-msg:StampedMatchesWithImages instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StampedMatchesWithImages>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header-val is deprecated.  Use custom_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pts1-val :lambda-list '(m))
(cl:defmethod pts1-val ((m <StampedMatchesWithImages>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:pts1-val is deprecated.  Use custom_msgs-msg:pts1 instead.")
  (pts1 m))

(cl:ensure-generic-function 'pts2-val :lambda-list '(m))
(cl:defmethod pts2-val ((m <StampedMatchesWithImages>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:pts2-val is deprecated.  Use custom_msgs-msg:pts2 instead.")
  (pts2 m))

(cl:ensure-generic-function 'image1-val :lambda-list '(m))
(cl:defmethod image1-val ((m <StampedMatchesWithImages>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:image1-val is deprecated.  Use custom_msgs-msg:image1 instead.")
  (image1 m))

(cl:ensure-generic-function 'image2-val :lambda-list '(m))
(cl:defmethod image2-val ((m <StampedMatchesWithImages>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:image2-val is deprecated.  Use custom_msgs-msg:image2 instead.")
  (image2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StampedMatchesWithImages>) ostream)
  "Serializes a message object of type '<StampedMatchesWithImages>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pts1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pts1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pts2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pts2))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StampedMatchesWithImages>) istream)
  "Deserializes a message object of type '<StampedMatchesWithImages>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pts1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pts1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pts2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pts2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StampedMatchesWithImages>)))
  "Returns string type for a message object of type '<StampedMatchesWithImages>"
  "custom_msgs/StampedMatchesWithImages")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StampedMatchesWithImages)))
  "Returns string type for a message object of type 'StampedMatchesWithImages"
  "custom_msgs/StampedMatchesWithImages")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StampedMatchesWithImages>)))
  "Returns md5sum for a message object of type '<StampedMatchesWithImages>"
  "77783dda85e700368d6d31138c7cf6ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StampedMatchesWithImages)))
  "Returns md5sum for a message object of type 'StampedMatchesWithImages"
  "77783dda85e700368d6d31138c7cf6ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StampedMatchesWithImages>)))
  "Returns full string definition for message of type '<StampedMatchesWithImages>"
  (cl:format cl:nil "Header header~%float32[] pts1~%float32[] pts2~%sensor_msgs/Image image1~%sensor_msgs/Image image2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StampedMatchesWithImages)))
  "Returns full string definition for message of type 'StampedMatchesWithImages"
  (cl:format cl:nil "Header header~%float32[] pts1~%float32[] pts2~%sensor_msgs/Image image1~%sensor_msgs/Image image2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StampedMatchesWithImages>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pts1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pts2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StampedMatchesWithImages>))
  "Converts a ROS message object to a list"
  (cl:list 'StampedMatchesWithImages
    (cl:cons ':header (header msg))
    (cl:cons ':pts1 (pts1 msg))
    (cl:cons ':pts2 (pts2 msg))
    (cl:cons ':image1 (image1 msg))
    (cl:cons ':image2 (image2 msg))
))
