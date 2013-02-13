; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude StampedStampedInt.msg.html

(cl:defclass <StampedStampedInt> (roslisp-msg-protocol:ros-message)
  ((header1
    :reader header1
    :initarg :header1
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (header2
    :reader header2
    :initarg :header2
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (count
    :reader count
    :initarg :count
    :type cl:fixnum
    :initform 0))
)

(cl:defclass StampedStampedInt (<StampedStampedInt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StampedStampedInt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StampedStampedInt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<StampedStampedInt> is deprecated: use custom_msgs-msg:StampedStampedInt instead.")))

(cl:ensure-generic-function 'header1-val :lambda-list '(m))
(cl:defmethod header1-val ((m <StampedStampedInt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header1-val is deprecated.  Use custom_msgs-msg:header1 instead.")
  (header1 m))

(cl:ensure-generic-function 'header2-val :lambda-list '(m))
(cl:defmethod header2-val ((m <StampedStampedInt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header2-val is deprecated.  Use custom_msgs-msg:header2 instead.")
  (header2 m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <StampedStampedInt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:count-val is deprecated.  Use custom_msgs-msg:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StampedStampedInt>) ostream)
  "Serializes a message object of type '<StampedStampedInt>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header2) ostream)
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StampedStampedInt>) istream)
  "Deserializes a message object of type '<StampedStampedInt>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header2) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StampedStampedInt>)))
  "Returns string type for a message object of type '<StampedStampedInt>"
  "custom_msgs/StampedStampedInt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StampedStampedInt)))
  "Returns string type for a message object of type 'StampedStampedInt"
  "custom_msgs/StampedStampedInt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StampedStampedInt>)))
  "Returns md5sum for a message object of type '<StampedStampedInt>"
  "456f6e7b11f035fb268f51b6a451715b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StampedStampedInt)))
  "Returns md5sum for a message object of type 'StampedStampedInt"
  "456f6e7b11f035fb268f51b6a451715b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StampedStampedInt>)))
  "Returns full string definition for message of type '<StampedStampedInt>"
  (cl:format cl:nil "Header header1~%Header header2~%int16 count~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StampedStampedInt)))
  "Returns full string definition for message of type 'StampedStampedInt"
  (cl:format cl:nil "Header header1~%Header header2~%int16 count~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StampedStampedInt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header2))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StampedStampedInt>))
  "Converts a ROS message object to a list"
  (cl:list 'StampedStampedInt
    (cl:cons ':header1 (header1 msg))
    (cl:cons ':header2 (header2 msg))
    (cl:cons ':count (count msg))
))
