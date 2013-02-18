; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude Visualisation.msg.html

(cl:defclass <Visualisation> (roslisp-msg-protocol:ros-message)
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
   (points1
    :reader points1
    :initarg :points1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (points2
    :reader points2
    :initarg :points2
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (reprojected1
    :reader reprojected1
    :initarg :reprojected1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (reprojected2
    :reader reprojected2
    :initarg :reprojected2
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Visualisation (<Visualisation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Visualisation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Visualisation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<Visualisation> is deprecated: use custom_msgs-msg:Visualisation instead.")))

(cl:ensure-generic-function 'header1-val :lambda-list '(m))
(cl:defmethod header1-val ((m <Visualisation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header1-val is deprecated.  Use custom_msgs-msg:header1 instead.")
  (header1 m))

(cl:ensure-generic-function 'header2-val :lambda-list '(m))
(cl:defmethod header2-val ((m <Visualisation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header2-val is deprecated.  Use custom_msgs-msg:header2 instead.")
  (header2 m))

(cl:ensure-generic-function 'points1-val :lambda-list '(m))
(cl:defmethod points1-val ((m <Visualisation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:points1-val is deprecated.  Use custom_msgs-msg:points1 instead.")
  (points1 m))

(cl:ensure-generic-function 'points2-val :lambda-list '(m))
(cl:defmethod points2-val ((m <Visualisation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:points2-val is deprecated.  Use custom_msgs-msg:points2 instead.")
  (points2 m))

(cl:ensure-generic-function 'reprojected1-val :lambda-list '(m))
(cl:defmethod reprojected1-val ((m <Visualisation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:reprojected1-val is deprecated.  Use custom_msgs-msg:reprojected1 instead.")
  (reprojected1 m))

(cl:ensure-generic-function 'reprojected2-val :lambda-list '(m))
(cl:defmethod reprojected2-val ((m <Visualisation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:reprojected2-val is deprecated.  Use custom_msgs-msg:reprojected2 instead.")
  (reprojected2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Visualisation>) ostream)
  "Serializes a message object of type '<Visualisation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header2) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'points1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'points2))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'reprojected1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'reprojected1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'reprojected2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'reprojected2))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Visualisation>) istream)
  "Deserializes a message object of type '<Visualisation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header2) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points1)))
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
  (cl:setf (cl:slot-value msg 'points2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points2)))
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
  (cl:setf (cl:slot-value msg 'reprojected1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'reprojected1)))
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
  (cl:setf (cl:slot-value msg 'reprojected2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'reprojected2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Visualisation>)))
  "Returns string type for a message object of type '<Visualisation>"
  "custom_msgs/Visualisation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Visualisation)))
  "Returns string type for a message object of type 'Visualisation"
  "custom_msgs/Visualisation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Visualisation>)))
  "Returns md5sum for a message object of type '<Visualisation>"
  "b425dd08e96d44f498f69a6461e54acd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Visualisation)))
  "Returns md5sum for a message object of type 'Visualisation"
  "b425dd08e96d44f498f69a6461e54acd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Visualisation>)))
  "Returns full string definition for message of type '<Visualisation>"
  (cl:format cl:nil "Header header1~%Header header2~%float32[] points1~%float32[] points2~%float32[] reprojected1~%float32[] reprojected2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Visualisation)))
  "Returns full string definition for message of type 'Visualisation"
  (cl:format cl:nil "Header header1~%Header header2~%float32[] points1~%float32[] points2~%float32[] reprojected1~%float32[] reprojected2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Visualisation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header2))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'reprojected1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'reprojected2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Visualisation>))
  "Converts a ROS message object to a list"
  (cl:list 'Visualisation
    (cl:cons ':header1 (header1 msg))
    (cl:cons ':header2 (header2 msg))
    (cl:cons ':points1 (points1 msg))
    (cl:cons ':points2 (points2 msg))
    (cl:cons ':reprojected1 (reprojected1 msg))
    (cl:cons ':reprojected2 (reprojected2 msg))
))
