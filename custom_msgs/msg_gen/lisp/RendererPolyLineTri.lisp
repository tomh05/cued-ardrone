; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude RendererPolyLineTri.msg.html

(cl:defclass <RendererPolyLineTri> (roslisp-msg-protocol:ros-message)
  ((floorlines
    :reader floorlines
    :initarg :floorlines
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (polygons
    :reader polygons
    :initarg :polygons
    :type (cl:vector geometry_msgs-msg:Polygon)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Polygon :initial-element (cl:make-instance 'geometry_msgs-msg:Polygon)))
   (triangles
    :reader triangles
    :initarg :triangles
    :type (cl:vector geometry_msgs-msg:Polygon)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Polygon :initial-element (cl:make-instance 'geometry_msgs-msg:Polygon))))
)

(cl:defclass RendererPolyLineTri (<RendererPolyLineTri>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RendererPolyLineTri>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RendererPolyLineTri)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<RendererPolyLineTri> is deprecated: use custom_msgs-msg:RendererPolyLineTri instead.")))

(cl:ensure-generic-function 'floorlines-val :lambda-list '(m))
(cl:defmethod floorlines-val ((m <RendererPolyLineTri>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:floorlines-val is deprecated.  Use custom_msgs-msg:floorlines instead.")
  (floorlines m))

(cl:ensure-generic-function 'polygons-val :lambda-list '(m))
(cl:defmethod polygons-val ((m <RendererPolyLineTri>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:polygons-val is deprecated.  Use custom_msgs-msg:polygons instead.")
  (polygons m))

(cl:ensure-generic-function 'triangles-val :lambda-list '(m))
(cl:defmethod triangles-val ((m <RendererPolyLineTri>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:triangles-val is deprecated.  Use custom_msgs-msg:triangles instead.")
  (triangles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RendererPolyLineTri>) ostream)
  "Serializes a message object of type '<RendererPolyLineTri>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'floorlines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'floorlines))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'polygons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'polygons))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'triangles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'triangles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RendererPolyLineTri>) istream)
  "Deserializes a message object of type '<RendererPolyLineTri>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'floorlines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'floorlines)))
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
  (cl:setf (cl:slot-value msg 'polygons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'polygons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Polygon))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'triangles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'triangles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Polygon))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RendererPolyLineTri>)))
  "Returns string type for a message object of type '<RendererPolyLineTri>"
  "custom_msgs/RendererPolyLineTri")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RendererPolyLineTri)))
  "Returns string type for a message object of type 'RendererPolyLineTri"
  "custom_msgs/RendererPolyLineTri")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RendererPolyLineTri>)))
  "Returns md5sum for a message object of type '<RendererPolyLineTri>"
  "2e629a34699ab4fdf40197c2a9032c96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RendererPolyLineTri)))
  "Returns md5sum for a message object of type 'RendererPolyLineTri"
  "2e629a34699ab4fdf40197c2a9032c96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RendererPolyLineTri>)))
  "Returns full string definition for message of type '<RendererPolyLineTri>"
  (cl:format cl:nil "float32[] floorlines~%geometry_msgs/Polygon[] polygons~%geometry_msgs/Polygon[] triangles~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RendererPolyLineTri)))
  "Returns full string definition for message of type 'RendererPolyLineTri"
  (cl:format cl:nil "float32[] floorlines~%geometry_msgs/Polygon[] polygons~%geometry_msgs/Polygon[] triangles~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RendererPolyLineTri>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'floorlines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'polygons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'triangles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RendererPolyLineTri>))
  "Converts a ROS message object to a list"
  (cl:list 'RendererPolyLineTri
    (cl:cons ':floorlines (floorlines msg))
    (cl:cons ':polygons (polygons msg))
    (cl:cons ':triangles (triangles msg))
))
