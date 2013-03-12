; Auto-generated. Do not edit!


(cl:in-package dynamics-srv)


;//! \htmlinclude capture_image_features-request.msg.html

(cl:defclass <capture_image_features-request> (roslisp-msg-protocol:ros-message)
  ((seq
    :reader seq
    :initarg :seq
    :type cl:fixnum
    :initform 0))
)

(cl:defclass capture_image_features-request (<capture_image_features-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <capture_image_features-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'capture_image_features-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamics-srv:<capture_image_features-request> is deprecated: use dynamics-srv:capture_image_features-request instead.")))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <capture_image_features-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:seq-val is deprecated.  Use dynamics-srv:seq instead.")
  (seq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <capture_image_features-request>) ostream)
  "Serializes a message object of type '<capture_image_features-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <capture_image_features-request>) istream)
  "Deserializes a message object of type '<capture_image_features-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<capture_image_features-request>)))
  "Returns string type for a service object of type '<capture_image_features-request>"
  "dynamics/capture_image_featuresRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'capture_image_features-request)))
  "Returns string type for a service object of type 'capture_image_features-request"
  "dynamics/capture_image_featuresRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<capture_image_features-request>)))
  "Returns md5sum for a message object of type '<capture_image_features-request>"
  "d2fb7032b25fab2a84d464cd5d61960b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'capture_image_features-request)))
  "Returns md5sum for a message object of type 'capture_image_features-request"
  "d2fb7032b25fab2a84d464cd5d61960b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<capture_image_features-request>)))
  "Returns full string definition for message of type '<capture_image_features-request>"
  (cl:format cl:nil "uint8 seq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'capture_image_features-request)))
  "Returns full string definition for message of type 'capture_image_features-request"
  (cl:format cl:nil "uint8 seq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <capture_image_features-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <capture_image_features-request>))
  "Converts a ROS message object to a list"
  (cl:list 'capture_image_features-request
    (cl:cons ':seq (seq msg))
))
;//! \htmlinclude capture_image_features-response.msg.html

(cl:defclass <capture_image_features-response> (roslisp-msg-protocol:ros-message)
  ((kppt
    :reader kppt
    :initarg :kppt
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray))
   (desc
    :reader desc
    :initarg :desc
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray)))
)

(cl:defclass capture_image_features-response (<capture_image_features-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <capture_image_features-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'capture_image_features-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamics-srv:<capture_image_features-response> is deprecated: use dynamics-srv:capture_image_features-response instead.")))

(cl:ensure-generic-function 'kppt-val :lambda-list '(m))
(cl:defmethod kppt-val ((m <capture_image_features-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:kppt-val is deprecated.  Use dynamics-srv:kppt instead.")
  (kppt m))

(cl:ensure-generic-function 'desc-val :lambda-list '(m))
(cl:defmethod desc-val ((m <capture_image_features-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:desc-val is deprecated.  Use dynamics-srv:desc instead.")
  (desc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <capture_image_features-response>) ostream)
  "Serializes a message object of type '<capture_image_features-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kppt) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desc) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <capture_image_features-response>) istream)
  "Deserializes a message object of type '<capture_image_features-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kppt) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desc) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<capture_image_features-response>)))
  "Returns string type for a service object of type '<capture_image_features-response>"
  "dynamics/capture_image_featuresResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'capture_image_features-response)))
  "Returns string type for a service object of type 'capture_image_features-response"
  "dynamics/capture_image_featuresResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<capture_image_features-response>)))
  "Returns md5sum for a message object of type '<capture_image_features-response>"
  "d2fb7032b25fab2a84d464cd5d61960b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'capture_image_features-response)))
  "Returns md5sum for a message object of type 'capture_image_features-response"
  "d2fb7032b25fab2a84d464cd5d61960b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<capture_image_features-response>)))
  "Returns full string definition for message of type '<capture_image_features-response>"
  (cl:format cl:nil "std_msgs/Float32MultiArray kppt~%std_msgs/Float32MultiArray desc~%~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding bytes at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'capture_image_features-response)))
  "Returns full string definition for message of type 'capture_image_features-response"
  (cl:format cl:nil "std_msgs/Float32MultiArray kppt~%std_msgs/Float32MultiArray desc~%~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding bytes at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <capture_image_features-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kppt))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desc))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <capture_image_features-response>))
  "Converts a ROS message object to a list"
  (cl:list 'capture_image_features-response
    (cl:cons ':kppt (kppt msg))
    (cl:cons ':desc (desc msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'capture_image_features)))
  'capture_image_features-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'capture_image_features)))
  'capture_image_features-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'capture_image_features)))
  "Returns string type for a service object of type '<capture_image_features>"
  "dynamics/capture_image_features")