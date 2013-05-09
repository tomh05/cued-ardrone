; Auto-generated. Do not edit!


(cl:in-package dynamics-srv)


;//! \htmlinclude CaptureImageFeatures-request.msg.html

(cl:defclass <CaptureImageFeatures-request> (roslisp-msg-protocol:ros-message)
  ((seq
    :reader seq
    :initarg :seq
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CaptureImageFeatures-request (<CaptureImageFeatures-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CaptureImageFeatures-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CaptureImageFeatures-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamics-srv:<CaptureImageFeatures-request> is deprecated: use dynamics-srv:CaptureImageFeatures-request instead.")))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <CaptureImageFeatures-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:seq-val is deprecated.  Use dynamics-srv:seq instead.")
  (seq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CaptureImageFeatures-request>) ostream)
  "Serializes a message object of type '<CaptureImageFeatures-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CaptureImageFeatures-request>) istream)
  "Deserializes a message object of type '<CaptureImageFeatures-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CaptureImageFeatures-request>)))
  "Returns string type for a service object of type '<CaptureImageFeatures-request>"
  "dynamics/CaptureImageFeaturesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImageFeatures-request)))
  "Returns string type for a service object of type 'CaptureImageFeatures-request"
  "dynamics/CaptureImageFeaturesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CaptureImageFeatures-request>)))
  "Returns md5sum for a message object of type '<CaptureImageFeatures-request>"
  "28b1941d1b0f79162798fe518cc30bc6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CaptureImageFeatures-request)))
  "Returns md5sum for a message object of type 'CaptureImageFeatures-request"
  "28b1941d1b0f79162798fe518cc30bc6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CaptureImageFeatures-request>)))
  "Returns full string definition for message of type '<CaptureImageFeatures-request>"
  (cl:format cl:nil "uint8 seq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CaptureImageFeatures-request)))
  "Returns full string definition for message of type 'CaptureImageFeatures-request"
  (cl:format cl:nil "uint8 seq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CaptureImageFeatures-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CaptureImageFeatures-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CaptureImageFeatures-request
    (cl:cons ':seq (seq msg))
))
;//! \htmlinclude CaptureImageFeatures-response.msg.html

(cl:defclass <CaptureImageFeatures-response> (roslisp-msg-protocol:ros-message)
  ((error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0)
   (kppt
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

(cl:defclass CaptureImageFeatures-response (<CaptureImageFeatures-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CaptureImageFeatures-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CaptureImageFeatures-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamics-srv:<CaptureImageFeatures-response> is deprecated: use dynamics-srv:CaptureImageFeatures-response instead.")))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <CaptureImageFeatures-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:error-val is deprecated.  Use dynamics-srv:error instead.")
  (error m))

(cl:ensure-generic-function 'kppt-val :lambda-list '(m))
(cl:defmethod kppt-val ((m <CaptureImageFeatures-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:kppt-val is deprecated.  Use dynamics-srv:kppt instead.")
  (kppt m))

(cl:ensure-generic-function 'desc-val :lambda-list '(m))
(cl:defmethod desc-val ((m <CaptureImageFeatures-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:desc-val is deprecated.  Use dynamics-srv:desc instead.")
  (desc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CaptureImageFeatures-response>) ostream)
  "Serializes a message object of type '<CaptureImageFeatures-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kppt) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desc) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CaptureImageFeatures-response>) istream)
  "Deserializes a message object of type '<CaptureImageFeatures-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kppt) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desc) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CaptureImageFeatures-response>)))
  "Returns string type for a service object of type '<CaptureImageFeatures-response>"
  "dynamics/CaptureImageFeaturesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImageFeatures-response)))
  "Returns string type for a service object of type 'CaptureImageFeatures-response"
  "dynamics/CaptureImageFeaturesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CaptureImageFeatures-response>)))
  "Returns md5sum for a message object of type '<CaptureImageFeatures-response>"
  "28b1941d1b0f79162798fe518cc30bc6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CaptureImageFeatures-response)))
  "Returns md5sum for a message object of type 'CaptureImageFeatures-response"
  "28b1941d1b0f79162798fe518cc30bc6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CaptureImageFeatures-response>)))
  "Returns full string definition for message of type '<CaptureImageFeatures-response>"
  (cl:format cl:nil "uint8 error~%std_msgs/Float32MultiArray kppt~%std_msgs/Float32MultiArray desc~%~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding bytes at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CaptureImageFeatures-response)))
  "Returns full string definition for message of type 'CaptureImageFeatures-response"
  (cl:format cl:nil "uint8 error~%std_msgs/Float32MultiArray kppt~%std_msgs/Float32MultiArray desc~%~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding bytes at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CaptureImageFeatures-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kppt))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desc))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CaptureImageFeatures-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CaptureImageFeatures-response
    (cl:cons ':error (error msg))
    (cl:cons ':kppt (kppt msg))
    (cl:cons ':desc (desc msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CaptureImageFeatures)))
  'CaptureImageFeatures-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CaptureImageFeatures)))
  'CaptureImageFeatures-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImageFeatures)))
  "Returns string type for a service object of type '<CaptureImageFeatures>"
  "dynamics/CaptureImageFeatures")