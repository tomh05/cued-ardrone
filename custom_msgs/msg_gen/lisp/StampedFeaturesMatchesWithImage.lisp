; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude StampedFeaturesMatchesWithImage.msg.html

(cl:defclass <StampedFeaturesMatchesWithImage> (roslisp-msg-protocol:ros-message)
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
   (descriptors1
    :reader descriptors1
    :initarg :descriptors1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (descriptors2
    :reader descriptors2
    :initarg :descriptors2
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (descriptors1_stride
    :reader descriptors1_stride
    :initarg :descriptors1_stride
    :type cl:fixnum
    :initform 0)
   (descriptors2_stride
    :reader descriptors2_stride
    :initarg :descriptors2_stride
    :type cl:fixnum
    :initform 0)
   (descriptors1_matcher
    :reader descriptors1_matcher
    :initarg :descriptors1_matcher
    :type cl:string
    :initform "")
   (descriptors2_matcher
    :reader descriptors2_matcher
    :initarg :descriptors2_matcher
    :type cl:string
    :initform "")
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

(cl:defclass StampedFeaturesMatchesWithImage (<StampedFeaturesMatchesWithImage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StampedFeaturesMatchesWithImage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StampedFeaturesMatchesWithImage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<StampedFeaturesMatchesWithImage> is deprecated: use custom_msgs-msg:StampedFeaturesMatchesWithImage instead.")))

(cl:ensure-generic-function 'header1-val :lambda-list '(m))
(cl:defmethod header1-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header1-val is deprecated.  Use custom_msgs-msg:header1 instead.")
  (header1 m))

(cl:ensure-generic-function 'header2-val :lambda-list '(m))
(cl:defmethod header2-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header2-val is deprecated.  Use custom_msgs-msg:header2 instead.")
  (header2 m))

(cl:ensure-generic-function 'points1-val :lambda-list '(m))
(cl:defmethod points1-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:points1-val is deprecated.  Use custom_msgs-msg:points1 instead.")
  (points1 m))

(cl:ensure-generic-function 'points2-val :lambda-list '(m))
(cl:defmethod points2-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:points2-val is deprecated.  Use custom_msgs-msg:points2 instead.")
  (points2 m))

(cl:ensure-generic-function 'descriptors1-val :lambda-list '(m))
(cl:defmethod descriptors1-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:descriptors1-val is deprecated.  Use custom_msgs-msg:descriptors1 instead.")
  (descriptors1 m))

(cl:ensure-generic-function 'descriptors2-val :lambda-list '(m))
(cl:defmethod descriptors2-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:descriptors2-val is deprecated.  Use custom_msgs-msg:descriptors2 instead.")
  (descriptors2 m))

(cl:ensure-generic-function 'descriptors1_stride-val :lambda-list '(m))
(cl:defmethod descriptors1_stride-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:descriptors1_stride-val is deprecated.  Use custom_msgs-msg:descriptors1_stride instead.")
  (descriptors1_stride m))

(cl:ensure-generic-function 'descriptors2_stride-val :lambda-list '(m))
(cl:defmethod descriptors2_stride-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:descriptors2_stride-val is deprecated.  Use custom_msgs-msg:descriptors2_stride instead.")
  (descriptors2_stride m))

(cl:ensure-generic-function 'descriptors1_matcher-val :lambda-list '(m))
(cl:defmethod descriptors1_matcher-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:descriptors1_matcher-val is deprecated.  Use custom_msgs-msg:descriptors1_matcher instead.")
  (descriptors1_matcher m))

(cl:ensure-generic-function 'descriptors2_matcher-val :lambda-list '(m))
(cl:defmethod descriptors2_matcher-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:descriptors2_matcher-val is deprecated.  Use custom_msgs-msg:descriptors2_matcher instead.")
  (descriptors2_matcher m))

(cl:ensure-generic-function 'image1-val :lambda-list '(m))
(cl:defmethod image1-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:image1-val is deprecated.  Use custom_msgs-msg:image1 instead.")
  (image1 m))

(cl:ensure-generic-function 'image2-val :lambda-list '(m))
(cl:defmethod image2-val ((m <StampedFeaturesMatchesWithImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:image2-val is deprecated.  Use custom_msgs-msg:image2 instead.")
  (image2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StampedFeaturesMatchesWithImage>) ostream)
  "Serializes a message object of type '<StampedFeaturesMatchesWithImage>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'descriptors1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'descriptors1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'descriptors2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'descriptors2))
  (cl:let* ((signed (cl:slot-value msg 'descriptors1_stride)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'descriptors2_stride)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'descriptors1_matcher))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'descriptors1_matcher))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'descriptors2_matcher))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'descriptors2_matcher))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StampedFeaturesMatchesWithImage>) istream)
  "Deserializes a message object of type '<StampedFeaturesMatchesWithImage>"
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
  (cl:setf (cl:slot-value msg 'descriptors1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'descriptors1)))
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
  (cl:setf (cl:slot-value msg 'descriptors2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'descriptors2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'descriptors1_stride) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'descriptors2_stride) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'descriptors1_matcher) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'descriptors1_matcher) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'descriptors2_matcher) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'descriptors2_matcher) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StampedFeaturesMatchesWithImage>)))
  "Returns string type for a message object of type '<StampedFeaturesMatchesWithImage>"
  "custom_msgs/StampedFeaturesMatchesWithImage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StampedFeaturesMatchesWithImage)))
  "Returns string type for a message object of type 'StampedFeaturesMatchesWithImage"
  "custom_msgs/StampedFeaturesMatchesWithImage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StampedFeaturesMatchesWithImage>)))
  "Returns md5sum for a message object of type '<StampedFeaturesMatchesWithImage>"
  "7f36f3de80a10b6c768dff20c34e5294")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StampedFeaturesMatchesWithImage)))
  "Returns md5sum for a message object of type 'StampedFeaturesMatchesWithImage"
  "7f36f3de80a10b6c768dff20c34e5294")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StampedFeaturesMatchesWithImage>)))
  "Returns full string definition for message of type '<StampedFeaturesMatchesWithImage>"
  (cl:format cl:nil "Header header1~%Header header2~%float32[] points1~%float32[] points2~%float32[] descriptors1~%float32[] descriptors2~%int16 descriptors1_stride~%int16 descriptors2_stride~%string descriptors1_matcher~%string descriptors2_matcher~%sensor_msgs/Image image1~%sensor_msgs/Image image2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StampedFeaturesMatchesWithImage)))
  "Returns full string definition for message of type 'StampedFeaturesMatchesWithImage"
  (cl:format cl:nil "Header header1~%Header header2~%float32[] points1~%float32[] points2~%float32[] descriptors1~%float32[] descriptors2~%int16 descriptors1_stride~%int16 descriptors2_stride~%string descriptors1_matcher~%string descriptors2_matcher~%sensor_msgs/Image image1~%sensor_msgs/Image image2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StampedFeaturesMatchesWithImage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header2))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'descriptors1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'descriptors2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
     2
     4 (cl:length (cl:slot-value msg 'descriptors1_matcher))
     4 (cl:length (cl:slot-value msg 'descriptors2_matcher))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StampedFeaturesMatchesWithImage>))
  "Converts a ROS message object to a list"
  (cl:list 'StampedFeaturesMatchesWithImage
    (cl:cons ':header1 (header1 msg))
    (cl:cons ':header2 (header2 msg))
    (cl:cons ':points1 (points1 msg))
    (cl:cons ':points2 (points2 msg))
    (cl:cons ':descriptors1 (descriptors1 msg))
    (cl:cons ':descriptors2 (descriptors2 msg))
    (cl:cons ':descriptors1_stride (descriptors1_stride msg))
    (cl:cons ':descriptors2_stride (descriptors2_stride msg))
    (cl:cons ':descriptors1_matcher (descriptors1_matcher msg))
    (cl:cons ':descriptors2_matcher (descriptors2_matcher msg))
    (cl:cons ':image1 (image1 msg))
    (cl:cons ':image2 (image2 msg))
))
