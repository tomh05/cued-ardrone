; Auto-generated. Do not edit!


(cl:in-package dynamics-srv)


;//! \htmlinclude HullSelect-request.msg.html

(cl:defclass <HullSelect-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HullSelect-request (<HullSelect-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HullSelect-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HullSelect-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamics-srv:<HullSelect-request> is deprecated: use dynamics-srv:HullSelect-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <HullSelect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:type-val is deprecated.  Use dynamics-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HullSelect-request>) ostream)
  "Serializes a message object of type '<HullSelect-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HullSelect-request>) istream)
  "Deserializes a message object of type '<HullSelect-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HullSelect-request>)))
  "Returns string type for a service object of type '<HullSelect-request>"
  "dynamics/HullSelectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HullSelect-request)))
  "Returns string type for a service object of type 'HullSelect-request"
  "dynamics/HullSelectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HullSelect-request>)))
  "Returns md5sum for a message object of type '<HullSelect-request>"
  "3f39d970b40bc0b36aa2136e89b31962")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HullSelect-request)))
  "Returns md5sum for a message object of type 'HullSelect-request"
  "3f39d970b40bc0b36aa2136e89b31962")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HullSelect-request>)))
  "Returns full string definition for message of type '<HullSelect-request>"
  (cl:format cl:nil "uint8 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HullSelect-request)))
  "Returns full string definition for message of type 'HullSelect-request"
  (cl:format cl:nil "uint8 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HullSelect-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HullSelect-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HullSelect-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude HullSelect-response.msg.html

(cl:defclass <HullSelect-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass HullSelect-response (<HullSelect-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HullSelect-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HullSelect-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamics-srv:<HullSelect-response> is deprecated: use dynamics-srv:HullSelect-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <HullSelect-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamics-srv:result-val is deprecated.  Use dynamics-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HullSelect-response>) ostream)
  "Serializes a message object of type '<HullSelect-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HullSelect-response>) istream)
  "Deserializes a message object of type '<HullSelect-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HullSelect-response>)))
  "Returns string type for a service object of type '<HullSelect-response>"
  "dynamics/HullSelectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HullSelect-response)))
  "Returns string type for a service object of type 'HullSelect-response"
  "dynamics/HullSelectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HullSelect-response>)))
  "Returns md5sum for a message object of type '<HullSelect-response>"
  "3f39d970b40bc0b36aa2136e89b31962")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HullSelect-response)))
  "Returns md5sum for a message object of type 'HullSelect-response"
  "3f39d970b40bc0b36aa2136e89b31962")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HullSelect-response>)))
  "Returns full string definition for message of type '<HullSelect-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HullSelect-response)))
  "Returns full string definition for message of type 'HullSelect-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HullSelect-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HullSelect-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HullSelect-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HullSelect)))
  'HullSelect-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HullSelect)))
  'HullSelect-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HullSelect)))
  "Returns string type for a service object of type '<HullSelect>"
  "dynamics/HullSelect")