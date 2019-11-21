; Auto-generated. Do not edit!


(cl:in-package multiple_machines-msg)


;//! \htmlinclude ProtocolMessage.msg.html

(cl:defclass <ProtocolMessage> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass ProtocolMessage (<ProtocolMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProtocolMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProtocolMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multiple_machines-msg:<ProtocolMessage> is deprecated: use multiple_machines-msg:ProtocolMessage instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <ProtocolMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multiple_machines-msg:flag-val is deprecated.  Use multiple_machines-msg:flag instead.")
  (flag m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ProtocolMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multiple_machines-msg:value-val is deprecated.  Use multiple_machines-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProtocolMessage>) ostream)
  "Serializes a message object of type '<ProtocolMessage>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'flag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'flag))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProtocolMessage>) istream)
  "Deserializes a message object of type '<ProtocolMessage>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flag) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'flag) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProtocolMessage>)))
  "Returns string type for a message object of type '<ProtocolMessage>"
  "multiple_machines/ProtocolMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProtocolMessage)))
  "Returns string type for a message object of type 'ProtocolMessage"
  "multiple_machines/ProtocolMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProtocolMessage>)))
  "Returns md5sum for a message object of type '<ProtocolMessage>"
  "5b6905edf78d9494d3df19c48997fa57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProtocolMessage)))
  "Returns md5sum for a message object of type 'ProtocolMessage"
  "5b6905edf78d9494d3df19c48997fa57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProtocolMessage>)))
  "Returns full string definition for message of type '<ProtocolMessage>"
  (cl:format cl:nil "string flag~%uint32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProtocolMessage)))
  "Returns full string definition for message of type 'ProtocolMessage"
  (cl:format cl:nil "string flag~%uint32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProtocolMessage>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'flag))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProtocolMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'ProtocolMessage
    (cl:cons ':flag (flag msg))
    (cl:cons ':value (value msg))
))
