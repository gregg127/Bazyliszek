; Auto-generated. Do not edit!


(cl:in-package multiple_machines-msg)


;//! \htmlinclude MotorsPwm.msg.html

(cl:defclass <MotorsPwm> (roslisp-msg-protocol:ros-message)
  ((left_motor_pwm
    :reader left_motor_pwm
    :initarg :left_motor_pwm
    :type cl:integer
    :initform 0)
   (right_motor_pwm
    :reader right_motor_pwm
    :initarg :right_motor_pwm
    :type cl:integer
    :initform 0))
)

(cl:defclass MotorsPwm (<MotorsPwm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorsPwm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorsPwm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multiple_machines-msg:<MotorsPwm> is deprecated: use multiple_machines-msg:MotorsPwm instead.")))

(cl:ensure-generic-function 'left_motor_pwm-val :lambda-list '(m))
(cl:defmethod left_motor_pwm-val ((m <MotorsPwm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multiple_machines-msg:left_motor_pwm-val is deprecated.  Use multiple_machines-msg:left_motor_pwm instead.")
  (left_motor_pwm m))

(cl:ensure-generic-function 'right_motor_pwm-val :lambda-list '(m))
(cl:defmethod right_motor_pwm-val ((m <MotorsPwm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multiple_machines-msg:right_motor_pwm-val is deprecated.  Use multiple_machines-msg:right_motor_pwm instead.")
  (right_motor_pwm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorsPwm>) ostream)
  "Serializes a message object of type '<MotorsPwm>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_motor_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'left_motor_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'left_motor_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'left_motor_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_motor_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'right_motor_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'right_motor_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'right_motor_pwm)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorsPwm>) istream)
  "Deserializes a message object of type '<MotorsPwm>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_motor_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'left_motor_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'left_motor_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'left_motor_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_motor_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'right_motor_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'right_motor_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'right_motor_pwm)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorsPwm>)))
  "Returns string type for a message object of type '<MotorsPwm>"
  "multiple_machines/MotorsPwm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorsPwm)))
  "Returns string type for a message object of type 'MotorsPwm"
  "multiple_machines/MotorsPwm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorsPwm>)))
  "Returns md5sum for a message object of type '<MotorsPwm>"
  "cae0766230de6a370bc83970582495e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorsPwm)))
  "Returns md5sum for a message object of type 'MotorsPwm"
  "cae0766230de6a370bc83970582495e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorsPwm>)))
  "Returns full string definition for message of type '<MotorsPwm>"
  (cl:format cl:nil "uint32 left_motor_pwm~%uint32 right_motor_pwm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorsPwm)))
  "Returns full string definition for message of type 'MotorsPwm"
  (cl:format cl:nil "uint32 left_motor_pwm~%uint32 right_motor_pwm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorsPwm>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorsPwm>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorsPwm
    (cl:cons ':left_motor_pwm (left_motor_pwm msg))
    (cl:cons ':right_motor_pwm (right_motor_pwm msg))
))
