;; Auto-generated. Do not edit!


(when (boundp 'multiple_machines::MotorsPwm)
  (if (not (find-package "MULTIPLE_MACHINES"))
    (make-package "MULTIPLE_MACHINES"))
  (shadow 'MotorsPwm (find-package "MULTIPLE_MACHINES")))
(unless (find-package "MULTIPLE_MACHINES::MOTORSPWM")
  (make-package "MULTIPLE_MACHINES::MOTORSPWM"))

(in-package "ROS")
;;//! \htmlinclude MotorsPwm.msg.html


(defclass multiple_machines::MotorsPwm
  :super ros::object
  :slots (_left_motor_pwm _right_motor_pwm ))

(defmethod multiple_machines::MotorsPwm
  (:init
   (&key
    ((:left_motor_pwm __left_motor_pwm) 0)
    ((:right_motor_pwm __right_motor_pwm) 0)
    )
   (send-super :init)
   (setq _left_motor_pwm (round __left_motor_pwm))
   (setq _right_motor_pwm (round __right_motor_pwm))
   self)
  (:left_motor_pwm
   (&optional __left_motor_pwm)
   (if __left_motor_pwm (setq _left_motor_pwm __left_motor_pwm)) _left_motor_pwm)
  (:right_motor_pwm
   (&optional __right_motor_pwm)
   (if __right_motor_pwm (setq _right_motor_pwm __right_motor_pwm)) _right_motor_pwm)
  (:serialization-length
   ()
   (+
    ;; uint32 _left_motor_pwm
    4
    ;; uint32 _right_motor_pwm
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _left_motor_pwm
       (write-long _left_motor_pwm s)
     ;; uint32 _right_motor_pwm
       (write-long _right_motor_pwm s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _left_motor_pwm
     (setq _left_motor_pwm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _right_motor_pwm
     (setq _right_motor_pwm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get multiple_machines::MotorsPwm :md5sum-) "cae0766230de6a370bc83970582495e0")
(setf (get multiple_machines::MotorsPwm :datatype-) "multiple_machines/MotorsPwm")
(setf (get multiple_machines::MotorsPwm :definition-)
      "uint32 left_motor_pwm
uint32 right_motor_pwm

")



(provide :multiple_machines/MotorsPwm "cae0766230de6a370bc83970582495e0")


