;; Auto-generated. Do not edit!


(when (boundp 'multiple_machines::ProtocolMessage)
  (if (not (find-package "MULTIPLE_MACHINES"))
    (make-package "MULTIPLE_MACHINES"))
  (shadow 'ProtocolMessage (find-package "MULTIPLE_MACHINES")))
(unless (find-package "MULTIPLE_MACHINES::PROTOCOLMESSAGE")
  (make-package "MULTIPLE_MACHINES::PROTOCOLMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude ProtocolMessage.msg.html


(defclass multiple_machines::ProtocolMessage
  :super ros::object
  :slots (_flag _value ))

(defmethod multiple_machines::ProtocolMessage
  (:init
   (&key
    ((:flag __flag) "")
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _flag (string __flag))
   (setq _value (round __value))
   self)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _flag
    4 (length _flag)
    ;; uint32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _flag
       (write-long (length _flag) s) (princ _flag s)
     ;; uint32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _flag
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _flag (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get multiple_machines::ProtocolMessage :md5sum-) "5b6905edf78d9494d3df19c48997fa57")
(setf (get multiple_machines::ProtocolMessage :datatype-) "multiple_machines/ProtocolMessage")
(setf (get multiple_machines::ProtocolMessage :definition-)
      "string flag
uint32 value

")



(provide :multiple_machines/ProtocolMessage "5b6905edf78d9494d3df19c48997fa57")


