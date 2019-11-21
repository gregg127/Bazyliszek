
(cl:in-package :asdf)

(defsystem "multiple_machines-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorsPwm" :depends-on ("_package_MotorsPwm"))
    (:file "_package_MotorsPwm" :depends-on ("_package"))
    (:file "ProtocolMessage" :depends-on ("_package_ProtocolMessage"))
    (:file "_package_ProtocolMessage" :depends-on ("_package"))
  ))