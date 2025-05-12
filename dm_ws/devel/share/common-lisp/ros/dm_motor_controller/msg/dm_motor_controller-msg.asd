
(cl:in-package :asdf)

(defsystem "dm_motor_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MITCommand" :depends-on ("_package_MITCommand"))
    (:file "_package_MITCommand" :depends-on ("_package"))
  ))