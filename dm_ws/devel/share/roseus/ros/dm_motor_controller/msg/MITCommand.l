;; Auto-generated. Do not edit!


(when (boundp 'dm_motor_controller::MITCommand)
  (if (not (find-package "DM_MOTOR_CONTROLLER"))
    (make-package "DM_MOTOR_CONTROLLER"))
  (shadow 'MITCommand (find-package "DM_MOTOR_CONTROLLER")))
(unless (find-package "DM_MOTOR_CONTROLLER::MITCOMMAND")
  (make-package "DM_MOTOR_CONTROLLER::MITCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude MITCommand.msg.html


(defclass dm_motor_controller::MITCommand
  :super ros::object
  :slots (_kp _kd _position _velocity _torque ))

(defmethod dm_motor_controller::MITCommand
  (:init
   (&key
    ((:kp __kp) 0.0)
    ((:kd __kd) 0.0)
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:torque __torque) 0.0)
    )
   (send-super :init)
   (setq _kp (float __kp))
   (setq _kd (float __kd))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _torque (float __torque))
   self)
  (:kp
   (&optional __kp)
   (if __kp (setq _kp __kp)) _kp)
  (:kd
   (&optional __kd)
   (if __kd (setq _kd __kd)) _kd)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:torque
   (&optional __torque)
   (if __torque (setq _torque __torque)) _torque)
  (:serialization-length
   ()
   (+
    ;; float64 _kp
    8
    ;; float64 _kd
    8
    ;; float64 _position
    8
    ;; float64 _velocity
    8
    ;; float64 _torque
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _kp
       (sys::poke _kp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _kd
       (sys::poke _kd (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torque
       (sys::poke _torque (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _kp
     (setq _kp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _kd
     (setq _kd (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torque
     (setq _torque (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dm_motor_controller::MITCommand :md5sum-) "8ffcd33c7fb2269670e5e49cb5a27d69")
(setf (get dm_motor_controller::MITCommand :datatype-) "dm_motor_controller/MITCommand")
(setf (get dm_motor_controller::MITCommand :definition-)
      "float64 kp
float64 kd
float64 position
float64 velocity
float64 torque

")



(provide :dm_motor_controller/MITCommand "8ffcd33c7fb2269670e5e49cb5a27d69")


