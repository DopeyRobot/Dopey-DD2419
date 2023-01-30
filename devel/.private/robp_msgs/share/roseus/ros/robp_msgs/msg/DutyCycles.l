;; Auto-generated. Do not edit!


(when (boundp 'robp_msgs::DutyCycles)
  (if (not (find-package "ROBP_MSGS"))
    (make-package "ROBP_MSGS"))
  (shadow 'DutyCycles (find-package "ROBP_MSGS")))
(unless (find-package "ROBP_MSGS::DUTYCYCLES")
  (make-package "ROBP_MSGS::DUTYCYCLES"))

(in-package "ROS")
;;//! \htmlinclude DutyCycles.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robp_msgs::DutyCycles
  :super ros::object
  :slots (_header _duty_cycle_left _duty_cycle_right ))

(defmethod robp_msgs::DutyCycles
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:duty_cycle_left __duty_cycle_left) 0.0)
    ((:duty_cycle_right __duty_cycle_right) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _duty_cycle_left (float __duty_cycle_left))
   (setq _duty_cycle_right (float __duty_cycle_right))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:duty_cycle_left
   (&optional __duty_cycle_left)
   (if __duty_cycle_left (setq _duty_cycle_left __duty_cycle_left)) _duty_cycle_left)
  (:duty_cycle_right
   (&optional __duty_cycle_right)
   (if __duty_cycle_right (setq _duty_cycle_right __duty_cycle_right)) _duty_cycle_right)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _duty_cycle_left
    8
    ;; float64 _duty_cycle_right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _duty_cycle_left
       (sys::poke _duty_cycle_left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _duty_cycle_right
       (sys::poke _duty_cycle_right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _duty_cycle_left
     (setq _duty_cycle_left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _duty_cycle_right
     (setq _duty_cycle_right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robp_msgs::DutyCycles :md5sum-) "543f5f160fe8968f328a223a093313c7")
(setf (get robp_msgs::DutyCycles :datatype-) "robp_msgs/DutyCycles")
(setf (get robp_msgs::DutyCycles :definition-)
      "Header header

# Value should be in [-1, 1], negative is backwards, positive forwards
float64 duty_cycle_left
float64 duty_cycle_right
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :robp_msgs/DutyCycles "543f5f160fe8968f328a223a093313c7")


