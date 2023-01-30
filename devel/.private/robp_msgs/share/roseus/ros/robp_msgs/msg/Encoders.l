;; Auto-generated. Do not edit!


(when (boundp 'robp_msgs::Encoders)
  (if (not (find-package "ROBP_MSGS"))
    (make-package "ROBP_MSGS"))
  (shadow 'Encoders (find-package "ROBP_MSGS")))
(unless (find-package "ROBP_MSGS::ENCODERS")
  (make-package "ROBP_MSGS::ENCODERS"))

(in-package "ROS")
;;//! \htmlinclude Encoders.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robp_msgs::Encoders
  :super ros::object
  :slots (_header _encoder_left _encoder_right _delta_encoder_left _delta_encoder_right _delta_time_left _delta_time_right ))

(defmethod robp_msgs::Encoders
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:encoder_left __encoder_left) 0)
    ((:encoder_right __encoder_right) 0)
    ((:delta_encoder_left __delta_encoder_left) 0)
    ((:delta_encoder_right __delta_encoder_right) 0)
    ((:delta_time_left __delta_time_left) 0.0)
    ((:delta_time_right __delta_time_right) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _encoder_left (round __encoder_left))
   (setq _encoder_right (round __encoder_right))
   (setq _delta_encoder_left (round __delta_encoder_left))
   (setq _delta_encoder_right (round __delta_encoder_right))
   (setq _delta_time_left (float __delta_time_left))
   (setq _delta_time_right (float __delta_time_right))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:encoder_left
   (&optional __encoder_left)
   (if __encoder_left (setq _encoder_left __encoder_left)) _encoder_left)
  (:encoder_right
   (&optional __encoder_right)
   (if __encoder_right (setq _encoder_right __encoder_right)) _encoder_right)
  (:delta_encoder_left
   (&optional __delta_encoder_left)
   (if __delta_encoder_left (setq _delta_encoder_left __delta_encoder_left)) _delta_encoder_left)
  (:delta_encoder_right
   (&optional __delta_encoder_right)
   (if __delta_encoder_right (setq _delta_encoder_right __delta_encoder_right)) _delta_encoder_right)
  (:delta_time_left
   (&optional __delta_time_left)
   (if __delta_time_left (setq _delta_time_left __delta_time_left)) _delta_time_left)
  (:delta_time_right
   (&optional __delta_time_right)
   (if __delta_time_right (setq _delta_time_right __delta_time_right)) _delta_time_right)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _encoder_left
    8
    ;; int64 _encoder_right
    8
    ;; int32 _delta_encoder_left
    4
    ;; int32 _delta_encoder_right
    4
    ;; float64 _delta_time_left
    8
    ;; float64 _delta_time_right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _encoder_left
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _encoder_left (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _encoder_left) (= (length (_encoder_left . bv)) 2)) ;; bignum
              (write-long (ash (elt (_encoder_left . bv) 0) 0) s)
              (write-long (ash (elt (_encoder_left . bv) 1) -1) s))
             ((and (class _encoder_left) (= (length (_encoder_left . bv)) 1)) ;; big1
              (write-long (elt (_encoder_left . bv) 0) s)
              (write-long (if (>= _encoder_left 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _encoder_left s)(write-long (if (>= _encoder_left 0) 0 #xffffffff) s)))
     ;; int64 _encoder_right
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _encoder_right (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _encoder_right) (= (length (_encoder_right . bv)) 2)) ;; bignum
              (write-long (ash (elt (_encoder_right . bv) 0) 0) s)
              (write-long (ash (elt (_encoder_right . bv) 1) -1) s))
             ((and (class _encoder_right) (= (length (_encoder_right . bv)) 1)) ;; big1
              (write-long (elt (_encoder_right . bv) 0) s)
              (write-long (if (>= _encoder_right 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _encoder_right s)(write-long (if (>= _encoder_right 0) 0 #xffffffff) s)))
     ;; int32 _delta_encoder_left
       (write-long _delta_encoder_left s)
     ;; int32 _delta_encoder_right
       (write-long _delta_encoder_right s)
     ;; float64 _delta_time_left
       (sys::poke _delta_time_left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_time_right
       (sys::poke _delta_time_right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _encoder_left
#+(or :alpha :irix6 :x86_64)
      (setf _encoder_left (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _encoder_left (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _encoder_right
#+(or :alpha :irix6 :x86_64)
      (setf _encoder_right (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _encoder_right (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int32 _delta_encoder_left
     (setq _delta_encoder_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _delta_encoder_right
     (setq _delta_encoder_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _delta_time_left
     (setq _delta_time_left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_time_right
     (setq _delta_time_right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robp_msgs::Encoders :md5sum-) "190037ad3ca3cb2954f783e368d7bd4a")
(setf (get robp_msgs::Encoders :datatype-) "robp_msgs/Encoders")
(setf (get robp_msgs::Encoders :definition-)
      "Header header

# Total number of ticks
int64 encoder_left
int64 encoder_right
# The number of ticks since the last reading
int32 delta_encoder_left
int32 delta_encoder_right
# The time elapsed since the last reading in milliseconds
float64 delta_time_left
float64 delta_time_right
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



(provide :robp_msgs/Encoders "190037ad3ca3cb2954f783e368d7bd4a")


