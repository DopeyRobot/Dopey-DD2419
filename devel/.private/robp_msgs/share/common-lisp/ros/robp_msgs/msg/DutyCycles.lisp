; Auto-generated. Do not edit!


(cl:in-package robp_msgs-msg)


;//! \htmlinclude DutyCycles.msg.html

(cl:defclass <DutyCycles> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (duty_cycle_left
    :reader duty_cycle_left
    :initarg :duty_cycle_left
    :type cl:float
    :initform 0.0)
   (duty_cycle_right
    :reader duty_cycle_right
    :initarg :duty_cycle_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass DutyCycles (<DutyCycles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DutyCycles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DutyCycles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robp_msgs-msg:<DutyCycles> is deprecated: use robp_msgs-msg:DutyCycles instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DutyCycles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robp_msgs-msg:header-val is deprecated.  Use robp_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'duty_cycle_left-val :lambda-list '(m))
(cl:defmethod duty_cycle_left-val ((m <DutyCycles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robp_msgs-msg:duty_cycle_left-val is deprecated.  Use robp_msgs-msg:duty_cycle_left instead.")
  (duty_cycle_left m))

(cl:ensure-generic-function 'duty_cycle_right-val :lambda-list '(m))
(cl:defmethod duty_cycle_right-val ((m <DutyCycles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robp_msgs-msg:duty_cycle_right-val is deprecated.  Use robp_msgs-msg:duty_cycle_right instead.")
  (duty_cycle_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DutyCycles>) ostream)
  "Serializes a message object of type '<DutyCycles>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'duty_cycle_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'duty_cycle_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DutyCycles>) istream)
  "Deserializes a message object of type '<DutyCycles>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duty_cycle_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duty_cycle_right) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DutyCycles>)))
  "Returns string type for a message object of type '<DutyCycles>"
  "robp_msgs/DutyCycles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DutyCycles)))
  "Returns string type for a message object of type 'DutyCycles"
  "robp_msgs/DutyCycles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DutyCycles>)))
  "Returns md5sum for a message object of type '<DutyCycles>"
  "543f5f160fe8968f328a223a093313c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DutyCycles)))
  "Returns md5sum for a message object of type 'DutyCycles"
  "543f5f160fe8968f328a223a093313c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DutyCycles>)))
  "Returns full string definition for message of type '<DutyCycles>"
  (cl:format cl:nil "Header header~%~%# Value should be in [-1, 1], negative is backwards, positive forwards~%float64 duty_cycle_left~%float64 duty_cycle_right~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DutyCycles)))
  "Returns full string definition for message of type 'DutyCycles"
  (cl:format cl:nil "Header header~%~%# Value should be in [-1, 1], negative is backwards, positive forwards~%float64 duty_cycle_left~%float64 duty_cycle_right~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DutyCycles>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DutyCycles>))
  "Converts a ROS message object to a list"
  (cl:list 'DutyCycles
    (cl:cons ':header (header msg))
    (cl:cons ':duty_cycle_left (duty_cycle_left msg))
    (cl:cons ':duty_cycle_right (duty_cycle_right msg))
))
