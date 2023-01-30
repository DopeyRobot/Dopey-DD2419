
(cl:in-package :asdf)

(defsystem "robp_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DutyCycles" :depends-on ("_package_DutyCycles"))
    (:file "_package_DutyCycles" :depends-on ("_package"))
    (:file "Encoders" :depends-on ("_package_Encoders"))
    (:file "_package_Encoders" :depends-on ("_package"))
  ))