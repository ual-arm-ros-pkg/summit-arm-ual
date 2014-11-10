
(cl:in-package :asdf)

(defsystem "robotnik_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "set_analog_output" :depends-on ("_package_set_analog_output"))
    (:file "_package_set_analog_output" :depends-on ("_package"))
    (:file "set_digital_output" :depends-on ("_package_set_digital_output"))
    (:file "_package_set_digital_output" :depends-on ("_package"))
    (:file "get_mode" :depends-on ("_package_get_mode"))
    (:file "_package_get_mode" :depends-on ("_package"))
    (:file "set_odometry" :depends-on ("_package_set_odometry"))
    (:file "_package_set_odometry" :depends-on ("_package"))
    (:file "set_ptz" :depends-on ("_package_set_ptz"))
    (:file "_package_set_ptz" :depends-on ("_package"))
    (:file "set_mode" :depends-on ("_package_set_mode"))
    (:file "_package_set_mode" :depends-on ("_package"))
  ))