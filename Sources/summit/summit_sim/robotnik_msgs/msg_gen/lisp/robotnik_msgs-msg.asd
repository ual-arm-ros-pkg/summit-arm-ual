
(cl:in-package :asdf)

(defsystem "robotnik_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ptz" :depends-on ("_package_ptz"))
    (:file "_package_ptz" :depends-on ("_package"))
    (:file "inputs_outputs" :depends-on ("_package_inputs_outputs"))
    (:file "_package_inputs_outputs" :depends-on ("_package"))
  ))