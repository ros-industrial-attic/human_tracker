
(cl:in-package :asdf)

(defsystem "Acuity-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LaserRange" :depends-on ("_package_LaserRange"))
    (:file "_package_LaserRange" :depends-on ("_package"))
  ))