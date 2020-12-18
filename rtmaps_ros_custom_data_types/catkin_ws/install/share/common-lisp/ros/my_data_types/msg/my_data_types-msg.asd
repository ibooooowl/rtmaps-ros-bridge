
(cl:in-package :asdf)

(defsystem "my_data_types-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "my_data_type" :depends-on ("_package_my_data_type"))
    (:file "_package_my_data_type" :depends-on ("_package"))
  ))