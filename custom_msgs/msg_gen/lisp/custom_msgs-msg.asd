
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "StampedMatchesWithImages" :depends-on ("_package_StampedMatchesWithImages"))
    (:file "_package_StampedMatchesWithImages" :depends-on ("_package"))
  ))