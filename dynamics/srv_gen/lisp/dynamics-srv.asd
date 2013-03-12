
(cl:in-package :asdf)

(defsystem "dynamics-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CamSelect" :depends-on ("_package_CamSelect"))
    (:file "_package_CamSelect" :depends-on ("_package"))
    (:file "HullSelect" :depends-on ("_package_HullSelect"))
    (:file "_package_HullSelect" :depends-on ("_package"))
    (:file "capture_image_features" :depends-on ("_package_capture_image_features"))
    (:file "_package_capture_image_features" :depends-on ("_package"))
    (:file "LedAnim" :depends-on ("_package_LedAnim"))
    (:file "_package_LedAnim" :depends-on ("_package"))
  ))