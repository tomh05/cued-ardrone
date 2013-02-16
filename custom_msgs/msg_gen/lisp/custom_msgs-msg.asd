
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RendererPolyLineTri" :depends-on ("_package_RendererPolyLineTri"))
    (:file "_package_RendererPolyLineTri" :depends-on ("_package"))
    (:file "StampedFrames" :depends-on ("_package_StampedFrames"))
    (:file "_package_StampedFrames" :depends-on ("_package"))
    (:file "StampedMatchesWithImage" :depends-on ("_package_StampedMatchesWithImage"))
    (:file "_package_StampedMatchesWithImage" :depends-on ("_package"))
    (:file "DescribedPointCloud" :depends-on ("_package_DescribedPointCloud"))
    (:file "_package_DescribedPointCloud" :depends-on ("_package"))
    (:file "StampedFeaturesWithImage" :depends-on ("_package_StampedFeaturesWithImage"))
    (:file "_package_StampedFeaturesWithImage" :depends-on ("_package"))
    (:file "StampedStampedInt" :depends-on ("_package_StampedStampedInt"))
    (:file "_package_StampedStampedInt" :depends-on ("_package"))
  ))