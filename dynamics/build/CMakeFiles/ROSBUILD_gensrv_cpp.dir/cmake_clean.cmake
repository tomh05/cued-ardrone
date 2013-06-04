FILE(REMOVE_RECURSE
  "../src/dynamics/msg"
  "../src/dynamics/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/dynamics/FollowerImageServer.h"
  "../srv_gen/cpp/include/dynamics/CamSelect.h"
  "../srv_gen/cpp/include/dynamics/CaptureImageFeatures.h"
  "../srv_gen/cpp/include/dynamics/HullSelect.h"
  "../srv_gen/cpp/include/dynamics/capture_image_features.h"
  "../srv_gen/cpp/include/dynamics/LedAnim.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
