FILE(REMOVE_RECURSE
  "../src/dynamics/msg"
  "../src/dynamics/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/dynamics/ARMarker.h"
  "../msg_gen/cpp/include/dynamics/Navdata.h"
  "../msg_gen/cpp/include/dynamics/ARMarkers.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
