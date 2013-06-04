FILE(REMOVE_RECURSE
  "../src/dynamics/msg"
  "../src/dynamics/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/dynamics/srv/__init__.py"
  "../src/dynamics/srv/_FollowerImageServer.py"
  "../src/dynamics/srv/_CamSelect.py"
  "../src/dynamics/srv/_CaptureImageFeatures.py"
  "../src/dynamics/srv/_HullSelect.py"
  "../src/dynamics/srv/_capture_image_features.py"
  "../src/dynamics/srv/_LedAnim.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
