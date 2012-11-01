FILE(REMOVE_RECURSE
  "../src/dynamics/msg"
  "../src/dynamics/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/dynamics/msg/__init__.py"
  "../src/dynamics/msg/_ARMarker.py"
  "../src/dynamics/msg/_Navdata.py"
  "../src/dynamics/msg/_ARMarkers.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
