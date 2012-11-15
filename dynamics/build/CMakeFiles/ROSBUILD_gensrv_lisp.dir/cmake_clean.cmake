FILE(REMOVE_RECURSE
  "../src/dynamics/msg"
  "../src/dynamics/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/CamSelect.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_CamSelect.lisp"
  "../srv_gen/lisp/HullSelect.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_HullSelect.lisp"
  "../srv_gen/lisp/LedAnim.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_LedAnim.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
