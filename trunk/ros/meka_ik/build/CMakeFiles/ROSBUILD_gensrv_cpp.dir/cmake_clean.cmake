FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/meka_ik/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/meka_ik/MekaIK.h"
  "../srv_gen/cpp/include/meka_ik/MekaFK.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
