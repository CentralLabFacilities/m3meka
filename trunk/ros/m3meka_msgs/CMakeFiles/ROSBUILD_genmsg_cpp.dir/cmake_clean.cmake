FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/m3meka_msgs/msg"
  "src/m3meka_msgs/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/m3meka_msgs/M3OmnibaseJoy.h"
  "msg_gen/cpp/include/m3meka_msgs/M3JointStatus.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
