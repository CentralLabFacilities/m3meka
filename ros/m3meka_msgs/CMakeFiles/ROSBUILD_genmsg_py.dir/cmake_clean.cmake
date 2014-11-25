FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/m3meka_msgs/msg"
  "src/m3meka_msgs/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/m3meka_msgs/msg/__init__.py"
  "src/m3meka_msgs/msg/_M3OmnibaseJoy.py"
  "src/m3meka_msgs/msg/_M3JointStatus.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
