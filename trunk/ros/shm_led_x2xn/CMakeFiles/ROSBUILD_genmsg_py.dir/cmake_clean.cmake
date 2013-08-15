FILE(REMOVE_RECURSE
  "msg_gen"
  "src/shm_led_x2xn/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/shm_led_x2xn/msg/__init__.py"
  "src/shm_led_x2xn/msg/_LEDX2XNCmd.py"
  "src/shm_led_x2xn/msg/_LEDX2XNRGB.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
