FILE(REMOVE_RECURSE
  "msg_gen"
  "src/shm_led_mouth/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/shm_led_mouth/msg/__init__.py"
  "src/shm_led_mouth/msg/_LEDMatrixRGB.py"
  "src/shm_led_mouth/msg/_LEDMatrixRow.py"
  "src/shm_led_mouth/msg/_LEDMatrixCmd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
