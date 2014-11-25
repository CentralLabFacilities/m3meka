FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/meka_ik/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/meka_ik/srv/__init__.py"
  "../src/meka_ik/srv/_MekaIK.py"
  "../src/meka_ik/srv/_MekaFK.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
