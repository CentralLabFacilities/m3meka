
SET(SUBS
toolbox
hardware
chains
inverse_kine
robot_ctrl
robots
shared_mem

vehicles
)

foreach(SUB ${SUBS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/${SUB})
endforeach()

foreach(SUB ${SUBS})
ADD_SUBDIRECTORY(${SUB})
endforeach()

