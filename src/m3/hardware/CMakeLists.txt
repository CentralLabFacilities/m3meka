cmake_minimum_required(VERSION 2.8)
project(hardware)
set(LIBNAME "m3hardware")


set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(RTAI REQUIRED)
#find_package(M3 COMPONENTS HARDWARE ROBOTS CHAINS TOOLBOX SHARED_MEM REQUIRED)
find_package(M3rt REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 3.2 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OROCOS-KDL REQUIRED)
SET(M3_INCLUDE_DIR 
	${CMAKE_CURRENT_SOURCE_DIR}/../..
	${CMAKE_CURRENT_SOURCE_DIR}/../chains
	${CMAKE_CURRENT_SOURCE_DIR}/../toolbox
)

include_directories(${M3RT_INCLUDE_DIR} ${RTAI_INCLUDE_DIR} ${THREADS_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIR} ${M3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${KDL_INCLUDE_DIRS})


# Start Protobuf stuff
## Cpp protobufs
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
SET(PROTOBUF_IMPORT_DIRS ${M3RT_INCLUDE_DIR} ${M3_INCLUDE_DIR})
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_SOURCE_DIR} ${ProtoFiles})
set_source_files_properties(${ProtoSources} ${ProtoHeaders} PROPERTIES
                            GENERATED TRUE)
add_custom_target(${LIBNAME}_protobuf_tmp  ALL DEPENDS ${ProtoSources}) # Force the re-generation
## Python protobufs
find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp REQUIRED) 
## Generate and copy the python protobuf
PROTOBUF_GENERATE_PYTHON(PySources ${CMAKE_CURRENT_BINARY_DIR} ${ProtoFiles}) 
add_custom_target(${LIBNAME}_python_protobuf_output ALL DEPENDS ${PySources}) # force the generation
# End Protobuf stuff

SET(LIBS ${RTAI_LIBRARIES}  pthread ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} m3toolbox orocos-kdl m3base)


set( ALL_SRCS 
actuator.cpp
actuator_ec.cpp
actuator_virtual.cpp
async_io.cpp
ctrl_simple.cpp
factory_proxy.cpp
joint.cpp
joint_slave.cpp
joint_zlift.cpp
joint_zlift_shm.cpp
led_matrix_ec.cpp
led_matrix_ec_shm.cpp
ledx2_ec.cpp
ledx2xn_ec.cpp
ledx2xn_ec_shm.cpp
loadx1.cpp
loadx1_ec.cpp
loadx6.cpp
loadx6_ec.cpp
loadx6_virtual.cpp
log_test.cpp
motor.cpp
pwr.cpp
pwr_ec.cpp
pwr_virtual.cpp
robot_monitor.cpp
sensor.cpp
tactile_pps22_ec.cpp
transmission.cpp

)
set( ALL_HDRS 
actuator_ec.h
actuator.h
actuator_virtual.h
async_io.h
ctrl_simple.h
joint.h
joint_slave.h
joint_zlift.h
joint_zlift_shm.h
joint_zlift_shm_sds.h
led_matrix_ec.h
led_matrix_ec_shm.h
led_matrix_ec_shm_sds.h
ledx2_ec.h
ledx2xn_ec.h
ledx2xn_ec_shm.h
ledx2xn_ec_shm_sds.h
loadx1_ec.h
loadx1.h
loadx6_ec.h
loadx6.h
loadx6_virtual.h
log_test.h
m3ec_def.h
m3ec_pdo_v0_def.h
m3ec_pdo_v1_def.h
m3ec_pdo_v2_def.h
m3ec_pdo_v3_def.h
m3ec_pdo_v4_def.h
m3meka_ec_def.h
motor.h
pwr_ec.h
pwr.h
pwr_virtual.h
robot_monitor.h
sensor.h
tactile_pps22_ec.h
transmission.h

)
list(APPEND ALL_SRCS ${ProtoSources})

list(APPEND ALL_HDRS
${ProtoHeaders}
${ProtoFiles}
)

## Building
add_library(${LIBNAME} SHARED ${ALL_SRCS})
target_link_libraries(${LIBNAME} ${LIBS})
set_target_properties(${LIBNAME} PROPERTIES COMPILE_FLAGS "-I${CMAKE_BINARY_DIR}/src")

## Installing
install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${ALL_HDRS} DESTINATION include/m3/${PROJECT_NAME})


execute_process ( 
   COMMAND ${PYTHON_EXECUTABLE} -c 
   	"import site, sys; sys.stdout.write(site.PREFIXES[-1])" 
   OUTPUT_VARIABLE PYTHON_PREFIX 
) 
file ( TO_CMAKE_PATH "${PYTHON_PREFIX}" PYTHON_PREFIX ) 
execute_process ( 
   COMMAND ${PYTHON_EXECUTABLE} -c 
   	"import site, sys; sys.stdout.write(site.getsitepackages()[-1])" 
   OUTPUT_VARIABLE PYTHON_SITE_DIR 
) 
file ( TO_CMAKE_PATH "${PYTHON_SITE_DIR}" PYTHON_SITE_DIR ) 
string ( REGEX REPLACE "^${PYTHON_PREFIX}/" "" 
   PYTHON_SITE_DIR "${PYTHON_SITE_DIR}" 
) 


install ( FILES ${PySources} 
   DESTINATION ${PYTHON_SITE_DIR}/m3/
   COMPONENT library 
) 


