cmake_minimum_required(VERSION 2.8)
project(vehicles)
set(LIBNAME "m3vehicles")

find_package(M3 COMPONENTS HARDWARE CHAINS TOOLBOX)
find_package(M3rt)
include_directories(${M3RT_INCLUDE_DIR} ${M3_INCLUDE_DIR})

find_package(Holomni)
if(HOLOMNI_FOUND)
	include_directories(${HOLOMNI_INCLUDE_DIR})
	set(LIBS ${LIBS} ${HOLOMNI_LIBRARY})
  message(STATUS "Found Holomni")
else()
message(WARNING "Holomni library not found, please install it first if you want to control the base: \n cd ~/mekabot/holmni_pcv && mkdir build;cd build && cmake .. && make && sudo make install")
return()
endif()


## First generate the python 
find_package(Protobuf REQUIRED)
SET(PROTOBUF_IMPORT_DIRS ${PROTOBUF_IMPORT_DIRS} ${M3RT_INCLUDE_DIR} ${M3_INCLUDE_DIR})

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
## Python protobufs
find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp REQUIRED) 
## Generate and copy the python protobuf
PROTOBUF_GENERATE_PYTHON(PySources ${CMAKE_CURRENT_BINARY_DIR} ${ProtoFiles}) 
add_custom_target(${LIBNAME}_python_protobuf_output ALL DEPENDS ${PySources}) # force the generation
# End Protobuf stuff


## YAML CPP 
find_package(PkgConfig REQUIRED)
#find_package(YamlCpp  REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
if(${YAMLCPP_VERSION} VERSION_LESS "0.5")
set_property(
   DIRECTORY
   PROPERTY COMPILE_DEFINITIONS YAMLCPP_03
   )
endif(${YAMLCPP_VERSION} VERSION_LESS "0.5")
set(LIBS ${LIBS} ${YAMLCPP_LIBRARY})

if(RTAI)
find_package(RTAI REQUIRED)
add_definitions(-D__RTAI__)
find_package(Boost REQUIRED)
find_package(Eigen3 3 REQUIRED)

#SET(M3_INCLUDE_DIR 
#	${CMAKE_CURRENT_SOURCE_DIR}/../..
#	${CMAKE_CURRENT_SOURCE_DIR}/../chains
#	${CMAKE_CURRENT_SOURCE_DIR}/../toolbox
#	${CMAKE_CURRENT_SOURCE_DIR}/../hardware
#)

include_directories(${M3RT_INCLUDE_DIR} ${RTAI_INCLUDE_DIR} ${THREADS_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIR} ${M3_INCLUDE_DIR})


# Start Protobuf stuff
## Cpp protobufs
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_BINARY_DIR} ${ProtoFiles})
set_source_files_properties(${ProtoSources} ${ProtoHeaders} PROPERTIES
                            GENERATED TRUE)
add_custom_target(${LIBNAME}_protobuf_tmp  ALL DEPENDS ${ProtoSources}) # Force the re-generation

# OROCOS KDL
find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})
set(LIBS ${LIBS} ${orocos_kdl_LIBRARIES})


set(LIBS ${LIBS} ${RTAI_LIBRARIES}  pthread ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${PROTOBUF_LIBRARIES} m3chains m3toolbox m3hardware m3base)


set( ALL_SRCS 
factory_proxy.cpp
omnibase.cpp
omnibase_shm.cpp
vehicle.cpp

)
set( ALL_HDRS 
omnibase.h
omnibase_shm.h
omnibase_shm_sds.h
vehicle.h

)
list(APPEND ALL_SRCS ${ProtoSources})

list(APPEND ALL_HDRS
${ProtoHeaders}
${ProtoFiles}
)



## Building
add_library(${LIBNAME} SHARED ${ALL_SRCS})
target_link_libraries(${LIBNAME} ${LIBS})
#set_target_properties(${LIBNAME} PROPERTIES COMPILE_FLAGS "-I${CMAKE_BINARY_DIR}/src")

## Installing
install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${ALL_HDRS} DESTINATION include/m3/${PROJECT_NAME})

endif(RTAI)

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
   DESTINATION ${PYTHON_SITE_DIR}/m3
   COMPONENT library 
) 
