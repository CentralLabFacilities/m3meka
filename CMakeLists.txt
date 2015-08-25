cmake_minimum_required(VERSION 2.8)
project(m3meka)
SET(MAJOR_VERSION "1")
SET(MINOR_VERSION "9")
SET(PATCH_VERSION "9")
set(M3CORE_VERSION
  ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

# paths to the FindXXX.cmake
set(CMAKE_MODULE_PATH
    "${MEKABOT}/meka-cmake-modules/Modules/"
    $ENV{M3_CMAKE_MODULES}
    ${CMAKE_MODULE_PATH})

OPTION(RTAI "Enable/Disable RTAI (used for python proxy users only)" ON)
if(RTAI)
  find_package(RTAI)
endif()
if(RTAI AND NOT RTAI_FOUND)
  set(RTAI OFF)
  message(WARNING "Rtai not present, only generating python")
endif()


#set(MEKABOT ${CMAKE_SOURCE_DIR})
#set(M3RT_SRC ${MEKABOT}/m3core/src )
#set(M3_SRC ${MEKABOT}/m3meka/src)

if(NOT MEKABOT)
message(WARNING "m3meka is not in mekabot, so the FindXXX.cmake might not be found, if so, please run:\n git clone https://github.com/ahoarau/meka-cmake-modules\n To get the FindXXX.cmake")
endif()

# Find M3RT
find_package(M3rt REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3RT_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3RT_INCLUDE_DIR})

# Compile in c++11
OPTION(C++11 "Use c++11 " 1)

if(C++11)
	set(CMAKE_CXX_FLAGS "-std=c++0x")
	message("-- Using c++11")
endif(C++11)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


set(M3_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src 
)

# Include dirs to look for
include_directories(
    ${M3_SRC}
    ${CMAKE_CURRENT_BINARY_DIR}/src
)


add_subdirectory(src)
add_subdirectory(python)
add_subdirectory(scripts)

if(NOT MEKABOT)
configure_file(setup.bash.in ${CMAKE_CURRENT_BINARY_DIR}/setup.bash)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.bash DESTINATION share)
endif()

IF(EXISTS "${CMAKE_ROOT}/Modules/CPack.cmake")
INCLUDE(InstallRequiredSystemLibraries)
 
SET(CPACK_SET_DESTDIR "on")
SET(CPACK_PACKAGING_INSTALL_PREFIX "/tmp")
SET(CPACK_GENERATOR "DEB")
 
SET(CPACK_PACKAGE_DESCRIPTION "m3meka : Meka robot specific control components")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Real-time control system for robot control - interfaced with ROS")
SET(CPACK_PACKAGE_VENDOR "Antoine Hoarau")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Antoine Hoarau <hoarau.robotics@gmail.com>")
SET(CPACK_PACKAGE_CONTACT "hoarau.robotics@gmail.com")
SET(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_MINOR "${MINOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_PATCH "${PATCH_VERSION}")
SET(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${MAJOR_VERSION}.${MINOR_VERSION}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${MAJOR_VERSION}.${MINOR_VERSION}.${CPACK_PACKAGE_VERSION_PATCH}")
#demo dependencies
#SET(CPACK_DEBIAN_PACKAGE_DEPENDS " kate , plasma-desktop, libc6 (>= 2.1.3) ")
 
#dependencies for this service menu
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "python2.7,libqt4-dev,moc,g++,libncurses5-dev,kernel-package,gcc-multilib,libc6-dev,libtool,automake,cmake,git,openssh-server,openssh-client,libeigen3-dev,libprotobuf-dev,protobuf-compiler,libboost-dev,protobuf-compiler,python-dev,libprotobuf-dev,python-protobuf,python-matplotlib,python-yaml,python-gnuplot,python-scipy,python-sip-dev,python-sip,sip-dev ")
 
SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
SET(CPACK_DEBIAN_PACKAGE_SECTION "libraries")
SET(CPACK_DEBIAN_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
 
#SET(CPACK_COMPONENTS_ALL Libraries ApplicationData)
INCLUDE(CPack)
 
ENDIF(EXISTS "${CMAKE_ROOT}/Modules/CPack.cmake")
