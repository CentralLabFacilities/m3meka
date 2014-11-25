AC_PREREQ(2.59)
AC_INIT([m3],[1.3],[info@mekabot.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([-Wall -Werror dist-bzip2])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_SRCDIR([config.h.in])
AC_CONFIG_MACRO_DIR([m4])

#------------------------------------------------------------------------------
# Global
#------------------------------------------------------------------------------

AC_PROG_CXX
AC_PROG_LIBTOOL
AM_PROG_CC_C_O
AM_PATH_PYTHON(2.3)
AC_PROG_SWIG(1.3.21)
#AC_PROG_SWIG(2.0)
SWIG_ENABLE_CXX
SWIG_PYTHON

#------------------------------------------------------------------------------
# Virtual M3
#------------------------------------------------------------------------------
AC_ARG_ENABLE([virtual-only],
[  --enable-virtual-only    Turn off realtime and ethercat],
[case "${enableval}" in
  yes) rtai=false ;;
  no)  rtai=true ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-virtual-only]) ;;
esac],[rtai=true])
AM_CONDITIONAL([RTAI], [test x$rtai = xtrue])


#------------------------------------------------------------------------------
# Eigen
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([EIGEN2], [eigen2 >= 2.0.15])

#------------------------------------------------------------------------------

#TODO: have autoconf find correct rtai dir
AC_SUBST(RTAI_DIR,[/usr/realtime])

#TODO: have autoconf find correct ros dir
AC_SUBST(ROS_DIR,[/opt/ros/cturtle/ros])

AC_CONFIG_FILES([
	Makefile
	python/Makefile
	src/apps/m3rt_printer/Makefile
	src/apps/m3humanoid_shm_test/Makefile
	src/apps/m3rt_torque_shm/Makefile
	src/apps/m3rt_timestats/Makefile
	src/m3/toolbox/Makefile
	src/m3/hardware/Makefile
	src/m3/inverse_kine/Makefile
	src/m3/robots/Makefile
	src/m3/chains/Makefile
	src/m3/shared_mem/Makefile
	src/m3/robot_ctrl/Makefile
	src/m3/vehicles/Makefile
])
AC_OUTPUT
