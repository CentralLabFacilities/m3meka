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
# Linux kernel directory
#------------------------------------------------------------------------------
AC_ARG_WITH([linux-dir],
    AC_HELP_STRING(
        [--with-linux-dir=<DIR>],
        [Linux kernel sources @<:@running kernel@:>@]
    ),
    [
        sourcedir=[$withval]
    ],
    [
        version=[`uname -r`]
        modulesdir=/lib/modules/${version}
        if test \! -d ${modulesdir} || test \! -d ${modulesdir}/build; then
           echo
           AC_MSG_ERROR([Failed to find Linux sources. Use --with-linux-dir!])
        fi
        sourcedir=`cd ${modulesdir}/build && pwd -P`
    ]
)

AC_MSG_CHECKING([for Linux kernel sources])

if test \! -r ${sourcedir}/.config; then
    echo
    AC_MSG_ERROR([No configured Linux kernel sources in $sourcedir])
fi

# Try to get kernel release string
if test -r ${sourcedir}/include/config/kernel.release; then
    kernelrelease=`cat $sourcedir/include/config/kernel.release`
elif test -r ${sourcedir}/.kernelrelease; then
    kernelrelease=`cat $sourcedir/.kernelrelease`
elif test -r ${sourcedir}/include/linux/utsrelease.h; then
    hdr=${sourcedir}/include/linux/utsrelease.h
    kernelrelease=`grep UTS_RELEASE $hdr | cut -d " " -f 3- | tr -d \"`
elif test -r ${sourcedir}/include/linux/version.h; then
    hdr=${sourcedir}/include/linux/version.h
    kernelrelease=`grep UTS_RELEASE $hdr | cut -d " " -f 3- | tr -d \"`
fi

if test -z "$kernelrelease"; then
    echo
    AC_MSG_ERROR([Failed to extract Linux kernel version!])
fi

# Extract three numbers from kernel release string
linuxversion=`echo $kernelrelease | grep -oE "^[[0-9]]+\.[[0-9]]+\.[[0-9]]+"`

AC_SUBST(LINUX_SOURCE_DIR,[$sourcedir])
AC_MSG_RESULT([$LINUX_SOURCE_DIR (Kernel $linuxversion)])

#------------------------------------------------------------------------------
# Linux module installation subdirectory
#------------------------------------------------------------------------------
AC_ARG_WITH([module-dir],
    AC_HELP_STRING(
        [--with-module-dir=<DIR>],
        [Linux module installation dir. Default: m3]
    ),
    [moddir=[$withval]],
    [moddir="m3"]
)
AC_SUBST(INSTALL_MOD_DIR,[$moddir])

AC_MSG_CHECKING([for Linux modules installation directory])
AC_MSG_RESULT([$INSTALL_MOD_DIR])

#------------------------------------------------------------------------------
# Ethercat
#------------------------------------------------------------------------------
#PKG_CHECK_MODULES([ETHERCAT], [ethercat >= 1.5.0])

#------------------------------------------------------------------------------
# Eigen
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([EIGEN2], [eigen2 >= 2.0.15])

#------------------------------------------------------------------------------

#TODO: have autoconf find correct rtai dir
AC_SUBST(RTAI_DIR,[/usr/realtime-2.6.32.11-rtai-3.8.1])

#TODO: have autoconf find correct ros dir
AC_SUBST(ROS_DIR,[/opt/ros/cturtle/ros])

AC_CONFIG_FILES([
	Makefile
	python/Makefile
	src/apps/m3rt_printer/Makefile
	src/apps/m3rt_torque_shm/Makefile
	src/apps/m3humanoid_shm_test/Makefile
	src/apps/m3rt_timestats/Makefile
	src/m3/toolbox/Makefile
	src/m3/hardware/Makefile
	src/m3/inverse_kine/Makefile
	src/m3/robots/Makefile
	src/m3/chains/Makefile
	src/m3/shared_mem/Makefile
	src/m3/robot_ctrl/Makefile
	src/m3/vehicles/Makefile
	src/m3/hardware_iq/Makefile
])
AC_OUTPUT
