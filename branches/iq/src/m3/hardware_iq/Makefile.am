# M3 -- Meka Robotics Robot Components
# Copyright (c) 2010 Meka Robotics
# Author: edsinger@mekabot.com (Aaron Edsinger)
# 
# M3 is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# M3 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details. 
# 
# You should have received a copy of the GNU Lesser General Public License
# along with M3.  If not, see <http://www.gnu.org/licenses/>.

## Process this file with automake to produce Makefile.in

#//////////////////////////////////////////////////////////////////

m3_dir = $(top_srcdir)/src
m3rt_dir = $(prefix)/include/m3rt
m3rtb_dir = $(prefix)/include/m3rt/base
m3tb_dir = $(prefix)/include/m3/toolbox

ros_dir = $(ROS_DIR)


libm3hardware_iqincludedir = $(includedir)/m3/hardware_iq
libm3hardware_iqinclude_HEADERS = \
	humanoid_ros_iq.pb.h \
	humanoid_ros_iq.h \
	humanoid_shm.pb.h \
	humanoid_shm.h

clean-local:
	rm -f *.pb.*
	
#////////////////////////// M3 SYS ////////////////////////////
humanoid_ros_iq.pb.h: humanoid_ros_iq.pb.cpp

humanoid_ros_iq.pb.cpp: humanoid_ros_iq.proto
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I../chains -I./ --cpp_out=./ humanoid_ros_iq.proto
	mv humanoid_ros_iq.pb.cc humanoid_ros_iq.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I../chains -I./ --python_out=$(top_srcdir)/python/m3 humanoid_ros_iq.proto

humanoid_shm.pb.h: humanoid_shm.pb.cpp

humanoid_shm.pb.cpp: humanoid_shm.proto
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I../chains -I./ --cpp_out=./ humanoid_shm.proto
	mv humanoid_shm.pb.cc humanoid_shm.pb.cpp
	protoc -I$(prefix)/include/m3rt/base -I../toolbox -I../hardware -I../chains -I./ --python_out=$(top_srcdir)/python/m3 humanoid_shm.proto

	
proto: humanoid_shm.pb.cpp 

proto_install: 
	if test -d $(includedir)/m3; then echo '' ; else mkdir $(includedir)/m3;fi
	if test -d $(includedir)/m3/chains; then echo '' ; else mkdir $(includedir)/m3/chains;fi
	cp *.proto $(includedir)/m3/chains
	
lib_LTLIBRARIES = libm3hardware_iq.la 
libm3hardware_iq_la_LIBADD = -lprotobuf -lm3base -lorocos-kdl -lm3hardware -lm3toolbox -lyaml-cpp -lXmlRpc -lros -lroslib
libm3hardware_iq_la_LDFLAGS = -version-info 1:1:0 -module \
	-L$(ros_dir)/core/roscpp/lib \
	-L$(ros_dir)/core/roslib/lib \
	-L$(ros_dir)/core/rosconsole/lib \
	-L$(ros_dir)/3rdparty/xmlrpcpp/lib \
	-L$(top_srcdir)/src/m3/toolbox \
	-L$(top_srcdir)/src/m3/hardware \
	-L$(ros_dir)/../stacks/ros_comm/utilities/cpp_common/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_traits/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_serialization/lib \
	-L$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp/lib \
	-L$(ros_dir)/../stacks/ros_comm/tools/rosconsole/lib \
	-L$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/lib
	

libm3hardware_iq_la_CPPFLAGS =   -I../hardware/ -I../toolbox/ -I../chains/ -I../robots/ -I$(m3_dir) \
	-I$(m3rt_dir) -I$(m3rtb_dir) -ffast-math -pipe -mhard-float \
	-I$(m3tb_dir) \
	$(EIGEN2_CFLAGS) \
	-I$(ros_dir)/core/roscpp/include \
	-I$(top_srcdir)/ros/m3meka_msgs/srv_gen/cpp/include \
	-I$(top_srcdir)/ros/m3meka_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/core/roslib/include \
	-I$(ros_dir)/core/rosconsole/include \
	-I$(ros_dir)/3rdparty/xmlrpcpp/src \
	-I$(ros_dir)/../stacks/ros_comm/utilities/cpp_common/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_traits/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp_serialization/include \
	-I$(ros_dir)/../stacks/ros_comm/clients/cpp/roscpp/include \
	-I$(ros_dir)/../stacks/ros_comm/tools/rosconsole/include \
	-I$(ros_dir)/../stacks/ros_comm/utilities/xmlrpcpp/src \
	-I$(ros_dir)/../stacks/ros_comm/utilities/rostime/include



libm3hardware_iq_la_SOURCES = \
	humanoid_shm.pb.cpp\
	humanoid_shm.cpp\
	factory_proxy.cpp

