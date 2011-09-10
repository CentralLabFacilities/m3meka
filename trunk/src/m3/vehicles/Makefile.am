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
ros_dir = /opt/ros/cturtle/ros
libm3vehiclesincludedir = $(includedir)/m3/vehicles
libm3vehiclesinclude_HEADERS = \
	vehicle.h\
	omnibase.h\
	omnibase.pb.h
	

clean-local:
	rm -f *.pb.*
	
omnibase.pb.h: omnibase.pb.cpp

omnibase.pb.cpp: omnibase.proto
	protoc -I$(prefix)/include/m3rt/base  -I../toolbox -I../chains -I../hardware -I./ --cpp_out=./ omnibase.proto
	mv omnibase.pb.cc omnibase.pb.cpp
	protoc -I$(prefix)/include/m3rt/base  -I../toolbox -I../chains -I../hardware -I./ --python_out=$(top_srcdir)/python/m3 omnibase.proto	
	
proto: omnibase.pb.cpp

proto_install: 
	if test -d $(includedir)/m3; then echo '' ; else mkdir $(includedir)/m3;fi
	if test -d $(includedir)/m3/vehicles; then echo '' ; else mkdir $(includedir)/m3/vehicles;fi
	cp *.proto $(includedir)/m3/vehicles
	
lib_LTLIBRARIES = libm3vehicles.la 
libm3vehicles_la_LIBADD = -lprotobuf -lm3base -lorocos-kdl -lm3hardware -lm3chains -lm3toolbox \
	-lyaml-cpp -lXmlRpc -lros -lroslib -lholomni_pcv
libm3vehicles_la_LDFLAGS = -version-info 1:1:0 -module \
	-L$(ros_dir)/core/roscpp/lib \
	-L$(ros_dir)/core/roslib/lib \
	-L$(ros_dir)/core/rosconsole/lib \
	-L$(ros_dir)/3rdparty/xmlrpcpp/lib
libm3vehicles_la_CPPFLAGS =   -I../chains -I../toolbox -I../hardware -I$(m3_dir) -I$(m3rt_dir) -I$(m3rtb_dir) -I$(eigen_dir) -ffast-math -pipe -mhard-float \
	-I$(ros_dir)/core/roscpp/include \
	-I$(top_srcdir)/ros/m3meka_msgs/srv_gen/cpp/include \
	-I$(top_srcdir)/ros/m3meka_msgs/msg_gen/cpp/include \
	-I$(ros_dir)/core/roslib/include \
	-I$(ros_dir)/core/rosconsole/include \
	-I$(ros_dir)/3rdparty/xmlrpcpp/src \
	-I$(includedir)/holomni_pcv
libm3vehicles_la_SOURCES = \
	omnibase.pb.cpp\
	omnibase.cpp\
	vehicle.cpp\
	factory_proxy.cpp
