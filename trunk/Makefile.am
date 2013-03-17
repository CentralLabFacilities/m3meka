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

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -g 

#SUBDIRS =   src/m3rt/base  src/m3rt/rt_system src/m3/toolbox src/m3/hardware src/m3/inverse_kine src/m3/chains \
#	src/m3/robots src/m3/robot_ctrl  python



all-local:
	bash disable_ros

if RTAI
SUBDIRS =   src/m3/toolbox \
	src/m3/hardware \
	src/m3/chains \
	src/m3/robots\
	src/m3/robot_ctrl \
	src/m3/inverse_kine \
	src/m3/shared_mem \
	src/apps/m3humanoid_shm_test \
	python
else
SUBDIRS =   src/m3/toolbox \
	src/m3/hardware \
	src/m3/chains \
	src/m3/robots\
	src/m3/robot_ctrl \
	src/m3/inverse_kine \
	python
endif
	
# src/m3/vehicles

dist_bin_SCRIPTS = scripts/m3rt_printer_run

includedir =  /usr/local/include/m3meka_msgs

include_HEADERS = ros/m3meka_msgs/msg_gen/cpp/include/m3meka_msgs/M3JointStatus.h \
		ros/m3meka_msgs/msg_gen/cpp/include/m3meka_msgs/M3OmnibaseJoy.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3HumanoidCmd.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3HumanoidParam.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3HumanoidStatus.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3JointArrayCmd.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3JointArrayParam.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3JointArrayStatus.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3LoadX6Cmd.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3LoadX6Param.h \
		ros/m3meka_msgs/srv_gen/cpp/include/m3meka_msgs/M3LoadX6Status.h


python_install:
	$(MAKE) -C python install

	


proto:
	$(MAKE) -C src/m3/hardware proto
	$(MAKE) -C src/m3/chains proto
	$(MAKE) -C src/m3/robots proto
	$(MAKE) -C src/m3/robot_ctrl proto
	$(MAKE) -C src/m3/vehicles proto

proto_install:	
	$(MAKE) -C src/m3/hardware proto_install
	$(MAKE) -C src/m3/chains proto_install
	$(MAKE) -C src/m3/robots proto_install
	$(MAKE) -C src/m3/robot_ctrl proto_install	
	$(MAKE) -C src/m3/vehicles proto_install
	
	
torque_shm:
	$(MAKE) -C src/apps/m3rt_torque_shm

torque_shm_install:
	$(MAKE) -C src/apps/m3rt_torque_shm install


printer:
	$(MAKE) -C src/apps/m3rt_printer
	
printer_install:
	$(MAKE) -C src/apps/m3rt_printer install


hardware:
	$(MAKE) -C src/m3/hardware

hardware_install:
	$(MAKE) -C src/m3/hardware install
	
chains:
	$(MAKE) -C src/m3/chains

chains_install:
	$(MAKE) -C src/m3/chains install


robots:
	$(MAKE) -C src/m3/robots

robots_install:
	$(MAKE) -C src/m3/robots install

robot_ctrl:
	$(MAKE) -C src/m3/robot_ctrl

robot_ctrl_install:
	$(MAKE) -C src/m3/robot_ctrl install

vehicles:
	$(MAKE) -C src/m3/vehicles

vehicles_install:
	$(MAKE) -C src/m3/vehicles install

m3_install:
	$(MAKE) -C src/m3/hardware install
	$(MAKE) -C src/m3/chains install
	$(MAKE) -C src/m3/invers_kine install
	$(MAKE) -C src/m3/robots install
	$(MAKE) -C src/m3/robot_ctrl install
		
ik:
	$(MAKE) -C src/m3/inverse_kine

ik_install:
	$(MAKE) -C src/m3/inverse_kine install
		
	
ros:
	$(MAKE) -C ros/m3_msgs
