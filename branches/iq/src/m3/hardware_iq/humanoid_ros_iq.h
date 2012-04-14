/* 
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef M3_HUMANOID_ROS_IQ_H
#define M3_HUMANOID_ROS_IQ_H

#include "m3rt/base/component.h"
#include <google/protobuf/message.h>
#include "m3/robots/humanoid.h"


namespace m3
{
	using namespace std;
	using namespace ros;


class M3HumanoidRosIQ: public m3rt::M3Component
{
	public:
		M3HumanoidRosIQ(): m3rt::M3Component(JOINT_PRIORITY),bot(NULL)
		{
			RegisterVersion("default",DEFAULT);	//RBL. Now works with default case/ambient values			
		}
		//API
		
		
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}			
		
	protected:
		enum{DEFAULT};
		
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		bool ReadConfig(const char * filename);
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3Humanoid * bot;
		M3JointStatus status;
		M3JointCommand command;
		M3JointParam param;

};


}

#endif


