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

#include "m3/hardware_iq/humanoid_ros_iq.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
#include <cmath>

namespace m3{
	
using namespace m3rt;
using namespace std;
using namespace ros;


bool M3HumanoidRosIQ::ReadConfig(const char * filename)
{
	YAML::Node doc;
	if (!M3Component::ReadConfig(filename))
		return false;

	GetYamlDoc(filename, doc);	
	
	return true;
}

bool M3HumanoidRosIQ::LinkDependentComponents()
{
	/*act=(M3Actuator*) factory->GetComponent(act_name);
	if (act==NULL)
	{
		M3_INFO("M3Actuator component %s not found for component %s\n",act_name.c_str(),GetName().c_str());
		return false;
	}
	if (!trans->LinkDependentComponents(factory))
		return false;*/
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3HumanoidRosIQ::Startup()
{
	if (bot!=NULL)
		SetStateSafeOp();
	else
		SetStateError();
}

void M3HumanoidRosIQ::Shutdown()
{
	
	bot=NULL;
}

void M3HumanoidRosIQ::StepStatus()
{
	if (IsStateError())
		return;
	
}


void M3HumanoidRosIQ::StepCommand()
{
	if (!bot || IsStateSafeOp())
		return;
	if(IsStateError())
	{
		//act->SetDesiredControlMode(ACTUATOR_MODE_OFF);
		return;
	}
	
	// TODO: Step Trajectory and set theta cmds 
	
	//bot->SetThetaDeg();
	
}


}