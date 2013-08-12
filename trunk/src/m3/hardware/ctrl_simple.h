// M3 -- Meka Robotics Robot Components
// Copyright (c) 2010 Meka Robotics
// Author: edsinger@mekabot.com (Aaron Edsinger)
// 
// M3 is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// M3 is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with M3.  If not, see <http://www.gnu.org/licenses/>.
 

#ifndef M3_CTRL_SIMPLE_H
#define M3_CTRL_SIMPLE_H


#include <google/protobuf/message.h>
#include <m3rt/base/component.h>
#include "../toolbox/toolbox.h"
#include "../toolbox/dfilter.h"
#include "ctrl_simple.pb.h"
#include "actuator.h"
#include "actuator.pb.h"

namespace m3
{
	using namespace std;
	using namespace m3rt;
	

class M3CtrlSimple: public  m3rt::M3Component
{
	public:
		M3CtrlSimple(): m3rt::M3Component(CONTROL_PRIORITY),	pnt_cnt(0),
																act(NULL)
		{
			RegisterVersion("default",DEFAULT);	
		}
		
	//Setters
		void SetDesiredControlMode(CTRL_SIMPLE_MODE x){command.set_ctrl_mode(x);}
		void SetDesiredCurrent(mReal i){command.set_desired_current(i);}
		void SetDesiredTheta(mReal q){command.set_desired_theta(q);}
		void SetDesiredTorque(mReal tq){command.set_desired_torque(tq);}
		void SetDesiredStiffness(mReal s){command.set_desired_stiffness(s);}
		void SetTorqueGravity(mReal tq){status.set_torque_gravity(tq);}
		
	//Access to Status Messages
		M3BaseStatus *			StatusBase()	{return status.mutable_base();}
		M3CtrlSimpleStatusCommand *	StatusCommand()	{return status.mutable_command();}
		M3ActuatorStatus *		StatusActuator(){return status.mutable_actuator();}
		
	//Access to Traj Params
		M3ParamTrajectory * 	ParamTrajCurrent()	{return param.mutable_traj_current();}
		M3ParamTrajectory * 	ParamTrajTheta()	{return param.mutable_traj_theta();}
		M3ParamTrajectory * 	ParamTrajTorque()	{return param.mutable_traj_torque();}
		M3ParamPID *		ParamPidTheta()		{return param.mutable_pid_theta();}
		M3ParamPID * 		ParamPidTorque()	{return param.mutable_pid_torque();}

	//Conversions
		virtual	bool	IsMotorPowerOn()		{if (!GetActuator()) return false; return act->IsMotorPowerOn();}
		M3Actuator * GetActuator()	{return act;}

	// getters
		mReal	GetJointTheta()			{return act->GetThetaRad();}
		mReal	GetJointThetaDot()		{return act->GetThetaDotRad();}
		mReal	GetJointTorque()		{return act->GetTorque();}
		mReal	GetJointTorqueDot()		{return act->GetTorqueDot();}
		
		int64_t GetTimestamp()			{return GetBaseStatus()->timestamp();}
		
		void ResetIntegrators(){pid_torque.ResetIntegrator();pid_theta.ResetIntegrator();}
		
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}

	protected:

		// required:
		enum{DEFAULT};
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		bool ReadConfig(const char * filename);
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		
		M3CtrlSimpleStatus	status;
		M3CtrlSimpleCommand	command;
		M3CtrlSimpleParam	param;
		
		
		M3PID	pid_theta;
		M3PID	pid_torque;		
		
		M3Actuator * act;
		string	act_name;
		CTRL_SIMPLE_MODE ctrl_mode_last;
		
		int pnt_cnt;
};

}

#endif


