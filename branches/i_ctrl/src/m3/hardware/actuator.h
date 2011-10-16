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

#ifndef M3_ACTUATOR_H
#define M3_ACTUATOR_H

#include "m3rt/base/component.h"
#include "m3/hardware/actuator.pb.h"
#include <m3/hardware/actuator_ec.pb.h>
#include "m3/hardware/actuator_ec.h"
#include "m3/hardware/sensor.h"
#include "m3/hardware/motor.h"
#include "m3/toolbox/toolbox.h"
#include "m3/toolbox/dfilter.h"
#include <google/protobuf/message.h>

namespace m3
{
	using namespace std;

	
class M3Actuator : public m3rt::M3Component
{
	public:
		M3Actuator(): m3rt::M3Component(CALIB_PRIORITY),ecc(NULL),tq_des_last(0),ignore_bounds(false),overload_cnt(0),old_ts(0),encoder_calib_req(1),old_ts_rtai(0),old_ticks(0),old_is_calibrated(false)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Updated safety thresholds to use motor model.
			RegisterVersion("uta",UTA);		//UTA. Updated to use current controllers and thermal models
			
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	public:
		//API
		
		//Setpoints
		void SetDesiredPwm(int val){command.set_pwm_desired(val);}
		void SetDesiredControlMode(ACTUATOR_MODE val){command.set_ctrl_mode(val);}
		void SetDesiredTrajectoryMode(ACTUATOR_TRAJ_MODE val){command.set_traj_mode(val);}
		void SetDesiredTorque(mReal val){ command.set_tq_desired(val);}
		void SetDesiredCurrent(mReal val){ command.set_i_desired(val);}
		void SetBrakeOff(bool off){command.set_brake_off(off);}
		
		//Status
		mReal GetDesiredTorque(){return command.tq_desired();}
		mReal GetMotorTemp(){return status.motor_temp();}
		mReal GetAmpTemp(){return status.amp_temp();}
		mReal GetAmbientTemp(){return status.ambient_temp();}
		mReal GetCaseTemp(){return status.case_temp();}
		mReal GetExtTemp(){return status.ext_temp();}
		mReal GetPower(){return status.power();}
		mReal GetCurrent(){return status.current();}
		mReal GetThetaDeg(){ return status.theta();}
		mReal GetThetaRad(){ return DEG2RAD(status.theta());}
		mReal GetThetaDotDeg(){return status.thetadot();}
		mReal GetThetaDotRad(){return DEG2RAD(status.thetadot());}
		mReal GetThetaDotDotDeg(){return status.thetadotdot();}
		mReal GetThetaDotDotRad(){return DEG2RAD(status.thetadotdot());}
		int64_t GetTimestamp(){return status.base().timestamp();}
		mReal GetTorque(){return status.torque();}
		mReal GetTorqueDot(){return status.torquedot();}
		int GetPwmCmd(){return status.pwm_cmd();}
		mReal GetCurrentCmd(){return status.i_cmd();}
		mReal GetTorqueCmd(){return status.tq_cmd();}
		int GetFlags(){return status.flags();}
		virtual bool IsMotorPowerOn()     {if (!GetActuatorEc()) return false; return ecc->IsMotorPowerOn();}
		
		
		//Conversions
		int mNmToTicks(mReal x){return tq_sense.mNmToTicks(x);}
		mReal ActuatorTorqueToMotorCurrent(mReal mNm){return MotorTorqueToMotorCurrent(ActuatorTorqueToMotorTorque(mNm));}
		mReal ActuatorTorqueToMotorTorque(mReal mNm){return mNm/motor.GetGearRatio();}
		mReal MotorTorqueToMotorCurrent(mReal mNm){return 1000*mNm/motor.GetTorqueConstant();} //mNm/A;
		
		//Encoder and Limits
		virtual int GetTicksQEI();
		virtual void SetZeroEncoder(){if (GetActuatorEc()) ecc->SetZeroEncoder();}
		virtual void ClrZeroEncoder(){if (GetActuatorEc()) ecc->ClrZeroEncoder();}
		virtual void SetLimitSwitchNegZeroEncoder(){if (GetActuatorEc()) ecc->SetLimitSwitchNegZeroEncoder();}
		virtual void ClrLimitSwitchNegZeroEncoder(){if (GetActuatorEc()) ecc->ClrLimitSwitchNegZeroEncoder();}
		virtual bool IsLimitSwitchPosOn() {if (!GetActuatorEc()) return false; return ecc->IsLimitSwitchPosOn();}
		virtual bool IsLimitSwitchNegOn() {if (!GetActuatorEc()) return false; return ecc->IsLimitSwitchNegOn();}
		virtual bool IsEncoderCalibrated(){if (!encoder_calib_req) return true; if (!GetActuatorEc()) return false; return ecc->IsEncoderCalibrated();}		
		
		//Misc
		M3ActuatorEc * GetActuatorEc(){return ecc;}
		M3ActuatorTrajectoryParam * 	ParamTorqueTraj(){return param.mutable_tq_traj();}
		M3ActuatorPIDParam * 	ParamPidTorqueIFF(){return param.mutable_pid_tq_iff();}
		
	protected:
		enum {DEFAULT, ISS,UTA};		
		M3AngleSensor q_sense;
		M3TorqueSensor tq_sense;
		M3TempSensor ex_sense;
		M3TempSensor at_sense;
		M3CurrentSensor i_sense;
		M3MotorModel motor;
		M3TimeAvg amp_temp_avg;
		M3JointFilter angle_df;
		M3DFilter torquedot_df;
		M3DitherToInt pwm_dither;
		M3PeakToPeak tq_p2p;
		M3PID2	     pid_tq_iff;
		
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		M3ActuatorStatus status;
		M3ActuatorCommand command;
		M3ActuatorParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		bool LinkDependentComponents();
		void StepOverloadDetect();
		string ecc_name;
		M3ActuatorEc * ecc; 	//Corresponding EtherCAT component 
		int tmp_cnt;
		bool ignore_bounds;
		bool safe_pwm_limit; 
		bool encoder_calib_req;
		int overload_cnt;
		mReal max_motor_temp; //V-DEFAULT
		mReal max_current;//V-DEFAULT
		mReal tq_des_last;
		unsigned long long old_ts;
		long long old_ts_rtai;
		int old_ticks;
		bool old_is_calibrated;
};


}

#endif


