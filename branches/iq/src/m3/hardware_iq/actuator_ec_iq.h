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

#ifndef M3_ACTUATOR_EC_IQ_H
#define M3_ACTUATOR_EC_IQ_H

#include <m3rt/base/component.h>
#include <m3rt/base/component_ec.h>
#include <m3/hardware/actuator_ec_iq.pb.h>
#include "m3/hardware/pwr.h"
#include "m3/toolbox/toolbox.h"
#include <google/protobuf/message.h>
#include "m3rt/base/m3ec_def.h"
#include "m3/hardware/m3ec_pdo_iq_def.h"



namespace m3{
using namespace std;


class M3ActuatorEcIQ : public  m3rt::M3ComponentEc{
	public:
		M3ActuatorEcIQ():ignore_pwm_slew(0), 
		pwr(NULL),pdo_status_size(0),toggle(0),pdo_cmd_size(0),pwm_ff(0),
		pwm_max_ext(0),error_printed(false),m3rt::M3ComponentEc()
		{
			memset(&exs,0,sizeof(M3ActPdoV2StatusExt));
			memset(&exc,0,sizeof(M3ActPdoV2CmdExt));
			memset(&axc,0,sizeof(M3ActPdoV2Cmd));
			memset(&acc,0,sizeof(M3ActPdoIQCmd));
			memset(&scc,0,sizeof(M3SeaPdoV0Cmd));
		
			RegisterVersion("default",DEFAULT);		
			RegisterPdo("actx1_pdo_iq", ACTX1_PDO_IQ);	
			RegisterPdo("actx2_pdo_iq", ACTX2_PDO_IQ);	
			RegisterPdo("actx3_pdo_iq", ACTX3_PDO_IQ);	 
			RegisterPdo("actx4_pdo_iq", ACTX4_PDO_IQ);	
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
		
		bool IsMotorPowerOn(){return pwr->IsMotorPowerOn();}
		void SetMotorEnable(int on){pwr->SetMotorEnable(on);}
		
		void SetZeroEncoder(){param.set_config(param.config()|ACTUATOR_EC_CONFIG_CALIB_QEI_MANUAL);}
		void ClrZeroEncoder(){param.set_config(param.config()&~ACTUATOR_EC_CONFIG_CALIB_QEI_MANUAL);}
		void SetLimitSwitchNegZeroEncoder(){param.set_config(param.config()|ACTUATOR_EC_CONFIG_CALIB_QEI_LIMITSWITCH_NEG);}
		void SetLimitSwitchPosZeroEncoder(){param.set_config(param.config()|ACTUATOR_EC_CONFIG_CALIB_QEI_LIMITSWITCH_POS);}
		void ClrLimitSwitchNegZeroEncoder(){param.set_config(param.config()&~ACTUATOR_EC_CONFIG_CALIB_QEI_LIMITSWITCH_NEG);}
		void ClrLimitSwitchPosZeroEncoder(){param.set_config(param.config()&~ACTUATOR_EC_CONFIG_CALIB_QEI_LIMITSWITCH_POS);}
		
		bool IsLimitSwitchPosOn() {return status.flags()&ACTUATOR_EC_FLAG_POS_LIMITSWITCH;}
		bool IsLimitSwitchNegOn() {return status.flags()&ACTUATOR_EC_FLAG_NEG_LIMITSWITCH;}
		bool IsAuxSwitchOn() {return status.flags()&ACTUATOR_EC_FLAG_AUX_SWITCH;}
		bool IsEncoderCalibrated(){return status.flags()&ACTUATOR_EC_FLAG_QEI_CALIBRATED;}
		
		void SetPwmMax(int val){pwm_max_ext=val;}
		
		//bool IsCurrentFaultMom() {return status.flags()&M3ACT_FLAG_I_FAULT_MOM;}
		//bool IsCurrentFaultCont() {return status.flags()&M3ACT_FLAG_I_FAULT_CONT;}
		
	protected:
		bool ReadConfig(const char * filename);
		M3EtherCATStatus * GetEcStatus(){return status.mutable_ethercat();}
		void SetStatusFromPdo(unsigned char * data);
		void SetStatusFromPdoV0(unsigned char * data);
		void SetStatusFromPdoV1(unsigned char * data);
		void SetStatusFromPdoV2(unsigned char * data);
		void SetStatusFromPdoV3(unsigned char * data);
		void SetPdoFromCommand(unsigned char * data);
		bool LinkDependentComponents();
		void ResetCommandPdo(unsigned char * pdo);
		void SetPdoV2FromPdoV1Command(unsigned char * data);
		void SetPdoV0FromPdoV1Command(unsigned char * data);
		void StepStatus();
	protected:
		enum {ACTX1_PDO_IQ, ACTX2_PDO_IQ, ACTX3_PDO_IQ, ACTX4_PDO_IQ};
		enum {DEFAULT};
		M3BaseStatus * GetBaseStatus();
		M3ActuatorEcIQStatus status;
		M3ActuatorEcIQCommand command;
		M3ActuatorEcIQParam param;
		M3Pwr * pwr;
		string pwr_name;
		M3TimeSlew pwr_slew;
		M3TimeSlew pwm_slew;
		int pdo_status_size;
		int pdo_cmd_size;
		int pwm_ff;
		int tmp_cnt;
		int chid;
		int pwm_max_ext;
		int ignore_pwm_slew;
		/*M3ActPdoV2StatusExt exs;
		M3ActPdoV2Cmd    axc;
		M3ActPdoV2CmdExt exc;
		M3ActPdoV1Cmd  acc;
		M3SeaPdoV0Cmd  scc;*/
		bool error_printed;
		
		int toggle;
		
};


































































































}
#endif


