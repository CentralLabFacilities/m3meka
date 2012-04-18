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

#ifndef M3BIP_SHM_H
#define M3BIP_SHM_H

#include <m3rt/base/component_shm.h>
#include "bip_shm.pb.h"
#include "bip_shm_sds.h"
#include <m3uta/controllers/uta_imu.h>
#include "bip_act_array.h"


namespace m3bip{
using namespace std;
using namespace m3;
using namespace m3uta;

class M3BotShm : public  m3::M3CompShm{
	public:
		M3BotShm(): sds_status_size(0),sds_cmd_size(0),M3CompShm(),bot(NULL),right_hand(NULL),right_loadx6(NULL),tmp_cnt(0)
		{		  
		  RegisterVersion("default",DEFAULT);		  
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		bool ReadConfig(const char * filename);		
		size_t GetStatusSdsSize();
		size_t GetCommandSdsSize();
		void SetCommandFromSds(unsigned char * data);
		void SetSdsFromStatus(unsigned char * data);
		bool LinkDependentComponents();
		void ResetCommandSds(unsigned char * sds);
		void Startup();		
		
		enum {DEFAULT};
		M3BaseStatus * GetBaseStatus();		
		M3BotShmCommand command;
		M3BotShmParam param;
		M3BotShmStatus status;
		M3Humanoid * bot;
		M3LoadX6 * right_loadx6;
		M3Hand * right_hand;
		M3BotShmSdsCommand command_from_sds;
		M3BotShmSdsStatus status_to_sds;
		int sds_status_size;
		int sds_cmd_size;	
		string bot_name, right_hand_name, right_loadx6;
		int64_t timeout;
		int tmp_cnt;		
		bool startup_motor_pwr_on;
};

}
#endif


