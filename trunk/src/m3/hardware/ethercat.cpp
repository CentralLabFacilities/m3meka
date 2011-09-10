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

#include "m3/hardware/ethercat.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
#include "inttypes.h"

namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////


void M3Ethercat::Startup()
{
	if (shm_ec = (M3EcSystemShm*) rtai_malloc (nam2num(SHMNAM_M3MKMD),1))
		M3_PRINTF("Found %d active M3 EtherCAT slaves\n",shm_ec->slaves_active);
	else
	{
		M3_ERR("Rtai_malloc failure for SHMNAM_M3KMOD\n",0);
		return false;
	}
	shm_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3LSHM));
	if (!shm_sem)
	{
		M3_ERR("Unable to find the SEMNAM_M3LSHM semaphore.\n",0);
		return false;
	}
	
	sync_sem=(SEM*)rt_get_adr(nam2num(SEMNAM_M3SYNC));
	if (!sync_sem)
	{
		M3_ERR("Unable to find the SYNCSEM semaphore.\n",0);
		return false;
	}
	
	for (int i=0; i < NUM_EC_DOMAIN; i++)
	{
	    s->add_ec_domains();
	}
}

void M3Ethercat::Shutdown()
{
    rt_shm_free(nam2num(SHMNAM_M3MKMD));
    
    shm_ec=NULL;
	shm_sem=NULL;
	sync_sem=NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3Ethercat::ReadConfig(const char * filename)
{	
	int val;
	mReal mval;
	YAML::Node doc;

	if (!M3Component::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);
	
	
	return true;
}

bool M3Ethercat::LinkDependentComponents()
{
	
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3Ethercat::StepStatus()
{
	if (IsStateError())
		return;
	rt_sem_wait(sync_sem);
	rt_sem_wait(shm_sem);
	
	s=factory->GetMonitorStatus();
	if (m3ec_list.size() != 0)
	{
	  for (int i=0;i<NUM_EC_DOMAIN;i++)
	  {
	      s->mutable_ec_domains(i)->set_t_ecat_wait_rx(shm_ec->monitor[i].t_ecat_wait_rx);
	      s->mutable_ec_domains(i)->set_t_ecat_rx(shm_ec->monitor[i].t_ecat_rx);
	      s->mutable_ec_domains(i)->set_t_ecat_wait_shm(shm_ec->monitor[i].t_ecat_wait_shm);
	      s->mutable_ec_domains(i)->set_t_ecat_shm(shm_ec->monitor[i].t_ecat_shm);
	      s->mutable_ec_domains(i)->set_t_ecat_wait_tx(shm_ec->monitor[i].t_ecat_wait_tx);
	      s->mutable_ec_domains(i)->set_t_ecat_tx(shm_ec->monitor[i].t_ecat_tx);	
	  }
	  s->set_num_ethercat_cycles(GetEcCounter());
	}
	rt_sem_signal(shm_sem);
	
}


void M3Ethercat::StepCommand()
{
	
}

}