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

#ifndef M3_CHAIN_H
#define M3_CHAIN_H

#include "m3/chains/dynamatics.h"
#include "m3/chains/joint_array.h"


namespace m3
{
	using namespace std;
	using namespace KDL;

class M3Dynamatics;
///////////////////////////////////////////////////////////////////////////

class M3JointChain : public M3JointArray
{
	public:
		M3JointChain() : dynamatics(NULL), M3JointArray(){}		
		M3Dynamatics * GetDynamatics(){return dynamatics;}
		void SetG(int i, mReal val);	
		void SetGCoupled();	
	protected:
		void Startup();
		bool ReadConfig(const char * filename);
		bool LinkDependentComponents();	
	private:
		string dynamatics_name;
		M3Dynamatics * dynamatics;
		JntArray G;
};

}

#endif


