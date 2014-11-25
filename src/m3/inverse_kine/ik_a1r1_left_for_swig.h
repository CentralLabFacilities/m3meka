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

#define IKFAST_NO_MAIN

#include "ik_a1r1_left_from_ikfast.h"


// Created IKSolver Wrapper class to avoid handling IKSolution data structure in python.
// To compile swig wrappers:
// swig -python -c++ ik_m3.i
// c++ -c ik_m3_wrap.cxx -I/usr/include/python2.6 -o ik_m3.o
// c++ -shared ik_m3.o -lstdc++ -o _ik_m3.so

class IKSolver
{
	public:
		//bool Solve(double trans0, double trans1, double trans2, double rot00, double rot01, double rot02, double rot10, 
		//	   double rot11, double rot12, double rot20, double rot21, double rot22, double free)
		bool Solve(const std::vector<double> veetrans, const std::vector<double> veerot, double free)
		{						
			IKReal pfree[1];
			
			pfree[0] = free;
							
			success = ik(&veetrans[0], &veerot[0], &pfree[0], vsolutions);
			return success;
		}
	
	
		int GetNumSolutions()
		{
			if (success)
				return vsolutions.size();
			else
				return 0;
		}
		
		std::vector<int> GetFree(int i)
		{
			return vsolutions[i].GetFree();
		}
	
		// Note:  GetFree does not seem to be needed, but we will handle in case any untested conditions arrise
		//		where it is required for a solution.
		std::vector<double> GetSolution(int i, std::vector<double> vsolfree)
		{
			std::vector<double> sol(getNumJoints());
			
			vsolutions[i].GetSolution(&sol[0],vsolfree.size()>0?&vsolfree[0]:NULL);		
		
			return sol;
		}
			
	private:
		
		bool success;
		std::vector<IKSolution> vsolutions;
		//IKReal eetrans[3];
		//IKReal eerot[9];
		
	
};