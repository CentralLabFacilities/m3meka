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

%module ik_a2r2_left
%{
	#include "ik_a2r2_left_for_swig.h"
%}
		
	%include "typemaps.i"
	%include "std_vector.i"

	namespace std
	{
		%template(DoubleVector) vector<double>;
		%template(IntVector) vector<int>;
	}

	
			
	class IKSolver {
		public:
			bool Solve(const std::vector<double> veetrans, const std::vector<double> veerot, double free);
			int GetNumSolutions();
			std::vector<double> GetSolution(int i, std::vector<double> free_angles);
			std::vector<int> GetFree(int i);	
	};
