/*****************************************************************************
    MOFSFuncTri is part of MOFS.

    MOFS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MOFS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MOFS.  If not, see <http://www.gnu.org/licenses/>.
    
    MOFS by Miguel A. Olivares-Mendez is licensed under a Creative Com-
    mons Attribution-NonCommercial-ShareAlike 3.0 Unported License.

    
    MOFS was created by Miguel A. Olivares-Mendez @ Computer Vision Group,
    Universidad Politecnica de Madrid, Centro de Automatica y Robotica
    
    you can found more information about it in www.vision4uav.eu
*****************************************************************************/

#include <mofs/MOFSFuncTri.h>
//#include "MOFSVar.h"
#include <mofs/MOFSRule.h>
    
#include <cstdlib>
#include <iostream>
using namespace std;

MOFSFuncTri::MOFSFuncTri() {
	// nothing to do ... at the moment
}

MOFSFuncTri::~MOFSFuncTri() {
	// nothing to do ... at the moment
}

float MOFSFuncTri::Calc(float i, float a, float b, bool slope_down) {
	//	now we must get the value of the input in this intervale
	//	based on this function:
	//	
	//	slope down = false -> In this way \ will be	:	T(i;a,b) = (i-a)/(b-a) = p;
	//
	//
	//	slope down = true  -> In the other way / :	T(i;a,b) = (b-i)/(b-a) = q;
	//
	float y;
	if (a!=b){
	  if (slope_down == false) {
	//	cout << "i:" << i <<" a:" << a << " b:" << b << endl;
		y = (i-a)/(b-a);
	  } else {
	//	cout << "i:" << i <<" a:" << a << " b:" << b << endl;
		y = (b-i)/(b-a);
	  }
	}else{  // is the end or begining of a variable --> trapezoidal
	  y = 1.0;
	}
	return y;
}

