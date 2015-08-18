/*****************************************************************************
    MOFSMCDefuzz is part of MOFS.

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

#ifndef MOFSMCDefuzz_H
#define MOFSMCDefuzz_H

#include "MOFSRule.h"
class MOFSMCDefuzz {

		/************************/
		/**  class variables   **/
		/************************/

		public:
			float upper_term;
			float lower_term;
			float output;

	struct RuleNode {
		MOFSRule*	rule;
		RuleNode*	next;
	};

		/***********************/
		/**  member functions  */
		/***********************/

		public:
			MOFSMCDefuzz();
			~MOFSMCDefuzz();
			float defuzz(void* rule_data_list, int var_number);

};

#endif

