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
    
	Now @ SnT University of Luxembourg: miguel.olivaresmendez@uni.lu

    you can found more information about it in www.vision4uav.eu
*****************************************************************************/

//#include "MOFSMCDefuzz.h"
//#include "MOFSVar.h" 

#include "MOFSModel.h"

//#ifdef _LINUX_VERSION_
#include <cstdlib>
#include <iostream>
using namespace std;
//#endif

MOFSMCDefuzz::MOFSMCDefuzz() {
	upper_term = 0.0;
	lower_term = 0.0;
	output = 0.0;
}

MOFSMCDefuzz::~MOFSMCDefuzz() {
	// nothing to do
}

float minimus(float min, float v) {
	if (v<min) return v;
	return min;
}

/**
 * \brief return the defuzzyfication value using this method and the product inference process
 */

float MOFSMCDefuzz::defuzz(void* rule_data_list, int var_number) {

	mofs::RuleData* aux_rule_data_list = static_cast <mofs::RuleData*> (rule_data_list);
	upper_term = 0.0;
	lower_term = 0.0;
	
/*	
	
	float minimun_rule;

	while (aux_rule_data_list != 0) {
		minimun_rule = aux_rule_data_list->inputs_values[0];
		for (int i = 1; i <var_number;i++) {
			minimun_rule = minimus(minimun_rule,aux_rule_data_list->inputs_values[i]);
		}
		minimun_rule *= (aux_rule_data_list->weight) / 3.0;
		lower_term += minimun_rule;
		upper_term += minimun_rule * aux_rule_data_list->rule_output;
		aux_rule_data_list = aux_rule_data_list->next;
	}

*/


///*
	while (aux_rule_data_list != 0) {
		
		float multiplicator_rule = aux_rule_data_list->inputs_values[0];
		//printf("defuzz: ");
		for (int i = 1; i<var_number;i++) {
		  if (!(aux_rule_data_list->inputs_values[i]==0.0)&&(aux_rule_data_list->inputs_values[i]==-0.0)){
			//printf("%f* ",aux_rule_data_list->inputs_values[i]);
			multiplicator_rule *= aux_rule_data_list->inputs_values[i];
		  }
		}
		multiplicator_rule *= (aux_rule_data_list->weight) / 1.0;
		lower_term += multiplicator_rule;
		upper_term += (multiplicator_rule*aux_rule_data_list->rule_output);
		aux_rule_data_list = aux_rule_data_list->next;
	}

//*/
//  printf("numerador%f, denominador%f\n",upper_term, lower_term);
	output = upper_term/lower_term;
//	cout << "Output: " << output << endl;
	
//	cin.get();
	return output;
}
