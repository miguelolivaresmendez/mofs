/*****************************************************************************
    MOFSRule is part of MOFS.

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

    you can found more information about it in www.vision4uav.eu											**
*****************************************************************************/

#include <string>
#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>


using namespace std;

#include "MOFSRule.h"


/**\brief--------------------------------------------------------------------------
**	Function:	MOFSRule
** 
**	Purpose:	1st constructor.
** 
**	Arguments:	int num_inputs_, char _inputs[],char _output
**
**	Returns:	nothing
**
**	Comments:	Insert all the data of the inputs and the output
**				and put the default value of weight
**-------------------------------------------------------------------------*/


MOFSRule::MOFSRule(int num_inputs_, char _inputs[],char _output) {

	max_gens = num_inputs_;
	Set* data = new Set;
	data = 0;
	increm = 0.3;
	decrem = 0.35;


	output = _output;
	int num_inputs = num_inputs_;
	inputs = new char[num_inputs];
    for (int i=0;i<num_inputs;i++) {
		inputs[i] = _inputs[i];
	}
	
	Set* new_data = new Set;		// first element;
	new_data->set = _inputs[0];
	new_data->next = 0;
	data = new_data;
	Set* aux_data = data;

	for (int j=1; j<num_inputs-1;j++) {
		Set* new_data = new Set;
		new_data->set = _inputs[j];
		new_data->next = 0;
		aux_data->next = new_data;
		aux_data = aux_data->next;
	}

	Set* new_data_output = new Set;
	new_data_output->set = _output;
	new_data_output->next=0;
	aux_data->next = new_data_output;
	weight	=	0.3;
	bit_act	=	0;
}
/**\brief--------------------------------------------------------------------------
**	Function:	MOFSRule
**
**	Purpose:	2nd constructor.
**
**	Arguments:	int num_inputs_, char _inputs[], char _output, float _weight
**
**	Returns:	nothing
**
**	Comments:	Insert all the data of the inputs, the output
**				and the value of weight
**-------------------------------------------------------------------------*/
MOFSRule::MOFSRule(int num_inputs_,char _inputs[],char _output, float _weight) {

	max_gens = num_inputs_;
	Set* data = new Set;
	data = 0;
	increm = 0.3;
	decrem = 0.35;
	output = _output;
	int num_inputs = num_inputs_;
	inputs = new char[num_inputs];
        for (int i=0;i<num_inputs;i++) {
		inputs[i] = _inputs[i];
	}
	
	Set* new_data = new Set;		// first element;
	new_data->set = _inputs[0];
	new_data->next = 0;
	data = new_data;

	Set* aux_data = data;

        for (int j=0; j<num_inputs;j++) {
		Set* new_data = new Set;
		new_data->set = _inputs[j];
		new_data->next = 0;
		aux_data->next = new_data;
		aux_data = aux_data->next;
	}

	Set* new_data_output = new Set;
	new_data_output->set = _output;
	new_data_output->next=0;
	aux_data->next = new_data_output;


	weight	=	_weight;
	bit_act	=	0;

}
		 		 
/**\brief--------------------------------------------------------------------------
**	Function:	~MOFSRule
** 
**	Purpose:	destructor.
** 
**	Arguments:	none
**
**	Returns:	nothing
**
**	Comments:	This action is done it by the RuleStore
**-------------------------------------------------------------------------*/
	 
MOFSRule::~MOFSRule() {
	//nothing to do
}

/**\brief--------------------------------------------------------------------------
**	Function:	updateWeight
** 
**	Purpose:	Increase or decreas the weight of the rule that was selected
** 
**	Arguments:	char _inputs[], char _output[], float output_values[], int _num_inputs
**
**	Returns:	int	 1	--->	weight has been modified
**					 1	--->	not modified
**
**	Comments:	If all the values are equals increment the weight 0.2,
**				If just the inputs are equals decrease the weight 0.1
**				otherwise don't do nothing
**-------------------------------------------------------------------------*/

int MOFSRule::updateWeight(char _inputs[], char _output[], float output_values[], int _num_inputs){

	bool	equal = true;		// while it is true, we will continue comparing the variables
	int		counter = 0;
	int		output_counter = 0;
	float	output_to_change, value_output_to_change;

	
//	Set* aux_data = data;
	int num_inputs = _num_inputs;
	int num_output = 2;


	while (equal && (counter < (max_gens-1))) {	// "output" variable is not compared
		if (inputs[counter] != _inputs[counter]) {
			equal = false;
		}else {

		}
		counter++;
	}

	if (equal == false) {
		return -1;
	}
	output_to_change = _output[output_counter];
	value_output_to_change = output_values[output_counter];
	IncBitAct();
	while (output_counter <= num_output-1) {
	//	cout << output << "==" << _output[output_counter] << "?" << endl;
		if ((output == _output[output_counter]) && (output_values[output_counter] > 0.0)) {
			// This rule has success in this iteration
			weight += output_values[output_counter]/4;
			if (weight > 3.0) weight = 3.0;
			//cout << "Peso incrementado. Regla: ";
			for (int ipts_counter = 0;ipts_counter < num_inputs; ipts_counter++) {
			//	cout << inputs[ipts_counter];
			}
			//cout << output << endl;
			if (output_values[output_counter]>value_output_to_change){
				output_to_change = _output[output_counter];
				value_output_to_change = output_values[output_counter];
			}
			return 1;
		}

		output_counter++;
	}
	weight -= decrem;
	//cout << "Peso decrementado. Regla: ";
	if (weight<0.0){
		updateOutput(output_to_change);
	}

	for (int ipts_counter = 0;ipts_counter < num_inputs; ipts_counter++) {
			//	cout << inputs[ipts_counter];
			}
	//cout << output << endl;
	return 1;
}




/**\brief--------------------------------------------------------------------------
**	Function:	updateOutput
** 
**	Purpose:	change the output value
** 
**	Arguments:	char _output
**
**	Returns:	int	-1	--->	error
**					 1	--->	otherwise
**-------------------------------------------------------------------------*/

int MOFSRule::updateOutput(char _output){

	bool	equal = true;		// while it is true, we will continue comparing the variables
	int		counter = 0;

	Set* aux_data = data;

	output = _output;
	weight = 0.5;
	return 1;

}
/**\brief--------------------------------------------------------------------------
**	Function:	compare
** 
**	Purpose:	Compare if the selected rule have the same inputs that one passed as argument
** 
**	Arguments:	char _inputs[]
**
**	Returns:	char	--->	the set value of the output
**
**	Comments:	return the output value if the selected rule is the one with the same inputs of the argument,
**				otherwise return 'Z' (zero)
**-------------------------------------------------------------------------*/

char MOFSRule::compare(char _inputs[])	{

	bool	equal = true;		// mientras sea true se seguiran comprobando las vars de la regla
	int		counter = 0;

	Set* aux_data = data;
	while (equal &&  (counter < (max_gens -1)) ) {	// no comparamos la var "output"
		if (aux_data->set != _inputs[counter]) {
			equal = false;
		}else {
			aux_data = aux_data->next;
		}
	}

	if (equal == false) {
		return 'Z';
	}
	return aux_data->set;

}

void MOFSRule::Sets(void* sets) {
	sets = data;
}

/**\brief
 * increment by 1 of the times that this rule is used
 **/
void MOFSRule::IncBitAct() {
	bit_act++;
}

		/************************/
		/*  trivial functions  */
		/************************/



/**\brief
 * return the weight assigned to a specific rule
 **/
float MOFSRule::Weight() {
	return weight;
}
/**\brief
 * return the of times this rules was activated
 **/
int MOFSRule::BitAct() {
	return bit_act;
}

/**\brief
 * store the values of the inputs of each specific situation
 **/
void MOFSRule::inputs_array(char _inputs[]) {
	for (int i=0; i<=max_gens; i++) {
		_inputs[i] = inputs[i];
	}
}
/**\brief
 * return the output value of this rule
 */
char MOFSRule::outputValue() {
	return output;
}

/**\brief
 * return the number of input of this rule
 */
int MOFSRule::num_inputs() {
	return max_gens;
}
/**\brief
 * Initialize the rule weight
 */
void MOFSRule::init(){
	weight = 0.3;
}
/**\brief
 * return the specific value of the first input
 */
char MOFSRule::input0() {
	return inputs[0];
}
/**\brief
 * return the specific value of the second input
 */
char MOFSRule::input1() {
	return inputs[1];
}
/**\brief
 * return the specific value of the third input
 */
char MOFSRule::input2() {
	return inputs[2];
}

