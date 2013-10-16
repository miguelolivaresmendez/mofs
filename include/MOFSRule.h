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
    
    you can found more information about it in www.vision4uav.eu								**
*****************************************************************************/

#ifndef MOFSRule_H
#define MOFSRule_H

using namespace std;

class MOFSRule {
	public:
		struct Set {
			char set;
			Set* next;
		};
		/************************/
		/*  class variables   */
		/************************/

		private:

			int	max_gens;
			Set*	data;
			float	weight;
			int	bit_act;		// bit de activacion, contabiliza las veces q ha sido 
									// seleccionada la regla
			float increm, decrem;
			char *inputs;
			char ipts;
			char output;


		/***********************/
		/*  member functions  */
		/***********************/

		public:
			
			// constructor/destructor
			MOFSRule(int num_inputs_,char _inputs[],char _output);
			MOFSRule(int num_inputs_,char _inputs[],char _output, float _weight);
			~MOFSRule();

			// acciones
			int updateWeight(char _inputs[], char _output[], float output_values[], int _num_inputs);	// Se le envia toda las output validas para esa entrada
																						// y se incrementar� el peso en funci�n del valor de dicha output
			int updateOutput(char _output);			// Se le envia la output predominante y en su caso de tener peso 0.0 se le cambia la output
			char compare(char _inputs[]);		
			// return the output value if the arguments, and the rule values are equals
			void IncBitAct();
			int BitAct();
			void Sets(void* sets);
			float Weight();
			void inputs_array(char _inputs[]);
//			char *ipt();
			char outputValue();
			int num_inputs();
			void init();
			char input0();
			char input1();
			char input2();

};
#endif
