/*****************************************************************************
    MOFSVar is part of MOFS.

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
#ifndef MOFSVar_H
#define MOFSVar_H

#include "MOFSFuncTri.h"

#include <stdio.h>

	class MOFSVar {

		

		/**********************/
		/*  class variables   */
		/**********************/
		
		int array_size;
		public: 
		struct VarValues {
					char		set;
					float		y;
					VarValues*	next;
		};

		struct XArray {
					float		x;
					int			bit_act;
		};
		private:
			int max_str;

			char		name[30];
			char		_name[30];
			MOFSFuncTri	 *function_type;
			
			float		max_x, min_x, max_y, min_y, max_central, min_central;
			XArray*		x_values_array;
			int			overlap_index;		// indice de solapamiento de conjuntos par un valor x dado
			int			crisp_value;		// tama�o del array. Define el crisp_index en funcion del
											// tama�o del intervalo total de la var
			float		crisp_index;		// cantidad de intervalos en toda la var
			float		set_lenght;			// tama�o de los subconjuntos
			int			crisp_central_value;
			float		crisp_central;
			char		set_index;
			VarValues*	values;
			float		input;				// The last input for this variable
	/*		char		sets[];			// All the set's possibles fuzzy result for the input
			float		output[];		// All the output's possibles float result
					Se definen dentro, dejar el array abierto para despues darle un tama�o da error
	*/
		/***********************/
		/*  member functions  */
		/***********************/

		public:

			MOFSVar(char* _name, float _max_x, float _min_x, int crisp);
			MOFSVar(char* _name, float _max_x, float _min_x, int crisp, float _max_central, float _min_central, int _crisp_central);
                        MOFSVar(FILE* file_in, int option);
			~MOFSVar();

			void calc();
			float center(char set);
			float end_interval(char set);
			float begin_interval(char set);
			int calc_values(float x);
		//	int values_array();
			int correctValue(char set);
			float yValue(char set);
			float xValue();
			char centerSet();		// devuelve el conjunto cuyo valor es mayor (mas centrado) para la ultima entrada dada
			char setIndex();
			int values_lenght();
			void SetsArray(char sets_array[]);
			void Outputs(float y_array[]);
			void SalvarIncidencia(FILE* file_out);

			void varName(char n[]);
			void VName(char name_[]);
			float maxX();
			float minX();
                        float crispIndex();
			float maxCentral();
			float minCentral();
            float crispCentral();

			
	};
#endif
