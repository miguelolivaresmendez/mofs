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

//#include "MOFSRule.h"
//#include "MOFSFuncTri.h"

//#ifdef _LINUX_VERSION_
#include <cstdlib>
#include <iostream>

#include <stdio.h>
#include <string.h>

#include <mofs/MOFSVar.h>


using namespace std;
//#endif
//#ifndef _LINUX_VERSION_
//#include <stdio.h>
//#endif


/*---------------------------------------------------------------------------
**	Function:	MOFSVar
** 
**	Purpose:	constructor.
** 
**	Arguments:	_name	-->	the name of the var
**			_max_x	-->	the maximus value for sets in the x axis
**			_min	-->	the minumus value for sets in the x axis
**			crisp	-->	numero de conjuntos
**
**	Returns:	nothing
**-------------------------------------------------------------------------*/


MOFSVar::MOFSVar(char* _name, float _max_x, float _min_x, int crisp) {

	strcpy(name,_name);
	cout << "nombre de la var: " << name << "   " << _name;
	max_x		=	_max_x;
	min_x		=	_min_x;
	max_central =	0.0;
	min_central =	0.0;
	crisp_central_value = 0;
	crisp_central = 0;
	max_y		= 1.0;
	min_y		= 0.0;
	values		=	0;
	max_str = 20;

	
	int crisp_value =	crisp;
	cout << "crisp_value: " << crisp_value << " crisp: " << crisp << endl;
//	overlap_index	=	2;
	crisp_index		=	(max_x - min_x)/crisp_value;	// tama�o del intervalo entre un conj y otro

	//char sets[overlap_index -1];			el tama�o de ambos se define en cada iteracion de cada entrada
	//float outputs[overlap_index -1];		en funcion de si la entrada coincide o no con un intervalo

	set_index	=	'A';		// This is the first linguistic value for sets
	cout << "crisp_value: " << crisp_value << " crisp: " << crisp << endl;
	//
	function_type = new MOFSFuncTri();
	x_values_array = new XArray[crisp_value+1];

	calc();


}





MOFSVar::MOFSVar(char* _name, float _max_x, float _min_x, int crisp, float _max_central, float _min_central, int _crisp_central) {
	
	strcpy(name,_name);
	cout << "nombre de la var: " << name << "   " << _name;
	max_x		=	_max_x;
	min_x		=	_min_x;
	max_central =	_max_central;
	min_central =	_min_central;

	crisp_central_value = _crisp_central;
	max_y		= 1.0;
	min_y		= 0.0;
	values		=	0;
	max_str = 20;

	
	int crisp_value =	crisp;
	crisp_index		=	((max_x - min_x) - (max_central-min_central))/crisp_value;

	//  modificar el crisp_index para con los valores absolutos para q se pueda utilizar variables 
	//  q vayan de positivo a negativo

	set_index	=	'A';
	function_type = new MOFSFuncTri();
	if (crisp_central_value > 0) {
			x_values_array = new XArray[crisp_value+crisp_central_value +2];
			crisp_central = (max_central - min_central)/(crisp_central_value +1);
	}else {
			x_values_array = new XArray[crisp_value+1];
			crisp_central = 0;
	}
	calc();
}



MOFSVar::MOFSVar(FILE* file_in, int option) {
	max_str = 20;
	values = 0;
	max_y		= 1.0;
	min_y		= 0.0;
	char		name[30];
	char line[30];
        //int option = 0;
	// vamos leyendo los distintos datos del fichero
    fscanf(file_in,"%s",_name);
	printf("name %s\n",_name);
	strcpy(name,_name);
	function_type = new MOFSFuncTri();
	set_index	=	'A';		// This is the first linguistic value for sets

	if (option == 0) {//se generan automáticamente los conjuntos borrosos
		fscanf(file_in,"%f",&max_x);
		fscanf(file_in,"%f",&min_x);
                fscanf(file_in,"%d",&crisp_value);
		fscanf(file_in,"%f",&max_central);
		fscanf(file_in,"%f",&min_central);
                fscanf(file_in,"%d",&crisp_central_value);
		
		crisp_index = ((max_x - min_x))/crisp_value;	// tama�o del intervalo entre un conj y otro

	
		cout << "nombre var: " << name << ";max_x: " << max_x << ";min_x: " << min_x;
		cout << ";crisp: " << crisp_value << endl << ";max_central: "<<max_central;
		cout << ";min_central: "<< min_central << ";crisp_central: " << crisp_central_value << endl;
		if (crisp_central_value > 0) {
			x_values_array = new XArray[crisp_value+crisp_central_value +2];
			crisp_central = (max_central - min_central)/(crisp_central_value + 1);
		}else {
			x_values_array = new XArray[crisp_value+1];
			crisp_central = 0.0;
		}
		calc();
	}else if (option == 1) {//se leen desde el fichero los distintos conjuntos borrosos
		int counter;
		
		fscanf(file_in,"%d",&counter);
		printf("counter: %d\n",counter);
		x_values_array = new XArray[counter-1];
		for (int i=0;i<counter;i++) {
			fscanf(file_in,"%f",&x_values_array[i].x);
			printf("%f ",x_values_array[i].x);
			set_index++;
		}
		min_x = x_values_array[0].x;
		max_x = x_values_array[counter-1].x;
	}

	
	
}



/*---------------------------------------------------------------------------
**	Function:	~MOFSVar
** 
**	Purpose:	desconstructor.
** 
**	Arguments:	none
**
**	Returns:	nothing
**
**	Comments:	nothing to do right now
**-------------------------------------------------------------------------*/


MOFSVar::~MOFSVar(){
	//nothing to do ...
}





/*---------------------------------------------------------------------------
**	Function:	calc
** 
**	Purpose:	Calc all the values of the x_values_array
** 
**	Arguments:	none
**
**	Returns:	nothing
**
**	Comments:	Calc all the x's values and the set values
**-------------------------------------------------------------------------*/

void MOFSVar::calc() {

		// calc the lenght of array in function of the crisp index and create
		// the array
	MOFSFuncTri* function_type = new MOFSFuncTri();
      //  printf("crisp_value:%d \n",crisp_value);
//	x_values_array[crisp_value+1];
	float next_value	=	min_x;

	int counter = 0;					//	variable to write in all positions
	float crisp = crisp_index;
	if (crisp_central >0) {
		while (next_value <= max_x) {
			if ((next_value >= min_central) && (x_values_array[counter-1].x < min_central)) {
				next_value = min_central;
				crisp = crisp_central;
			}

			x_values_array[counter].x  =	 next_value;
			x_values_array[counter].bit_act = 0;
                        cout << "next value: " << next_value << endl;
			next_value	+=	 crisp;
	
			if ((crisp != crisp_index) && (next_value > max_central)) {
				crisp = crisp_index;
				float j = min_x;
				while (j < next_value) {
					j += crisp;
				}
				next_value = j;
			}

			counter++;
			set_index++;
		}
	}else {
		while (next_value <= max_x) {
			x_values_array[counter].x = next_value;
			//printf("set_index: %d, counter: %d ",(set_index- int('A')), counter);
                        cout << "next value: " << next_value << endl;
			next_value += crisp;
			counter++;
			set_index++;
		}
	}
		// insert another x_point for represent the far away values
	if (counter == crisp_value) {
		x_values_array[counter].x = next_value;
		max_x = next_value;
                cout << "next value: " << next_value << endl;
		set_index++;
	
	}
	//set_index--;
	//printf("set_index final: %d ",(set_index- int('A')));

}


/*---------------------------------------------------------------------------
**	Function:	center
** 
**	Purpose:	Return the center value of the set
** 
**	Arguments:	char set	-->	the set which center is wanted to konw
**
**	Returns:	the center of the set
**-------------------------------------------------------------------------*/


float MOFSVar::center(char set) {

	int center_pos = int (set) - (int ('A'));
	return x_values_array[center_pos].x;
	// solo vale para overlap_index pares

}



float MOFSVar::end_interval(char set) {
	int center_pos = int (set) - (int ('A'));
//	cout << "end interval: " << x_values_array[center_pos + 1] << endl;
	x_values_array[center_pos + 1].bit_act++;
	return (x_values_array[center_pos + 1].x);
}



float MOFSVar::begin_interval(char set) {
	int center_pos = int (set) - (int ('A'));
//	cout << "begin interval: " << x_values_array[center_pos - 1] << endl;
	x_values_array[center_pos - 1].bit_act++;
	return (x_values_array[center_pos - 1].x);
}





/*---------------------------------------------------------------------------
**	Function:	calc_value
** 
**	Purpose:	Calc the sets and y values for the input x
** 
**	Arguments:	none
**
**	Returns:	nothing
**
**	Comments:	Calc all the x's values and the set values
**-------------------------------------------------------------------------*/

int MOFSVar::calc_values(float x) {
	
	//	en primer lugar se calcula el intervalo perteneciente a la x dada
	input = x;
	int counter = 0;		// variable para recorrer el array entero
	int end		= int (set_index) - int ('A');
	values = 0;
	
	
	if (x > max_x) {
		input = max_x;//printf("El valor es mayor %f,input=%f, end=%d\n",x,input, end);
	}else if (x < min_x) {
		input = min_x;//printf("El valor es menor %f,input=%f, end=%d\n",x,input, end);
	}
	while ((counter <= end) && (input > x_values_array[counter].x)) {
		//printf("counter: %d, x_value%f\n",counter, x_values_array[counter].x);
		counter++;
	}

	if (counter > end) {
		counter = end;
	}
	//	if (counter <= end) {
	//printf("counter: %d, x_value%f\n",counter, x_values_array[counter].x);
	int sets_begining = counter - 1;		// pos inicial
	int sets_ending	= counter;		// pos final

	if (sets_begining < 0) {
			// Entonces el valor est� entre los intervalos iniciales
			// donde el solapamiento es menor por lo tanto el array tambien ser� menor
		sets_begining = 0;
	}
	if (sets_ending > end) {
		sets_ending = end;
			// Pertenece a los �ltimos conjuntos que tienen menos �ndice de solapamiento
	}
	//printf("inicio: %d, fin: %d\n",sets_begining,sets_ending);
	for (int next_set = sets_begining; next_set <= sets_ending; next_set++) {
			VarValues* data = new VarValues;
			data->set	= char (next_set + (int ('A')));
			float c = center(data->set);
	//		cout << "c: " << c << "next_set: " << next_set << endl;
			if (c < input) {
				// pendiente hacia abajo
				float b = end_interval(data->set);//printf("data->set: %c. b:%f, end:%d ",data->set,b, end);
				bool slope_down = true;
				data->y	= function_type->Calc(input,c,b,slope_down);
			}else {
				float a = begin_interval(data->set);//printf("data->set: %c. a:%f, end:%d",data->set,a,end);
				bool slope_down = false;
				data->y	= function_type->Calc(input, a,c, slope_down);
			}
			if (values == 0) {
				data->next = 0;
				values = data;
			}else {
				data->next = values;
				values = data;
			}
			
			//printf("center: %f. x: %f, input: %f, y: %f\n",c,x, input, data->y);
		
	}
/*	} else {
		// Es un numero mayor que el �ltimo valor del x_values_array
		VarValues* data = new VarValues;
		data->set = set_index;
		data->y = 1.0;
		data->next = 0;
		values = data;
	}*/
	//	printf("------------------------\n");	
	return 1;
}


/*---------------------------------------------------------------------------
**	Function:	correctValue
** 
**	Purpose:	check if a set is set value is valid for the last entry
** 
**	Arguments:	set	-->	The set we want to know
**
**	Returns:	-1	-->	If there is no entry
**				 0	-->	If the set is not valid
**				 1	-->	valid set
**-------------------------------------------------------------------------*/

int MOFSVar::correctValue(char set) {
	if (values == 0) {
		//	There aren't values
		return -1;
	}
	VarValues* aux_values = values;
	while (aux_values != 0) {
		if (aux_values->set == set) {
			return 1;
		}
		aux_values = aux_values->next;
	}
	return 0;
}


float MOFSVar::yValue(char set) {
	if (values == 0) {
		//	There aren't values
		return -1;
	}
	VarValues* aux_values = values;
	while (aux_values != 0) {
		if (aux_values->set == set) {
			return aux_values->y;
		}
		aux_values = aux_values->next;
	}
	return 0.0;
}


/*---------------------------------------------------------------------------
**	Function:	setValue
** 
**	Purpose:	return the set of the minimus y value
** 
**	Arguments:	none
**
**	Returns:	char set
**-------------------------------------------------------------------------

char MOFSVar::setValue() {
	if (output1 < output2) {
		return set1;
	} else {
		return set2;
	}
}

*/

/*---------------------------------------------------------------------------
**	Function:	values
** 
**	Purpose:	return the direction of the begining of the values list
** 
**	Arguments:	none
**
**	Returns:	nothing
**-------------------------------------------------------------------------
int MOFSVar::values_array() {
	return values;
}

*/


		/************************/
		/**  trivial functions  */
		/************************/

int MOFSVar::values_lenght() {
	int counter = 0;
	VarValues* aux_values = values;
	while (aux_values != 0) {
		counter++;
		aux_values = aux_values->next;
	}
	return counter;
}

char MOFSVar::centerSet() {

	char center_set;
	float center_y = 0.0;
	VarValues* aux_values = values;
	while (aux_values != 0) {
		if (aux_values->y >center_y) {
			center_y = aux_values->y;
			center_set = aux_values->set;
		}
		aux_values = aux_values->next;
	}
	return center_set;
}


void MOFSVar::SetsArray(char sets_array[]) {

	VarValues* aux_values = values;
//	int lenght = values_lenght();
//	sets_array = new char [lenght];
	int counter = 0;
	while (aux_values != 0) {
		sets_array[counter] = aux_values->set;
		aux_values = aux_values->next;
		counter++;
	}	

}


void MOFSVar::Outputs(float y_array[]) {
	VarValues* aux_values = values;
//	int lenght = values_lenght();
//	y_array = new float[lenght];
	int counter = 0;
	while (aux_values != 0) {
		y_array[counter] = aux_values->y;
		aux_values = aux_values->next;
		counter++;
	}	
}

void MOFSVar::SalvarIncidencia(FILE* file_out) {
	fprintf(file_out," set index: %d\n",set_index);
	for (int i=0;i<(int(set_index) - int('A'));i++) {
		fprintf(file_out,"%c --> %c",char(i+ int('A')), x_values_array[i].bit_act);
	}
}

			
float MOFSVar::xValue() {
	return input;
}


char MOFSVar::setIndex() {
	return set_index;
}

// MOFSVar(char* _name, float _max_x, float _min_x, int crisp, int _overlap);


void MOFSVar::varName(char n[]) {
	int i = 0;
	while ((i < 20) && (_name[i] != 0)) {
		n[i] = _name[i];
		i++;
	}
}


void MOFSVar::VName(char name_[]) {
	strcpy(name,name_);
}
float MOFSVar::maxX() {
	return max_x;
}

float MOFSVar::minX() {
	return min_x;
}

float MOFSVar::crispIndex() {
	return crisp_value;
}


float MOFSVar::maxCentral() {
	return max_central;
}

float MOFSVar::minCentral() {
	return min_central;
}

float MOFSVar::crispCentral() {
	return crisp_central_value;
}
