/*****************************************************************************
    MOFSModel is part of MOFS.

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


#include <MOFSModel.h>
#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <string>


using namespace mofs;
/**\brief---------------------------------------------------------------------------
**	Function:	MOFSModel
** 
**	Purpose:	constructor.
** 
**	Arguments:	none
**
**	Returns:	nothing
**-------------------------------------------------------------------------*/
MOFSModel::MOFSModel() {
  
}

/**\brief---------------------------------------------------------------------------
**	Function:	~MOFSModel
**
**	Purpose:	desconstructor.
**
**	Arguments:	none
**
**	Returns:	nothing
**-------------------------------------------------------------------------*/

MOFSModel::~MOFSModel() {

	while (fuzzy_input_vars != 0) {
		VarsNode *aux_vars =  fuzzy_input_vars;
		fuzzy_input_vars = aux_vars->next;
		delete aux_vars;
	}


	while (rule_store != 0) {
		RuleNode *aux_rules = rule_store;
		rule_store = aux_rules->next;
		delete aux_rules;
	}
}

void MOFSModel::open1(int _mode) {
	fuzzy_input_vars	= 0;
	fuzzy_output_vars	= 0;
	rule_data_list		= 0;
	rule_store = 0;
	num_vars = 0;
	num_inputs = 0;
	mode = _mode;
	defuzz_method = new MOFSMCDefuzz();
/*	ofstream pesos("pesos1.txt");
	ofstream error_out("error1.txt");
	ofstream vars_out("vars_out1.txt");
	if (!pesos)  cout << "error creando el fichero de pesos!";
	if (!error_out)  cout << "error creando el fichero de error!";
	if (!vars_out)  cout << "error creando el fichero de vars_out!";*/
	loadVars();
	genRulebase();
}


void MOFSModel::open2(char *fileVars, char *fileRules, int _mode, int genRules, int option4CreateVars) {
        FILE* file_inVars;
        file_inVars = fopen(fileVars,"r");
        if (!file_inVars) {
		fprintf(stderr,"[!]Error abriendo el fichero de datos!\n");
	}else {
		fuzzy_input_vars	= 0;
		fuzzy_output_vars	= 0;
		rule_data_list		= 0;
		rule_store = 0;
		num_vars = 0;
		num_inputs = 0;
		mode = _mode;//mode = 0 --> manual training. ask to the user if the output is correct or not
			     //mode = 1 --> learning proccess active
			     //mode = 2 --> normal process
		defuzz_method = new MOFSMCDefuzz();
                readVars(file_inVars, option4CreateVars);

        //      showVars();
	//
                if (genRules==1){
                    genRulebase();
                }else{
                    FILE* file_inRules;
                    file_inRules = fopen(fileRules,"r");
                    if (!file_inRules) {
                            cout << "error abriendo el fichero de datos!";
                    }else {
                        readRulebase(file_inRules);
                    }
                }
		fclose(file_inVars);
		error_out = fopen("errorPrueba.txt","a+");

	}
}



/**\brief--------------------------------------------------------------------------
**	Function:	readData
** 
**	Purpose:	Auxiliar function for read the values to create a variable
** 
**	Arguments:	none
**
**	Returns:	nothing
**-------------------------------------------------------------------------*/

void MOFSModel::readData(float &max_x, float &min_x, int &crisp) {
	//float num[2];
	cout << "insert the max value of x" << endl;
	cin >> max_x;
	cout << "insert the min value of y" << endl;
	cin >> min_x;
	cout << "insert the crisp value (differents values for the variable)" << endl;
	cin >> crisp;
	cout << "crisp: " << crisp << endl;
}



void MOFSModel::insertVar(VarsNode* new_var, char option) {
	if (option == 'I') {
		if (fuzzy_input_vars == 0) {
			// primera variable
			fuzzy_input_vars = new_var;
		} else {
			// inserta la variable por detras
			VarsNode* aux_inputs = fuzzy_input_vars;
			while (aux_inputs->next != 0) {
				aux_inputs = aux_inputs->next;
			}
			aux_inputs->next = new_var;
		}
	}else{
		//output var
		if (fuzzy_output_vars == 0) {
			fuzzy_output_vars = new_var;
		} else {
			VarsNode* aux_output = fuzzy_output_vars;
			while (aux_output->next != 0) {
				aux_output = aux_output->next;
			}
			aux_output->next = new_var;
		}
	}
}

/**\brief--------------------------------------------------------------------------
**	Function:	loadVars
** 
**	Purpose:	Read the main values for create the variables
** 
**	Arguments:	none
**
**	Returns:	-1	-->	error
**				1	--> otherwise
**-------------------------------------------------------------------------*/
int MOFSModel::loadVars() {


	float max_x, min_x;
	int  crisp;
	char name[20];
	//char basura;
	char option;

	while (1) {

		fflush(stdin);
		VarsNode* new_var = new VarsNode;
		cout << "Insert the name of the variable: ";
		cin.getline(name,20);
		cout << name << endl;
		if (cin.gcount() <= 1) {
			cout << "no more vars";
			return 1;	// no more variables
		}
		do {
			cout << "Insert I or O for Input/Output variable: ";
			cin >> option;
		}while ((option != 'I') && (option != 'O'));		
		readData(max_x, min_x, crisp);
		new_var->var = new MOFSVar(name,max_x, min_x, crisp);
		new_var->next = 0;
		new_var->values = 0;
		insertVar(new_var, option);
		if (option == 'I') {
			num_inputs++;
		}
		cin.get();
		fflush(stdin);
	}
}


//int MOFSModel::readVars(FILE* file_in, int option4CreateVars) {
int MOFSModel::readVars(FILE* file_in, int option4CreateVars) {	
	char var_option;
	fscanf(file_in,"%d",&num_vars);
	printf("numvars:%d\n",num_vars);
 //      fscanf(file_in,"%s",&var_option);
	for (int i = num_vars; i>0; i--) {
		VarsNode* new_var;
		new_var = new VarsNode;


		fscanf(file_in,"%s",&var_option);
                printf("\nvar_option: %c\n",var_option);
//                new_var->var = new MOFSVar(file_in, option4CreateVars);
		new_var->var = new MOFSVar(file_in, option4CreateVars);
		new_var->next = 0;
		new_var->values = 0;

		insertVar(new_var,var_option);
		if (var_option == 'I') {
			num_inputs++;
		}
	}

	cout << "num_inputs: " << num_inputs << endl;
//	cin.get();
	return 1;
}

/**\brief--------------------------------------------------------------------------
**	Function:	genRule
** 
**	Purpose:	Generate each rule based on the input vars
** 
**	Arguments:	none
**
**	Returns:	void
**
**-------------------------------------------------------------------------*/

void MOFSModel::genRule(char inputs_[], VarsNode* aux, int cont_vars){
    if (cont_vars>=num_inputs){ //caso base
     aux = fuzzy_output_vars;
     char max_index= aux->var->setIndex();
     int outputRandom = rand()%(max_index-65);
     RuleNode* new_rule = new RuleNode;
     new_rule->rule = new MOFSRule(num_inputs,inputs_, ((char)outputRandom+65));
     new_rule->next = rule_store;
     rule_store = new_rule;
     int tam=10;
     char itp[num_inputs];
    // printf("cont_var:%d, tamaño inputs:%d, tamañonitp=%d\n",cont_vars, sizeof(inputs_),sizeof(itp));
     //printf("inputs: %c,%c,%c,%c,%c\n",inputs[0],inputs[1],inputs[2],inputs[3],inputs[4]);
    }else{
      char max_index= aux->var->setIndex();
      for (char i='A';i<max_index;i++) {
	inputs_[cont_vars]=i;
;
      	VarsNode* aux_ = fuzzy_input_vars;
	aux_ = aux->next;
	genRule(inputs_, aux_, cont_vars+1);
      }
    }
}


/**\brief--------------------------------------------------------------------------
**	Function:	genRuleBase
** 
**	Purpose:	Generate a full rule-base based on the input vars
** 
**	Arguments:	none
**
**	Returns:	-1  --> error
**				1	--> otherwise
**
**-------------------------------------------------------------------------*/

int MOFSModel::genRulebase() {
  srand(time(0));
  VarsNode* aux = fuzzy_input_vars;
  char max_index= aux->var->setIndex();
  char inputs_[num_inputs];
 // printf("numero de entradas= %d, %d\n",num_inputs,sizeof(inputs_));
    int cont_vars=0;
  genRule(inputs_, aux, cont_vars);
 // printf("numero de entradas= %d, %d\n",num_inputs,sizeof(inputs_));
  return 1;

}



int MOFSModel::readRulebase(FILE* file_in)
{
	if (!file_in) { 
		cout << "error al leer del fichero"; 
		cin.get();
		return 1;
	}
	char *inputs = new char[num_inputs];
   //     printf("num_inputsReadRuleBase: %d ", num_inputs);
	char output;
	float weight;
		
	while (!feof(file_in)) {
		RuleNode* new_rule = new RuleNode;
		fscanf(file_in,"%s",inputs);
        //        printf("inputs: %s", inputs);
		fscanf(file_in,"%c",&output);fscanf(file_in,"%c",&output);
		fscanf(file_in,"%f",&weight);
		new_rule->rule = new MOFSRule(num_inputs,inputs,output,weight);
		new_rule->next = rule_store;
		rule_store = new_rule;
	}
	return 1;
}

int MOFSModel::learning(char* file) {
	FILE* file_learning;
	file_learning = fopen(file,"r");
	if (!file_learning) { cout << "error abriendo el fichero de datos!";return -1;
	}else {
		int __num_inputs;
		float _output;
		float* output;
		float* ipts;

		fscanf(file_learning,"%d",&__num_inputs);
		if (__num_inputs!=num_inputs){
		  printf("ERROR!!!");
		}
		ipts = new float[num_inputs];
		while (!feof(file_learning)) {
			for (int i=0;i<num_inputs;i++) {
				fscanf(file_learning,"%f",&ipts[i]);
			}
			fscanf(file_learning,"%f",&_output);
			output = &_output;
			inputIrq(ipts,output);

		}
		delete(ipts);
	}
	fclose(file_learning);
	return 1;
}

/**\brief--------------------------------------------------------------------------
**	Function:	inputIrq
** 
**	Purpose:	Active function of the robot behavior
** 
**	Arguments:	none
**
**	Returns:	system action
**
**	Comments:	fill in the rule_data array 
**-------------------------------------------------------------------------*/


float MOFSModel::inputIrq(float inputs[],float* _output) {
  
	rule_data_list = 0;	// resetea la lista de reglas obtenidas con las entradas de la vez anterior
	//printf("MOFS.inputs: %f, %f, %f :: %f\n",inputs[0],inputs[1],inputs[2], _output);
	// ahora calculamos todos los posibles valores fuzzy y resultados float para
	// las entradas dadas ---> crear lista de cromosomas v�lidos

	VarsNode* aux = fuzzy_input_vars;			// auxiliar para recorrer la lista de entradas
	int counter		= 0;							// variable para recorrer el array de entradas
	while (counter < num_inputs) {	
	  	// numero de variables de entrada en el sistema
		aux->var->calc_values(inputs[counter]);
		aux = aux->next;
		counter ++;
	}
	aux = fuzzy_input_vars;
	getValidRules();
	return Action(_output);

}
	

/**\brief--------------------------------------------------------------------------
**	Function:	getValidRules
** 
**	Purpose:	get all the rules possibles based on the inputs
** 
**	Arguments:	none
**
**	Returns:	int
**
**	Comments:	fill the rules in one array of rule_datas
**-------------------------------------------------------------------------*/	

void MOFSModel::getValidRules()	{		// obtiene el array de reglas v�lidas para las entradas dadas.

	RuleNode* aux_rule	= rule_store;			// Variable para recorrer la base de reglas

	
//	char*	rule_sets;
	char *ipts;
	ipts = new char[num_inputs];
	while (aux_rule != 0) {
			// toma los valores de los conjuntos de esa regla
//			char *ipts;
//			ipts = new char[num_inputs];
			aux_rule->rule->inputs_array(ipts);
			char rule_output = aux_rule->rule->outputValue();

			VarsNode* aux_vars	= fuzzy_input_vars;	// Variable para recorrer las entradas
			int counter = 0;
			bool	ok	= true;

			while (ok && (aux_vars != 0)) {
				if (aux_vars->var->correctValue(ipts[counter])) {
					// chequea cada variable de la regla si coincide con las entradas pasadas  
					aux_vars = aux_vars->next;
					counter++;
				}else {
					ok = false;
				}
			}

			if (ok==true)  {
				// si coinciden todos los valores de las variables, se guarda el cromosoma de dicha regla
				saveRule(ipts,rule_output,aux_rule->rule->Weight());
			}
			aux_rule = aux_rule->next;
	}
	delete(ipts);
}


void MOFSModel::insertRuleVar(RuleValues* &r_values,RuleValues* rv) {

		if (r_values == 0) {
			// primera variable
			r_values = rv;
		} else {
			// inserta la variable por detras
			RuleValues* aux_rule_values = r_values;
			while (aux_rule_values->next != 0) {
				aux_rule_values = aux_rule_values->next;
			}
			aux_rule_values->next = rv;
		}
}
	
void MOFSModel::showRules() {
	RuleNode *aux_rules = rule_store;
	char *ipts;
	ipts = new char[num_inputs];
	cout << "numero de inputs= " << num_inputs << endl;
	while (aux_rules != 0) {
		aux_rules->rule->inputs_array(ipts);
		cout << "inputs: ";
// 		for (int i =0;i<num_inputs;i++) {
// 			cout << ipts[i] << " ";
// 		}
	//	printf("inputs: %c,%c,%c,%c,%c\n",ipts[0],ipts[1],ipts[2],ipts[3],ipts[4]);
		cout << " output: " << aux_rules->rule->outputValue() <<  " weight: " << aux_rules->rule->Weight() <<endl;
		aux_rules = aux_rules->next;
	}
}

/**\brief--------------------------------------------------------------------------
**	Function:	saveRule
** 
**	Purpose:	save the 2 set and y values of each valid rule
** 
**	Arguments:	rule_sets	-->	the values of the rule
**
**	Returns:	int
**
**	Comments:	save the rules in one array of rule_datas structure
**-------------------------------------------------------------------------*/

void MOFSModel::saveRule(char ipts[],char rule_output, float _weight) {
//	MOFSRule::Set* rule_sets = static_cast<MOFSRule::Set*> (ruleSets);
	RuleData* rule_data = new RuleData;
	VarsNode* aux_vars	= fuzzy_input_vars;		// Variable para recorrer las entradas

//	cout << "Regla valida: ";
	rule_data->inputs_sets = new char[num_inputs];
	rule_data->inputs_values = new float[num_inputs];
	for (int i=0; i<num_inputs;i++) {
		rule_data->inputs_sets[i] = ipts[i];
		rule_data->inputs_values[i] = aux_vars->var->yValue(ipts[i]);
//		cout << rule_data->inputs_sets[i] << " " << rule_data->inputs_values[i] << " ; ";
		aux_vars = aux_vars->next;
	}
	rule_data->weight = _weight;
	rule_data->rule_set_output	=	rule_output;		// Store the output
	rule_data->rule_output	=	fuzzy_output_vars->var->center(rule_output);

	if (rule_data_list == 0) {
		// es la primera regla q insertamos
		rule_data->next	=	rule_data_list;
		rule_data_list		=	rule_data;
	} else {
		RuleData *aux = rule_data_list;
		while (aux->next != 0) {
			aux = aux->next;
		}
		rule_data->next = 0;
		aux->next = rule_data;
	}
}	

void MOFSModel::initRulestore() {
	RuleNode* aux_rulestore = rule_store;
	while (aux_rulestore!= 0) {
		aux_rulestore->rule->init();
		aux_rulestore = aux_rulestore->next;
	}
}

void MOFSModel::SaveWeightStats() {
	
	A_rules_S = fopen("A_rules_S.txt","a+");
	B_rules_S = fopen("B_rules_S.txt","a+");
	C_rules_S = fopen("C_rules_S.txt","a+");
	D_rules_S = fopen("D_rules_S.txt","a+");
	A_rules_N = fopen("A_rules_N.txt","a+");
	B_rules_N = fopen("B_rules_N.txt","a+");
	C_rules_N = fopen("C_rules_N.txt","a+");
	D_rules_N = fopen("D_rules_N.txt","a+");
	RuleNode* aux_rulestore = rule_store;

	float sum = 0.0;
	float counter_sum = 0.0;
	if (aux_rulestore->rule->input0() == 'A') {
		//orden alfabetico
//		cout << "'A'" << endl;
		while ((aux_rulestore->rule->input0() == 'A') && (aux_rulestore->rule->input2() <= 'G')){
			sum += aux_rulestore->rule->Weight();
			counter_sum += 1.0;
			aux_rulestore = aux_rulestore->next;
			while ((aux_rulestore->rule->input1() == 'A') && (aux_rulestore->rule->input2() == 'A')) {
				//solventa error que se produce al grabar reglas en fichero con repeticion de regla AAA
				aux_rulestore = aux_rulestore->next;
			}
		}
		fprintf(A_rules_S,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while (aux_rulestore->rule->input0() == 'A') {
			sum += aux_rulestore->rule->Weight();
			counter_sum += 1.0;
			aux_rulestore = aux_rulestore->next;
		}
//		cout << "ANorte-> sum: " << sum <<" , counter: " << counter_sum << endl;
		fprintf(A_rules_N,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while ((aux_rulestore->rule->input0() == 'B') && (aux_rulestore->rule->input2() <= 'G')){
			sum += aux_rulestore->rule->Weight();
			counter_sum += 1.0;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(B_rules_S,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		cout << "BNorte->" << aux_rulestore->rule->input0() << aux_rulestore->rule->input1() <<aux_rulestore->rule->input2() << endl;
		while (aux_rulestore->rule->input0() == 'B') {
			sum += aux_rulestore->rule->Weight();
			counter_sum += 1.0;
			aux_rulestore = aux_rulestore->next;
		}
		cout << "BNorte-> sum: " << sum <<" , counter: " << counter_sum << endl;
		fprintf(B_rules_N,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while ((aux_rulestore->rule->input0() == 'C') && (aux_rulestore->rule->input2() <= 'G')){
			sum += aux_rulestore->rule->Weight();
			counter_sum += 1.0;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(C_rules_S,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while (aux_rulestore->rule->input0() == 'C') {
			sum += aux_rulestore->rule->Weight();
			counter_sum += 1.0;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(C_rules_N,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while ((aux_rulestore->rule->input0() == 'D') && (aux_rulestore->rule->input2() <= 'G')){
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(D_rules_S,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while (aux_rulestore->rule->input0() == 'D') {
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(D_rules_N,"%f\n",(sum/counter_sum));
	}else {
		while (aux_rulestore->rule->input0() != 'D') {
			aux_rulestore = aux_rulestore->next;
		}
		while ((aux_rulestore->rule->input0() == 'D') && (aux_rulestore->rule->input2() >= 'H')){
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		cout << "DNorte-> sum: " << sum <<" , counter: " << counter_sum << endl;
		fprintf(D_rules_N,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while (aux_rulestore->rule->input0() == 'D') {
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		cout << "DSur-> sum: " << sum <<" , counter: " << counter_sum << endl;
		fprintf(D_rules_S,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while ((aux_rulestore->rule->input0() == 'C') && (aux_rulestore->rule->input2() >= 'H')) {
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		cout << "CNorte-> sum: " << sum <<" , counter: " << counter_sum << endl;
		fprintf(C_rules_N,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while (aux_rulestore->rule->input0() == 'C') {
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(C_rules_S,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while ((aux_rulestore->rule->input0() == 'B') && (aux_rulestore->rule->input2() >= 'H')){
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(B_rules_N,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while (aux_rulestore->rule->input0() == 'B') {
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(B_rules_S,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while ((aux_rulestore->rule->input0() == 'A') && (aux_rulestore->rule->input2() >= 'H')){
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
		}
		fprintf(A_rules_N,"%f\n",(sum/counter_sum));
		sum = 0.0;
		counter_sum = 0.0;
		while ((aux_rulestore != 0) && (aux_rulestore->rule->input0() == 'A')) {
			sum += aux_rulestore->rule->Weight();
			counter_sum ++;
			aux_rulestore = aux_rulestore->next;
			while ((aux_rulestore != 0) && (aux_rulestore->rule->input1() == 'A') && (aux_rulestore->rule->input2() == 'A')) {
				//solventa error que se produce al grabar reglas en fichero con repeticion de regla AAA
				aux_rulestore = aux_rulestore->next;
			}
		}
		fprintf(A_rules_S,"%f\n",(sum/counter_sum));
	}

	fclose(A_rules_N);
	fclose(B_rules_N);
	fclose(C_rules_N);
	fclose(D_rules_N);
	fclose(A_rules_S);
	fclose(B_rules_S);
	fclose(C_rules_S);
	fclose(D_rules_S);

}

/*sistema de aprendizaje tutorizado*/


/**\brief--------------------------------------------------------------------------
**	Function:	Action
** 
**	Purpose:	take from the defuzzification model the output
** 
**	Arguments:	none
**
**	Returns:	nothing
**-------------------------------------------------------------------------*/	


float MOFSModel::Action(float* _output) {
	fuzzy_output_vars->var->calc_values(defuzz_method->defuzz(rule_data_list,num_inputs));

	// Ahora se conoce el valor float del movimiento del robot, a continuaci�n
	// se chequear� si este valor es correcto en funci�n de su valor fuzzy
	// con todas las reglas anteriormente seleccionadas y se realizar� una 
	// actualizaci�n de los pesos en funci�n de acierto de sus resultados
	// en caso de no encontrar ninguna regla acertada se actualizar� la salida 
	// de una o varias reglas
		//return fuzzy_output_vars->var.xValue();
	float output = fuzzy_output_vars->var->xValue();
	float user_output = checkOutput(_output);
	
// 	error_out = fopen("errorPrueba.txt","w");
// 	printf("fichero_abierto");
// 	fprintf(error_out,"%f\n",(output - user_output));
// 	printf("escribiendo fichero");
// 	fclose(error_out);
	//SaveWeightStats();
	if (user_output != output) {	
		return user_output;
	}
	return output;
}

/**\brief--------------------------------------------------------------------------
**	Function:	checkOutput
** 
**	Purpose:	Ask to the user if it not correct
** 
**	Arguments:	none
**
**	Returns:	nothing
**-------------------------------------------------------------------------*/
float MOFSModel::checkOutput(float* _output) {
	// se comprueba si el valor obtenido con las reglas es razonable
	// si no lo es se cambiar� por el centro del conjunto seleccionado
	// por el usuario
	char user_set_option;		// variable donde guardaremos la entrada por teclado de la opcion del usuario
	float user_value_option;
	float correct_output = fuzzy_output_vars->var->xValue();
	//cout << fuzzy_output_vars->var->xValue();
	if (mode == 0) {
		cout << "  ->si no es correcto introduzca el conjunto adecuado (en otro caso introduzca 'Z'): ";
		cin >> user_set_option;
		if ((user_set_option >= 'A') && (user_set_option < fuzzy_output_vars->var->setIndex())) {		// No es enter
			// el usuario ha introducido una opcion mejor
			correct_output = fuzzy_output_vars->var->center(user_set_option);
			fuzzy_output_vars->var->calc_values(correct_output);
			// se modifica el valor de la salida
			fuzzy_output_vars->var->calc_values(correct_output);
			// modificada la salida ahora se llama al proceso de actualizaci�n de reglas	
		}
	} else if (mode == 1) {
		user_value_option = *_output;
		if ((user_value_option >= fuzzy_output_vars->var->minX()) && (user_value_option <= fuzzy_output_vars->var->maxX())) {
				fuzzy_output_vars->var->calc_values(user_value_option);
				correct_output = user_value_option;
		}else {
			correct_output = *_output;
		}
	} else if (mode == 2) {
	//	cout << correct_output << endl;
	}
//	error_out << *_output - correct_output << endl;
	if (mode != 2) {
		updateRules();
	}

	return correct_output;
	// Manda la salida al simulador para que realice la accion adecuada
}


/**\brief--------------------------------------------------------------------------
**	Function:	updateRules
** 
**	Purpose:	update the weight and the output if it would be necesary
** 
**	Arguments:	char user_option	-->	the correct output
**
**	Returns:	nothing
**-------------------------------------------------------------------------*/

void MOFSModel::updateRules() {
	// chequea todas las reglas seleccionadas y actualiza los pesos y si el peso es 0.0 entonces
	// cambia la salida

	// se supone que encontrar� las reglas seleccionadas en orden

	// hacer inversa de la lista de cromosomas

	char* sets_array;
	float* y_array;
	int Acounter = 0;
	int Bcounter = 0;
	int Ccounter = 0;
	int Dcounter = 0;
	
	char user_option = fuzzy_output_vars->var->centerSet();
	RuleData* aux_rule_data_list = rule_data_list;
	RuleNode* aux_rulestore = rule_store;
//	int overlap_index = fuzzy_output_vars->var->overlapIndex();

//	cout << "overlap index = " << overlap_index << endl;

	sets_array = new char[2];
	y_array = new float[2];
//	cout << " sizeof(sets_array) : " << sizeof(sets_array) << endl;
	fuzzy_output_vars->var->SetsArray(sets_array);
	fuzzy_output_vars->var->Outputs(y_array);
//	cout << " sizeof(sets_array) : " << sizeof(sets_array) << endl;
	for (int i = 0;i<2;i++) {
		//cout << sets_array[i] << " ----> " << y_array[i] << endl;
	}
//	cin.get();

	while ((aux_rule_data_list != 0) &&(aux_rulestore != 0)) {
		if (aux_rulestore->rule->updateWeight(aux_rule_data_list->inputs_sets,sets_array,y_array, num_inputs) == 1) {		
			// la regla ha sido modificada
			//cout << aux_rulestore->rule->Weight() << endl;
			if (aux_rulestore->rule->Weight() <= 0.0) {
				aux_rulestore->rule->updateOutput(user_option);
			}
			aux_rule_data_list = aux_rule_data_list->next;
		}
//		cout << "regla no actualizada" << endl;
		aux_rulestore = aux_rulestore->next;
	}
	
	delete(sets_array);
	delete(y_array);
}


void MOFSModel::showVars() {
	VarsNode *aux_vars =  fuzzy_input_vars;
	int counter = 1;
	while (aux_vars != 0) {
			cout << "variable num " << counter  <<endl;
		//	char name[20];aux_vars->var->varName(name);
		//	cout << name << endl;
			//cout << aux_vars->var->VName() << endl;
			cout << aux_vars->var->maxX() << endl;
			cout << aux_vars->var->minX() << endl;
			cout << aux_vars->var->crispIndex() << endl;
//			cout << aux_vars->var->overlapIndex() << endl;

			aux_vars = aux_vars->next;
			counter++;
	}

	aux_vars =  fuzzy_output_vars;
//	char name[20];aux_vars->var->varName(name);
//	cout << name << endl;
	//cout << aux_vars->var->VName() << endl;
	cout << aux_vars->var->maxX() << endl;
	cout << aux_vars->var->minX() << endl;
	cout << aux_vars->var->crispIndex() << endl;
//	cout << aux_vars->var->overlapIndex() << endl;
	cout << endl << endl << "pulse una tecla para continuar" << endl;
//	cin.get();
}

void MOFSModel:: saveModel(char *file) {
	FILE* file_out;
	file_out = fopen(file,"w");
	FILE* save_statistics;
//	save_statistics = fopen("Statistics.txt","w");
	if (!file_out) {
		cout << " Error al salvar los datos del modelo fuzzy";
	}else{
		VarsNode *aux_vars =  fuzzy_input_vars;
		fprintf(file_out,"%d\n\n",num_vars);
		while (aux_vars != 0) {
		//	char *name;aux_vars->var->varName(name);
			fprintf(file_out,"I\n");
			char nombre[20];
			aux_vars->var->VName(nombre);
			//fprintf("name: %s\n",nombre);
			fprintf(file_out,"var\n");
                        fprintf(file_out,"%.2f\n",aux_vars->var->maxX());
                        fprintf(file_out,"%.2f\n",aux_vars->var->minX());
                        fprintf(file_out,"%.0f\n",aux_vars->var->crispIndex());
                        fprintf(file_out,"%.2f\n",aux_vars->var->maxCentral());
                        fprintf(file_out,"%.2f\n",aux_vars->var->minCentral());
                        fprintf(file_out,"%.0f\n",aux_vars->var->crispCentral());

			aux_vars = aux_vars->next;
		}

		aux_vars = fuzzy_output_vars;
		fprintf(file_out,"O\n");
		//aux_vars->var->VName(nombre);
		fprintf(file_out,"var_output\n");
                fprintf(file_out,"%.2f\n",aux_vars->var->maxX());
                fprintf(file_out,"%.2f\n",aux_vars->var->minX());
                fprintf(file_out,"%.0f\n",aux_vars->var->crispIndex());
                fprintf(file_out,"%.2f\n",aux_vars->var->maxCentral());
                fprintf(file_out,"%.2f\n",aux_vars->var->minCentral());
                fprintf(file_out,"%.0f\n",aux_vars->var->crispCentral());


		RuleNode *aux_rules = rule_store;
		char *ipts;
		ipts = new char[num_inputs];
		while (aux_rules != 0) {
                             //   printf(" inputs ");
				aux_rules->rule->inputs_array(ipts);
				for (int i =0;i<num_inputs;i++) {
                              //          printf("%c.",ipts[i]);
					fprintf(file_out,"%c",ipts[i]);
				//	fprintf(save_statistics,"%c",ipts[i]);
				}
				fprintf(file_out,"\n");
		//	char * inputs;aux_rules->rule->inputs_array(inputs);
		//	file_out << inputs << endl;
			fprintf(file_out,"%c\n",aux_rules->rule->outputValue());
		//	fprintf(save_statistics,"%c Peso: ", aux_rules->rule->outputValue());
                        fprintf(file_out,"%.2f\n",aux_rules->rule->Weight());
		//	fprintf(save_statistics,"%f --> %d\n",aux_rules->rule->Weight(), aux_rules->rule->BitAct());
			aux_rules = aux_rules->next;
		}
		fclose(file_out);
		delete(ipts);
	//	fclose(var1);
		cout << "datos guardados con exito. Pulse una tecla";
		cin.get();
	}
/*	pesos.close();
	error_out.close();
	vars_out.close();*/
	cin.get();
	//MOFSModel::~MOFSModel();
}
