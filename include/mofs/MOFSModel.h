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
    
    you can found more information about it in www.vision4uav.eu								**
*****************************************************************************/
#ifndef MOFSModel_H
#define MOFSModel_H

//namespace MOFS {

#include "MOFSMCDefuzz.h"
#include "MOFSVar.h"
#include "MOFSModelLkg.hh"


//#ifdef _LINUX_VERSION_
#include <iostream>
//#endif
//#ifndef _LINUX_VERSION_
//#include <fstream.h>
//#endif
using namespace std;


namespace mofs{







//public:
	struct RuleNode {
		MOFSRule*	rule;
		RuleNode*	next;
	};

	struct RuleValues {
		char		set;
		float		y;
		RuleValues*	next;
	};

	struct VarsNode {
		MOFSVar* 	var;
		MOFSVar::VarValues*	values;
		VarsNode*	next;
	};

	struct RuleData {
		char*		inputs_sets;
		float*		inputs_values;
		float		rule_output;
		char		rule_set_output;
		float		weight;
		RuleData*	next;
	};
	struct Set {
		char set;
		Set* next;
	};
	

		
		/************************/
		/**  class variables   **/
		/************************/

	class MOFSModel {
		private:

			float*		inputsVars;				// array de las entradas
			int		num_vars;				// numero de variables de entrada
			RuleNode*	rule_store;				// Almacena todas las reglas
			RuleData*	rule_data_list;			// Almacena todos las reglas validas, 
												// valor fuzzy
			VarsNode*	fuzzy_input_vars;		// variables fuzzy de entrada
			VarsNode*	fuzzy_output_vars;		// variables fuzzy de salida
			float		output;
			char		set_output;
			int			mode;					// modo de correccion en el aprendizaje tutorizado
			MOFSMCDefuzz*		defuzz_method;
			int num_inputs;
			FILE *pesos, *error_out, *vars_out, *var1, *ypos_file, *xpos_file;
			FILE *A_rules_N, *B_rules_N, *C_rules_N, *D_rules_N;
			FILE *A_rules_S, *B_rules_S, *C_rules_S, *D_rules_S;
			FILE *A_output, *B_output, *C_output, *D_output;

		/***********************/
		/**  member functions  */
		/***********************/
		public:
			MOFSModel();
			~MOFSModel();

			void open1(int _mode);				// _mode hace referencia al modo de correcci�n en el aprendizaje tutorizado de la salida
            void open2(char *fileVars, char *fileRules, int _mode, int genRules, int option4CreateVars);	// expresada por el sistema: mode = 0 -> se selecciona un conjunto como salida
                                            // mode = 1 -> se introduce un valor o se lee desde fichero
                                            // mode = 2 -> sin aprendizaje
                                            //genRules = 0 -> no genera reglas, genRules = 1 -> genera nuevas reglas
                                            // option4CreateVars = 0 -> Automatic creation of the variables
                                            // option$CreateVars = 1 -> Read all the fuzzy set from file

			int loadVars();					// lee las vars ling��sticas del sistema
			int readVars(FILE* file_in, int option4CreateVars);	// lee las variables desde un fichero
			int genRulebase();			// genera la base de reglas
			int readRulebase(FILE* file_in);	// lee una base de reglas desde un fichero
			float inputIrq(float inputs[],float* _output);				// siempre activo, genera accion de salida
			void getValidRules();			// obtiene el array de reglas v�lidas para
										// las entradas dadas.
			float Action(float* _output);
			void readData(float &max_x, float &min_x, int &crisp);
			void saveModel(char *file);
			void showRules();
			void showVars();
			int learning(char *file);
			void guardaPos(float x,float y);
			void initRulestore();
			
		//	int exit();

		private:
			void insertVar(VarsNode* new_var, char option);
			void saveRule(char ipts[], char rule_output, float weight) ;
			void insertRuleVar(RuleValues* &r_values,RuleValues* rv);
			float checkOutput(float* _output);
			void updateRules();
			void SaveWeightStats();
			void genRule(char inputs[], VarsNode* aux, int cont_vars);
			
			
			
	};

}

#endif
