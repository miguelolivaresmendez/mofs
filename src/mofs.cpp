/**
 * mofs.cpp
 *
 *  Created on: 27 May 2013
 *      Author: Miguel A. Olivares-Mendez
 *      SnT - University of Luxembourg
 *      Automation Research Group
 *
 *      mofs was previously a C++ library and now a ROS package developed by Miguel Angel Olivares-Mendez
 *      at the Computer Vision Group - Universidad Politecnica de Madrid and at Automation Research Group
 *      - SnT - University of Luxembourg.
 *      This software allow to create and use Fuzzy Logic controllers to command whatever you want
 *
 *      Used under the conditions of ROS
 *
 *      any doubts don't hesitate to contact me: miguel.olivaresmendez@uni.lu
 */

/**
 * \brief main file of the mofs-ROS package
 */

#include "ros/ros.h"
#include <MOFSModel.h>
#include <string.h>
#include "mofs/FuzzyControl_IO.h"

mofs::MOFSModel flc;
char *fileController, *fileRules, *serviceName;
//change the path to the one where your controllers are stored
const char *location = "/home/miguelolivaresmendez/code/fuerte_workspace/sandbox/mofs/include/FLC/";
char controller[100], rules[100];

bool fuzzy_control_IO_callback(mofs::FuzzyControl_IO::Request &request, mofs::FuzzyControl_IO::Response &response){

	int size = sizeof(request.inputs)/sizeof(request.inputs[0]);
	float inputs[size];
	int size2 = sizeof(inputs)/sizeof(*inputs);
	for (int i=0;i<size;i++){
		inputs[i] = (float)request.inputs[i];
	}
	//ROS_INFO("Size of the array: %d, %d",size,size2);

	float output;
	output = flc.inputIrq(inputs,0);
	response.output = output;
	ROS_INFO("sending output: %f",response.output);
	return true;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "mofs_controller_server");
	strcpy(controller,location);
	strcpy(rules,location);
	if (argc >= 3){
		fileController = argv[1];
		fileRules = argv[2];
		serviceName = argv[3];
		strcat(controller,fileController);
		strcat(rules,fileRules);
	}else{
		ROS_INFO("mofs need two parameters: controller definition and rules files");
	//	sleep(50);
		return 0;
	}
//strcat(controller,"PIDyawEPFL.txt");
//strcat(rules,"125rulesEPFL.txt");
//  flc.open2("../include/FLC/PIDyawEPFL.txt","../include/FLC/125rulesEPFL.txt" ,2,0,1);
//	ROS_INFO(controller);
	flc.open2(controller,rules ,2,0,1);

	ros::NodeHandle n;
//
	  ros::ServiceServer service = n.advertiseService(serviceName, fuzzy_control_IO_callback);
////	ros::ServiceServer service = n.advertiseService(serviceName, fuzzy_control_IO_callback);
	ROS_INFO("Ready to control whatever you want.");
	ros::spin();

  return 0;
}
