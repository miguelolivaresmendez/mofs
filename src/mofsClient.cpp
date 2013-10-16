/**
 * mofs.cpp
 *
 *  Created on: 27 May 2013
 *      Author: Miguel A. Olivares-Mendez
 *      SnT - University of Luxembourg
 *      Automation Research Group
 *
 *      mofs was previously a C++ library and now a ROS package developed by Miguel Angel Olivares Mendez
 *      at the Computer Vision Group - Universidad Politecnica de Madrid and at Automation Research Group
 *      - SnT - University of Luxembourg.
 *      This software allow to create and use Fuzzy Logic controllers to command whatever you want
 *
 *      Used under the conditions of ROS
 *
 *      any doubts don't hesitate to contact me: miguel.olivaresmendez@uni.lu
 */

/**
 * \brief example of a client to communicate with the mofs-ROS package
 */

#include "ros/ros.h"
#include "mofs/FuzzyControl_IO.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mofs_controller_client");
  if (argc != 1)
  {
    ROS_INFO("usage: mofs_controller_client inputs = otuput");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mofs::FuzzyControl_IO>("flc");
  mofs::FuzzyControl_IO srv;
  srv.request.inputs[0] = 1.1;
  srv.request.inputs[1] = 3.1;
  srv.request.inputs[2] = 4.1;
  ros::Rate loop_rate(10);
  while(ros::ok) {
	  if (client.call(srv))
	  {
		ROS_INFO("Output controller: %f", srv.response.output);
	  }
	  else
	  {
		ROS_ERROR("Failed to call service mofs_controller");
		return 1;
	  }
  }


  return 0;
}
