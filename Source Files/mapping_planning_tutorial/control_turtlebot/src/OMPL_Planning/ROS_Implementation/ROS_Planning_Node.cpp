/**
 * \file     ROS_ Planning_Node.cpp
 * \author   Harlé Antoine (antoine.h@flylab.io)
 * \date     7/2017 - 8/2017
 * \brief    Planning ROS node.
 *
 */

#include "ROS_OMPL_Planner.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "offline_planner_with_services_R2");
	ros::NodeHandle node_handler_;
	ROS_INFO("%s: using OMPL version %s", ros::this_node::getName().c_str(), OMPL_VERSION);
	ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

	std::string planning_dimension;
	node_handler_.getParam("planning_dimension", planning_dimension);

	
	if(planning_dimension.compare("2D")==0) //2D planning
	{
		//Initialization planner
		ROS_OMPL_Planner planner(2);

		ros::Rate loop_rate(10);
		while (ros::ok())
		{
			if(! planner.getObstacleFlag()) //Constantly check while the path is valid
			{
				planner.updatePath();
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else //3D planning
	{
		//Initialization planner
		ROS_OMPL_Planner planner(3);

		ros::Rate loop_rate(10);
		while (ros::ok())
		{
			if(! planner.getObstacleFlag()) //Constantly check while the path is valid
			{
				planner.updatePath();
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	return 0;
}