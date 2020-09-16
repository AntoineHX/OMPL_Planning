/**
 * \file     Hello_World.cpp
 * \author   Harlé Antoine (antoine.h@flylab.io)
 * \date     7/2017 - 8/2017
 * \brief    HelloWorld for OMPL_Planner class.
 */

#include <iostream>

#include "../OMPL_Planner/OMPL_Planner.hpp"

int main(int argc, char **argv)
{
	int planningDimension = 2, solvingTime = 5;
	std::string plannerName, mapFile;
	std::vector<double> start, goal, planning_bounds_x_, planning_bounds_y_, planning_bounds_z_;

	planning_bounds_x_.resize(2);
	planning_bounds_y_.resize(2);
	planning_bounds_z_.resize(2);

	cout<<"Path planning with OMPL version "<<OMPL_VERSION<<"\n";
	cout<<"Choose a dimension for planning 2D or 3D (default :"<<planningDimension<<") : ";
	cin >> planningDimension;

	start.resize(planningDimension);
	goal.resize(planningDimension);

	cout<<"Choose a planner (available planner : CForest, RRTstar) : ";
	cin >> plannerName;

	OMPL_Planner planner(planningDimension);

	planning_bounds_x_[0]=-15;
	planning_bounds_x_[1]=15;
	planning_bounds_y_[0]=-15;
	planning_bounds_y_[1]=15;
	planning_bounds_z_[0]=-15;
	planning_bounds_z_[1]=15;

	if (planningDimension == 3)
		planner.setPlanningBounds(planning_bounds_x_, planning_bounds_y_, planning_bounds_z_);
	else 
		planner.setPlanningBounds(planning_bounds_x_, planning_bounds_y_);

	planner.setPlanningDepth();

	planner.setPlanner(plannerName);

	cout<<"Enter direction to the Octomap file (.bt file usually) : ";
	cin>> mapFile;
	planner.setStateChecker(mapFile,0.5,0.5,0.5);

	cout<<"\n Planner initialization done !\n";

	start[2]=0;
	goal[2]=0;

	cout<<"Choose a start position (expecting "<< planningDimension <<" values) : ";
	if(planningDimension==2)
		cin>> start[0] >> start[1];
	else
		cin>> start[0] >> start[1] >> start[2];
	
	cout<<"Choose a goal position (expecting "<< planningDimension <<" values) : ";
	if(planningDimension==2)
		cin>> goal[0] >> goal[1];
	else
		cin>> goal[0] >> goal[1] >> goal[2];

	planner.setStartGoal(start,goal);

	cout<<"Choose solving time (default :"<<solvingTime<<") : ";
	cin>> solvingTime;

	//cout<<"Searching path from "<<start[0]<<" / "<<start[1]<<" / "<<start[2]<<" to "<<goal[0]<<" / "<<goal[1]<<" / "<<goal[2]<<"\n";

	if(planner.findPath(solvingTime))
	{
		og::PathGeometric path = planner.getPath();
		path.print(cout);
	}
	else
		cout<<"\n No path found\n";

	cout<<"Thanks for using this awsome Hello World !\n";

	return 0;
}


//// DOXYGEN DOC ////

/*! \mainpage Home page
 *
 * \section intro_sec Introduction
 *
 * This project regroup tools for path planning in 2D or in 3D using OMPL (http://ompl.kavrakilab.org/index.html). 
 * It also include a ROS implementation for dynamic plannification.
 *
 * Work with Ubuntu 14.04 (and ROS Indigo) or with Ubuntu 16.04 (and ROS Kinetic).
 *
 * \section install_sec Installation
 *
 * \subsection step1 Step 1 : Workspace intialization
 *
 * Catkin Workspace : 
 *		mkdir -p ~/catkin_ws/src
 *		cd ~/catkin_ws/src
 *		catkin_init_workspace
 *		
 *		cd ~/catkin_ws/
 *		catkin_make
 *
 *		echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
 *		source ~/.bashrc 
 *
 *  \subsection step2 Step 2 : Compilation
 *
 *  It's possible to use the Turtlebot or the Hector Quadrotor (ROS Indigo only).
 *
 *  \subsubsection ste2a Step 2a : Turtlebot
 *
 * Turtlebot dependencies :
 *
 *		sudo apt-get install ros-kinetic-turtlebot* ros-kinetic-octomap* ros-kinetic-octovis
 *		
 *	    or
 *
 *	    sudo apt-get install ros-indigo-turtlebot* ros-indigo-octomap* ros-indigo-octovis
 *
 *	Turtlebot install :
 *
 *		 copy "mapping_planning_tutorial" folder (in Source Files directory) to "~/catkin_ws/src".
 *		
 *		 cd ~/catkin_ws/
 *
 *		 catkin_make (mignt need to be performed multiple times if you have error messages with "FindPathToGoal". Warning : it's possible that you won't manage to compile if you don't have enough computation power)
 *
 * \subsubsection ste2b Step 2b : Hector Quadrotor (ROS Indigo only)    WARNING : There might need some modification for the package path to make it work
 *
 * 		copy "hector_quadrotor_tutorial" folder (in Source Files directory) to "~/catkin_ws/src".
 *
 * Differences with Turtlebot
 *
 *   		Launch file : start_turtlebot_mapping_planning_control.launch
 *
 *     	 	"/odom"  ===> "/world"
 *
 *    	  	<node pkg="control_turtlebot" type="controller_turtlebot.py" name="controller_turtlebot" respawn="true" output="screen" />  ===> <node pkg="control_turtlebot" type="controller_quadrotor.py" name="controller_quadrotor" respawn="true" output="screen" />
 *      
 *
 *  ROS_OMPL_Planner.cpp :
 *
 *      odom_sub_ = _node_hand.subscribe("/odom", 1, &ROS_OMPL_Planner::odomCallback, this); ==>  odom_sub_ = node_handler_.subscribe("/ground_truth/state", 1, &ROS_OMPL_Planner::odomCallback, this);
 *
 *      visual_result_path.header.frame_id = result_path.header.frame_id = q_init_goal.header.frame_id = visual_rrt.header.frame_id =  "/odom"; ===> visual_result_path.header.frame_id = result_path.header.frame_id = q_init_goal.header.frame_id = visual_rrt.header.frame_id =  "/world";
 *
 *
 * \section run Running
 *
 * You can run HelloWorld program to understand how works OMPL_Planner alone or you can run the ROS implementation (ROS_OMPL_Planner) which will include the ROS implementation, the visual simulation, the Octomap server, etc.
 *
 * \subsection runHello Run HelloWorld
 *
 *  	copy the octomap file (.bt) (in Maps directory) you want to use from the Map folder in the project to "~/catkin_ws/devel/lib/control_turtlebot" folder.
 *
 *  	cd ~/catkin_ws/devel/lib/control_turtlebot
 *
 *  	run HelloWorld (./HelloWorld)
 *
 *  Enjoy !
 * 
 * \subsection runROS Run ROS implementation
 *
 *  	1st terminal (Gazebo / Rviz / Controller) : 
 *
 * 		Turtlebot :  roslaunch control_turtlebot start_turtlebot_modules.launch
 *
 * 		Quadrotor :  roslaunch hector_quadrotor_demo indoor
 *       
 * 	 	2nd terminal (Octomap / Planning): roslaunch control_turtlebot start_turtlebot_mapping_planning_control.launch
 *      
 *  	3rd terminal (Planning request) : rosservice call /controller_turtlebot/find_path_to_goal "goal_state_x: 0.0
 *      	                                                                                       goal_state_y: 0.0
 *          	                                                                                   goal_state_z: 0.0"
 *              	                      rosservice call /controller_turtlebot/goto "goal_state_x: 0.0
 *                  	                                                              goal_state_y: 0.0
 *                      	                                                          goal_state_z: 0.0" 
 *
 * \subsubsection config Configurations files
 *
 * See launch folder for runtime configurations.
 *
 * In config folder : planner_parameters.yaml
 *
 * Contains all the necessary parameters for a basic use of the planner (planning dimension, collision mask, etc.).
 * 
 *
 * See also start_turtlebot_mapping_planning_control.launch file and src folder for a deeper understanding of the ROS implementation.
 *
 * \section todo Todo
 *
 *	BUG : Compilation sur certain linux : FindPathToGoal.h pb de génération avec cmake
 *
 * Frontier exploration
 *
 * Clean the dynmacilly allowed memory (Needed for dynamic check)
 *
 * Optimisation :
 * 
 *   prevent creation of too close state (Ex : start = goal is a problem)
 *
 *	 path check : don't check path already done
 *
 *	 Improving sending speed of the octomap
 *
 *
 */