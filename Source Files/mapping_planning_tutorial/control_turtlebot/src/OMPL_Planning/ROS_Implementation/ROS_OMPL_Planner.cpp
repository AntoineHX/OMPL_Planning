/**
 * \file     ROS_ OMPL_Planner.cpp
 * \author    Harlé Antoine (antoine.h@flylab.io)
 * \date      7/2017 - 8/2017
 * \brief     ROS implementation of OMPL_Planner.
 */

#include "ROS_OMPL_Planner.hpp"

ROS_OMPL_Planner::ROS_OMPL_Planner(int dimension) : planningDimension(dimension), planner(dimension), _obstacle_flag(true)
{
	//=======================================================================
	// Subscribers.
	//=======================================================================
	//Navigation data
	odom_sub_ = _node_hand.subscribe("/odom", 1, &ROS_OMPL_Planner::odomCallback, this);

	//=======================================================================
	// Publishers (to visualize the resulting path).
	//=======================================================================
	visual_traj_pub_ = _node_hand.advertise<visualization_msgs::Marker> ("/controller_turtlebot/solution_path", 1, true);
	path_pub_ = _node_hand.advertise<geometry_msgs::PoseArray> ("/Planning_Module/path", 1, true);

	//=======================================================================
	// Service
	//=======================================================================
	service_ = _node_hand.advertiseService("/controller_turtlebot/find_path_to_goal", &ROS_OMPL_Planner::findPathToGoal, this);

	planning_bounds_x_.resize(2);
	planning_bounds_y_.resize(2);
	planning_bounds_z_.resize(2);
	current_position_.resize(3);
	goal_state.resize(3);

	//=======================================================================
	// Get parameters from configuration file.
	//=======================================================================
	_node_hand.getParam("planning_bounds_x", planning_bounds_x_);
	_node_hand.getParam("planning_bounds_y", planning_bounds_y_);
	_node_hand.getParam("planning_bounds_z", planning_bounds_z_);

	_node_hand.getParam("solving_time", solving_time_);
	_node_hand.getParam("planner_name", planner_name_);

	_node_hand.getParam("collision_mask_x", collision_mask.x);
	_node_hand.getParam("collision_mask_y", collision_mask.y);

	if(dimension==2)
		_node_hand.getParam("collision_mask_z_2D", collision_mask.z);
	else 
		_node_hand.getParam("collision_mask_z_3D", collision_mask.z);

	_node_hand.getParam("planning_depth_limit", planning_depth_limit);

	//=======================================================================
	// Planner setup.
	//=======================================================================

	planner.setPlanningDepth(planning_depth_limit);

	if (dimension == 2)
		planner.setPlanningBounds(planning_bounds_x_, planning_bounds_y_);
	else 
		planner.setPlanningBounds(planning_bounds_x_, planning_bounds_y_, planning_bounds_z_);

	planner.setPlanner(planner_name_);

	updateMap();
	planner.setStateChecker(octree_, collision_mask.x, collision_mask.y, collision_mask.z);

	ROS_INFO("%s: %dD Planner Init done !", ros::this_node::getName().c_str(), dimension);
}

bool ROS_OMPL_Planner::updateMap()
{
	if(octree_ !=nullptr)
	{
	delete octree_;
	octree_ = nullptr;
	}

	ROS_INFO("%s: requesting the map to %s...", ros::this_node::getName().c_str(), _node_hand.resolveName(serv_name).c_str());

	while((_node_hand.ok() && !ros::service::call(serv_name, req, resp)) || resp.map.data.size()==0)
	{
		ROS_WARN("Request to %s failed; trying again...", _node_hand.resolveName(serv_name).c_str());
		usleep(1000000);
	}
	if (_node_hand.ok()){ // skip when CTRL-C
		abs_octree_ = octomap_msgs::msgToMap(resp.map);

		if (abs_octree_)
		{
			octree_ = dynamic_cast<octomap::OcTree*>(abs_octree_);
		}
	
		if (octree_){
			ROS_INFO("Octomap received (%zu nodes)", octree_->size());
			return true;
			} 
		else{
			ROS_ERROR("Error reading OcTree from stream");
			return false;
			}
	}
}

void ROS_OMPL_Planner::odomCallback(const nav_msgs::OdometryPtr &odom_msg)
{
	current_position_[0] = odom_msg->pose.pose.position.x;
	current_position_[1] = odom_msg->pose.pose.position.y;
	current_position_[2] = odom_msg->pose.pose.position.z;
}

bool ROS_OMPL_Planner::findPathToGoal(control_turtlebot::FindPathToGoal::Request& request, control_turtlebot::FindPathToGoal::Response& response)
{
	goal_state[0] = request.goal_state_x;
	goal_state[1] = request.goal_state_y;
	goal_state[2] = request.goal_state_z;

	//Prevent invalid states because of too low altitude (Octomap issue)
	if(goal_state[2]<planning_depth_limit) goal_state[2] = planning_depth_limit;
	if(current_position_[2]<planning_depth_limit) current_position_[2] = planning_depth_limit;

	updateMap();
	planner.updateStateChecker(octree_);

	// Start and Goal checking
	if(! planner.isValid(current_position_))
		ROS_WARN("%s: Start Position %lf/%lf/%lf is invalid !\n",ros::this_node::getName().c_str(), current_position_[0],current_position_[1],current_position_[2]);
	if(! planner.isValid(goal_state))
		ROS_WARN("%s: Goal Position %lf/%lf/%lf is invalid !\n",ros::this_node::getName().c_str(), goal_state[0],goal_state[1],goal_state[2]);

	planner.setStartGoal(current_position_, goal_state);
	
	ROS_INFO("%s: Searching path from %lf/%lf/%lf to %lf/%lf/%lf ...\n",ros::this_node::getName().c_str(), current_position_[0],current_position_[1],current_position_[2], goal_state[0],goal_state[1],goal_state[2]);

	if(planner.findPath(solving_time_))
	{
		_obstacle_flag = false;
		ROS_INFO("%s: path has been found with simple_setup", ros::this_node::getName().c_str());
		PublishPath(response);
	}
	else
		ROS_WARN("%s: path has not been found", ros::this_node::getName().c_str());

	return true;
}

//REALLY Ugly function
void ROS_OMPL_Planner::visualizeRRT(const og::SimpleSetup simple_setup)
{
	og::PathGeometric geopath = simple_setup.getSolutionPath();
	// %Tag(MARKER_INIT)%
	visualization_msgs::Marker q_init_goal, visual_rrt, result_path, visual_result_path;
	visual_result_path.header.frame_id = result_path.header.frame_id = q_init_goal.header.frame_id = visual_rrt.header.frame_id =  "/odom";
	visual_result_path.header.stamp = result_path.header.stamp = q_init_goal.header.stamp = visual_rrt.header.stamp = ros::Time::now();
	q_init_goal.ns = "online_planner_points";
	visual_rrt.ns = "online_planner_rrt";
	result_path.ns = "online_planner_result";
	visual_result_path.ns = "online_planner_result_path";
	visual_result_path.action = result_path.action = q_init_goal.action = visual_rrt.action = visualization_msgs::Marker::ADD;

	visual_result_path.pose.orientation.w = result_path.pose.orientation.w = q_init_goal.pose.orientation.w = visual_rrt.pose.orientation.w = 1.0;
	// %EndTag(MARKER_INIT)%

	// %Tag(ID)%
	q_init_goal.id = 0;
	visual_rrt.id = 1;
	result_path.id = 2;
	visual_result_path.id = 3;
	// %EndTag(ID)%

	// %Tag(TYPE)%
	result_path.type = q_init_goal.type = visualization_msgs::Marker::POINTS;
	visual_rrt.type = visual_result_path.type = visualization_msgs::Marker::LINE_LIST;
	// %EndTag(TYPE)%

	// %Tag(SCALE)%
	// POINTS markers use x and y scale for width/height respectively
	result_path.scale.x = q_init_goal.scale.x = 0.5;
	result_path.scale.y = q_init_goal.scale.y = 0.5;
	result_path.scale.z = q_init_goal.scale.z = 0.5;

	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	visual_rrt.scale.x = 0.01;
	visual_result_path.scale.x = 0.02;
	// %EndTag(SCALE)%

	// %Tag(COLOR)%
	// Points are green
	visual_result_path.color.g = 1.0;
	result_path.color.g = q_init_goal.color.g = 1.0;
	visual_result_path.color.a = result_path.color.a = q_init_goal.color.a = 1.0;

	// Line strip is blue
	visual_rrt.color.b = 1.0;
	visual_rrt.color.a = 1.0;


	ob::PlannerData planner_data(simple_setup.getSpaceInformation());
	simple_setup.getPlannerData(planner_data);
	std::vector< unsigned int > edgeList;
	int num_parents;
	//const ob::SE2StateSpace::StateType *state_se2;
	const ob::RealVectorStateSpace::StateType *state_r2;

	const ob::RealVectorStateSpace::StateType *state;
	geometry_msgs::Point p;

	if(planningDimension==3)
	{
		for (unsigned int i = 1 ; i < planner_data.numVertices() ; ++i)
		{
			if (planner_data.getVertex(i).getState() && planner_data.getIncomingEdges(i,edgeList) > 0)
			{
				state_r2 = planner_data.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
				p.x = state_r2->values[0];
				p.y = state_r2->values[1];
				p.z = state_r2->values[2];//3D change
				visual_rrt.points.push_back(p);

				state_r2 = planner_data.getVertex(edgeList[0]).getState()->as<ob::RealVectorStateSpace::StateType>();
				p.x = state_r2->values[0];
				p.y = state_r2->values[1];
				p.z = state_r2->values[2];//3D change
				visual_rrt.points.push_back(p);
			}
		}
		std::vector< ob::State * > states = geopath.getStates();

		for (uint32_t i = 0; i < geopath.getStateCount(); ++i)
		{
			// extract the component of the state and cast it to what we expect
			state = states[i]->as<ob::RealVectorStateSpace::StateType>();

			p.x = state->values[0];
			p.y = state->values[1];
			p.z = state->values[2];//3D change

			result_path.points.push_back(p);

			if(i>0)
			{
				visual_result_path.points.push_back(p);
				state = states[i-1]->as<ob::RealVectorStateSpace::StateType>();

				p.x = state->values[0];
				p.y = state->values[1];
				p.z = state->values[2];//3D change
				visual_result_path.points.push_back(p);
			}
		}
	}
	else
	{
		for (unsigned int i = 1 ; i < planner_data.numVertices() ; ++i)
		{
			if (planner_data.getVertex(i).getState() && planner_data.getIncomingEdges(i,edgeList) > 0)
			{
				state_r2 = planner_data.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
				p.x = state_r2->values[0];
				p.y = state_r2->values[1];
				p.z = planning_depth_limit; //2D change
				visual_rrt.points.push_back(p);

				state_r2 = planner_data.getVertex(edgeList[0]).getState()->as<ob::RealVectorStateSpace::StateType>();
				p.x = state_r2->values[0];
				p.y = state_r2->values[1];
				p.z = planning_depth_limit; //2D change
				visual_rrt.points.push_back(p);
			}
		}
		std::vector< ob::State * > states = geopath.getStates();

		for (uint32_t i = 0; i < geopath.getStateCount(); ++i)
		{
			// extract the component of the state and cast it to what we expect
			state = states[i]->as<ob::RealVectorStateSpace::StateType>();

			p.x = state->values[0];
			p.y = state->values[1];
			p.z = planning_depth_limit; //2D change

			result_path.points.push_back(p);

			if(i>0)
			{
				visual_result_path.points.push_back(p);
				state = states[i-1]->as<ob::RealVectorStateSpace::StateType>();

				p.x = state->values[0];
				p.y = state->values[1];
				p.z = planning_depth_limit; //2D change
				visual_result_path.points.push_back(p);
			}
		}
	}
	//visual_traj_pub_.publish(q_init_goal);
	visual_traj_pub_.publish(visual_rrt);
	visual_traj_pub_.publish(visual_result_path);
	//visual_traj_pub_.publish(result_path);
}

//Ugly function
void ROS_OMPL_Planner::PublishPath(control_turtlebot::FindPathToGoal::Response& response)
{
	og::PathGeometric path = planner.getPath();
	//Display path
	visualizeRRT(planner.getSimpleSetup());

	std::vector< ob::State * > states = path.getStates();
	const ob::RealVectorStateSpace::StateType *state;

	uint32_t state_nb = path.getStateCount();

	geometry_msgs::PoseArray path_msg;

	//A CHANGER : passage du nombre de state
	//Passer par stamp dans le header ?
	geometry_msgs::Pose state_nb_msg;
	state_nb_msg.position.x=state_nb;
	path_msg.poses.push_back(state_nb_msg);

	//ROS_INFO("State nb : %d",state_nb);
	if(planningDimension==3)
	{
		for (uint32_t i = 0; i < state_nb; ++i)
		{
			geometry_msgs::Pose pose;
			geometry_msgs::Point point;
			// extract the component of the state and cast it to what we expect
			state = states[i]->as<ob::RealVectorStateSpace::StateType>();

			point.x = state->values[0];
			point.y = state->values[1];
			point.z = state->values[2];
				
			pose.position = point;

			path_msg.poses.push_back(pose);
			//Send path as response of the planning request
			response.poses.push_back(pose);
		}
	}
	else
	{
		for (uint32_t i = 0; i < state_nb; ++i)
		{
			geometry_msgs::Pose pose;
			geometry_msgs::Point point;
			// extract the component of the state and cast it to what we expect
			state = states[i]->as<ob::RealVectorStateSpace::StateType>();

			point.x = state->values[0];
			point.y = state->values[1];
			point.z = planning_depth_limit;
				
			pose.position = point;

			path_msg.poses.push_back(pose);
			//Send path as response of the planning request
			response.poses.push_back(pose);
		}
	}
	//Publish the path on a topic
	path_pub_.publish(path_msg);
}

void ROS_OMPL_Planner::PublishPath()
{
	og::PathGeometric path = planner.getPath();
	//Display path
	visualizeRRT(planner.getSimpleSetup());

	std::vector< ob::State * > states = path.getStates();
	const ob::RealVectorStateSpace::StateType *state;

	uint32_t state_nb = path.getStateCount();

	geometry_msgs::PoseArray path_msg;

	//A CHANGER : passage du nombre de state
	//Passer par stamp dans le header ?
	geometry_msgs::Pose state_nb_msg;
	state_nb_msg.position.x=state_nb;
	path_msg.poses.push_back(state_nb_msg);

	//ROS_INFO("State nb : %d",state_nb);
	if(planningDimension==3)
	{
		for (uint32_t i = 0; i < state_nb; ++i)
		{
			geometry_msgs::Pose pose;
			geometry_msgs::Point point;
			// extract the component of the state and cast it to what we expect
			state = states[i]->as<ob::RealVectorStateSpace::StateType>();

			point.x = state->values[0];
			point.y = state->values[1];
			point.z = state->values[2];
				
			pose.position = point;

			path_msg.poses.push_back(pose);
		}
	}
	else
	{
		for (uint32_t i = 0; i < state_nb; ++i)
		{
			geometry_msgs::Pose pose;
			geometry_msgs::Point point;
			// extract the component of the state and cast it to what we expect
			state = states[i]->as<ob::RealVectorStateSpace::StateType>();

			point.x = state->values[0];
			point.y = state->values[1];
			point.z = planning_depth_limit;
				
			pose.position = point;

			path_msg.poses.push_back(pose);
		}
	}
	//Publish the path on a topic
	path_pub_.publish(path_msg);
}

bool ROS_OMPL_Planner::getObstacleFlag() const
{
	return _obstacle_flag;
}

bool ROS_OMPL_Planner::updatePath()
{
	ROS_INFO("%s: Updating path ...\n",ros::this_node::getName().c_str());

	if(current_position_[2]<planning_depth_limit) current_position_[2] = planning_depth_limit;

	updateMap();

	// Start and Goal checking
	if(! planner.isValid(current_position_))
		ROS_WARN("%s: Start Position %lf/%lf/%lf is invalid !\n",ros::this_node::getName().c_str(), current_position_[0],current_position_[1],current_position_[2]);
	if(! planner.isValid(goal_state))
		ROS_WARN("%s: Goal Position %lf/%lf/%lf is invalid !\n",ros::this_node::getName().c_str(), goal_state[0],goal_state[1],goal_state[2]);

	if(planner.updatePath(octree_, solving_time_, current_position_, goal_state))
	{
		_obstacle_flag=false;
		PublishPath();
		ROS_INFO("%s: Path update done !\n",ros::this_node::getName().c_str());
		return true;
	}
	else
	{
		_obstacle_flag = true;
		ROS_WARN("%s: Path update failed !\n",ros::this_node::getName().c_str());
		return false;
	}
}

ROS_OMPL_Planner::~ROS_OMPL_Planner()
{

}