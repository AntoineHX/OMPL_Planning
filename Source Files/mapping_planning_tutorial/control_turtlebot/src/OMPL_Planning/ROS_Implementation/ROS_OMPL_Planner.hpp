/**
 * \file      ROS_ OMPL_Planner.hpp
 * \author    Harlé Antoine (antoine.h@flylab.io)
 * \date      7/2017 - 8/2017
 * \brief     ROS implementation of OMPL_Planner.
 *
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <control_turtlebot/FindPathToGoal.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

//ROS-Octomap interface
using octomap_msgs::GetOctomap;

#include "../OMPL_Planner/OMPL_Planner.hpp"

/**
 * \class ROS_OMPL_Planner
 * \brief ROS implementation class for OMPL_Planner.
 */
class ROS_OMPL_Planner
{
	public:
		//!  Constructor.
		/*!
		 * Load planning parameters from configuration file.
		 * Create the connection with other ROS nodes.
		 * Create the OMPL_Planner instance.
		*/
		ROS_OMPL_Planner(int dimension);

		//! Request and load the actual map from the Octomap server.
		bool updateMap();

		//! Callback for getting current vehicle position
		void odomCallback(const nav_msgs::OdometryPtr &odom_msg);

		//! Find a path
		/*!
		 * Find a path from the current position to the goal request.
		 * Update the state checker, check the validity of the start/goal state.
		*/
		bool findPathToGoal(control_turtlebot::FindPathToGoal::Request& request, control_turtlebot::FindPathToGoal::Response& response);

		//! Procedure to visualize the resulting path.
		/*!
 		* Visualize resulting path.
		*/
		void visualizeRRT(const og::SimpleSetup simple_setup);

		//! Publish the resulting path.
		/*!
		 * Publish the path on a topic and in response of the request.
		 * Display the path.
		 * TOPIC : the first state of the path contains the number of states of the path.
		*/
		void PublishPath(control_turtlebot::FindPathToGoal::Response& response);
		
		//! Publish the resulting path.
		/*!
		 * Publish the path on a topic.
		 * Display the path.
		 * TOPIC : the first state of the path contains the number of states of the path.
		*/
		void PublishPath();

		//! Return the flag representing the validity of the current path.
		bool getObstacleFlag() const;

		//! Update the path.
		/*!
		 * Search a new path from start_state to goal state if there's a collision with the path and an obstacle from the new map.
		 * Update the state checker and the obstacle_flag.Check the validity of the start/goal state.
		 * Return True if a new path is found or if the path is collision-free. Return False, if the planner found an obstacle but failed to found a new path.
		 */
		bool updatePath();

		//!  Destructor.
		/*!
		 * Free dynamicly allowed data.
		*/
		~ROS_OMPL_Planner();

	private:
		OMPL_Planner planner; /*!< Planner for 2D and 3D planning. */

		//Octomap
		octomap::AbstractOcTree* abs_octree_;
		octomap::OcTree* octree_ = nullptr; /*!< Pointer to the object containing the map (Heavy object ?).*/

		// *** ROS ***
		ros::NodeHandle _node_hand;
		ros::Publisher visual_traj_pub_, path_pub_;
		ros::Subscriber odom_sub_; 
		ros::ServiceServer service_; 
		// Octomap server
		GetOctomap::Request req;
		GetOctomap::Response resp;
		std::string serv_name = "/octomap_full"; /*!< Address of the Octomap server. */

		bool _obstacle_flag; /*!< Flag representing the validity of the current path, whether there is obstacle on the path or not. */
		int planningDimension;
		double solving_time_,planning_depth_limit;
		std::string planner_name_;
		std::vector<double> current_position_, goal_state;
		std::vector<double> planning_bounds_x_, planning_bounds_y_, planning_bounds_z_; /*!< Limits of the planning workspace. */

		Checking_Box collision_mask; /*!< Virtual representation of the drone used for collision checking. */
};