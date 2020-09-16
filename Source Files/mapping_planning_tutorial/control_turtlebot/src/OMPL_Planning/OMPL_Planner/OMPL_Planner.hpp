/**
 * \file      OMPL_Planner.hpp
 * \author    Harlé Antoine (antoine.h@flylab.io)
 * \date      7/2017 - 8/2017
 * \brief     Path planning (without differential constraints) using OMPL with Octomap to represent workspace and to validate if configurations are collision-free.
 *
 */

//#include <string>

//using namespace std;

//OMPL//
//#include <ompl/config.h>
//#include <ompl/base/SpaceInformation.h>
//#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

//OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

#include "StateChecker.hpp"

/**
 * \class OMPL_Planner
 * \brief Class for planner for 2D and 3D planning.
 */
class OMPL_Planner
{
	public :

		//!  Constructor.
		/*!
		 * Create space and simple_setup instance for OMPL planning.
		 * Set planning dimension.
		*/
		OMPL_Planner(int dim);

		//! Set the bounds of the planner workspace.
		/*!
 		* Workspace defined by a box with planning_bounds limits.
 		* Optional argument : Planning_bounds_z (used only for 3D planning).
		*/
		void setPlanningBounds(const std::vector<double> planning_bounds_x, const std::vector<double> planning_bounds_y, const std::vector<double> planning_bounds_z = std::vector<double>());
		
		//! Set the planning depth.
		/*! Set the lowest altitude for states.
		 * Represent the planning depth (constant) for 2D planning.
		 * Used to prevent unwanted invalid state (Issue with the ground in Octomap).
		*/
		void setPlanningDepth(double depth_limit = 1);
		
		//! Set the planning algorithm used to solve the problem.
		/*!
		 * Planner avalaible : RRTstar, CForest.
		 * If no argument provided, RRTstar will be used (reliable planner for most of the situations).
		 * Implementation of other planners (except control-based planners) is quite simple.
		*/
		void setPlanner(const std::string plannerName = "RRTstar");

		//! Set the state checker.
		/*!
		 * Check if a given position is collision-free.
		*/
		void setStateChecker(const ob::StateValidityCheckerPtr state_checker);
		void setStateChecker(const std::string mapFile, double collision_mask_x, double collision_mask_y, double collision_mask_z = 0.2);
		void setStateChecker(octomap::OcTree* octree, double collision_mask_x, double collision_mask_y, double collision_mask_z);

		//! Update the state checker.
		/*!
		 * The new map will be used for state checking.
		 * If the state checker wasn't set before, there's no update and no warning message will be displayed.
		 */
		void updateStateChecker(octomap::OcTree* octree);

		//! Set the planning objectives.
		/*!
		 * If the state has an altitude lower than the planning depth, it's altitude will be set to the planning depth.
		 */
		void setStartGoal(const std::vector<double> start_state, const std::vector<double> goal_state);

		//! Solve the planning problem.
		/*!
		 * Find a path from the start state to the goal state in the map given to the state checker.
		 * The planner will run during solving_time and optimize the solution during this time.
		 */
		bool findPath(double solving_time);

		//! Check if position is collision-free.
		/*!
		 * Checking is done on the last map given to the state checker.
		 */
		bool isValid(std::vector<double> position) const;

		//! Check if the path is valid.
		bool checkPath() const;

		//! Update the path.
		/*!
		 * Search a new path from start_state to goal state if there's a collision with the path and an obstacle from the new map.
		 * Update the state checker.
		 * Return True if a new path is found or if the path is collision-free. Return False, if the planner found an obstacle but failed to found a new path.
		 */
		bool updatePath(octomap::OcTree* octree, double solving_time, const std::vector<double> start_state, const std::vector<double> goal_state);

		//! Return the path found by the planner.
		og::PathGeometric getPath() const;

		//! Return the OMPL object containing most of the information of the planning problem.
		og::SimpleSetup getSimpleSetup() const;

		//!  Destructor.
		/*!
		 * Free dynamicly allowed data.
		*/
		~OMPL_Planner();

	private :

		int _planningDimension; /*!< 2D or 3D */

		ob::SpaceInformationPtr _si; /*!< Contain all the information on the workspace*/
		ob::StateSpacePtr _space; /*!< Space in which we're working (2D /3D) */
		og::SimpleSetup _simple_setup; /*!< OMPL object for planning in simple configurations*/

		StateChecker* _checker = nullptr; /*!< Object for checking if a position is collision-free */

		//std::vector<double> _start_state, _goal_state;
		double _depth_limit; /*!< lowest altitude for checking (constant depth for 2D checking). Used to prevent unwanted invalid state (Issue with the ground in Octomap). */
};