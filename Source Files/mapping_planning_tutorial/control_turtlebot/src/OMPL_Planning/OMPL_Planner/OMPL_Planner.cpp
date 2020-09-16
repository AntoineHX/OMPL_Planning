/**
 * \file      OMPL_Planner.cpp
 * \author    Harlé Antoine (antoine.h@flylab.io)
 * \date      7/2017 - 8/2017
 * \brief     Path planning (without differential constraints) using OMPL with Octomap to represent workspace and to validate if configurations are collision-free.
 *
 */

#include "OMPL_Planner.hpp"

OMPL_Planner::OMPL_Planner(int dim) : _planningDimension(dim), _space(new ob::RealVectorStateSpace(dim)), _simple_setup(_space)
{
	_si = _simple_setup.getSpaceInformation();
	/*
	_start_state.resize(3);
	_goal_state.resize(3);
	*/
}

OMPL_Planner::~OMPL_Planner()
{
	/*
	if(_checker !=nullptr)
	{
		delete _checker;
		_checker = nullptr;
	}
	*/
}

void OMPL_Planner::setPlanningBounds(const std::vector<double> planning_bounds_x, const std::vector<double> planning_bounds_y, const std::vector<double> planning_bounds_z)
{
	ob::RealVectorBounds bounds(_planningDimension);

	if(!planning_bounds_z.empty() && _planningDimension ==3)
	{
		bounds.setLow(2, planning_bounds_z[0]);
		bounds.setHigh(2, planning_bounds_z[1]);
	}

	bounds.setLow(0, planning_bounds_x[0]);
	bounds.setHigh(0, planning_bounds_x[1]);
	bounds.setLow(1, planning_bounds_y[0]);
	bounds.setHigh(1, planning_bounds_y[1]);

	_space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}

void OMPL_Planner::setPlanningDepth(double depth_limit)
{
	_depth_limit = depth_limit;
	if(_checker !=nullptr)
		_checker->setDepthLimit(depth_limit);
}

void OMPL_Planner::setPlanner(const std::string plannerName)
{
	ob::PlannerPtr planner;
	if(plannerName.compare("CForest")==0)
		planner = ob::PlannerPtr(new og::CForest(_si)); //Deleted by simple_setup destructor ?
	else //Default planner
		planner = ob::PlannerPtr(new og::RRTstar(_si));

	//planner->as<og::RRTstar>()->setRange(0.2);
	//=======================================================================
	// Set the setup planner
	//=======================================================================
	_simple_setup.setPlanner(planner);
}

void OMPL_Planner::setStateChecker(const ob::StateValidityCheckerPtr state_checker)
{
	if(state_checker != nullptr)
		_simple_setup.setStateValidityChecker(state_checker);
	else
		cout<<"ERROR : invalid pointer on state_checker\n";
}

void OMPL_Planner::setStateChecker(const std::string mapFile, double collision_mask_x, double collision_mask_y, double collision_mask_z)
{
	if(_checker !=nullptr)
	{
		delete _checker;
		_checker = nullptr;
	}
	
	_checker = new StateChecker(_si);
	_checker->loadMap(mapFile);
	_checker->setCollisionMask(collision_mask_x,collision_mask_y,collision_mask_z);
	_checker->setDepthLimit(_depth_limit);
	_simple_setup.setStateValidityChecker(ob::StateValidityCheckerPtr(_checker));
}

void OMPL_Planner::setStateChecker(octomap::OcTree* octree, double collision_mask_x, double collision_mask_y, double collision_mask_z)
{
	if(_checker !=nullptr)
	{
		delete _checker;
		_checker = nullptr;
	}
	
	_checker = new StateChecker(_si);
	_checker->loadMap(octree);
	_checker->setCollisionMask(collision_mask_x,collision_mask_y,collision_mask_z);
	_checker->setDepthLimit(_depth_limit);
	_simple_setup.setStateValidityChecker(ob::StateValidityCheckerPtr(_checker));
}

void OMPL_Planner::updateStateChecker(octomap::OcTree* octree)
{
	if(_checker !=nullptr)
		_checker->loadMap(octree);
	//No error warned
}

void OMPL_Planner::setStartGoal(const std::vector<double> start_state, const std::vector<double> goal_state)
{
	ob::ScopedState<> start(_space);
	ob::ScopedState<> goal(_space);
	start[0] = double(start_state[0]);
	start[1] = double(start_state[1]);

	goal[0] = double(goal_state[0]);
	goal[1] = double(goal_state[1]);

	if(_planningDimension==3)
	{
		start[2] = double(start_state[2]);
		goal[2] = double(goal_state[2]);

		//Prevent invalid states because of too low altitude (Octomap issue)
		if (start[2] < _depth_limit) start[2] = _depth_limit;
		if (goal[2] < _depth_limit) goal[2] = _depth_limit;
	}

	_simple_setup.setStartAndGoalStates(start, goal);
}

bool OMPL_Planner::findPath(double solving_time)
{
	//Clear all previous planning data (Planner settings, start/goal states not affected)
	_simple_setup.clear();

	_simple_setup.setup();

	//_simple_setup.print(cout);

	ob::PlannerStatus solved = _simple_setup.solve( solving_time );

	if (solved && _simple_setup.haveExactSolutionPath())
	{
		return true;
	}
	else
		return false;
}

bool OMPL_Planner::isValid(std::vector<double> position) const
{
	if(_checker !=nullptr)
	{
		ob::ScopedState<> pos(_space);

		if(_planningDimension==2)
		{
			pos[0] = double(position[0]);
			pos[1] = double(position[1]);
			return _checker->isValid(pos->as<ob::SE2StateSpace::StateType>());
		}
		else
		{
			pos[0] = double(position[0]);
			pos[1] = double(position[1]);
			
			//Prevent invalid states because of too low altitude (Octomap issue)
			if(position[2]<_depth_limit)
				pos[2]=_depth_limit;
			else
				pos[2] = double(position[2]);
			return _checker->isValid(pos->as<ob::SE3StateSpace::StateType>());
		}
	}
	else
		return false; //No error warned
}

bool OMPL_Planner::checkPath() const
{
	og::PathGeometric path = getPath();

	return path.check();
}

bool OMPL_Planner::updatePath(octomap::OcTree* octree, double solving_time, const std::vector<double> start_state, const std::vector<double> goal_state)
{
	updateStateChecker(octree);

	if(! checkPath()) //Invalid path
	{
		setStartGoal(start_state,goal_state);
		return findPath(solving_time);
	}
	else
		return true;
}

og::PathGeometric OMPL_Planner::getPath() const
{
	return _simple_setup.getSolutionPath();
}

og::SimpleSetup OMPL_Planner::getSimpleSetup() const
{
	return _simple_setup;
}