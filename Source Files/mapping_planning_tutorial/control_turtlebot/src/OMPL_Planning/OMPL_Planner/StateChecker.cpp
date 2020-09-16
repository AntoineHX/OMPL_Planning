/**
 * \file      StateChecker.cpp
 * \author    Harlé Antoine
 * \date      7/2017 - 8/2017
 * \brief     Check if a given configuration is collision-free.
 *  The workspace is represented by an octomap.
 */

#include "StateChecker.hpp"

StateChecker::StateChecker(const ob::SpaceInformationPtr &si): ob::StateValidityChecker(si)
{
	planningDimension = si->getStateDimension();
}

StateChecker::~StateChecker()
{
	if(_map !=nullptr)
	{
		delete _map;
		_map = nullptr;
	}
	
}

void StateChecker::loadMap(const std::string mapFile) //Take a look to memory mangaement for map
{
	/*
	if(_map !=nullptr)
	{
		delete _map;
		_map = nullptr;
	}
	*/
	_map = (new octomap::OcTree(mapFile));
	_map_res = _map->getResolution();
}

void StateChecker::loadMap(octomap::OcTree* octree)
{
	/*
	if(_map !=nullptr)
	{
		delete _map;
		_map = nullptr;
	}
	*/
	_map = octree;
	_map_res = _map->getResolution();
}

void StateChecker::setCollisionMask(double x, double y, double z)
{
	_check_box.x=x;
	_check_box.y=y;
	_check_box.z=z;
}

void StateChecker::setDepthLimit(double depth_limit)
{
	_depth_limit = depth_limit;
}

bool StateChecker::isValid(const ob::State *state) const
{
	OcTreeNode* result;
	point3d query; //Octomap
	bool collision(false);
	double node_occupancy;

	// extract the component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

	//Constant depth for 2D planning (There might be a better way to implement this)
	
	if(planningDimension==2)
	{
		pos->values[2]=_depth_limit;
	}
	
	//Recherche dans le collision mask autour de pos
	for(double xi = pos->values[0]-(_check_box.x/2.0);xi <= pos->values[0]+(_check_box.x/2.0);xi=xi+_map_res)
		for(double yi = pos->values[1]-(_check_box.y/2.0);yi <= pos->values[1]+(_check_box.y/2.0);yi=yi+_map_res)
			for(double zi = pos->values[2]-(_check_box.z/2.0);zi <= pos->values[2]+(_check_box.z/2.0);zi=zi+_map_res){ //Trop sensible sur z
				query.x() = xi;
				query.y() = yi;
				query.z() = zi;
				result = _map->search (query); //Retourne le pointeur de la node à la position query, NULL si elle n'existe pas (terrain inconnu)

				if(result != NULL){ // Node connue

					node_occupancy = result->getOccupancy();
			
					if (node_occupancy > 0.4)
					{
						collision = true;
						break;
					}
				}
			}

	return !collision;
}