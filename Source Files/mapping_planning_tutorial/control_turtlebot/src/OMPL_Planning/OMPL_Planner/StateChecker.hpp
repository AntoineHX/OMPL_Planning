/**
 * \file      State_Checker.hpp
 * \author    Harlé Antoine
 * \date      7/2017 - 8/2017
 * \brief     Check if a given configuration is collision-free.
 *  The workspace is represented by an octomap.
 */

#include <string>

using namespace std;

//OCTOMAP//
#include <octomap/octomap.h>

using namespace octomap;

//OMPL//
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;

/**
 * \struct Checking_Box
 * \brief Collision Mask : virtual representation of the drone as a box with x,y,z length
 */
struct Checking_Box
{
	double x;
	double y;
	double z;
};

/**
 * \class StateChecker
 * \brief Class for state checking for 2D and 3D planning.
 * Extension of an abstract class used to implement the state validity checker over an octomap (using FCL ?).
 */
class StateChecker : public ob::StateValidityChecker
{
	public:
	
		//! Constructor.
		/*!
		 * Call the StateValidityChecker constructor from OMPL library and set checking dimension.
		*/
		StateChecker(const ob::SpaceInformationPtr &si);

		//! Load the octomap where the states are checked
		void loadMap(const std::string mapFile);
		void loadMap(octomap::OcTree* octree);

		//! Set the area around every state considered for state checking.
		void setCollisionMask(double x, double y, double z = 0.2);

		//! Set the depth limit (constant for 2D checking)
		/*! Set the lowest altitude for checking (constant depth for 2D checking).
		 * Used to prevent unwanted invalid state (Issue with the ground in Octomap)
		*/
		void setDepthLimit(double depth_limit);

		//! State validator.
		/*!
		 * Function that verifies if the given state is valid (i.e. is free of collision) using FCL.
		 */
		virtual bool isValid(const ob::State *state) const;

		//!  Destructor.
		/*!
		 * Free dynamicly allowed data.
		*/
		~StateChecker();

	private :

		int planningDimension; /*!< 2D or 3D */
		//Octomap
		octomap::OcTree* _map = nullptr; /*!< Pointer to the object containing the map (Heavy object ?)*/
		double _map_res, _depth_limit;

		Checking_Box _check_box; /*!< Collision mask : area around every state considered for state checking. */
};