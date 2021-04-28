//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "RVO2DAgent.h"
#include "RVO2DAIModule.h"
#include "SteerLib.h"
#include "Definitions.h"
#include "RVO2D_Parameters.h"
// #include "util/Geometry.h"

/// @file RVO2DAgent.cpp
/// @brief Implements the RVO2DAgent class.

#undef min
#undef max

#define MAX_FORCE_MAGNITUDE 3.0f
// #define MAX_SPEED 1.33f
#define AGENT_MASS 1.0f
#define BAG_DISTANCE 1.0f // Beyond this distance an owner and bag will attempt to reunite as the primary goal
//set this define if using the merseyrail/PTI testcase
//#define TRAINHACKS
//#define SLOWREGION
//#define DRAW_VESTIBULE

using namespace Util;
using namespace RVO2DGlobals;
using namespace SteerLib;


#ifdef TRAINHACKS
// PTI goal locations hack
float depth = 0.f;

//#define SUBURBAN
#ifdef SUBURBAN
std::vector<Util::Point> PossibleGoals = { Util::Point(-13,0,depth),
								Util::Point(-7,0,depth),
								Util::Point(7,0,depth),
								Util::Point(13,0,depth),
								Util::Point(27, 0, depth),
								Util::Point(33, 0, depth) };
#endif
#ifdef INTERCITY
std::vector<Util::Point> PossibleGoals = { Util::Point(-22,0,depth),
								Util::Point(-2,0,depth),
								Util::Point(2,0,depth),
								Util::Point(22,0,depth),
								Util::Point(26, 0, depth),
								Util::Point(46, 0, depth) };
#endif

#endif //TRAINHACKS
#ifdef SLOWREGION
	Util::AxisAlignedBox slowRegion = Util::AxisAlignedBox(-5, 5, 0, 0, -5, 5);
#endif


// #define _DEBUG_ENTROPY 1

RVO2DAgent::RVO2DAgent()
{
	_RVO2DParams.rvo_max_neighbors  = rvo_max_neighbors ;
	_RVO2DParams.rvo_max_speed  = rvo_max_speed ;
	_RVO2DParams.rvo_neighbor_distance  = rvo_neighbor_distance ;
	_RVO2DParams.rvo_time_horizon  = rvo_time_horizon ;
	_RVO2DParams.rvo_time_horizon_obstacles  = rvo_time_horizon_obstacles ;
	_RVO2DParams.next_waypoint_distance = next_waypoint_distance;
	_enabled = false;
}

RVO2DAgent::~RVO2DAgent()
{
	// std::cout << this << " is being deleted" << std::endl;
	/*
	if (this->enabled())
	{
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		// getSimulationEngine()->getSpatialDatabase()->removeObject( this, bounds);
	}*/
	// std::cout << "Someone is removing an agent " << std::endl;
}

SteerLib::EngineInterface * RVO2DAgent::getSimulationEngine()
{
	return _gEngine;
}

void RVO2DAgent::addGoal(const SteerLib::AgentGoalInfo& newGoal)
{
	if (newGoal.goalType != SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET &&
		newGoal.goalType != GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
		newGoal.goalType != GOAL_TYPE_SEEK_DYNAMIC_TARGET) {
		throw Util::GenericException("Currently the RVO agent does not support goal types other than GOAL_TYPE_SEEK_STATIC_TARGET, GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL and GOAL_TYPE_SEEK_DYNAMIC_TARGET.");
	}
	_goalQueue.push(newGoal);
	if (_goalQueue.size() == 1) {
		_currentGoal = newGoal;
		if (_currentGoal.targetIsRandom) {

			SteerLib::AgentGoalInfo _goal;
			_goal.targetLocation = getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
			_goalQueue.push(_goal);
			_currentGoal.targetLocation = _goal.targetLocation;
		}
	}

}
void RVO2DAgent::setParameters(Behaviour behave)
{
	this->_RVO2DParams.setParameters(behave);
}

void RVO2DAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// std::cout << "this agent is being disabled " << this << std::endl;
	assert(_enabled==true);


	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;


}

/*
bool RVO2DAgent::collidesAtTimeWith(const Util::Point & p1, const Util::Vector & directiontToRightOfAgent, float otherAgentRadius, float timeStamp, float otherAgentFootX, float otherAgentFootZ)
{
	// std::cout << "not supported by this agent" << ", pos " << p1 << std::endl;
	if ( (computePenetration( p1, otherAgentRadius) > 0.0001) |
			(computePenetration( Util::Point(otherAgentFootX, 0.0, otherAgentFootZ), otherAgentRadius) > 0.0001) |
			(computePenetration( p1+directiontToRightOfAgent, otherAgentRadius) > 0.0001) |
			(computePenetration( p1-directiontToRightOfAgent, otherAgentRadius) > 0.0001)
			)
	{
		return true;
	}
	return false;
}*/

void RVO2DAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	counted_this_frame = false;
	close_frames = 0;
	//random amount of variation in initial conditions
	std::normal_distribution<> d{ 10, 5 };
	std::random_device rd;
	std::mt19937 gen{ rd() };

	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	// std::cout << "resetting agent " << this << std::endl;

	if ( initialConditions.colorSet == true )
	{
		this->_color = initialConditions.color;
	}
	else
	{
		this->_color = Util::gBlue;
	}

	_waypoints.clear();
	agentNeighbors_.clear();
	obstacleNeighbors_.clear();
	orcaPlanes_.clear();
	orcaLines_.clear();

	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);


	// initialize the agent based on the initial conditions
	/*
	position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
	radius_ = initialConditions.radius;
	velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
	velocity_ = velocity_ * initialConditions.speed;
*/
	// initialize the agent based on the initial conditions
	// std::cout << initialConditions << std::endl; 
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	//_radius = initialConditions.radius + d(gen);
	_radius = initialConditions.radius;
	_sdradius = initialConditions.sdradius;
	_isBag = initialConditions.isBag;
	_velocity = initialConditions.speed * _forward;

	neighborDist_ = _RVO2DParams.rvo_neighbor_distance;
	maxNeighbors_ = _RVO2DParams.rvo_max_neighbors;
	timeHorizon_ = _RVO2DParams.rvo_time_horizon;
	maxSpeed_ = _RVO2DParams.rvo_max_speed;
	timeHorizonObst_ = _RVO2DParams.rvo_time_horizon_obstacles;
	next_waypoint_distance_ = _RVO2DParams.next_waypoint_distance;

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		getSimulationEngine()->getSpatialDatabase()->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		// std::cout << "new position is " << _position << std::endl;
		// std::cout << "new bounds are " << newBounds << std::endl;
		// std::cout << "reset update " << this << std::endl;
		getSimulationEngine()->getSpatialDatabase()->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
		// engineInfo->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET||
				initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
			}
		}
		// For RVO Model, GOAL_TYPE_SEEK_DYNAMIC_TARGET refers to a bag following a person
		else if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_DYNAMIC_TARGET)
		{
			_goalQueue.push(initialConditions.goals[i]);
		}
		else {
			throw Util::GenericException("Unsupported goal type; RVO2DAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET, GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL and GOAL_TYPE_SEEK_DYNAMIC_TARGET.");
		}
	}

	// runLongTermPlanning2();

	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	 * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	 * And that _waypoints is not empty
	 */
	Util::Vector goalDirection;
	// Hacky stuff for acclmesh
	// NavMeshDataBase * nav = dynamic_cast<NavMeshDataBase * >(getSimulationEngine()->getSpatialDatabase());
	// Util::Point vert = nav->closestVert(this->position());
	// this->_position = vert;
	// std::cout << "Number of goals " << _goalQueue.size() << std::endl;
	// std::cout << "location of goals.front " << _goalQueue.front().targetLocation << std::endl;
	runLongTermPlanning(_goalQueue.front().targetLocation, dont_plan);
	 
	if ( !_waypoints.empty() )
	{
		// this->updateLocalTarget2();
		goalDirection = normalize( _waypoints.front() - position());
	}
	else
	{
		goalDirection = normalize( _goalQueue.front().targetLocation - position());
	}

	_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
			" and current velocity is: " << velocity_ << std::endl;
#endif


	// _gEngine->addAgent(this, rvoModule);
	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);

	_max_radius = _radius;
	_receprocity_factor = 1;//d(gen);
	//SteerLib::AgentGoalInfo insidetrainGoal;
	//insidetrainGoal.goalType = GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL;
	//insidetrainGoal.targetIsRandom = false;
	//insidetrainGoal.timeDuration = 1000;
	//insidetrainGoal.desiredSpeed = 1.3f;
	//insidetrainGoal.targetRegion = Util::AxisAlignedBox(-18.f, 38.f, -1.f, 1.f, 3.f, 5.f);
	//addGoal(insidetrainGoal);


	// read params from file
	std::string param_name;
	float param_value;
	std::ifstream paramsFile("params.txt");
	if (paramsFile.is_open()) {
		while (paramsFile >> param_name >> param_value)
		{
			if (param_name == "region_close") {
				_near_dist = param_value;
			}
			else if (param_name == "region_far") {
				_far_dist = param_value;
			}
			else if (param_name == "min_soc_dist") {
				_min_radius = param_value;
			}
		}
		paramsFile.close();
	}

#ifdef TRAINHACKS
	//set boarding status depending on position of agent. Those in the train must alight, those outside must board
	if( _position.z >= 0 ){
		loading_status = status::agent_alighting;
		_color = Util::gDarkOrange;
	}
	else {
		loading_status = status::agent_boarding;
	}
#endif
}

/*
void RVO2DAgent::computeNeighbors()
{
	agentNeighbors_.clear();

	if (_RVO2DParams.rvo_max_neighbors > 0) {
		// std::cout << "About to segfault" << std::endl;
		dynamic_cast<RVO2DAIModule *>(rvoModule)->kdTree_->computeAgentNeighbors(this, _RVO2DParams.rvo_neighbor_distance * _RVO2DParams.rvo_neighbor_distance);
		// std::cout << "Made it past segfault" << std::endl;
	}
}*/

bool compareDist(std::pair<float, const SteerLib::AgentInterface *> a1,
			std::pair<float, const SteerLib::AgentInterface *> a2 )
	{
		return a1.first < a2.first;
	}

void RVO2DAgent::computeNeighbors()
{
	obstacleNeighbors_.clear();
	float rangeSq = sqr(_RVO2DParams.rvo_time_horizon_obstacles * _RVO2DParams.rvo_max_speed + _radius);
	// dynamic_cast<RVO2DAIModule *>(rvoModule)->kdTree_->computeObstacleNeighbors(this, rangeSq);
	getSimulationEngine()->getSpatialDatabase()->computeObstacleNeighbors(this, rangeSq);

	// std::cout << "Number of obstacle neighbours " << obstacleNeighbors_.size() << std::endl;

	agentNeighbors_.clear();

	if (_RVO2DParams.rvo_max_neighbors > 0)
	{
	
		/*
		 * Old ORCA method
		 */
		rangeSq = sqr(_RVO2DParams.rvo_neighbor_distance);
		// dynamic_cast<RVO2DAIModule *>(rvoModule)->kdTree_->computeAgentNeighbors(this, rangeSq);
		// std::cout << "RVO spatial database: " << getSimulationEngine()->getSpatialDatabase() << std::endl;
		getSimulationEngine()->getSpatialDatabase()->computeAgentNeighbors(this, rangeSq);
		/*
		 * This was updated to use the SteerLib griddatabase instead
		 * It is a bad idea to keep two serperate structures to facilitate
		 * neighbor query operations.
		 */
		/*
		std::set<SpatialDatabaseItemPtr>  neighborList;
		rangeSq = _RVO2DParams.rvo_neighbor_distance;
		getSimulationEngine()->getSpatialDatabase()->getItemsInRange(neighborList, position().x-rangeSq, position().x+rangeSq, position().z-rangeSq, position().z+rangeSq, dynamic_cast<SpatialDatabaseItemPtr>(this));
		// WIll need to sort these.

		for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = neighborList.begin();  neighbor != neighborList.end();  neighbor++)
		{
			if ( (*neighbor)->isAgent())
			{
				SteerLib::AgentInterface * tmpA = dynamic_cast<SteerLib::AgentInterface *>(*neighbor);
				// if ( ((dynamic_cast<SteerLib::AgentInterface *>(*neighbor))->position()-position).length()  )
				{
					agentNeighbors_.push_back(std::make_pair((tmpA->position()-position()).length(), tmpA));
					//Util::DrawLib::drawStar(this->position() + ((dynamic_cast<AgentInterface*>(*neighbor)->position() - this->position())/2), Util::Vector(1,0,0), 1.14f, gRed);
					//std::cerr << "COLLISION FOUND AT TIME " << gTempCurrentTime << "\n";
				}
			}
		}
		// Will need to sort these.
		std::sort (agentNeighbors_.begin(), agentNeighbors_.end(), compareDist); // 12 32 45 71(26 33 53 80)
		agentNeighbors_.resize(_RVO2DParams.rvo_max_neighbors);
		// dynamic_cast<RVO2DAIModule *>(rvoModule)->kdTree_->computeAgentNeighbors(this, rangeSq);
		*/
	}
}

/*
bool RVO2DAgent::compareDist(std::pair<float, const SteerLib::AgentInterface *> a1,
		std::pair<float, const SteerLib::AgentInterface *> a2 )
{
	return ( a1.first < a2.first );
}


bool RVO2DAgent::compareDist(SteerLib::AgentInterface * a1, SteerLib::AgentInterface * a2 )
{
	return ( (position() - a1->position()).length() < (position() - a2->position()).length() );
}
*/

/* Search for the best new velocity. */
void RVO2DAgent::computeNewVelocity(float dt)
{
	orcaLines_.clear();

	const float invTimeHorizonObst = 1.0f / _RVO2DParams.rvo_time_horizon_obstacles;
	
	//auto original_radius = _radius;
	//_radius = MIN_RADIUS;
	/* Create obstacle ORCA lines. */
	for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {

		const ObstacleInterface *obstacle1 = obstacleNeighbors_[i].second;
		const ObstacleInterface *obstacle2 = obstacle1->nextObstacle_;

		const Util::Vector relativePosition1 = obstacle1->point_ - position();
		const Util::Vector relativePosition2 = obstacle2->point_ - position();

		//printf("obs1: (%f, %f)\t", obstacle1->point_.x, obstacle1->point_.z);
		//printf("obs2: (%f, %f)\n", obstacle2->point_.x, obstacle2->point_.z);

		/*
		 * Check if velocity obstacle of obstacle is already taken care of by
		 * previously constructed obstacle ORCA lines.
		 */
		bool alreadyCovered = false;

		for (size_t j = 0; j < orcaLines_.size(); ++j) {
			if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * _radius >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * _radius >=  -RVO_EPSILON) {
				alreadyCovered = true;
				break;
			}
		}

		if (alreadyCovered) {
			continue;
		}

		/* Not yet covered. Check for collisions. */

		const float distSq1 = absSq(relativePosition1);
		const float distSq2 = absSq(relativePosition2);

		const float radiusSq = sqr(_radius);

		const Util::Vector obstacleVector = obstacle2->point_ - obstacle1->point_;
		const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
		const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);

		Line line;

		if (s < 0.0f && distSq1 <= radiusSq) {
			/* Collision with left vertex. Ignore if non-convex. */
			if (obstacle1->isConvex_) {
				line.point = Util::Vector(0.0f, 0.0f, 0.0f);
				line.direction = normalize(Util::Vector(-relativePosition1.z, 0.0f, relativePosition1.x));
				orcaLines_.push_back(line);
			}

			continue;
		}
		else if (s > 1.0f && distSq2 <= radiusSq) {
			/* Collision with right vertex. Ignore if non-convex
			 * or if it will be taken care of by neighoring obstace */
			if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
				line.point = Util::Vector(0.0f, 0.0f, 0.0f);
				line.direction = normalize(Util::Vector(-relativePosition2.z, 0.0f, relativePosition2.x));
				orcaLines_.push_back(line);
			}

			continue;
		}
		else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
			/* Collision with obstacle segment. */
			line.point = Util::Vector(0.0f, 0.0f, 0.0f);
			line.direction = -obstacle1->unitDir_;
			orcaLines_.push_back(line);
			continue;
		}

		/*
		 * No collision.
		 * Compute legs. When obliquely viewed, both legs can come from a single
		 * vertex. Legs extend cut-off line when nonconvex vertex.
		 */

		Util::Vector leftLegDirection, rightLegDirection;

		if (s < 0.0f && distSqLine <= radiusSq) {
			/*
			 * Obstacle viewed obliquely so that left vertex
			 * defines velocity obstacle.
			 */
			if (!obstacle1->isConvex_) {
				/* Ignore obstacle. */
				continue;
			}

			obstacle2 = obstacle1;

			const float leg1 = std::sqrt(distSq1 - radiusSq);
			leftLegDirection = Util::Vector(relativePosition1.x * leg1 - relativePosition1.z * radius(), 0.0f, relativePosition1.x * radius() + relativePosition1.z * leg1) / distSq1;
			rightLegDirection = Util::Vector(relativePosition1.x * leg1 + relativePosition1.z * radius(), 0.0f, -relativePosition1.x * radius() + relativePosition1.z * leg1) / distSq1;
		}
		else if (s > 1.0f && distSqLine <= radiusSq) {
			/*
			 * Obstacle viewed obliquely so that
			 * right vertex defines velocity obstacle.
			 */
			if (!obstacle2->isConvex_) {
				/* Ignore obstacle. */
				continue;
			}

			obstacle1 = obstacle2;

			const float leg2 = std::sqrt(distSq2 - radiusSq);
			leftLegDirection = Util::Vector(relativePosition2.x * leg2 - relativePosition2.z * radius(), 0.0f, relativePosition2.x * radius() + relativePosition2.z * leg2) / distSq2;
			rightLegDirection = Util::Vector(relativePosition2.x * leg2 + relativePosition2.z * radius(), 0.0f, -relativePosition2.x * radius() + relativePosition2.z * leg2) / distSq2;
		}
		else {
			/* Usual situation. */
			if (obstacle1->isConvex_) {
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = Util::Vector(relativePosition1.x * leg1 - relativePosition1.z * radius(), 0.0f, relativePosition1.x * radius() + relativePosition1.z * leg1) / distSq1;
			}
			else {
				/* Left vertex non-convex; left leg extends cut-off line. */
				leftLegDirection = -obstacle1->unitDir_;
			}

			if (obstacle2->isConvex_) {
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				rightLegDirection = Util::Vector(relativePosition2.x * leg2 + relativePosition2.z * radius(), 0.0f, -relativePosition2.x * radius() + relativePosition2.z * leg2) / distSq2;
			}
			else {
				/* Right vertex non-convex; right leg extends cut-off line. */
				rightLegDirection = obstacle1->unitDir_;
			}
		}

		/*
		 * Legs can never point into neighboring edge when convex vertex,
		 * take cutoff-line of neighboring edge instead. If velocity projected on
		 * "foreign" leg, no constraint is added.
		 */

		const ObstacleInterface *const leftNeighbor = obstacle1->prevObstacle_;

		bool isLeftLegForeign = false;
		bool isRightLegForeign = false;

		if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
			/* Left leg points into obstacle. */
			leftLegDirection = -leftNeighbor->unitDir_;
			isLeftLegForeign = true;
		}

		if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
			/* Right leg points into obstacle. */
			rightLegDirection = obstacle2->unitDir_;
			isRightLegForeign = true;
		}

		/* Compute cut-off centers. */
		const Util::Vector leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position());
		const Util::Vector rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position());
		const Util::Vector cutoffVec = rightCutoff - leftCutoff;

		/* Project current velocity on velocity obstacle. */

		/* Check if current velocity is projected on cutoff circles. */
		const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity() - leftCutoff) * cutoffVec) / absSq(cutoffVec));
		const float tLeft = ((velocity() - leftCutoff) * leftLegDirection);
		const float tRight = ((velocity() - rightCutoff) * rightLegDirection);

		if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
			/* Project on left cut-off circle. */
			const Util::Vector unitW = normalize(velocity() - leftCutoff);

			line.direction = Util::Vector(unitW.z, 0.0f, -unitW.x);
			line.point = leftCutoff + radius() * invTimeHorizonObst * unitW;
			orcaLines_.push_back(line);
			continue;
		}
		else if (t > 1.0f && tRight < 0.0f) {
			/* Project on right cut-off circle. */
			const Util::Vector unitW = normalize(velocity() - rightCutoff);

			line.direction = Util::Vector(unitW.z, 0.0f, -unitW.x);
			line.point = rightCutoff + radius() * invTimeHorizonObst * unitW;
			orcaLines_.push_back(line);
			continue;
		}

		/*
		 * Project on left leg, right leg, or cut-off line, whichever is closest
		 * to velocity.
		 */
		const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(velocity() - (leftCutoff + t * cutoffVec)));
		const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity() - (leftCutoff + tLeft * leftLegDirection)));
		const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity() - (rightCutoff + tRight * rightLegDirection)));

		if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
			/* Project on cut-off line. */
			line.direction = -obstacle1->unitDir_;
			line.point = leftCutoff + radius() * invTimeHorizonObst * Util::Vector(-line.direction.z, 0.0f, line.direction.x);
			orcaLines_.push_back(line);
			continue;
		}
		else if (distSqLeft <= distSqRight) {
			/* Project on left leg. */
			if (isLeftLegForeign) {
				continue;
			}

			line.direction = leftLegDirection;
			line.point = leftCutoff + radius() * invTimeHorizonObst * Util::Vector(-line.direction.z, 0.0f, line.direction.x);
			orcaLines_.push_back(line);
			continue;
		}
		else {
			/* Project on right leg. */
			if (isRightLegForeign) {
				continue;
			}

			line.direction = -rightLegDirection;
			line.point = rightCutoff + radius() * invTimeHorizonObst * Util::Vector(-line.direction.z, 0.0f, line.direction.x);
			orcaLines_.push_back(line);
			continue;
		}
	}

	const size_t numObstLines = orcaLines_.size();

	const float invTimeHorizon = 1.0f / _RVO2DParams.rvo_time_horizon;

	//set radius of agent-agent interactions to include social distancing
	//_radius = original_radius;

	/* Create agent ORCA lines. */
	for (size_t i = 0; i < agentNeighbors_.size(); ++i)
	{
		const SteerLib::AgentInterface * other = agentNeighbors_[i].second;

		Util::Vector relativePosition = (other->position()) - position(); // This is fine
		Util::Vector relativeVelocity = velocity() - other->velocity();
		const float distSq = absSq(relativePosition);
		float combinedRadius = radius() + sdradius() + other->radius() + other->sdradius();
		float combinedRadiusSq = sqr(combinedRadius);

		//default amount to avoid collision
		float reciprocal_fov = 0.5f;


		//Alternative behaviour if other agent is the owner's bag - ignore bag as a constraint
		if (other->isBag() && std::stoi(other->currentGoal().targetName) == id()) {
			if (bag_id == -1 && !owned_bag) {
				//if bag is unset as bag_id, set it here
				bag_id = other->id();
				owned_bag = other;
			}

			continue;
		}
		//reverse of previous - ensure bag takes full responsibility to avoid owner
		else if (isBag() && std::stoi(currentGoal().targetName) == other->id()) {
			reciprocal_fov = 1.f;
		}
		//behaviour is agent's bag, and other agent is the owner - ignore social distancing between bag and owner
		else if (isBag() && std::stoi(currentGoal().targetName) == other->id()) {
			combinedRadius = radius() + other->radius(); //other radius is the physical person radius
			combinedRadiusSq = sqr(combinedRadius);
		}
		//if bag - ignore other agents
		else if (isBag() && !other->isBag()) {
			continue;
		}
		
		
		//if other agent is bag, and this is a person, take full responsibility for avoidance
		if (!this->isBag() && other->isBag()) {
			reciprocal_fov = 1.f;
		}


		//count SD invalidations
		if (abs(relativePosition) < SD && !counted_this_frame) {
			counted_this_frame = true;
			close_frames++;
		}


		Line line;
		Util::Vector u;

		if (distSq > combinedRadiusSq) {
			/* No collision. */
			const Util::Vector w = relativeVelocity - invTimeHorizon * relativePosition;
			/* Vector from cutoff center to relative velocity. */
			const float wLengthSq = absSq(w);

			const float dotProduct1 = w * relativePosition;

			if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
				/* Project on cut-off circle. */
				const float wLength = std::sqrt(wLengthSq);
				const Util::Vector unitW = w / wLength;

				line.direction = Util::Vector(unitW.z, 0.0f, -unitW.x);
				u = (combinedRadius * invTimeHorizon - wLength) * unitW;
			}
			else {
				/* Project on legs. */
				const float leg = std::sqrt(distSq - combinedRadiusSq);

				if (det(relativePosition, w) > 0.0f) {
					/* Project on left leg. */
					line.direction = Util::Vector(relativePosition.x * leg - relativePosition.z * combinedRadius, 0.0f, relativePosition.x * combinedRadius + relativePosition.z * leg) / distSq;
				}
				else {
					/* Project on right leg. */
					line.direction = -Util::Vector(relativePosition.x * leg + relativePosition.z * combinedRadius, 0.0f, -relativePosition.x * combinedRadius + relativePosition.z * leg) / distSq;
				}

				const float dotProduct2 = relativeVelocity * line.direction;

				u = dotProduct2 * line.direction - relativeVelocity;
			}
		}
		else {
			/* Collision. Project on cut-off circle of time timeStep. */
			const float invTimeStep = 1.0f / dt;

			/* Vector from cutoff center to relative velocity. */
			const Util::Vector w = relativeVelocity - invTimeStep * relativePosition;

			const float wLength = abs(w);
			const Util::Vector unitW = w / wLength;

			line.direction = Util::Vector(unitW.z, 0.0f, -unitW.x);
			u = (combinedRadius * invTimeStep - wLength) * unitW;
		}
		const RVO2DAgent* otherRVO = dynamic_cast<const RVO2DAgent*>(other);
		//float combined_reciprocity = _receprocity_factor / (_receprocity_factor + otherRVO->_receprocity_factor);


		

		
		
		
		// simulate boarding behvaiours- occurs when goalqueue size is set to 2 (one to get on train, one to get within train)
#ifdef TRAINHACKS
		if (_goalQueue.size() == 2 && other->_goalQueue.size() == 2) {
			const Util::Vector forwardVec = _goalQueue.front().targetLocation - position();
			const Util::Vector otherForwardVev = other->_goalQueue.front().targetLocation - other->position();
			//const Util::Vector upVec = Util::Vector(1, 0, 0);
			//const Util::Vector rightVec = Util::Vector(0, 0, 1);
			const float angle_to_other = abs(atan2(det(forwardVec, relativePosition), dot(forwardVec, relativePosition)));
			//angle_to_other = abs(atan2(det(upVec, rightVec), dot(upVec, rightVec)));
			const float angle_of_other = abs(atan2(det(otherForwardVev, -relativePosition), dot(otherForwardVev, -relativePosition)));
			const float visible_angle = M_PI / 5;
			//case 1: this cannot see the other. Other sees this
			if (abs(angle_to_other) > visible_angle && abs(angle_of_other) < visible_angle) {
				reciprocal_fov = 0.f;
				continue;
			}
			//case 2: this can see other. other cannot see this
			if (abs(angle_to_other) < visible_angle && abs(angle_of_other) > visible_angle) {
				reciprocal_fov = 1.f;
			}
			//case 3: This cannot see other. Other cannot see this
			if (abs(angle_to_other) > visible_angle && abs(angle_of_other) > visible_angle) {
				reciprocal_fov = 0.5f;
			}
			//case 4: this can see other. Other can see this
			if (abs(angle_to_other) < visible_angle && abs(angle_of_other) < visible_angle) {
				reciprocal_fov = 0.5f;
			}
		}
#endif //TRAINHACKS
		line.point = velocity() + reciprocal_fov * u;
		orcaLines_.push_back(line);
	}

	counted_this_frame = false;

	size_t lineFail = linearProgram2(orcaLines_, _RVO2DParams.rvo_max_speed, _prefVelocity, false, _newVelocity);

	if (lineFail < orcaLines_.size()) {
		linearProgram3(orcaLines_, numObstLines, lineFail, _RVO2DParams.rvo_max_speed, _newVelocity);
	}

}

void RVO2DAgent::insertAgentNeighbor(const SteerLib::AgentInterface *agent, float &rangeSq)
{
	if (this != agent) {
		const float distSq = absSq(position() - (agent->position()));

		if (distSq < rangeSq) {
			if (agentNeighbors_.size() < _RVO2DParams.rvo_max_neighbors) {
				agentNeighbors_.push_back(std::make_pair(distSq, agent));
			}

			size_t i = agentNeighbors_.size() - 1;

			while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
				agentNeighbors_[i] = agentNeighbors_[i - 1];
				--i;
			}

			agentNeighbors_[i] = std::make_pair(distSq, agent);

			if (agentNeighbors_.size() == _RVO2DParams.rvo_max_neighbors) {
				rangeSq = agentNeighbors_.back().first;
			}
		}
	}
}

void RVO2DAgent::insertObstacleNeighbor(const ObstacleInterface *obstacle, float rangeSq)
{
	const ObstacleInterface *const nextObstacle = obstacle->nextObstacle_;
	//printf("insterting obstacle: pos(%f, %f)\n", obstacle->point_.x, obstacle->point_.z);
	//printf("\t pos: (%f, %f)\n", obstacle->nextObstacle_->point_.x, obstacle->nextObstacle_->point_.z);

	const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, position());

	if (distSq < rangeSq) {
		obstacleNeighbors_.push_back(std::make_pair(distSq, obstacle));

		size_t i = obstacleNeighbors_.size() - 1;

		while (i != 0 && distSq < obstacleNeighbors_[i - 1].first) {
			obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
			--i;
		}

		obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
	}
}

//interpolated the value x to a vlaue betweeen y_max and y_min depending on the value of x compared to x_max and x_min 
float interpolation(float y_max, float y_min, float x_max, float x_min, float x)
{
	if (x_max == x_min) {
		return y_max;
	}
	if (x > x_max) {
		return y_max;
	}
	else if (x < x_min) {
		return y_min;
	}
	else {
		return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min;
	}
}

void RVO2DAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	// std::cout << "_RVO2DParams.rvo_max_speed " << _RVO2DParams._RVO2DParams.rvo_max_speed << std::endl;
	Util::AutomaticFunctionProfiler profileThisFunction( &RVO2DGlobals::gPhaseProfilers->aiProfiler );
	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	Util::Vector goalDirection;
	if ( ! _midTermPath.empty() ) // && (!this->hasLineOfSightTo(goalInfo.targetLocation)) )
	{
		if (reachedCurrentWaypoint())
		{
			this->updateMidTermPath();
		}

		this->updateLocalTarget();

		goalDirection = normalize(_currentLocalTarget - position());

	}
	// Update target location to be closest point of the goal box
	else if (goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
	{
		float nearest_x;
		if (position().x < goalInfo.targetRegion.xmin) nearest_x = goalInfo.targetRegion.xmin;
		else if (position().x > goalInfo.targetRegion.xmax) nearest_x = goalInfo.targetRegion.xmax;
		else nearest_x = position().x;

		float nearest_y;
		if (position().y < goalInfo.targetRegion.ymin) nearest_y = goalInfo.targetRegion.ymin;
		else if (position().y > goalInfo.targetRegion.ymax) nearest_y = goalInfo.targetRegion.ymax;
		else nearest_y = position().y;

		float nearest_z;
		if (position().z < goalInfo.targetRegion.zmin) nearest_z = goalInfo.targetRegion.zmin;
		else if (position().z > goalInfo.targetRegion.zmax) nearest_z = goalInfo.targetRegion.zmax;
		else nearest_z = position().z;

		goalInfo.targetLocation = Util::Point(nearest_x, nearest_y, nearest_z);
		goalDirection = normalize(goalInfo.targetLocation - position());
	}
	else if (goalInfo.goalType == GOAL_TYPE_SEEK_DYNAMIC_TARGET)
	{
		// Find the Agent to follow by id/name
		auto agents = getSimulationEngine()->getAgents();
		for (auto it = agents.begin(); it != agents.end(); ++it)
		{
			if ((*it)->id() == std::stoi(goalInfo.targetName))
			{
				//if owner finished, so should the bag
				if (!(*it)->enabled())
				{
					disable();
				}

				//auto goal_position = (*it)->position();
				goalDirection = normalize((*it)->position() - position());
				goalInfo.targetLocation = (*it)->position();
				//_prefVelocity = goalDirection;

				//print helpful vals
				//std::cout << "goal dir: " << goalDirection << 
				//	"\tvel: " << velocity() << 
				//	"\t_prefVelocity"  << _prefVelocity << std::endl;
			}
		}
	}
	else
	{
		goalDirection = normalize(goalInfo.targetLocation - position());
	}

#ifdef TRAINHACKS
	/*// If agent is in train, ensure goal is within train
	if (_position.z > 0.4f && _goalQueue.size() == 2 && loading_status == status::agent_boarding ) {
		_goalQueue.pop();
	}
	else if (status::agent_alighting && _goalQueue.size() == 2 && _position.z < -0.4f) {
		_goalQueue.pop();
	}*/

	int agent_to_print = -1;
	if (id() == agent_to_print) {
		printf("chosen door: %d %d \t goals: %d \t gz: %f\n", chosen_door, loading_status, _goalQueue.size(), goalInfo.targetLocation.z);
	}

	//Boarding/alighting affects whether to want to move
	switch(loading_status){
		case status::agent_boarding:
		{
			//see if anyone nearby is alighting and using the same door
			bool can_board = true;

			for (size_t i = 0; i < agentNeighbors_.size(); ++i)
			{
				const SteerLib::AgentInterface* other = agentNeighbors_[i].second;
				if (id() == agent_to_print) {
					printf("%d: %d %d \t", other->id(),  other->chosen_door, other->loading_status);
				}
				if (other->chosen_door == chosen_door && other->loading_status == status::agent_alighting && other->position().z > -1.5f) {
					can_board = false;
				}
			}

			if (can_board) {
				_prefVelocity = goalDirection * _RVO2DParams.rvo_max_speed;
			}
			else {
				_prefVelocity = Util::Vector(0, 0, 0);
			}
		}
			break;

		case status::agent_alighting:
			_prefVelocity = goalDirection * _RVO2DParams.rvo_max_speed;
			break;
		default:
			_prefVelocity = goalDirection * _RVO2DParams.rvo_max_speed;
	}
	if (id() == agent_to_print) {
		printf("\n ");
	}
#endif

#ifdef SLOWREGION
	//scale prefered velocity if agent is "on stairs"
	Util::Color slow_color = Util::gCyan; // _color for ticket barrier. _gCyan for stairs
	static Util::Color original_color = _color;
	float slow_speed_factor = 0.4; //0.9 for ticket barrier. 0.7 for stairs
	if (_position.x < slowRegion.xmax && _position.x > slowRegion.xmin && _position.z < slowRegion.zmax && _position.z > slowRegion.zmin) {
		_prefVelocity *= slow_speed_factor;
		_color = slow_color;
	}
	else {
		_color = original_color;
	}
#endif

#ifdef _DEBUG_ENTROPY
	std::cout << "Preferred velocity is: " << prefVelocity_ << std::endl;
#endif
	_prefVelocity = goalDirection;
	(this)->computeNeighbors();
	(this)->computeNewVelocity(dt);
	// (this)->computeNewVelocity(dt);
	_prefVelocity.y = 0.0f;

	// These are the internal RVO values calculated
	_velocity = _newVelocity;
#ifdef _DEBUG_ENTROPY
	std::cout << "new velocity is " << velocity_ << std::endl;
#endif
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	 * Or when the agent is removed for example its true location will not reflect its location in the grid database.
	 * Not only that but this error will appear random depending on how well the agent lines up with the grid database
	 * boundaries when removed.
	 */
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);


	// If bag owner and not set to reach bag, check bag is not too far away and create new goal
	if (!isBag() && owned_bag && !(_goalQueue.front().goalType == GOAL_TYPE_SEEK_DYNAMIC_TARGET))
	{
		if ((position() - owned_bag->position()).length() > BAG_DISTANCE)
		{
			SteerLib::AgentGoalInfo newGoal;
			auto currentGoal = _goalQueue.front();

			newGoal.goalType = GOAL_TYPE_SEEK_DYNAMIC_TARGET;
			newGoal.timeDuration = currentGoal.timeDuration;
			newGoal.desiredSpeed = currentGoal.desiredSpeed;
			newGoal.targetName = std::to_string(owned_bag->id());

			// Add the door goal at front of queue
			_goalQueue.pop();
			addGoal(newGoal);
			addGoal(currentGoal);
		}
	}


	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).length() < radius() * REACHED_WAYPOINT_MULTIPLIER)
	{
		// std::cout << "removing a waypoint" << std::endl;
		_waypoints.erase(_waypoints.begin());
	}
	/*
	 * Now do the conversion from RVO2DAgent into the SteerSuite coordinates
	 */
	_velocity.y = 0.0f;

	//reached current goal
	if(((goalInfo.targetLocation - position()).length() < radius()*REACHED_GOAL_MULTIPLIER ) ||
			(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
			Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
			goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
	{
		_goalQueue.pop();
		// std::cout << "Made it to a goal" << std::endl;
		if (_goalQueue.size() >= 1) {
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else {
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			/*
			AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
			getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

			//  2. set enabled = false
			_enabled = false;
				*/
			return;
		}
	}


	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if ( velocity().length() > 0.0 )
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}
	// _position = _position + (_velocity * dt);

	/*
	 * Stuff for mesh database
	 */

	_position.y = getSimulationEngine()->getSpatialDatabase()->getLocation(this).y;


	 // Update goal door 
#ifdef TRAINHACKS
	 // Shortest distance to nearest door goal
	float shortestDist = INFINITY;
	if (_goalQueue.front().goalType == GOAL_TYPE_SEEK_STATIC_TARGET) {
		int index = -1;
		for (auto it = PossibleGoals.begin(); it != PossibleGoals.end(); it++) {
			index++;
			if ((*it - position()).length() < shortestDist) {
				shortestDist = (*it - position()).length();
				chosen_door = index;

				//update goal if doors are the current goal
				_goalQueue.front().targetLocation = *it;
				goalInfo.targetLocation = *it;

				//alighting people have door target outside of the train
				if (loading_status == status::agent_alighting) {
					goalInfo.targetLocation.z = 0.f;
				}
			}
		}
	}
	// Adjust agent radius depending on distance to goal
	_radius = interpolation(_min_radius, _max_radius, _far_dist, _near_dist, _position.z);
	//_radius = interpolation(_max_radius, _min_radius, 3.f, 0.f, abs(_position.z));

	// In case an agent passes the first goal marker but gets stuck on the train outside. Add a door as a goal
	if (_position.z < -0.2f && _goalQueue.size() == 1 && shortestDist > MIN_RADIUS && loading_status == status::agent_boarding) {
		// Shortest distance to nearest door goal
		Util::Point newGoalLoc;
		float shortestDist = (goalInfo.targetLocation - position()).length();
		int index = -1;
		for (auto it = PossibleGoals.begin(); it != PossibleGoals.end(); it++) {
			index++;
			if ((*it - position()).length() < shortestDist) {
				shortestDist = (*it - position()).length();
				newGoalLoc = *it;
				chosen_door = index;
			}
		}


		SteerLib::AgentGoalInfo newGoal;
		newGoal.goalType = GOAL_TYPE_SEEK_STATIC_TARGET;
		newGoal.targetIsRandom = false;
		newGoal.timeDuration = 1000;
		newGoal.desiredSpeed = 1.3f;
		newGoal.targetLocation = newGoalLoc;

		// Add the door goal at front of queue
		auto currentGoal = _goalQueue.front();
		_goalQueue.pop();
		addGoal(newGoal);
		addGoal(currentGoal);
	}
	if (_position.z > 0.1f && _goalQueue.size() == 1 && loading_status == status::agent_alighting) {
		// Shortest distance to nearest door goal
		Util::Point newGoalLoc;
		float shortestDist = (goalInfo.targetLocation - position()).length();
		int index = -1;
		for (auto it = PossibleGoals.begin(); it != PossibleGoals.end(); it++) {
			index++;
			if ((*it - position()).length() < shortestDist) {
				shortestDist = (*it - position()).length();
				newGoalLoc = *it;
				newGoalLoc.z = 0.f;
				chosen_door = index;
			}
		}


		SteerLib::AgentGoalInfo newGoal;
		newGoal.goalType = GOAL_TYPE_SEEK_STATIC_TARGET;
		newGoal.targetIsRandom = false;
		newGoal.timeDuration = 1000;
		newGoal.desiredSpeed = 1.3f;
		newGoal.targetLocation = newGoalLoc;

		// Add the door goal at front of queue
		auto currentGoal = _goalQueue.front();
		_goalQueue.pop();
		addGoal(newGoal);
		addGoal(currentGoal);
	}
#endif
}


void RVO2DAgent::draw()
{
#ifdef ENABLE_GUI
#ifdef DRAW_VESTIBULE
	//draw vestibules
	for (auto& it = PossibleGoals.begin(); it != PossibleGoals.end(); it++) {
		Util::DrawLib::drawBox(it->x - 2.5, it->x + 2.5, 0, 0.1, -1.5, 5, Util::gDarkGreen);
	}
#endif

	AgentInterface::draw();
	_position.y = getSimulationEngine()->getSpatialDatabase()->getLocation(this).y;
	// if the agent is selected, do some annotations just for demonstration
	/*
	if (_gEngine->isAgentSelected(this))
	{
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (getSimulationEngine()->getSpatialDatabase()->trace(ray, t, objectFound, this, false))
		{
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}
		Util::DrawLib::drawFlag( this->currentGoal().targetLocation, Color(0.5f,0.8f,0), 2);
		
		if ( this->currentGoal().goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL )
		{
			Color color(0.4,0.9,0.4);
			DrawLib::glColor(color);
			DrawLib::drawQuad(Util::Point(this->currentGoal().targetRegion.xmin, 0.1, this->currentGoal().targetRegion.zmin),
					Util::Point(this->currentGoal().targetRegion.xmin, 0.1, this->currentGoal().targetRegion.zmax),
					Util::Point(this->currentGoal().targetRegion.xmax, 0.1, this->currentGoal().targetRegion.zmax),
					Util::Point(this->currentGoal().targetRegion.xmax, 0.1, this->currentGoal().targetRegion.zmin));
		}
	}
	else {
		Util::DrawLib::drawAgentDisc(_position, forward(), getSimulationEngine()->getSpatialDatabase()->getUpVector(this), _radius, this->_color);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}
	*/
/*
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
	 	_position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->computePenetration(this->position(), this->_radius) > 0.1f)
		{
			Util::DrawLib::drawStar(this->position() + ((dynamic_cast<AgentInterface*>(*neighbor)->position() - this->position())/2), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}*/

#ifdef DRAW_ANNOTATIONS

	for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
	{
		if ( _gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
		}
		else
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
		}
	}


	for (int i=0; i < (_waypoints.size()); i++)
	{
		DrawLib::drawFlag(_waypoints.at(i), gBlue, 1.0);
	}

/*
	if (_gEngine->isAgentSelected(this))
	{
		for (int l=0; l < orcaLines_.size(); l++)
		{
			// Util::Point p = position() + Util::Point(orcaLines_.at(l).point.x, orcaLines_.at(l).point.y,
				//	orcaLines_.at(l).point.z);
			Util::Point p = position();//  + orcaLines_.at(l).point;
			DrawLib::drawLine(Util::Point(0,0,0) + orcaLines_.at(l).point ,
					(Util::Point(0,0,0) + orcaLines_.at(l).point + (orcaLines_.at(l).direction*_RVO2DParams.rvo_time_horizon)),
					gOrange);
		}
	}
*/
#endif

#endif
}


bool linearProgram1(const std::vector<Line> &lines, size_t lineNo, float radius, const Util::Vector &optVelocity, bool directionOpt, Util::Vector &result)
{
	const float dotProduct = lines[lineNo].point * lines[lineNo].direction;
	const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(lines[lineNo].point);

	if (discriminant < 0.0f) {
		/* Max speed circle fully invalidates line lineNo. */
		return false;
	}

	const float sqrtDiscriminant = std::sqrt(discriminant);
	float tLeft = -dotProduct - sqrtDiscriminant;
	float tRight = -dotProduct + sqrtDiscriminant;

	for (size_t i = 0; i < lineNo; ++i) {
		const float denominator = det(lines[lineNo].direction, lines[i].direction);
		const float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);

		if (std::fabs(denominator) <= RVO_EPSILON) {
			/* Lines lineNo and i are (almost) parallel. */
			if (numerator < 0.0f) {
				return false;
			}
			else {
				continue;
			}
		}

		const float t = numerator / denominator;

		if (denominator >= 0.0f) {
			/* Line i bounds line lineNo on the right. */
			tRight = std::min(tRight, t);
		}
		else {
			/* Line i bounds line lineNo on the left. */
			tLeft = std::max(tLeft, t);
		}

		if (tLeft > tRight) {
			return false;
		}
	}

	if (directionOpt) {
		/* Optimize direction. */
		if (optVelocity * lines[lineNo].direction > 0.0f) {
			/* Take right extreme. */
			result = lines[lineNo].point + tRight * lines[lineNo].direction;
		}
		else {
			/* Take left extreme. */
			result = lines[lineNo].point + tLeft * lines[lineNo].direction;
		}
	}
	else {
		/* Optimize closest point. */
		const float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

		if (t < tLeft) {
			result = lines[lineNo].point + tLeft * lines[lineNo].direction;
		}
		else if (t > tRight) {
			result = lines[lineNo].point + tRight * lines[lineNo].direction;
		}
		else {
			result = lines[lineNo].point + t * lines[lineNo].direction;
		}
	}

	return true;
}

size_t linearProgram2(const std::vector<Line> &lines, float radius, const Util::Vector &optVelocity, bool directionOpt, Util::Vector &result)
{
	if (directionOpt) {
		/*
		 * Optimize direction. Note that the optimization velocity is of unit
		 * length in this case.
		 */
		result = optVelocity * radius;
	}
	else if (absSq(optVelocity) > sqr(radius)) {
		/* Optimize closest point and outside circle. */
		result = normalize(optVelocity) * radius;
	}
	else {
		/* Optimize closest point and inside circle. */
		result = optVelocity;
	}

	for (size_t i = 0; i < lines.size(); ++i) {
		if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
			/* Result does not satisfy constraint i. Compute new optimal result. */
			const Util::Vector tempResult = result;

			if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
				result = tempResult;
				return i;
			}
		}
	}

	return lines.size();
}

void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Util::Vector &result)
{
	float distance = 0.0f;

	for (size_t i = beginLine; i < lines.size(); ++i) {
		if (det(lines[i].direction, lines[i].point - result) > distance) {
			/* Result does not satisfy constraint of line i. */
			std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

			for (size_t j = numObstLines; j < i; ++j) {
				Line line;

				float determinant = det(lines[i].direction, lines[j].direction);

				if (std::fabs(determinant) <= RVO_EPSILON) {
					/* Line i and line j are parallel. */
					if (lines[i].direction * lines[j].direction > 0.0f) {
						/* Line i and line j point in the same direction. */
						continue;
					}
					else {
						/* Line i and line j point in opposite direction. */
						line.point = 0.5f * (lines[i].point + lines[j].point);
					}
				}
				else {
					line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
				}

				line.direction = normalize(lines[j].direction - lines[i].direction);
				projLines.push_back(line);
			}

			const Util::Vector tempResult = result;

			if (linearProgram2(projLines, radius, Util::Vector(-lines[i].direction.z, 0.0, lines[i].direction.x), true, result) < projLines.size()) {
				/* This should in principle not happen.  The result is by definition
				 * already in the feasible region of this linear program. If it fails,
				 * it is due to small floating point error, and the current result is
				 * kept.
				 */
				result = tempResult;
			}

			distance = det(lines[i].direction, lines[i].point - result);
		}
	}
}

