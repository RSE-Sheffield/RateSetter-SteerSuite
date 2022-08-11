//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "RVO2DAgent.h"
#include "RVO2DAIModule.h"
#include "SteerLib.h"
#include "Definitions.h"
#include "RVO2D_Parameters.h"
#include "RVO2DHelper.h"
// #include "util/Geometry.h"

/// @file RVO2DAgent.cpp
/// @brief Implements the RVO2DAgent class.

#undef min
#undef max

#define BAG_DISTANCE 2.0f // Beyond this distance between the edges of the agents, an owner and bag will attempt to reunite as the primary goal

using namespace Util;
using namespace RVO2DGlobals;
using namespace SteerLib;



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
{}

bool RVO2DAgent::tooFarFromBag()
{
	return ((position() - owned_bag->position()).length() >= (BAG_DISTANCE + radius() + owned_bag->radius()));
}

SteerLib::EngineInterface * RVO2DAgent::getSimulationEngine()
{
	return _gEngine;
}

void RVO2DAgent::addGoal(const SteerLib::AgentGoalInfo& newGoal)
{
	if (newGoal.goalType != SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET &&
		newGoal.goalType != GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
		newGoal.goalType != GOAL_TYPE_SEEK_DYNAMIC_TARGET &&
		newGoal.goalType != GOAL_TYPE_SEEK_STATIC_TARGET_SET) {
		throw Util::GenericException("Currently the RVO agent does not support goal types other than GOAL_TYPE_SEEK_STATIC_TARGET, GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL and GOAL_TYPE_SEEK_DYNAMIC_TARGET.");
	}
	if (!isBag() && owned_bag) {
		_color = Util::Color(0, 0, 0);
	}
	_goalQueue.push_back(newGoal);
	if (_goalQueue.size() == 1) {
		_currentGoal = newGoal;
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
	assert(_enabled==true);


	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;


}

void RVO2DAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	//random amount of variation in initial conditions
	std::normal_distribution<> d{ 10, 5 };
	std::random_device rd;
	std::mt19937 gen{ rd() };

	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.

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
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_sdradius = initialConditions.sdradius;
	_isBag = initialConditions.isBag;
	_velocity = initialConditions.speed * _forward;
	_groupId = initialConditions.groupId;

	neighborDist_ = _RVO2DParams.rvo_neighbor_distance;
	maxNeighbors_ = _RVO2DParams.rvo_max_neighbors;
	timeHorizon_ = _RVO2DParams.rvo_time_horizon;
	maxSpeed_ = _RVO2DParams.rvo_max_speed;
	timeHorizonObst_ = _RVO2DParams.rvo_time_horizon_obstacles;
	next_waypoint_distance_ = _RVO2DParams.next_waypoint_distance;

	behaviours = initialConditions.behaviours;

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		getSimulationEngine()->getSpatialDatabase()->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		getSimulationEngine()->getSpatialDatabase()->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop_front();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET||
				initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			_goalQueue.push_back(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push_back(_goal);
			}
		}
		// For RVO Model, GOAL_TYPE_SEEK_DYNAMIC_TARGET refers to a bag following a person
		else if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_DYNAMIC_TARGET)
		{
			_goalQueue.push_back(initialConditions.goals[i]);
		}
		else if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET_SET)
		{
			_goalQueue.push_back(initialConditions.goals[i]);
		}
		else {
			throw Util::GenericException("Unsupported goal type; RVO2DAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET,\
										 GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL, GOAL_TYPE_SEEK_DYNAMIC_TARGET and GOAL_TYPE_SEEK_STATIC_TARGET_SET.");
		}
	}

	/*
	 * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	 * And that _waypoints is not empty
	 */
	Util::Vector goalDirection;
	runLongTermPlanning(_goalQueue.front().targetLocation, dont_plan);
	 
	if ( !_waypoints.empty() )
	{
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


	// Disabled assertion due to initialConditions.direction being unused, leading to this firing.
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}

void RVO2DAgent::computeNeighbors()
{
	obstacleNeighbors_.clear();
	float rangeSq = sqr(_RVO2DParams.rvo_time_horizon_obstacles * _RVO2DParams.rvo_max_speed + _radius);
	getSimulationEngine()->getSpatialDatabase()->computeObstacleNeighbors(this, rangeSq);

	agentNeighbors_.clear();

	if (_RVO2DParams.rvo_max_neighbors > 0)
	{
			rangeSq = sqr(_RVO2DParams.rvo_neighbor_distance);
		getSimulationEngine()->getSpatialDatabase()->computeAgentNeighbors(this, rangeSq);
	}
}

/* Search for the best new velocity. */
void RVO2DAgent::computeNewVelocity(float dt)
{
	orcaLines_.clear();

	const float invTimeHorizonObst = 1.0f / _RVO2DParams.rvo_time_horizon_obstacles;
	
	/* Create obstacle ORCA lines. */
	for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {

		const ObstacleInterface *obstacle1 = obstacleNeighbors_[i].second;
		const ObstacleInterface *obstacle2 = obstacle1->nextObstacle_;

		const Util::Vector relativePosition1 = obstacle1->point_ - position();
		const Util::Vector relativePosition2 = obstacle2->point_ - position();

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
			if (bag_id == Value::unset && !owned_bag) {
				//if bag is unset as bag_id, set it here
				bag_id = other->id();
				owned_bag = other;
			}

			continue;
		}
		//reverse of previous - ensure bag takes full responsibility to avoid owner when no attempting to re-join
		else if (isBag() && std::stoi(currentGoal().targetName) == other->id()) {
			reciprocal_fov = 1.f;
			combinedRadius = radius() + other->radius(); //other radius is the physical person radius
			combinedRadiusSq = sqr(combinedRadius);
		}
		//if bag - try to ignore other agents
		else if (isBag() && !other->isBag()) {
			reciprocal_fov = 0.1f;
		}
		
		
		//if other agent is bag, and this is a person, take full responsibility for avoidance
		if (!this->isBag() && other->isBag()) {
			reciprocal_fov = 1.f;
		}

		// // If both agents are part of the same group
		// if (groupId() == other->groupId() && groupId() != Value::unset) {
		// 	combinedRadius = radius() + other->radius(); //other radius is the physical person radius
		// 	combinedRadiusSq = sqr(combinedRadius);
		// }


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

		line.point = velocity() + reciprocal_fov * u;
		orcaLines_.push_back(line);
	}

	size_t lineFail = linearProgram2(orcaLines_, _RVO2DParams.rvo_max_speed, _prefVelocity, false, _newVelocity);

	if (lineFail < orcaLines_.size()) {
		linearProgram3(orcaLines_, numObstLines, lineFail, _RVO2DParams.rvo_max_speed, _newVelocity);
	}

}

bool RVO2DAgent::hasGoalBehaviour(std::string key, std::string value) const
{
	BehaviourParameter tofind(key, value);
	auto paramvec = _goalQueue.front().targetBehaviour.getParameters();
	return (std::find(paramvec.begin(), paramvec.end(), tofind) != paramvec.end());
}

bool RVO2DAgent::hasGoalBehaviour(std::string key) const
{
	bool found = false;
	auto paramvec = _goalQueue.front().targetBehaviour.getParameters();
	for (auto it = paramvec.begin(); it != paramvec.end(); it++)
	{
		if (it->key == key) {
			return true;
		}
	}
	return false;
}

bool RVO2DAgent::hasAgentBehaviour(std::string name) const
{
	return (behaviours.find(name) != behaviours.end());
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

void RVO2DAgent::insertObstacleNeighbor(const ObstacleInterface* obstacle, float rangeSq)
{
	const ObstacleInterface* const nextObstacle = obstacle->nextObstacle_;
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

std::pair< Util::Vector, SteerLib::AgentGoalInfo> RVO2DAgent::updateAI_goal()
{
	Util::Vector goalDirection;
	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();

	if (!_midTermPath.empty())
	{	}

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
				if (!(*it)->enabled() && isBag())
				{
					disable();
				}

				goalDirection = normalize((*it)->position() - position());
				goalInfo.targetLocation = (*it)->position();

				// if bag is too far from owner - stop moving TODO: and don't steer
				if (((*it)->position() - position()).length() > BAG_DISTANCE && isBag() )
				{
					goalDirection = Util::Vector(0, 0, 0);
				}

				break;
			}
		}
	}
	// Set front goal as the nearest of the GOAL_TYPE_SEEK_STATIC_TARGET_SET goals
	else if (goalInfo.goalType == GOAL_TYPE_SEEK_STATIC_TARGET_SET)
	{
		// Shortest distance to nearest goal in the set
		float shortestDist = INFINITY;
		for (auto it = goalInfo.targetLocationsSet.begin(); it != goalInfo.targetLocationsSet.end(); it++) {
			if ((*it - position()).length() < shortestDist) {
				shortestDist = (*it - position()).length();
				goalInfo.targetLocation = *it;
				_goalQueue.front().targetLocation = *it;
			}
		}
		goalDirection = normalize(goalInfo.targetLocation - position());
	}
	else
	{
		goalDirection = normalize(goalInfo.targetLocation - position());
	}

	return std::make_pair(goalDirection, goalInfo);
}

void RVO2DAgent::updateAI_agentBehaviour()
{

}

std::pair < Util::Vector, float> RVO2DAgent::updateAI_groups()
{
	// When distance to COM is less than this, so not attempt to move towards the COM
	const float affectRadius = 0.2;

	// if no group then no additional direction
	if (groupId() == Value::unset)
		return std::make_pair(Util::Vector(0, 0, 0), 0);

	//calculate group centre of mass (COM)
	int inGroup = 1;
	Util::Point com = position();
	for (auto agent : getSimulationEngine()->getAgents())
	{
		// ignore self
		if (agent == this)
			continue;

		if (agent->groupId() == groupId())
		{
			inGroup++;
			com = com + agent->position();
		}
	}
	com = com / inGroup;

	//calculate the weight - proportional to distance away from COM
	float groupWeight = std::pow((com - position()).length(), 2) * 0.1;

	//Zero'd for 1 member groups, or when very close to COM
	Util::Vector dir = (inGroup == 1 || (com - position()).length() < affectRadius) ? Util::Vector(0, 0, 0) : normalize(com - position());

	return std::make_pair(dir, groupWeight);
}

void RVO2DAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	Util::AutomaticFunctionProfiler profileThisFunction( &RVO2DGlobals::gPhaseProfilers->aiProfiler );
	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	auto [goalDirection, goalInfo] = updateAI_goal();

	updateAI_agentBehaviour();

	auto [groupDirection, groupWeight] = updateAI_groups();

	_prefVelocity = (((goalDirection+ groupDirection).length() < 1) ? goalDirection + groupWeight * groupDirection : normalize(goalDirection + groupWeight * groupDirection)) * _goalQueue.front().desiredSpeed;
	(this)->computeNeighbors();
	(this)->computeNewVelocity(dt);
	_prefVelocity.y = 0.0f;

	// These are the internal RVO values calculated
	_velocity = _newVelocity;

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
		if (tooFarFromBag())
		{
			SteerLib::AgentGoalInfo newGoal;
			auto currentGoal = _goalQueue.front();

			newGoal.goalType = GOAL_TYPE_SEEK_DYNAMIC_TARGET;
			newGoal.timeDuration = currentGoal.timeDuration;
			newGoal.desiredSpeed = currentGoal.desiredSpeed;
			newGoal.targetName = std::to_string(owned_bag->id());

			// Add the door goal at front of queue
			_goalQueue.push_front(newGoal);
		}
	}


	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).length() < radius() * REACHED_WAYPOINT_MULTIPLIER)
	{
		_waypoints.erase(_waypoints.begin());
	}
	/*
	 * Now do the conversion from RVO2DAgent into the SteerSuite coordinates
	 */
	_velocity.y = 0.0f;

	//reached current goal - either 1: reach target waypoint, 2: within target box. Must not be a bag - which will only complete when the owner does
	if((((goalInfo.targetLocation - position()).length() < radius()*REACHED_GOAL_MULTIPLIER ) ||
			(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
				Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
				goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius()))) && 
		!isBag())
	{
		//save goal in memory
		if(hasGoalBehaviour("condition-memory-z"))
			completed_goals.push_back(_goalQueue.front());
		
		_goalQueue.pop_front();
		if (_goalQueue.size() >= 1) {
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else {
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}


	// Hear the 2D solution from RVO is converted into the 3D used by StecerSuite
	if ( velocity().length() > 0.0 )
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}

	/*
	 * Stuff for mesh database
	 */

	_position.y = getSimulationEngine()->getSpatialDatabase()->getLocation(this).y;

}


void RVO2DAgent::draw()
{
#ifdef ENABLE_GUI

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

	// for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
	// {
	// 	if ( _gEngine->isAgentSelected(this) )
	// 	{
	// 		DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
	// 	}
	// 	else
	// 	{
	// 		DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
	// 	}
	// }


	// for (int i=0; i < (_waypoints.size()); i++)
	// {
	// 	DrawLib::drawFlag(_waypoints.at(i), gBlue, 1.0);
	// }


	if (_gEngine->isAgentSelected(this))
	{
		// Draw ORCA lines
		for (int l=0; l < orcaLines_.size(); l++)
		{
			// Util::Point p = position() + Util::Point(orcaLines_.at(l).point.x, orcaLines_.at(l).point.y,
				//	orcaLines_.at(l).point.z);
			Util::Point p = position();//  + orcaLines_.at(l).point;
			DrawLib::drawLine(
					Util::Point(0,0,0) + orcaLines_.at(l).point - (orcaLines_.at(l).direction*_RVO2DParams.rvo_time_horizon),
					Util::Point(0,0,0) + orcaLines_.at(l).point + (orcaLines_.at(l).direction*_RVO2DParams.rvo_time_horizon),
					gOrange);
			//show which side is not permitted - draw repeated lines getting brighter in the prohibited region
			Util::Vector offset_vec = rotateInXZPlane(orcaLines_.at(l).direction, M_PI/2);
			float od = 0.03; // control how spaced out the repeated lines are. Larger value = more spacing
			int rmax = 10; //number of extra lines to draw
			for (int r=1; r<rmax; r++)
			{
				DrawLib::drawLineAlpha(
						Util::Point(0,0,0) + (offset_vec * r * od) + orcaLines_.at(l).point - (orcaLines_.at(l).direction*_RVO2DParams.rvo_time_horizon),
						Util::Point(0,0,0) + (offset_vec * r * od) + orcaLines_.at(l).point + (orcaLines_.at(l).direction*_RVO2DParams.rvo_time_horizon),
						gOrange, float(rmax-r)/float(rmax));
			}
		}
		//Draw desired velocity
		DrawLib::drawFlag(_position + _prefVelocity, gGreen);

		//Draw resulting velocity
		DrawLib::drawFlag(_position + _velocity, gRed, 2);

	}

#endif

#endif
}

