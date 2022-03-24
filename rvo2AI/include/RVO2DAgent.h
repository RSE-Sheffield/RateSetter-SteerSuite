//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __SIMPLE_AGENT__
#define __SIMPLE_AGENT__

/// @file SimpleAgent.h
/// @brief Declares the SimpleAgent class.
#include <queue>
#include "SteerLib.h"
// #include "SimpleAgent.h"
// #include "RVO2DAIModule.h"
#include "Obstacle.h"
#include "RVO2D_Parameters.h"


/**
 * @brief An example agent with very basic AI, that is part of the simpleAI plugin.
 *
 * This agent performs extremely simple AI using forces and Euler integration, simply
 * steering towards static goals without avoiding any other agents or static obstacles.
 * Agents that are "selected" in the GUI will have some simple annotations that
 * show how the spatial database and engine interface can be used.
 *
 * This class is instantiated when the engine calls SimpleAIModule::createAgent().
 *
 */

#define USE_ACCLMESH 1


class RVO2DAgent : public SteerLib::AgentInterface
{
public:
	RVO2DAgent();
	~RVO2DAgent();
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	std::pair<Util::Vector, SteerLib::AgentGoalInfo> updateAI_goal();
	void updateAI_goalBehaviour(Util::Vector& goalDirection, const SteerLib::AgentGoalInfo& goalInfo);
	void updateAI_agentBehaviour();
	std::pair<Util::Vector, float> updateAI_groups();
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void disable();
	void draw();

	bool enabled() const { return _enabled; }
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	Util::Vector velocity() const {return _velocity; }
	float radius() const { return _radius; }
	float sdradius() const { return _sdradius; }
	bool isBag() const { return _isBag;  }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _goalQueue.front(); }
	size_t id() const { return _id;}
	int groupId() const { return _groupId; }
	const std::deque<SteerLib::AgentGoalInfo> & agentGoals() const { return _goalQueue; }
	void addGoal(const SteerLib::AgentGoalInfo& newGoal);
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for ORCAAgent"); }
	void setParameters(SteerLib::Behaviour behave);
	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::SpatialDataBaseInterface spatial database;
	/// The Util namespace helper functions do the job nicely for basic circular agents.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}
	// Navigation to find velocity
	void path_planning(SteerLib::AgentGoalInfo goalInfo);

	//variables regarding social distance proximity metrics
	int SDviolation = 0;                      // total number of SD violations
	int arraymax = 2000;                      // initial array size - dynamically expanded later as needed
	int* SDframes = new int[arraymax];        // allocated to hold info on frames with SD violation
	float* SDDistance = new float[arraymax];  // allocated to hold distances of SD violation
	int* SDNeighbour = new int[arraymax];     // which agent is the SD violation with
	float* SDPositionX = new float[arraymax]; // allocated to hold position of SD violation
	float* SDPositionZ = new float[arraymax]; // allocated to hold position of SD violation
	float* SDPosXNeighbour = new float[arraymax]; // allocated to hold neighbour's position during SD violation
	float* SDPosZNeighbour = new float[arraymax]; // allocated to hold neighbour's position during SD violation
	status * SDStatus = new status[arraymax]; // allocated to hold boarding or alighting status of each conflicting agent


	// virtual void updateLocalTarget();
	// bool collidesAtTimeWith(const Util::Point & p1, const Util::Vector & rightSide, float otherAgentRadius, float timeStamp, float footX, float footZ);
	void insertAgentNeighbor(const SteerLib::AgentInterface * agent, float &rangeSq);
	// bool compareDist(SteerLib::AgentInterface * a1, SteerLib::AgentInterface * a2 );
#ifdef USE_ACCLMESH
	virtual void updateLocalTarget()
	{
		Util::Point tmpTarget = this->_goalQueue.front().targetLocation;
		// std::cout << "Size of mid term path: " << this->_midTermPath.size() << std::endl;
		unsigned int i=0;
		for (i=0; (i < 5) &&
				i < this->_midTermPath.size(); i++ )
		{
			tmpTarget = this->_midTermPath.at(i);
			if ( this->hasLineOfSightTo(tmpTarget) )
			{
				this->_currentLocalTarget = tmpTarget;
			}
		}
		this->_currentLocalTarget = _waypoints.front();
	}
#endif
	//int chosen_door;

	bool tooFarFromBag();

protected:
	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	// void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);

	RVO2DParameters _RVO2DParams;
	virtual SteerLib::EngineInterface * getSimulationEngine();

	/**
	 * \brief      Inserts a static obstacle neighbor into the set of neighbors
	 *             of this agent.
	 * \param      obstacle        The number of the static obstacle to be
	 *                             inserted.
	 * \param      rangeSq         The squared range around this agent.
	 */
	void insertObstacleNeighbor(const ObstacleInterface *obstacle, float rangeSq);

	/**
	 * \brief   Computes the neighbors of this agent.
	 */
	void computeNeighbors();

	/**
	 * \brief   Computes the new velocity of this agent.
	 */
	void computeNewVelocity(float dt);


	/**
	* \brief check if this agent, for this current goal, has a specified behaviour "key"
	*/
	bool hasGoalBehaviour(std::string key, std::string value) const;
	bool hasGoalBehaviour(std::string key) const;

	bool hasAgentBehaviour(std::string name) const;

	// Add a goal to the front of the _goalQueue
	void addGoalToFront(SteerLib::AgentGoalInfo goal);

	// iterate through completed goals, see if they have memory conditions, and add them to front of the queue if yes.
	void rememberGoals();

	/**
		 * \brief   Updates the three-dimensional position and three-dimensional velocity of this agent.
		 */
	void update(float timeStamp, float dt, unsigned int frameNumber);

	// Stuff specific to RVO
	// should be normalized
	// Util::Vector prefVelocity_; // This is the velocity the agent wants to be at
	// Util::Vector newVelocity_;
	size_t maxNeighbors_;
	float maxSpeed_;
	float neighborDist_;
	float timeHorizon_;
	float timeHorizonObst_;
	int next_waypoint_distance_;
	// std::vector<std::pair<float, const SteerLib::AgentInterface *> > agentNeighbors_;
	// std::vector<std::pair<float, const Obstacle *> > obstacleNeighbors_;
	std::vector<Util::Plane> orcaPlanes_;
	std::vector<Line> orcaLines_;
	SteerLib::ModuleInterface * rvoModule;
	size_t bag_id = -1;
	const AgentInterface * owned_bag = NULL; // Reference to the bag(s) owned by this actor
	bool far_bag_count_flag = false; 
	std::vector<int> far_bag_count; //vector of frames duration for each time owner and bag are separated

	SteerLib::EngineInterface * _gEngine;

	//hack PTI behaviour
	bool bHitFirstGoal = false;


	friend class KdTree;
	friend class RVO2DAIModule;

	////Agent PTI radius
	//float _max_radius;
	//float _min_radius;
	//float _near_dist;
	//float _far_dist;
	// how many frames spent less than SD.
	//int close_frames = 0;
	//bool counted_this_frame = false;
	//float _receprocity_factor;
	//status loading_status;

	std::list< SteerLib::AgentGoalInfo> completed_goals;


};

/**
 * \relates    Agent
 * \brief      Solves a one-dimensional linear program on a specified line
 *             subject to linear constraints defined by lines and a circular
 *             constraint.
 * \param      lines         Lines defining the linear constraints.
 * \param      lineNo        The specified line constraint.
 * \param      radius        The radius of the circular constraint.
 * \param      optVelocity   The optimization velocity.
 * \param      directionOpt  True if the direction should be optimized.
 * \param      result        A reference to the result of the linear program.
 * \return     True if successful.
 */
bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
					float radius, const Util::Vector &optVelocity,
					bool directionOpt, Util::Vector &result);

/**
 * \relates    Agent
 * \brief      Solves a two-dimensional linear program subject to linear
 *             constraints defined by lines and a circular constraint.
 * \param      lines         Lines defining the linear constraints.
 * \param      radius        The radius of the circular constraint.
 * \param      optVelocity   The optimization velocity.
 * \param      directionOpt  True if the direction should be optimized.
 * \param      result        A reference to the result of the linear program.
 * \return     The number of the line it fails on, and the number of lines if successful.
 */
size_t linearProgram2(const std::vector<Line> &lines, float radius,
					  const Util::Vector &optVelocity, bool directionOpt,
					  Util::Vector &result);

/**
 * \relates    Agent
 * \brief      Solves a two-dimensional linear program subject to linear
 *             constraints defined by lines and a circular constraint.
 * \param      lines         Lines defining the linear constraints.
 * \param      numObstLines  Count of obstacle lines.
 * \param      beginLine     The line on which the 2-d linear program failed.
 * \param      radius        The radius of the circular constraint.
 * \param      result        A reference to the result of the linear program.
 */
void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
					float radius, Util::Vector &result);

#endif
