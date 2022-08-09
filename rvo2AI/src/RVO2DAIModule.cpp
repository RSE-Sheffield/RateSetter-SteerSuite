//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file RVO2DAIModule.cpp
/// @brief Implements the RVO2DAIModule plugin.


#include "SteerLib.h"
#include "SimulationPlugin.h"
#include "RVO2DAIModule.h"
#include "RVO2DAgent.h"

#include "LogObject.h"
#include "LogManager.h"


// globally accessible to the simpleAI plugin
// SteerLib::EngineInterface * gEngine;
// SteerLib::SpatialDataBaseInterface * gSpatialDatabase;


float depths = 0.5f;
std::vector<Util::Point> traindoors = { Util::Point(-13,0,depths),
								Util::Point(-7,0,depths),
								Util::Point(7,0,depths),
								Util::Point(13,0,depths),
								Util::Point(27, 0, depths),
								Util::Point(33, 0, depths) };



namespace RVO2DGlobals
{

	// SteerLib::EngineInterface * gEngineInfo;
	// SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	unsigned int gLongTermPlanningPhaseInterval;
	unsigned int gMidTermPlanningPhaseInterval;
	unsigned int gShortTermPlanningPhaseInterval;
	unsigned int gPredictivePhaseInterval;
	unsigned int gReactivePhaseInterval;
	unsigned int gPerceptivePhaseInterval;
	bool gUseDynamicPhaseScheduling;
	bool gShowStats;
	bool gShowAllStats;
	bool dont_plan;


	// Adding a bunch of parameters so they can be changed via input
	float rvo_neighbor_distance;
	float rvo_time_horizon;
	float rvo_max_speed;
	float rvo_preferred_speed;
	float rvo_time_horizon_obstacles;
	int rvo_max_neighbors;
	int next_waypoint_distance;


	PhaseProfilers* gPhaseProfilers;
}

using namespace RVO2DGlobals;

PLUGIN_API SteerLib::ModuleInterface* createModule()
{
	return new RVO2DAIModule;
}

PLUGIN_API void destroyModule(SteerLib::ModuleInterface* module)
{
	delete module;
}


void RVO2DAIModule::init(const SteerLib::OptionDictionary& options, SteerLib::EngineInterface* engineInfo)
{
	_gEngine = engineInfo;
	// gSpatialDatabase = engineInfo->getSpatialDatabase();

	gUseDynamicPhaseScheduling = false;
	gShowStats = false;
	logStats = false;
	gShowAllStats = false;
	logFilename = "rvo2AI.log";
	dont_plan = true;

	rvo_max_neighbors = MAX_NEIGHBORS;
	rvo_max_speed = MAX_SPEED;
	rvo_neighbor_distance = NEIGHBOR_DISTANCE;
	rvo_time_horizon = TIME_HORIZON;
	rvo_time_horizon_obstacles = TIME_HORIZON_OBSTACLES;
	next_waypoint_distance = NEXT_WAYPOINT_DISTANCE;

	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter) {
		std::stringstream value((*optionIter).second);
		if ((*optionIter).first == "")
		{
			value >> gLongTermPlanningPhaseInterval;
		}
		else if ((*optionIter).first == "rvo_neighbor_distance")
		{
			value >> rvo_neighbor_distance;
		}
		else if ((*optionIter).first == "rvo_time_horizon")
		{
			value >> rvo_time_horizon;
		}
		else if ((*optionIter).first == "rvo_max_speed")
		{
			value >> rvo_max_speed;
		}
		else if ((*optionIter).first == "rvo_preferred_speed")
		{
			value >> rvo_preferred_speed;
		}
		else if ((*optionIter).first == "rvo_time_horizon_obstacles")
		{
			std::cout << "Setting rvo_time_horizon_obstacles to: " << value.str() << std::endl;
			value >> rvo_time_horizon_obstacles;
		}
		else if ((*optionIter).first == "rvo_max_neighbors")
		{
			value >> rvo_max_neighbors;
		}
		else if ((*optionIter).first == "next_waypoint_distance")
		{
			std::cout << "Setting next_waypoint_distance to: " << value.str() << std::endl;
			value >> next_waypoint_distance;
		}
		else if ((*optionIter).first == "ailogFileName")
		{
			logFilename = value.str();
			logStats = true;
		}
		else if ((*optionIter).first == "stats")
		{
			gShowStats = Util::getBoolFromString(value.str());
		}
		else if ((*optionIter).first == "allstats")
		{
			gShowAllStats = Util::getBoolFromString(value.str());
		}
		else if ((*optionIter).first == "dont_plan")
		{
			dont_plan = Util::getBoolFromString(value.str());
		}
		else
		{
			// throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to PPR AI module.");
		}
	}

	_rvoLogger = LogManager::getInstance()->createLogger(logFilename, LoggerType::BASIC_WRITE);

	_rvoLogger->addDataField("number_of_times_executed", DataType::LongLong);
	_rvoLogger->addDataField("total_ticks_accumulated", DataType::LongLong);
	_rvoLogger->addDataField("shortest_execution", DataType::LongLong);
	_rvoLogger->addDataField("longest_execution", DataType::LongLong);
	_rvoLogger->addDataField("fastest_execution", DataType::Float);
	_rvoLogger->addDataField("slowest_execution", DataType::Float);
	_rvoLogger->addDataField("average_time_per_call", DataType::Float);
	_rvoLogger->addDataField("total_time_of_all_calls", DataType::Float);
	_rvoLogger->addDataField("tick_frequency", DataType::Float);

	if (logStats)
	{
		// LETS TRY TO WRITE THE LABELS OF EACH FIELD
		std::stringstream labelStream;
		unsigned int i;
		for (i = 0; i < _rvoLogger->getNumberOfFields() - 1; i++)
			labelStream << _rvoLogger->getFieldName(i) << " ";
		labelStream << _rvoLogger->getFieldName(i);
		// _data = labelStream.str() + "\n";

		_rvoLogger->writeData(labelStream.str());

	}
}

void RVO2DAIModule::initializeSimulation()
{
	//
	// initialize the performance profilers
	//
	gPhaseProfilers = new PhaseProfilers;
	gPhaseProfilers->aiProfiler.reset();
	gPhaseProfilers->longTermPhaseProfiler.reset();
	gPhaseProfilers->midTermPhaseProfiler.reset();
	gPhaseProfilers->shortTermPhaseProfiler.reset();
	gPhaseProfilers->perceptivePhaseProfiler.reset();
	gPhaseProfilers->predictivePhaseProfiler.reset();
	gPhaseProfilers->reactivePhaseProfiler.reset();
	gPhaseProfilers->steeringPhaseProfiler.reset();

}

void RVO2DAIModule::finish()
{
	// nothing to do here
}

void RVO2DAIModule::preprocessSimulation()
{
	// kdTree_->buildObstacleTree();
}

void RVO2DAIModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	if (frameNumber == 1)
	{
		// Adding in this extra one because it seemed sometimes agents would forget about obstacles.
		// kdTree_->buildObstacleTree();
	}
	if (!agents_.empty())
	{
		// kdTree_->buildAgentTree();
	}

	/*
	for (int i = 0; i < static_cast<int>(agents_.size()); ++i)
	{
		dynamic_cast<RVO2DAgent *>(agents_[i])->computeNeighbors();
		dynamic_cast<RVO2DAgent *>(agents_[i])->computeNewVelocity(dt);
	}*/

}

void RVO2DAIModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	//SDproximity metrics
	float TotalMetric = 0;
	int TotalFrames = 0;
	float AgentMetric = 0;
	float deltaMetric;
	float distSq;
	Util::Vector relativePosition;
	SteerLib::AgentInterface* other;
	float Agentmean;
	int NeighbourCount;
	float Agentmin;
	float Agentmax;
	float SDplanned;

	//check if quiting this iteration
	bool quitting = true;
	for (auto& agent : agents_) {
		if (!agent->finished()) {
			quitting = false;
			break;
		}
	}

	//Record people being too far from their bags
	for (auto& agent : agents_) {
		//this person has a bag
		if (!agent->isBag() && agent->bag_id != -1)
		{
			if (agent->tooFarFromBag())
			{
				if (agent->far_bag_count_flag) {
					agent->far_bag_count.back()++;
				}
				else {
					agent->far_bag_count.push_back(1);
					agent->far_bag_count_flag = true;
				}
			}
			else {
				agent->far_bag_count_flag = false;
			}
		}
	}
}
void RVO2DAIModule::printBagsToFile(std::string outfile)
{
	std::ofstream BagsFile(outfile);
	for (auto& agent : agents_) {
		std::string towrite = std::to_string(agent->id());
		for (auto& frames : agent->far_bag_count) {
			towrite += "\t" + std::to_string(frames);
		}
		towrite += "\n";
		BagsFile << towrite;
	}

	BagsFile.close();
}
SteerLib::AgentInterface* RVO2DAIModule::createAgent()
{
	RVO2DAgent* agent = new RVO2DAgent;
	agent->rvoModule = this;
	agent->_id = agents_.size();
	agents_.push_back(agent);
	agent->_gEngine = this->_gEngine;
	return agent;
}

void RVO2DAIModule::destroyAgent(SteerLib::AgentInterface* agent)
{
	delete agent;
}

void RVO2DAIModule::cleanupSimulation()
{
	agents_.clear();
	// kdTree_->deleteObstacleTree(kdTree_->obstacleTree_);
	// kdTree_->agents_.clear();

	LogObject rvoLogObject;

	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getNumTimesExecuted());
	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTicksAccumulated());
	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinTicks());
	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxTicks());
	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinExecutionTimeMills());
	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxExecutionTimeMills());
	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getAverageExecutionTimeMills());
	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTime());
	rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getTickFrequency());

	_logData.push_back(rvoLogObject.copy());
	if (logStats)
	{
		_rvoLogger->writeLogObject(rvoLogObject);

		// cleanup profiling metrics for next simulation/scenario
	}

	gPhaseProfilers->aiProfiler.reset();
	gPhaseProfilers->longTermPhaseProfiler.reset();
	gPhaseProfilers->midTermPhaseProfiler.reset();
	gPhaseProfilers->shortTermPhaseProfiler.reset();
	gPhaseProfilers->perceptivePhaseProfiler.reset();
	gPhaseProfilers->predictivePhaseProfiler.reset();
	gPhaseProfilers->reactivePhaseProfiler.reset();
	gPhaseProfilers->steeringPhaseProfiler.reset();
	// kdTree_->deleteObstacleTree(kdTree_->obstacleTree_);
}