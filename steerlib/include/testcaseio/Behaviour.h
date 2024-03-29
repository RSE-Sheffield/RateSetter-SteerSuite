//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/*
 * Behaviour.h
 *
 *  Created on: 2013-12-05
 *      Author: glenpb
 */

#ifndef BEHAVIOUR_H_
#define BEHAVIOUR_H_

#include "Globals.h"
#include "BehaviourParameter.h"
#ifdef _WIN32
// see steerlib/util/DrawLib.h for explanation
#pragma warning( push )
#pragma warning( disable : 4251 )
#endif

#include <string>
#include <vector>
namespace SteerLib 
{
	class STEERLIB_API Behaviour
	{
	public:
		Behaviour();
		Behaviour(std::string steeringAlg, std::vector<BehaviourParameter> parameters);
		~Behaviour();

		void addParameter(BehaviourParameter param);
		void setSteeringAlg(std::string algName);
		void setName(std::string name);
		std::string getSteeringAlg() const;
		std::vector<BehaviourParameter> getParameters() const;
		std::string getName() const;


	private:
		std::vector<BehaviourParameter > parameters;
		std::string steeringAlg;
		std::string name;
	};
}

#endif /* BEHAVIOUR_H_ */
