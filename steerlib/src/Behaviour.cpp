//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/*
 * Behaviour.cpp
 *
 *  Created on: 2013-12-05
 *      Author: glenpb
 */

#include "testcaseio/Behaviour.h"

using namespace SteerLib;

Behaviour::Behaviour() 
{
	// TODO Auto-generated constructor stub

}

Behaviour::Behaviour(std::string steeringAlg, std::vector<BehaviourParameter > parameters)
{
	// TODO Auto-generated constructor stub
	this->steeringAlg = steeringAlg;
	this->parameters = parameters;
}

void Behaviour::addParameter(BehaviourParameter param)
{
	this->parameters.push_back(param);
}

void Behaviour::setSteeringAlg(std::string algName)
{
	this->steeringAlg = algName;
}

void SteerLib::Behaviour::setName(std::string name)
{
	this->name = name;
}

std::string Behaviour::getSteeringAlg() const
{
	return this->steeringAlg;
}

std::vector<BehaviourParameter> Behaviour::getParameters() const
{
	return this->parameters;
}

std::string Behaviour::getName() const
{
	return this->name;
}

Behaviour::~Behaviour()
{
	// TODO Auto-generated destructor stub
	this->parameters.clear();
}