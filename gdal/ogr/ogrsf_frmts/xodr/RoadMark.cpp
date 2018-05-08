/******************************************************************************
* $Id$
*
* Project:  OpenGIS Simple Features for OpenDRIVE
* Purpose:  Implementation of converted road mark elements.
* Author:   Michael Scholz, michael.scholz@dlr.de, German Aerospace Center (DLR)
*			Cristhian Eduardo Murcia Galeano, cristhianmurcia182@gmail.com
*
******************************************************************************
* Copyright 2017 German Aerospace Center (DLR), Institute of Transportation Systems
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
****************************************************************************/

#include "RoadMark.h"

RoadMark::RoadMark(void)
{
}

RoadMark::~RoadMark(void)
{
}

RoadMark::RoadMark(int laneId, std::string type, std::string weight, std::string color, std::string laneChange, std::string rule, OGRMultiLineString line) {
	this->laneId = laneId;
	this->type = type;
	this->weight = weight;
	this->color = color;
	this->laneChange = laneChange;
	this->rule = rule;
	this->line = line;
}