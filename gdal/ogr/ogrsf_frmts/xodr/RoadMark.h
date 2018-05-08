/******************************************************************************
* $Id$
*
* Project:  OpenGIS Simple Features for OpenDRIVE
* Purpose:  Definition of converted road mark elements.
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

#ifndef ROADMARK_H
#define ROADMARK_H
#include <string>
#include "ogr_geometry.h"

class RoadMark {

public:
	RoadMark(void);
	~RoadMark(void);
	RoadMark(int laneId, std::string type, std::string weight, std::string color, std::string laneChange, std::string rule, OGRMultiLineString line);
	std::string parentRoadId;
	int laneId;
	std::string type ="";
	std::string weight = "";
	std::string color = "";
	std::string rule = "";
	std::string laneChange = "";
	OGRMultiLineString line;
	double length;
	double space; 
	double tOffset;
	double increment;
};
#endif /* ROADMARK_H */