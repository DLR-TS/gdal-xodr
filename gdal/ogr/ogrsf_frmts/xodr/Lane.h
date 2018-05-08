/******************************************************************************
* $Id$
*
* Project:  OpenGIS Simple Features for OpenDRIVE
* Purpose:  Definition of converted lane elements.
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

#ifndef LANE_H
#define LANE_H
#include <string>
#include "ogr_geometry.h"
#include "RoadMark.h"
#include <OpenDRIVE_1.4H.h>

class Lane {

public:
	Lane(void);
	~Lane(void);
	Lane(int id, std::string type, std::string level, OGRLineString line);
	std::string parentRoadName;
	std::string parentRoadId;
	int id;
	std::string type;
	std::string singleSide ="";
	OGRLineString line;
	std::string level;
	std::vector<RoadMark> roadMarksCollection;
	const lane::roadMark_sequence rMarkSequence;
	
};
#endif /* LANE_H */