/******************************************************************************
* $Id$
*
* Project:  OpenGIS Simple Features for OpenDRIVE
* Purpose:  Definition of converted object elements.
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

#ifndef OBJECT_H
#define OBJECT_H
#include <string>
#include "ogr_geometry.h"
class Object {

	//<object id="268444380" type="pole" s="140.99342772971738" t="-6.2698130102249285" height="2.338565367367238" zOffset="0" roll="0" pitch="0" hdg="0" validLength="0" orientation="none" 
	//width="0.040000000000000001" length="0.040000000000000001" name="roadsignpole" />

	//<object id="6811" s="0" t="6.7160576200754152" orientation="none" validLength="21.065665623564225"
	//length="0" roll="0" pitch="0" hdg="0" type="railing" name="guardrail" width="0.080000000000000002" height="0.17999999999999999">

	//<repeat length = "21.065665623564225" s = "0" distance = "0" tStart = "6.7160576200754152" tEnd = "6.5104920565721827" zOffsetStart = "0.58621251553297049" 
	//zOffsetEnd = "0.57563350819814452" heightStart = "0.17999999999999999" heightEnd = "0.17999999999999999" / >

public:
	Object(void);
	~Object(void);
	Object(OGRPoint position);
	std::string roadId;
	std::string id;
	std::string type;
	double height;
	double zOffset;
	std::string roll;
	std::string pitch;
	double hdg = 0;
	std::string radius;
	std::string validLength = "";
	std::string orientation = "";
	std::string parkingSpace = "";
	std::string material = "";
	std::string geometryType = "";
	double width;
	double length;
	std::string name = "";
	double zOffsetStart;
	double zOffsetEnd;
	double heightStart;
	double heightEnd;
	OGRPoint position;
	OGRPolygon shape;
	OGRMultiLineString line;
};
#endif /* OBJECT_H */