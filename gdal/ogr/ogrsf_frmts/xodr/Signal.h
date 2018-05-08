/******************************************************************************
* $Id$
*
* Project:  OpenGIS Simple Features for OpenDRIVE
* Purpose:  Definition of converted signal elements.
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

#ifndef SIGNAL_H
#define SIGNAL_H
#include <string>
#include "ogr_geometry.h"

class Signal {

	//s="1.5556690634801200e+02" t="0.0000000000000000e+00" id="300606" name="SgRMHoldingline-1Lane-300.flt" dynamic="no" orientation="+" zOffset="0.0000000000000000e+00" type="294" country="OpenDRIVE" subtype="-1" hOffset="0.0000000000000000e+00" pitch="0.0000000000000000e+00" roll="0.0000000000000000e+00" height="0.03">
public:
	Signal(void);
	~Signal(void);
	Signal(std::string roadId, std::string id, std::string name, std::string dynamic, std::string orientation, std::string type, std::string country,  std::string subtype, OGRPoint position);
	std::string roadId;
	std::string id;
	std::string name = "";
	std::string dynamic = "";
	std::string orientation = "";
	std::string type = "";
	std::string country = "";
	std::string subtype = "";
	OGRPoint position;
};
#endif /* SIGNAL_H */