/******************************************************************************
* $Id$
*
* Project:  OpenGIS Simple Features for OpenDRIVE
* Purpose:  Implementation of converted signal elements.
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

#include "Signal.h"

Signal::Signal(void)
{
}

Signal::~Signal(void)
{
}

Signal::Signal(std::string roadId, std::string id, std::string name, std::string dynamic, std::string orientation, std::string type, std::string country, std::string subtype, OGRPoint position){
	this->roadId = roadId;
	this->id = id;
	this->name = name;
	this->dynamic = dynamic;
	this->orientation = orientation;
	this->type = type;
	this->country = country;
	this->subtype = subtype;
	this->position = position;	
}