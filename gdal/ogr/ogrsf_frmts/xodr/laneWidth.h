/******************************************************************************
* $Id$
*
* Project:  OpenGIS Simple Features for OpenDRIVE
* Purpose:  Definition of converted lane width elements.
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

#ifndef LANEWIDTH_H
#define LANEWIDTH_H
#include <cmath>
class laneWidth {

public:
	laneWidth(void);
	~laneWidth(void);
	laneWidth(double i, double e, double aP, double bP, double cP, double dP);
	double init;
	double end;
	double a;
	double b;
	double c;
	double d;
	double polynomial(double s);
};
#endif /* LANEWIDTH_H */