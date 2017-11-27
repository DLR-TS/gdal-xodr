/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Implementation of a cubic polynomials.
 * Author:   Michael Scholz, michael.scholz@dlr.de, German Aerospace Center (DLR)
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

#include "CubicPolynomials.h"
#include <cmath>

CubicPolynomial::CubicPolynomial(const double coeffA, const double coeffB, const double coeffC, const double coeffD)
: a(coeffA), b(coeffB), c(coeffC), d(coeffD) {
}

CubicPolynomial::CubicPolynomial(const CubicPolynomial& orig) {
    a = orig.a;
    b = orig.b;
    c = orig.c;
    d = orig.d;
}

CubicPolynomial::~CubicPolynomial() {
}

double CubicPolynomial::value(const double x) {
    return a + b * x + c * pow(x, 2) + d * pow(x, 3);
}

ParametricCubicPolynomial::ParametricCubicPolynomial(const double coeffUA, const double coeffUB, const double coeffUC,
        const double coeffUD, const double coeffVA, const double coeffVB, const double coeffVC, const double coeffVD)
: polyU(coeffUA, coeffUB, coeffUC, coeffUD), polyV(coeffVA, coeffVB, coeffVC, coeffVD) {
}

ParametricCubicPolynomial::ParametricCubicPolynomial(const ParametricCubicPolynomial& orig) 
: polyU(orig.polyU), polyV(orig.polyV) {
}

ParametricCubicPolynomial::~ParametricCubicPolynomial() {
}

double ParametricCubicPolynomial::valueU(const double parameter) {
    return polyU.value(parameter);
}

double ParametricCubicPolynomial::valueV(const double parameter) {
    return polyV.value(parameter);
}