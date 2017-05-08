/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Definition of a cubic polynomials.
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

#ifndef CUBICPOLYNOMIALS_H
#define CUBICPOLYNOMIALS_H

#include <string>

class CubicPolynomial {
public:
    CubicPolynomial(const double coeffA, const double coeffB, const double coeffC, const double coeffD);
    CubicPolynomial(const CubicPolynomial& orig);
    virtual ~CubicPolynomial();
    double value(const double x);
private:
    double a;
    double b;
    double c;
    double d;
};

class ParametricCubicPolynomial {
public:
    ParametricCubicPolynomial(const double coeffUA, const double coeffUB, const double coeffUC, const double coeffUD,
            const double coeffVA, const double coeffVB, const double coeffVC, const double coeffVD);
    ParametricCubicPolynomial(const ParametricCubicPolynomial& orig);
    virtual ~ParametricCubicPolynomial();
    double valueU(const double parameter);
    double valueV(const double parameter);
private:
    CubicPolynomial polyU;
    CubicPolynomial polyV;
};

#endif /* CUBICPOLYNOMIALS_H */

