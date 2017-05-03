/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Definition of algorithms for conversion of OpenDRIVE to OGC Simple Features.
 * Author:   Michael Scholz, michael.scholz@dlr.de, German Aerospace Center (DLR)
 *			 Ana Maria Orozco, ana.orozco.net@gmail.com
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


#ifndef _XODR_H_INCLUDED
#define _XODR_H_INCLUDED

#include "OpenDRIVE_1.4H.h"
#include "ogr_geometry.h"
#include <cmath>
#include <iostream>
#include <memory>   
#include <string>
#include <typeinfo>
#include <vector>
#include "MatrixTransformations2D.h"

class XODR {
    std::auto_ptr<OpenDRIVE> op;

public:
    XODR(const char *pszFilename);
    ~XODR();
    int getMinorRevision();
    const OpenDRIVE::road_sequence& getXODRRoads() const;
    void integrateGeometryParts(OGRMultiLineString* ogrRoad) const;
    std::auto_ptr<OGRLineString> lineToLinestring(const geometry &geoParam) const;
    std::auto_ptr<OGRLineString> arcToLinestring(const geometry &geoParam) const;
    std::auto_ptr<OGRLineString> spiralToLinestring(const geometry &geoParam) const;
    std::auto_ptr<OGRLineString> toOGRGeometry(const geometry& xodrGeometry) const;
    OGRMultiLineString toOGRGeometry(const planView& planView) const;
    
    /**
     * Samples a "default" Euler spiral, i.e. a spiral with start curvature 0.0. The sample points are created
     * relative to the coordinate system origin (0, 0).
     * @param length Length of the spiral.
     * @param endCurvature End curvature of the Euler spiral.
     * @param sampleDistance Point sample distance in coordinate system units (usually metres).
     * @param lineString The LineString to create from the sample points.
     * @return Tangent direction at sampled end point in radians. This is useful for rotations.
     */
    double sampleDefaultSpiral(const double length, const double endCurvature, const double sampleDistance,
            OGRLineString* lineString) const;
    
    int getNumberOfRoads();
    std::string getGeoReferenceString();
    
    /**
     * Performs a 2D affine geometry transformation with the help of a transformation matrix.
     * @param geom The geometry to transform.
     * @param matrix The 2D affine transformation matrix.
     */
    void transformLineString(OGRLineString* geom, const Matrix2D& matrix) const;
};
#endif /* ndef _XODR_H_INCLUDED */