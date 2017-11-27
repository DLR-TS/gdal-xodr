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

    /**
     * Closes small gaps between road geometry parts in 2D. Roads as MultiLineStrings may consist of multiple individually 
     * sampled geometry parts which are not necessarily topologically touching. This can be replaced by proper snapping
     * later.
     * @param ogrRoad
     * @param tolerance
     */
    // TODO Move distance tolerance out into layer creation option
    void integrateGeometryParts(OGRMultiLineString* ogrRoad, const double tolerance) const;

    std::auto_ptr<OGRLineString> lineToLinestring(const geometry &geoParam) const;
    // TODO Move angleStepSize out into layer creation option
    std::auto_ptr<OGRLineString> arcToLinestring(const geometry &geoParam, const double angleStepSize = 0) const;
    // TODO Move sampleDistance out into layer creation option
    std::auto_ptr<OGRLineString> spiralToLinestring(const geometry &geoParam, const double sampleDistance) const;
    // TODO Move sampleDistance out into layer creation option
    std::auto_ptr<OGRLineString> poly3ToLinestring(const geometry &geoParam, const double sampleDistance) const;
    // TODO Move sampleDistance out into layer creation option
    std::auto_ptr<OGRLineString> paramPoly3ToLinestring(const geometry &geoParam, const double sampleDistance) const;
    std::auto_ptr<OGRLineString> toOGRGeometry(const geometry& xodrGeometry) const;
    OGRMultiLineString toOGRGeometry(const planView& planView) const;

    /**
     * Samples an arc.
     * @param length Arc length.
     * @param curvature Arc curvature.
     * @param angleStepSizeDegrees The arcs sampling angle step size in degrees.
     * @return The sampled arc.
     */
    OGRLineString* sampleArc(const double length, const double curvature,
            const double angleStepSizeDegrees) const;

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

    /**
     * Samples a cubic polynomial. The sample points are created relative to the coordinate system origin (0, 0).
     * @param length Length of the polynomial.
     * @param a Polynomial parameter a.
     * @param b Polynomial parameter b.
     * @param c Polynomial parameter c.
     * @param d Polynomial parameter d.
     * @param sampleDistance Point sample distance in coordinate system units (usually metres).
     * @param lineString The LineString to create from the sample points.
     */
    void samplePoly3(const double length, const double a, const double b, const double c, const double d,
            const double sampleDistance, OGRLineString* lineString) const;

    /**
     * Samples a parametric cubic polynomial. The sample points are created relative to the coordinate system 
     * origin (0, 0).
     * @param range Either the length of the geometry (of this cubic polynomial) or 1.0 in case of a normalised range.
     * @param uA Polynomial parameter uA.
     * @param uB Polynomial parameter uB.
     * @param uC Polynomial parameter uC.
     * @param uD Polynomial parameter uD.
     * @param vA Polynomial parameter vA.
     * @param vB Polynomial parameter vB.
     * @param vC Polynomial parameter vC.
     * @param vD Polynomial parameter vD.
     * @param stepSize The sample step size scaled appropriately to the range parameter.
     * @param lineString The LineString to create from the sample points.
     */
    void sampleParamPoly3(const double range, const double uA, const double uB, const double uC, const double uD,
            const double vA, const double vB, const double vC, const double vD,
            const double stepSize, OGRLineString* lineString) const;

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