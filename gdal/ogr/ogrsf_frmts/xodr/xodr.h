/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Definition of algorithms for conversion of OpenDRIVE to OGC Simple Features.
 * Author:   Michael Scholz, michael.scholz@dlr.de, German Aerospace Center (DLR)
 *			 Cristhian Eduardo Murcia Galeano, cristhianmurcia182@gmail.com
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
#include <math.h>       /* atan2 */

#define PI 3.14159265
#include "OpenDRIVE_1.4H.h"
#include "ogr_geometry.h"
#include <cmath>
#include <iostream>
#include <memory>   
#include <string>
#include <typeinfo>
#include <vector>
#include "MatrixTransformations2D.h"
#include "laneWidth.h"
#include "Border.h"
#include "Lane.h"
#include "RoadMark.h"
#include "Signal.h"
#include "Object.h"

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



	//Helper Functions
	
	OGRLineString multiLineStringToLineString(OGRMultiLineString* multiple) const;
	OGRPoint normalUnitaryVector(OGRLineString line) const;
	OGRPoint normalProjectedPoint(OGRPoint roadPoint, OGRPoint normalUnitaryVector, double borderWidth) const;
	OGRMultiLineString splitLineWithSpace(OGRLineString *lineSegment, double length, double space) const;
	std::vector<double> samplingRange(double init, double end, double increment)const;
	void printLine(OGRLineString line)const;
	OGRPoint valueInLine(OGRLineString line, double s)const;
	OGRLineString adjustLine(std::vector<double> range, OGRLineString line) const;
	void offsetLine(OGRLineString* lineSegment, double tOffset) const;
	void translate(OGRLineString* lineString, double x, double y) const;
	void samplePoly3(double init, double end, double a, double b, double c, double d, double sampleDistance, OGRLineString* lineString) const;
	
	//**********REFERENCE LINE********//

	OGRMultiLineString toOGRGeometry(const planView& planView) const;


	//********* LANES **********//

	std::vector<Lane> roadLanes(const road& xodrRoad)const;
	
	// When Lanes are defined with "BORDER" elements we use the following algorithms
	std::vector<Lane>  roadLanesB(const road& xodrRoad) const;
	bool rightBorderPresent(const lanes& lanes)const;
	bool leftBorderPresent(const lanes& lanes)const;		
	void samplePoly3Border(const border& borderSegment, double init, double end, const double sampleDistance, OGRLineString* lineString, char direction) const;
	OGRLineString projectLanesB(OGRLineString baseLine, OGRLineString lineSegment) const;	
	std::vector<OGRLineString> projectLanesW(std::vector<Lane> lanesRight, OGRMultiLineString referenceLine) const;
	std::vector<Border> createBorderList(double sLaneSectionI, double sLaneSectionE, const lane::border_sequence& borderSequence)const;
	OGRLineString borderLine(std::vector<Border> bList, char direction)const;
	void samplePoly3Border(Border b, OGRLineString* lineString, char direction) const;	
	std::vector<Lane> borderRight(const road& xodrRoad, OGRLineString  referenceLine)const;
	std::vector<Lane> borderLeft(const road& xodrRoad, OGRLineString  referenceLine)const;	
	
	
	//When Lanes are defined with "WIDTH" elements we use the following algorithms
	std::vector<Lane> roadLanesW(const road& xodrRoad)const;
	void samplePoly3Width(laneWidth w, OGRLineString* lineString, char direction) const;
	std::vector<laneWidth> createWidthList(double sLaneSectionI, double sLaneSectionE, const lane::width_sequence& widthSequence)const;
	OGRLineString widthLine(std::vector<laneWidth> wList, std::vector<double> fixedSamplingRange, char direction)const;
	std::vector<Lane> lanesWidthRight(const road& xodrRoad, const right::lane_sequence& lanes, double laneSectionI, double laneSectionE, std::string singleSide)const;
	std::vector<Lane> lanesWidthLeft(const road& xodrRoad, const left::lane_sequence& lanes, double laneSectionI, double laneSectionE, std::string singleSide)const;
	std::vector<Lane> lanesWidthRight(const road& xodrRoad, int laneSectionIndex, double roadLength)const;
	std::vector<Lane> lanesWidthLeft(const road& xodrRoad, int laneSectionIndex, double roadLength)const;	
	Lane laneOffset(const lanes::laneOffset_sequence& laneOffsetSeq, OGRLineString  referenceLine, std::string roadId)const;
	bool leftWidthPresent(const lanes& lanes)const;
	bool rightWidthPresent(const lanes& lanes)const;


	//********* ROAD MARKS **********//
	std::vector<RoadMark> roadMarks(const road& xodrRoad)const;
	std::vector<RoadMark> roadMarks(const lane::roadMark_sequence roadMarkSequence, double sLaneSectionI, double sLaneSectionE, Lane laneElement)const;
	RoadMark roadMarkElement(const roadMark& rMark, double roadMarkSI, double roadMarkSE, Lane laneElement)const;
	RoadMark roadMarkElement(const roadMark1& rMark, double roadMarkSI, double roadMarkSE, Lane laneElement)const;
	std::vector<RoadMark> roadMarksLanes(const road& xodrRoad)const;
	std::vector<RoadMark> roadMarksReferenceLine(const road& xodrRoad)const;
	RoadMark roadMarkElement(const roadMark1& rMark, double roadMarkSI, double roadMarkSE, OGRLineString  referenceLine, std::string roadId)const;

	//*****Signals****//
	std::vector<Signal> roadSignals(const road& xodrRoad) const;
	bool signalPresent(const road& xodrRoad) const;

	//*****Object****//
	bool objectPresent(const road& xodrRoad) const;
	bool objectLinePresent(const road& xodrRoad) const;
	bool objectPolygonPresent(const road& xodrRoad) const;
	std::vector<Object> roadObject(const road& xodrRoad) const;

	Object createPointObject(double s, double t, const object& o, OGRLineString  referenceLine, std::string roadId) const;
	OGRPolygon createPolygonObject(Object o, const outline::cornerLocal_sequence& cornerLocalSeq) const;
	OGRPolygon createPolygonObject(Object o, const outline::cornerRoad_sequence& cornerRoadSeq, OGRLineString  referenceLine) const;
};



#endif /* ndef _XODR_H_INCLUDED */