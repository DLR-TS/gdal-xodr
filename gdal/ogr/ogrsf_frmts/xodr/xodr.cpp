/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Implementation of algorithms for conversion of OpenDRIVE to OGC Simple Features.
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


#include <stdlib.h>

#ifdef _WIN32
#define WINPAUSE system("pause")
#endif

extern "C" {
#include <odrSpiral.h>
}
#include <cmath>
#include <iostream>
#include <memory>   
#include <string>
#include <typeinfo>
#include <vector>
#include "cpl_conv.h"
#include <OpenDRIVE_1.4H.h>
#include <sstream>
#include "xodr.h"
#include "ogr_geometry.h"
#include "CubicPolynomials.h"
#include "MatrixTransformations2D.h"
#include "laneWidth.h"
#include "Border.h"
#include <algorithm>
#include <vector>
#include "Lane.h"
#include "RoadMark.h"
#include "Signal.h"
#include "Object.h"

using namespace std;
using namespace xml_schema;

XODR::XODR(const char * pszFilename) {
    op = OpenDRIVE_(pszFilename, flags::keep_dom | flags::dont_validate);
}

XODR::~XODR() {
}

int XODR::getMinorRevision() {
	
    OpenDRIVE::header_type h = op->header();
    header::revMinor_optional& minorRev = h.revMinor();
    int minorRevision = minorRev.get();
	
    return minorRevision;
}

std::string XODR::getGeoReferenceString() {

    OpenDRIVE::header_type h;
    h = op->header();
    header::geoReference_optional& geoRef = h.geoReference();
    return geoRef.present() ? geoRef.get().c_str() : "";
}

const OpenDRIVE::road_sequence& XODR::getXODRRoads() const {
    return op->road();
}

OGRMultiLineString XODR::toOGRGeometry(const planView& planView) const {
    const planView::geometry_sequence& geometries = planView.geometry();

    OGRMultiLineString ogrRoad;
    for (planView::geometry_const_iterator it = geometries.begin(); it != geometries.end(); ++it) {
        const geometry& xodrGeometry = *it;
        std::auto_ptr<OGRLineString> ogrGeometry = toOGRGeometry(xodrGeometry);
        if (!ogrGeometry->IsEmpty()) {
            ogrRoad.addGeometry(ogrGeometry.get());
        }
    }
    integrateGeometryParts(&ogrRoad, 0.01);
    return ogrRoad;
}

void XODR::integrateGeometryParts(OGRMultiLineString* ogrRoad, const double tolerance) const {
    int numGeoms = ogrRoad->getNumGeometries();
    if (numGeoms > 1) {
        for (int g = 1; g < ogrRoad->getNumGeometries(); ++g) {
            OGRLineString* current = static_cast<OGRLineString*> (ogrRoad->getGeometryRef(g));
            OGRPoint* currentStart = new OGRPoint();
            current->getPoint(0, currentStart);

            OGRLineString* pred = static_cast<OGRLineString*> (ogrRoad->getGeometryRef(g - 1));
            int numPts = pred->getNumPoints();
            OGRPoint* predEnd = new OGRPoint();
            pred->getPoint(numPts - 1, predEnd);
            if (currentStart->Disjoint(predEnd)) {
                if (currentStart->Distance(predEnd) < tolerance) {
                    pred->setPoint(numPts - 1, currentStart);
                } else {
                   // CPLError(CE_Warning, CPLE_AppDefined, "Road geometry parts are disjoint at point (%lf, %lf) with a distance greater than %lf", currentStart->getX(), currentStart->getY(), tolerance);
                }
            }
        }

    }
}
std::auto_ptr<OGRLineString> XODR::lineToLinestring(const geometry &geom) const {
    // Basic geometry attributes    
    double xStart = geom.x().get();
    double yStart = geom.y().get();
    double hdg = geom.hdg().get();
    double length = geom.length().get();
	
    std::auto_ptr<OGRLineString> lineString(new OGRLineString());
    OGRPoint ptStart(0.0, 0.0);
    lineString->addPoint(&ptStart);
    OGRPoint ptEnd(length, 0.0);
    lineString->addPoint(&ptEnd);

    Matrix2D m;
    MatrixTransformations::initMatrix(m);
    // T(geomStartPt) x R(geomHdg) x geometry
    MatrixTransformations::translate(m, xStart, yStart);
    MatrixTransformations::rotate(m, hdg);
    transformLineString(lineString.get(), m);

    return lineString;
}

std::auto_ptr<OGRLineString> XODR::arcToLinestring(const geometry &geom, const double angleStepSize) const {
    // Basic geometry attributes    
    double xStart = geom.x().get();
    double yStart = geom.y().get();
    double hdg = geom.hdg().get();
    double length = geom.length().get();

    // Arc attributes
    const geometry::arc_optional& arc = geom.arc();
    double curvature = arc->curvature().get();

    OGRLineString* arcGeom = sampleArc(length, curvature, angleStepSize);

    Matrix2D m;
    MatrixTransformations::initMatrix(m);
    // T(geomStartPt) x R(geomHdg) x geometry
    MatrixTransformations::translate(m, xStart, yStart);
    MatrixTransformations::rotate(m, hdg);
    transformLineString(arcGeom, m);

    return std::auto_ptr<OGRLineString>(arcGeom);
}

std::auto_ptr<OGRLineString> XODR::spiralToLinestring(const geometry &geom, const double sampleDistance) const {
    // Basic geometry attributes
    double xStart = geom.x().get();
    double yStart = geom.y().get();
    double hdg = geom.hdg().get();
    double length = geom.length().get();

    // Spiral attributes
    const geometry::spiral_optional& spiral = geom.spiral();
    double startCurvature = spiral->curvStart().get();
    double endCurvature = spiral->curvEnd().get();

    std::auto_ptr<OGRLineString> lineString(new OGRLineString());
    Matrix2D m;
    MatrixTransformations::initMatrix(m);
    if (startCurvature == 0.0 && endCurvature != 0.0) {
        sampleDefaultSpiral(length, endCurvature, sampleDistance, lineString.get());

        // T(geomStartPt) x R(geomHdg) x geometry
        MatrixTransformations::translate(m, xStart, yStart);
        MatrixTransformations::rotate(m, hdg);
        transformLineString(lineString.get(), m);
    } else if (startCurvature != 0.0 && endCurvature == 0.0) {
        double tangentDir = sampleDefaultSpiral(length, -startCurvature, sampleDistance, lineString.get());
        lineString->reversePoints();
        tangentDir += M_PI; // To comply with the line reversion
        OGRPoint lineStart;
        lineString->StartPoint(&lineStart);

        // T(geomStartPt) x R(-(tangentDir - geomHdg)) x T(-lineStartPt) x reversedGeometry
        MatrixTransformations::translate(m, xStart, yStart);
        MatrixTransformations::rotate(m, -(tangentDir - hdg));
        MatrixTransformations::translate(m, -lineStart.getX(), -lineStart.getY());
        transformLineString(lineString.get(), m);
    } else if (startCurvature == 0.0 && endCurvature == 0.0) {
        return lineToLinestring(geom);
    } else {
        CPLError(CE_Warning, CPLE_NotSupported, "Spirals having both curvStart and curvEnd != 0.0 not supported");
    }

    return lineString;
}

std::auto_ptr<OGRLineString> XODR::poly3ToLinestring(const geometry &geom, const double sampleDistance) const {
    // Basic geometry attributes    
    double xStart = geom.x().get();
    double yStart = geom.y().get();
    double hdg = geom.hdg().get();
    double length = geom.length().get();

    // Poly3 attributes
    const geometry::poly3_optional poly3 = geom.poly3();
    double a = poly3->a().get();
    double b = poly3->b().get();
    double c = poly3->c().get();
    double d = poly3->d().get();

    std::auto_ptr<OGRLineString> lineString(new OGRLineString());
    samplePoly3(length, a, b, c, d, sampleDistance, lineString.get());

    Matrix2D m;
    MatrixTransformations::initMatrix(m);
    // T(geomStartPt) x R(geomHdg) x geometry
    MatrixTransformations::translate(m, xStart, yStart);
    MatrixTransformations::rotate(m, hdg);
    transformLineString(lineString.get(), m);

    return lineString;
}

std::auto_ptr<OGRLineString> XODR::paramPoly3ToLinestring(const geometry &geom, const double sampleDistance) const {
	
	// Basic geometry attributes    
    double xStart = geom.x().get();
    double yStart = geom.y().get();
    double hdg = geom.hdg().get();
    double length = geom.length().get();

    // ParamPoly3 attributes
    const geometry::paramPoly3_optional paramPoly3 = geom.paramPoly3();
    double uA = paramPoly3->aU().get();
    double uB = paramPoly3->bU().get();
    double uC = paramPoly3->cU().get();
    double uD = paramPoly3->dU().get();
    double vA = paramPoly3->aV().get();
    double vB = paramPoly3->bV().get();
    double vC = paramPoly3->cV().get();
    double vD = paramPoly3->dV().get();
	std::auto_ptr<OGRLineString> lineString(new OGRLineString());
	/*std::auto_ptr<OGRLineString> lineString(new OGRLineString());
	//We check whether the paramPoly3 object has 
	if (paramPoly3->pRange().present()) {
		pRange parameterRange = paramPoly3->pRange().get();		
		switch (parameterRange) {
		case pRange::value::arcLength:
			sampleParamPoly3(length, uA, uB, uC, uD, vA, vB, vC, vD, sampleDistance, lineString.get());
			break;
		case pRange::value::normalized:
			sampleParamPoly3(1.0, uA, uB, uC, uD, vA, vB, vC, vD, sampleDistance / length, lineString.get());
			break;
		}
	}
	else {
		sampleParamPoly3(1.0, uA, uB, uC, uD, vA, vB, vC, vD, sampleDistance / length, lineString.get());
	}*/	
	if (paramPoly3->pRange().present()) {
		sampleParamPoly3(length, uA, uB, uC, uD, vA, vB, vC, vD, sampleDistance, lineString.get());
	}
	else {
		sampleParamPoly3(1.0, uA, uB, uC, uD, vA, vB, vC, vD, sampleDistance / length, lineString.get());
	}

	
	//sampleParamPoly3(1.0, uA, uB, uC, uD, vA, vB, vC, vD, sampleDistance / length, lineString.get());
    Matrix2D m;
    MatrixTransformations::initMatrix(m);
    // T(geomStartPt) x R(geomHdg) x geometry
    MatrixTransformations::translate(m, xStart, yStart);
    MatrixTransformations::rotate(m, hdg);
    transformLineString(lineString.get(), m);

    return lineString;
}

OGRLineString* XODR::sampleArc(const double length, const double curvature,
        const double angleStepSizeDegrees) const {

    // *********     Step 1 : Arc definition     ************
    // To find the points that define the arc (Xo,Yo), (Xm,Ym) and (Xe,Ye)
    // General formula for finding points
    // for polar coordinates with center (0,0)

    double r = 1 / abs(curvature);
    double alpha = length / r;

    double xStart, yStart;
    double xMiddle, yMiddle;
    double xEnd, yEnd;

    //   For positive curvature, the points are given by
    //   x = r*sin  and  y = r*-cos 
    if (curvature > 0) {

        // Initial point: alpha = 0
        xStart = r * sin(0.0);
        yStart = r * (-cos(0.0));

        // Middle point: angle = alpha/2
        xMiddle = r * sin(alpha / 2);
        yMiddle = r * (-cos(alpha / 2));

        // End point: angle = alpha
        xEnd = r * sin(alpha);
        yEnd = r * (-cos(alpha));

    } else {
        //  [!] For negative curvature, the points are given by
        //   x = r*sin  and  y = r*cos 

        // Initial point: alpha = 0
        xStart = r * sin(0.0);
        yStart = r * (cos(0.0));

        // Middle point: angle = alpha/2
        xMiddle = r * sin(alpha / 2);
        yMiddle = r * (cos(alpha / 2));

        // End point: angle = alpha
        xEnd = r * sin(alpha);
        yEnd = r * (cos(alpha));
    }

    OGRLineString* arc = OGRGeometryFactory::curveToLineString(xStart, yStart, 0, xMiddle, yMiddle, 0, xEnd, yEnd, 0,
            false, angleStepSizeDegrees, NULL);

    // Translate by radius to obtain arc's start point at the origin (0, 0)
    // for later following transformation into road coordinate system
    double yt = curvature > 0 ? r : -r;
    Matrix2D m;
    MatrixTransformations::initMatrix(m);
    MatrixTransformations::translate(m, 0, yt);
    transformLineString(arc, m);
    return arc;
}

double XODR::sampleDefaultSpiral(const double length, const double endCurvature, const double sampleDistance,
        OGRLineString* lineString) const {
    double curvatureChange = endCurvature / length;
    double x;
    double y;
    double t;

    // First and all intermediate points
    for (double s = 0.0; s < length; s += sampleDistance) {
        odrSpiral(s, curvatureChange, &x, &y, &t);
        OGRPoint ptIntermediate(x, y);
        lineString->addPoint(&ptIntermediate);
    }

    // End point
    odrSpiral(length, curvatureChange, &x, &y, &t);
    OGRPoint ptEnd(x, y);
    lineString->addPoint(&ptEnd);
    return t;
}

void XODR::samplePoly3(const double length, const double a, const double b, const double c, const double d,
        const double sampleDistance, OGRLineString* lineString) const {
    double x;
    double y;
    CubicPolynomial poly3(a, b, c, d);

    // First and all intermediate points
    for (double s = 0.0; s < length; s += sampleDistance) {
        x = s;
        y = poly3.value(x);
        OGRPoint ptIntermediate(x, y);
        lineString->addPoint(&ptIntermediate);
    }

    // End point
    x = length;
    y = poly3.value(x);
    OGRPoint ptEnd(x, y);
    lineString->addPoint(&ptEnd);
}

void XODR::samplePoly3(double init, double end, double a, double b, double c, double d, double sampleDistance, OGRLineString* lineString) const {
	double x;
	double y;
	CubicPolynomial poly3(a, b, c, d);
	double length = end - init;
	// First and all intermediate points
	for (double s = 0.0; s < length; s += sampleDistance) {
		x = s;
		y = poly3.value(x);
		OGRPoint ptIntermediate(x + init, y);
		lineString->addPoint(&ptIntermediate);
	}

	// End point
	x = length;
	y = poly3.value(x);
	OGRPoint ptEnd(x + init, y);
	lineString->addPoint(&ptEnd);
}


void XODR::sampleParamPoly3(const double range, const double uA, const double uB, const double uC, const double uD,
        const double vA, const double vB, const double vC, const double vD,
        const double stepSize, OGRLineString* lineString) const {
    double x;
    double y;
    ParametricCubicPolynomial pramPoly3(uA, uB, uC, uD, vA, vB, vC, vD);
    // First and all intermediate points
    for (double p = 0.0; p < range; p += stepSize) {
        x = pramPoly3.valueU(p);
        y = pramPoly3.valueV(p);
        OGRPoint ptIntermediate(x, y);
        lineString->addPoint(&ptIntermediate);
    }

    // End point
    x = pramPoly3.valueU(range);
    y = pramPoly3.valueV(range);
    OGRPoint ptEnd(x, y);
    lineString->addPoint(&ptEnd);
}

int XODR::getNumberOfRoads() {
    return op->road().size();
}

void XODR::transformLineString(OGRLineString* geom, const Matrix2D& matrix) const {
    int numPts = geom->getNumPoints();
    for (int p = 0; p < numPts; ++p) {
        Vector2D pt;
        pt.x = geom->getX(p);
        pt.y = geom->getY(p);
        pt.w = 1.0;
        MatrixTransformations::transform(pt, matrix);
        geom->setPoint(p, pt.x, pt.y);
    }
}


///////////////////////////////
//Helper convenient functions//

std::vector<double> XODR::samplingRange(double init, double end, double increment)const {
	//Returns an equally spaced vector with values between init and end	
	std::vector<double> range;
	for (double s = init; s < end - 0.3; s += increment) {

		range.push_back(s);
	}
	range.push_back(end);
	return range;
}

void XODR::printLine(OGRLineString line)const {
	//Prints line coordinates
	int nPoints = line.getNumPoints();
	OGRPoint p;
	for (int i = 0; i < nPoints; i++)
	{
		line.getPoint(i, &p);
		std::cout << "x " << p.getX() << ", y " << p.getY() << "\n";

	}
}


void XODR::offsetLine(OGRLineString* lineSegment, double tOffset) const {
	//Translates a line with the value of tOffset, following a normal vector
	//In other words projects a line "tOffset" units
	OGRPoint startingPoint;
	OGRPoint normalVector;
	OGRLineString line = *lineSegment;
	lineSegment->StartPoint(&startingPoint);
	normalVector = normalUnitaryVector(line);
	OGRPoint tOffsetPoint = normalProjectedPoint(startingPoint, normalVector, tOffset);
	translate(lineSegment, (tOffsetPoint.getX() - startingPoint.getX())*-1, (tOffsetPoint.getY() - startingPoint.getY())*-1);

}

void XODR::translate(OGRLineString* lineString, double x, double y) const {
	Matrix2D m;
	MatrixTransformations::initMatrix(m);
	MatrixTransformations::translate(m, x, y);
	transformLineString(lineString, m);
}



OGRPoint XODR::normalUnitaryVector(OGRLineString line) const {
	// Returns an unitary point that forms a  normal angle with respect to the provided line
	OGRPoint startingPoint;
	OGRPoint endingPoint;

	double vx;
	double vy;
	double ux;
	double uy;
	double distance = line.get_Length();

	line.StartPoint(&startingPoint);
	line.EndPoint(&endingPoint);

	vx = endingPoint.getX() - startingPoint.getX();
	vy = endingPoint.getY() - startingPoint.getY();

	ux = -vy / distance;
	uy = vx / distance;
	OGRPoint normalUnitaryVector(ux, uy);
	return normalUnitaryVector;
}



OGRPoint XODR::normalProjectedPoint(OGRPoint roadPoint, OGRPoint normalUnitaryVector, double borderWidth) const {
	//Returns a point that forms a normal line with respect to the coordinates of the parameter roadPoint and is within the distance given by the parameter borderWidth
	double x;
	double y;
	x = roadPoint.getX() + borderWidth * normalUnitaryVector.getX();
	y = roadPoint.getY() + borderWidth * normalUnitaryVector.getY();
	OGRPoint bPoint(x, y);
	return bPoint;
}

OGRMultiLineString XODR::splitLineWithSpace( OGRLineString *lineSegment, double length, double space) const {

	//This function takes a continous line (lineSegment parameter), split it accoring to a predefined length (length parameter), leaving  spaces between each segment (space parameter). 
	//In short it stores non continous segments inside a multilineString (lineSegmentM), creating the effect of a stripped line	
	OGRMultiLineString  lineSegmentM;	
	double lineSegmentLength = lineSegment->get_Length();
	OGRLineString* l;
	double from;
	double to;
	double increment = length + space;
	if (increment > 0 && increment <lineSegmentLength) {
		for (double movingL = 0; movingL <= lineSegmentLength; movingL += increment) {
			from = movingL;
			if (movingL + length > lineSegmentLength) {
				to = lineSegmentLength;
			}
			else {
				to = movingL + length;
			}
			if (from < to) {
				l = lineSegment->getSubLine(from, to, 0);
				lineSegmentM.addGeometry(l);
			}
		}		
	}
	else {
		lineSegmentM.addGeometry(lineSegment);
	}
	return lineSegmentM;
	
}
OGRLineString XODR::multiLineStringToLineString(OGRMultiLineString* multiple) const {
	// Transfomrs a OGRMultiLineString into a OGRLineString. The porpuse is to use the methods that are only available inside the OGRLineString class.
	int numGeoms = multiple->getNumGeometries();
	OGRLineString l;
	for (int i = 0; i < numGeoms; i++)
	{
		OGRLineString* current = static_cast<OGRLineString*> (multiple->getGeometryRef(i));
		l.addSubLineString(current, 0, -1);
	}
	return l;
}

OGRPoint XODR::valueInLine(OGRLineString line, double s)const {
	// Examinates the line and returns a point whose x coordinate matches the provided s value, in case of not finding such value it returns a point with coordinates -1,-1
	OGRPoint p;
	int nPoints = line.getNumPoints();
	int index;
	bool found = false;
	for (int i = 0; i < nPoints; i++)
	{
		line.getPoint(i, &p);
		if (abs(p.getX() - s)< 0.05) {
			found = true;
			index = i;
			return p;
		}
	}
	return OGRPoint(-1, -1);
}
//************REFERENCE LINE***************//


std::auto_ptr<OGRLineString> XODR::toOGRGeometry(const geometry& xodrGeometry) const {
	std::auto_ptr<OGRLineString> convertedGeom(new OGRLineString());
	if (xodrGeometry.line().present()) {
		convertedGeom = lineToLinestring(xodrGeometry);
	}
	else if (xodrGeometry.arc().present()) {
		convertedGeom = arcToLinestring(xodrGeometry, 0); // default angle step size
	}
	else if (xodrGeometry.spiral().present()) {
		convertedGeom = spiralToLinestring(xodrGeometry, 0.5);
	}
	else if (xodrGeometry.poly3().present()) {
		convertedGeom = poly3ToLinestring(xodrGeometry, 0.5);
	}
	else if (xodrGeometry.paramPoly3().present()) {	
		convertedGeom = paramPoly3ToLinestring(xodrGeometry, 0.5);
	}
	return convertedGeom;
}



//****************LANES********************//
//******Methods used to generate lanes*****//

std::vector<Lane> XODR::roadLanes(const road& xodrRoad)const {
	//This functions retrieves the lanes of the given xodrRoad parameter		
	
	std::vector<Lane> roadLanes;
	const lanes& lanes = xodrRoad.lanes();
	// Lanes can be described either with  "border"  or  "width" elements
	// In case that both elements exist only the latter will prevail (Please refer to opendrive 1.4 specifications)
	if (rightWidthPresent(lanes) || leftWidthPresent(lanes)) {
		roadLanes = roadLanesW(xodrRoad);
	}
	else if (rightBorderPresent(lanes) || leftBorderPresent(lanes)) {
		roadLanes = roadLanesB(xodrRoad);
	}
	return roadLanes;
}

// When lanes are defined by "WIDTH" elements we use the following algorithms :

std::vector<Lane> XODR::roadLanesW(const road& xodrRoad)const {
	//Retrieves a vector containing the lanes allocated along the given xodrRoad parameter
	// This function handles the case where lanesheights  are defined with "width" elements	
	
	const lanes& lanes = xodrRoad.lanes();	
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	const planView& planView = xodrRoad.planView();
	OGRMultiLineString  referenceLine = toOGRGeometry(planView);
	int nLaneSections = laneSections.size();
	std::vector<Lane> lanesCompilation;
	OGRLineString  referenceLineString = multiLineStringToLineString(&referenceLine);
	double roadLength = referenceLineString.get_Length();

	// Lane offsets are a special Lane case, so they are handled individually
	const lanes::laneOffset_sequence& laneOffsetSeq = lanes.laneOffset();
	Lane laneOffseElement = laneOffset(laneOffsetSeq, referenceLineString, xodrRoad.id().get());
	
	if (laneOffseElement.line.getNumPoints() > 0) {
		lanesCompilation.push_back(laneOffseElement);
	}

	for (int i = 0; i < nLaneSections; i++)
	{
		const laneSection& lSection = laneSections[i];
		if (lSection.left().present()) {
			std::vector<Lane> lWidthL = lanesWidthLeft(xodrRoad, i, roadLength);
			
			int numGeomsL = lWidthL.size();
			for (int i = 0; i < numGeomsL; i++)
			{
				Lane laneElement = lWidthL.at(i);
				lanesCompilation.push_back(laneElement);
			}
		}
		if (lSection.right().present()) {
			//Computing the lane section located inside the "i" index
			std::vector<Lane>  lWidthR = lanesWidthRight(xodrRoad, i, roadLength);
			int numGeomsL = lWidthR.size();
			for (int i = 0; i < numGeomsL; i++)
			{
				Lane laneElement = lWidthR.at(i);
				lanesCompilation.push_back(laneElement);
			}
		}
	}	
	return lanesCompilation;
}

Lane XODR::laneOffset(const lanes::laneOffset_sequence& laneOffsetSeq, OGRLineString  referenceLine, std::string roadId)const {
	// Returns a laneOfsset element. This object represent shifts of the reference line
	double end;
	double init;
	
	OGRLineString projectedLine;
	OGRLineString laneOffsetLine;
	std::vector<OGRLineString> linesVector;
	int nLaneOffset = laneOffsetSeq.size();
	for (int i = 0; i < nLaneOffset; i++) {
		OGRLineString line;
		init = laneOffsetSeq[i].s().get();
		double a = laneOffsetSeq[i].a().get();
		double b = laneOffsetSeq[i].b().get();
		double c = laneOffsetSeq[i].c().get();
		double d = laneOffsetSeq[i].d().get();
		if (i < nLaneOffset - 1) {
			end = laneOffsetSeq[i + 1].s().get();
		}
		else {
			end =  referenceLine.get_Length();
		}
		//Computing line
		samplePoly3(init, end, a, b, c, d, 0.3, &line);
		//Projecting line according to the reference line
		projectedLine = projectLanesB(referenceLine, line);
		linesVector.push_back(projectedLine);
	}
	// All the line segments stored inside the linesVector list are stored ina single line
	for (int i = 0; i < linesVector.size(); i++)
	{
		OGRLineString temp = linesVector.at(i);
		int nPoints = temp.getNumPoints();
		OGRPoint p;
		for (int j = 0; j < nPoints; j++) {
			temp.getPoint(j, &p);
			laneOffsetLine.addPoint(&p);
		}
	}
	Lane lOffsetElement = Lane(0, "LaneOffset", "", laneOffsetLine);
	lOffsetElement.parentRoadId = roadId;
	return lOffsetElement;
}


std::vector<Lane> XODR::lanesWidthRight(const road& xodrRoad, int laneSectionIndex, double roadLength)const {
	//Returns a vector containing all the lanes widths inside the selected lane section (according to the laneSectionIndex)		
	
	std::vector<Lane> lanesRight;
	std::string singleSide ="";
	OGRMultiLineString multiLineRight;
	const lanes& lanes = xodrRoad.lanes();	
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	double sLaneSectionI;
	double sLaneSectionE;
	const laneSection& lSection = laneSections[laneSectionIndex];
	sLaneSectionI = lSection.s().get();	
	//If there is a lane section succesor available we take their sOffset as the ending range value (sLaneSectionE) of the current lane section
	//If that is not the case we simply take the road's length
	if (laneSectionIndex < nLaneSections - 1) {
		sLaneSectionE = laneSections[laneSectionIndex + 1].s().get();
	}
	else {
		sLaneSectionE = roadLength;
	}	
	//Collecting lanes allocated along the right side of the road
	if (lSection.singleSide().present()) {
		singleSide = lSection.singleSide().get();
	}
	const laneSection::right_optional& rightLaneSection = lSection.right();
	const right::lane_sequence& lanesR = rightLaneSection.get().lane();
	lanesRight = lanesWidthRight(xodrRoad, lanesR, sLaneSectionI, sLaneSectionE, singleSide);
	
	return lanesRight;
}


std::vector<Lane>  XODR::lanesWidthLeft(const road& xodrRoad, int laneSectionIndex, double roadLength)const {
	//Returns a multilinestring containing all the lanes widths inside the selected lane section (according to the laneSectionIndex)

	std::vector<Lane> lanesLeft;
	std::string singleSide="";
	OGRMultiLineString multiLineLeft;	
	const lanes& lanes = xodrRoad.lanes();	
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	double sLaneSectionI;
	double sLaneSectionE;
	const laneSection& lSection = laneSections[laneSectionIndex];
	sLaneSectionI = lSection.s().get();

	//If there is a lane section succesor available we take their sOffset as the ending range value (sLaneSectionE) of the current lane section
	//If that is not the case we simply take the road's length
	if (laneSectionIndex < nLaneSections - 1) {
		sLaneSectionE = laneSections[laneSectionIndex + 1].s().get();
	}
	else {
		sLaneSectionE = roadLength;
	}
	
	//Collecting lanes allocated along the left side of the road
	if (lSection.singleSide().present()) {
		singleSide = lSection.singleSide().get();
	}
	const laneSection::left_optional& leftLaneSection = lSection.left();
	const left::lane_sequence& lanesL = leftLaneSection.get().lane();
	lanesLeft = lanesWidthLeft(xodrRoad, lanesL, sLaneSectionI, sLaneSectionE, singleSide);
	return lanesLeft;
}

std::vector<Lane> XODR::lanesWidthRight(const road& xodrRoad, const right::lane_sequence& lanes, double laneSectionI, double laneSectionE, std::string singleSide)const {
	// Returns a vector containing the Right lanes according to the given parameters

	std::vector<Lane> lanesRight;
	int nLanes = lanes.size();
	std::vector<RoadMark> roadMarksCollection;
	OGRLineString wLineSegment;
	std::vector<laneWidth> widthElements;
	std::string roadId = xodrRoad.id().get();
	std::string roadName = xodrRoad.name().get();
	const planView& planView = xodrRoad.planView();
	OGRMultiLineString  ogrRoad = toOGRGeometry(planView);

	//All the lanes must share the same sampling space this is crucial for our algorithm. Here the sample space is called range
	std::vector<double> range = samplingRange(laneSectionI, laneSectionE,0.3);

	//This for loop goes through each lane, computes the lanes width and creates an object that stores their parameters
	for (int i = 0; i < nLanes; i++)
	{
		const lane& lane = lanes[i];
		int id = lane.id().get();
		std::string type = lane.type().get();
		std::string level = lane.level().get();
		const lane::width_sequence& widthSequence = lane.width();
		widthElements = createWidthList(laneSectionI, laneSectionE, widthSequence);
		wLineSegment = widthLine(widthElements, range,'R');
		//If needed we adjust the generated line so that it contains all the elements inside the sampling range, this constrained is crucial for the projection algorithm
		wLineSegment = adjustLine(range, wLineSegment);
		Lane laneElement = Lane(id, type, level, wLineSegment);
		laneElement.parentRoadId = roadId;
		laneElement.parentRoadName = roadName;
		laneElement.singleSide = singleSide;			
		lanesRight.push_back(laneElement);		
	}

	// We take the lanes width lines and project them according to the main reference line to obtain the lane
	std::vector<OGRLineString> projectedLines = projectLanesW(lanesRight, ogrRoad);
	
	// We replace the old lanes width lines, with the actual projected lane lines
	for (int i = 0; i < nLanes; i++)
	{
		lanesRight.at(i).line = projectedLines.at(i);
	}
	//Once we have collected the projected lanes, we can compute the road marks.
	for (int i = 0; i < nLanes; i++)
	{

		const lane& lane = lanes[i];
		const lane::roadMark_sequence rMarkSequence = lane.roadMark();
		roadMarksCollection = roadMarks(rMarkSequence, laneSectionI, laneSectionE, lanesRight.at(i));
		// Road marks are considered a lane attributes
		lanesRight.at(i).roadMarksCollection = roadMarksCollection;
		
	}
	return lanesRight;
}

std::vector<Lane> XODR::lanesWidthLeft(const road& xodrRoad, const left::lane_sequence& lanes, double laneSectionI, double laneSectionE, std::string singleSide)const {
	// Returns a vector containing the Left lanes according to the given parameters

	std::vector<RoadMark> roadMarksCollection;
	std::vector<Lane> lanesLeft;
	int nLanes = lanes.size();
	OGRMultiLineString multiLine;
	OGRLineString wLineSegment;
	std::vector<laneWidth> widthElements;
	std::string roadId =  xodrRoad.id().get();
	std::string roadName = xodrRoad.name().get();

	const planView& planView = xodrRoad.planView();
	OGRMultiLineString  ogrRoad = toOGRGeometry(planView);
	//All the lanes must share the same sampling space this is crucial for our algorithm. Here the sample space is called range
	std::vector<double> range = samplingRange(laneSectionI, laneSectionE, 0.3);

	for (int i = 0; i < nLanes; i++)
	{
		const lane& lane = lanes[i];		
		int id = lane.id().get();
		std::string type = lane.type().get();
		std::string level = lane.level().get();		
		const lane::width_sequence& widthSequence = lane.width();
		widthElements = createWidthList(laneSectionI, laneSectionE, widthSequence);	
		wLineSegment = widthLine(widthElements, range, 'L');		
		//If needed we adjust the generated line so that it contains all the elements inside the sampling range, this constrained is crucial for the projection algorithm
		wLineSegment = adjustLine(range, wLineSegment);		
		Lane laneElement = Lane(id, type, level, wLineSegment);
		laneElement.parentRoadId = roadId;
		laneElement.parentRoadName = roadName;
		laneElement.singleSide = singleSide;		
		lanesLeft.push_back(laneElement);	
	}
	// We take the lanes width lines and project them according to the main reference line to obtain the lane
	std::vector<OGRLineString> projectedLines = projectLanesW(lanesLeft, ogrRoad);

	// We replace the old lanes width lines, with the actual projected lane lines
	for (int i = 0; i < nLanes; i++)
	{
		lanesLeft.at(i).line = projectedLines.at(i);
	}
	//Once we have collected the projected lanes, we can compute the road marks.
	for (int i = 0; i < nLanes; i++)
	{
		
		const lane& lane = lanes[i];
		const lane::roadMark_sequence rMarkSequence = lane.roadMark();
		roadMarksCollection = roadMarks(rMarkSequence, laneSectionI, laneSectionE, lanesLeft.at(i));
		// Road marks are one of the lane attributes
		lanesLeft.at(i).roadMarksCollection = roadMarksCollection;

	}	
	return lanesLeft;
}

OGRLineString XODR::adjustLine(std::vector<double> range, OGRLineString line) const{

	//Returns a line whose x axis contains all the values inside the range parameter
	//Because lanes are represented by width elements that do not share the same starting and ending point, they often do not have the same number of points.
	//To apply the projection algorithm we must remove this incongruency (all lanes must have the exact number of points), hence we add additional points when needed

	int nRange = range.size();
	OGRLineString adjustedLine;
	OGRPoint previousPoint;
	line.getPoint(0, &previousPoint);
	adjustedLine.addPoint(&previousPoint);

	for (int i = 1; i < nRange -1; i++)
	{
		double s = range.at(i);
		OGRPoint p = valueInLine(line, s);
		
		//In case of not finding the value we added it manually
		if (p.getX() == -1) {
			p.setX(s);
			p.setY(previousPoint.getY());
			adjustedLine.addPoint(&p);
			previousPoint = p;
		}
		else {
			adjustedLine.addPoint(&p);
			previousPoint = p;
		}
	}

	line.getPoint(line.getNumPoints()-1, &previousPoint);
	adjustedLine.addPoint(&previousPoint);

	return adjustedLine;
}


std::vector<OGRLineString> XODR::projectLanesW(std::vector<Lane> lanesRight, OGRMultiLineString referenceLine) const{
	//Takes as input a vector containing lane elements, extracts their line attributes which describe the lane's width and project them according to the reference line to generate a line that depicts projected lanes

	int numGeoms = lanesRight.size();
	OGRLineString  referenceLineString = multiLineStringToLineString(&referenceLine);
	// We start by defining a vector with empty lineString whose size is equivalent to the lanes size
	// This vector will store the projected lanes
	std::vector<OGRLineString> projectedLineList;
	for (int i = 0; i < numGeoms; i++)
	{
		projectedLineList.push_back(OGRLineString());
	}	
	// Due to the adjust line algorithm all the lanes share the same number of points, hence we take the first one to get the number of points which will be used later
	OGRLineString laneElement = lanesRight.at(0).line;
	OGRPoint referenceLinePoint;
	OGRPoint normalPoint;
	double acumulatedWidth;
	int nLP = laneElement.getNumPoints();
	OGRLineString* segmentReferenceLine;
	OGRPoint lanePoint;
	OGRPoint lanePointSuccesor;

	// Here we select each point located at index i, inside the lanesWidth lines and project them according to the reference line
	for (double i = 0; i < nLP; i++) {
		acumulatedWidth = 0;
		//Looping through each of the lines inside the lanesWidth parameter	
		for (int g = 0; g < numGeoms; g++) {

			OGRLineString laneElement = lanesRight.at(g).line;
			laneElement.getPoint(i, &lanePoint);
			//To project a lane we take into account the width of previous lanes, this is depicted in the variable acumulated width
			//Before projecting we check whether the x coordinate of the lane is less or equal to the total reference line length, otherwise we get an error
			if (lanePoint.getX() <= referenceLineString.get_Length()) {
		
				acumulatedWidth += lanePoint.getY();
				referenceLineString.Value(lanePoint.getX(), &referenceLinePoint);
				if (i < nLP - 1) {		

					if (lanePoint.getX() + 0.3 <= referenceLineString.get_Length() && lanePoint.getX() < referenceLineString.get_Length()) {
						segmentReferenceLine = referenceLineString.getSubLine(lanePoint.getX(), lanePoint.getX() + 0.3, 0);
					}
					else {
						segmentReferenceLine = referenceLineString.getSubLine(lanePoint.getX() -0.3, lanePoint.getX(), 0);
					}
				}
				
				else {
					laneElement.getPoint(i - 1, &lanePointSuccesor);
					segmentReferenceLine = referenceLineString.getSubLine(lanePoint.getX() - 0.3, lanePoint.getX(), 0);
				}

				normalPoint = normalUnitaryVector(*segmentReferenceLine);
				OGRPoint projectedPoint = normalProjectedPoint(referenceLinePoint, normalPoint, acumulatedWidth);
				projectedLineList.at(g).addPoint(&projectedPoint);
			}
		}
	}
	//Finally we replace the old line attribute of each lane, and replace it with the projected line, which describes the lane geometry
	for (int i = 0; i < numGeoms; i++)
	{
		lanesRight.at(i).line = projectedLineList.at(i);
	}
	return projectedLineList;
}

OGRLineString XODR::widthLine(std::vector<laneWidth> wList, std::vector<double> fixedSamplingRange, char direction)const {
	// This function takes as input each of the laneWidth elements inside the wList vector and computes a lineString which describes the lane's width

	int nW = wList.size();
	OGRLineString line;
	std::vector<double> newRange;
	int rangeSize = fixedSamplingRange.size();
	bool valuesInside;

	//Before generating the lineString we check that the laneWidth elements contain the values inside the fixedSamplingRange attributes, if that is not the case, we add it manually. This is important, in future stages we must
	//ensure that all the lanes inside a lane section have the same number of points because their rely on each others.
	for (int i = 0; i < nW; i++)
	{
		valuesInside = false;
		std::vector<double> newRange;
		laneWidth *w = &wList.at(i);
		for (int j = 0; j < rangeSize; j++)
		{
			double s = fixedSamplingRange.at(j);

			if (s >= w->init && s <= w->end) {
				valuesInside = true;
				newRange.push_back(s);
			}
		}
		if (valuesInside) {
			w->init = newRange.at(0);
			w->end = newRange.at(newRange.size() - 1);
		}
	}
	// After adjusting the lane according to the fixedSamplingRange parameter we generate the lineStrings
	for (int i = 0; i < nW; i++)
	{
		OGRLineString temp;
		laneWidth w = wList.at(i);
		samplePoly3Width(w, &temp, direction);
		line.addSubLineString(&temp, 0, -1);
	}
	return line;
}

std::vector<laneWidth> XODR::createWidthList(double sLaneSectionI, double sLaneSectionE, const lane::width_sequence& widthSequence)const {
	//Extracts relevant information from the widthSequence parameter and creates a list of laneWidth objects.

	double sE;
	double sI;
	std::vector<laneWidth> widthElements;
	int nW = widthSequence.size();
	for (int i = 0; i < nW; i++)
	{
		const width& w = widthSequence[i];
		sI = sLaneSectionI + w.sOffset().get();
		//If there are succesor width elements available we take their sOffset as the maximum posible value of the polinomyal described by the current width
		// If there are not succesor width element available we take the sLaneSectionE parameter
		if (i < nW - 1) {
			const width& wS = widthSequence[i + 1];
			sE = wS.sOffset().get() + sLaneSectionI;

		}
		else {
			sE = sLaneSectionE;
		}
		laneWidth lW = laneWidth(sI, sE, w.a().get(), w.b().get(), w.c().get(), w.d().get());
		widthElements.push_back(lW);

	}
	return widthElements;
}

void XODR::samplePoly3Width(laneWidth w, OGRLineString* lineString, char direction) const {
	//Take as imput an object of type lanewidth, and samples a line using a polinomyal function, according to their parameters

	double a = w.a;
	double b = w.b;
	double c = w.c;
	double d = w.d;
	double end = w.end;
	double init = w.init;
	double x;
	double y;
	CubicPolynomial poly3(a, b, c, d);
	double length = end - init;
	double constant;

	//According to the documentation right borders generate -t values (depict internal lanes)
	if (direction == 'R') {
		constant = -1;
	}
	else {
		constant = 1;
	}

	// First and all intermediate points
	for (double s = 0; s < length; s += 0.3) {
		y = poly3.value(s);
		OGRPoint ptIntermediate(s + init, y*constant);
		lineString->addPoint(&ptIntermediate);
	}
	// End point

	y = poly3.value(length);
	OGRPoint ptEnd(end, y*constant);
	lineString->addPoint(&ptEnd);
}


bool XODR::leftWidthPresent(const lanes& lanes)const {
	//Checks whether there are left width elements available inside the given lanes
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	bool widthPresent = false;
	for (int i = 0; i < nLaneSections; i++)
	{
		const laneSection& ls = laneSections[i];
		if (ls.left().present()) {
			const laneSection::left_optional& xodrLeft = ls.left();
			const left::lane_sequence& xodrLaneS = xodrLeft.get().lane();
			int nLanes = xodrLaneS.size();
			for (int j = 0; j < nLanes; j++) {
				const lane& xodrLane = xodrLaneS[j];
				if (xodrLane.width().size()>0) {
					widthPresent = true;
				}
			}			
		}
	}
	return widthPresent;
}

bool XODR::rightWidthPresent(const lanes& lanes)const {
	//Checks whether there are left width elements available inside the given lanes
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	bool widthPresent = false;

	for (int i = 0; i < nLaneSections; i++)
	{
		const laneSection& ls = laneSections[i];
		if (ls.right().present()) {
			const laneSection::right_optional& xodrRight = ls.right();
			const right::lane_sequence& xodrLaneS = xodrRight.get().lane();
			int nLanes = xodrLaneS.size();
			for (int j = 0; j < nLanes; j++) {
				const lane& xodrLane = xodrLaneS[j];
				if (xodrLane.width().size()>0) {
					widthPresent = true;
				}
			}
		}
	}
	return widthPresent;
}

// When lanes are defined by "BORDER" elements we use the following algorithms :

std::vector<Lane>  XODR::roadLanesB(const road& xodrRoad) const {
	// Retrieves a multilinestring containing the lanes allocated along the given xodrRoad parameter
	// This function handles the case where lane's heights are defined with "BORDER" elements
	std::vector<Lane> lanesCompilation;
	std::vector<Lane> lanesL;
	std::vector<Lane> lanesR;
	const planView& planView = xodrRoad.planView();
	OGRMultiLineString  ogrRoad = toOGRGeometry(planView);
	OGRLineString  referenceLine = multiLineStringToLineString(&ogrRoad);
	double roadLength = referenceLine.get_Length();
	const lanes& lanes = xodrRoad.lanes();
	
	//In first place I make sure that there are borders otherwise I will get an error
	bool bLeftPresent = leftBorderPresent(lanes);
	bool bRightPresent = rightBorderPresent(lanes);
	
	
	if (bLeftPresent) {
		
		lanesL = borderLeft(xodrRoad, referenceLine);
		int numGeomsL = lanesL.size();
		for (int i = 0; i < numGeomsL; i++)
		{
			Lane laneElement = lanesL.at(i);
			lanesCompilation.push_back(laneElement);
		}
	}

	if (bRightPresent) {
	
		lanesR = borderRight(xodrRoad, referenceLine);
		int numGeomsL = lanesR.size();
		for (int i = 0; i < numGeomsL; i++)
		{
			Lane laneElement = lanesR.at(i);
			lanesCompilation.push_back(laneElement);
		}
	}
	return lanesCompilation;
}

std::vector<Lane> XODR::borderLeft(const road& xodrRoad, OGRLineString  referenceLine)const {
	// Returns a vector containing the left border elements along the xodrRoad parameter. 
	OGRLineString laneLineString;
	OGRLineString border;
	double sLaneSectionI;
	double sLaneSectionE;
	const lanes& lanes = xodrRoad.lanes();
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	double roadLength = referenceLine.get_Length();
	std::vector<Border> borderElements;

	// We extract information related to the parent road to know where does the border element come from
	std::string roadId = xodrRoad.id().get();
	std::string roadName = xodrRoad.name().get();
	std::string singleSide = "";
	std::vector<Lane> lanesCollection;
	std::vector<RoadMark> roadMarksCollection;

	for (int i = 0; i < nLaneSections; i++)
	{
		const laneSection& ls = laneSections[i];
		if (ls.singleSide().present()) {
			singleSide = ls.singleSide().get();
		}
		sLaneSectionI = ls.s().get();
		//If there is a lane section successor available we take their sOffset as the ending range value (sLaneSectionE) of the current lane section
		//If that is not the case we simply take the road's length
		if (i < nLaneSections - 1) {
			sLaneSectionE = laneSections[i + 1].s().get();
		}
		else {
			sLaneSectionE = roadLength;
		}
		if (ls.left().present()) {
			const laneSection::left_optional& xodrLeft = ls.left();
			const left::lane_sequence& xodrLaneS = xodrLeft.get().lane();
			int nLanes = xodrLaneS.size();
			for (int j = 0; j < nLanes; j++) {
				const lane& XODRLane = xodrLaneS[j];
				const lane::border_sequence& borderSequence = XODRLane.border();
				//We capture all the border elements inside a vector
				borderElements = createBorderList(sLaneSectionI, sLaneSectionE, borderSequence);
				// We take each of the elements contained within this vector and cumpute the border
				border = borderLine(borderElements, 'L');
				// We project the border according to the reference line, as a result we obtain the lane			
				laneLineString = projectLanesB(referenceLine, border);
				std::string type = XODRLane.type().get();
				std::string level = XODRLane.level().get();
				int id = XODRLane.id().get();
				// Creating a Lane element which contains not only the lane attributes but also their geometries
				Lane laneElement = Lane(id, type, level, laneLineString);
				laneElement.parentRoadId = roadId;
				laneElement.parentRoadName = roadName;
				laneElement.singleSide = singleSide;
				lanesCollection.push_back(laneElement);

				//RoadMarks
				const lane::roadMark_sequence rMarkSequence = XODRLane.roadMark();
				roadMarksCollection = roadMarks(rMarkSequence, sLaneSectionI, sLaneSectionE, laneElement);
				laneElement.roadMarksCollection = roadMarksCollection;
				lanesCollection.push_back(laneElement);
			}
		}
	}
	return lanesCollection;
}

std::vector<Lane>  XODR::borderRight(const road& xodrRoad, OGRLineString  referenceLine)const {
	// Returns a vector containing the left border elements along the xodrRoad parameter. 
	std::vector<RoadMark> roadMarksCollection;
	OGRLineString laneLineString;
	OGRLineString border;
	double sLaneSectionI;
	double sLaneSectionE;
	const lanes& lanes = xodrRoad.lanes();
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	double roadLength = referenceLine.get_Length();
	std::vector<Border> borderElements;	
	// We extract information related to the parent road to know where does the border element come from
	std::string roadId = xodrRoad.id().get();
	std::string roadName = xodrRoad.name().get();
	std::string singleSide = "";
	std::vector<Lane> lanesCollection;

	for (int i = 0; i < nLaneSections; i++)
	{
		const laneSection& ls = laneSections[i];
		if (ls.singleSide().present()) {
			singleSide = ls.singleSide().get();
		}
		sLaneSectionI = ls.s().get();
		//If there is a lane section successor available we take their sOffset as the ending range value (sLaneSectionE) of the current lane section
		//If that is not the case we simply take the road's length
		if (i < nLaneSections - 1) {
			sLaneSectionE = laneSections[i + 1].s().get();
		}
		else {
			sLaneSectionE = roadLength;
		}
		if (ls.right().present()) {
			const laneSection::right_optional& xodrRight = ls.right();
			const right::lane_sequence& xodrLaneS = xodrRight.get().lane();
			int nLanes = xodrLaneS.size();
			for (int j = 0; j < nLanes; j++) {
				const lane& XODRLane = xodrLaneS[j];
				const lane::border_sequence& borderSequence = XODRLane.border();
				// We capture all the border elements inside a vector
				borderElements = createBorderList(sLaneSectionI, sLaneSectionE, borderSequence);
				// We take each of the elements contained within this vector and cumpute the border
				border = borderLine(borderElements, 'R');
				// We project the border according to the reference line, as a result we obtain the lane					
				laneLineString = projectLanesB(referenceLine, border);
				std::string type = XODRLane.type().get();
				std::string level = XODRLane.level().get();
				int id = XODRLane.id().get();
				// Creating a Lane element which contains not only the lane attributes but also their geometries
				Lane laneElement = Lane(id, type, level, laneLineString);
				laneElement.parentRoadId = roadId;
				laneElement.parentRoadName = roadName;
				laneElement.singleSide = singleSide;			

				//RoadMarks
				const lane::roadMark_sequence rMarkSequence = XODRLane.roadMark();
				roadMarksCollection = roadMarks(rMarkSequence, sLaneSectionI, sLaneSectionE, laneElement);				
				laneElement.roadMarksCollection = roadMarksCollection;
				lanesCollection.push_back(laneElement);
				
			}
		}
	}
	return lanesCollection;
}

OGRLineString XODR::projectLanesB(OGRLineString referenceLine, OGRLineString border) const {
	// Projects the border (which depicts the lane's heights) according to the reference line
	OGRLineString projectedLine;
	OGRLineString* segmentL;
	OGRPoint referenceLinePoint;
	OGRPoint projectedPoint;
	OGRPoint normalPoint;
	OGRPoint borderPoint;
	double lengthLineSegment;
	double lengthReferenceLine;

	int nPS = border.getNumPoints();
	lengthLineSegment = border.get_Length();
	lengthReferenceLine = referenceLine.get_Length();
	for (int i = 0; i < nPS; i++) {
		// We loop through each of the border points, find its corresponding point inside the reference line, this point is projected according to the height of the border point, following a normal orientation
		border.getPoint(i, &borderPoint);
		if (borderPoint.getX() <= lengthReferenceLine) {

			if (i < nPS - 1) {
				segmentL = referenceLine.getSubLine(borderPoint.getX(), borderPoint.getX() + 0.3, 0);
			}
			else {
				segmentL = referenceLine.getSubLine(borderPoint.getX() - 0.3, borderPoint.getX(), 0);
			}

			referenceLine.Value(borderPoint.getX(), &referenceLinePoint);
			normalPoint = normalUnitaryVector(*segmentL);

			projectedPoint = normalProjectedPoint(referenceLinePoint, normalPoint, borderPoint.getY());
			projectedLine.addPoint(&projectedPoint);
		}
	}
	return projectedLine;
}

void XODR::samplePoly3Border(Border bElement, OGRLineString* lineString, char direction) const {
	double a = bElement.a;
	double b = bElement.b;
	double c = bElement.c;
	double d = bElement.d;
	double end = bElement.end;
	double init = bElement.init;
	double x;
	double y;
	CubicPolynomial poly3(a, b, c, d);
	double length = end - init;
	double constant;

	//According to the documentation right borders generate -t values (depict internal lanes)
	if (direction == 'R') {
		constant = -1;
	}
	else {
		constant = 1;
	}

	// First and all intermediate points
	for (double s = 0; s < length - 0.3; s += 0.3) {

		y = poly3.value(s);
		OGRPoint ptIntermediate(s + init, y*constant);
		lineString->addPoint(&ptIntermediate);
	}
	// End point

	y = poly3.value(length);
	OGRPoint ptEnd(length + init, y*constant);
	lineString->addPoint(&ptEnd);
}

bool XODR::rightBorderPresent(const lanes& lanes)const {

	//Checks whether there are right border elements available inside the given lanes
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	bool borderPresent = false;

	for (int i = 0; i < nLaneSections; i++)
	{
		const laneSection& ls = laneSections[i];

		if (ls.right().present()) {
			const laneSection::right_optional& xodrRight = ls.right();
			const right::lane_sequence& xodrLaneS = xodrRight.get().lane();
			int nLanes = xodrLaneS.size();
			for (int j = 0; j < nLanes; j++) {
				const lane& xodrLane = xodrLaneS[0];
				if (xodrLane.border().size()>0) {
					borderPresent = true;
				}
			}
		}
	}
	return borderPresent;
}

bool XODR::leftBorderPresent(const lanes& lanes)const {
	//Checks whether there are left border elements available inside the given lanes
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	bool borderPresent = false;

	for (int i = 0; i < nLaneSections; i++)
	{
		const laneSection& ls = laneSections[i];
		if (ls.left().present()) {
			const laneSection::left_optional& xodrLeft = ls.left();
			const left::lane_sequence& xodrLaneS = xodrLeft.get().lane();
			int nLanes = xodrLaneS.size();
			for (int j = 0; j < nLanes; j++) {
				const lane& xodrLane = xodrLaneS[0];
				if (xodrLane.border().size()>0) {
					borderPresent = true;
				}
			}
		}
	}
	return borderPresent;
}

void XODR::samplePoly3Border(const border& borderSegment, double init, double end, const double sampleDistance, OGRLineString* lineString, char direction) const {
	double a = borderSegment.a().get();
	double b = borderSegment.b().get();
	double c = borderSegment.c().get();
	double d = borderSegment.d().get();
	double x;
	double y;
	CubicPolynomial poly3(a, b, c, d);
	double length = end - init;
	double constant;

	//According to the documentation right borders generate -t values (depict internal lanes)
	if (direction == 'R') {
		constant = -1;
	}
	else {
		constant = 1;
	}

	// First and all intermediate points
	for (double s = 0.0; s < length; s += sampleDistance) {
		x = s;
		y = poly3.value(x);
		OGRPoint ptIntermediate(x + init, y*constant);
		lineString->addPoint(&ptIntermediate);
	}
	// End point
	x = length;
	y = poly3.value(x);
	OGRPoint ptEnd(x + init, y*constant);
	lineString->addPoint(&ptEnd);
}

std::vector<Border> XODR::createBorderList(double sLaneSectionI, double sLaneSectionE, const lane::border_sequence& borderSequence)const {
	// Generates a list containing border instances
	double sE;
	double sI;
	std::vector<Border> borderElements;
	int nB = borderSequence.size();
	for (int i = 0; i < nB; i++)
	{
		const border& b = borderSequence[i];
		sI = sLaneSectionI + b.sOffset().get();
		if (i < nB - 1) {
			const border& bS = borderSequence[i + 1];
			sE = bS.sOffset().get() + sLaneSectionI;

		}
		else {
			sE = sLaneSectionE;
		}
		Border lB = Border(sI, sE, b.a().get(), b.b().get(), b.c().get(), b.d().get());
		borderElements.push_back(lB);
	}
	return borderElements;
}

OGRLineString XODR::borderLine(std::vector<Border> borderList, char direction)const {
	//Retrieves a line by parsing the elements inside the borderList
	int nElements = borderList.size();
	OGRLineString line;
	for (int i = 0; i < nElements; i++)
	{
		OGRLineString temp;
		Border b = borderList.at(i);
		samplePoly3Border(b, &temp, direction);
		line.addSubLineString(&temp, 0, -1);
	}
	return line;
}

//*******************ROAD MARKS************************//

std::vector<RoadMark> XODR::roadMarksLanes(const road& xodrRoad)const {
	//This functions retrieves the lane's road marks
	
	const lanes& lanes = xodrRoad.lanes();	
	std::vector<RoadMark> roadMarksCollection;
	//Road Marks are one of the lane attributes, hence, we compute the lanes and then extract their corresponding road marks
	std::vector<Lane> roadLanesCollection = roadLanes(xodrRoad);	
	int nLanes = roadLanesCollection.size();
	int nRoadMarks;
	int counter = 0;
	for (int i = 0; i < nLanes; i++)
	{
		Lane laneElement = roadLanesCollection.at(i);
		std::vector<RoadMark> roadMarksTemp = laneElement.roadMarksCollection;
		nRoadMarks = roadMarksTemp.size();
	
		for (int j = 0; j < nRoadMarks; j++)
		{
			RoadMark roadMarkTemp1 = roadMarksTemp.at(j);
			roadMarksCollection.push_back(roadMarkTemp1);					
		}
	}
	return roadMarksCollection;
}

std::vector<RoadMark> XODR::roadMarks(const road& xodrRoad)const {
	//Returns the road marks allocated along the xodrRoad parameter (Both lane and reference line road marks)

	std::vector<RoadMark> roadMarksCollection;
	//Collecting road marks along the lanes
	std::vector<RoadMark> rmLanes = roadMarksLanes(xodrRoad);	
	//Collecting road marks along the main reference line, different methods are required because the data types are not the same.
	std::vector<RoadMark> rmReferenceLine = roadMarksReferenceLine(xodrRoad);

	for (int i = 0; i < rmLanes.size(); i++)
	{		
		roadMarksCollection.push_back(rmLanes.at(i));
	}

	for (int j = 0; j < rmReferenceLine.size(); j++)
	{
		roadMarksCollection.push_back(rmReferenceLine.at(j));
	}

	return roadMarksCollection;
}

std::vector<RoadMark> XODR::roadMarksReferenceLine(const road& xodrRoad)const {
	// Retrieves a vector containing the road marks allocated along the reference line
	std::vector<RoadMark> roadMarksCollection;
	const planView& planView = xodrRoad.planView();
	std::string roadId = xodrRoad.id().get();
	OGRMultiLineString  ogrRoad = toOGRGeometry(planView);
	OGRLineString  referenceLine = multiLineStringToLineString(&ogrRoad);

	double roadMarkSI;
	double roadMarkSE;
	double sLaneSectionI;
	double sLaneSectionE;
	
	const lanes& lanes = xodrRoad.lanes();
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	int nLaneSections = laneSections.size();
	double roadLength = referenceLine.get_Length();


	for (int i = 0; i < nLaneSections; i++)
	{
		const laneSection& ls = laneSections[i];
		const center& lsCenter = ls.center();
		sLaneSectionI = ls.s().get();
		//If there is a lane section successor available we take their sOffset as the ending range value (sLaneSectionE) of the current lane section
		//If that is not the case we simply take the road's length
		if (i < nLaneSections - 1) {
			sLaneSectionE = laneSections[i + 1].s().get();
		}
		else {
			sLaneSectionE = roadLength;
		}

		const center::lane_optional  centerLaneOptional = lsCenter.lane();
		centerLane cLane = (centerLane)centerLaneOptional.get();
		const centerLane::roadMark_sequence rMarks = cLane.roadMark();
		if (rMarks.size() > 0) {

			for (int j = 0; j < rMarks.size(); j++)
			{
				const roadMark1& rMark = rMarks[j];
				roadMarkSI = rMark.sOffset().get() + sLaneSectionI;

				if (j < rMarks.size() - 1) {
					roadMarkSE = rMarks[j + 1].sOffset().get() + sLaneSectionI;
				}
				else {
					roadMarkSE = sLaneSectionE;
				}
				RoadMark rElement = roadMarkElement(rMark, roadMarkSI, roadMarkSE, referenceLine, roadId);
				
				roadMarksCollection.push_back(rElement);
			}
		}
	}
	return roadMarksCollection;
}


RoadMark XODR::roadMarkElement(const roadMark1& rMark, double roadMarkSI, double roadMarkSE, OGRLineString  referenceLine, std::string roadId)const {
	//Creates and returns a RoadMark instance, this function handles objects of the type roadMark1 which differ to the type roadMark

	RoadMark roadMarkElement = RoadMark();
	OGRMultiLineString rMarkLine;
	roadMarkElement.parentRoadId = roadId;
	if (rMark.type1().present()) {
		roadMarkElement.type = rMark.type1().get();
	}

	if (rMark.weight().present()) {
		roadMarkElement.weight = rMark.weight().get();
	}
	if (rMark.color().present()) {
		roadMarkElement.color = rMark.color().get();
	}
	if (rMark.laneChange().present()) {
		roadMarkElement.laneChange = rMark.laneChange().get();
	}

	// Extracting Road Mark Attributes
	double roadMarkSOffset = rMark.sOffset().get();
	bool roadTypeAvailable = rMark.type().present();

	OGRLineString *laneSegment;

	double referenceLineLength = referenceLine.get_Length();
	double sI;
	double sE;

	//Road type is an optional paramaeter, usually it determines whether or not the line is broken into several pieces and the length of the gaps
	if (roadTypeAvailable) {
		const roadMark1::type_optional rMarkTypeOptional = rMark.type();
		type1 rMarkType = (type1)rMarkTypeOptional.get();
		const type1::line_sequence lineSequence = rMarkType.line();
		// Each type tag contains several lines that depict road mark's geometries
		for (int i = 0; i < lineSequence.size(); i++)
		{
			line l = lineSequence[i];
			if (l.rule().present()) {
				roadMarkElement.rule = l.rule().get();
			}
			
			sI = l.sOffset().get() + roadMarkSI;
			if (i < lineSequence.size() - 1) {
				sE = lineSequence[i + 1].sOffset().get() + roadMarkSI;
			}
			else {
				sE = roadMarkSE;
			}
			//Extract line characteristics
			double lengthSegment = l.length().get();
			double space = l.space().get();
			double length = l.length().get();
			double tOffset = l.tOffset().get();
			double lengthExtent = sE - sI;
			//Extract road marks with the previous characteristics, using the reference line
			// Checking that lines are not splitted with values that are outside their lengths, otherwise we might get errors 

			if (sE <= referenceLineLength && sI<sE) {
				// Road marks are related to the correponding reference line (border, reference central line etc). Basically they are a modified version
				// A reference line is broken into several pieces and a tOffset is applied (this depends on the line element characteristics)		
				laneSegment = referenceLine.getSubLine(sI, sE, 0);
				// Applying tOffset
				offsetLine(laneSegment, tOffset);
				//Splitting reference line according to the road marks attributes and storing the results in the multi line string final roadMark
				rMarkLine = splitLineWithSpace(laneSegment, length, space);
			}
		}
		roadMarkElement.line = rMarkLine;
	}
	else {
		//If there is not a "type" tag available, we do not have line sequences as a result we do not split the road mark line
		sI = roadMarkSI;
		sE = roadMarkSE;
		double length = sE - sI;
		if (sE <= referenceLineLength && sI < sE && sI > 0 && sE > 0) {

			laneSegment = referenceLine.getSubLine(sI, sE, 0);
			rMarkLine.addGeometry(laneSegment);			
		}
		roadMarkElement.line = rMarkLine;
	}
	return roadMarkElement;
}

RoadMark XODR::roadMarkElement(const roadMark& rMark, double roadMarkSI, double roadMarkSE, Lane laneElement)const {
	//Creates and returns a RoadMark instance, this function handles objects of the type roadMark which differ to the type roadMark1
	RoadMark roadMarkElement = RoadMark();
	OGRMultiLineString rMarkLine;
	// Extracting Road Mark Attributes
	OGRLineString laneLine = laneElement.line;
	roadMarkElement.laneId = laneElement.id;
	
	if (rMark.type1().present()) {
		roadMarkElement.type = rMark.type1().get();
	}
	
	if (rMark.weight().present()) {
		roadMarkElement.weight = rMark.weight().get();
	}
	if (rMark.color().present()) {
		roadMarkElement.color = rMark.color().get();
	}
	if (rMark.laneChange().present()) {
		roadMarkElement.laneChange = rMark.laneChange().get();
	}
	
	
	double roadMarkSOffset = rMark.sOffset().get();
	bool roadTypeAvailable = rMark.type().present();
	OGRLineString *laneSegment;
	double laneLineLength = laneLine.get_Length();
	double sI;
	double sE;

	//Road type is an optional paramaeter, usually it determines whether or not the line is broken into several pieces and the length of the gaps
	if (roadTypeAvailable) {

		const roadMark::type_optional rMarkTypeOptional = rMark.type();
		const roadMark::type_type rMarkType = (const roadMark::type_type) rMarkTypeOptional.get();
		const roadMark::type_type::line_sequence lineSequence = rMarkType.line();
		// Each type tag contains several lines that depict road mark's geometries

		for (int i = 0; i < lineSequence.size(); i++)
		{
			line l = lineSequence[i];
			if (l.rule().present()) {
				roadMarkElement.rule = l.rule().get();
			}
			

			sI = l.sOffset().get() + roadMarkSI;
			if (i < lineSequence.size() - 1) {
				sE = lineSequence[i + 1].sOffset().get() + roadMarkSI;
			}
			else {
				sE = roadMarkSE;
			}
			//Extract line characteristics
			double lengthSegment = l.length().get();
			double space = l.space().get();
			double tOffset = l.tOffset().get();
			double length = sE - sI;
			//Extract road marks with the previous characteristics, using the reference line
			// Checking that lines are not splitted with values that are outside their lengths, otherwise we might get errors 
			if (length <= laneLineLength && length >0) {
				// Road marks are related to the correponding reference line (border, reference central line etc). Basically they are a modified version
				// A reference line is broken into several pieces and a tOffset might be applied (this depends on the line element characteristics)				

				laneSegment = laneLine.getSubLine(0, length, 0);
				
				// Applying tOffset
				offsetLine(laneSegment, tOffset);
				//Spliting reference line according to the road marks attributes and storing the results in the multi line string final roadMark
				rMarkLine = splitLineWithSpace(laneSegment, lengthSegment, space);
			}
		}
		roadMarkElement.line = rMarkLine;
	}
	else {
		//If there is not a "type" tag available, we do not have line sequences as a result we do not split the road mark line
		sI = roadMarkSI;
		sE = roadMarkSE;
		double length = sE - sI;
		
		if (length <= laneLineLength && length >0) {

			laneSegment = laneLine.getSubLine(0, length, 0);
			
			rMarkLine.addGeometry(laneSegment);
		}
		roadMarkElement.line = rMarkLine;
	}

	return roadMarkElement;
}

std::vector<RoadMark> XODR::roadMarks(const lane::roadMark_sequence roadMarkSequence, double sLaneSectionI, double sLaneSectionE, Lane laneElement)const {
	//Retrives a list containing road marks 

	double roadMarkSI;
	double roadMarkSE;	
	std::vector<RoadMark> roadMarksVector;
	for (int j = 0; j < roadMarkSequence.size(); j++)
	{		
		const roadMark& rMark = roadMarkSequence[j];
		//To compute the starting and ending point of the road marks we take into account information ralated to the lane sections where they are allocated
		double roadMarkSI = rMark.sOffset().get() + sLaneSectionI;
		if (j < roadMarkSequence.size() - 1) {
			roadMarkSE = roadMarkSequence[j + 1].sOffset().get() + sLaneSectionI;
		}
		else {
			roadMarkSE = sLaneSectionE;
		}		
		RoadMark roadMarkObject = roadMarkElement(rMark, roadMarkSI, roadMarkSE, laneElement);
		roadMarksVector.push_back(roadMarkObject);
	}
	return roadMarksVector;
}


/****SIGNALS****/

std::vector<Signal> XODR::roadSignals(const road& xodrRoad) const {	
	//Returns a vector containing the signals allocated along the xodrRoad parameter
	OGRMultiLineString  ogrRoad = toOGRGeometry(xodrRoad.planView());
	OGRLineString  referenceLine = multiLineStringToLineString(&ogrRoad);
	std::string roadId = xodrRoad.id().get();
	const lanes& lanes = xodrRoad.lanes();
	const lanes::laneSection_sequence& laneSections = lanes.laneSection();
	const planView& planView = xodrRoad.planView();
	std::vector<Signal> signalsVector;
	std::string sId = "";
	std::string sName = "";
	std::string sDynamic = "";
	std::string sOrientation = "";
	std::string sType = "";
	std::string sCountry = "";
	std::string sSubtype = "";

	if (xodrRoad.signals().present()) {
		const road::signals_optional& signalsO = xodrRoad.signals();
		signals ogrSignal = (signals)signalsO.get();
		const signals::signal_sequence& signals = ogrSignal.signal();
		int signalsSize = signals.size();
		for (int i = 0; i < signalsSize; i++)
		{		
			const signal& s = signals[i];
			OGRPoint p;
			referenceLine.Value(s.s().get(), &p);
			OGRLineString *lineSegment = referenceLine.getSubLine(s.s().get() - 0.3, s.s().get(), 0);
			OGRPoint normalV = normalUnitaryVector( *lineSegment);
			OGRPoint normalP =  normalProjectedPoint(p, normalV,s.t().get());
			if (s.id().present()) {
				sId = s.id().get();
			}
			if (s.name().present()) {
				sName = s.name().get();
			}
			if (s.dynamic().present()) {
				sDynamic = s.dynamic().get();
			}
			if (s.orientation().present()) {
				sOrientation = s.orientation().get();
			}
			if (s.type().present()) {
				sType = s.type().get();
			}
			if (s.country().present()) {
				sCountry = s.country().get();
			}
			if (s.subtype().present()) {
				sSubtype = s.subtype().get();
			}
			Signal roadSignal = Signal(roadId, sId, sName, sDynamic, sOrientation, sType, sCountry, sSubtype, normalP);

			signalsVector.push_back(roadSignal);
		}
	}
	return signalsVector;
}

bool XODR::signalPresent(const road& xodrRoad) const {
	// Returns true in case of finding signals available inside the given xodrRoad parameter	
	if (xodrRoad.signals().present() == false) {
		return false;
	}
	
	const road::signals_optional& signalsO = xodrRoad.signals();
	signals ogrSignal = (signals)signalsO.get();
	const signals::signal_sequence& signals = ogrSignal.signal();
	int signalsSize = signals.size();
	if (signalsSize > 0) {
		return true;
	}
	else {
		return false;
	}
}

/**********/
//Object//

bool XODR::objectPresent(const road& xodrRoad) const {
	if (!xodrRoad.objects().present()) {
		return false;
	}
	
	const road::objects_optional& objectO = xodrRoad.objects();
	objects ogrObject = (objects)objectO.get();
	const objects::object_sequence& objects = ogrObject.object();
	int objectsSize = objects.size();

	if (objectsSize > 0) {
		return true;
	}
	else {
		return false;
	}	
}

bool XODR::objectPolygonPresent(const road& xodrRoad) const {
	if (!xodrRoad.objects().present()) {
		return false;
	}

	const road::objects_optional& objectO = xodrRoad.objects();
	objects ogrObject = (objects)objectO.get();
	const objects::object_sequence& objects = ogrObject.object();
	int objectsSize = objects.size();

	for (int i = 0; i < objectsSize; i++)
	{

		if (objects[i].outline().present()) {
			return true;
		}
	}
	return false;
}


bool XODR::objectLinePresent(const road& xodrRoad) const {
	if (!xodrRoad.objects().present()) {
		return false;
	}

	const road::objects_optional& objectO = xodrRoad.objects();
	objects ogrObject = (objects)objectO.get();
	const objects::object_sequence& objects = ogrObject.object();
	int objectsSize = objects.size();

	for (int i = 0; i < objectsSize; i++)
	{

		if (objects[i].repeat().size() > 0) {

			const object::repeat_sequence& rS = objects[i].repeat();
			int rSize = rS.size();
			for (int j = 0; j < rSize; j++)
			{
				const repeat& r = rS[j];


				double rDistance = r.distance().get();
				if (rDistance == 0) {
					return true;
				}
			}
		}
	}
	return false;
}



std::vector<Object> XODR::roadObject(const road& xodrRoad) const {
	OGRMultiLineString  ogrRoad = toOGRGeometry(xodrRoad.planView());
	OGRLineString  referenceLine = multiLineStringToLineString(&ogrRoad);
	OGRLineString *lineSegment;

	std::vector<Object> objectCollection;
	if (objectPresent(xodrRoad)) {
		
		const road::objects_optional& objectO = xodrRoad.objects();
		objects ogrObject = (objects)objectO.get();
		const objects::object_sequence& objects = ogrObject.object();
		int objectsSize = objects.size();
		for (int i = 0; i < objectsSize; i++)
		{
			const object& o = objects[i];
			
			if (o.s().present()) {
				//Some erroneous files contain object records tha do not have the "s" attribute, as a result the code crashes. We only model oject elements that contain the s attribute.
				if (o.repeat().size() > 0) {

					const object::repeat_sequence& rS = o.repeat();
					int rSize = rS.size();
					for (int j = 0; j < rSize; j++)
					{
						const repeat& r = rS[j];

						double rDistance = r.distance().get();
						if (rDistance == 0) {
							//If the distance is equal to zero we assume that the repeat statement is representing a linear object such as a wall (see OpenDRIVE 1.4 reference), in this case we create a linear object.
							OGRLineString lineE;
							OGRMultiLineString multiLine;
							Object objectElement = createPointObject(r.s().get(), r.tStart().get(), o, referenceLine, xodrRoad.id().get());
							lineE.addPoint(&objectElement.position);
							objectElement = createPointObject(r.s().get() + r.length().get(), r.tEnd().get(), o, referenceLine, xodrRoad.id().get());
							lineE.addPoint(&objectElement.position);
							multiLine.addGeometry(&lineE);
							objectElement.geometryType = "line";
							objectElement.line = multiLine;
							objectCollection.push_back(objectElement);
						}
						else {
							// If the attribute distace is different to zero, we create several point objects equally spaced with the distance attribute
							for (double k = r.s().get(); k < r.s().get() + r.length().get(); k += rDistance)
							{
								Object objectElement = createPointObject(k, r.tStart().get(), o, referenceLine, xodrRoad.id().get());
								objectElement.geometryType = "point";
								objectCollection.push_back(objectElement);
							}
							Object objectElement = createPointObject(r.s().get() + r.length().get(), r.tEnd().get(), o, referenceLine, xodrRoad.id().get());
							objectElement.geometryType = "point";
							objectCollection.push_back(objectElement);
						}
					}

				}
				else if (o.outline().present()) {
					// If the objects contain an outline attribute, their geometries are represented with polygons
					Object objectElement = createPointObject(o.s().get(), o.t().get(), o, referenceLine, xodrRoad.id().get());

					const object::outline_optional& outlineElementO = o.outline();
					outline outlineElement = (outline)outlineElementO.get();
					if (outlineElement.cornerLocal().size()> 0) {
						const outline::cornerLocal_sequence& cornerLocalSeq = outlineElement.cornerLocal();

						objectElement.shape = createPolygonObject(objectElement, cornerLocalSeq);
						objectElement.geometryType = "polygon";
						objectCollection.push_back(objectElement);

					}
					else if (outlineElement.cornerRoad().size()>0) {
						const outline::cornerRoad_sequence& cornerRoadSeq = outlineElement.cornerRoad();
						objectElement.shape = createPolygonObject(objectElement, cornerRoadSeq, referenceLine);
						objectElement.geometryType = "polygon";
						objectCollection.push_back(objectElement);
					}

				}
				else {
					// If the object element does not contain either outline or repeat attributes, they are represented as points
					Object objectElement = createPointObject(o.s().get(), o.t().get(), o, referenceLine, xodrRoad.id().get());
					objectElement.geometryType = "point";
					objectCollection.push_back(objectElement);
				}
			}
			
		}
	}

	return objectCollection;
}

Object XODR::createPointObject(double s, double t, const object& o, OGRLineString  referenceLine, std::string roadId) const {
	OGRPoint p;
	OGRLineString *lineSegment;
	referenceLine.Value(o.s().get(), &p);
	if (s + 0.1 < referenceLine.get_Length()) {
		lineSegment = referenceLine.getSubLine(s, s + 0.1, 0);
	}
	else {
		lineSegment = referenceLine.getSubLine(s - 0.1, s, 0);
	}

	OGRPoint normalV = normalUnitaryVector(*lineSegment);
	OGRPoint normalP = normalProjectedPoint(p, normalV, t);
	
	Object objectElement = Object(normalP);
	objectElement.roadId = roadId;
	const object::outline_optional& outlineElementO = o.outline();
	
	
	if (o.hdg().present()) {
		objectElement.hdg = o.hdg().get();
	}
	if (o.height().present()) {
		objectElement.height = o.height().get();
	}
	if (o.id().present()) {
		objectElement.id = o.id().get();
	}

	if (o.name().present()) {
		objectElement.name = o.name().get();
	}
	if (o.orientation().present()) {
		objectElement.orientation = o.orientation().get();
	}

	if (o.pitch().present()) {
		objectElement.pitch = o.pitch().get();
	}
	if (o.radius().present()) {
		objectElement.radius = o.height().get();
	}
	if (o.roll().present()) {
		objectElement.roll = o.roll().get();
	}
	if (o.type().present()) {
		objectElement.type = o.type().get();
	}
	return objectElement;
}

OGRPolygon XODR::createPolygonObject(Object o, const outline::cornerLocal_sequence& cornerLocalSeq) const {
	//Creates a polygon whose origin is a pivot point of coordinates u,v
	OGRPolygon poly;
	OGRLinearRing ring1;
	int size = cornerLocalSeq.size();

	for (int i = 0; i < size; i++)
	{
		const cornerLocal& c = cornerLocalSeq[i];
		double x = o.position.getX() + c.u().get();
		double y = o.position.getY() + c.v().get();
		//The coordinates are rotated according to the hdg atribute
		double xRotated = cos(o.hdg) * x - sin(o.hdg) * y;
		double yRotated = sin(o.hdg) * x + cos(o.hdg) * y;		
		ring1.addPoint(xRotated, yRotated);
	}	
	ring1.closeRings();
	poly.addRing(&ring1);
	return poly;
}


OGRPolygon XODR::createPolygonObject(Object o, const outline::cornerRoad_sequence& cornerRoadSeq, OGRLineString  referenceLine) const {
	//Creates a polygon whose coordinates are mapped with respect to the reference line
	OGRPolygon poly;
	OGRLinearRing ring1;
	int size = cornerRoadSeq.size();
	for (int i = 0; i < size; i++)
	{
		const cornerRoad& c = cornerRoadSeq[i];
		OGRPoint p;
		OGRLineString *lineSegment;
		referenceLine.Value(c.s().get(), &p);
		if (c.s().get() + 0.1 < referenceLine.get_Length()) {
			lineSegment = referenceLine.getSubLine(c.s().get(), c.s().get() + 0.1, 0);
		}
		else {
			lineSegment = referenceLine.getSubLine(c.s().get() - 0.1, c.s().get(), 0);
		}
		OGRPoint normalV = normalUnitaryVector(*lineSegment);
		OGRPoint normalP = normalProjectedPoint(p, normalV, c.t().get());
		ring1.addPoint(normalP.getX(), normalP.getY());
	}
	ring1.closeRings();
	poly.addRing(&ring1);
	return poly;
}
