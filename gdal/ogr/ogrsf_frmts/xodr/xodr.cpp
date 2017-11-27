/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Implementation of algorithms for conversion of OpenDRIVE to OGC Simple Features.
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
                    CPLError(CE_Warning, CPLE_AppDefined,
                            "Road geometry parts are disjoint at point (%lf, %lf) with a distance greater than %lf",
                            currentStart->getX(), currentStart->getY(), tolerance);
                }
            }
        }

    }
}

std::auto_ptr<OGRLineString> XODR::toOGRGeometry(const geometry& xodrGeometry) const {
    std::auto_ptr<OGRLineString> convertedGeom(new OGRLineString());
    if (xodrGeometry.line().present()) {
        convertedGeom = lineToLinestring(xodrGeometry);
    } else if (xodrGeometry.arc().present()) {
        convertedGeom = arcToLinestring(xodrGeometry, 0); // default angle step size
    } else if (xodrGeometry.spiral().present()) {
        convertedGeom = spiralToLinestring(xodrGeometry, 0.5);
    } else if (xodrGeometry.poly3().present()) {
        convertedGeom = poly3ToLinestring(xodrGeometry, 0.5);
    } else if (xodrGeometry.paramPoly3().present()) {
        convertedGeom = paramPoly3ToLinestring(xodrGeometry, 0.5);
    }
    return convertedGeom;
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
    pRange parameterRange = paramPoly3->pRange().get();

    std::auto_ptr<OGRLineString> lineString(new OGRLineString());
    switch (parameterRange) {
        case pRange::value::arcLength: 
            sampleParamPoly3(length, uA, uB, uC, uD, vA, vB, vC, vD, sampleDistance, lineString.get());
            break;
        case pRange::value::normalized: 
            sampleParamPoly3(1.0, uA, uB, uC, uD, vA, vB, vC, vD, sampleDistance / length, lineString.get());
            break;
    }

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
