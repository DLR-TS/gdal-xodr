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
#include "xodr.h"
#include "ogr_geometry.h"
#include <cmath>
#include <iostream>
#include <memory>   
#include <string>
#include <typeinfo>
#include <vector>
#include "cpl_conv.h"
#include <OpenDRIVE_1.4H.h>
#include <sstream>

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
    integrateGeometryParts(&ogrRoad, 0.001);
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
                            "Road geometry parts are disjoint at point (%lf, %lf) and have a distance greater than %lf", 
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
        convertedGeom = arcToLinestring(xodrGeometry);
    } else if (xodrGeometry.spiral().present()) {
        convertedGeom = spiralToLinestring(xodrGeometry);
    } else if (xodrGeometry.poly3().present()) {
        CPLError(CE_Warning, CPLE_NotSupported, "Geometries of type poly3 not supported");
    } else if (xodrGeometry.paramPoly3().present()) {
        CPLError(CE_Warning, CPLE_NotSupported, "Geometries of type paramPoly3 not supported");
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

std::auto_ptr<OGRLineString> XODR::arcToLinestring(const geometry &geom) const {
    // Basic geometry attributes    
    double xStart = geom.x().get();
    double yStart = geom.y().get();
    double heading = geom.hdg().get();
    double length = geom.length().get();

    // Arc attributes
    const geometry::arc_optional& arc = geom.arc();
    double curvature = arc->curvature().get();

    // *********     Step 1 : Arc definition     ************
    // To find the points that define the arc (Xo,Yo), (Xm,Ym) and (Xe,Ye)
    // General formula for finding points
    // for polar coordinates with center (0,0)

    double r = 1 / abs(curvature);
    double alpha = length / r;

    double alpha_0 = 0;
    double xo = 0;
    double yo = 0;
    double xm = 0;
    double ym = 0;
    double xe = 0;
    double ye = 0;

    //   For negative curvature, the points are given by
    //   x = r*sin  and  y = r*-cos 
    if (curvature > 0) {

        // Initial point: alpha = 0
        alpha_0 = 0;
        xo = r * sin(alpha_0);
        yo = r * (-cos(alpha_0));

        // Middle point: angle = alpha/2
        xm = r * sin(alpha / 2);
        ym = r * (-cos(alpha / 2));

        // End point: angle = alpha
        xe = r * sin(alpha);
        ye = r * (-cos(alpha));

    } else {
        //  [!] For negative curvature, the points are given by
        //   x = r*sin  and  y = r*cos 

        // Initial point: alpha = 0
        alpha_0 = 0;
        xo = r * sin(alpha_0);
        yo = r * (cos(alpha_0));

        // Middle point: angle = alpha/2
        xm = r * sin(alpha / 2);
        ym = r * (cos(alpha / 2));

        // End point: angle = alpha
        xe = r * sin(alpha);
        ye = r * (cos(alpha));
    }

    // *********  Step 2 : Arc Translation ************
    // Tranlation of the points with the Translation matrix T 
    // General formula for finding the points 
    // XY_T = T + P    T = [0, r]

    // Variables initialization 
    double xt = 0;
    double yt = 0;

    // Translation matrix definition for positive curvature
    if (curvature > 0) {
        xt = 0;
        yt = r;
    } else {
        xt = 0;
        yt = -r;
    }

    // Translation: Initial point
    double xo_t = xt + xo;
    double yo_t = yt + yo;

    // Translation: Middle point
    double xm_t = xt + xm;
    double ym_t = yt + ym;

    // Translation: End point
    double xe_t = xt + xe;
    double ye_t = yt + ye;


    /* *********  Step 3 : Arc Rotation  ************
    // Rotation of the points with the heading
    // General formula for finding the points 
    // R =  cos  -sin
    //      sin   cos								 */

    // Rotation: Initial point
    xo = xo_t * cos(heading) - yo_t * sin(heading);
    yo = xo_t * sin(heading) + yo_t * cos(heading);

    // Rotation: Middle point
    xm = xm_t * cos(heading) - ym_t * sin(heading);
    ym = xm_t * sin(heading) + ym_t * cos(heading);

    // Rotation: End point
    xe = xe_t * cos(heading) - ye_t * sin(heading);
    ye = xe_t * sin(heading) + ye_t * cos(heading);


    /* *********  Step 4 : Translation from polar to OpenDRIVE coordinates *******
    // From origen (0,0) of the polar coordinates to the Openroad (x,y)  
    // coordinates plane. 
    // coord_final (x_f, y_f) = polar + opendrive								*/

    // Translation: Initial point
    double xo_f = xo + xStart;
    double yo_f = xo + yStart;

    // Translation: Middle point
    double xm_f = xm + xStart;
    double ym_f = ym + yStart;

    // Translation: End point
    double xe_f = xe + xStart;
    double ye_f = ye + yStart;


    // *********  Step 5 : Create the geometry as OGRLineString *******
    // Sample rate, given by sampling angle: 15 grades
    // Initial, middle and end points with translation (r), rotation(hdg) and plane translation

    std::auto_ptr<OGRLineString> arcLineString(OGRGeometryFactory::curveToLineString(xo_f, yo_f, 0, xm_f, ym_f, 0, xe_f, ye_f, 0, false, 15, NULL));
    return arcLineString;
}

std::auto_ptr<OGRLineString> XODR::spiralToLinestring(const geometry &geom) const {
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
        sampleDefaultSpiral(length, endCurvature, 0.5, lineString.get());

        // T(geomStartPt) x R(geomHdg) x geometry
        MatrixTransformations::translate(m, xStart, yStart);
        MatrixTransformations::rotate(m, hdg);
        transformLineString(lineString.get(), m);
    } else if (startCurvature != 0.0 && endCurvature == 0.0) {
        double tangentDir = sampleDefaultSpiral(length, -startCurvature, 0.5, lineString.get());
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
