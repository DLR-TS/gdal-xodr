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
#include "odrSpiral.h"
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
#include "OpenDRIVE_1.4H.h"
#include <sstream>

using namespace std;
using namespace xml_schema;

/************************************************************************/
/*                            XODR Class Constructor                    */

/************************************************************************/
XODR::XODR(const char * pszFilename) {
    op = OpenDRIVE_(pszFilename, flags::keep_dom | flags::dont_validate);
}

XODR::~XODR() {
}

/************************************************************************/
/*                       getMinorRevision                               */

/************************************************************************/
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
    //    integrateGeometryParts(&ogrRoad);
    return ogrRoad;
}

void XODR::integrateGeometryParts(OGRMultiLineString* ogrRoad) const {
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
            // Move distance tolerance out into external OGR format parameter
            if (currentStart->Disjoint(predEnd) && currentStart->Distance(predEnd) < 0.001) {
                pred->setPoint(numPts - 1, currentStart);
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
    } else if (xodrGeometry.paramPoly3().present()) {
    }
    return convertedGeom;
}

std::auto_ptr<OGRLineString> XODR::lineToLinestring(const geometry &geoParam) const {
    // Arguments: Geometry and Line with the following values/[units]
    // s [m], x [m], y [m], hdg [rad], length [m] 

    double xStart = geoParam.x().get(); // initial coordinate x (X_init, Y_init)
    double yStart = geoParam.y().get(); // initial coordinate y
    double hdg = geoParam.hdg().get(); // heading (polar coordinate)
    double length = geoParam.length().get(); // length

    // Calculation of end coordinates (X_end, Y_end), from polar coord and lenght:
    // Formula ( x + cos(hdg) * length, y + sin(hdg) * length)

    double xEnd = xStart + (std::cos(hdg)) * length;
    double yEnd = yStart + (std::sin(hdg)) * length;

    std::auto_ptr<OGRLineString> lineString(new OGRLineString());
    OGRPoint ptStart(xStart, yStart);
    lineString->addPoint(&ptStart);
    OGRPoint ptEnd(xEnd, yEnd);
    lineString->addPoint(&ptEnd);

    return lineString;
}

std::auto_ptr<OGRLineString> XODR::arcToLinestring(const geometry &geoParam) const {

    // Arguments: Geometry and Arc with the following values/[units]
    // s [m], x [m], y [m], hdg [rad], length [m], curvature [1/m] 

    //From: <geometry> tag
    //geometry::s_optional& s = geoParam.s();
    const geometry::x_optional& x = geoParam.x();
    const geometry::y_optional& y = geoParam.y();
    const geometry::hdg_optional& hdg = geoParam.hdg();
    const geometry::length_optional& length = geoParam.length();

    // From: <arc> tag
    const geometry::arc_optional& arc = geoParam.arc();
    const arc::curvature_optional& curv = arc->curvature();

    //double s_value = s.get(); // reference
    double x_value = x.get(); // initial coordinate x (X_init, Y_init)
    double y_value = y.get(); // initial coordinate y
    double heading = hdg.get(); // heading (polar coordinate)
    double length_value = length.get(); // length	
    double curvature = curv.get();

    /* Verification of variables (for testing)
    printf("curvature [1/m]    :  %f \n ", curvature);
    printf("length    [m]      :  %f \n ", length_value);
    printf("heading   [rad ]   :  %f \n ", heading);
     */

    // *********     Step 1 : Arc definition     ************
    // To find the points that define the arc (Xo,Yo), (Xm,Ym) and (Xe,Ye)
    // General formula for finding points
    // for polar coordinates with center (0,0)

    double r = 1 / abs(curvature);
    double alpha = length_value / r;

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
    double xo_f = xo + x_value;
    double yo_f = xo + y_value;

    // Translation: Middle point
    double xm_f = xm + x_value;
    double ym_f = ym + y_value;

    // Translation: End point
    double xe_f = xe + x_value;
    double ye_f = ye + y_value;


    // *********  Step 5 : Create the geometry as OGRLineString *******
    // Sample rate, given by sampling angle: 15 grades
    // Initial, middle and end points with translation (r), rotation(hdg) and plane translation

    std::auto_ptr<OGRLineString> arcLineString(OGRGeometryFactory::curveToLineString(xo_f, yo_f, 0, xm_f, ym_f, 0, xe_f, ye_f, 0, false, 15, NULL));
    return arcLineString;
}

std::auto_ptr<OGRLineString> XODR::spiralToLinestring(const geometry &geoParam) const {
    // Basic geometry attributes
    double xStart = geoParam.x().get(); // initial coordinate x (X_init, Y_init)
    double yStart = geoParam.y().get(); // initial coordinate y
    double hdg = geoParam.hdg().get(); // heading (polar coordinate)
    double length = geoParam.length().get(); // length

    // Spiral attributes
    const geometry::spiral_optional& spiral = geoParam.spiral();
    double startCurvature = spiral->curvStart().get();
    double endCurvature = spiral->curvEnd().get();

    std::auto_ptr<OGRLineString> lineString(new OGRLineString());
    if (startCurvature == 0.0) {
        discretiseStandardSpiralPoints(xStart, yStart, hdg, length, endCurvature, lineString.get());
    } else if (startCurvature != 0.0 && endCurvature == 0.0) {
        discretiseStandardSpiralPoints(xStart, yStart, hdg, length, startCurvature, lineString.get());
        transformCase2(lineString.get());
    }

    return lineString;
}

void XODR::discretiseStandardSpiralPoints(double xStart, double yStart, double heading, double length,
        double endCurvature, OGRLineString* lineString) const {
    // Intermediate points
    // http://math.stackexchange.com/questions/1785816/calculating-coordinates-along-a-clothoid-betwen-2-curves?newreg=5b5dce86ebc5450c9042f47aeb7a0163
    double curvatureChange = endCurvature / length;
    double x;
    double y;
    double t;
    for (double s = 0.0; s < length; s += 0.5) {
        odrSpiral(s, curvatureChange, &x, &y, &t);
        //        double xRot = x * cos(heading) - y * sin(heading);
        //        double yRot = x * sin(heading) + y * cos(heading);
        //        double xRotTrans = xRot + xStart;
        //        double yRotTrans = yRot + yStart;
        double xRotTrans = x + xStart;
        double yRotTrans = y + yStart;
        OGRPoint ptIntermediate(xRotTrans, yRotTrans);
        lineString->addPoint(&ptIntermediate);
    }

    // End point
    odrSpiral(length, curvatureChange, &x, &y, &t);
    //    double xRot = x * cos(heading) - y * sin(heading);
    //    double yRot = x * sin(heading) + y * cos(heading);
    //    double xRotTrans = xRot + xStart;
    //    double yRotTrans = yRot + yStart;
    double xRotTrans = x + xStart;
    double yRotTrans = y + yStart;
    OGRPoint ptEnd(xRotTrans, yRotTrans);
    lineString->addPoint(&ptEnd);
}

void XODR::transformCase2(OGRLineString* lineString) const {
    OGRPoint* start = new OGRPoint();
    lineString->getPoint(0, start);

    int numPts = lineString->getNumPoints();
    double x, y;
    for (int p = 0; p < numPts; p++) {
        OGRPoint* pt = new OGRPoint();
        lineString->getPoint(p, pt);
        x = pt->getX() - start->getX();
        y = pt->getY() - start->getY();
        double xFlip = x * 1 + y * 0;
        double yFlip = x * 0 + y * -1;
        pt->setX(xFlip);
        pt->setY(yFlip);
        lineString->setPoint(p, pt);
    }
//    lineString->getPoint(0, start);
//    OGRPoint* end = new OGRPoint();
//    lineString->getPoint(numPts, end);
    for (int p = 0; p < numPts; p++) {
        OGRPoint* pt = new OGRPoint();
        lineString->getPoint(p, pt);
        x = pt->getX();
        y = pt->getY();
        x += start->getX();
        y += start->getY();
        pt->setX(x);
        pt->setY(y);
        lineString->setPoint(p, pt);
    }
}

int XODR::getNumberOfRoads() {
    return op->road().size();
}