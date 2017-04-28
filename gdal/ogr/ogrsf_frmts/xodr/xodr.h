/******************************************************************************
 * $Id:  $
 *
 * Project:   OpenGIS Simple Features for OpenDRIVE format
 * Purpose:   Implements OGRXODRDriver class.
 * Authors:   Ana Maria Orozco  ana dot orozco at tum dot de
 *			  Michael Scholz    michael dot scholz at dlr dot de
 *            Deutsches Zentrum f�r Luft- und Raumfahrt, DLR
 ******************************************************************************/


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


/************************************************************************/
/*                               XODR                                   */

/************************************************************************/

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
    void discretiseStandardSpiralPoints(double xStart, double yStart, double heading, double length,
        double endCurvature, OGRLineString* lineString) const;
    void transformCase2(OGRLineString* lineString) const;
    int getNumberOfRoads();
    std::string getGeoReferenceString();
};
#endif /* ndef _XODR_H_INCLUDED */