/******************************************************************************
 * $Id:  $
 *
 * Project:   OpenGIS Simple Features for OpenDRIVE format
 * Purpose:   Implements OGRXODRDriver class.
 * Authors:   Ana Maria Orozco  ana dot orozco at tum dot de
 *			  Michael Scholz    michael dot scholz at dlr dot de
 *            Deutsches Zentrum f�r Luft- und Raumfahrt, DLR
 ******************************************************************************/

#include "ogr_xodr.h"
#include "ogr_api.h"
#include "ogr_geometry.h"
#include <cmath>
#include <iostream>
#include <memory>   
#include <string>
#include <typeinfo>
#include <vector>
#include <iterator> 

using namespace std;
using namespace xml_schema;

OGRXODRLayer::OGRXODRLayer(XODR* xodr, const char* pszName) :
xodr(xodr),
xodrRoads(xodr->getXODRRoads()),
xodrRoadsIte(xodrRoads.begin()),
poFeatureDefn(new OGRFeatureDefn(CPLGetBasename(pszName))) {
    poFeatureDefn->SetGeomType(wkbMultiLineString);
    poFeatureDefn->Reference();

    ResetReading();

    // Sparial Reference (for OpenDRIVE >= 1.4)
    poSRS = NULL;
    if (xodr -> getMinorRevision() > 3) {
        std::string geoRef = xodr->getGeoReferenceString();
        if (!geoRef.empty()) {
            poSRS = new OGRSpatialReference();
            poSRS->importFromProj4(geoRef.c_str());
            //            poSRS->dumpReadable();
            poFeatureDefn->GetGeomFieldDefn(0)->SetSpatialRef(poSRS);
        }
    }

    OGRFieldDefn oFieldName("Name", OFTString);
    poFeatureDefn->AddFieldDefn(&oFieldName);
    OGRFieldDefn oFieldId("Id", OFTString);
    poFeatureDefn->AddFieldDefn(&oFieldId);
    OGRFieldDefn oFieldJunction("Junction", OFTString);
    poFeatureDefn->AddFieldDefn(&oFieldJunction);
}

OGRXODRLayer::~OGRXODRLayer() {
    if (NULL != poFeatureDefn) {
        poFeatureDefn->Release();
    }
    if (NULL != poSRS) {
        poSRS->Release();
    }
}

void OGRXODRLayer::ResetReading() {
    xodrRoadsIte = xodrRoads.begin();
    nNextFID = 0;
}

OGRFeature* OGRXODRLayer::GetNextFeature() {
    if (xodrRoadsIte != xodrRoads.end()) {
        const road& xodrRoad = *xodrRoadsIte;
        const planView& planView = xodrRoad.planView();
        OGRMultiLineString ogrRoad = xodr->toOGRGeometry(planView);

        OGRFeature* poFeature = new OGRFeature(poFeatureDefn);
        poFeature->SetGeometryDirectly(ogrRoad.clone());
        poFeature->SetField(poFeatureDefn->GetFieldIndex("Name"), xodrRoad.name().get().c_str());
        poFeature->SetField(poFeatureDefn->GetFieldIndex("Id"), xodrRoad.id().get().c_str());
        poFeature->SetField(poFeatureDefn->GetFieldIndex("Junction"), xodrRoad.junction().get().c_str());
        poFeature->SetFID(nNextFID++);

        ++xodrRoadsIte;

        if ((m_poFilterGeom == NULL || FilterGeometry(poFeature->GetGeometryRef()))
                && (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate(poFeature))) {
            return poFeature;
        }
        delete poFeature;
    }
    return NULL;
}
