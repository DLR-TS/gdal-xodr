/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Implementation of OGRXODRLayer.
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

//pass third argument
OGRXODRLayer::OGRXODRLayer(XODR* xodr, const char* pszName, std::string lName) :
xodr(xodr),
xodrRoads(xodr->getXODRRoads()),
xodrRoadsIte(xodrRoads.begin()),
poFeatureDefn(new OGRFeatureDefn(CPLGetBasename(pszName))) {

	//Defines a data structure according to the lName parameter (layer name), this parameter is passed by the user in the command prompt
	layerName = lName;
    poFeatureDefn->SetGeomType(wkbMultiLineString);
    poFeatureDefn->Reference();

    ResetReading();

    // Spatial Reference (for OpenDRIVE >= 1.4)
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

	if (layerName == "referenceLine") {		
		OGRFieldDefn oFieldName("Name", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldName);
		OGRFieldDefn oFieldId("Id", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldId);
		OGRFieldDefn oFieldJunction("Junction", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldJunction);
	}
	else if (layerName == "lane") {
		
		OGRFieldDefn oFieldId("Id", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldId);
		OGRFieldDefn oFieldType("Type", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldType);
		OGRFieldDefn oFieldLevel("Level", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldLevel);
		OGRFieldDefn oFieldSingleSide("SingleSide", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldSingleSide);
		OGRFieldDefn oFieldRoadID("RoadID", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRoadID);
	}
	else if (layerName == "roadMark") {

		OGRFieldDefn oFieldId("Id", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldId);
		OGRFieldDefn oFieldType("Type", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldType);
		OGRFieldDefn oFieldWeight("Weight", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldWeight);
		OGRFieldDefn oFieldColor("Color", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldColor);
		OGRFieldDefn oFieldLaneChange("LaneChange", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldLaneChange);
		OGRFieldDefn oFieldRule("Rule", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRule);
	}
	else if (layerName == "signal") {
		
		//We create a vector containing only the roads that contain signals
		for (OpenDRIVE::road_const_iterator it = xodrRoads.begin(); it != xodrRoads.end(); ++it) {
			const road& xodrRoad = *it;
		
			if (xodr->signalPresent(xodrRoad)) {	
				signalRoads.push_back(xodrRoad);
			}
			
		}
		signalRoadSize = signalRoads.size();
		OGRFieldDefn oFieldRoadId("RoadId", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRoadId);
		poFeatureDefn->SetGeomType(wkbPoint);
		OGRFieldDefn oFieldId("Id", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldId);
		OGRFieldDefn oFieldName("Name", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldName);
		OGRFieldDefn oFieldDynamic("Dynamic", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldDynamic);
		OGRFieldDefn oFieldOrientation("Orient", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldOrientation);
		OGRFieldDefn oFieldType("Type", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldType);		
		OGRFieldDefn oFieldCountry("Country", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldCountry);
		OGRFieldDefn oFieldSubtype("Subtype", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldSubtype);
	}

	else if (layerName == "objectPoint") {
		//We create a vector containing only the roads that contain object elements
		for (OpenDRIVE::road_const_iterator it = xodrRoads.begin(); it != xodrRoads.end(); ++it) {
			const road& xodrRoad = *it;

			if (xodr->objectPresent(xodrRoad)) {
				objectRoads.push_back(xodrRoad);
			}

		}
		objectRoadSize = objectRoads.size();
		OGRFieldDefn oFieldRoadId("RoadId", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRoadId);
		poFeatureDefn->SetGeomType(wkbPoint);
		OGRFieldDefn oFieldId("Id", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldId);
		OGRFieldDefn oFieldName("Name", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldName);
		OGRFieldDefn oFieldOrientation("Orient", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldOrientation);
		OGRFieldDefn oFieldPitch("Pitch", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldPitch);
		OGRFieldDefn oFieldRoll("Roll", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRoll);
		OGRFieldDefn oFieldRadius("Radius", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRadius);
		OGRFieldDefn oFieldType("Type", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldType);
	}

	else if (layerName == "objectPolygon") {

		for (OpenDRIVE::road_const_iterator it = xodrRoads.begin(); it != xodrRoads.end(); ++it) {
			const road& xodrRoad = *it;

			if (xodr->objectPolygonPresent(xodrRoad)) {
				objectRoads.push_back(xodrRoad);
			}

		}

		objectRoadSize = objectRoads.size();
		OGRFieldDefn oFieldRoadId("RoadId", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRoadId);
		poFeatureDefn->SetGeomType(wkbPolygon);
		OGRFieldDefn oFieldId("Id", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldId);
		OGRFieldDefn oFieldName("Name", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldName);
		OGRFieldDefn oFieldOrientation("Orient", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldOrientation);
		OGRFieldDefn oFieldPitch("Pitch", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldPitch);
		OGRFieldDefn oFieldRoll("Roll", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRoll);
		OGRFieldDefn oFieldRadius("Radius", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRadius);
		OGRFieldDefn oFieldType("Type", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldType);
	}

	else if (layerName == "objectLine") {
		//We create a vector containing only the roads that contain object elements
		for (OpenDRIVE::road_const_iterator it = xodrRoads.begin(); it != xodrRoads.end(); ++it) {
			const road& xodrRoad = *it;

			if (xodr->objectLinePresent(xodrRoad)) {
		
				objectRoads.push_back(xodrRoad);
			}

		}
		objectRoadSize = objectRoads.size();
		OGRFieldDefn oFieldRoadId("RoadId", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRoadId);
		OGRFieldDefn oFieldId("Id", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldId);
		OGRFieldDefn oFieldName("Name", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldName);
		OGRFieldDefn oFieldOrientation("Orient", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldOrientation);
		OGRFieldDefn oFieldPitch("Pitch", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldPitch);
		OGRFieldDefn oFieldRoll("Roll", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRoll);
		OGRFieldDefn oFieldRadius("Radius", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldRadius);
		OGRFieldDefn oFieldType("Type", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldType);
	}

	else {
	
		OGRFieldDefn oFieldName("Name", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldName);
		OGRFieldDefn oFieldId("Id", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldId);
		OGRFieldDefn oFieldJunction("Junction", OFTString);
		poFeatureDefn->AddFieldDefn(&oFieldJunction);

	}
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
	// Retrieves a layer according to the current layerName parameter, this parameter is parsed by the user in the command prompt
	if (layerName == "referenceLine") {
		if (print) {
			print = false;
			std::cout << "Generating reference lines \n";
		}

		return referenceLineFeature();
	} else if (layerName == "lane") {
		if (print) {
			print = false;
			std::cout << "Generating Lanes \n";
		}
		return laneFeature();
	}
	else if (layerName == "roadMark") {
		if (print) {
			print = false;
			std::cout << "Generating road marks \n";
		}
		return roadMarkFeature();
	}
	else if (layerName == "signal") {
		if (print) {
			print = false;
			std::cout << "Generating signals \n";
		}
		//pass vector containing only the roads that have signal
		return signalFeature(signalRoads);
	}
	else if (layerName == "objectPoint") {
		if (print) {
			print = false;
			std::cout << "Generating objects \n";
		}
		//pass vector containing only the roads that have signal
		return objectFeature(objectRoads);
	}
	else if (layerName == "objectPolygon") {
		if (print) {
			print = false;
			std::cout << "Generating objects of the type polygon\n";
		}
		return objectFeaturePolygon();
	}
	else if (layerName == "objectLine") {
		if (print) {
			print = false;
			std::cout << "Generating objects of the type line\n";
		}
		return objectFeatureLine();
	}

	else {
		return referenceLineFeature();
	}

}


OGRFeature* OGRXODRLayer::referenceLineFeature() {
	if (xodrRoadsIte != xodrRoads.end()) {

		const road& xodrRoad = *xodrRoadsIte;
		const planView& planView = xodrRoad.planView();

		std::cout << "Processing referenceLine elements of road " << xodrRoad.id().get() << "\n";			
		


		OGRMultiLineString  ogrRoad = xodr->toOGRGeometry(planView);
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

OGRFeature* OGRXODRLayer::laneFeature() {
	
	if (xodrRoadsIte != xodrRoads.end() ) {
		const road& xodrRoad = *xodrRoadsIte;
		const planView& planView = xodrRoad.planView();
		const lanes& lanes = xodrRoad.lanes();
		OGRMultiLineString  ogrRoad = xodr->toOGRGeometry(planView);
		
		//Lanes are only computed once
		//This method is called several times, to reduce the execution time, we only compute the lanes once in every road (when the lane index is equal to 0)
		if (laneIndex == 0) {
			std::cout << "Processing lane elements of road " << xodrRoad.id().get() << "\n";
			lanesList = xodr->roadLanes(xodrRoad);
			laneSize = lanesList.size();			
		} 		
		OGRFeature* poFeature = new OGRFeature(poFeatureDefn);
		//We check whether there are lanes available, because some roads do not have lanes
		if (laneSize > 0) {
			OGRMultiLineString laneLine;
			Lane l = lanesList.at(laneIndex);
			laneLine.addGeometry(&l.line);			
			poFeature->SetGeometryDirectly(laneLine.clone());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Id"), (char)l.id);
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Type"), l.type.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Level"), l.level.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("SingleSide"), l.singleSide.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("RoadID"), l.parentRoadId.c_str());
			laneIndex++;
		}
		//If there are not more availale Lane elements to retrieve we move to the next road
		if (laneIndex >= laneSize) {
			xodrRoadsIte++;
			laneIndex = 0;
		}
		if ((m_poFilterGeom == NULL || FilterGeometry(poFeature->GetGeometryRef()))
			&& (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate(poFeature))) {
			return poFeature;
		}
		delete poFeature;
	}
	return NULL;
}

OGRFeature* OGRXODRLayer::roadMarkFeature() {
	if (xodrRoadsIte != xodrRoads.end()) {
		const road& xodrRoad = *xodrRoadsIte;
		const planView& planView = xodrRoad.planView();
		const lanes& lanes = xodrRoad.lanes();
		OGRMultiLineString  ogrRoad = xodr->toOGRGeometry(planView);

		//This method is called several times, to reduce the execution time,  road marks are only computed once in every road (when the roadMarkIndex is equal to 0)
		if (roadMarkIndex == 0) {
			std::cout << "Processing road mark elements of road " << xodrRoad.id().get() << "\n";
			roadMarkList = xodr->roadMarks(xodrRoad);
			xodr->roadSignals(xodrRoad);
			roadMarksSize = roadMarkList.size();
		}
		OGRFeature* poFeature = new OGRFeature(poFeatureDefn);
		//We check whether there are road marks available, because some roads do not have road marks
		if (roadMarksSize > 0) {
			
			RoadMark r = roadMarkList.at(roadMarkIndex);
			OGRMultiLineString roadMarkLine = r.line;
			poFeature->SetGeometryDirectly(roadMarkLine.clone());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Id"), (char)r.laneId);
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Type"), r.type.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Weight"), r.weight.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Color"), r.color.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("LaneChange"), r.laneChange.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Rule"), r.rule.c_str());
			roadMarkIndex++;
		}
		//If there are not more availale road Mark  elements to retrieve we move to the next road
		if (roadMarkIndex >= roadMarksSize) {
			++xodrRoadsIte;
			roadMarkIndex = 0;
		}
		if ((m_poFilterGeom == NULL || FilterGeometry(poFeature->GetGeometryRef()))
			&& (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate(poFeature))) {
			return poFeature;
		}
		delete poFeature;
	}
	return NULL;
}


OGRFeature* OGRXODRLayer::signalFeature(std::vector<road> signalRoads) {
	if (signalRoadIndex < signalRoadSize) {
		const road& xodrRoad = signalRoads.at(signalRoadIndex);	
		//This method is called several times, to reduce the execution time,  signals are only computed once in every road (when the signal index is equal to 0), and then we retrieve them one by one
		if (signalIndex == 0) {
			std::cout << "Processing signal elements of road " << xodrRoad.id().get() << "\n";
			signalList =  xodr->roadSignals(xodrRoad);
			signalSize = signalList.size();		
			
		}
		OGRFeature* poFeature = new OGRFeature(poFeatureDefn);
		//We check whether there are signals available, because some roads do not have signals
		if (signalSize > 0) {

			Signal s = signalList.at(signalIndex);
			OGRPoint sinalPoint = s.position;
			poFeature->SetGeometryDirectly(sinalPoint.clone());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("RoadId"), s.roadId.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Id"), s.id.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Name"), s.name.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Dynamic"), s.dynamic.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Orient"), s.orientation.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Type"), s.type.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Country"), s.country.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Subtype"), s.subtype.c_str());
			signalIndex++;
		}

		//If there are not more availale signal elements to retrieve we move to the next road
		if (signalIndex >= signalSize) {
			signalRoadIndex++;
			signalIndex = 0;
		}
		if ((m_poFilterGeom == NULL || FilterGeometry(poFeature->GetGeometryRef()))
			&& (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate(poFeature))) {
			return poFeature;
		}
		delete poFeature;
	}
	return NULL;
}



OGRFeature* OGRXODRLayer::objectFeature(std::vector<road> objectRoads) {
	if (objectRoadIndex < objectRoadSize) {
		const road& xodrRoad = objectRoads.at(objectRoadIndex);
		//This method is called several times, to reduce the execution time,  signals are only computed once in every road (when the signal index is equal to 0), and then we retrieve them one by one
		if (objectIndex == 0) {
			std::cout << "Processing object elements of road " << xodrRoad.id().get() << "\n";
			objectList = xodr->roadObject(xodrRoad);
			objectSize = objectList.size();
		}
		OGRFeature* poFeature = new OGRFeature(poFeatureDefn);
		//We check whether there are signals available, because some roads do not have signals
		if (objectSize > 0) {
			Object o = objectList.at(objectIndex);
			OGRPoint objectPoint = o.position;
			poFeature->SetGeometryDirectly(objectPoint.clone());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("RoadId"), o.roadId.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Id"), o.id.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Name"), o.name.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Orient"), o.orientation.c_str());
			poFeature->SetField(poFeatureDefn->GetFieldIndex("Type"), o.type.c_str());

			objectIndex++;
		}
		
		//If there are not more availale signal elements to retrieve we move to the next road
		if (objectIndex >= objectSize) {
			objectRoadIndex++;
			objectIndex = 0;
		}
		if ((m_poFilterGeom == NULL || FilterGeometry(poFeature->GetGeometryRef()))
			&& (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate(poFeature))) {
			return poFeature;
		}
		delete poFeature;
	}
	return NULL;
}


OGRFeature* OGRXODRLayer::objectFeaturePolygon() {
	if (objectRoadIndex < objectRoadSize) {
		const road& xodrRoad = objectRoads.at(objectRoadIndex);
		//This method is called several times, to reduce the execution time,  signals are only computed once in every road (when the signal index is equal to 0), and then we retrieve them one by one
		if (objectIndex == 0) {
			std::cout << "Processing object elements of road " << xodrRoad.id().get() << "\n";
			std::vector<Object> objectListPolygon;
			objectList = xodr->roadObject(xodrRoad);
			for (int i = 0; i < objectList.size(); i++)
			{
				Object o = objectList.at(i);
				if (o.geometryType == "polygon") {
					objectListPolygon.push_back(o);
				}
			}
			objectList = objectListPolygon;
			objectSize = objectList.size();
		}
		OGRFeature* poFeature = new OGRFeature(poFeatureDefn);
		//We check whether there are signals available, because some roads do not have signals
		if (objectSize > 0) {
			Object o = objectList.at(objectIndex);
			if (o.geometryType == "polygon") {
				OGRPolygon objectPolygon = o.shape;
				poFeature->SetGeometryDirectly(objectPolygon.clone());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("RoadId"), o.roadId.c_str());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("Id"), o.id.c_str());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("Name"), o.name.c_str());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("Orient"), o.orientation.c_str());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("Type"), o.type.c_str());

			}
			
			objectIndex++;
		}

		//If there are not more availale signal elements to retrieve we move to the next road
		if (objectIndex >= objectSize) {
			objectRoadIndex++;
			objectIndex = 0;
		}
		if ((m_poFilterGeom == NULL || FilterGeometry(poFeature->GetGeometryRef()))
			&& (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate(poFeature))) {
			return poFeature;
		}
		delete poFeature;
	}
	return NULL;
}



OGRFeature* OGRXODRLayer::objectFeatureLine() {
	if (objectRoadIndex < objectRoadSize) {
		const road& xodrRoad = objectRoads.at(objectRoadIndex);
		//This method is called several times, to reduce the execution time,  signals are only computed once in every road (when the signal index is equal to 0), and then we retrieve them one by one
		if (objectIndex == 0) {
			std::cout << "Processing object elements of road " << xodrRoad.id().get() << "\n";
			std::vector<Object> objectListLine;
			objectList = xodr->roadObject(xodrRoad);
			for (int i = 0; i < objectList.size(); i++)
			{
				Object o = objectList.at(i);
				if (o.geometryType == "line") {
					objectListLine.push_back(o);
				}
			}
			objectList = objectListLine;
			objectSize = objectList.size();
		}
		OGRFeature* poFeature = new OGRFeature(poFeatureDefn);
		//We check whether there are signals available, because some roads do not have signals
		if (objectSize > 0) {
			Object o = objectList.at(objectIndex);
			if (o.geometryType == "line") {
				OGRMultiLineString objectLine= o.line;
				poFeature->SetGeometryDirectly(objectLine.clone());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("RoadId"), o.roadId.c_str());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("Id"), o.id.c_str());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("Name"), o.name.c_str());				
				poFeature->SetField(poFeatureDefn->GetFieldIndex("Orient"), o.orientation.c_str());
				poFeature->SetField(poFeatureDefn->GetFieldIndex("Type"), o.type.c_str());
			}

			objectIndex++;
		}

		//If there are not more availale signal elements to retrieve we move to the next road
		if (objectIndex >= objectSize) {
			objectRoadIndex++;
			objectIndex = 0;
		}
		if ((m_poFilterGeom == NULL || FilterGeometry(poFeature->GetGeometryRef()))
			&& (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate(poFeature))) {
			return poFeature;
		}
		delete poFeature;
	}
	return NULL;
}