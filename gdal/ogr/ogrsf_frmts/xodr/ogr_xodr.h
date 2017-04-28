/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Definition of OGR driver components for OpenDRIVE.
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

#ifndef _OGR_XODR_H_INCLUDED
#define _OGR_XODR_H_INCLUDED

#include "ogrsf_frmts.h"
#include "ogr_api.h"
#include "xodr.h"
#include <vector>

/************************************************************************/
/*                           OGRXODRLayer                               */

/************************************************************************/

class OGRXODRLayer : public OGRLayer {
protected:
    XODR* xodr;
    const OpenDRIVE::road_sequence& xodrRoads;
    OpenDRIVE::road_const_iterator xodrRoadsIte;
    OGRFeatureDefn* poFeatureDefn;
    OGRSpatialReference *poSRS;
//    std::vector<OGRMultiLineString>::iterator roadsIte;
    int nNextFID;
public:
    /**
     * C-tor.
     * @param pszName Layer name.
     */
    OGRXODRLayer(XODR* xodr, const char* pszName);

    /**
     * D-tor.
     */
    ~OGRXODRLayer();

    void ResetReading();

    /**
     * Gets the next feature of this OGR layer during feature iteration.
     * @return The next feature.
     */
    OGRFeature * GetNextFeature();

    OGRFeatureDefn * GetLayerDefn() {
        return poFeatureDefn;
    }

    /**
     * Tests for certain OGR capabilities of this layer.
     * @param The capability to test for.
     * @return True if requested capability is supported.
     */
    int TestCapability(const char *) {
        return FALSE;
    }
};

/************************************************************************/
/*                           OGRXODRDataSource                          */
/************************************************************************/

/**
 * The actual GDAL/OGR datasource. In former GDAL versions it was extending 
 * OGRDataSource instead of GDALDataset.
 */
class OGRXODRDataSource : public GDALDataset {
    OGRXODRLayer **papoLayers;
    int nLayers;
    XODR* xodr;

public:
    /**
     * C-tor.
     */
    OGRXODRDataSource();

    /**
     * D-tor.
     */
    ~OGRXODRDataSource();

    int Open(GDALOpenInfo* poOpenInfo);

    int GetLayerCount() {
        return nLayers;
    }

    OGRLayer* GetLayer(int);

    /**
     * Tests for certain OGR capabilities of this datasource.
     * @param The capability to test for.
     * @return True if requested capability is supported.
     */
    int TestCapability(const char *);
};

#endif /* ndef _OGR_XODR_H_INCLUDED */