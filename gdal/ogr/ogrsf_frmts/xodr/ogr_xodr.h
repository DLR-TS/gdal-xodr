/******************************************************************************
 * $Id:  $
 *
 * Project:   OpenGIS Simple Features for OpenDRIVE format
 * Purpose:   Implements OGRXODRDriver class.
 * Authors:   Ana Maria Orozco  ana dot orozco at tum dot de
 *			  Michael Scholz    michael dot scholz at dlr dot de
 *            Deutsches Zentrum f�r Luft- und Raumfahrt, DLR
 ******************************************************************************/

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