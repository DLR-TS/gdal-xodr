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
#include "cpl_conv.h"
#include "xodr.h"

OGRXODRDataSource::OGRXODRDataSource() {
    papoLayers = NULL;
    nLayers = 0;
    xodr = NULL;
}

OGRXODRDataSource::~OGRXODRDataSource() {
    for (int i = 0; i < nLayers; i++) {
        delete papoLayers[i];
    }
    CPLFree(papoLayers);
    if (xodr) {
        delete xodr;
    }
}

int OGRXODRDataSource::Open(GDALOpenInfo* openInfo) {
    int bUpdate = openInfo->eAccess == GA_Update;
    if (bUpdate) {
        CPLError(CE_Failure, CPLE_OpenFailed,
                "Update access not supported by the XODR driver.");
        return FALSE;
    }
    xodr = new XODR(openInfo->pszFilename);

    nLayers = 1;
    papoLayers = (OGRXODRLayer **)
            CPLRealloc(papoLayers, sizeof (OGRXODRLayer *) * (nLayers));
    std::string layer1Name = "reference line";
    papoLayers[0] = new OGRXODRLayer(xodr, layer1Name.c_str());
    return TRUE;
}

OGRLayer *OGRXODRDataSource::GetLayer(int iLayer) {
    if (iLayer < 0 || iLayer >= nLayers)
        return NULL;
    else
        return papoLayers[iLayer];
}

int OGRXODRDataSource::TestCapability(const char * capability) {
    if (EQUAL(capability, ODsCCreateLayer))
        return TRUE;
    else
        return FALSE;
}