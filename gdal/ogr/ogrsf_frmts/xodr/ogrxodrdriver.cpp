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
#include "cpl_conv.h"
#include "cpl_error.h"

extern "C" void CPL_DLL RegisterOGRXODR();

/**
 * Identify compatible driver by checking if input file (extension) is supported
 * by this driver.
 * @param openInfo Input file information.
 * @return True if input file is supported by this driver.
 */
static int OGRXODRDriverIdentify(GDALOpenInfo* openInfo) {
    // -------------------------------------------------------------------- 
    //      Does this appear to be an .xodr file?                           
    // --------------------------------------------------------------------
    return EQUAL(CPLGetExtension(openInfo->pszFilename), "xodr");
}

/**
 * Create a dataset/datasource associated with this driver.
 * @param openInfo Input file information.
 * @return Dataset for the input file.
 */
static GDALDataset* OGRXODRDriverOpen(GDALOpenInfo* openInfo) {
    if (!OGRXODRDriverIdentify(openInfo))
        return NULL;

    OGRXODRDataSource* ds = new OGRXODRDataSource();
    if (!ds->Open(openInfo)) {
        delete ds;
        return NULL;
    } else {
        return ds;
    }
}

/**
 * Register this driver to the GDAL/OGR library. This method is automatically 
 * searched for when dynamically linking this driver as an OGR plugin.
 */
 void RegisterOGRXODR() {
    GDALDriver *driver;

    if (GDALGetDriverByName("XODR") == NULL) {
        driver = new GDALDriver();

        driver->SetDescription("XODR");
        driver->SetMetadataItem(GDAL_DCAP_VECTOR, "YES");
        driver->SetMetadataItem(GDAL_DMD_LONGNAME, "OpenDRIVE");
        driver->SetMetadataItem(GDAL_DMD_EXTENSION, "xodr");
        driver->SetMetadataItem(GDAL_DMD_HELPTOPIC, "drv_xodr.html");

        // Function pointers for compatibility check and datasource opening
        driver->pfnIdentify = OGRXODRDriverIdentify;
        driver->pfnOpen = OGRXODRDriverOpen;

        GetGDALDriverManager()->RegisterDriver(driver);
    }
}