/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Implementation of OGRXODRDriver.
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