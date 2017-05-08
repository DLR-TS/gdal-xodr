# OpenDRIVE Driver for OGR
By extending GDAL/OGR with the ability to read [OpenDRIVE](http://www.opendrive.org/) XML files, a broad and well-established toolset of GIS functions will be made available for OpenDRIVE processing. This OGR extension is described in a scientific paper which has been submitted for the [Driving Simulation Conference 2017 Europe<sup>VR</sup>](http://dsc2017.org/) and will be published in September 2017.

This repository focusses on the development of an OpenDRIVE driver for the OGR [Simple Features](http://www.opengeospatial.org/standards/sfa) Library (OGR). Currently, the still prototypical OpenDRIVE driver is not yet integrated into GDAL but can be built as shared library against GDAL to provide a pluggable extension. For the development of our driver the original GDAL repository has been forked for easier integration into GDAL later.

## Current Functionality
As of OpenDRIVE version 1.4 the specified coordinate reference system, given as PROJ.4 string, is correctly interpreted. For now one `MultiLineString` layer is created containing just the road reference line geometries.

## Further To-Dos
Geometry:
- [ ] Specify point sampling distance for mathematical geometries as layer creation option
- [ ] Create 3D geometries from polynomial OpenDRIVE elevation profile
- [ ] Add `Point` layer for road objects (e.g. signals, signs)
- [ ] Add `Polygon` layer for driving lanes, parking spaces
- [ ] Add additional `LineString` layer(s) to contain
  - [ ] driving lane boundaries
  - [ ] road marks
  - [ ] linear objects (e.g. guardrails, barriers)
- [ ] Implement sampling of `spiral` geometries, with both `curvStart` and `curvEnd` `!= 0`

Misc:
- [ ] Catch invalid XODR file path ("terminate called after throwing an instance of 'xsd::cxx::tree::parsing<char>'")
- [ ] [Insure proper resource deallocation](https://trac.osgeo.org/gdal/wiki/FAQMiscellaneous#HowshouldIdeallocateresourcesacquaintedfromGDALonWindows)

## Building
### 0 Dependencies
The driver works only for GDAL 2.x. and does not support GDAL 1.x. It depends on

- [odrSpiral](https://github.com/DLR-TS/odrSpiral)
- [xodr](https://github.com/DLR-TS/xodr)
- [CodeSynthesis XSD](http://codesynthesis.com/products/xsd/)
- [Xerces-C++](https://xerces.apache.org/xerces-c/)
- [GEOS](https://trac.osgeo.org/geos/) support enabled in the GDAL base library

and building it is divided into

- building the original GDAL base library which we link our shared extension against, followed by
- building the actual OpenDRIVE driver as shared library extension of GDAL/OGR.

We tested on Ubuntu Linux 16.04 x64 and Windows 7 x64. Get started by cloning the [ogr/xodr](https://github.com/DLR-TS/gdal/tree/ogr/xodr) branch of GDAL:
```bash
git clone https://github.com/DLR-TS/gdal.git -b ogr/xodr --single-branch <gdal>
```
If needed, substitute `<gdal>` with the desired path name to clone into. 

### 1 Building on Linux
#### 1.1 GDAL Base on Linux
We basically follow the official [GDAL building instructions for Unix](https://trac.osgeo.org/gdal/wiki/BuildingOnUnix).

Configure GDAL to support creation of shared libraries. At least for our Ubuntu 16.04 test environment we also had to disable libtool because it caused problems during later linking of the driver shared library:
```bash
cd <gdal>/gdal/
./configure --prefix ~/dev/gdal/gdal/build -enable-shared --without-libtool --with-geos=yes
```
For Debug configuration append `--enable-debug` to `./configure`. Build with
```bash
make -f GNUmakefile
```
followed by a 
```bash
make -f GNUmakefile install
```

which copies the resulting GDAL binaries and library  into the abovely specified CMake `--prefix` directory.

#### 1.2 OpenDRIVE Driver as Shared Library on Linux
Navigate into the OpenDRIVE OGR driver directory
```bash
cd <gdal>/gdal/ogr/ogrsf_frmts/xodr/
```
Configure the paths for all required Unix dependencies in `XODRmake.opt`. Then
```bash
make -f GNUmakefile plugin-install
```
This will build a shared library of the plugin and automatically copy it into GDAL's plugin directory relative to the CMake `--prefix` directory specified in the previous section.

### 2 Building on Windows
#### 2.1 GDAL Base on Windows
We basically follow the official [GDAL building instructions for Windows](https://trac.osgeo.org/gdal/wiki/BuildingOnWindows). Things have gotten easier starting from GDAL 2.3.x. where GDAL provides a comfortable script `generate_vcxproj.bat` to generate project definitions for recent Microsoft's Visual Studio editions. An exemplary project for Visual Studio 2015 x64 can be generated from the "VS2015 x64 Native Tools Command Prompt" as follows:
```bash
generate_vcxproj.bat 14.0 64 gdal_vs2015
```
Open the generated `.vcxproj` in Visual Studio and build GDAL for the desired configuration (e.g. Release or Debug). Alternatively, for a Release build use `nmake` from command line:
```bash
cd <gdal>/gdal/
nmake -f makefile.vc MSVC_VER=1900 WIN64=1
```
Lean back, enjoy a freshly brewed Lapsang Souchong and after a few minutes your raw GDAL library is built. To pack all executables and the library conveniently together specify the desired output directory `GDAL_HOME` in a lokal configuration file `nmake.lokal` 
```bash
echo GDAL_HOME="C:\dev\gdal\gdal\build" > nmake.lokal
```
and run `nmake install` afterwards
```bash
nmake -f makefile.vc MSVC_VER=1900 WIN64=1 install
```

#### 2.2 OpenDRIVE Driver as Shared Library on Windows
Navigate into the OpenDRIVE OGR driver directory
```bash
cd <gdal>/gdal/ogr/ogrsf_frmts/xodr/
```
Configure the paths for all required Windows dependencies in `XODRnmake.opt`. Then
```bash
nmake -f makefile.vc MSVC_VER=1900 WIN64=1 plugin-install
```
This also copies the required Xerces DLL into GDAL's binary install directory!

### 3 Testing the OGR OpenDRIVE Driver
If everything went right the built GDAL/OGR is extended by our OpenDRIVE driver and can be tested by running one of the utility programs. Running `ogrinfo` for the above examples would look like

| Linux  		      | Windows                      |
| --------------------------- | ---------------------------- |
| `cd <gdal>/gdal/build/lib/` | `cd <gdal>/gdal/build/bin/`  |
| `../bin/ogrinfo --formats`  | `ogrinfo.exe --formats`	     |

This should yield the an OGR driver list extended by the new **OpenDRIVE** driver. To convert an OpenDRIVE XML file into, e.g., an ESRI Shapefile, use the provided utility `ogr2ogr`:
```bash
ogr2ogr -f "ESRI Shapefile" CulDeSac.shp CulDeSac.xodr
```
OpenDRIVE datasets for testing can be found in the official [Download section](http://opendrive.org/download.html). For advanced debug console output of those utility programs and the implemented drivers add `CPL_DEBUG=ON` to your running environment.
  				
## General Development Notes
To easily add new drivers to GDAL as shared libraries GDAL provides the GDALDriverManager with its [`AutoLoadDrivers()`](http://www.gdal.org/classGDALDriverManager.html#a77417ede570b33695e5b318fbbdb1968) function. The FileGDB driver serves as good orientation for shared library development in GDAL/OGR, see `RegisterOGRFileGDB()` in [`FGdbDriver.cpp`](../filegdb/FGdbDriver.cpp). Also consider the [FileGDB Building Notes](http://www.gdal.org/drv_filegdb.html).

