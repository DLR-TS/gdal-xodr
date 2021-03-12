# OpenDRIVE Driver for OGR
This repository focusses on the development of an OpenDRIVE driver for the OGR [Simple Features](http://www.opengeospatial.org/standards/sfa) Library. By extending GDAL/OGR with the ability to read [OpenDRIVE](http://www.opendrive.org/) XML files, a broad and well-established toolset of GIS functions will be made available for OpenDRIVE processing. This OGR extension is based on the work by [Orozco Idrobo (2015)][@OrozcoIdrobo2015] and [Scholz et al. (2017)][@Scholz2017]. Currently, the still prototypical OpenDRIVE driver is not yet integrated into the official GDAL distribution. Steps to built the driver as a shared library against GDAL to provide a pluggable extension are described below.

## Current Functionality
With OpenDRIVE version 1.4 the specified coordinate reference system - given as PROJ.4 string - is correctly interpreted during geometry creation. Different `MultiLineString`, `Point` and `Polygon` layers are created for the desired element output. Details on driver capabilities can be found in the [driver documentation](https://github.com/DLR-TS/gdal/blob/ogr/xodr/gdal/doc/source/drivers/vector/xodr.rst).

## Further To-Dos
Geometry:
- [ ] Specify point sampling distance for geometries as layer creation option
- [ ] Create 3D geometries from polynomial OpenDRIVE elevation profile
- [x] Add `Point` layer for road objects (e.g. signals, signs)
- [x] Add `Polygon` layer for parking spaces
- [ ] Add `Polygon` layer for driving lanes
- [x] Add additional `LineString` layer(s) to contain
  - [x] driving lane boundaries
  - [x] road marks
  - [x] linear objects (e.g. guardrails, barriers)
- [ ] Implement sampling of `spiral` geometries, with both `curvStart` and `curvEnd` `!= 0`

Misc:
- [x] Catch invalid XODR file path ("terminate called after throwing an instance of 'xsd::cxx::tree::parsing<char>'")
- [ ] [Ensure proper resource deallocation](https://trac.osgeo.org/gdal/wiki/FAQMiscellaneous#HowshouldIdeallocateresourcesacquaintedfromGDALonWindows)

## Building
We tested on Ubuntu Linux 20.04 x64 (gcc 9.3.0) and Windows 10 x64 (Visual Studio 15 2017). Get started by cloning our [ogr/xodr](https://github.com/DLR-TS/gdal/tree/ogr/xodr) branch:
```bash
git clone https://github.com/DLR-TS/gdal.git -b ogr/xodr --single-branch <gdal>
```
If needed, substitute `<gdal>` with the desired path name to clone into. 

### 0 Dependencies
The XODR driver is tested to work for GDAL 3.x. It depends on the following libraries:

- [odrSpiral](https://github.com/DLR-TS/odrSpiral)
- [xodr](https://github.com/DLR-TS/xodr)
- [GEOS](https://trac.osgeo.org/geos/); on Windows we recommend using the version distributed with [OSGeo4W](https://trac.osgeo.org/osgeo4w/)
- [PROJ >= 6](https://proj.org/download.html#current-release)
- [SQLite3](https://www.sqlite.org/index.html); on Windows we recommend using the version distributed with [OSGeo4W](https://trac.osgeo.org/osgeo4w/)

Building it is divided into

- building the original GDAL base library which we link our shared extension against, followed by
- building the actual OpenDRIVE driver as shared library extension of GDAL/OGR.

### 1 Building on Linux
#### 1.1 GDAL Base on Linux
Install dependencies:

```
sudo apt install libgeos-dev
sudo apt install libproj-dev
sudo apt install libsqlite3-dev
```

In the following steps we basically follow the official [GDAL building instructions for Unix](https://trac.osgeo.org/gdal/wiki/BuildingOnUnix). Configure GDAL to support creation of shared libraries. At least for our Ubuntu 20.04 test environment we also had to disable libtool because it caused problems during later linking of the driver shared library. With `--prefix` you specify the target directory for the resulting binaries and library:

```bash
cd <gdal>/gdal/
./configure --prefix ~/dev/gdal/gdal/build -enable-shared --without-libtool --with-geos=yes
```

Check the output for successful recognition of geos and xerces. For Debug configuration append `--enable-debug`. Build with

```bash
make -f GNUmakefile
```

followed by a 

```bash
make -f GNUmakefile install
```

which copies the resulting GDAL binaries and library into the specified `--prefix` directory.

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
We basically follow the official [GDAL building instructions for Windows](https://trac.osgeo.org/gdal/wiki/BuildingOnWindows). Things have gotten easier starting from GDAL 2.3.x. where GDAL provides a comfortable script `generate_vcxproj.bat` to generate project definitions for recent Microsoft's Visual Studio editions. An exemplary project for Visual Studio 2017 x64 can be generated from the "VS2017 x64 Native Tools Command Prompt" as follows:

```bash
generate_vcxproj.bat 15.0 64 gdal_vs2017
```

Now configure your GEOS and Xerces dependencies by adding the corresponding include directory and library paths into a _new_ lokal NMake configuration file `nmake.local`. It should contain something like the following (consider `nmake.opt` as a reference):

```bash
GDAL_HOME = "D:\dev\gdal\gdal\build"

OSGEO4W64_DIR = C:\OSGeo4W64

# GEOS
#GEOS_DIR = D:\dev\geos\distro
GEOS_DIR = $(OSGEO4W64_DIR)
GEOS_CFLAGS = -I$(GEOS_DIR)\include -DHAVE_GEOS
GEOS_LIB = $(GEOS_DIR)\lib\geos_c.lib

# Xerces
XERCES_DIR = D:\dev\xerces-c-3.2.3\distro
XERCES_INCLUDE = -I$(XERCES_DIR)\include -I$(XERCES_DIR)\include\xercesc
!IFNDEF DEBUG
XERCES_LIB = $(XERCES_DIR)\lib\xerces-c_3.lib
XERCES_DLL = $(XERCES_DIR)\bin\xerces-c_3_2.dll
!ELSE
XERCES_LIB = $(XERCES_DIR)\lib\xerces-c_3D.lib
XERCES_DLL = $(XERCES_DIR)\bin\xerces-c_3_2D.dll
!ENDIF

# PROJ >= 6
PROJ_INCLUDE = -ID:\dev\proj-6.3.2\distro\include
!IFNDEF DEBUG
PROJ_LIBRARY = D:\dev\proj-6.3.2\distro\lib\proj.lib
!ELSE
PROJ_LIBRARY = D:\dev\proj-6.3.2\distro\lib\proj_d.lib
!ENDIF

# SQLite3
SQLITE_INC = -I$(OSGEO4W64_DIR)\include
SQLITE_LIB = $(OSGEO4W64_DIR)\lib\sqlite3_i.lib
```

Build the base GDAL library with `nmake` from the Visual Studio Command Prompt for the desired configuration. Attach `DEBUG=1` for a Debug build:

```bash
cd <gdal>/gdal/
nmake -f makefile.vc MSVC_VER=1910 WIN64=1
```

Lean back, enjoy a freshly brewed Lapsang Souchong and after a few minutes your raw GDAL library is built. To pack all executables and the library conveniently together, make sure to specify the desired output directory `GDAL_HOME` in your lokal configuration file `nmake.local`

```bash
GDAL_HOME="D:\dev\gdal\gdal\build"
```

and run `nmake install` afterwards

```bash
nmake -f makefile.vc MSVC_VER=1910 WIN64=1 install
```

#### 2.2 OpenDRIVE Driver as Shared Library on Windows
Navigate into the OpenDRIVE OGR driver directory

```bash
cd <gdal>/gdal/ogr/ogrsf_frmts/xodr/
```

Configure the paths for all required Windows dependencies in the provided `XODRnmake.opt`. Then

```bash
nmake -f makefile.vc MSVC_VER=1910 WIN64=1 plugin-install
```

This also copies the required Xerces DLL into GDAL's binary install directory.

### 3 Testing the OGR OpenDRIVE Driver
GDAL/OGR is now extended by our OpenDRIVE driver which can be tested by running one of the utility programs, for example `ogrinfo`:

| Linux  		                  | Windows                      |
| --------------------------- | ---------------------------- |
| `cd <gdal>/gdal/build/bin/` | `cd <gdal>/gdal/build/bin/`  |
| `./ogrinfo --formats`       | `ogrinfo.exe --formats`	     |

This will print a list of supported OGR formats with the new format **`XODR -vector- (ro): OpenDRIVE`** in the first row. 

If you are on Linux, depending on your environment you might experience linker errors like: 

```
error while loading shared libraries: libgdal.so: cannot open shared object file: No such file or directory
```

In such a case set the following environment variables:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<gdal>/gdal/build/lib
export GDAL_DRIVER_PATH=<gdal>/gdal/build/lib/gdalplugins
```

To convert an OpenDRIVE XML file into, e.g., an ESRI Shapefile, use the provided utility `ogr2ogr`:

```bash
ogr2ogr -f "ESRI Shapefile" CulDeSac.shp CulDeSac.xodr referenceLine
```

Details on driver capabilities can be found in the [driver documentation](https://github.com/DLR-TS/gdal/blob/ogr/xodr/gdal/doc/source/drivers/vector/xodr.rst). For advanced debug console output of those utility programs and the implemented drivers add `CPL_DEBUG=ON` to your running environment.
  				
## General Development Notes
To easily add new drivers to GDAL as shared libraries GDAL provides the GDALDriverManager with its [`AutoLoadDrivers()`](https://gdal.org/doxygen/classGDALDriverManager.html#a77417ede570b33695e5b318fbbdb1968) function. The FileGDB driver serves as good orientation for shared library development in GDAL/OGR, see `RegisterOGRFileGDB()` in [`FGdbDriver.cpp`](../filegdb/FGdbDriver.cpp). Also consider the [https://gdal.org/drivers/vector/filegdb.html#building-notes).

[@OrozcoIdrobo2015]: http://elib.dlr.de/103827/ "Orozco Idrobo, Ana Maria (2015). Extension of the Geospatial Data Abstraction Library (GDAL/OGR) for OpenDRIVE Support in GIS Applications for Visualisation and Data Accumulation for Driving Simulators. Master's thesis, Technical University of Munich."
[@Scholz2017]: http://elib.dlr.de/110123/ "Scholz, Michael and Orozco Idrobo, Ana Maria (2017). Supporting the Implementation of Driving Simulator Environments Through Established GIS Approaches by Extending the Geospatial Data Abstraction Library (GDAL) with OpenDRIVE. In: Proceedings of the Driving Simulator Conference 2017 Europe VR, pp. 51-54. Driving Simulation Conference 2017, Stuttgart, Germany."