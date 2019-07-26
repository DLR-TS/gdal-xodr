.. _vector.xodr:

XODR – OpenDRIVE Road Description Format
========================================

.. shortname:: XODR

`OpenDRIVE <http://opendrive.org/>`_ is an open format for the logical description of road networks. Over the last decade it has been widely used by transportation and automobile companies around the world, thus, becoming a de facto standard for driving simulation applications. OpenDRIVE files are stored using the Extensible Markup Language XML. These files describe roads as a collection of nodes associated to a particular category such as reference line, lane, feature and controller.

With the XODR library OGR features support for reading of certain OpenDRIVE elements. *XODR is especially useful for converting OpenDRIVE files into common GIS formats such as ESRI Shapefiles*.

At this stage, the version supported by the XODR driver is OpenDRIVE 1.4.

Supported OpenDRIVE elements
----------------------------

The XODR driver supports reading/sampling of geometrical OpenDRIVE elements into 2-dimensional geometry types. These designated OpenDRIVE elements also serve as **layer creation options**:

* *referenceLine*: Retrieves a road's reference line ``<planView>`` geometries as wkbMultiLineString.
* *lane*: Retrieves a road lane's outer border snippets as a wkbMultiLineString. The driver supports both, lanes defined through ``<width>`` or ``<border>`` elements.
* *roadMark*: Retrieves a road's marks as wkbMultiLineString.
* *signal*: Retrieves a road's signals as wkbPoint.
* *objectPoint*: Retrieves a road's point objects as wkbPoint.
* *objectLine*: Retrieves a road's linear objects as wkbLineString.
* *objectPolygon*: Retrieves a road's polygonal objects as wkbPolygon. 

Further information about these elements can be found in the `format specification 1.4H <http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf>`_.

These OpenDRIVE element classes are converted into individual OGR layers. Hence, the driver is capable to retrieve not only their geometries but also their attributes. In OpenDRIVE all the elements are related to the road's ``referenceLine`` record. Therefore, in addition to their native attributes, each layer has a ``roadId`` attribute (foreign key to the parent ``referenceLine``).

All coordinates are relative to the reference system defined by a PROJ.4 string inside the XML header as shown in this example:
::

  <header revMajor="1" revMinor="4"
       name="fancy dataset" 
       date="12-12-2012" 
       north="5.435122298357e+06" 
       east="5.435122298357e+06" 
       south="5.435122298357e+06" 
       west="5.435122298357e+06" 
       vendor="We are the greatest. Period.">
    <geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=9 +k=0.9996 +x_0=500000 +y_0=0 +datum=WGS84 +units=m +no_defs]]></geoReference>
  </header>

Example
-------

The ogr2ogr utility can be used to extract OpenDRIVE elments to ESRI Shapefiles:

* *ogr2ogr -f "ESRI Shapefile" roadNetworkPointObjects.shp roadNetwork.xodr objectPoint*

  The command line shown above parses an OpenDRIVE XML file called ``roadNetwork.xodr``, extracts the road objects whose geometry is of the type point and writes them into the ``roadNetworkPointObjects.shp`` Shapefile. The OpenDRIVE element class which is to be extracted is specified as layer creation option and defined as the last parameter of the function call, which in this case is *objectPoint*.

* *ogr2ogr -f "ESRI Shapefile" roadMarks.shp germanRoad.xodr roadMark*

  The command line shown above parses the file ``germanRoad.xodr``, extracts its road marks and writes them into a Shapefile called ``roadMarks.shp``.

See also
--------

* Home page for OpenDRIVE format: http://www.opendrive.org/
* OGR/XODR GitHub repository: https://github.com/DLR-TS/xodr
