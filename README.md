geocrop
=======

Small tool based on GDAL/PROJ4 to automatic crop box on GeoTIFF maps (in most cases - Russians).

Input maps must be with RGB color space: croped box replaces by transparent color. 
Output maps also in RGB color space. To convert between indexed pallete and RGB and back you can use:
    pct2rgb.py 
and 
    rgb2pct.py
tools.

Build
-----

On Ubuntu/Mint you need install:
    sudo apt-get install libgdal1-dev libproj-dev

On ArchLinux you need simple packages `gdal` and `proj` installed.

To build:
  - Get & unpack sources
  - Simple type: `make`

Installation does not required, you can run application in-place. Or you can "install" it manually:
    sudo cp ./geocrop /usr/local/bin

On some systems `gdal-config` tool can be not present. You can manually pass params to build:
   make GDAL_INCLUDE=/usr/include/gdal GDAL_LUB="-L/usr/lib/gdal -lgdal"

Use
---

Simple run programm without params to get next help:
    Tool for automatic crop raster maps
    (C) Alexander 'hatred' Drozdov, 2012. Distributed under GPLv2 terms

    Use: ./geocrop <scale> <in geotiff> <croped geotiff>
      Input geotiff MUST be in RGB pallete, so, use pct2rgb.py to convert from indexed
      Output geotiff croped and nodata areas is transparency
      Scale must be:
        100k     for 1:100 000 plates
        50k      for 1:50 000 plates
      1M (1:1 000 000) support by default
      Any other scales currently not support

