geocrop
=======

Small tool based on GDAL/PROJ4 to automatic crop box on raster referenced maps (GDAL supported, in most cases - Russians).

Input maps must be with RGB color space: croped box replaces by transparent color.
Output maps also in RGB color space. To convert between indexed pallete and RGB and back you can use:
    pct2rgb.py
and
    rgb2pct.py
tools.

Under debian-based distibutives this scripts present at the `python-gdal` package:
    sudo apt-get install python-gdal

Cropper uses `gdalwarp` command line tool, under debian-based distributives it present at the `gdal-bin` package:
    sudo apt-get install gdal-bin


Build
-----

On Ubuntu/Mint you need to install:
    sudo apt-get install libgdal1-dev libproj-dev cmake

On ArchLinux you need simple packages `gdal`, `proj` and `cmake` installed.

To build:
  - Get & unpack sources
  - Create build directory and goes to it: `mkdir build && cd build`
  - Run cmake: `cmake ..`
  - Simple type: `make`

Installation does not required, you can run application in-place. Or you can "install" it manually:

    sudo cp ./geocrop /usr/local/bin

If `gdal` and/or `proj` includes or libraries not found, double check if all needed development files installed.

Use
---

Simple run without any options to see help.

TBD
