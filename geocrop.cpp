#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <cassert>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <proj.h>

#include "version.h"

using namespace std;

static const char * const s_options = "f:s:Sw:ngp:Vh";

struct GeoTransformMatrix
{
    std::array<double, 6> matrix;

    double* get()
    {
        return matrix.data();
    }

    double pixelWidth() const
    {
        return matrix[1];
    }

    double pixelHeight() const
    {
        return matrix[5];
    }

    double left() const
    {
        return matrix[0];
    }

    double top() const
    {
        return matrix[3];
    }

    // TODO
    double rtnX() const
    {
        return matrix[2];
    }

    // TODO
    double rtnY() const
    {
        return matrix[4];
    }


    void pixelToCoordinate(int    rasterX, int    rasterY,
                           double &geoX,   double &geoY)
    {
        geoX = left() + rasterX * pixelWidth() + rasterY * rtnY();
        geoY = top()  + rasterY * pixelHeight() + rasterX * rtnX();
    }

    void coordinateToPixel(double geoX,     double geoY,
                           int    &rasterX, int    &rasterY)
    {
        // TODO: optimize
        rasterX = (geoX - left() - rtnY()/pixelHeight()*geoY + top()*rtnY()/pixelHeight()) / (pixelWidth() - rtnX()*rtnY()/pixelHeight());
        rasterY = (geoY - top() - rasterX * rtnX()) / pixelHeight();
    }
};

void version()
{
    cout << "version: " << GEOCROP_VERSION << '\n';
}

void usage(char *name)
{
    cout << "Tool for automatic crop raster maps\n";
    cout << "(C) Alexander 'hatred' Drozdov, 2012-2017. Distributed under GPLv2 terms\n";
    version();
    cout << "\nUse: " << name << " [args] <in geodata> [<croped geodata>] [-- [optional gdalwarp arguments]]\n"
         << "  Input geotiff MUST be in RGB pallete, so, use pct2rgb.py to convert from indexed\n"
            "  Output geotiff croped and nodata areas is transparency\n"
            "\n"
            "  Args:\n"
            "  -s SCALE\n"
            "     Scale must be:\n"
            "     500k     for 1:500 000 plates\n"
            "     200k     for 1:200 000 plates\n"
            "     100k(*)  for 1:100 000 plates, used by default\n"
            "     50k      for 1:50 000 plates\n"
            "     25k      for 1:25 000 plates\n"
            "     1M (1:1 000 000) used by default\n"
            "     Any other scales currently not support\n"
            "  -S turn off auto-detection for doubled plates (60-76 degrees) and quadruple\n"
            "     plates (over 76 degree)\n"
            "  -f OUTPUT_FORMAT\n"
            "     Any format supported by GDAL. GTiff is default one\n"
            "  -n\n"
            "     Disable '-crop_to_cutline' option. Border will be only filled with transparent color\n"
            "  -w GDALWARP\n"
            "     Allow to override `gdalwarp` name and path, by default simple `gdalwarp` and search under PATH\n"
            "  -g\n"
            "     Only generate CVS description for the `cutline` on `stdout` without `gdalwarp` execution.\n"
            "     Output files with this option will be ignored and can be omited as arguments.\n"
            "  -p srs_def\n"
            "     Override projection for dataset\n"
            "  -V\n"
            "     Output version and exit.\n"
            "  -h\n"
            "     This screen.\n"
         ;
}

string generate_temp_file_name()
{
    char name[] = "/tmp/geocrop.XXXXXX";
    close(mkstemp(name));
    string result(name);
    return result;
}


//
// [RU]
// Определяем подлист листа с указанным обрамлением. Для листов делящихся последовательно на 4
// Наприме, лист километровки делится на 4 листа полукилометровок, если нам нужно получить
// обрамление листа полукилометровки, расчитываем оное для километровки, передаём в качестве входных
// параметров сюда а так же пробную точку, по которой определяем подлист.
//
// Далее, так же последовательно можно сделать расчёт для двухсотпятидесятиметровки
//
char get_subplate(double lon,
                  double lat,
                  double &subPlateTop,
                  double &subPlateLeft,
                  double &subPlateBottom,
                  double &subPlateRight)
{
    double centerLon = subPlateLeft + (subPlateRight - subPlateLeft) / 2;
    double centerLat = subPlateBottom + (subPlateTop - subPlateBottom) / 2;
    char   subPlateCh = 0;

    if (lon >= subPlateLeft && lon < centerLon)
    {
        // 1-3 четверти
        subPlateRight = centerLon;

        if (lat >= subPlateBottom && lat < centerLat)
        {
            subPlateCh = 'C'; // 3 четверть
            subPlateTop = centerLat;
        }
        else if (lat >= centerLat && lat <= subPlateTop)
        {
            subPlateCh = 'A'; // 1 четверть
            subPlateBottom = centerLat;
        }
    }
    else if (lon >= centerLon && lon <= subPlateRight)
    {
        // 2-4 четверти
        subPlateLeft = centerLon;

        if (lat >= subPlateBottom && lat < centerLat)
        {
            subPlateCh = 'D'; // 3 четверть
            subPlateTop = centerLat;
        }
        else if (lat >= centerLat && lat <= subPlateTop)
        {
            subPlateCh = 'B'; // 1 четверть
            subPlateBottom = centerLat;
        }
    }

    return subPlateCh;
}

bool load_projection(const char* projfilename, GDALDataset *dataset)
{
    ifstream ifs(projfilename);
    if (ifs)
    {
        string wktprojection;
        string line;
        while (getline(ifs, line))
        {
            wktprojection += line;
        }

        return dataset->SetProjection(wktprojection.c_str()) == CE_None;
    }
    return false;
}

template<typename T>
struct Point
{
    Point() : x(0.0), y(0.0) {}
    Point(T x, T y) : x(x), y(y) {}

    T x;
    T y;
};

typedef Point<double> PointReal;
typedef Point<int>    PointInt;

/** Adds a " +type=crs" suffix to a PROJ string (if it is a PROJ string) */
static std::string pj_add_type_crs_if_needed(std::string_view str)
{
    std::string ret(str);
    if( (str.starts_with("proj=") ||
         str.starts_with("+proj=") ||
         str.starts_with("+init=") ||
         str.starts_with("+title=")) &&
        str.find("type=crs") == std::string::npos )
    {
        ret += " +type=crs";
    }
    return ret;
}

class CrsInstance
{
public:
    CrsInstance();
    explicit CrsInstance(std::string_view definition);

    bool isLatLong() const;
    bool isLatFirst() const;
    double toRadians() const;

    std::string_view definition() const;

    operator bool() const;
    PJ* get() const;

    void reset();
    PJ* release();

    CrsInstance getGeographicCrs() const;

private:
    static void deleter(PJ *ptr)
    {
        if (ptr)
            proj_destroy(ptr);
    }
    using pj_ptr_t = std::unique_ptr<PJ, decltype(&deleter)>;

    static pj_ptr_t wrap(PJ *pj);

    pj_ptr_t m_crs;
    std::string m_definition;

    bool m_isLongLat = false;
    bool m_isLatFirst = false;
    double m_toRadians = 0.0;
};

//
// http://www.gdal.org/gdal_tutorial_ru.html
//
int main(int argc, char **argv)
{
    GDALDataset *dataset = nullptr;

    bool         autodetectDoubles = true;
    int          mapsCountMult = 1;

    string       scale = "100k";
    string       inFileName;
    string       ouFileName;
    string       prjFileName;
    string       ouDriver = "GTiff";

    GeoTransformMatrix geoTransform;
    int          rasterXSize = 0;
    int          rasterYSize = 0;
    int          rasterXCenter = 0;
    int          rasterYCenter = 0;

    string        gdalwarpPath = "gdalwarp";
    vector<char*> gdalwarpOpts; // Additional options for gdalwarp

    bool cropToCutline = true;
    bool generateOnly  = false;

    while (true)
    {
        int opt = getopt(argc, argv, s_options);
        if (opt == -1)
            break;
        switch (opt)
        {
            case 'f':
                ouDriver = optarg;
                break;
            case 's':
                scale = optarg;
                break;
            case 'S':
                autodetectDoubles = false;
                break;
            case 'w':
                gdalwarpPath = optarg;
                break;
            case 'n':
                cropToCutline = false;
                break;
            case 'g':
                generateOnly = true;
                break;
            case 'p':
                prjFileName = optarg;
                break;
            case 'V':
                version();
                return 0;
            default:
                usage(argv[0]);
                return opt == 'h' ? 0 : 1;
        }
    }

    if (optind > argc - (generateOnly ? 1 : 2))
    {
        usage(argv[0]);
        return 1;
    }

    inFileName = argv[optind++];
    if (!generateOnly)
        ouFileName = argv[optind++];

    if (optind < argc)
    {
        gdalwarpOpts.reserve(argc - optind);
        while (optind++ < argc)
        {
            gdalwarpOpts.push_back(argv[optind - 1]);
        }
    }

    GDALAllRegister();

    dataset = (GDALDataset*)GDALOpen(inFileName.c_str(), GA_ReadOnly);
    if (dataset == 0)
    {
        cerr << "Can't open file: " << inFileName << endl;
        return 1;
    }

    clog << "InDriver: " << dataset->GetDriver()->GetDescription() << "/"
         << dataset->GetDriver()->GetMetadata(GDAL_DMD_LONGNAME) << endl;

    clog << "InSize: " << dataset->GetRasterXSize() << "x" << dataset->GetRasterYSize() << "x"
         << dataset->GetRasterCount() << endl;

    // Override or detect projection
    if (!prjFileName.empty())
    {
        clog << "Override projection with: '" << prjFileName << "'\n";
        if (!load_projection(prjFileName.c_str(), dataset))
        {
            cerr << "Can't load projection\n";
            return 1;
        }

        gdalwarpOpts.push_back(::strdup("-s_srs"));
        gdalwarpOpts.push_back(::strdup(prjFileName.c_str()));
    }
    else if (dataset->GetProjectionRef() == nullptr || *dataset->GetProjectionRef() == '\0')
    {
        clog << "Empty projection...\n";
        for (auto ext : {"prj", "prf"})
        {
            auto projfilename = CPLResetExtension(inFileName.c_str(), ext);
            clog << "  Try load from the '" << projfilename << "'\n";
            if (load_projection(projfilename, dataset))
            {
                gdalwarpOpts.push_back(::strdup("-s_srs"));
                gdalwarpOpts.push_back(::strdup(projfilename));
                break;
            }
        }
    }

    if (dataset->GetProjectionRef() != nullptr)
    {
        clog << "Projection: " << dataset->GetProjectionRef() << endl;
    }

    if (dataset->GetGeoTransform(geoTransform.get()) == CE_None)
    {
        fprintf(stderr,
                "Coordinate zero (%.6f,%.6f)\n",
                geoTransform.left(), geoTransform.top() );

        fprintf(stderr,
                "Pixel size (%.6f,%.6f)\n",
                geoTransform.pixelWidth(), geoTransform.pixelHeight() );

        rasterXSize = dataset->GetRasterXSize();
        rasterYSize = dataset->GetRasterYSize();
        rasterXCenter = rasterXSize / 2;
        rasterYCenter = rasterYSize / 2;

        vector<PointReal> shapeBorderCoordinates(4);
        vector<PointInt>  shapeBorderPixels(4);

        double geoXCenter;
        double geoYCenter;

        geoTransform.pixelToCoordinate(rasterXCenter, rasterYCenter, geoXCenter, geoYCenter);

        fprintf(stderr, "Shape center: (%5d, %5d)\n", rasterXCenter, rasterYCenter);
        fprintf(stderr, "Shape center coordinates: (%.6f, %.6f)\n", geoXCenter, geoYCenter);

        // Back conversion check
        //geoToPixelCoordinate(geoTransform, geoXCenter, geoYCenter, rasterXCenter, rasterYCenter);
        //printf("Центр листа: (%5d, %5d)\n", rasterXCenter, rasterYCenter);

        // Convert WKT projection representation to the PRIJ4 form
        OGRSpatialReference inSrs;
        inSrs.importFromWkt(dataset->GetProjectionRef());
        char *proj4Ref;
        inSrs.exportToProj4(&proj4Ref);
        string inProj = proj4Ref;

        clog << "PROJ4: " << inProj << endl;

        // Recalc metric coordinates to the LonLat for shape detection
        CrsInstance pjSrc{inProj};
        PJ *transform = nullptr;

        double geoLonCenter = geoXCenter;
        double geoLatCenter = geoYCenter;

        if (!pjSrc.isLatLong())
        {
            auto pjTar = pjSrc.getGeographicCrs();

            clog << "PROJ: " << pjTar.definition() << endl;

            if (!pjSrc|| !pjTar)
            {
                cerr << "Can't setup projection'\n";
                return 1;
            }

            transform = proj_create_crs_to_crs_from_pj(PJ_DEFAULT_CTX,
                                                       pjSrc.get(), pjTar.get(),
                                                       nullptr, nullptr);
            if (!transform) {
                clog << "Can't create PROJ transformation\n";
                return 1;
            }

            PJ_COORD coord{};
            coord.xy.x = geoXCenter;
            coord.xy.y = geoYCenter;
            coord = proj_trans(transform, PJ_FWD, coord);
            geoLonCenter = coord.uv.u;
            geoLatCenter = coord.uv.v;

            fprintf(stderr, "Shape center coordinates (lat/lon): (%.6f, %.6f)\n",
                   geoLatCenter, geoLonCenter);
        }

        //
        // Detect 1M shape
        //

        // Detect shape letter (in other word: shape latitude)
        int letterNumber = (int)ceil(geoLatCenter / 4);
        // Degree border of the corresponding 1M map
        double top1m    = letterNumber * 4;
        double bottom1m = top1m - 4;

        // Detect double/quad maps
        if (autodetectDoubles) {
            // geoLatCenter, geoLonCenter
            // To detect doublding/quad maps we should use only geoLatCenter
            // it allows us correctly detect shape

            // TODO: add support for sourth
            if (bottom1m >= 60.0) {
                mapsCountMult = 2; // doubled maps
            }

            if (bottom1m >= 76.0) {
                mapsCountMult = 4; // quad maps
            }
        }

        fprintf(stderr, "Shape bottom: %f, assume %d maps in Lon direction\n", bottom1m, mapsCountMult);

        // Zone depends on shapes count
        int Nz = (int)ceil(geoLonCenter / (6 * mapsCountMult));
        double right1m  = Nz * (6 * mapsCountMult);
        double left1m   = right1m - (6 * mapsCountMult);

        shapeBorderCoordinates[0] = PointReal(top1m, left1m);
        shapeBorderCoordinates[1] = PointReal(top1m, right1m);
        shapeBorderCoordinates[2] = PointReal(bottom1m, left1m);
        shapeBorderCoordinates[3] = PointReal(bottom1m, right1m);

        for (int i = 0; i < 4; ++i)
        {
            if (transform)
            {
                // At the LatLon representation, first is a latitude that is a Y. Second -
                // longitude that is X. Before pass it to the pj_transform() we must put to
                // correct places:
                //    first - X (lon), second - Y (lat)
                PJ_COORD coord{};
                coord.uv.u = shapeBorderCoordinates[i].y;
                coord.uv.v = shapeBorderCoordinates[i].x;
                coord = proj_trans(transform, PJ_INV, coord);
                shapeBorderCoordinates[i].x = coord.xy.x;
                shapeBorderCoordinates[i].y = coord.xy.y;
            }

            // Recalc coordinates to the pixels
            geoTransform.coordinateToPixel(shapeBorderCoordinates[i].x, shapeBorderCoordinates[i].y, shapeBorderPixels[i].x, shapeBorderPixels[i].y);
        }

        char plateCh = 'A' + letterNumber - 1;              // Shape letter
        int  plateNum = Nz*mapsCountMult + 30;              // Shape number

        stringstream plateNameStream;
        plateNameStream << plateCh << "-" << plateNum;
        string plateName = plateNameStream.str();
#if 0
        plateName.push_back(plateCh);

        switch (mapsCountMult) {
            case 1:
                plateName = plateNameStream.str();
                break;
            case 2:
            case 4:
            {
                int first = plateNum > 1 ? plateNum / mapsCountMult * mapsCountMult - 1 : 1;
                plateName.append("-");

                for (int i = 0; i < mapsCountMult; ++i) {
                    plateName.append(to_string(first + i));
                    if (i != mapsCountMult - 1)
                        plateName.append(",");
                }
                break;
            }
        }
#endif

        clog << "Shape of the 1M map: " << plateName << endl;
        fprintf(stderr,
                "Shape border coordinates (lat/lon) %s:\n"
                "   (%.6f, %.6f)   (%.6f, %.6f)\n"
                "   (%.6f, %.6f)   (%.6f, %.6f)\n",
                plateName.c_str(),
                top1m, left1m, top1m, right1m,
                bottom1m, left1m, bottom1m, right1m);


        int tmpx = 1;   // shape row number at the 1M base map, from 0 left-to-right
        int tmpy = 1;   // shape column number at the 1M base map, from 0 top-to-bottm
        int plateSubNum = 1; // linear shape number at the 1M base map, valid for 500k, 200k and 100k subsets
                             // more huge scales bases on 100k settings
                             // for 500k subbser, linear number is a letter 'А'-'Г' on Russian or
                             // 'A'-'D', like O-50-A
        double plateWidthMin  = 360 * mapsCountMult; // Shape width in minutes, 360 - for 1M map
        double plateHeightMin = 240; // Shape height in minutes, 240 - for 1M map
        int    plateWidthInSubplates = 1; // 1M shape width in subset scales,
                                          // for example, 1M map width of 500k maps is a 2

        double subPlateTop    = top1m;      // Shape top in LonLat coordinates
        double subPlateBottom = bottom1m;   // Shape bottom in LonLat coordinates
        double subPlateLeft   = left1m;     // Shape left in LonLat coordinates
        double subPlateRight  = right1m;    // Shape right in LonLat coordinates

        bool   knownScale     = false;      // true if we known specified scale. Otherwise assume that
                                            // we deals with 1M map

        if (scale == "100k" || scale == "50k" || scale == "25k")
        {
            // 100k subsets (50k, 25k) calculates based on superset (100k) settings
            plateWidthInSubplates = 12;
            plateWidthMin         = 30 * mapsCountMult;
            plateHeightMin        = 20;
            knownScale            = true;
        }
        else if (scale == "200k")
        {
            plateWidthInSubplates = 6;
            plateWidthMin         = 60 * mapsCountMult;
            plateHeightMin        = 40;
            knownScale            = true;
        }
        else if (scale == "500k")
        {
            plateWidthInSubplates = 2;
            plateWidthMin         = 180 * mapsCountMult;
            plateHeightMin        = 120;
            knownScale            = true;
        }

        if (knownScale)
        {
            // Detect shape number
            tmpx = (int)((geoLonCenter - left1m)    * 60) / plateWidthMin;
            tmpy = (int)((geoLatCenter  - bottom1m) * 60) / plateHeightMin;

            tmpy = plateWidthInSubplates - 1 - tmpy; // инвертируем, т.к. растем сверху вниз
            clog << "tmpx = " << tmpx << ", tmpy = " << tmpy << endl;

            // Shape number
            plateSubNum = tmpy * plateWidthInSubplates + tmpx + 1;

            plateNameStream << "-";

            // Forming shape name
            if (scale == "100k" || scale == "50k" || scale == "25k")
            {
                if (plateSubNum < 10)
                {
                    plateNameStream << "00";
                }
                else if (plateSubNum > 9 && plateSubNum < 100)
                {
                    plateNameStream << "0";
                }

            }
            else if (scale == "200k")
            {
                if (plateSubNum < 10)
                {
                    plateNameStream << "0";
                }
            }

            if (scale == "500k")
            {
                // 500k maps marks with A-D letters
                char plate500kCh = 'A' + plateSubNum - 1;
                plateNameStream << plate500kCh;
            }
            else
                plateNameStream << plateSubNum;

            // Shape borders in LonLat coordinates
            subPlateTop    = top1m - tmpy  * plateHeightMin/60.0;
            subPlateBottom = subPlateTop   - plateHeightMin/60.0;
            subPlateLeft   = left1m + tmpx * plateWidthMin/60.0;
            subPlateRight  = subPlateLeft  + plateWidthMin/60.0;

            // Be more patient for the huge scales
            if (scale == "50k" || scale == "25k")
            {
                char plate50kCh = get_subplate(geoLonCenter,
                                               geoLatCenter,
                                               subPlateTop,
                                               subPlateLeft,
                                               subPlateBottom,
                                               subPlateRight);
                if (!plate50kCh)
                {
                    cerr << "Incorrect page of 1:50000\n";
                    return 1;
                }

                plateNameStream << '-' << plate50kCh;

                if (scale == "25k")
                {
                    char plate25kCh = get_subplate(geoLonCenter,
                                                   geoLatCenter,
                                                   subPlateTop,
                                                   subPlateLeft,
                                                   subPlateBottom,
                                                   subPlateRight);
                    if (!plate25kCh)
                    {
                        cerr << "Incorrect page of 1:25000\n";
                        return 1;
                    }

                    plate25kCh -= ('A' - 'a'); // change register :-)
                    plateNameStream << '-' << plate25kCh;
                }

            }

            clog << "Shape number: " << plateNameStream.str() << endl;

            fprintf(stderr,
                    "Shape border coordinates (lat/lon) %s:\n"
                    "   (%.6f, %.6f)   (%.6f, %.6f)\n"
                    "   (%.6f, %.6f)   (%.6f, %.6f)\n",
                    plateNameStream.str().c_str(),
                    subPlateTop,    subPlateLeft, subPlateTop,    subPlateRight,
                    subPlateBottom, subPlateLeft, subPlateBottom, subPlateRight);

            // Recalc coordinates to the source SRS
            shapeBorderCoordinates[0] = PointReal(subPlateTop,    subPlateLeft);
            shapeBorderCoordinates[1] = PointReal(subPlateTop,    subPlateRight);
            shapeBorderCoordinates[2] = PointReal(subPlateBottom, subPlateLeft);
            shapeBorderCoordinates[3] = PointReal(subPlateBottom, subPlateRight);

            for (int i = 0; i < 4; ++i)
            {
                if (transform)
                {
                    // At the LatLon representation, first is a latitude that is a Y. Second -
                    // longitude that is X. Before pass it to the pj_transform() we must put to
                    // correct places:
                    //    first - X (lon), second - Y (lat)
                    PJ_COORD coord{};
                    coord.uv.u = shapeBorderCoordinates[i].y;
                    coord.uv.v = shapeBorderCoordinates[i].x;
                    coord = proj_trans(transform, PJ_INV, coord);
                    shapeBorderCoordinates[i].x = coord.xy.x;
                    shapeBorderCoordinates[i].y = coord.xy.y;
                }

                // Recalculate coordinates to the raster pixels
                //geo_to_pixel_coordinate(geoTransform,
                //                     shapeBorderCoordinates[i].x, shapeBorderCoordinates[i].y, shapeBorderPixels[i].x, shapeBorderPixels[i].y);
                geoTransform.coordinateToPixel(shapeBorderCoordinates[i].x, shapeBorderCoordinates[i].y, shapeBorderPixels[i].x, shapeBorderPixels[i].y);

                // TODO: add processing for partial shapes: there is a lot of scanned maps with breaks
                //       one shape to the seleveral ones. Some notes:
                //       - if coordinates negate, set it to the zero
                //       - if coordinates greater real shape size, limit it with shape size
            }
        }

        fprintf(stderr,
                "Shape border coordinates %s:\n"
                "   (%.6f, %.6f)   (%.6f, %.6f)\n"
                "   (%.6f, %.6f)   (%.6f, %.6f)\n",
                plateNameStream.str().c_str(),
                shapeBorderCoordinates[0].x, shapeBorderCoordinates[0].y,
                shapeBorderCoordinates[1].x, shapeBorderCoordinates[1].y,
                shapeBorderCoordinates[2].x, shapeBorderCoordinates[2].y,
                shapeBorderCoordinates[3].x, shapeBorderCoordinates[3].y);
        fprintf(stderr,
                "Shape border in pixels %s:\n"
                "   (%5d, %5d)   (%5d, %5d)\n"
                "   (%5d, %5d)   (%5d, %5d)\n",
                plateNameStream.str().c_str(),
                shapeBorderPixels[0].x, shapeBorderPixels[0].y,
                shapeBorderPixels[1].x, shapeBorderPixels[1].y,
                shapeBorderPixels[2].x, shapeBorderPixels[2].y,
                shapeBorderPixels[3].x, shapeBorderPixels[3].y);

        // Prepare "cutline" file. Use CSV with WKT:
        //   http://www.gdal.org/drv_csv.html
        //   https://en.wikipedia.org/wiki/Well-known_text

        struct CutlineStreamFree
        {
            void operator()(ostream *ost)
            {
                if (ost != &cout &&
                    ost != &cerr &&
                    ost != &clog)
                {
                    delete ost;
                }
            }
        };

        unique_ptr<ostream, CutlineStreamFree> cutline(&cout);
        string cutlineFile = "<stdout>";

        if (!generateOnly)
        {
            cutlineFile = generate_temp_file_name() + ".csv";
            cutline.reset(new ofstream(cutlineFile.c_str()));
        }

        *cutline << "WKT,dummy\n" << "\"";
        stringstream cutlinePolygon;

        cutlinePolygon << "POLYGON((";

        if (knownScale)
        {
            cutlinePolygon << shapeBorderCoordinates[0].x << " " << shapeBorderCoordinates[0].y << ",";
            cutlinePolygon << shapeBorderCoordinates[1].x << " " << shapeBorderCoordinates[1].y << ",";
            cutlinePolygon << shapeBorderCoordinates[3].x << " " << shapeBorderCoordinates[3].y << ",";
            cutlinePolygon << shapeBorderCoordinates[2].x << " " << shapeBorderCoordinates[2].y;
        }
        else
        {
            cutlinePolygon << shapeBorderCoordinates[0].x << " " << shapeBorderCoordinates[0].y << ",";

            // Additional cut dots
            double lat = top1m;
            for (double lon = left1m + 1; lon < right1m; lon += 1.00)
            {
                double u = lon;
                double v = lat;

                if (transform)
                {
                    PJ_COORD coord{};
                    coord.uv.u = u;
                    coord.uv.v = v;
                    coord = proj_trans(transform, PJ_INV, coord);
                    u = coord.uv.u;
                    v = coord.uv.v;
                }

                cutlinePolygon << u << " " << v << ",";
            }

            cutlinePolygon << shapeBorderCoordinates[1].x << " " << shapeBorderCoordinates[1].y << ",";
            cutlinePolygon << shapeBorderCoordinates[3].x << " " << shapeBorderCoordinates[3].y << ",";

            // Additional cut dots
            lat = bottom1m;
            for (double lon = right1m - 1; lon > left1m; lon -= 1.00)
            {
                double u = lon;
                double v = lat;

                if (transform)
                {
                    PJ_COORD coord{};
                    coord.uv.u = u;
                    coord.uv.v = v;
                    coord = proj_trans(transform, PJ_INV, coord);
                    u = coord.uv.u;
                    v = coord.uv.v;
                }

                cutlinePolygon << u << " " << v << ",";
            }

            cutlinePolygon << shapeBorderCoordinates[2].x << " " << shapeBorderCoordinates[2].y;
        }

        // Close Polygon to make Gdal happy
        cutlinePolygon << "," << shapeBorderCoordinates[0].x << " " << shapeBorderCoordinates[0].y;
        cutlinePolygon << "))";

        *cutline << cutlinePolygon.str();
        *cutline << "\",\n";
        cutline.reset();

        clog << cutlineFile << endl;
        clog << "Polygon:\n" << cutlinePolygon.str() << endl;

        if (!generateOnly)
        {

            {
                auto pid = fork();
                if (pid == 0)
                {
                    vector<char*> args = {
                        ::strdup(gdalwarpPath.c_str()),
                        ::strdup("-of"),
                        ::strdup(ouDriver.c_str()),
                        ::strdup("-dstalpha"),
                        ::strdup("-cutline"),
                        ::strdup(cutlineFile.c_str()),
                    };

                    if (cropToCutline)
                        args.push_back(::strdup("-crop_to_cutline"));

                    if (!gdalwarpOpts.empty())
                    {
                        // Reserve +1 for future nullptr and input/output file
                        args.reserve(args.size() + gdalwarpOpts.size() + 3);
                        copy(begin(gdalwarpOpts), end(gdalwarpOpts), back_inserter(args));
                    }

                    args.insert(args.end(), {::strdup(inFileName.c_str()),
                                             ::strdup(ouFileName.c_str()),
                                             nullptr});

                    clog << "gdalwarp args:\n";
                    for (auto arg : args)
                    {
                        if (arg)
                            clog << "   " << arg << endl;
                    }

                    if (execvp(gdalwarpPath.c_str(), args.data()) < 0)
                        exit(1);
                }
                waitpid(pid, nullptr, 0);
            }

            unlink(cutlineFile.c_str());
        }
    }

    return 0;
}


CrsInstance::CrsInstance()
    : m_crs(nullptr, deleter)
{
}

CrsInstance::CrsInstance(std::string_view definition)
    : CrsInstance()
{
    m_definition = definition;
    m_crs.reset(proj_create(PJ_DEFAULT_CTX,
                            pj_add_type_crs_if_needed(definition).c_str()));
    if (m_crs)
        // can be checked via `operator bool()`
        return;

    auto type = proj_get_type(m_crs.get());
    if (type == PJ_TYPE_BOUND_CRS) {
        m_crs.reset(proj_get_source_crs(PJ_DEFAULT_CTX, m_crs.get()));
        type = proj_get_type(m_crs.get());
    }

    if (type == PJ_TYPE_GEOGRAPHIC_2D_CRS ||
        type == PJ_TYPE_GEOGRAPHIC_3D_CRS ||
        type == PJ_TYPE_GEODETIC_CRS) {
        pj_ptr_t cs = {proj_crs_get_coordinate_system(PJ_DEFAULT_CTX, m_crs.get()), deleter};
        assert(cs);

        const char *axisName = "";
        proj_cs_get_axis_info(PJ_DEFAULT_CTX, cs.get(), 0,
                              &axisName, // name
                              nullptr,   // abbrev
                              nullptr,   // direction
                              &m_toRadians,
                              nullptr,   // unit name
                              nullptr,   // unit authority
                              nullptr    // unit code
                              );

        std::clog << __PRETTY_FUNCTION__ << ": axisName=" << axisName << '\n';

        auto axisNameView = std::string_view(axisName);
        m_isLatFirst = axisNameView.find("latitude") != std::string_view::npos;
        m_isLongLat = m_isLatFirst || axisNameView.find("longitude") != std::string_view::npos;
    }
}

bool CrsInstance::isLatLong() const
{
    return m_isLongLat;
}

bool CrsInstance::isLatFirst() const
{
    return m_isLatFirst;
}

double CrsInstance::toRadians() const
{
    return m_toRadians;
}

std::string_view CrsInstance::definition() const
{
    return m_definition;
}

CrsInstance::operator bool() const
{
    return bool(m_crs);
}

PJ *CrsInstance::get() const
{
    return m_crs.get();
}

void CrsInstance::reset()
{
    if (m_crs)
        m_crs.reset();
}

PJ *CrsInstance::release()
{
    return m_crs.release();
}

CrsInstance CrsInstance::getGeographicCrs() const
{
    double toRadians = 0.0;
    bool isLatFirst = false;

    auto srcType = proj_get_type(m_crs.get());
    if (srcType != PJ_TYPE_PROJECTED_CRS)
        return {};

    auto base = wrap(proj_get_source_crs(PJ_DEFAULT_CTX, m_crs.get()));
    assert(base);

    auto baseType = proj_get_type(base.get());
    if (baseType != PJ_TYPE_GEOGRAPHIC_2D_CRS &&
        baseType != PJ_TYPE_GEOGRAPHIC_3D_CRS) {
        return {};
    }

    auto cs = wrap(proj_crs_get_coordinate_system(PJ_DEFAULT_CTX, base.get()));
    assert(cs);

    const char *axisName = "";
    proj_cs_get_axis_info(nullptr, cs.get(), 0,
                          &axisName, // name,
                          nullptr,   // abbrev
                          nullptr,   // direction
                          &toRadians,
                          nullptr, // unit name
                          nullptr, // unit authority
                          nullptr  // unit code
                          );

    std::clog << __PRETTY_FUNCTION__ << ": axisName=" << axisName << '\n';

    isLatFirst = std::string_view(axisName).find("latitude") != std::string_view::npos;

    auto retCStr = proj_as_proj_string(PJ_DEFAULT_CTX, base.get(), PJ_PROJ_5, nullptr);

    CrsInstance result;

    result.m_crs = std::move(base);
    result.m_definition = retCStr ? retCStr : "";
    result.m_toRadians = toRadians;
    result.m_isLatFirst = isLatFirst;
    result.m_isLongLat = true;

    std::clog << __PRETTY_FUNCTION__ << ": result.m_definition=" << result.m_definition << '\n';

    return result;
}

CrsInstance::pj_ptr_t CrsInstance::wrap(PJ *pj)
{
    return {pj, deleter};
}
