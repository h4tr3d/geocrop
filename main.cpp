#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <iostream>
#include <sstream>
#include <fstream>

#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <proj_api.h>

using namespace std;

void usage(char *name)
{
    cout << "Tool for automatic crop raster maps\n";
    cout << "(C) Alexander 'hatred' Drozdov, 2012. Distributed under GPLv2 terms\n\n";
    cout << "Use: " << name << " <scale> <in geotiff> <croped geotiff>\n"
         << "  Input geotiff MUST be in RGB pallete, so, use pct2rgb.py to convert from indexed\n"
         << "  Output geotiff croped and nodata areas is transparency\n"
         << "  Scale must be:\n"
         << "    100k     for 1:100 000 plates\n"
         << "    50k      for 1:50 000 plates\n"
         << "  1M (1:1 000 000) support by default\n"
         << "  Any other scales currently not support\n";

}

void pixelToGeoCoordinate(double geoTransform[6],
                          int    rasterX, int    rasterY,
                          double &geoX,   double &geoY)
{
    // geoTransformation to World File notation
    double A = geoTransform[1]; // ширина пикселя
    double B = geoTransform[2]; // TODO: или 4, но для северного полушария 0
    double C = geoTransform[4]; // TODO: или 2, но для северного полушария 0
    double D = geoTransform[5]; // высота пикселя
    double E = geoTransform[0]; // top left X geo location
    double F = geoTransform[3]; // top left Y geo location

    // Перевод координат пикселей в географические координаты
    geoX = E + rasterX * A + rasterY * C;
    geoY = F + rasterY * D + rasterX * B;
}

void geoToPixelCoordinate(double geoTransform[6],
                          double geoX,     double geoY,
                          int    &rasterX, int    &rasterY)
{
    // geoTransformation to World File notation
    double A = geoTransform[1]; // ширина пикселя
    double B = geoTransform[2]; // TODO: или 4, но для северного полушария 0
    double C = geoTransform[4]; // TODO: или 2, но для северного полушария 0
    double D = geoTransform[5]; // высота пикселя
    double E = geoTransform[0]; // top left X geo location
    double F = geoTransform[3]; // top left Y geo location

    // TODO: optimize
    rasterX = (geoX - E - C/D*geoY + F*C/D) / (A - C*B/D);
    rasterY = (geoY - F - rasterX * B) / D;
}

string genTempFileName()
{
    char name[] = "/tmp/geocrop.XXXXXX";
    close(mkstemp(name));
    string result(name);
    return result;
}


//
// Определяем подлист листа с указанным обрамлением. Для листов делящихся последовательно на 4
// Наприме, лист километровки делится на 4 листа полукилометровок, если нам нужно получить
// обрамление листа полукилометровки, расчитываем оное для километровки, передаём в качестве входных
// параметров сюда а так же пробную точку, по которой определяем подлист.
//
// Далее, так же последовательно можно сделать расчёт для двухсотпятидесятиметровки
//
char getSubplate(double lon, double lat, double &subPlateTop,
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


//
// http://www.gdal.org/gdal_tutorial_ru.html
//
int main(int argc, char **argv)
{
    GDALDataset *dataset;

    string       scale = "100k";
    string       inFileName;
    string       ouFileName;

    double       geoTransform[6];
    int          rasterXSize = 0;
    int          rasterYSize = 0;
    int          rasterXCenter = 0;
    int          rasterYCenter = 0;

    if (argc != 4)
    {
        usage(argv[0]);
        return 1;
    }

    scale = argv[1];
    inFileName = argv[2];
    ouFileName = argv[3];

    GDALAllRegister();

    dataset = (GDALDataset*)GDALOpen(inFileName.c_str(), GA_ReadOnly);
    if (dataset == 0)
    {
        cerr << "Can't open file: " << inFileName.c_str() << endl;
        return 1;
    }

    cout << "Драйвер: " << dataset->GetDriver()->GetDescription() << "/"
         << dataset->GetDriver()->GetMetadata(GDAL_DMD_LONGNAME) << endl;

    cout << "Размер: " << dataset->GetRasterXSize() << "x" << dataset->GetRasterYSize() << "x"
         << dataset->GetRasterCount() << endl;

    if (dataset->GetProjectionRef() != 0)
    {
        cout << "Проекция: " << dataset->GetProjectionRef() << endl;
    }

    if (dataset->GetGeoTransform(geoTransform) == CE_None)
    {
        printf( "Начало координат (%.6f,%.6f)\n",
                        geoTransform[0], geoTransform[3] );

        printf( "Размер пиксела (%.6f,%.6f)\n",
                        geoTransform[1], geoTransform[5] );

        //
        // Определяем границы листа
        //
        rasterXSize = dataset->GetRasterXSize();
        rasterYSize = dataset->GetRasterYSize();
        rasterXCenter = rasterXSize / 2;
        rasterYCenter = rasterYSize / 2;

        double box[4][2];    // пары координат, обрамляющие номенклатурный лист
        int    boxPix[4][2]; // координаты пикселей, обрамляющие полезную область
        string plateName;    // полное имя номенклатурного листа

        double geoXCenter;
        double geoYCenter;

        pixelToGeoCoordinate(geoTransform, rasterXCenter, rasterYCenter, geoXCenter, geoYCenter);

        printf("Центр листа: (%5d, %5d)\n", rasterXCenter, rasterYCenter);
        printf("Координаты центра листа: (%.6f, %.6f)\n", geoXCenter, geoYCenter);

        // проверка обратного преобразования
        //geoToPixelCoordinate(geoTransform, geoXCenter, geoYCenter, rasterXCenter, rasterYCenter);
        //printf("Центр листа: (%5d, %5d)\n", rasterXCenter, rasterYCenter);

        // преобразуем описание проекции из WKT в формат PROJ4
        OGRSpatialReference inSrs;
        char *wktRef = (char *) dataset->GetProjectionRef();
        inSrs.importFromWkt(&wktRef);
        char *proj4Ref;
        inSrs.exportToProj4(&proj4Ref);
        string inProj = proj4Ref;

        cout << "PROJ4: " << inProj << endl;

        // Нам нужны координаты не в метрической системе, а в географических, что бы выполнить
        // расчёт по определению листа карты
        double u = 0.0, v = 0.0;
        projPJ pjSrc = 0;
        projPJ pjTar = 0;

        pjSrc = pj_init_plus(inProj.c_str());

        double geoLonCenter = geoXCenter;
        double geoLatCenter = geoYCenter;

        if (!pj_is_latlong(pjSrc))
        {
            pjTar = pj_latlong_from_proj(pjSrc);

            projPJ pjTmp = pj_latlong_from_proj(pjSrc);
            cout << "PROJ4: "  << pj_get_def(pjTmp, 0) << endl;

            if (pjSrc == 0 || pjTar == 0)
            {
                cerr << "Не могу настроить проекцию\n";
                return 1;
            }

            u = geoXCenter;
            v = geoYCenter;
            pj_transform(pjSrc, pjTar, 1, 0, &u, &v, 0);

            geoLonCenter = u * RAD_TO_DEG;
            geoLatCenter = v * RAD_TO_DEG;

            printf("Географические координаты центра листа: (%.6f, %.6f)\n",
                   geoLatCenter, geoLonCenter);
        }

        // определим номенклатурный лист миллионки (1:1000000)
        int Nz = (int)ceil(geoLonCenter / 6);
        int letterNumber = (int)ceil(geoLatCenter / 4);

        // градусная сетка обрамляющяя лист миллионки
        double top1m    = letterNumber * 4;
        double bottom1m = top1m - 4;
        double right1m  = Nz * 6;
        double left1m   = right1m - 6;

        box[0][0] = top1m; box[0][1] = left1m;
        box[1][0] = top1m; box[1][1] = right1m;
        box[2][0] = bottom1m; box[2][1] = left1m;
        box[3][0] = bottom1m; box[3][1] = right1m;

        for (int i = 0; i < 4; ++i)
        {
            if (pjTar)
            {
                // В представлении LatLon первой указывается широта, которая, по сути
                // координата Y, а второй - долгота, которая, по сути, координата X
                // поэтому перед передачей pj_transform мы должны поставить всё на свои места:
                //   первой координатой идёт X (lon), второй Y (lat)
                double u = box[i][1] * DEG_TO_RAD;
                double v = box[i][0] * DEG_TO_RAD;
                pj_transform(pjTar, pjSrc, 1, 0, &u, &v, 0);

                box[i][0] = u;
                box[i][1] = v;
            }

            // Пересчитаем координаты в пиксели на картинке
            geoToPixelCoordinate(geoTransform,
                                 box[i][0], box[i][1], boxPix[i][0], boxPix[i][1]);
        }

        char plateCh = 'A' + letterNumber - 1;              // Буква листа
        int  plateNum = Nz + 30;                            // Номер листа

        stringstream plateNameStream;
        plateNameStream << plateCh << "-" << plateNum;
        plateName = plateNameStream.str();

        cout << "Лист миллионки: " << plateName << endl;
        printf("Географические координаты обрамляющие лист %s:\n"
               "   (%.6f, %.6f)   (%.6f, %.6f)\n"
               "   (%.6f, %.6f)   (%.6f, %.6f)\n",
               plateNameStream.str().c_str(),
               top1m, left1m, top1m, right1m,
               bottom1m, left1m, bottom1m, right1m);


        int tmpx = 1;   // номер листа в ряду миллионки, от 0, слева на право
        int tmpy = 1;   // номер листа в столбце миллионки, от 0, сверху вниз
        int plateSubNum = 1; // сквозной номер листа в миллионке, имеет смысл для 100k, 200k и 500k
                             // более крупные масштабы основываются на 100k с добавлениями
                             // для 500k, это номер буквы, т.к. он обозначается как O-50-A
        double plateWidthMin  = 360; // Ширина листа в минутах, 360 - для миллионки
        double plateHeightMin = 240; // Высота листа в минутах, 240 - для миллионки
        int    plateWidthInSubplates = 1; // Ширина листа миллионки в листах более крупного масштаба

        double subPlateTop    = top1m;      // верх листа в географических координатах
        double subPlateBottom = bottom1m;   // низ листа в географических координатах
        double subPlateLeft   = left1m;     // левый край листа в географических координатах
        double subPlateRight  = right1m;    // правый край листа в географических координатах

        bool   knownScale     = false;      // известен ли нам данным масштаб, будет если false
                                            // будет принятор, что у нас миллионка

        if (scale == "100k" || scale == "50k" || scale == "25k")
        {
            // Параметры для километровки, более мелкие масштабы считаются уже как дочерние к ним
            plateWidthInSubplates = 12;
            plateWidthMin         = 30;
            plateHeightMin        = 20;
            knownScale            = true;
        }
        else if (scale == "200k")
        {
            plateWidthInSubplates = 6;
            plateWidthMin         = 60;
            plateHeightMin        = 40;
            knownScale            = true;
        }
        else if (scale == "500k")
        {
            plateWidthInSubplates = 2;
            plateWidthMin         = 180;
            plateHeightMin        = 120;
            knownScale            = true;
        }

        if (knownScale)
        {
            // определяем номер листа
            tmpx = (int)((geoLonCenter - left1m)    * 60) / plateWidthMin;
            tmpy = (int)((geoLatCenter  - bottom1m) * 60) / plateHeightMin;

            tmpy = plateWidthInSubplates - 1 - tmpy; // инвертируем, т.к. растем сверху вниз
            cout << "tmpx = " << tmpx << ", tmpy = " << tmpy << endl;

            // сам номер листа
            plateSubNum = tmpy * plateWidthInSubplates + tmpx + 1;

            plateNameStream << "-";
            // Сформируем имя листа
            if (scale == "100k" || scale == "50k" || scale == "25k")
            {
                // проставим лидирующие нули
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
                // пятикилометровка обозначается буквой
                char plate500kCh = 'A' + plateSubNum - 1;
                plateNameStream << plate500kCh;
            }
            else
                plateNameStream << plateSubNum;

            // границы листа в географических координатах
            subPlateTop    = top1m - tmpy  * plateHeightMin/60.0;
            subPlateBottom = subPlateTop   - plateHeightMin/60.0;
            subPlateLeft   = left1m + tmpx * plateWidthMin/60.0;
            subPlateRight  = subPlateLeft  + plateWidthMin/60.0;

            // Уточняемся для крупных масштабов
            if (scale == "50k" || scale == "25k")
            {
                char plate50kCh = getSubplate(geoLonCenter,
                                              geoLatCenter,
                                              subPlateTop,
                                              subPlateLeft,
                                              subPlateBottom,
                                              subPlateRight);
                if (!plate50kCh)
                {
                    cerr << "Некорректный лист масштаба 1:50000\n";
                    return 1;
                }

                plateNameStream << '-' << plate50kCh;

                if (scale == "25k")
                {
                    char plate25kCh = getSubplate(geoLonCenter,
                                                  geoLatCenter,
                                                  subPlateTop,
                                                  subPlateLeft,
                                                  subPlateBottom,
                                                  subPlateRight);
                    if (!plate25kCh)
                    {
                        cerr << "Некорректный лист масштаба 1:25000\n";
                        return 1;
                    }

                    plate25kCh -= ('A' - 'a'); // хитрый способ смены регистра :-)
                    plateNameStream << '-' << plate25kCh;
                }

            }

            cout << "Номер листа: " << plateNameStream.str() << endl;

            printf("Географические координаты обрамляющие лист %s:\n"
                   "   (%.6f, %.6f)   (%.6f, %.6f)\n"
                   "   (%.6f, %.6f)   (%.6f, %.6f)\n",
                   plateNameStream.str().c_str(),
                   subPlateTop,    subPlateLeft, subPlateTop,    subPlateRight,
                   subPlateBottom, subPlateLeft, subPlateBottom, subPlateRight);

            // Переведём координаты в исходную систему
            box[0][0] = subPlateTop;    box[0][1] = subPlateLeft;
            box[1][0] = subPlateTop;    box[1][1] = subPlateRight;
            box[2][0] = subPlateBottom; box[2][1] = subPlateLeft;
            box[3][0] = subPlateBottom; box[3][1] = subPlateRight;

            for (int i = 0; i < 4; ++i)
            {
                if (pjTar)
                {
                    // В представлении LatLon первой указывается широта, которая, по сути
                    // координата Y, а второй - долгота, которая, по сути, координата X
                    // поэтому перед передачей pj_transform мы должны поставить всё на свои места:
                    //   первой координатой идёт X (lon), второй Y (lat)
                    double u = box[i][1] * DEG_TO_RAD;
                    double v = box[i][0] * DEG_TO_RAD;
                    pj_transform(pjTar, pjSrc, 1, 0, &u, &v, 0);

                    box[i][0] = u;
                    box[i][1] = v;
                }

                // Пересчитаем координаты в пиксели на картинке
                geoToPixelCoordinate(geoTransform,
                                     box[i][0], box[i][1], boxPix[i][0], boxPix[i][1]);

                // TODO: тут можно сделать так, что если это всего лишь часть листа.
                // например если координаты отрицательные, значит выставить в ноль
                // если больше ширины листа - выставить в максимальный размер.
                // Но вопрос требует более детального рассмотрения.
            }

        }

        printf("Координаты обрамляющие лист %s:\n"
               "   (%.6f, %.6f)   (%.6f, %.6f)\n"
               "   (%.6f, %.6f)   (%.6f, %.6f)\n",
               plateNameStream.str().c_str(),
               box[0][0], box[0][1],
               box[1][0], box[1][1],
               box[2][0], box[2][1],
               box[3][0], box[3][1]);
        printf("Точки растра обрамляющие полезную область %s:\n"
               "   (%5d, %5d)   (%5d, %5d)\n"
               "   (%5d, %5d)   (%5d, %5d)\n",
               plateNameStream.str().c_str(),
               boxPix[0][0], boxPix[0][1],
               boxPix[1][0], boxPix[1][1],
               boxPix[2][0], boxPix[2][1],
               boxPix[3][0], boxPix[3][1]);

        // Теперь сделаем кроп
        string cutlineFile = genTempFileName() + ".csv";
        ofstream cutline(cutlineFile.c_str());
        cutline << "WKT,dummy\n" << "\"POLYGON((";

        cutline << box[0][0] << " " << box[0][1] << ",";
        cutline << box[1][0] << " " << box[1][1] << ",";
        cutline << box[3][0] << " " << box[3][1] << ",";
        cutline << box[2][0] << " " << box[2][1];
        cutline << "))\",\n";
        cutline.close();
        cout << cutlineFile << endl;

        stringstream cropCommand;
        string tmp1 = genTempFileName() + ".tif";
        cropCommand << "gdalwarp "
                    //<< " -wo \"INIT_DEST=255,255,255,0\" "
                    //<< " -dstnodata \"255\" "
                    << " -dstalpha "
                    << " -crop_to_cutline "
                    << " -cutline  " << cutlineFile << " "
                    << inFileName
                    << " "
                    << ouFileName;
        system(cropCommand.str().c_str());
#if 0
        stringstream transparencyCommand;
        transparencyCommand << "gdalwarp "
                            << " -srcnodata \"255\" "
                            << " -dstalpha "
                            << tmp1 << " " << ouFileName;
        system(transparencyCommand.str().c_str());
#endif

        unlink(cutlineFile.c_str());
        unlink(tmp1.c_str());
    }

    return 0;
}

