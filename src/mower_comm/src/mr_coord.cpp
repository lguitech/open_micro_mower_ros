/*********************************************************************
 *
 *  This file is part of the [OPEN_MICRO_MOWER_ROS] project.
 *  Licensed under the MIT License for non-commercial purposes.
 *  Author: Brook Li
 *  Email: lguitech@126.com
 *
 *  For more details, refer to the LICENSE file or contact [lguitech@126.com].
 *
 *  Commercial use requires a separate license.
 *
 *  This software is provided "as is", without warranty of any kind.
 *
 *********************************************************************/

#include "mr_coord.h"
#include <math.h>
#include <iostream>


MR_Coord::MR_Coord() {
   pj_wgs84 = pj_init_plus(param_wgs84); 
   pj_utm = pj_init_plus(param_utm_default);
   
   m_isnorth = default_isnorth;
   m_zone = default_zone;
}


MR_Coord::~MR_Coord() {
    if (pj_wgs84) {
        pj_free(pj_wgs84);
        pj_wgs84 = NULL;
    }
    if (pj_utm) {
        pj_free(pj_utm);
        pj_utm = NULL;
    }
}

MR_Coord* MR_Coord::getInstance() {
  static MR_Coord instance;
  return &instance;
}


bool MR_Coord::reset_utmproj_param(double lon, double lat) 
{
    int zone = (int)((lon+186.0)/6.0);
    bool isnorth = lat > 0 ? true : false;
    if (zone != m_zone || isnorth != m_isnorth) {
        m_zone = zone;
        m_isnorth = isnorth;

        std::string strparam = "+proj=utm +zone=" + std::to_string(zone);
        if (!isnorth) {
            strparam += " +south";
        }
        strparam += " +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
        const char* szparam = strparam.c_str();
        if (pj_utm != NULL) {
            pj_free(pj_utm);
            pj_utm = NULL;
        }
        printf("pj_init_plus param = %s\r\n", szparam);
        pj_utm = pj_init_plus(szparam);
        if (pj_utm == NULL) {
            printf("set utm param fail!\r\n");
            return false;
        }
    }
    return true;
}

bool MR_Coord::convert_to_utm_reset_param(const double lon, const double lat, double* utm_x, double* utm_y)
{
    if (!reset_utmproj_param(lon, lat)) {
        return false;
    }

    *utm_x = lon * DEG_TO_RAD;
    *utm_y = lat * DEG_TO_RAD;
    pj_transform(pj_wgs84, pj_utm, 1, 1, utm_x, utm_y, NULL);
    return true;
}
 

bool MR_Coord::convert_to_utm(const double lon, const double lat, double* utm_x, double* utm_y)
{
    if (!reset_utmproj_param(lon, lat)) {
        return false;
    }

    
    *utm_x = lon * DEG_TO_RAD;
    *utm_y = lat * DEG_TO_RAD;
    pj_transform(pj_wgs84, pj_utm, 1, 1, utm_x, utm_y, NULL);
    return true;
} 

bool MR_Coord::convert_to_wgs84(const double utm_x, const double utm_y, double* lon, double* lat)
{
    if (pj_utm == NULL) {
        printf("pj_utm not initialized\r\n");
        return false;
    }

    *lon = utm_x;
    *lat = utm_y;
    pj_transform(pj_utm, pj_wgs84, 1, 1,lon, lat, NULL);
    *lon = *lon * RAD_TO_DEG;
    *lat = *lat * RAD_TO_DEG;
    return true;
}




