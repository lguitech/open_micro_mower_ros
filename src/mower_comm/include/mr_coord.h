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


#ifndef __MR_COORD_H__
#define __MR_COORD_H__
#include <proj_api.h>
#include <string>


class MR_Coord {
public:
    
    ~MR_Coord();
    
public:
    static MR_Coord* getInstance();
    bool convert_to_utm(const double lon, const double lat, double* utm_x, double* utm_y);
    bool convert_to_utm_reset_param(const double lon, const double lat, double* utm_x, double* utm_y);
    bool convert_to_wgs84(const double utm_x, const double utm_y, double* lon, double* lat);
    bool reset_utmproj_param(double lon, double lat);   

private:
    MR_Coord();
    const char* param_wgs84 = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";
    const char* param_utm_default = "+proj=utm +zone=50 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    const int default_zone = 50;
    const bool default_isnorth = true;
     
    projPJ pj_wgs84;
    projPJ pj_utm;  
    int m_zone;
    bool m_isnorth;
};


#endif
