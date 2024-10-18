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

#ifndef __MR_MAINPROC_H__
#define __MR_MAINPROC_H__

#include "map_comm.h"

class mr_class_mapproc 
{
private:

public:
    mr_class_mapproc();
    ~mr_class_mapproc();
    
    bool do_mapproc(std::string& strContent, mr_mower_map& mower_map);
};


#endif
