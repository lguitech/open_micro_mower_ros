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

#ifndef __MR_PREPROC_TOOL_H__
#define __MR_PREPROC_TOOL_H__

#include "map_comm.h"


class mr_preproc_tool 
{
private:
    bool parse_raw_map(const char* buffer, mr_raw_map& raw_map);
    bool pre_process(mr_raw_map& raw_map);//转换为UTM，识别pt_min
    bool offset_map(mr_raw_map& map_input, mr_raw_map& map_output);
public:
    mr_preproc_tool();
    ~mr_preproc_tool();

    bool get_preprocess_map(const char* buffer, mr_raw_map& preprocess_map);
   
};


#endif
