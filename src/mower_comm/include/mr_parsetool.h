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

#ifndef __MR_PARSETOOL_H__
#define __MR_PARSETOOL_H__
#include "mr_navi_types.h"

class mr_class_parsetool {
private:
    static const int MAX_LEN = 4096;
    char buffer_main[MAX_LEN];
    int pos_tail;
public:
    mr_class_parsetool();
    ~mr_class_parsetool();
    bool append_data(const char* sz_data, int len, std::vector<std::string>& vec_result);
};


#endif