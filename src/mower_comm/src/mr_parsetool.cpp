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

#include "mr_parsetool.h"

mr_class_parsetool::mr_class_parsetool()
{
    pos_tail = 0;
}
mr_class_parsetool::~mr_class_parsetool()
{

}
bool mr_class_parsetool::append_data(const char* sz_data, int len, std::vector<std::string>& vec_result)
{
    if (pos_tail + len > MAX_LEN) {
        pos_tail = 0;
        return false;
    }

    strncpy(buffer_main + pos_tail, sz_data, len);
    pos_tail += len;

    vec_result.clear();


    int index_start = -1;
    int index_end = -1;

    for (int i=0; i<pos_tail; i++) {
        if (buffer_main[i] == '$') {
            index_start = i;
        }
        else if (buffer_main[i] == '\n') {
            index_end = i;
            if (index_end > index_start && index_start != -1) {
                int len_sentence = index_end - index_start + 1;
                char sz_temp[MAX_LEN];
                strncpy(sz_temp, buffer_main + index_start, len_sentence);
                sz_temp[len_sentence] = 0;
                vec_result.push_back(std::string(sz_temp));

                index_start = -1;
                index_end = -1;
            }
        }
    }

    if(index_start == -1) {
        pos_tail = 0;
    }
    else if (index_start > 0 ){
        if (index_end == -1) {
            for (int i=index_start; i<pos_tail; i++) {
                buffer_main[i-index_start] = buffer_main[i]; 
            }
            pos_tail = (pos_tail - index_start);
        }        
    }
    return true;
}