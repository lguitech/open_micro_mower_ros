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

#ifndef __MR_BLETOOL_H__
#define __MR_BLETOOL_H__
#include <vector>
#include <string>
#include <string.h>
#include <stdlib.h>

class mr_ble_data {
public:
    int reqid;
    unsigned char bufAttach[1024];
};


class mr_class_bletool {
private:
    static const int MAX_LEN = 2048;
    unsigned char buffer_main[MAX_LEN];
    int pos_tail;
    bool split_string(char* sz_data, std::vector<std::string>& vec_result);
public:
    mr_class_bletool();
    ~mr_class_bletool();
    void append_data(const unsigned char* sz_data, int len);
    unsigned char calc_checksum(const unsigned char* sentence, int len);
    bool parse_data(mr_ble_data& ble_data);
};


#endif