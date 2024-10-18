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

#include "ros/ros.h"
#include "mr_bletool.h"

mr_class_bletool::mr_class_bletool()
{
    pos_tail = 0;
}
mr_class_bletool::~mr_class_bletool()
{

}
bool mr_class_bletool::split_string(char* sz_data, std::vector<std::string>& vec_result)
{
    const char s[2] = ",";
    char *token;
   
    token = strtok(sz_data, s);

    while( token != NULL ) {
        vec_result.push_back(std::string(token));
        token = strtok(NULL, s);
    }

    if (vec_result.size() != 5) {
        return false;
    }
    if (strncmp(vec_result[0].c_str(), "$r", 2) != 0){
        return false;
    }
    if (strncmp(vec_result[4].c_str(), "*AA\r\n", 5) != 0) {
        return false;
    }

    return true;
}


void mr_class_bletool::append_data(const unsigned char* sz_data, int len)
{
    if (pos_tail + len > MAX_LEN) {
        return;
    }
    memcpy(buffer_main + pos_tail, sz_data, len);
    pos_tail += len;

}

unsigned char mr_class_bletool::calc_checksum(const unsigned char* sentence, int len)
{
    unsigned char checksum = 0;

    for (int i = 0; i < len; i++) {
        checksum ^= sentence[i];
    }
    return checksum;
}

bool mr_class_bletool::parse_data(mr_ble_data& ble_data)
{
    bool result = false;
    if (pos_tail < 7) {
        return false;
    }

    int pos_head = -1;
    for (int i=0; i<pos_tail-1; i++) {
        if (buffer_main[i] == '$' && buffer_main[i+1] == '$') {
            pos_head =i;
            break;
        }
    }
    if (pos_head == -1) { 
        pos_tail = 0;
        return false;
    }
    if (pos_head + 2 >= pos_tail) {
        return false;
    }
    int len_frame = buffer_main[pos_head + 2];

    if (pos_head + len_frame - 1 >= pos_tail) {
        return false;
    }

    if(buffer_main[pos_head + len_frame - 1] != 0x0a ||
        buffer_main[pos_head + len_frame - 2] != 0x0d)
    {
        pos_tail = 0;
        return false;
    }

    if (len_frame >= 7) {
        unsigned char cs = calc_checksum(buffer_main + pos_head + 4, len_frame - 7);        
        if (cs != buffer_main[pos_head + len_frame - 3]) {
            result = false;
        }
        else {
            ble_data.reqid = buffer_main[pos_head + 3];
            memcpy(ble_data.bufAttach, buffer_main + pos_head + 4, len_frame - 7);
            ble_data.bufAttach[len_frame - 7] = 0; 
            result = true;
        }
    }

    if (pos_head + len_frame < pos_tail) {
        for (int i = pos_head + len_frame; i < pos_tail; i++) {
            buffer_main[i -pos_head + len_frame] = buffer_main[i];
        }
        pos_tail = pos_tail - (pos_head + len_frame);
    }
    else {
        pos_tail = 0;
    }

    return result;
}
