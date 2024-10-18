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

#ifndef __MR_SERIALPORT_H__
#define __MR_SERIALPORT_H__

class mr_class_serialport
{
private:
    int h_serialport;
    int open_port(const char* uartname);
    int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
public:
    mr_class_serialport();
    ~mr_class_serialport();
    bool init(const char* dev, int baudrate);
    void uninit();
    int send(unsigned char* buffer, int size);
    int recv(unsigned char* buffer, int size);
};

#endif
