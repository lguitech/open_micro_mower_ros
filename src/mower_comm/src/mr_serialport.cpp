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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include<sys/ioctl.h>

#include "mr_serialport.h"



int mr_class_serialport::open_port(const char* uartname)
{
    int fd = open(uartname, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if (-1 == fd) {
        perror("Can't Open Serial Port");
        return(-1);
    }
     if(fcntl(fd, F_SETFL, 0)<0) {
            printf("fcntl failed!\n");
     }
     else{
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
     }

     if(isatty(STDIN_FILENO)==0) {
        printf("standard input is not a terminal device\n");
     }
     else {
        printf("isatty success!\n");
     }
     printf("fd-open=%d\n",fd);
     return fd;
}


int mr_class_serialport::set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    case 921600:
        printf("B921600\n");
        cfsetispeed(&newtio, B921600);
                cfsetospeed(&newtio, B921600);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
        newtio.c_cflag &=  ~CSTOPB;
    else if ( nStop == 2 )
    newtio.c_cflag |=  CSTOPB;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    return 0;
}


mr_class_serialport::mr_class_serialport()
{
    h_serialport = -1;
}

mr_class_serialport::~mr_class_serialport()
{
    if (h_serialport != -1) {
        close(h_serialport);
    }

}

bool mr_class_serialport::init(const char* dev, int baudrate)
{
    h_serialport = open_port(dev);
    if (h_serialport < 0) {
        printf("open serial port failed\r\n");
        return false;
    }

    set_opt(h_serialport, baudrate, 8, 'N', 1);
    printf("open serial port success\r\n");
    return true;
}

void mr_class_serialport::uninit()
{
    if (h_serialport != -1) {
        close(h_serialport);
        h_serialport = -1;
    }
}

int mr_class_serialport::send(unsigned char* buffer, int size)
{
    return write(h_serialport, buffer, size);
}

int mr_class_serialport::recv(unsigned char* buffer, int size)
{
    return read(h_serialport, buffer, size);
}
