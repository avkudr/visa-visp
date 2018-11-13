#ifndef VISA_SOCKET_ADAPTER_H
#define VISA_SOCKET_ADAPTER_H

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <vector>
#include <string>
#include <string.h>

#include <cpp-base64/base64.h>

#ifdef WITH_OPENCV
#include <opencv2/opencv.hpp>
#endif

#ifdef WITH_VISP
#include <visp3/io/vpImageIo.h>
#endif

#ifdef _WIN32

#include <string>
#include <winsock2.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")

#elif __linux__ || __APPLE__

#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#endif

#include <iostream>
#include <chrono>

class vpVisaAdapter
{
    public:

        vpVisaAdapter();
        ~vpVisaAdapter();
        bool connect(const char* = "127.0.0.1", const unsigned int = 2408);
        void disconnect();
        const bool isConnected(){ return connected; }

        const bool setJointPosAbs(std::vector<double>);
        const bool setJointPosRel(std::vector<double>);
        const bool setJointVel(std::vector<double>);
        const bool homing();

        void getJointPos(std::vector<double> & );
        void getToolTransform(std::vector<double> & );
        void getCalibMatrix(std::vector<double> & );
        
        std::vector<unsigned char> getImage();

        #ifdef WITH_OPENCV
            cv::Mat getImageOpenCV();
            cv::Mat getImageBWOpenCV();
        #endif

        #if defined(WITH_OPENCV) && defined(WITH_VISP)
            vpImage<unsigned char> getImageViSP();
            vpImage<unsigned char> getImageBWViSP();
            vpMatrix get_eJe();
            vpMatrix get_fJe();
            vpHomogeneousMatrix get_fMe();
        #endif

    private:
        #ifdef _WIN32
            WSADATA WSAData; // configuration socket
            SOCKET sock;
            SOCKADDR_IN sin;
        #elif __linux__ || __APPLE__
            int sock;
            struct sockaddr_in server_socket;
        #endif

        const bool sendCmd(std::string, std::vector<double>);

        unsigned char * bufferImage;
        unsigned char * bufferMsg;
        bool connected;
        bool isVelCtrlActive; // not used yet
};
#endif // VISA_SOCKET_ADAPTER_H
