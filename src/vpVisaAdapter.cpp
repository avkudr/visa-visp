#include "vpVisaAdapter.h"

// =============================================================================
// STRING MANIPULATIONS
// =============================================================================

std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r_ ")
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

// =============================================================================
// FUNCTIONS
// =============================================================================

vpVisaAdapter::vpVisaAdapter()
    : connected(false)
{

}

vpVisaAdapter::~vpVisaAdapter()
{
    this->disconnect();
}


bool vpVisaAdapter::connect(const char* host, const unsigned int port)
{
    #ifdef _WIN32
        WSAStartup(MAKEWORD(2,0), &WSAData);
    #endif

    server_socket.sin_addr.s_addr = inet_addr(host);
    server_socket.sin_family	  = AF_INET;
    server_socket.sin_port		  = htons(port);

    sock = socket(AF_INET, SOCK_DGRAM , IPPROTO_UDP);
	//sock = socket(AF_INET, SOCK_STREAM, 0);

    #ifdef _WIN32
        connected = connected = (::connect(sock, (SOCKADDR*)&sin, sizeof(sin)) != SOCKET_ERROR);
    #elif __linux__ || __APPLE__
        auto res = (::connect(sock, (struct sockaddr*)&server_socket, sizeof(server_socket))<0);
        connected = (res == 0);
        usleep(50*1000);
    #endif

    std::vector<double> K;
    this->getCalibMatrix(K);
    int width = K[6]*2;
    int height = K[7]*2;
    this->bufferImage = new unsigned char[width*height+2];

    return connected;
}

void vpVisaAdapter::disconnect()
{
    if (this->connected){
        //dtor
        #ifdef _WIN32
            closesocket(sock); // Fermeture du socket
            WSACleanup();
        #elif __linux__ || __APPLE__
            close(sock); // Fermeture du socket
        #endif

        connected = false;
    }
}

const bool vpVisaAdapter::sendCmd(std::string cmd, std::vector<double> args)
{
    std::string msg = cmd;
    for (auto i = 0; i < args.size(); i++){
        msg.append(",");
        msg.append( std::to_string(args[i]) );
    }
    std::cout << msg << std::endl;

    char buffer[1024];
    strncpy(buffer, msg.c_str(), msg.size());
    ::send(sock,buffer,msg.size(),0);

    char bufferResponse[500];
    ::recv(sock, bufferResponse, 500, 0);
    std::string str(bufferResponse);
    //std::cout << "response from visa" << str << std::endl;
    rtrim(str);
    if (str.compare(0,2,"OK") == 0){
        return true;
    }
    else{
        std::cerr << "ERROR: " << bufferResponse << std::endl;
        return false;
    }
}

const bool vpVisaAdapter::setJointPosAbs(std::vector<double> joints)
{
    return sendCmd("SETJOINTPOSABS",joints);
}

const bool vpVisaAdapter::setJointPosRel(std::vector<double> joints)
{
    return sendCmd("SETJOINTPOSREL",joints);
}

const bool vpVisaAdapter::setJointVel(std::vector<double> velocities)
{
    return sendCmd("SETJOINTVEL",velocities);
}

const bool vpVisaAdapter::homing()
{
    return sendCmd("HOMING",{});
}

void vpVisaAdapter::getCalibMatrix(std::vector<double> & matrix)
{
    matrix.clear();
    char buffer[12] = "GETCALIBMAT";
    char bufferResponse[500]; //too large but sure to fit
    ::send(sock, buffer,sizeof(buffer)-1,0);
    ::recv(sock, bufferResponse, 500, 0);
    std::string str(bufferResponse);
    rtrim(str);
    std::vector<std::string> valuesStr = split(str, ',');

    matrix.clear();
    matrix.resize(valuesStr.size());
    for (auto i = 0; i < matrix.size(); i++){
        matrix[i] = std::stof(valuesStr[i]);
    }
}

void vpVisaAdapter::getJointPos(std::vector<double> & values)
{
    char buffer[12] = "GETJOINTPOS";
    char bufferResponse[500]; //too large but sure to fit
    ::send(sock, buffer,sizeof(buffer)-1,0);
    ::recv(sock, bufferResponse, 500, 0);
    std::string str(bufferResponse);
    rtrim(str);
    std::vector<std::string> valuesStr = split(str, ',');

    values.clear();
    values.resize(valuesStr.size());
    for (auto i = 0; i < values.size(); i++){
        values[i] = std::stof(valuesStr[i]);
    }
}

void vpVisaAdapter::getToolTransform(std::vector<double> & matrix)
{
    char buffer[11] = "GETTOOLPOS";
    char bufferResponse[500]; //too large but sure to fit
    ::send(sock, buffer,sizeof(buffer)-1,0);
    ::recv(sock, bufferResponse, 500, 0);
    std::string str(bufferResponse);
    rtrim(str);
    std::vector<std::string> valuesStr = split(str, ',');

    matrix.clear();
    matrix.resize(valuesStr.size());
    for (auto i = 0; i < matrix.size(); i++){
        matrix[i] = std::stof(valuesStr[i]);
    }
}

std::vector<unsigned char> vpVisaAdapter::getImage()
{


    char buffer[9] = "GETIMAGE";
    char bufferResponse[500]; //UDP max package size
    std::string msgPrefix = "PACKAGE_LENGTH:";

    ::send(sock, buffer, 8, 0);
    ::recv(sock, bufferResponse, 500, 0);

	std::string message(bufferResponse);
	message = message.substr(msgPrefix.size(),10);
	message.erase(std::remove_if(message.begin(), message.end(),
                        [](char c) { return !std::isdigit(c); }),
         message.end());
	int imageSize = std::stoi(message); //parse int
	//std::cout << "imageSize: " << imageSize << std::endl;


    char * bufferImage = new char[imageSize+2]; //allocate memory

	//acquire the image
//auto start = std::chrono::steady_clock::now();
	::recv(sock, bufferImage, imageSize+1, MSG_WAITALL);
    //delete prefix and decode
//auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
//std::cout << "Time,ms:" << duration.count() << std::endl;

    
    std::string encodedImage(bufferImage);

    std::string type = encodedImage.substr(0,23);
    int start = 23; //jpeg
    if (type.find("png") != std::string::npos) {
        start = 22; //png
    }
    std::cout << type << " : " << start << std::endl;
    
    auto res  = base64_decode_array(&bufferImage[start],imageSize - start);
    delete[] bufferImage;

    return res;
}

#ifdef WITH_OPENCV
cv::Mat vpVisaAdapter::getImageOpenCV()
{
    std::vector<unsigned char> vectordata = this->getImage();
    cv::Mat data_mat(vectordata,true);

    cv::Mat image(cv::imdecode(data_mat,1)); //put 0 if you want greyscale
    return image;
}

cv::Mat vpVisaAdapter::getImageBWOpenCV()
{
    char buffer[11] = "GETIMAGEBW";
    char bufferResponse[500]; //UDP max package size
    std::string msgPrefix = "PACKAGE_LENGTH:";

    ::send(sock, buffer, 10, 0);
    ::recv(sock, bufferResponse, 500, 0);

	std::string message(bufferResponse);
	message = message.substr(msgPrefix.size(),10);
	message.erase(std::remove_if(message.begin(), message.end(),
                        [](char c) { return !std::isdigit(c); }),
                    message.end());
	int imageSize = std::stoi(message); //parse int

    //unsigned char * bufferImage = new unsigned char[imageSize+2]; //allocate memory
	//acquire the image
	::recv(sock, bufferImage, imageSize+1, MSG_WAITALL);

    cv::Mat image(480, 640, CV_8UC1, bufferImage);
    return image;
}
#endif

#if defined(WITH_OPENCV) && defined(WITH_VISP)
vpImage<unsigned char> vpVisaAdapter::getImageViSP()
{
    vpImage<unsigned char> I;
    vpImageConvert::convert(this->getImageOpenCV(), I);
    return I;
}

vpImage<unsigned char> vpVisaAdapter::getImageBWViSP()
{
    vpImage<unsigned char> I;
    vpImageConvert::convert(this->getImageBWOpenCV(), I);
    return I;
}

vpMatrix vpVisaAdapter::get_fJe()
{
    char buffer[12] = "GETJACOBIAN";
    char bufferResponse[500]; //too large but sure to fit
    ::send(sock, buffer,sizeof(buffer)-1,0);
    ::recv(sock, bufferResponse, 500, 0);
    std::string str(bufferResponse);
    rtrim(str);
    std::vector<std::string> valuesStr = split(str, ',');

    std::vector<double> values;
    values.clear();
    values.resize(valuesStr.size());
    for (auto i = 0; i < values.size(); i++){
        values[i] = std::stof(valuesStr[i]);
    }

    vpMatrix J;
    int nbDOFs = values.size() / 6;
    J = vpMatrix(6, nbDOFs);
    for (int i = 0; i < 6; i++){
        for (int j = 0; j < nbDOFs; j++){
            J[i][j] = values[nbDOFs*i + j];
        }        
    }
    return J;
}

vpHomogeneousMatrix vpVisaAdapter::get_fMe(){
    std::vector<double> fMe_vector;
    this->getToolTransform(fMe_vector);

    vpHomogeneousMatrix fMe;
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            fMe[i][j] = fMe_vector[4*j+i];
        }   
    }
    return fMe;
}

vpMatrix vpVisaAdapter::get_eJe()
{
    auto fMe = this->get_fMe();
    auto fJe = this->get_fJe();
    vpVelocityTwistMatrix tmp(fMe.inverse());
    vpVelocityTwistMatrix fVe;
    fVe[0][0] = tmp[0][0];
    fVe[0][1] = tmp[0][1];
    fVe[0][2] = tmp[0][2];
    fVe[1][0] = tmp[1][0];
    fVe[1][1] = tmp[1][1];
    fVe[1][2] = tmp[1][2];
    fVe[2][0] = tmp[2][0];
    fVe[2][1] = tmp[2][1];
    fVe[2][2] = tmp[2][2];

    fVe[3][3] = tmp[3][3];
    fVe[3][4] = tmp[3][4];
    fVe[3][5] = tmp[3][5];
    fVe[4][3] = tmp[4][3];
    fVe[4][4] = tmp[4][4];
    fVe[4][5] = tmp[4][5];
    fVe[5][3] = tmp[5][3];
    fVe[5][4] = tmp[5][4];
    fVe[5][5] = tmp[5][5];

    return fVe * fJe;
}
#endif
