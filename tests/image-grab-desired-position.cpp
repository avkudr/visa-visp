#include <iostream>
#include <chrono>
#include <thread>

#include "vpVisaAdapter.h"

#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpTime.h>

int main ()
{
    // init communication with simulator
    vpVisaAdapter * adapter = new vpVisaAdapter();
    adapter->connect();

    std::vector<double> calibMatrix;
    adapter->getCalibMatrix(calibMatrix);
    double px = calibMatrix[0];
    double py = calibMatrix[4];
    double u0 = calibMatrix[6];
    double v0 = calibMatrix[7];

    std::cout << "Focal distances (x,y) = (" << px << ", " << py << ")" << std::endl;
    std::cout << "Principal point = (" << u0 << ", " << v0 << ")" << std::endl; 
    // image capture
    vpImage<unsigned char> I(v0*2, u0*2, 0);
    vpDisplayOpenCV d;
    vpDisplayOpenCV display(I, 100, 100, "-- current image --") ;

    // camera parameters
    vpCameraParameters::vpCameraParametersProjType
            projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(px,py,u0,v0);

    vpMouseButton::vpMouseButtonType button0;
    vpImagePoint ip0_tmp;

    auto startTime_ = std::chrono::system_clock::now();
    double ms = 0;
    int frames = 0;
    double t;
    while(1)
    {
        //t = vpTime::measureTimeMs();
        startTime_ = std::chrono::system_clock::now();      

        I = adapter->getImageViSP();

        vpDisplay::display(I);
        vpDisplay::displayCharString(I, 10, 10, "Mouse right click on the image to select feature points ...",vpColor::orange);
        vpDisplay::getClick(I,ip0_tmp, button0, false);
        vpDisplay::flush(I);
        if(button0 == 3) {
            break;
        }
        vpDisplay::flush(I);

        //vpTime::wait(t, 40); // Loop time is set to 40 ms, ie 25 Hz

        auto endTime = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime_).count();
        ms += duration;
        frames++;
        if (ms > 1000000){
            std::cout << "FPS: " << frames << std::endl; 
            ms = 0;
            frames = 0;
        }
    }
    button0 = vpMouseButton::vpMouseButtonType::button1;

    vpDisplay::display(I);
    vpDisplay::flush(I);


    // iterations: dot tracker
    vpDot2 blobs[4]; // detected blobs
    vpImagePoint blobsCOG; // blob gravity center
    double Z = 0.05; // depth of desired points

    std::ofstream myfile;

    std::cout << "                           " << std::endl;
    std::cout << "TAKE A DESIRED POSITION ..." << std::endl;
    std::cout << "                           " << std::endl;

    vpDisplay::display(I);
    vpDisplay::displayCharString(I, 10, 10,"click on the dot to initialize the tracker",vpColor::darkGreen);
    vpDisplay::displayCharString(I, 30, 10,"Order: top-left,top-right,bottom-right,bottom-left",vpColor::darkGreen);

    vpDisplay::flush(I);

    vpFeaturePoint pd[4] ;
    std::string filePrefix = "coord_desired" + std::to_string(0) + ".txt";
    myfile.open (filePrefix);
    for ( int i = 0 ; i < 4 ; i++ )
    {
        blobs[i].initTracking(I) ;
        blobsCOG = blobs[i].getCog();

        vpDisplay::displayCross(I, blobsCOG, 10, vpColor::blue) ;
        vpDisplay::flush(I);
        double x = 0, y = 0;
        vpPixelMeterConversion::convertPoint(cam, blobsCOG, x, y) ;
        pd[i].set_xyZ(x,y,Z);
        std::cout << "pd_X[" << i << "] = " << pd[i].get_x() << std::endl;
        std::cout << "pd_Y[" << i << "] = " << pd[i].get_y() << std::endl;
        std::cout << "pd_Z[" << i << "] = " << pd[i].get_Z() << std::endl;
        myfile << blobs[i].getCog().get_i() << std::endl;
        myfile << blobs[i].getCog().get_j() << std::endl;
    }
    vpDisplay::close(I);
    myfile.close();
}
