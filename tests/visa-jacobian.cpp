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

#include <opencv2/opencv.hpp>

int main ()
{
    // init communication with simulator
    vpVisaAdapter adapter;
    adapter.connect();

    while(1){
        double t = vpTime::measureTimeMs();

        auto fJe = adapter.get_fJe(); //geometric jacobian
        auto eJe = adapter.get_eJe(); //analytical jacobian
        auto fMe = adapter.get_fMe();

        // std::cout << "fMe: \n" << fMe << std::endl;
        // return 0;

        vpColVector v(6);
        v[0] =     0; //vx
        v[1] =     0; //vy
        v[2] = -0.02; //vz
        v[3] =     0; //wx
        v[4] =     0; //wy
        v[5] =     0; //wz
        
        vpColVector qdot;

        //qdot = fJe.pseudoInverse() * v; // vf -- good!
        qdot = eJe.pseudoInverse() * v; // ve -- not good!
        
        std::cout << "fJe: \n" <<  fJe << std::endl;

        std::vector<double> qdotVec(qdot.size());
        for (int i = 0; i < qdot.size(); i++){
            qdotVec[i] = qdot[i];
        }

        adapter.setJointVel(qdotVec);

        vpTime::wait(t, 40);
    }    
}