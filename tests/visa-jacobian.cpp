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

        auto fJe = adapter.get_fJe();
        auto fMe = adapter.get_fMe();
        vpVelocityTwistMatrix fVe(fMe);

        vpColVector v(6);
        v[2] = 0.02;
        vpColVector qdot(7);

        qdot = fJe.pseudoInverse() * fVe * v;

        std::vector<double> qdot_(qdot.size());
        for (int i = 0; i < qdot.size(); i++){
            qdot_[i] = qdot[i];
        }

        adapter.setJointVel(qdot_);

        vpTime::wait(t, 40);
    }    
}