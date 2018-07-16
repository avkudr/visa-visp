#include "vpVisaAdapter.h"

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
//#if (defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_DC1394))

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpViper650.h>
//#include <visp3/robot/vpRobotViper850.h>
//#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#define L 0.003 // to deal with a 12.7cm by 12.7cm square


int main()
{
  vpViper650 robot;

  // init communication with simulator
  vpVisaAdapter * adapter = new vpVisaAdapter();
  adapter->connect();

  vpMatrix eJe;
  vpHomogeneousMatrix fMe;

  vpHomogeneousMatrix eMc(vpTranslationVector(0, 0, 0), vpRotationMatrix(vpRxyzVector(0, 0, -M_PI/2.)));

  std::cout << "eMc: \n" << eMc << std::endl;


  vpVelocityTwistMatrix cVe(eMc.inverse());



  std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
  while (1) {

    // Get the jacobian of the robot
    std::vector<double> qvec;
    adapter->getJointPos(qvec);
    vpColVector v(6), q(qvec);


    //v[0] = 0.01; //vpMath::rad(10); vz
    //v[2] = 0.01; //vpMath::rad(10); vz
    v[3] = vpMath::rad(10); // wx
    //v[5] = vpMath::rad(10); // wz



    std::cout << "q: " << q.t() << std::endl;
    robot.get_eJe(q, eJe);
    robot.get_fMe(q, fMe);
    std::cout << "fMe: \n" << fMe << std::endl;
    std::cout << "fMc: \n" << fMe * eMc << std::endl;



    vpColVector q_dot = (cVe * eJe).pseudoInverse() * v;

    std::vector<double> q_dot_;
    for (unsigned int i=0; i < q_dot.size(); i++) {
      q_dot_.push_back(q_dot[i]);
    }

    adapter->setJointVel(q_dot_);

    vpTime::wait(40);

  }
  delete adapter;
}

//#else
//int main()
//{
//  std::cout << "You do not have an Viper 850 robot connected to your computer..." << std::endl;
//  return EXIT_SUCCESS;
//}
//#endif
