/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the articular frame
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \example servoViper850FourPoints2DArtVelocityInteractionCurrent.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Viper S850 robot (arm with 6 degrees of freedom). The velocities resulting
  from visual servo are here joint velocities. Visual features are the image
  coordinates of 4 points. The target is made of 4 dots arranged as a 10cm by
  10cm square.

*/

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

#define L 0.03 // to deal with a 12.7cm by 12.7cm square

/*!

  Compute the pose \e cMo from the 3D coordinates of the points \e point and
  their corresponding 2D coordinates \e dot. The pose is computed using a Lowe
  non linear method.

  \param point : 3D coordinates of the points.

  \param dot : 2D coordinates of the points.

  \param ndot : Number of points or dots used for the pose estimation.

  \param cam : Intrinsic camera parameters.

  \param cMo : Homogeneous matrix in output describing the transformation
  between the camera and object frame.

  \param cto : Translation in ouput extracted from \e cMo.

  \param cro : Rotation in ouput extracted from \e cMo.

  \param init : Indicates if the we have to estimate an initial pose with
  Lagrange or Dementhon methods.

*/
void compute_pose(vpPoint point[], vpDot2 dot[], int ndot, vpCameraParameters cam, vpHomogeneousMatrix &cMo,
                  vpTranslationVector &cto, vpRxyzVector &cro, bool init)
{
  vpHomogeneousMatrix cMo_dementhon; // computed pose with dementhon
  vpHomogeneousMatrix cMo_lagrange;  // computed pose with dementhon
  vpRotationMatrix cRo;
  vpPose pose;
  vpImagePoint cog;
  for (int i = 0; i < ndot; i++) {

    double x = 0, y = 0;
    cog = dot[i].getCog();
    vpPixelMeterConversion::convertPoint(cam, cog, x,
                                         y); // pixel to meter conversion
    point[i].set_x(x);                       // projection perspective          p
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) {
    pose.computePose(vpPose::DEMENTHON, cMo_dementhon);
    // Compute and return the residual expressed in meter for the pose matrix
    // 'cMo'
    double residual_dementhon = pose.computeResidual(cMo_dementhon);
    pose.computePose(vpPose::LAGRANGE, cMo_lagrange);
    double residual_lagrange = pose.computeResidual(cMo_lagrange);

    // Select the best pose to initialize the lowe pose computation
    if (residual_lagrange < residual_dementhon)
      cMo = cMo_lagrange;
    else
      cMo = cMo_dementhon;

  } else { // init = false; use of the previous pose to initialise LOWE
    cRo.buildFrom(cro);
    cMo.buildFrom(cto, cRo);
  }
  pose.computePose(vpPose::LOWE, cMo);
  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
}

int main()
{
  try {
    vpHomogeneousMatrix eMc(vpTranslationVector(0, 0, 0), vpRotationMatrix(vpRxyzVector(0, 0, -M_PI/2.)));
    vpVelocityTwistMatrix cVe(eMc.inverse());

    vpServo task;
  
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
    int i;

    I = adapter->getImageViSP();

//    g.acquire(I);

#ifdef VISP_HAVE_X11
    vpDisplayX display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I, 100, 100, "Current image");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpDot2 dot[4];
    vpImagePoint cog;

    std::cout << "Click on the 4 dots clockwise starting from upper/left dot..." << std::endl;

    for (i = 0; i < 4; i++) {
      dot[i].setGraphics(true);
      dot[i].initTracking(I);
      cog = dot[i].getCog();
      vpDisplay::displayCross(I, cog, 10, vpColor::blue);
      vpDisplay::flush(I);
    }

    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    cam.printParameters();

    {
    double hfov = vpMath::rad(30);
    double vfov = vpMath::rad(30);
    cam.initFromFov(I.getWidth(), I.getHeight(), hfov, vfov);

    cam.initPersProjWithoutDistortion(cam.get_py(), cam.get_py(), u0, v0);
    std::cout << cam << std::endl;
    }

    // Sets the current position of the visual feature
    vpFeaturePoint p[4];
    for (i = 0; i < 4; i++)
      vpFeatureBuilder::create(p[i], cam, dot[i]); // retrieve x,y  of the vpFeaturePoint structure

    // Set the position of the square target in a frame which origin is
    // centered in the middle of the square
    vpPoint point[4];
    point[0].setWorldCoordinates(-L, -L, 0);
    point[1].setWorldCoordinates(L, -L, 0);
    point[2].setWorldCoordinates(L, L, 0);
    point[3].setWorldCoordinates(-L, L, 0);

    // Initialise a desired pose to compute s*, the desired 2D point features
    vpHomogeneousMatrix cMo;
    vpTranslationVector cto(0, 0, 0.5); // tz = 0.5 meter
    vpRxyzVector cro(vpMath::rad(0), vpMath::rad(10), vpMath::rad(20));
    vpRotationMatrix cRo(cro); // Build the rotation matrix
    cMo.buildFrom(cto, cRo);   // Build the homogeneous matrix

    // Sets the desired position of the 2D visual feature
    vpFeaturePoint pd[4];
    // Compute the desired position of the features from the desired pose
    for (int i = 0; i < 4; i++) {
      vpColVector cP, p;
      point[i].changeFrame(cMo, cP);
      point[i].projection(cP, p);

      pd[i].set_x(p[0]);
      pd[i].set_y(p[1]);
      pd[i].set_Z(cP[2]);
    }

    // We want to see a point on a point
    for (i = 0; i < 4; i++)
      task.addFeature(p[i], pd[i]);

    // Set the proportional gain
    task.setLambda(0.1);

    // Display task information
    task.print();

    // Define the task
    // - we want an eye-in-hand control law
    // - articular velocity are computed
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task.print();

    task.set_cVe(cVe);
    task.print();

    // Set the Jacobian (expressed in the end-effector frame)
    vpMatrix eJe;
    bool quit = false;

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
    while (! quit) {
      double t = vpTime::measureTimeMs();

      // Acquire a new image from the camera
      I = adapter->getImageViSP();

      // Display this image
      vpDisplay::display(I);

      try {
        // For each point...
        for (i = 0; i < 4; i++) {
          // Achieve the tracking of the dot in the image
          dot[i].track(I);
          // Display a green cross at the center of gravity position in the
          // image
          cog = dot[i].getCog();
          vpDisplay::displayCross(I, cog, 10, vpColor::green);
        }
      } catch (...) {
        quit = true;
      }

      // During the servo, we compute the pose using LOWE method. For the
      // initial pose used in the non linear minimisation we use the pose
      // computed at the previous iteration.
      compute_pose(point, dot, 4, cam, cMo, cto, cro, false);

      std::cout << "cMo:\n" << cMo << std::endl;
      vpHomogeneousMatrix fMe;
      for (i = 0; i < 4; i++) {
        // Update the point feature from the dot location
        vpFeatureBuilder::create(p[i], cam, dot[i]);
        // Set the feature Z coordinate from the pose
        vpColVector cP;
        point[i].changeFrame(cMo, cP);

        p[i].set_Z(cP[2]);
      }

      // Get the jacobian of the robot
      std::vector<double> qvec;
      adapter->getJointPos(qvec);
      vpColVector q(qvec);
      eJe = adapter->get_eJe();

      // Update this jacobian in the task structure. It will be used to
      // compute the velocity skew (as an articular velocity) qdot = -lambda *
      // L^+ * cVe * eJe * (s-s*)
      task.set_eJe(eJe);

//      robot.get_fMe(q, fMe);
//      std::cout << "fMc:\n" << fMe * eMc << std::endl;
//      quit = true;



      vpColVector v;
      // Compute the visual servoing skew vector
      v = task.computeControlLaw();
      std::vector<double> v_;
      for (unsigned int i=0; i < v.size(); i++) {
        v_.push_back(v[i]);
      }

      v.rad2deg();
      std::cout << "Send qdot in deg: " << v.t() << std::endl;

      adapter->setJointVel(v_);

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);

      // Apply the computed joint velocities to the robot
      //robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);

      std::cout << "Joint vel: " << v.t() << std::endl;

      // Flush the display
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }

      // std::cout << "|| s - s* || = "  << ( task.getError() ).sumSquare() <<
      // std::endl;
      vpTime::wait(t, 40); // Loop time is set to 40 ms, ie 25 Hz
    }

    adapter->setJointVel({0,0,0,0,0,0,0}); // stop robot

    std::cout << "Display task information: " << std::endl;
    task.print();
    task.kill();
    delete adapter;
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}

//#else
//int main()
//{
//  std::cout << "You do not have an Viper 850 robot connected to your computer..." << std::endl;
//  return EXIT_SUCCESS;
//}
//#endif
