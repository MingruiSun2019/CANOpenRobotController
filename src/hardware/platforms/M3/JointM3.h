/**
 * \file JointM3.h
 * \author Vincent Crocher
 * \brief An M3 actuated joint
 * \version 0.1
 * \date 2020-06-16
 * 
 * \copyright Copyright (c) 2020
 * 
 */
#ifndef JOINTM3_H_INCLUDED
#define JOINTM3_H_INCLUDED

#include "ActuatedJoint.h"
#include "KincoDrive.h"

/**
 * \brief M3 actuated joints, using Kinoc drives. 
 *
 */
class JointM3 : public ActuatedJoint {
   private:
    double lastQCommand = 0;

    double fromDriveUnits(int driveValue) { return driveValue / 10000; };
    int toDriveUnits(double jointValue) { return jointValue * 10000; };

   public:
    JointM3(int jointID, double jointMin, double jointMax);
    ~JointM3();
    bool updateValue();
    setMovementReturnCode_t setPosition(double desQ);
    bool initNetwork();
    double getQ();
};

#endif