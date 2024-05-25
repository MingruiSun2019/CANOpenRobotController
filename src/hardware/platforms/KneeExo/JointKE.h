/**
 * \file JointKE.h
 * \author Vincent Crocher, Mingrui Sun
 * \brief An KneeExosketon actuated joint
 * \version 0.2
 * \date 2020-07-27
 *
 * \copyright Copyright (c) 2020
 *
 */
#ifndef JOINTKE_H_INCLUDED
#define JOINTKE_H_INCLUDED

#include <iostream>
#include <cmath>

#include "Joint.h"
#include "EPOS4Drive.h"

/**
 * \brief KE actuated joints, using EPOS4 drives.
 *
 */
class JointKE : public Joint {
   private:
    const short int sign;
    const double qMin, qMax, dqMin, dqMax, tauMin, tauMax;
    int encoderCounts = 4096;  //Encoder counts per turn
    int outShaftEncCnt = 24000;  // Output shaft encoder counts per turn
    double reductionRatio = 144.0;  //to be calibrated

    double Ipeak;               //!< Drive max current (used in troque conversion)
    double motorTorqueConstant; //!< Motor torque constant

    public:
    // Motor shaft
    double driveUnitToJointPosition(int driveValue) { return sign * driveValue * (2. * M_PI) / (double)encoderCounts / reductionRatio; };
    int jointPositionToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * (double)encoderCounts * reductionRatio; };
    
    // Output shaft
    double outShaftUnitToJointPosition(int outSftValue) { return sign * outSftValue * (2. * M_PI) / (double)outShaftEncCnt; };
    int jointPositionToOutShaftUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * (double)outShaftEncCnt; };


    // The unit of driveValue is rpm
    double driveUnitToJointVelocity(int driveValue) { return sign * driveValue / reductionRatio; };
    int jointVelocityToDriveUnit(double jointValue) { return reductionRatio * jointValue; };
    
    // driveValue is the per tousand value of motor rated torque (see EPOS4 doc)
    double driveUnitToJointTorque(int driveValue) { return sign * driveValue / 1000.0 * Ipeak * motorTorqueConstant * reductionRatio; };
    int jointTorqueToDriveUnit(double jointValue) { return sign * jointValue * 1000.0 / Ipeak / motorTorqueConstant / reductionRatio; };

    /**
     * \brief motor drive position control profile paramaters, user defined.
     *
     */
    motorProfile posControlMotorProfile{100, 10000, 10000};

   
    JointKE(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, double i_peak = 45.0 /*Kinco FD123 peak current*/ , double motorTorqueConstant_ = 0.132 /*SMC series constant*/, EPOS4Drive *drive = NULL, const std::string& name="");
    ~JointKE();
    /**
     * \brief Cehck if current velocity and torque are within limits.
     *
     * \return OUTSIDE_LIMITS if outside the limits (!), SUCCESS otherwise
     */
    setMovementReturnCode_t safetyCheck();

    ControlMode setMode(ControlMode driveMode_, motorProfile controlMotorProfile);

    setMovementReturnCode_t setPosition(double qd);
    setMovementReturnCode_t setVelocity(double dqd);
    setMovementReturnCode_t setTorque(double taud);

    double getSpringPosition();
    double getSpringVelocity();

    void setPosOffset(double posOffset);

    bool initNetwork();
};

#endif
