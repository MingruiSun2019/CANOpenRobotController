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
    int encoderCounts = 10000;  //Encoder counts per turn
    double reductionRatio = 22.;

    double Ipeak;               //!< Drive max current (used in troque conversion)
    double motorTorqueConstant; //!< Motor torque constant

    double driveUnitToJointPosition(int driveValue) { return sign * driveValue * (2. * M_PI) / (double)encoderCounts / reductionRatio; };
    int jointPositionToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointVelocity(int driveValue) { return sign * driveValue * (2. * M_PI) / 60. / 512. / (double)encoderCounts * 1875 / reductionRatio; };
    int jointVelocityToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * 60. * 512. * (double)encoderCounts / 1875 * reductionRatio; };
    double driveUnitToJointTorque(int driveValue) { return sign * driveValue / Ipeak / 1.414 * motorTorqueConstant * reductionRatio; };
    int jointTorqueToDriveUnit(double jointValue) { return sign * jointValue * Ipeak * 1.414 / motorTorqueConstant / reductionRatio; };

    /**
     * \brief motor drive position control profile paramaters, user defined.
     *
     */
    motorProfile posControlMotorProfile{10, 10000, 10000};

   public:
    JointKE(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, double i_peak = 45.0 /*Kinco FD123 peak current*/ , double motorTorqueConstant_ = 0.132 /*SMC series constant*/, EPOS4Drive *drive = NULL, const std::string& name="");
    ~JointKE();
    /**
     * \brief Cehck if current velocity and torque are within limits.
     *
     * \return OUTSIDE_LIMITS if outside the limits (!), SUCCESS otherwise
     */
    setMovementReturnCode_t safetyCheck();

    setMovementReturnCode_t setPosition(double qd);
    setMovementReturnCode_t setVelocity(double dqd);
    setMovementReturnCode_t setTorque(double taud);

    bool initNetwork();
};

#endif
