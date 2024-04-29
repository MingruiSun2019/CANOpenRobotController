/**
 * \file EPOS4Drive.h
 * \author Mingrui Sun
 * \brief  An implementation of the Drive Object, specifically for the Kinco Drive
 *
 * This class enables low level functions to the system. It does limited error
 * checking.
 * \version 0.2
 * \date 2020-07-27
 *
 * \copyright Copyright (c) 2020
 *
 */
#ifndef EPOS4DRIVE_H_INCLUDED
#define EPOS4DRIVE_H_INCLUDED
#include "Drive.h"

enum EPOS4ControlMode {
    CM_UNCONFIGURED = 0,
    CM_PROFILE_POSITION_CONTROL = 1,
    CM_CYCLIC_POSITION_CONTROL = 2,
    CM_PROFILE_VELOCITY_CONTROL = 3,
    CM_CYCLIC_VELOCITY_CONTROL = 4,
    CM_TORQUE_CONTROL = 5,
    CM_ERROR = -1,
    CM_UNACTUATED_JOINT = -2
};

/**
 * \brief An implementation of the Drive Object, specifically for EPOS4 (currently used in the knee exoskeleton)
 *
 */
class EPOS4Drive : public Drive {
   public:
    /**
         * \brief Construct a new EPOS4 Drive object
         *
         * \param NodeID CANopen Node ID
         */
    EPOS4Drive(int NodeID);

    /**
         * \brief Destroy the EPOS4 Drive object
         *
         */
    ~EPOS4Drive();
    /**
         * Setup PDOs and initialises the drive
         *
         * \return True if successful, False if not
         */
    bool init();
    /**
         * Setup PDOs and motor profile (acceleration and velocity)
         *
         * \return True if successful, False if not
         */
    bool init(motorProfile profile);
    /**
     * \todo Move jointMinMap and jointMaxMap to set additional parameters (bit 5 in 0x6041 makes updates happen immediately)
     *
     */

    /**
         * Sets the drive to Position control with default parameters (through SDO messages)
         *
         * Note: Should be overloaded to allow parameters to be set
         *
         * \return true if successful
         * \return false if not
         */
    bool initProfilePosControl(motorProfile posControlMotorProfile);
    bool initCyclicPosControl(motorProfile posControlMotorProfile);

    /**
         * Sets the drive to Velocity control with default parameters (through SDO messages)
         *
         * Note: Should be overloaded to allow parameters to be set
         *
         * \return true if successful
         * \return false if not
         */
    bool initProfileVelControl(motorProfile velControlMotorProfile);
    bool initCyclicVelControl(motorProfile velControlMotorProfile);

    /**
         * Sets the drive to Torque control with default parameters (through SDO messages)
         *
         * Note: Should be overloaded to allow parameters to be set
         *
         * \return true if successful
         * \return false if not
         */
    bool initTorqueControl();
    /**
          * \brief Overloaded method from Drive, specifically for Kinco Drive implementation.
          *     Generates the list of commands required to configure Position control in CANopen motor drive
          *
          * /param Profile Velocity, value used by position mode motor trajectory generator.
          *            Units: 0.1 counts/sec
          *            Range:0 - 500,000,000
          * /param Profile Acceleration, value position mode motor trajectory generator will attempt to achieve.
          *            Units: 10 counts/sec^2
          *            Range:0 - 200,000,000
          * /param Profile Deceleration, value position mode motor trajectory generator will use at end of trapezoidal profile.
          *             see programmers manual for other profile types use.
          *            Units: 10 counts/sec^2
          *            Range:0 - 200,000,000
          *
          *    NOTE: More details on params and profiles can be found in the CANopne CiA 402 series specifications:
          *           https://www.can-cia.org/can-knowledge/canopen/cia402/
          *
          */

    /**
        * \brief Overwrite drive class to ignore this function
        *
        * \return true The control word was previously 0 (i.e. successful set point confirm)
        * \return false The control word was previously 1 (i.e. unsuccessful set point confirm)
        */
       
    bool posControlConfirmSP();

    bool initPDOs();

    bool resetError();

    std::vector<std::string> writeSDOMessage(int address, int value);
    std::vector<std::string> readSDOMessage(int address, int len);
    std::vector<std::string> generateCyclicPosControlConfigSDO(motorProfile positionProfile);
    std::vector<std::string> generateProfilePosControlConfigSDO(motorProfile positionProfile);
    std::vector<std::string> generateProfileVelControlConfigSDO(motorProfile velocityProfile);
    std::vector<std::string> generateCyclicVelControlConfigSDO(motorProfile velocityProfile);

    std::vector<std::string> generateTorqueControlConfigSDO();
    std::vector<std::string> generateResetErrorSDO();
};

#endif
