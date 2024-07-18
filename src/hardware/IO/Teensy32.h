/*/**
 * \file Teensy.h
 * \author Justin Fong
 * \brief  Class representing a Robotous Force Torque sensor. 
 * 
 *    NOTE: this is not a CANOpen Device, and the PDO-like messages are send on non-standard COB-IDs
 * 
 * \version 0.1
 * \date 2021-01-12
 * \copyright Copyright (c) 2021
 *
 */

#ifndef TEENSY_H_INCLUDED
#define TEENSY_H_INCLUDED

#include <CANopen.h>
#include <CO_command.h>
#include <string.h>

#include <Eigen/Dense>

#include "InputDevice.h"
#include "logging.h"
#include "RPDO.h"
#include "TPDO.h"

class Teensy : public InputDevice {
   private:
        int commandID;     //   COB-ID of command messages    
        int responseID1; // COB-ID of 1st received message
        int responseID2;  // COB-ID of 2nd received message
        int responseID3;  // COB-ID of 2nd received message
        int responseID4; // COB-ID of 1st received message
        int responseID5;  // COB-ID of 2nd received message
        int responseID6;  // COB-ID of 2nd received message


        bool streaming=false; 

        // Raw Data store
        // Because the Robotous people are stupid, one of the variables is split over the two messages. So we have to
        // store the raw data and then convert it
        // RespH: [D1 D2 D3 D4 D5 D6 D7 D8]
        // [0x10, Fx_u, Fx_l, Fy_u, Fy_l, Fz_u, Fz_l, Tx_u] 
        // RespL: [D9 D10 D11 D12 D13 D14 D15 D16]
        // [Tx_l, Ty_u, Ty_l, Tz_u, Tz_l, OL_status, 0x00, 0x00]

        // Objects representing the PDOs (used to create the PDOs in the OD)
        RPDO *rpdo1;
        RPDO *rpdo2;
        RPDO *rpdo3;
        RPDO *rpdo4;
        RPDO *rpdo5;
        RPDO *rpdo6;


        /// Raw data - these variables are linked to the PDOs
        UNSIGNED8 rawData_A[24] = {0};
        UNSIGNED8 rawData_B[24] = {0};
        UNSIGNED8 cmdData = 0;
        UNSIGNED32 cmdDataPad = 0; // This is to make sure that the message is the full 8 bytes because of Robotous' not-CANopen implementation

        // Number of mapped parameters for RPDOs (lengthData) and TPDO (lengthCmd)
        UNSIGNED8 lengthData =8; // 8 for each of the RPDOs - I cheat and reuse this variable
        UNSIGNED8 lengthCmd = 2; // Second one is for padding

        float accl_range = 100.;
        float gyro_range = 2000.;
        float oren_roll_range = 180.;
        float oren_pitch_range = 90.;
        float oren_yaw_range = 360.;
        float oren_quant_range = 1.;

        // OD Parameters
        // Will need to be modified to take into number of items, data size and location
        // Data size and number of items will be constant, function will be used to change location
        // Data variables
        Eigen::VectorXd accl_A;
        Eigen::VectorXd gyro_A;
        Eigen::VectorXd orien_A;
        Eigen::VectorXd linAccel;  // Linear acceleration of sensor id 0 for teensy upper leg, sensor id 1 for teensy lower leg

        Eigen::VectorXd accl_B;
        Eigen::VectorXd gyro_B;
        Eigen::VectorXd orien_B;

        float range_mapping(float msg_val, float sensor_range, float msg_max);

       public:
        /**
        * \brief Sets up the Robotous sensor, including data storage and setting up PDOs
        *
        * \param commandID_ the COB-ID used to send messages to this device
        * \param responseID1_ the COB-ID of the first data message (sent from this device) 
        * \param responseID2_ the COB-ID of the second data message (sent from this device) 
        */
        Teensy(int responseID1_, int responseID2_, int responseID3_, int responseID4_, int responseID5_, int responseID6_);

        /**
         * \brief Sets up the receiving PDOs (note: will have issues if commands are sent, as the response are on the same COB-IDs)
         * 
         */
        bool configureMasterPDOs();

        /**
         * \brief Updates the forces from the raw data
         * 
         */
        void updateInput();

        /**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd X,Y,Z forces
         */
        Eigen::VectorXd& getAccl(int senosr_id);

        /**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd 
         */
        Eigen::VectorXd& getGyro(int senosr_id);

        /**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd 
         */
        Eigen::VectorXd& getOrien(int senosr_id);

        Eigen::VectorXd& getLinAccl();
};
#endif