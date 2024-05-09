/**
 * @brief An implementation of the Drive Object, specifically for the Copley Drive
 *
 */
#include "EPOS4Drive.h"
#include "application.h"

#include <iostream>


EPOS4Drive::EPOS4Drive(int NodeID) : Drive::Drive(NodeID) {
    ;
}
EPOS4Drive::~EPOS4Drive() {
    spdlog::debug("EPOS4Drive Deleted");
}

bool EPOS4Drive::init() {
    spdlog::debug("NodeID {} EPOS4Drive::init()", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    resetError();
    if(initPDOs()) {
        resetError();
        return true;
    }
    return false;
}
bool EPOS4Drive::init(motorProfile profile) {
    spdlog::debug("NodeID {} EPOS4Drive::init(motorProfile profile)", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    //resetError();  //TODO: Do we really need it?
    if(setMotorProfile(profile)) {
        if(initPDOs()) {
            return true;
        }
    }
    return false;
}
bool EPOS4Drive::resetError(){
    spdlog::debug("NodeID {} reset error", NodeID);
    sendSDOMessages(generateResetErrorSDO());
    return true;
}

bool EPOS4Drive::initProfilePosControl(motorProfile posControlMotorProfile){
    spdlog::debug("NodeID {} Initialising Profile Position Control", NodeID);

    sendSDOMessages(EPOS4Drive::generateProfilePosControlConfigSDO(posControlMotorProfile));
    return true;
}

bool EPOS4Drive::initCyclicPosControl(motorProfile posControlMotorProfile){
    spdlog::debug("NodeID {} Initialising Cyclic Position Control", NodeID);

    sendSDOMessages(EPOS4Drive::generateCyclicPosControlConfigSDO(posControlMotorProfile));
    return true;
}

bool EPOS4Drive::initProfileVelControl(motorProfile velControlMotorProfile){
    spdlog::debug("NodeID {} Initialising Profile Velocity Control", NodeID);

    sendSDOMessages(EPOS4Drive::generateProfileVelControlConfigSDO(velControlMotorProfile));
    return true;
}

bool EPOS4Drive::initCyclicVelControl(motorProfile velControlMotorProfile){
    spdlog::debug("NodeID {} Initialising Cyclic Velocity Control", NodeID);

    sendSDOMessages(EPOS4Drive::generateCyclicVelControlConfigSDO(velControlMotorProfile));
    return true;
}

bool EPOS4Drive::initTorqueControl() {
    spdlog::debug("NodeID {} Initialising Torque Control", NodeID);
    sendSDOMessages(generateTorqueControlConfigSDO());

    return true;
}

std::vector<std::string> EPOS4Drive::generateCyclicPosControlConfigSDO(motorProfile positionProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //enable cyclic synchornous position mode 0x6060-00
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 8";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // set up max motor speed 0x6080-00
    sstream << "[1] " << NodeID << " write 0x6080 0 u32 " << std::dec << 4550;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // optional: set up max gear input speed 0x3003-03
    //           (depends on the gear ratio in EPOS studio, both can be set up in EPOS studio)
    // set up profile deceleration 0x6084-00
    //           (can also be set up in EPOS studio)
    sstream << "[1] " << NodeID << " write 0x6084 0 i32 " << std::dec << positionProfile.profileDeceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // optional: set up quick0-stop decelartion 0x6085-00
    //           (can also be set up in EPOS studio)

    // set up interpolation time period (should be equal to Master's SYNC period)
    //    0x60C2-1: Interpolation time period value
    //    0x60C2-2 Interpolation time index (I don't know what this is)
    // a value of 0 indicates step response for each set value, results in noisy motion
    // a value properly set to SYNC enables interpolates new set values, results in smooth motion
    sstream << "[1] " << NodeID << " write 0x60C2 1 u8 " << std::dec << CANUpdateLoopPeriodInms*2;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // set up Controlword (shutdown)
    sstream << "[1] " << NodeID << " write 0x6040 1 u16 " << 0x0006;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // set up Controlword (shutdown)
    sstream << "[1] " << NodeID << " write 0x6040 1 u16 " << 0x000F;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
};

std::vector<std::string> EPOS4Drive::generateProfilePosControlConfigSDO(motorProfile positionProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
  
    // Enable Profile Position Mode 0x6060-00 to 0x01
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 1";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Following error window 0x6065-00 (unit: inc)
    sstream << "[1] " << NodeID << " write 0x6065 0 u32 20000";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Max profile velocity 0x607F-00 (unit: rpm)
    sstream << "[1] " << NodeID << " write 0x607F 0 u32 4550";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Profile velocity 0x6081-00 (unit: rpm)
    sstream << "[1] " << NodeID << " write 0x6081 0 u32 " << std::dec << positionProfile.profileVelocity;
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Profile acceleration 0x6083-00  (unit: rpm/s)
    sstream << "[1] " << NodeID << " write 0x6083 0 u32 " << std::dec << positionProfile.profileAcceleration;
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Profile deceleration 0x6084-00  (unit: rpm/s)
    sstream << "[1] " << NodeID << " write 0x6084 0 u32 " << std::dec << positionProfile.profileDeceleration;
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Quick stop deceleration 0x6085-00  (unit: rpm/s)
    sstream << "[1] " << NodeID << " write 0x6085 0 u32 10000";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Motion profile type 0x6086-00 to 0 (Linear ramp)
    sstream << "[1] " << NodeID << " write 0x6086 0 i16 0";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set Controlword to Shutdown (0x0006)
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0006";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set Controlword to Switch on & Enable (0x000F)
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x000F";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    return CANCommands;
};

std::vector<std::string> EPOS4Drive::generateProfileVelControlConfigSDO(motorProfile velocityProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
        CANCommands.push_back(sstream.str());
        sstream.str(std::string());

        // Enable Profile Velocity Mode 0x6060-00 to 0x03
        sstream << "[1] " << NodeID << " write 0x6060 0 i8 3";
        CANCommands.push_back(sstream.str());
        sstream.str("");

        // Set up Max profile velocity 0x607F-00 (unit: rpm)
        sstream << "[1] " << NodeID << " write 0x607F 0 u32 4550";
        CANCommands.push_back(sstream.str());
        sstream.str("");

        // Set up Profile acceleration 0x6083-00 (unit: rpm/s)
        sstream << "[1] " << NodeID << " write 0x6083 0 u32 " << std::dec << velocityProfile.profileAcceleration;
        CANCommands.push_back(sstream.str());
        sstream.str("");

        // Set up Profile deceleration 0x6084-00 (unit: rpm/s)
        sstream << "[1] " << NodeID << " write 0x6084 0 u32 " << std::dec << velocityProfile.profileDeceleration;
        CANCommands.push_back(sstream.str());
        sstream.str("");

        // Set up Quick stop deceleration 0x6085-00 (unit: rpm/s)
        sstream << "[1] " << NodeID << " write 0x6085 0 u32 10000";
        CANCommands.push_back(sstream.str());
        sstream.str("");

        // Set up Motion profile type 0x6086-00 to 0 (Linear ramp)
        sstream << "[1] " << NodeID << " write 0x6086 0 i16 0";
        CANCommands.push_back(sstream.str());
        sstream.str("");

        // Set Controlword to Shutdown (0x0006)
        sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0006";
        CANCommands.push_back(sstream.str());
        sstream.str("");

        // Set Controlword to Switch on & Enable (0x000F)
        sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x000F";
        CANCommands.push_back(sstream.str());
        sstream.str("");

    return CANCommands;
};

std::vector<std::string> EPOS4Drive::generateCyclicVelControlConfigSDO(motorProfile velocityProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // Enable Cyclic Synchronous Velocity Mode 0x6060-00 to 0x09
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 9";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Max motor speed 0x6080-00
    sstream << "[1] " << NodeID << " write 0x6080 0 u32 4550";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Max gear input speed 0x3003-03
    sstream << "[1] " << NodeID << " write 0x3003 3 u32 4550";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Profile deceleration 0x6084-00
    sstream << "[1] " << NodeID << " write 0x6084 0 u32 10000";  // for stopping only
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Quick stop deceleration 0x6085-00
    sstream << "[1] " << NodeID << " write 0x6085 0 u32 10000";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set up Interpolation time period 0x60C2-xx (assuming index 1 for period)
    sstream << "[1] " << NodeID << " write 0x60C2 1 u8 " << std::dec << CANUpdateLoopPeriodInms*2;
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set Controlword to Shutdown (0x0006)
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0006";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    // Set Controlword to Switch on & Enable (0x000F)
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x000F";
    CANCommands.push_back(sstream.str());
    sstream.str("");

    return CANCommands;
};

std::vector<std::string> EPOS4Drive::generateTorqueControlConfigSDO() {
    return Drive::generateTorqueControlConfigSDO(); /*<!execute base class function*/
}

std::vector<std::string> EPOS4Drive::generateResetErrorSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // shutdown
    spdlog::debug("NodeID {} step 1", NodeID);
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x06";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    spdlog::debug("NodeID {} step 2", NodeID);

    // reset fault
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x80";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    spdlog::debug("NodeID {} step 3", NodeID);

    //TODO: check if this is correct
    // reset fault
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x06";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    spdlog::debug("NodeID {} step 4", NodeID);

    return CANCommands;
}

/* TODO
std::vector<std::string> EPOS4Drive::generatePositionOffsetSDO(int offset) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // set mode of operation
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 6";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set the home offset
    sstream << "[1] " << NodeID << " write 0x607C 0 i32 "<< std::dec << offset;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set homing method to 0
    sstream << "[1] " << NodeID << " write 0X6098 0 i8 0";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set control word to start homing
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set control word to start homing
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x1f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    return CANCommands;
}

bool EPOS4Drive::setPositionOffset(int offset) {
    spdlog::debug("NodeID {} Setting Position Offset", NodeID);

    sendSDOMessages(generatePositionOffsetSDO(offset));

    return true;


}

bool EPOS4Drive::setTrackingWindow(INTEGER32 window) {
    spdlog::debug("NodeID {} Tracking Window", NodeID);

    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    sstream << "[1] " << NodeID << " write 0x2120 0 i32 " << std::dec << window;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    sendSDOMessages(CANCommands);

    return true;
}

bool EPOS4Drive::setFaultMask(UNSIGNED32 mask) {
    spdlog::debug("NodeID {} Fault mask set to {0:x}", NodeID, mask);

    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    sstream << "[1] " << NodeID << " write 0x2182 0 i32 " << std::dec << mask;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    sendSDOMessages(CANCommands);

    return true;
}
*/

bool EPOS4Drive::initPDOs() {
    spdlog::debug("EPOS4Drive::initPDOs");

    spdlog::debug("Set up STATUS_WORD TPDO on Node {}", NodeID);
    int TPDO_Num = 1;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0xFF)) < 0) {
        spdlog::error("Set up STATUS_WORD TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up ACTUAL_POS and ACTUAL_VEL TPDO on Node {}", NodeID);
    TPDO_Num = 2;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up ACTUAL_POS and ACTUAL_VEL TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up ACTUAL_TOR TPDO on Node {}", NodeID);
    TPDO_Num = 3;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up ACTUAL_TOR TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up DIGITAL_IN RPDO on Node {}", NodeID);
    TPDO_Num = 4;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up DIGITAL_IN TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up CONTROL_WORD and DIGITAL_OUT RPDO on Node {}", NodeID);
    int RPDO_Num = 1;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up CONTROL_WORD and DIGITAL_OUT RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_POS RPDO on Node {}", NodeID);
    RPDO_Num = 2;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up TARGET_POS RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_VEL RPDO on Node {}", NodeID);
    RPDO_Num = 3;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up ARGET_VEL RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_TOR RPDO on Node {}", NodeID);
    RPDO_Num = 4;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up TARGET_TOR RPDO FAILED on node {}", NodeID);
        return false;
    }

    return true;
}

std::vector<std::string> EPOS4Drive::readSDOMessage(int address, int datetype) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // read message from drive
    sstream.str(std::string());

    switch (datetype) {
        case 1:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u8";
            break;
        case 2:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u16";
            break;
        case 3:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 i8";
            break;
        case 4:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 i32";
            break;
        default:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u8";
            break;
    }

    std::cout << sstream.str() << "\n";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    sendSDOMessages(CANCommands);
    return CANCommands;
}

std::vector<std::string> EPOS4Drive::writeSDOMessage(int address, int value) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // read message from drive
    sstream.str(std::string());
    sstream << "[1] " << NodeID << " write 0x" << std::hex << address << " 0 i32 0x" << std::hex << value;
    std::cout << sstream.str() << "\n";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

bool EPOS4Drive::posControlConfirmSP() {
    // for kinco driver, there is no need to set postion control confirm
//    DEBUG_OUT("NodeID " << NodeID << " Kinco::posControlConfirmSP")
//    Drive::posControlConfirmSP();
    return true;
}
