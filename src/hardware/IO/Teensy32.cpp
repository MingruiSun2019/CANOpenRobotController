#include "Teensy32.h"

#define INT16_MAX 32767

Teensy::Teensy(int responseID1_, int responseID2_, int responseID3_, int responseID4_, int responseID5_, int responseID6_) {
    spdlog::info("Robotous Sensor Created");

    // Change the parameters
    responseID1 = responseID1_;
    responseID2 = responseID2_;
    responseID3 = responseID3_;
    responseID4 = responseID4_;
    responseID5 = responseID5_;
    responseID6 = responseID6_;

    // Initialise variables as zeros
    accl_A = Eigen::VectorXd::Zero(3);
    gyro_A = Eigen::VectorXd::Zero(3);
    orien_A = Eigen::VectorXd::Zero(4);
    linAccel = Eigen::VectorXd::Zero(4);
    accl_B = Eigen::VectorXd::Zero(3);
    gyro_B = Eigen::VectorXd::Zero(3);
    orien_B = Eigen::VectorXd::Zero(4);
}

bool Teensy::configureMasterPDOs() {

    //spdlog::info("Teensy {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaH, &RPDOMapParamH, RPDOCommEntryH, dataStoreRecordH, RPDOMapParamEntryH));
    UNSIGNED16 dataSize[8] = {1,1,1,1,1,1,1,1};
    void *dataEntryAccl_A[8] = {(void *)&rawData_A[0],
                            (void *)&rawData_A[1],
                            (void *)&rawData_A[2],
                            (void *)&rawData_A[3],
                            (void *)&rawData_A[4],
                            (void *)&rawData_A[5],
                            (void *)&rawData_A[6],
                            (void *)&rawData_A[7]};
    void *dataEntryGyro_A[8] = {(void *)&rawData_A[8],
                          (void *)&rawData_A[9],
                          (void *)&rawData_A[10],
                          (void *)&rawData_A[11],
                          (void *)&rawData_A[12],
                          (void *)&rawData_A[13],
                          (void *)&rawData_A[14],
                          (void *)&rawData_A[15]};
    void *dataEntryOrien_A[8] = {(void *)&rawData_A[16],
                          (void *)&rawData_A[17],
                          (void *)&rawData_A[18],
                          (void *)&rawData_A[19],
                          (void *)&rawData_A[20],
                          (void *)&rawData_A[21],
                          (void *)&rawData_A[22],
                          (void *)&rawData_A[23]};
    rpdo1 = new RPDO(responseID1, 0xff, dataEntryAccl_A, dataSize, lengthData);
    rpdo2 = new RPDO(responseID2, 0xff, dataEntryGyro_A, dataSize, lengthData);
    rpdo3 = new RPDO(responseID3, 0xff, dataEntryOrien_A, dataSize, lengthData);

    // For sensor B
    void *dataEntryAccl_B[8] = {(void *)&rawData_B[0],
                            (void *)&rawData_B[1],
                            (void *)&rawData_B[2],
                            (void *)&rawData_B[3],
                            (void *)&rawData_B[4],
                            (void *)&rawData_B[5],
                            (void *)&rawData_B[6],
                            (void *)&rawData_B[7]};
    void *dataEntryGyro_B[8] = {(void *)&rawData_B[8],
                            (void *)&rawData_B[9],
                            (void *)&rawData_B[10],
                            (void *)&rawData_B[11],
                            (void *)&rawData_B[12],
                            (void *)&rawData_B[13],
                            (void *)&rawData_B[14],
                            (void *)&rawData_B[15]};
    void *dataEntryOrien_B[8] = {(void *)&rawData_B[16],
                            (void *)&rawData_B[17],
                            (void *)&rawData_B[18],
                            (void *)&rawData_B[19],
                            (void *)&rawData_B[20],
                            (void *)&rawData_B[21],
                            (void *)&rawData_B[22],
                            (void *)&rawData_B[23]};
    rpdo4 = new RPDO(responseID4, 0xff, dataEntryAccl_B, dataSize, lengthData);
    rpdo5 = new RPDO(responseID5, 0xff, dataEntryGyro_B, dataSize, lengthData);
    rpdo6 = new RPDO(responseID6, 0xff, dataEntryOrien_B, dataSize, lengthData);

    spdlog::debug("TPOD and RPDO of Robotous initialised");
    return true;
}

float Teensy::range_mapping(float msg_val, float sensor_range, float msg_max) {
    return (msg_val / (msg_max /  sensor_range));
    //return msg_val / 100.;
}

void Teensy::updateInput() {
    // If the last command was a streamed command, update the local copy of forces

        int16_t accl_A_x = rawData_A[0] * 256 + rawData_A[1];
        int16_t accl_A_y = rawData_A[2] * 256 + rawData_A[3];
        int16_t accl_A_z = rawData_A[4] * 256 + rawData_A[5];
        int16_t linAccl_x = rawData_A[6] * 256 + rawData_A[7];

        int16_t gyro_A_x = rawData_A[8] * 256 + rawData_A[9];
        int16_t gyro_A_y = rawData_A[10] * 256 + rawData_A[11];
        int16_t gyro_A_z = rawData_A[12] * 256 + rawData_A[13];
        int16_t linAccl_y = rawData_A[14] * 256 + rawData_A[15];

        int16_t orien_A_w = rawData_A[16] * 256 + rawData_A[17];
        int16_t orien_A_x = rawData_A[18] * 256 + rawData_A[19];
        int16_t orien_A_y = rawData_A[20] * 256 + rawData_A[21];
        int16_t orien_A_z = rawData_A[22] * 256 + rawData_A[23];

        int16_t accl_B_x = rawData_B[0] * 256 + rawData_B[1];
        int16_t accl_B_y = rawData_B[2] * 256 + rawData_B[3];
        int16_t accl_B_z = rawData_B[4] * 256 + rawData_B[5];
        int16_t linAccl_z = rawData_B[6] * 256 + rawData_B[7];
        
        int16_t gyro_B_x = rawData_B[8] * 256 + rawData_B[9];
        int16_t gyro_B_y = rawData_B[10] * 256 + rawData_B[11];
        int16_t gyro_B_z = rawData_B[12] * 256 + rawData_B[13];
        int16_t linAccl_vec = rawData_B[14] * 256 + rawData_B[15];

        int16_t orien_B_w = rawData_B[16] * 256 + rawData_B[17];
        int16_t orien_B_x = rawData_B[18] * 256 + rawData_B[19];
        int16_t orien_B_y = rawData_B[20] * 256 + rawData_B[21];
        int16_t orien_B_z = rawData_B[22] * 256 + rawData_B[23];

        accl_A[0] = range_mapping(accl_A_x, accl_range, INT16_MAX);
        accl_A[1] = range_mapping(accl_A_y, accl_range, INT16_MAX);
        accl_A[2] = range_mapping(accl_A_z, accl_range, INT16_MAX);
        gyro_A[0] = range_mapping(gyro_A_x, gyro_range, INT16_MAX);
        gyro_A[1] = range_mapping(gyro_A_y, gyro_range, INT16_MAX);
        gyro_A[2] = range_mapping(gyro_A_z, gyro_range, INT16_MAX);
        orien_A[0] = range_mapping(orien_A_w, oren_quant_range, INT16_MAX);
        orien_A[1] = range_mapping(orien_A_x, oren_quant_range, INT16_MAX);
        orien_A[2] = range_mapping(orien_A_y, oren_quant_range, INT16_MAX);
        orien_A[3] = range_mapping(orien_A_z, oren_quant_range, INT16_MAX);

        accl_B[0] = range_mapping(accl_B_x, accl_range, INT16_MAX);
        accl_B[1] = range_mapping(accl_B_y, accl_range, INT16_MAX);
        accl_B[2] = range_mapping(accl_B_z, accl_range, INT16_MAX);
        gyro_B[0] = range_mapping(gyro_B_x, gyro_range, INT16_MAX);
        gyro_B[1] = range_mapping(gyro_B_y, gyro_range, INT16_MAX);
        gyro_B[2] = range_mapping(gyro_B_z, gyro_range, INT16_MAX);
        orien_B[0] = range_mapping(orien_B_w, oren_quant_range, INT16_MAX);
        orien_B[1] = range_mapping(orien_B_x, oren_quant_range, INT16_MAX);
        orien_B[2] = range_mapping(orien_B_y, oren_quant_range, INT16_MAX);
        orien_B[3] = range_mapping(orien_B_z, oren_quant_range, INT16_MAX);

        linAccel[0] = range_mapping(linAccl_x, accl_range, INT16_MAX);
        linAccel[1] = range_mapping(linAccl_y, accl_range, INT16_MAX);
        linAccel[2] = range_mapping(linAccl_z, accl_range, INT16_MAX);
        linAccel[3] = range_mapping(linAccl_vec, accl_range, INT16_MAX);


        // This is not that efficient if data is stopped. This could be modified to do a check first
        /*
        UNSIGNED16 Fx = rawData_A[1] * 256 + rawData_A[2];
        UNSIGNED16 Fy = rawData_A[3] * 256 + rawData_A[4];
        UNSIGNED16 Fz = rawData_A[5] * 256 + rawData_A[6];
        UNSIGNED16 Tx = rawData_A[7] * 256 + rawData_A[8];
        UNSIGNED16 Ty = rawData_A[9] * 256 + rawData_A[10];
        UNSIGNED16 Tz = rawData_A[11] * 256 + rawData_A[12];

        accl_A[0] = static_cast<INTEGER16> (Fx)/50.0;
        accl_A[1] = static_cast<INTEGER16>(Fy) / 50.0;
        accl_A[2] = static_cast<INTEGER16>(Fz) / 50.0;

        gyro_A[0] = static_cast<INTEGER16> (Tx)/2000.0 ;
        gyro_A[1] = static_cast<INTEGER16>(Ty) / 2000.0 ;
        gyro_A[2] = static_cast<INTEGER16>(Tz) / 2000.0 ;
        */
    // Else, don't do anything
}

Eigen::VectorXd& Teensy::getAccl(int sensor_id) {
    if (sensor_id == 0) {
        return accl_A;
    } else {
        return accl_B;
    }
}

Eigen::VectorXd& Teensy::getGyro(int sensor_id) {
    if (sensor_id == 0) {
        return gyro_A;
    } else {
        return gyro_B;
    }
}

Eigen::VectorXd& Teensy::getOrien(int sensor_id) {
    if (sensor_id == 0) {
        return orien_A;
    } else {
        return orien_B;
    }
}

Eigen::VectorXd& Teensy::getLinAccl() {
    return linAccel;
}
