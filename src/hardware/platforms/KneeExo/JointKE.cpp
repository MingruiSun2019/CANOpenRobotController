#include "JointKE.h"


JointKE::JointKE(int jointID, double q_min, double q_max, short int sign_, double dq_min, double dq_max, double tau_min, double tau_max, double i_peak, double motorTorqueConstant_, EPOS4Drive *drive, const std::string& name) :   Joint(jointID, q_min, q_max, drive, name),
                                                                                                                                                                sign(sign_),
                                                                                                                                                                qMin(q_min), qMax(q_max),
                                                                                                                                                                dqMin(dq_min), dqMax(dq_max),
                                                                                                                                                                tauMin(tau_min), tauMax(tau_max),
                                                                                                                                                                Ipeak(i_peak),
                                                                                                                                                                motorTorqueConstant(motorTorqueConstant_)
                                                                                                                                                                {
                                                                                                                                                                    spdlog::debug("MY JOINT ID: {} ({})", this->id, name);
                                                                                                                                                                }

JointKE::~JointKE() {
    // This delete is because drives aren't instantiated in Knee Exo (as they are in other examples. )
    delete drive;
}

setMovementReturnCode_t JointKE::safetyCheck() {
    if (velocity > dqMax || velocity < dqMin) {
        return OUTSIDE_LIMITS;
    }
    if (torque > tauMax || torque < tauMin) {
        return OUTSIDE_LIMITS;
    }
    return SUCCESS;
}

setMovementReturnCode_t JointKE::setPosition(double qd) {
    calibrated = 1;
    if (calibrated) {
        if (qd >= qMin && qd <= qMax && std::isfinite(qd)) {
                if (actuated) {
                    if (std::isfinite(qd)) {
                        if (driveMode == CM_POSITION_CONTROL) {

                            spdlog::debug("q0: {}, q: {}, q2: {}", q0, qd, jointPositionToDriveUnit(qd + q0));

                            drive->setPos(jointPositionToDriveUnit(qd + q0));
                            //drive->posControlExecuteToggle();
                            return SUCCESS;
                        } else {
                            return INCORRECT_MODE;
                        }
                    }
                    else {
                        spdlog::error("Joint {} set position to incorrect value ({})", id, qd);
                        return OUTSIDE_LIMITS;
                    }
                }
                return UNACTUATED_JOINT;
        } else {
            return OUTSIDE_LIMITS;
        }
    } else {
        return NOT_CALIBRATED;
    }
}

setMovementReturnCode_t JointKE::setVelocity(double dqd) {
    //Position protection first only if calibrated
    if (calibrated) {
        if (position <= qMin && dqd < 0) {
            dqd = 0;
        }
        if (position >= qMax && dqd > 0) {
            dqd = 0;
        }
    }
    //Caped velocity
    if (dqd >= dqMin && dqd <= dqMax && std::isfinite(dqd)) {
        return Joint::setVelocity(dqd);
    } else {
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointKE::setTorque(double taud) {
    //Position protection first only if calibrated
    if (calibrated) {
        if (position <= qMin && taud < 0) {
            taud = 0;
        }
        if (position >= qMax && taud > 0) {
            taud = 0;
        }
    }
    //Caped torque
    if (taud >= tauMin && taud <= tauMax && std::isfinite(taud)) {
        return Joint::setTorque(taud);
    } else {
        return OUTSIDE_LIMITS;
    }
}

bool JointKE::initNetwork() {
    spdlog::debug("JointKE::initNetwork()");
    if (((EPOS4Drive *)drive)->init(posControlMotorProfile)) {
        return true;
    } else {
        return false;
    }
}
