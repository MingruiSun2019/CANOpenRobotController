#include "RobotKE.h"

using namespace Eigen;
using namespace std;

RobotKE::RobotKE(string robot_name, string yaml_config_file) :  Robot(robot_name, yaml_config_file),
                                                                endEffTool(&KEHandle),
                                                                calibrated(false),
                                                                maxEndEffVel(2),
                                                                maxEndEffForce(60),
                                                                velFilt(2, VM3::Zero()) {
    //Check if YAML file exists and contain robot parameters
    initialiseFromYAML(yaml_config_file);

      // Set up the motor profile
    controlMotorProfile.profileVelocity = 100;  // rpm
    controlMotorProfile.profileAcceleration = 10000;  // rpm/s
    controlMotorProfile.profileDeceleration = 10000;  // rpm/s

    //TODO: to add joint specific parameters (reduction, torque constant) and associated YAML loading

    //Define the robot structure: each joint with limits and drive
    joints.push_back(new JointKE(0, qLimits[0], qLimits[1], qSigns[0], -dqMax, dqMax, -tauMax, tauMax, iPeakDrives[0], motorCstt[0], new EPOS4Drive(5), "q1"));

    // Define the force/torque sensors
    inputs.push_back(ftsensor1 = new RobotousRFT(ftSensorThighRecvID, ftSensorThighTransID1, ftSensorThighTransID2));
    inputs.push_back(ftsensor2 = new RobotousRFT(ftSensorShankRecvID, ftSensorShankTransID1, ftSensorShankTransID2));

    //Possible inputs: keyboard and joystick
    inputs.push_back(keyboard = new Keyboard());

    last_update_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}
RobotKE::~RobotKE() {
    spdlog::debug("Delete RobotKE object begins");
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    joints.clear();
    delete keyboard;
    inputs.clear();
    spdlog::debug("RobotKE deleted");
}


void RobotKE::fillParamVectorFromYaml(YAML::Node node, std::vector<double> &vec) {
    if(node){
        for(unsigned int i=0; i<vec.size(); i++)
            vec[i]=node[i].as<double>();
    }
}

bool RobotKE::loadParametersFromYAML(YAML::Node params) {
    YAML::Node params_r=params[robotName]; //Specific node corresponding to the robot

    if(params_r["dqMax"]){
        dqMax = fmin(fmax(0., params_r["dqMax"].as<double>()), 360.) * M_PI / 180.; //Hard constrained for safety
    }

    if(params["tauMax"]){
        tauMax = fmin(fmax(0., params_r["tauMax"].as<double>()), 80.); //Hard constrained for safety
    }

    fillParamVectorFromYaml(params_r["iPeakDrives"], iPeakDrives);
    fillParamVectorFromYaml(params_r["motorCstt"], motorCstt);
    fillParamVectorFromYaml(params_r["linkLengths"], linkLengths);
    fillParamVectorFromYaml(params_r["massCoeff"], massCoeff);
    fillParamVectorFromYaml(params_r["qSpringK"], springK);
    fillParamVectorFromYaml(params_r["qSpringKo"], springKo);
    fillParamVectorFromYaml(params_r["frictionVis"], frictionVis);
    fillParamVectorFromYaml(params_r["frictionCoul"], frictionCoul);

    if(params_r["qLimits"]){
        for(unsigned int i=0; i<qLimits.size(); i++)
            qLimits[i]=params_r["qLimits"][i].as<double>() * M_PI / 180.;
    }

    fillParamVectorFromYaml(params_r["qSigns"], qSigns);

    if(params_r["qCalibration"]){
        qCalibration=params_r["qCalibration"].as<double>() * M_PI / 180.;
    }

    if(params_r["qCalibrationSpring"]){
        qCalibrationSpring=params_r["qCalibrationSpring"].as<double>() * M_PI / 180.;
    }

    //Create and replace existing tool if one specified
    if(params_r["tool"]){
        if(params_r["tool"]["name"] && params_r["tool"]["length"] && params_r["tool"]["mass"]) {
            KETool *t = new KETool(params_r["tool"]["length"].as<double>(), params_r["tool"]["mass"].as<double>(), params_r["tool"]["name"].as<string>()); //Will be destroyed at end of app
            endEffTool = t;
        }
    }

    spdlog::info("Using YAML M3 parameters of {} (Tool: {}).", robotName, endEffTool->name);
    return true;
}

int RobotKE::getCommandID(){
    //spdlog::debug("Geting FT sensors ID");
    return ftsensor1->getCommandID();
}

Eigen::VectorXd& RobotKE::getForces(){
    //spdlog::debug("Geting FT sensors ID");
    return ftsensor1->getForces();
}

bool RobotKE::initialiseSensors(){
    spdlog::debug("Initialising FT sensors");
    ftsensor1->configureMasterPDOs();
    usleep(10000);
    ftsensor2->configureMasterPDOs();
    usleep(10000);
    spdlog::debug("Initialising FT sensors finished");

    return true;
}

bool RobotKE::startSensorStreaming(){
    spdlog::debug("start streaming");
    ftsensor1->startStream();
    usleep(10000);
    ftsensor2->startStream();
    usleep(10000);
    spdlog::debug("FT sensors streaming started");
    return true;
}

bool RobotKE::stopSensorStreaming(){
    spdlog::debug("stop streaming");
    ftsensor1->stopStream();
    usleep(10000);
    ftsensor2->stopStream();
    usleep(10000);
    spdlog::debug("FT sensors streaming stopped");
    return true;
}

bool RobotKE::initialiseJoints() {
    return true;
}
bool RobotKE::initialiseNetwork() {
    spdlog::debug("RobotKE::initialiseNetwork()");
    //initialiseSensors(); // ft sensors

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation

    spdlog::debug("...");
    for (int i = 0; i < 5; i++) {
        spdlog::debug(".");
        usleep(10000);
    }
    //Start node
    for (auto joint : joints) {
        joint->start();
        spdlog::debug("Joint started.");
    }

    //Enable drive
    int n = 0;
    for (auto joint : joints) {
        bool joint_ready = false;
        for (int i = 0; (i < 10) && (!joint_ready); i++) {
            joint->readyToSwitchOn();
            usleep(10000);
            joint_ready = ((joint->getDriveStatus() & 0x01) == 0x01);
        }
        #ifndef NOROBOT
        if (!joint_ready) {
            spdlog::error("M3: Failed to enable joint {} (status: {})", n, joint->getDriveStatus());
            return false;
        }
        #endif
        n++;
    }
    spdlog::debug("Drive enabled");

    updateRobot();
    spdlog::debug("Roobot updated");
    return true;
}
bool RobotKE::initialiseInputs() {
    /*nothing to do*/
    return true;
}

void RobotKE::applyCalibration(int step) {
    // \todo make step enum
    if (step == 2){
        joints[0]->setPositionOffset(qCalibration);
        calibrated = true;
        return;
    }
    else if (step == 1) {
        spdlog::debug("qCalibrationSpring: {}", qCalibrationSpring);
        ((JointKE *)joints[0])->setExtraPositionOffset(qCalibrationSpring);
        calibrated = false;
        return;
    }
}

void RobotKE::updateRobot() {
    spdlog::trace("RobotKE::updateRobot()");
    Robot::updateRobot();

    /* Implement this later
    //Update copies of end-effector values
    endEffPositions = directKinematic(getPosition());
    Matrix3d _J = J();
    endEffVelocities = _J * getVelocity();
    endEffAccelerations = calculateEndEffAcceleration();
    Matrix3d _Jtinv = (_J.transpose()).inverse();
    endEffForces = _Jtinv * getTorque();
    //Todo: improve by including friction compensation (dedicated calculation function...)
    interactionForces = endEffForces - _Jtinv * calculateGravityTorques();

    if (safetyCheck() != SUCCESS) {
        disable();
    }
    */
    last_update_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}

setMovementReturnCode_t RobotKE::safetyCheck() {
    //End-effector safeties if calibrated
    if (calibrated) {
        if (getEndEffVelocity().norm() > maxEndEffVel) {
            spdlog::error("M3: Max velocity reached ({}m.s-1)!", getEndEffVelocity().norm());
            return OUTSIDE_LIMITS;
        }
        //if(getEndEffForce().norm()>maxEndEffForce) {
        //   spdlog::error("M3: Max force reached ({}N)!", getEndEffForce().norm());
        //   return OUTSIDE_LIMITS;
        //}
    }
    //otherwise basic joint safeties
    else {
        for (unsigned int i = 0; i < 3; i++) {
            if (((JointKE *)joints[i])->safetyCheck() != SUCCESS) {
                spdlog::error("M3: Joint {} safety triggered!", i);
                return OUTSIDE_LIMITS;
            }
        }
    }
    return SUCCESS;
}

void RobotKE::printStatus() {
    std::cout << std::setprecision(3) << std::fixed << std::showpos;
    std::cout << "X=[ " << getEndEffPosition().transpose() << " ]\t";
    std::cout << "dX=[ " << getEndEffVelocity().transpose() << " ]\t";
    std::cout << "F=[ " << getEndEffForce().transpose() << " ]\t";
    std::cout <<  std::endl;
    std::cout <<  std::noshowpos;
}
void RobotKE::printJointStatus() {
    std::cout << std::setprecision(1) << std::fixed << std::showpos;
    std::cout << "q=[ " << getPosition().transpose() * 180 / M_PI << " ]\t";
    std::cout << "dq=[ " << getVelocity().transpose() * 180 / M_PI << " ]\t";
    std::cout << "tau=[ " << getTorque().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointKE *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
    std::cout <<  std::noshowpos;
}

bool RobotKE::initProfilePositionControl(motorProfile controlMotorProfile) {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointKE *)p)->setMode(CM_PROFILE_POSITION_CONTROL, controlMotorProfile) != CM_PROFILE_POSITION_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointKE *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointKE *)p)->enable();
    }

    //TODO:CHECK STATUS 0x07
    return returnValue;
}

bool RobotKE::initCyclicPositionControl(motorProfile controlMotorProfile) {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointKE *)p)->setMode(CM_CYCLIC_POSITION_CONTROL, controlMotorProfile) != CM_CYCLIC_POSITION_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointKE *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointKE *)p)->enable();
    }

    //TODO:CHECK STATUS 0x07
    return returnValue;
}

bool RobotKE::initProfileVelocityControl(motorProfile controlMotorProfile) {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointKE *)p)->setMode(CM_PROFILE_VELOCITY_CONTROL, controlMotorProfile) != CM_PROFILE_VELOCITY_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointKE *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointKE *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}

bool RobotKE::initCyclicVelocityControl(motorProfile controlMotorProfile) {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointKE *)p)->setMode(CM_CYCLIC_VELOCITY_CONTROL, controlMotorProfile) != CM_CYCLIC_VELOCITY_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointKE *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointKE *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}

bool RobotKE::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointKE *)p)->setMode(CM_TORQUE_CONTROL, controlMotorProfile) != CM_TORQUE_CONTROL) {
            // Something bad happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointKE *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointKE *)p)->enable();
    }
    //TODO:CHECK STATUS 0x07
    return returnValue;
}

setMovementReturnCode_t RobotKE::applyPosition(double position) {
    //spdlog::debug("In applyPosition");
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    calibrated = 1; // do this later
    if (!calibrated) {
        returnValue = NOT_CALIBRATED;
    } else {
        for (auto p : joints) {
            spdlog::debug("Joint {} position reference : {} ", position);
            setMovementReturnCode_t setPosCode = ((JointKE *)p)->setPosition(position);
            if (setPosCode == INCORRECT_MODE) {
                spdlog::error("Joint {} : is not in Position Control", p->getId());
                returnValue = INCORRECT_MODE;
            } else if (setPosCode != SUCCESS) {
                // Something bad happened
                spdlog::error("Joint {} position error : {} ", p->getId(), setMovementReturnCodeString[setPosCode]);
                returnValue = UNKNOWN_ERROR;
            }
            i++;
        }
    }
    return returnValue;
}

setMovementReturnCode_t RobotKE::applySpringPosition(double positions, double velocities) {
    // Make sure it is in velocity control mode
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)

    float gainP = 2;
    float gainI = 0.4;

    auto p = joints[KNEE];

    float actualSpringPos = ((JointKE *)p)->getSpringPosition();
    float targetSprigPos = positions;
    float targetSpringVel = velocities;
    // make it a PI
    spdlog::debug("targetSpringVel: {}, targetSprigPos: {}, actualSpringPos: {}", targetSpringVel, targetSprigPos, actualSpringPos);

    errAcumu += targetSprigPos - actualSpringPos;
    float targetVel = targetSpringVel + gainP * (targetSprigPos - actualSpringPos);// + gainI * errAcumu;
    spdlog::debug("Spring Position: {}, velocity control: {}", actualSpringPos, targetVel);
    spdlog::debug("P term: {}, I term: {}", gainP * (targetSprigPos - actualSpringPos), gainI * errAcumu);

    setMovementReturnCode_t setPosCode = ((JointKE *)p)->setVelocity(targetVel);
    if (setPosCode == INCORRECT_MODE) {
        spdlog::error("Joint {} : is not in Position Control", p->getId());
        returnValue = INCORRECT_MODE;
    } else if (setPosCode != SUCCESS) {
        // Something bad happened
        spdlog::error("Joint {} position error : {} ", p->getId(), setMovementReturnCodeString[setPosCode]);
        returnValue = UNKNOWN_ERROR;
    }

    return returnValue;
}

setMovementReturnCode_t RobotKE::applySpringPositionTorControl(double positions, double velocities, double acc) {
    // Make sure it is in velocity control mode
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)

    float gainP = 20;
    float gainD = 10;

    auto p = joints[KNEE];

    float actualSpringPos = ((JointKE *)p)->getSpringPosition();
    float actualSpringVel = ((JointKE *)p)->getSpringVelocity();
    float targetSprigPos = positions;
    float targetSpringVel = velocities;
    // make it a PI
    spdlog::debug("targetSpringVel: {}, targetSprigPos: {}, actualSpringPos: {}", targetSpringVel, targetSprigPos, actualSpringPos);

    float errPos = targetSprigPos - actualSpringPos;
    float errVel = targetSpringVel - actualSpringVel;
    float targetAcc = ((JointKE *)p)->jointVelocityToDriveUnit(acc) + gainP * errPos + gainD * errVel;// + gainI * errAcumu;
    float targetTor = 100*(massShankRob * pow(CoMShankRob, 2) + JShankRob)*targetAcc; // - CoMShankRob * massShankRob * gravity;
    spdlog::debug("Spring Position: {}, torque control: {}", actualSpringPos, targetTor);
    spdlog::debug("P term: {}, I term: {}", gainP * errPos, gainD * errVel);

    setMovementReturnCode_t setPosCode = ((JointKE *)p)->setTorque(targetTor);
    if (setPosCode == INCORRECT_MODE) {
        spdlog::error("Joint {} : is not in Position Control", p->getId());
        returnValue = INCORRECT_MODE;
    } else if (setPosCode != SUCCESS) {
        // Something bad happened
        spdlog::error("Joint {} position error : {} ", p->getId(), setMovementReturnCodeString[setPosCode]);
        returnValue = UNKNOWN_ERROR;
    }

    return returnValue;
}

setMovementReturnCode_t RobotKE::applyVelocity(std::vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setVelCode = ((JointKE *)p)->setVelocity(velocities[i]);
        if (setVelCode == INCORRECT_MODE) {
            spdlog::error("Joint {} : is not in Velocity Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setVelCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} velocity error : {} ", p->getId(), setMovementReturnCodeString[setVelCode]);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}
setMovementReturnCode_t RobotKE::applyTorque(std::vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointKE *)p)->setTorque(torques[i]);
        if (setTorCode == INCORRECT_MODE) {
            spdlog::error("Joint {} : is not in Torque Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setTorCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} torque error : {} ", p->getId(), setMovementReturnCodeString[setTorCode]);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

VM3 RobotKE::directKinematic(VM3 q) {
    VM3 X;

    std::vector<double> L = linkLengths;

    double F1 = (L[2] * sin(q[1]) + (L[3]+endEffTool->length) * cos(q[2]) + L[0]);

    X[0] = -F1 * cos(q[0]);
    X[1] = -F1 * sin(q[0]);
    X[2] = L[2] * cos(q[1]) - (L[3]+endEffTool->length) * sin(q[2]);

    return X;
}
VM3 RobotKE::inverseKinematic(VM3 X) {
    VM3 q;

    std::vector<double> L = linkLengths;

    //Check accessible workspace
    double normX = X.norm();
    if ((L[3] < L[2] && normX < L[2] - (L[3]+endEffTool->length)) || ((L[3]+endEffTool->length) > L[2] && normX < sqrt((L[4]+endEffTool->length) * (L[3]+endEffTool->length) - L[2] * L[2])) || normX > (L[2] + (L[3]+endEffTool->length) + L[0]) || X[0] > 0) {
        spdlog::error("RobotKE::inverseKinematic() error: Point not accessible. NaN returned.");
        q[0] = q[1] = q[2] = nan("");
        return q;
    }

    //Compute first joint
    q[0] = -atan2(X[1], -X[0]);

    //Project onto parallel mechanism plane
    VM3 tmpX;
    if (X[0] > 0) {
        //should never happen as outside of workspace...
        tmpX[0] = sqrt(X[0] * X[0] + X[1] * X[1]);
    } else {
        tmpX[0] = -sqrt(X[0] * X[0] + X[1] * X[1]);
    }
    //Remove offset along -x
    tmpX[0] = tmpX[0] + L[0];
    tmpX[1] = X[1];
    tmpX[2] = X[2];

    //Calculate joints 2 and 3
    double beta = acos((L[2] * L[2] + L[3] * L[3] - tmpX[0] * tmpX[0] - tmpX[2] * tmpX[2]) / (2. * (L[2] * L[3])));
    q[1] = acos(L[3] * sin(beta) / sqrt(tmpX[0] * tmpX[0] + tmpX[2] * tmpX[2])) - atan2(tmpX[2], -tmpX[0]);
    q[2] = M_PI / 2. + q[1] - beta;

    return q;
}
Matrix3d RobotKE::J() {
    Matrix3d J;
    VM3 q;
    for (unsigned int i = 0; i < 3; i++) {
        q(i) = ((JointKE *)joints[i])->getPosition();
    }

    std::vector<double> L = linkLengths;

    //Pre calculate factors for optimisation
    double F1 = (L[3]+endEffTool->length) * sin(q[2]);
    double F2 = -L[2] * cos(q[1]);
    double F3 = (L[3]+endEffTool->length) * cos(q[2]) + L[2] * sin(q[1]) + L[0];

    //Jacobian matrix elements
    J(0, 0) = F3 * sin(q[0]);
    J(0, 1) = F2 * cos(q[0]);
    J(0, 2) = F1 * cos(q[0]);

    J(1, 0) = -F3 * cos(q[0]);
    J(1, 1) = F2 * sin(q[0]);
    J(1, 2) = F1 * sin(q[0]);

    J(2, 0) = 0;
    J(2, 1) = -L[2] * sin(q[1]);
    J(2, 2) = -(L[3]+endEffTool->length) * cos(q[2]);

    return J;
}

VM3 RobotKE::calculateGravityTorques() {
    VM3 tau_g;

    //For convenience
    std::vector<double> L = linkLengths;
    std::vector<double> M = massCoeff;

    float g = 9.81;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet

    //Get current configuration
    VM3 q;
    for (unsigned int i = 0; i < 3; i++) {
        q(i) = ((JointKE *)joints[i])->getPosition();
    }

    //Calculate gravitational torques
    tau_g[0] = springKo[0] + springK[0]*q[0];
    tau_g[1] = M[0]*sin(q[1])*g + springKo[1] + springK[1]*q[1];
    tau_g[2] = M[1]*cos(q[2])*g + springKo[2] + springK[2]*q[2];
    tau_g += J().transpose() * VM3(0, 0, endEffTool->mass*g); //Tool gravity
    return tau_g;
}

VM3 RobotKE::calculateEndEffAcceleration() {

    VM3 endEffVelocitiesFiltered_new = VM3::Zero();
    double dt = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6) - last_update_time;

    //Filter
    if(!velFilt.isInitialised()) {
        //Initialise filter at 10Hz w/ current sampling freq (Butterworth order 2)
        if(dt<1.) { //dt not reliable at startup
            velFilt.initButter2low(10.*dt);
        }
        endEffVelocitiesFiltered = VM3::Zero();
    }
    else {
        if(isCalibrated()) {
            //Filter velocity
            if(endEffVelocities.allFinite()) {
                endEffVelocitiesFiltered_new = velFilt.filt(endEffVelocities);
            }
            else {
                spdlog::warn("RobotKE::calculateEndEffAcceleration(): Non finite velocity value, skipping filtering step.");
            }
        }
    }

    //Diff
    endEffAccelerations = (endEffVelocitiesFiltered_new - endEffVelocitiesFiltered) / dt;

    //Update value
    endEffVelocitiesFiltered = endEffVelocitiesFiltered_new;

    return endEffAccelerations;
}

const VX& RobotKE::getEndEffPosition() {
    return endEffPositions;
}
const VX& RobotKE::getEndEffVelocity() {
    return endEffVelocities;
}
const VX& RobotKE::getEndEffVelocityFiltered() {
    return endEffVelocitiesFiltered;
}
const VX& RobotKE::getEndEffAcceleration() {
    return endEffAccelerations;
}
const VX& RobotKE::getEndEffForce() {
    return endEffForces;
}
const VX& RobotKE::getInteractionForce() {
    return interactionForces;
}


setMovementReturnCode_t RobotKE::setJointPosition(double q) {
    //spdlog::debug("In RobotKE");
    return applyPosition(q);
}
setMovementReturnCode_t RobotKE::setJointVelocity(double dq) {
    std::vector<double> vel{dq};
    return applyVelocity(vel);
}
setMovementReturnCode_t RobotKE::setJointTorque(double tau) {
    std::vector<double> tor{tau};
    return applyTorque(tor);
}

setMovementReturnCode_t RobotKE::setEndEffForceWithCompensation(VM3 F, bool friction_comp) {
    if (!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if (!1) {
        return OUTSIDE_LIMITS;
    }
    //TODO: calculate joint angle (shank angle to ground) based torque compensation
    double torComp = 0;

    return setJointTorque(torComp);
}
