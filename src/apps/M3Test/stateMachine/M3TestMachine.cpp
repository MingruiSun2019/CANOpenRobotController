#include "M3TestMachine.h"

#define OWNER ((M3TestMachine *)owner)

M3TestMachine::M3TestMachine() {
    robot = new RobotM3();

    // Create PRE-DESIGNED State Machine events and state objects.
    testState = new M3TestState(this, robot, "Test M3");

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(testState);
}
/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 * 
 */
void M3TestMachine::init() {
    DEBUG_OUT("M3TestMachine::init()")
    robot->initialise();
    running = true;
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 * 
 */
void M3TestMachine::hwStateUpdate(void) {
    robot->updateRobot();
}
