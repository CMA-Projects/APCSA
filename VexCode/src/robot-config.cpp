#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotor1 = motor(PORT9, ratio6_1, false);
motor leftMotor2 = motor(PORT10, ratio6_1, false);
motor leftMotor3 = motor(PORT3, ratio6_1, false);
motor rightMotor1 = motor(PORT4, ratio6_1, false);
motor rightMotor2 = motor(PORT5, ratio6_1, false);
motor rightMotor3 = motor(PORT6, ratio6_1, false);
motor Intake = motor(PORT7, ratio6_1, false);
motor Catapult = motor(PORT2, ratio18_1, false);
digital_out leftWing = digital_out(Brain.ThreeWirePort.G);
digital_out rightWing = digital_out(Brain.ThreeWirePort.H);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}