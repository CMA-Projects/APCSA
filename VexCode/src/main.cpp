/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// leftMotor1           motor         9               
// leftMotor2           motor         10              
// leftMotor3           motor         3               
// rightMotor1          motor         4               
// rightMotor2          motor         5               
// rightMotor3          motor         6               
// Intake               motor         7               
// Catapult             motor         2               
// leftWing             digital_out   G               
// rightWing            digital_out   H               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

vex::controller Controller1 = vex::controller();
vex::controller::axis leftStickY = Controller1.Axis3; //Axis3 --> up and down movement of left stick
vex::controller::axis rightStickY = Controller1.Axis2; // up/down movement of right stick

vex::motor_group leftMotors = vex::motor_group(leftMotor1, leftMotor2, leftMotor3);
vex::motor_group rightMotors = vex::motor_group(rightMotor1, rightMotor2, rightMotor3);


void openWings(){
  leftWing.set(true);
  rightWing.set(false);
}

void closeWings(){
  leftWing.set(false);
  rightWing.set(true);
}

void intake(double seconds, int sleep){
  Intake.spin(vex::directionType::rev, 22, vex::velocityUnits::pct);
  vex::task::sleep(seconds * 1000);
  Intake.stop();
  vex::task::sleep(sleep);
}

void outtake(double seconds, int sleep){
  Intake.spin(vex::directionType::fwd, 22, vex::velocityUnits::pct);
  vex::task::sleep(seconds * 1000);
  Intake.stop();
  vex::task::sleep(sleep);
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

// Settings
double kP = 0.1;
double kI = 0.001;
double kD = 0.01;

double turnkP = 0.1;
double turnkI = 0.001;
double turnkD = 0.01;

//Autonomous Settings
int desiredValue = 300;
int desiredTurnValue = 0;
int currentValue = 0;
int currentTurnValue = 0;

int error; //SensorValue - DesiredValue = Positional
int prevError = 0; //Position 20 milliseconds ago
int derivative; // error - prevError: Speed
int totalError; //totalError = totalError + error

int turnError; //SensorValue - DesiredValue = Positional
int turnPrevError = 0; //Position 20 milliseconds ago
int turnDerivative; // error - prevError: Speed
int turnTotalError; //totalError = totalError + error

bool resetDriveSensors = false;

// Variables modified for use
bool enableDrivePID = false;

void setMotorSpeedEithPID(vex::motor_group& motors, int targetPosition){
  int currentPosition = motors.position(vex::rotationUnits::deg);
  int error = targetPosition - currentPosition; 

  //PID Terms
  double proportional = kP * error;
  double integral = kI * error;
  double derivative = kD * (error - prevError);

  double output = proportional + integral + derivative;

  motors.spin(vex::directionType::fwd, output, vex::velocityUnits::pct);

}

int drivePID(){
  while(enableDrivePID){

    if (resetDriveSensors){
      resetDriveSensors = false;

      leftMotor1.setPosition(0, degrees);
      leftMotor2.setPosition(0, degrees);
      leftMotor3.setPosition(0, degrees);
      rightMotor1.setPosition(0, degrees);
      rightMotor2.setPosition(0, degrees);
      rightMotor3.setPosition(0, degrees);
    }
    
    // Get the positions of all left motors
    int leftMotor1Position = leftMotor1.position(degrees);
    int leftMotor2Position = leftMotor2.position(degrees);
    int leftMotor3Position = leftMotor3.position(degrees);

    // Get the positions of all right motors
    int rightMotor1Position = rightMotor1.position(degrees);
    int rightMotor2Position = rightMotor2.position(degrees);
    int rightMotor3Position = rightMotor3.position(degrees);

    //Lateral Movement PID
    ////////////////////////////////

    printf("Left Position: %d, Right Position: %d\n", leftMotor1Position, rightMotor1Position);



    // Get the average position of both motors
    int averageLeftPosition = (leftMotor1Position + leftMotor2Position + leftMotor3Position) / 3;
    int averageRightPosition = (rightMotor1Position + rightMotor2Position + rightMotor3Position) / 3;

    //Get avg of the two motors
    int averagePosition = (averageLeftPosition + averageRightPosition)/2;

    // Potential
    error = averagePosition - desiredValue;

    //Derivative
    derivative = error - prevError;
    //Integral
    totalError += error; 

    double lateralMotorPower = error * kP + derivative * kD + totalError * kI; 

    printf("lateral power: %f\n", lateralMotorPower);

    //Turning Movement PID
    //////////////////////////////////

    int turnDifference = (averageLeftPosition - averageRightPosition) / 2;

    //Potential
    turnError = turnDifference - desiredTurnValue; 
    //Derivative
    turnDerivative = turnError - turnPrevError;
    //Integral
    turnTotalError += turnError; 


    printf("Total Turn Error: %d\n", turnTotalError);

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;

    printf("Turn Motor Power: %f\n", turnMotorPower);


    /////////////////////////////////

    leftMotor1.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    leftMotor2.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    leftMotor3.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);

    rightMotor1.spin(reverse, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    rightMotor2.spin(reverse, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    rightMotor3.spin(reverse, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    // leftMotor1.spin(vex::directionType::fwd, lateralMotorPower + turnMotorPower, vex::velocityUnits::pct);
    // leftMotor2.spin(vex::directionType::fwd, lateralMotorPower + turnMotorPower, vex::velocityUnits::pct);
    // leftMotor3.spin(vex::directionType::fwd, lateralMotorPower + turnMotorPower, vex::velocityUnits::pct);
    
    // rightMotor1.spin(vex::directionType::rev, lateralMotorPower - turnMotorPower, vex::velocityUnits::pct);
    // rightMotor2.spin(vex::directionType::rev, lateralMotorPower - turnMotorPower, vex::velocityUnits::pct);
    // rightMotor3.spin(vex::directionType::rev, lateralMotorPower - turnMotorPower, vex::velocityUnits::pct);

    //code
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }
  return 1;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  
  





  //setMotorSpeedEithPID(leftMotors, 1);


  // enableDrivePID = true; 

  // drivePID();

  // resetDriveSensors = true;
  // desiredValue = 100; //forward 300
  // desiredTurnValue = 200; //turn 600
  // vex::task::sleep(1000); 
  // enableDrivePID = false;

  // resetDriveSensors = true;
  // desiredValue = 300; //forward 300
  // desiredTurnValue = 600; //turn 600


  // vex::task::sleep(1000);


  // // Stop the PID control task at the end of autonomous
  // enableDrivePID = false;
  // vex::task::sleep(1000);  // Allow time for the task to stop

  // setMotorSpeedEithPID(leftMotors, 10); 


  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    

    while (true) { // always looking for input
    // Read the joystick

      int leftSpeed = leftStickY.position(); //speed = how far stick is pushed
      int rightSpeed = rightStickY.position();
      

      // Set the motor speed after receiving the controller input variable
      leftMotor1.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
      leftMotor2.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
      leftMotor3.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
      
      rightMotor1.spin(vex::directionType::rev, leftSpeed, vex::velocityUnits::pct);
      rightMotor2.spin(vex::directionType::rev, leftSpeed, vex::velocityUnits::pct);
      rightMotor3.spin(vex::directionType::rev, leftSpeed, vex::velocityUnits::pct);

      // Catapult

      if(Controller1.ButtonL1.pressing()) {
        Catapult.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      }
      else {
        Catapult.stop(); 
      }

      // Intake

      if(Controller1.ButtonR2.pressing()){ //outtake
        Intake.spin(vex::directionType::fwd, 22, vex::velocityUnits::pct);
      }
      else if(Controller1.ButtonR1.pressing()){
        Intake.spin(vex::directionType::rev, 22, vex::velocityUnits::pct); //intake
      }
      else {
        Intake.stop(); 
      }

      // Wings

      // bool wingsOpen = false; 

      if(Controller1.ButtonL2.pressing()){
        openWings();
      }

      if(Controller1.ButtonUp.pressing()){
        closeWings();
      }

      // while (wingsOpen == true){
      //   leftWing.set(true);
      //   rightWing.set(false);
      // }
      // while (wingsOpen == false){
      //   leftWing.set(false);
      //   rightWing.set(true);
      // }

      // void toggleWings() {
      //   wingsOpen = !wingsOpen;
      // }

      

    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
