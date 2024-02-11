#include "vex.h"

using namespace vex;

// Global instance
competition Competition;

//Motor Confguration (*right side motors are put in reverse*)
vex::motor_group leftMotors = vex::motor_group(leftMotor1, leftMotor2, leftMotor3);
vex::motor_group rightMotors = vex::motor_group(rightMotor1, rightMotor2, rightMotor3);

// Set the Controller
vex:: controller Controller1

// Define PID constants
const double kP = 0.1;
const double kI = 0.001;
const double kD = 0.01;

// Initialize error
int prevError = 0;
int totalError = 0;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void userControl(void){
    while(1){
        //Basic Movement
        while(true){
            //speed = how far stick is pushed
            int leftSpeed = leftStickY.position(); 
            int rightSpeed = rightStickY.position();

            leftMotors.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct)
            rightMotors.spin(vex::directionType::rev, rightSpeed, vex::velocityUnits::pct)

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
            if(Controller1.ButtonL2.pressing()){
                openWings();
            }

            if(Controller1.ButtonUp.pressing()){
                closeWings();
            }
        }
    }
}

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

void autonomous() {
    // Set the desired values for autnomous movement
    desiredValue = 300;
    desiredTurnValue = 0;

    // Enable PID
    enableDrivePID = true;

    // Reset sensor values
    resetDriveSensors = true;

    // Wait for sensor reset
    vex::task::sleep(20)

    // Wait for car to reach desired position
    while(averagePosition <= desiredValue){
        pidControl();
        vex::task::sleep(20)
    }

    // Disable PID
    enableDrivePID = false;

    // Stop the motors
    leftMotors.stop();
    rightMotors.stop();
}

void pidControl(){
    // Get the average position from motors
    int averageLeftPosition = (leftMotor1.position(degrees) + leftMotor2.position(degrees) + leftMotor3.position(degrees)) / 3);
    int averageRightPosition = (rightMotor1.position(degrees) + rightMotor2.position(degrees) + rightMotor3.position(degrees)) / 3);

    //Get the average of both
    int averagePosition = (averageLeftPosition + averageRightPosition) / 2;

    //Calculate error and update PID variables
    int error = desiredValue - averagePosition;
    int derivative = error - prevError;
    totalError += error;

    // Calculate the PID output
    double lateralMotorPower = kP * error + kI * totalError + kD * derivative;

    // Update motor speeds
    leftMotors.spin(vex::directionType::fwd, rightSpeed + lateralMotorPower, vex::velocityUnits::pct);
    rightMotors.spin(vex::directionType::rev, rightSpeed + lateralMotorPower, vex::velocityUnits::pct);

    // Update PID again
    prevError = error;

    //Sleep for a bit
    vex::task::sleep(20);
}

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