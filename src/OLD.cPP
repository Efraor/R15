// #include "main.h"
// #include "lemlib/api.hpp"

// void autonomous() {
//     pros::lcd::print(3, "Empezando autonomo...");
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); 

//     chassis.setPose(47.127, 0, 270);

//     move_roller();

//     // chassis.moveToPose(19.317, 19.005, 315, 3000, {.horizontalDrift = 2, .lead = 0.1, .maxSpeed = 90 });

//     chassis.moveToPose(23.317, 23.005, 315, 3000, {.horizontalDrift = 2, .lead = 0.1, .maxSpeed = 50 }, false);

//     pros::delay(1700);

//     chassis.turnToHeading(180,700);

//     No_move_roller();

//     chassis.moveToPose(23.317, -20.005, 180, 3000, {.horizontalDrift = 2, .lead = 0.1, .maxSpeed = 90 }, false);
//     chassis.moveToPose(23.317, -26.005, 180, 3000, {.horizontalDrift = 2, .lead = 0.1, .maxSpeed = 60 }, false);

//     pros::delay(500);

//     chassis.turnToHeading(320,800);
//     No_move_roller();


//     chassis.moveToPose(46.293, -55.3, 320, 3000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 90 }, false);

//     chassis.turnToHeading(270,700);

//     chassis.moveToPose(36, -57.3, 270, 3000, {.horizontalDrift = 2, .lead = 0.1, .maxSpeed = 90 }, false);
//     piston1.set_value(true);

//     move_roller();
//     pros::delay(6000);

// }


// void autonomous2 () {
//     pros::lcd::print(3, "Empezando autonomo...");
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); 

//     chassis.setPose(47.127, 0, 90);
//     piston2.set_value(true);
//     move_roller();
//     pros::delay(1000);
//     chassis.moveToPose(25, 0, 90, 3000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 70 });
//     pros::delay(100);
//     chassis.moveToPose(45, 0, 90, 3000, { .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 70 });
//     piston2.set_value(false);
//     pros::delay(100);
//     No_move_roller();
//     /*
    
//     chassis.turnToHeading(180,700);

//     chassis.moveToPose(53, -60, 90, 3000, { .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 90 });

//     chassis.moveToPose(53, -70, 90, 3000, { .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 90 });
//     two_roller();
//     pros::delay(1500);

//     chassis.moveToPose(53, -60, 90, 3000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 90 });
//     chassis.turnToHeading(310,700);

//     chassis.moveToPose(20.1, -27.54, 90, 3000, { .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 90 });
//     pros::delay(100);
//     move_roller();
//     pros::delay(5000);

//     */

    










// }


// void skills1 () {
//     chassis.setPose(47.127, 0, 90);
//     piston2.set_value(true);
//     pros::delay(500);
//     chassis.moveToPose(20, 0, 90, 3000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 70,}, false);
//     chassis.turnToHeading(270,1000,{},false);
//     pros::delay(100);
//     piston2.set_value(false);
//     robot_move(-100);
//     pros::delay(4000);
//     leftMotors.move(0);
//     rightMotors.move(0);
//     pros::delay(1000);

// }

// void skills2 () {
//     chassis.setPose(47.127, 0, 90);
//     piston2.set_value(true);
//     pros::delay(500);
//     // move_roller();
//     chassis.moveToPose(35, 0, 90, 2000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     chassis.turnToHeading(180,2000,{.maxSpeed = 50},false);
//     chassis.moveToPose(35, 24, 180, 2000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     chassis.turnToHeading(90,1000,{},false);
//     chassis.moveToPose(53, 24, 90, 2000, {.forwards = true, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     piston2.set_value(false);

//     chassis.moveToPose(-23.7, 24, 90, 3000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     chassis.turnToHeading(180,1000,{},false);
    
//     chassis.moveToPose(-23.7, -2, 180, 3000, {.forwards = true, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     chassis.turnToHeading(270,1000,{},false);
//     chassis.moveToPose(-49, -2, 270, 1500, {.forwards = true, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     robot_move(50);
//     pros::delay(300);

//     piston2.set_value(true);
//     pros::delay(500);

//     chassis.moveToPose(-30, -2, 270, 3000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     chassis.turnToHeading(90,1000,{},false);
//     piston2.set_value(false);
//     robot_move(-80);
//     pros::delay(3000);
// // chassis.turnToHeading(270,1000,{},false);

//     pros::delay(100);


// }

// void skills3 () {
//     chassis.setPose(47.127, 0, 90);
//     piston2.set_value(true);
//     pros::delay(500);
//     // move_roller();
//     chassis.moveToPose(35, 0, 90, 2000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     chassis.turnToHeading(180,2000,{.maxSpeed = 50},false);
//     chassis.moveToPose(35, 24, 180, 2000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     chassis.turnToHeading(90,1000,{},false);
//     chassis.moveToPose(53, 24, 90, 2000, {.forwards = true, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     piston2.set_value(false);

//     chassis.moveToPose(-62, 30, 90, 7000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);

//     chassis.turnToHeading(16,1000,{},false);

//     robot_move(-120);
//     pros::delay(3000);
// }

// void idea () {
//     chassis.setPose(47.127, 0, 90);
//     piston2.set_value(true);
//     pros::delay(500);
//     // move_roller();
//     chassis.moveToPose(35, 0, 90, 2000, {.forwards = false, .horizontalDrift = 2, .lead = 0.1,.maxSpeed = 60,}, false);
//     chassis.turnToHeading(270,5000,{.maxSpeed = 40},false);
//     piston2.set_value(false);
//     robot_move(-120);
//     pros::delay(3000);
// }

