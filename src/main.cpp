#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
//#include "lemlib-tarball/api.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, -14},
                            pros::MotorGearset::green); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({12, 13}, pros::MotorGearset::green); // right motor group - ports 6, 7, 9 (reversed)

pros::MotorGroup rollers ({21,10}, pros::MotorGearset::green);
pros::Motor motor(15, pros::MotorGearset::green);
pros::adi::DigitalOut piston('C');
pros::adi::DigitalIn limit('F');

// Inertial Sensor on port 10
pros::Imu imu(18);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
//pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              333.33, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            60 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(1, // joystick deadband out of 127
                                     2, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(1, // joystick deadband out of 127
                                  2, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    imu.reset(); // iniciar calibración del IMU
    pros::lcd::print(0, "Calibrando IMU...");

    // Esperar hasta que termine de calibrarse (tarda unos segundos)
    while (imu.is_calibrating()) {
        pros::delay(100); // esperar un poco para no saturar la CPU
    }

    pros::lcd::print(1, "IMU calibrado!");

    chassis.calibrate(); // calibrate sensors
    piston.set_value(false);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(2, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(3, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(4, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(pathrue_txt); // '.' replaced with "_" to make c++ happy
ASSET(path22_txt); // '.' replaced with "_" to make c++ happy



/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
/*
void autonomous() {
    chassis.setPose(0, 0, 0);

    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(40, 0, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    //chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    //chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    //chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    //chassis.waitUntil(10);
    //pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    //chassis.waitUntilDone();
    //pros::lcd::print(4, "pure pursuit finished!");
}
*/    

    /*
void autonomous() {
    pros::lcd::print(3, "Starting autonomous...");

    // Mientras el chasis sigue el path, mostramos posición en el control
    pros::Task displayTask([]() {
        while (true) {
            auto pose = chassis.getPose();
            controller.print(0, 0, "X: %.2f", pose.x);
            controller.print(1, 0, "Y: %.2f", pose.y);
            controller.print(2, 0, "Theta: %.2f", pose.theta);
            pros::delay(100);
        }
    });
    
    // Seguir el path generado en path_jerryioo.txt con lookahead 15 y timeout 4000ms
    chassis.follow(path_jerryioo_txt, 15, 10000, false);

    // Esperar hasta que haya recorrido 10 pulgadas del path
    chassis.waitUntil(10);
    pros::lcd::print(4, "Recorrió 10 pulgadas!");

    // Esperar hasta que termine todo el movimiento
    chassis.waitUntilDone();
    pros::lcd::print(5, "Ruta terminada!");
}
*/



//Este anda a la perfeccion

/*
void autonomous() {
    pros::lcd::print(3, "Starting autonomous...");

    chassis.setPose(62.877, -23.617, 270);

    
    // Seguir el path generado en path_jerryioo.txt con lookahead 15 y timeout 4000ms
    chassis.follow(pathrue_txt, 15, 20000, true);

    // Esperar hasta que haya recorrido 10 pulgadas del path
   // chassis.waitUntil(10);
   // pros::lcd::print(4, "Recorrió 10 pulgadas!");

    // Esperar hasta que termine todo el movimiento
    //chassis.waitUntilDone();
    //pros::lcd::print(5, "Ruta terminada!");

    //pros::delay(150);
}

*/



void autonomous() {
    pros::lcd::print(3, "Starting autonomous...");

    chassis.setPose(63.74, 23.129, 270);

    
    // Seguir el path generado en path_jerryioo.txt con lookahead 15 y timeout 4000ms
    chassis.follow(path22_txt, 15, 30000, true);

    // Esperar hasta que haya recorrido 10 pulgadas del path
   // chassis.waitUntil(10);
   // pros::lcd::print(4, "Recorrió 10 pulgadas!");

    // Esperar hasta que termine todo el movimiento
    //chassis.waitUntilDone();
    //pros::lcd::print(5, "Ruta terminada!");

    //pros::delay(150);
}





/*
void autonomous() {
    pros::lcd::print(3, "Starting autonomous...");

    chassis.setPose(63.915, -22.96, 270);

    chassis.follow(decoder["ecoder1"], 15, 6000);
    chassis.follow(decoder["ecoder2"], 15, 6000);
    piston.set_value(true);
    chassis.follow(decoder["ecoder3"], 15, 6000);





    // Esperar hasta que haya recorrido 10 pulgadas del path
   // chassis.waitUntil(10);
   // pros::lcd::print(4, "Recorrió 10 pulgadas!");

    // Esperar hasta que termine todo el movimiento
    //chassis.waitUntilDone();
    //pros::lcd::print(5, "Ruta terminada!");

    //pros::delay(150);
}

*/


/*
void autonomous() {
    pros::lcd::print(3, "Starting autonomous...");

    chassis.setPose(0, 0, 0);

    chassis.moveToPose(0, 96.003, 0, 8000);

    chassis.turnToHeading(90, 2000);

    chassis.moveToPose(45, 96.003, 90, 8000);

    chassis.turnToHeading(180, 2000);

    chassis.moveToPose(45, 24, 180, 8000);

    chassis.turnToHeading(270, 2000);

    chassis.moveToPose(0, 24, 270, 8000);

    chassis.turnToHeading(0, 2000);

    chassis.moveToPose(0, 0, 0, 8000, {
        .forwards = false,
        .maxSpeed = 127,
        .minSpeed = 60
    });








    //chassis.moveToPose(25.876, -29.33, 270,8000);

    //chassis.turnToHeading(0, 2000);

    //chassis.moveToPose(25.876, 66.673, 0,8000);



    //chassis.follow(path_jerry4_txt, 8, 20000, true);

    
    pros::delay(150);
}

*/

/**
 * Runs in driver control
 */
void opcontrol() {
    bool pistonState = false;
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);


        // Control de los rollers
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            rollers.move(127); // girar rollers hacia adelante
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            rollers.move(-127); // girar rollers hacia atrás
        } else {
            rollers.brake(); // detener rollers si no se presiona nada
        }
            // Control del pistón
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            piston.set_value(true); // extender
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            piston.set_value(false); // retraer 
        }

            // Control del motor suelto
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            motor.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            motor.move(-127);
        } else {
            motor.brake();
        }
        // delay to save resources
        pros::delay(25);
    }
}