#include "robot/chassis_config.hpp"
#include "robot/motors.hpp"

// IMU
pros::Imu imu(17);

// Drivetrain
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    12.5,
    lemlib::Omniwheel::NEW_325,
    466.66,
    2
);

// PID lineal
lemlib::ControllerSettings linearController(10,0,3,3,1,100,3,500,60);

// PID angular
lemlib::ControllerSettings angularController(4,0,40,3,1,100,3,500,0);

// Odometría (solo IMU)
lemlib::OdomSensors sensors(nullptr,nullptr,nullptr,nullptr,&imu);

// Curvas manejo
lemlib::ExpoDriveCurve throttleCurve(1,2,1);
lemlib::ExpoDriveCurve steerCurve(1,2,1);

// Chasis
lemlib::Chassis chassis(
    drivetrain,
    linearController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);

void chassis_setup(){
    pros::lcd::initialize();
    imu.reset();
    pros::lcd::print(0, "Calibrando IMU...");
    chassis.calibrate();

    while(imu.is_calibrating()) pros::delay(100);
    pros::lcd::print(1, "IMU calibrado!");

    piston1Brazo.set_value(false);
    piston2Loader.set_value(false);
    piston3Puerta.set_value(false);
}
