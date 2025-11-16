#include "robot/motors.hpp"

// Motores del chasis
pros::MotorGroup leftMotors({-18, -19}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({16, 15}, pros::MotorGearset::green);

// Motores para rollers
pros::Motor intake(-20, pros::MotorGearset::green);
pros::Motor roller(1, pros::MotorGearset::green);

// Pistones
pros::adi::DigitalOut piston1Brazo('G');
pros::adi::DigitalOut piston2Loader('H');
pros::adi::DigitalOut piston3Puerta('F');

// Funciones
void move_roller(int number){
    intake.move(number);
    roller.move(number*0.7);
}

void No_move_roller(){
    intake.move(0);
    roller.move(0);
}

void robot_move(int number){
    leftMotors.move(number);
    rightMotors.move(number);
}
