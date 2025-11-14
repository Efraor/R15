#pragma once
#include "main.h"

// Motores principales
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// Rollers / intake
extern pros::Motor intake;
extern pros::Motor roller;

// Pistones
extern pros::adi::DigitalOut piston1Brazo;
extern pros::adi::DigitalOut piston2Loader;
extern pros::adi::DigitalOut piston3Puerta;

// Funciones
void move_roller(int power);
void No_move_roller();
void robot_move(int power);
