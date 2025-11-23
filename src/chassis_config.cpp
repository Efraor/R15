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
lemlib::ControllerSettings linearController(10,0,15,3,1,100,3,500,60);

// PID angular
lemlib::ControllerSettings angularController(4.2,0,35,3,1,100,1.5,500,0);

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

    pros::Task screenTask([&]() {
        while (true) {
            // -------------------------
            // ODOMETRÍA (LemLib)
            // -------------------------
            pros::lcd::print(2, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(3, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(4, "Theta: %.2f", chassis.getPose().theta);
            // -------------------------
            // IMU - PITCH
            // -------------------------
            double pitch = imu.get_pitch();     // -180 a 180
            double absPitch = fabs(pitch);      // magnitud absoluta
            pros::lcd::print(5, "Pitch: %.2f", pitch);
            pros::lcd::print(6, "|Pitch|: %.2f", absPitch);
            // -------------------------
            // Estado estimado (para pruebas)
            // -------------------------
            const double umbralSubida = 6.0;   // pruébalo y ajusta
            const double umbralBajada = 2.0;
            const char* estado = "Piso";
            if (absPitch > umbralSubida)
                estado = "SUBIENDO RAMPA";
            else if (absPitch < umbralBajada)
                estado = "PLANO / ARRIBA";
            pros::lcd::print(7, "Estado: %s", estado);
            // -------------------------
            // Telemetría LemLib (PC)
            // -------------------------
            lemlib::telemetrySink()->info("Pose: {}", chassis.getPose());
            lemlib::telemetrySink()->info("Pitch: {}", pitch);
            pros::delay(50);
        }
    });
    piston1Brazo.set_value(false);
    piston2Loader.set_value(true);
    piston3Puerta.set_value(false);
}
