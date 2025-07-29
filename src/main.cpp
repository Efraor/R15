#include "main.h"
#include "lemlib/api.hpp"


// Control
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Grupo de motores chassis
pros::MotorGroup leftMotors({-11, -14},pros::MotorGearset::green); // Grupo de motores izquierdo - puerto 11 (reversa), 14(reversa)
pros::MotorGroup rightMotors({12, 13}, pros::MotorGearset::green); // Grupo de motores derecgos - puerto 12 y 13


pros::MotorGroup arm ({9,10}, pros::MotorGearset::red); // Grupo de motores para el brazo puerto 9 y 10

pros::Motor roller(15, pros::MotorGearset::green); // Ruedas de gomas puerto 15

pros::adi::DigitalOut piston('C');   // Piston puerto "C"

pros::adi::DigitalIn limit('F');    // Limit switch puerto "F"

//Sensor inercial
pros::Imu imu(18);  //Puerto 18


// Esto no usamos por el momento pero es para usar los tracking wheels


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
//pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// Configuracion del drivetrain 
lemlib::Drivetrain drivetrain(&leftMotors,  // Grupo de motores izquierdo
                              &rightMotors, // Grupo de motores derecho
                              12.5,         // 12.5 inch track width(la distancia entre las líneas centrales de dos ruedas en el mismo eje en pulgadas)
                              lemlib::Omniwheel::NEW_325, // rueda omni de 3,25
                              333.33,       // drivetrain rpm is 333.33
                              2             // la deriva horizontal es 2. Si tuviéramos ruedas de tracción, habría sido 8
);

// controlador de movimiento lateral
lemlib::ControllerSettings linearController(10,     // ganancia proporcional (kP)
                                            0,      // ganancia integral (kI)
                                            3,      // ganancia integral (kI)
                                            3,      // anti windup
                                            1,      // rango de error pequeño, en pulgadas
                                            100,    // tiempo de espera para rango de error pequeño, en milisegundos
                                            3,      // rango de error grande, en pulgadas
                                            500,    // tiempo de espera para rango de error grande, en milisegundos
                                            60      // aceleración máxima (giro)
);

// Controlador de movimiento angular
lemlib::ControllerSettings angularController(2,     // ganancia proporcional (kP)
                                             0,     // ganancia integral (kI)
                                             10,    // ganancia integral (kI)
                                             3,     // anti windup
                                             1,     // rango de error pequeño, en grados
                                             100,   // tiempo de espera para rango de error pequeño, en milisegundos
                                             3,     // rango de error grande, en grados
                                             500,   // tiempo de espera para rango de error grande, en milisegundos
                                             0      // aceleración máxima (giro)
);

// sensores para la odometria
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, Establecida en nullptr ya que no tenemos una segunda
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, Establecida en nullptr ya que no tenemos una segunda
                            &imu    // Sensor inercial
);

// curva de entrada para la entrada del acelerador durante el control del conductor
lemlib::ExpoDriveCurve throttleCurve(1, // Banda muerta del joystick de 127
                                     2, // salida mínima donde la transmisión se moverá fuera de 127
                                     1 // ganancia de la curva
);

// curva de entrada para la entrada de dirección durante el control del conductor
lemlib::ExpoDriveCurve steerCurve(1, // Banda muerta del joystick de 127
                                  2, // salida mínima donde la transmisión se moverá fuera de 127
                                  1 // ganancia de la curva
);

// creamios el chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/*
*   Ejecuta el código de inicialización. Esto ocurre en cuanto se inicia el programa.
*   Todos los demás modos de competición están bloqueados por la inicialización; se recomienda
*   mantener el tiempo de ejecución de este modo por debajo de unos pocos segundos.
*/
void initialize() {
    pros::lcd::initialize(); // Inicia la pantalladel cerebro

    imu.reset(); // inicia la calibración del IMU
    pros::lcd::print(0, "Calibrando IMU...");

    // Esperar hasta que termine de calibrarse (tarda unos segundos)
    while (imu.is_calibrating()) {
        pros::delay(100); // esperar un poco para no saturar la CPU
    }

    pros::lcd::print(1, "IMU calibrado!");

    chassis.calibrate(); // Calibra sensores
    piston.set_value(false);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread para mostrar información en la pantalla del controlador principal y guardar datos de posición.
    pros::Task screenTask([&]() {
        while (true) {
            // imprime la ubicación del robot en la pantalla del cerebro
            pros::lcd::print(2, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(3, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(4, "Theta: %f", chassis.getPose().theta); // heading
            // telemetría de posición del registro
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay para ahorrar recursos
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

void autonomous() {
    pros::lcd::print(3, "Empezando autonomo...");

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

void opcontrol() {
    bool pistonState = false;
    // Control
    // bucle para actualizar continuamente los motores
    while (true) {
        // obtener posiciones del joystick
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // Mover el robot en modo tanque
        chassis.tank(leftY, rightY);


        // Control del brazo
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            arm.move(127); // girar arm hacia adelante
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            arm.move(-127); // girar arm hacia atrás
        } else {
            arm.brake(); // detener arm si no se presiona nada
        }
            // Control del pistón
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            piston.set_value(true); // extender
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            piston.set_value(false); // retraer 
        }

            // Control del rodillo
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            roller.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            roller.move(-127);
        } else {
            roller.brake();
        }
        // delay para ahorrar recursos
        pros::delay(25);
    }
}