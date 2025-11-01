#include "main.h"
#include "lemlib/api.hpp"


// Control
pros::Controller controller(pros::E_CONTROLLER_MASTER);

std::string formatDecimal(double value) {
    std::ostringstream stream;
    stream.precision(2);     // 2 decimales
    stream << std::fixed << value;
    return stream.str();
}

// Grupo de motores chassis
pros::MotorGroup leftMotors({-13, -14},pros::MotorGearset::green); // Grupo de motores izquierdo - puerto 11 (reversa), 14(reversa)
pros::MotorGroup rightMotors({11, 12}, pros::MotorGearset::green); // Grupo de motores derecgos - puerto 12 y 13

// Motores para los rollers
pros::Motor intake(-2, pros::MotorGearset::green); //  puerto 15
pros::Motor roller(9, pros::MotorGearset::green); //  puerto 16
// pros::Motor roller3(-1, pros::MotorGearset::green); //  puerto 17

pros::adi::DigitalOut piston1('A');   // Piston puerto "C"
pros::adi::DigitalOut piston2('B');   // Piston puerto "C"
pros::adi::DigitalOut piston3('C');   // Piston puerto "C"

//Sensor inercial
pros::Imu imu(17);  //Puerto 18




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
                              466.66,       // drivetrain rpm is 333.33
                              2             // la deriva horizontal es 2. Si tuviéramos ruedas de tracción, habría sido 8
);

// controlador de movimiento lineal
lemlib::ControllerSettings linearController(10,     // ganancia proporcional (kP)
                                            0,      // ganancia integral (kI)
                                            3,      // ganancia derivativa (kD)
                                            3,      // anti windup
                                            1,      // rango de error pequeño, en pulgadas
                                            100,    // tiempo de espera para rango de error pequeño, en milisegundos
                                            3,      // rango de error grande, en pulgadas
                                            500,    // tiempo de espera para rango de error grande, en milisegundos
                                            60      // aceleración máxima (giro)
);

// Controlador de movimiento angular
lemlib::ControllerSettings angularController(4,     // ganancia proporcional (kP)
                                             0,     // ganancia integral (kI)
                                             40,    // ganancia integral (kI)
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
    chassis.calibrate(); // Calibra sensores
    
    // Esperar hasta que termine de calibrarse (tarda unos segundos)
    while (imu.is_calibrating()) {
        pros::delay(100); // esperar un poco para no saturar la CPU
    }

    pros::lcd::print(1, "IMU calibrado!");

    piston1.set_value(false);
    piston2.set_value(false);
    piston3.set_value(false);

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

ASSET(path1_txt); // '.' replaced with "_" to make c++ happy
ASSET(chicoskills_txt); // '.' replaced with "_" to make c++ happy

void move_roller (int number) {
    intake.move(number);
    roller.move(number);
    //roller3.move(127);
}

void No_move_roller () {
    intake.move(0);
    roller.move(0);
    //roller3.move(0);
}


void robot_move (int number) {
    leftMotors.move(number);
    rightMotors.move(number);
}




/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

void autonomous() {
    chassis.setPose(-46.403, 5.138, 0);
    chassis.moveToPoint(-46.40, 45.7, 2000, {.maxSpeed = 70} , false);
    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);
    piston2.set_value(true);
    move_roller(127);
    chassis.moveToPoint(-59, 45.7, 3000, {} , false);
    pros::delay(1500);
    No_move_roller();
    chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false} , false);
    piston2.set_value(false);

    chassis.turnToHeading(135, 1000, {.maxSpeed = 60}, false);
    chassis.moveToPoint(-11.83, 10.52, 3000, {.maxSpeed = 70} , false);
    piston3.set_value(true);
    move_roller(127);
    pros::delay(3000);
    No_move_roller();

}

void autonomous2() {
    chassis.setPose(-46.403, 5.138, 0);
    chassis.moveToPoint(-46.40, 45.7, 2000, {.maxSpeed = 70} , false);
    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);
    piston2.set_value(true);
    move_roller(127);
    chassis.moveToPoint(-61, 45.7, 3000, {} , false);
    pros::delay(2100);
    No_move_roller();
    chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false} , false);
    piston2.set_value(false);


    chassis.turnToHeading(315, 1000, {.maxSpeed = 60}, false);
    move_roller(-127);
    pros::delay(850);
    No_move_roller();
    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);
    piston2.set_value(true);
    chassis.moveToPoint(-59, 45.7, 3000, {} , false);
    move_roller(127);
    pros::delay(2100);
    No_move_roller();
    chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false} , false);
    piston2.set_value(false);

    chassis.turnToHeading(135, 1000, {.maxSpeed = 60}, false);
    piston3.set_value(true);
    chassis.moveToPose(-10.3, 8.6, -225, 3000, {.maxSpeed = 70} , false);
    move_roller(127);
    pros::delay(3000);
    No_move_roller();
    piston3.set_value(false);




    // chassis.turnToHeading(90, 1000, {.maxSpeed = 60}, false);
    // piston1.set_value(true);
    // piston3.set_value(true);
    // chassis.moveToPoint(-29.8, 45.7, 2000,{}, false);
    // move_roller(127);
    // pros::delay(4000);


}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); 

    bool estadoPiston1 = false;
    bool estadoPiston2 = false;
    bool estadoPiston3 = false;
    // Control
    // bucle para actualizar continuamente los motores
    while (true) {
        // obtener posiciones del joystick
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        leftMotors.move(leftY);
        rightMotors.move(rightY);
        // Mover el robot en modo tanque
        //chassis.tank(leftY, rightY);

        lemlib::Pose pose = chassis.getPose();
        std::string xStr = "X:" + formatDecimal(pose.x);
        std::string yStr = " Y:" + formatDecimal(pose.y);

        controller.set_text(0, 0, xStr + yStr);

        // Gira los rollers todos hacia adelante
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
            roller.move(127);
            //roller3.move(127);
        }
        // Gira los rollers todos hacia atras
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-127);
            roller.move(-127);
            //roller3.move(-127);
        }
        // Solo dos motores hacia adelante
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(127);
            roller.brake();
            // roller.move(127);
            //roller3.brake();
        }
        // Solo un motor hacia adelante
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);
            roller.brake();
            //roller3.brake();
        }
        // Frenar todos
        else {
            intake.brake();
            roller.brake();
            //roller3.brake();
        }

        
            // Control del pistón
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        estadoPiston1 = !estadoPiston1;         // Cambia de true a false
        piston1.set_value(estadoPiston1);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        estadoPiston2 = !estadoPiston2;
        piston2.set_value(estadoPiston2);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        estadoPiston3 = !estadoPiston3;
        piston3.set_value(estadoPiston3);
        }

        if (controller.get_digital_new_press(DIGITAL_DOWN)) {
            chassis.setPose(-46.403, 5.138, 0);
        }

        if (controller.get_digital_new_press(DIGITAL_B)) {

            autonomous2();

        }
        // if (controller.get_digital_new_press(DIGITAL_X)) {
        //     chassis.moveToPoint(0, 12, 2000); // moverse 12" adelante
        // }


        // delay para ahorrar recursos
        pros::delay(25);
    }
}