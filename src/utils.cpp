#include "robot/utils.hpp"
#include "robot/chassis_config.hpp"
#include "robot/motors.hpp"
#include <sstream>
#include <cmath>

std::string formatDecimal(double value){
    std::ostringstream s;
    s.precision(2);
    s << std::fixed << value;
    return s.str();
}

// Corrección del parking
void correccion() {
    double targetHeading = 180.0;
    int duration = 1230;
    int startTime = pros::millis();

    while (pros::millis() - startTime < duration) {
        double currentHeading = fmod(chassis.getPose().theta, 360.0);
        if (currentHeading < 0) currentHeading += 360.0;

        if (currentHeading >= 160 && currentHeading <= 200) {
            robot_move(-80);
        } else {
            robot_move(0);
            chassis.turnToHeading(targetHeading, 1000, {.maxSpeed = 70});
        }
        pros::delay(20);
    }
    robot_move(0);
}


void correccionMach() {
    double targetHeading = 0;
    int duration = 1250;
    int startTime = pros::millis();

    while (pros::millis() - startTime < duration) {
        double currentHeading = fmod(chassis.getPose().theta, 360.0);
        if (currentHeading < 0) currentHeading += 360.0;

        if (currentHeading >= 160 || currentHeading <= 200) {
            robot_move(-70);
            if (chassis.getPose().y == 1.5) {
                robot_move(0); 
            }
        } else {
            robot_move(0);
            chassis.turnToHeading(targetHeading, 1000, {.maxSpeed = 70});
        }
        pros::delay(20);
    }
    robot_move(0);
}


void estacionarSubidaBajada(double yObjetivo) {
    const double headingObjetivo = 180;

    // Umbrales IMU (ajusta según tu robot)
    const double umbralSubida = 6.0;   // empieza a subir
    const double umbralBajada = 2.0;   // vuelve a estar plano

    // Umbral de odometría
    const double toleranciaY = 1.5;    // cerca de la posición deseada
    const double toleranciaHeading = 4;

    bool detectoSubida = false;
    bool detectoBajada = false;

    int start = pros::millis();
    const int tiempoMax = 2800; // tiempo total del intento

    while (pros::millis() - start < tiempoMax) {

        // ---- Lectura de sensores ----
        double y = chassis.getPose().y;
        double heading = chassis.getPose().theta;
        double pitch = imu.get_pitch();  // o get_roll()

        // Normalizar heading
        heading = fmod(heading, 360.0);
        if (heading < 0) heading += 360.0;

        // Error del heading
        double errorH = heading - headingObjetivo;
        if (errorH > 180) errorH -= 360;
        if (errorH < -180) errorH += 360;

        // ---------- 1. Corrección de heading ----------
        if (fabs(errorH) > toleranciaHeading) {
            robot_move(0);
            chassis.turnToHeading(headingObjetivo, 300, {.maxSpeed = 40});
            continue;
        }

        // ---------- 2. Detectar subida ----------
        if (!detectoSubida && fabs(pitch) > umbralSubida) {
            detectoSubida = true;
        }

        // ---------- 3. Detectar bajada ----------
        if (detectoSubida && !detectoBajada && fabs(pitch) < umbralBajada) {
            detectoBajada = true;
        }

        // ---------- 4. Condición final de estacionado ----------
        // Caso A: Detectó subida y bajada (ya pasó la rampa y llegó al plato)
        if (detectoSubida && detectoBajada) {
            robot_move(0);
            break;
        }

        // Caso B: Odómetro dice que está en la ZONA del estacionado
        if (fabs(y - yObjetivo) <= toleranciaY) {
            robot_move(0);
            break;
        }

        // ---------- 5. Movimiento de avance ----------
        double distY = fabs(y - yObjetivo);
        int velocidad = -90;  // base

        // Ajuste por distancia (para no pasarse)
        if (distY > 10) velocidad = -90;
        else if (distY > 5) velocidad = -80;
        else velocidad = -70;

        // Ajuste por inclinación (cuando está subiendo → más lento)
        if (fabs(pitch) > umbralSubida) velocidad = -60;

        robot_move(velocidad);

        pros::delay(20);
    }

    robot_move(0);
}

void loaderLoad(int number){
    move_roller(100);
    robot_move(55);
    pros::delay(400);

    for (int i = 0; i < number; i++) {
        robot_move(50);
        pros::delay(200);
        robot_move(0);
        pros::delay(200);
    }
}