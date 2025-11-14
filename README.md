# 🤖 Robot 15 Pulgadas — VEX V5 (PROS + LemLib)

Proyecto de control avanzado para robot VEX V5 utilizando el framework **PROS** y la librería **LemLib**, implementado con una arquitectura modular y rutinas de autónomos optimizadas. Este repositorio incluye control manual, autonómicos completos, uso de IMU, PID de movimiento y estructura limpia para trabajo en equipo.

---

# 🧭 Descripción

Este proyecto está desarrollado para un robot de **15 pulgadas de Spark Robótica** con:

- PROS para gestión del firmware y control bajo nivel  
- LemLib para:
  - Odometría por IMU
  - Control PID lineal y angular
  - Movimientos avanzados: moveToPoint, moveToPose, turnToHeading, swingToHeading
  - Telemetría integrada
- Código **totalmente modularizado**  
- Rutinas de autonómicos listas para competencia  
- Control manual con toggles, rollers e intake  

---

# 📁 Estructura del Proyecto

/include
/robot
motors.hpp # Declaración de motores, pistones y helpers
chassis_config.hpp # Configuración LemLib (PID, IMU, drivetrain)
utils.hpp # formatDecimal, corrección, helpers
autonomous.hpp # Declaración de rutinas autonómicas
opcontrol.hpp # Declaración del control manual

/src
main.cpp # Entry point de PROS (initialize/auto/opcontrol)
motors.cpp
chassis_config.cpp
utils.cpp
autonomous.cpp
opcontrol.cpp

/src/oldCodes # Código antiguo archivado (no se compila)


---

# 🚀 Funcionalidades Principales

### ✔ Control manual
- Tanque directo
- Deadzone suave
- Toggle de pistones (Y, X, A)
- Control de rollers e intake
- Odometría en tiempo real (X, Y, θ) en la pantalla del control

### ✔ Autónomos completos
Incluye:

- `autonomous()` — principal  
- `autonomous2()`  
- `autonomous3()`  
- `autonomous4()`  
- `skills()`  
- `skills2()`  

Optimizado con PID y moveToPose/moveToPoint.

### ✔ Chasis configurado profesionalmente
- PID lineal y angular
- IMU en puerto 17
- Drivetrain con 3.25" omnis
- Curvas exponenciales para teleoperado

### ✔ Función de corrección automática
Perfecta para parking o ajustes finos al final del autonómico.

---

# 🛠️ Instalación y Uso

Clona este repositorio:

```bash
git clone https://github.com/Efraor/R15.git

