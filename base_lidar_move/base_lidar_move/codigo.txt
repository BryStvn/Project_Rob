#include <Stepper.h>

// Define el número de pasos por revolución del motor
#define STEPS_PER_REV 2048

// Define los pines a los que está conectado el controlador ULN2003
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;

// Crea una instancia de la clase Stepper
Stepper motor(STEPS_PER_REV, IN1, IN3, IN2, IN4);

// Variable para almacenar el número de pasos
int steps_to_turn = 0;

void setup() {
  // Configura la velocidad del motor en revoluciones por minuto (RPM)
  motor.setSpeed(10);

  // Inicia la comunicación serial
  Serial.begin(9600);
}

void loop() {
  // Espera a recibir datos por comunicación serial
  if (Serial.available() > 0) {
    // Lee el valor enviado desde Python
    steps_to_turn = Serial.parseInt();
    motor.step(steps_to_turn);
    delay(1000); // Espera 1 segundo entre movimientos
  }
}
