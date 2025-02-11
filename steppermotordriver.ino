#include <Arduino.h>
#include <AccelStepper.h>

// ESP32 Pins für X-Achse
#define STEP_PIN_X     33  // Step-Pin
#define DIR_PIN_X      23  // Richtungspin
#define ENDSTOP_X      32  // Endstop-Pin für X-Achse

// ESP32 Pins für Y-Achse
#define STEP_PIN_Y     26  // Step-Pin
#define DIR_PIN_Y      27  // Richtungspin
#define ENDSTOP_Y      34  // Endstop-Pin für Y-Achse

// Not-Aus (E-Stop) Pin
#define ESTOP_PIN      25  // Ungenutzter Pin mit Pull-up, E-Stop (normally open)

// Maximale Verfahrstrecke in mm
#define MAX_TRAVEL_MM  220
#define STEPS_PER_MM   80
#define MAX_STEPS      (MAX_TRAVEL_MM * STEPS_PER_MM)

// Initialisierung der Stepper-Instanzen
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

void setup() {
    delay(1000);
    Serial.begin(115200);

    // Pins als Eingang setzen
    pinMode(ENDSTOP_X, INPUT_PULLUP);
    pinMode(ENDSTOP_Y, INPUT_PULLUP);
    pinMode(ESTOP_PIN, INPUT_PULLUP); // Not-Aus mit Pull-up

    // Geschwindigkeit und Beschleunigung setzen
    stepperX.setMaxSpeed(15000);
    stepperX.setAcceleration(12000);
    stepperY.setMaxSpeed(15000);
    stepperY.setAcceleration(12000);

    Serial.println("Schrittmotoren bereit!");
}

void homeAxis(AccelStepper &stepper, int endstopPin) {
    Serial.println("Starte Homing...");

    stepper.setSpeed(-800);
    while (digitalRead(endstopPin) == LOW) {
        stepper.runSpeed();
    }

    Serial.println("Endstop erreicht. Rückzug...");
    delay(200);

    stepper.setSpeed(400);
    stepper.move(200);
    while (stepper.distanceToGo() > 0) {
        stepper.run();
    }
    delay(200);

    stepper.setSpeed(-200);
    while (digitalRead(endstopPin) == LOW) {
        stepper.runSpeed();
    }

    stepper.move(500);
    stepper.runToPosition();
    stepper.setCurrentPosition(0);
    Serial.println("Homing abgeschlossen.");
}

void moveToPosition(float x_mm, float y_mm) {
    long x_steps = x_mm * STEPS_PER_MM;
    long y_steps = y_mm * STEPS_PER_MM;

    if (x_steps < 0 || x_steps > MAX_STEPS || y_steps < 0 || y_steps > MAX_STEPS) {
        Serial.println("error target out of reach");
        return;
    }

    stepperX.moveTo(x_steps);
    stepperY.moveTo(y_steps);

    while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
        stepperX.run();
        stepperY.run();
    }
    Serial.printf("reached %l %l\n", x_mm, y_mm);
    
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "home") {
            homeAxis(stepperX, ENDSTOP_X);
            homeAxis(stepperY, ENDSTOP_Y);
            Serial.println("homing complete");
        }
        else if (command.startsWith("pos ")) {
            float x, y;
            sscanf(command.c_str(), "pos %f %f", &x, &y);
            moveToPosition(x, y);
        }
    }
}