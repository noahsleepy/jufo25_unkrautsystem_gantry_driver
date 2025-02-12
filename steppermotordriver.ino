#include <Arduino.h>
#include <AccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define USE_ESTOP 0

// ESP32 Pins für X-Achse
#define STEP_PIN_X     33  
#define DIR_PIN_X      23  
#define ENDSTOP_X      32  

// ESP32 Pins für Y-Achse
#define STEP_PIN_Y     26  
#define DIR_PIN_Y      27  
#define ENDSTOP_Y      34  

// Not-Aus (E-Stop) Pin
#define ESTOP_PIN      25  

// Laser Control Pin
#define LASER_PIN      22  // Change this to your desired GPIO pin

// Maximale Verfahrstrecke in mm
#define MAX_TRAVEL_MM  220
#define STEPS_PER_MM   80
#define MAX_STEPS      (MAX_TRAVEL_MM * STEPS_PER_MM)

// Initialisierung der Stepper-Instanzen
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

volatile float targetX = 0, targetY = 0;
volatile bool newTargetAvailable = false;

void moveTask(void *pvParameters);
void serialTask(void *pvParameters);

void setup() {
    delay(1000);
    Serial.begin(115200);

    pinMode(ENDSTOP_X, INPUT_PULLUP);
    pinMode(ENDSTOP_Y, INPUT_PULLUP);
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW); // Ensure laser is off at startup

    stepperX.setMaxSpeed(15000);
    stepperX.setAcceleration(12000);
    stepperY.setMaxSpeed(15000);
    stepperY.setAcceleration(12000);

    Serial.println("ready");

    xTaskCreatePinnedToCore(moveTask, "MoveTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(serialTask, "SerialTask", 4096, NULL, 1, NULL, 1);
}

void homeAxis(AccelStepper &stepper, int endstopPin) {
    stepper.setSpeed(-800);
    while (digitalRead(endstopPin) == LOW) {
        stepper.runSpeed();
    }

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
}

void moveTask(void *pvParameters) {
    while (true) {
        if (newTargetAvailable) {
            long x_steps = targetX * STEPS_PER_MM;
            long y_steps = targetY * STEPS_PER_MM;

            if (x_steps >= 0 && x_steps <= MAX_STEPS && y_steps >= 0 && y_steps <= MAX_STEPS) {
                stepperX.moveTo(x_steps);
                stepperY.moveTo(y_steps);
            } else {
                Serial.println("error target out of reach");
                newTargetAvailable = false;
                continue;
            }

            while ((stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) && newTargetAvailable) {
                stepperX.run();
                stepperY.run();
            }
            Serial.printf("reached %f %f\n", targetX, targetY);
            newTargetAvailable = false;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void serialTask(void *pvParameters) {
    while (true) {
        if (Serial.available()) {

            if (digitalRead(ESTOP_PIN) == HIGH && USE_ESTOP) {
                Serial.println("estop on");
                vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay to avoid spam (1 second)
                continue;
            }

            String command = Serial.readStringUntil('\n');
            command.trim();
            
            if (command == "home") {
                Serial.println("homing...");
                homeAxis(stepperX, ENDSTOP_X);
                homeAxis(stepperY, ENDSTOP_Y);
                Serial.println("homing complete");

            } else if (command.startsWith("pos ")) {
                float x, y;
                sscanf(command.c_str(), "pos %f %f", &x, &y);
                targetX = x;
                targetY = y;
                newTargetAvailable = true;

            } else if (command == "laser on") {
                digitalWrite(LASER_PIN, HIGH);

            } else if (command == "laser off") {
                digitalWrite(LASER_PIN, LOW);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}