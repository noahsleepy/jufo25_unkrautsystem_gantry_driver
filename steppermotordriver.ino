#include <Arduino.h>
#include <AccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// ESP32 Pins for X-Axis
#define STEP_PIN_X     33  
#define DIR_PIN_X      23  
#define ENDSTOP_X      32  

// ESP32 Pins for Y-Axis
#define STEP_PIN_Y     26  
#define DIR_PIN_Y      27  
#define ENDSTOP_Y      34  

// Laser Control Pin
#define LASER_PIN      22  

// Home button pin
#define HOME_BUTTON_PIN 4   

#define MAX_TRAVEL_MM  220
#define STEPS_PER_MM   80
#define MAX_STEPS      (MAX_TRAVEL_MM * STEPS_PER_MM)

AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

// Buffered coordinate storage
float coordXBuffer[10], coordYBuffer[10];
int coordCount = 0;
bool homingActive = false;
bool calibrationMode = false;
int calibrationStep = 0;

// Laser and movement control
bool newTargetAvailable = false;
float targetX = 0, targetY = 0;
unsigned long lastPosCommandTime = 0;
uint8_t laser_state = 0;

//--------------------------------------------------------
// Homing function
void homeAxis(AccelStepper &stepper, int endstopPin) {
    homingActive = true;
    stepper.setSpeed(-1600);
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
    homingActive = false;
}

//--------------------------------------------------------
// Laser control function
void updateLaserState(bool state) {
    digitalWrite(LASER_PIN, state ? HIGH : LOW);
    if (laser_state != state) {
        Serial.println(state ? "Laser ON" : "Laser OFF");
        laser_state = state;
    }
}

//--------------------------------------------------------
// Move steppers to a position
void moveToPosition(float x, float y) {
    stepperX.moveTo(x * STEPS_PER_MM);
    stepperY.moveTo(y * STEPS_PER_MM);
    while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
        stepperX.run();
        stepperY.run();
    }
}

//--------------------------------------------------------
// Process Buffered Coordinates
void processBufferedCoordinates() {
    if (coordCount < 10) return;  

    float sumX = 0, sumY = 0;
    for (int i = 0; i < 10; i++) {
        sumX += coordXBuffer[i];
        sumY += coordYBuffer[i];
    }
    float centerX = sumX / 10;
    float centerY = sumY / 10;

    // Validate all coordinates are within 30mm of the center
    for (int i = 0; i < 10; i++) {
        float dx = coordXBuffer[i] - centerX;
        float dy = coordYBuffer[i] - centerY;
        if (sqrt(dx * dx + dy * dy) > 30.0) {
            coordCount = 0; // Reset buffer
            return;
        }
    }

    // Move to calculated center
    Serial.printf("Moving to: %.2f, %.2f\n", centerX, centerY);
    moveToPosition(centerX, centerY);
    updateLaserState(true);
    delay(2000); // Laser ON for 2 sec

    // Return to home position
    moveToPosition(0, 0);
    updateLaserState(false);
    coordCount = 0; // Reset buffer
}

//--------------------------------------------------------
// Serial processing
void serialTask(void *pvParameters) {
    while (true) {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            
            if (command.startsWith("pos ")) {
                float x, y;
                sscanf(command.c_str(), "pos %f %f", &x, &y);

                if (!homingActive && !calibrationMode) {
                    coordXBuffer[coordCount] = x;
                    coordYBuffer[coordCount] = y;
                    coordCount++;

                    if (coordCount == 10) {
                        processBufferedCoordinates();
                    }
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//--------------------------------------------------------
// Calibration function
void calibrationRoutine() {
    float positions[4][2] = { {0, 0}, {220, 0}, {220, 220}, {0, 220} };
    
    if (calibrationStep < 4) {
        moveToPosition(positions[calibrationStep][0], positions[calibrationStep][1]);
        calibrationStep++;
    } else {
        moveToPosition(0, 0);
        updateLaserState(false);
        calibrationStep = 0;
        calibrationMode = false;
    }
}

//--------------------------------------------------------
// Home button task
void homeButtonTask(void *pvParameters) {
    unsigned long pressStartTime = 0;
    bool buttonPressed = false;

    while (true) {
        if (digitalRead(HOME_BUTTON_PIN) == LOW) {
            if (!buttonPressed) {
                pressStartTime = millis();
                buttonPressed = true;
            } else if (millis() - pressStartTime >= 2000) {
                updateLaserState(true);
                delay(200);
                updateLaserState(false);
            }
        } else {
            if (buttonPressed) {
                if (millis() - pressStartTime >= 2000) {
                    calibrationMode = true;
                    calibrationStep = 0;
                    updateLaserState(true);
                } else {
                    Serial.println("Homing triggered");
                    homeAxis(stepperX, ENDSTOP_X);
                    homeAxis(stepperY, ENDSTOP_Y);
                    Serial.println("Homing complete");
                }
                buttonPressed = false;
            }
        }

        if (calibrationMode) {
            calibrationRoutine();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//--------------------------------------------------------
// Setup function
void setup() {
    delay(1000);
    Serial.begin(115200);

    pinMode(ENDSTOP_X, INPUT_PULLUP);
    pinMode(ENDSTOP_Y, INPUT_PULLUP);
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    pinMode(HOME_BUTTON_PIN, INPUT_PULLUP);

    stepperX.setMaxSpeed(20000);
    stepperX.setAcceleration(20000);
    stepperY.setMaxSpeed(20000);
    stepperY.setAcceleration(20000);

    Serial.println("ready");

    xTaskCreatePinnedToCore(serialTask, "SerialTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(homeButtonTask, "HomeButtonTask", 4096, NULL, 1, NULL, 1);
}

//--------------------------------------------------------
// Main loop does nothing
void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}