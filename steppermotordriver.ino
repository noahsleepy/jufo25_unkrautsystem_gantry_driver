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
#define LASER_PIN      22  // Change this to your desired GPIO pin

// Home button pin for triggering homing (assumes an active LOW button)
#define HOME_BUTTON_PIN 4   // Adjust to your wiring

// Maximum travel and conversion factors
#define MAX_TRAVEL_MM  220
#define STEPS_PER_MM   80
#define MAX_STEPS      (MAX_TRAVEL_MM * STEPS_PER_MM)

// Instantiate stepper objects
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

// Global variables for the target position and control flags
volatile float targetX = 0, targetY = 0;
volatile bool newTargetAvailable = false;
// Timestamp of the last received pos command (in milliseconds)
volatile unsigned long lastPosCommandTime = 0;

//--------------------------------------------------------
// Homing function (uses the endstop pins to set zero)
void homeAxis(AccelStepper &stepper, int endstopPin) {
    // Move slowly towards the endstop
    stepper.setSpeed(-1600);
    while (digitalRead(endstopPin) == LOW) {
        stepper.runSpeed();
    }
    delay(200);
    
    // Back off a bit
    stepper.setSpeed(400);
    stepper.move(200);
    while (stepper.distanceToGo() > 0) {
        stepper.run();
    }
    delay(200);

    // Slowly approach again for accuracy
    stepper.setSpeed(-200);
    while (digitalRead(endstopPin) == LOW) {
        stepper.runSpeed();
    }

    // Final move to set position zero
    stepper.move(500);
    stepper.runToPosition();
    stepper.setCurrentPosition(0);
}

//--------------------------------------------------------
// Update the laser state based on the timing and positional conditions.
uint8_t laser_state = 0;
void updateLaserState() {
    unsigned long now = millis();
    // If a pos command came in within the last 1000 ms...
    if (now - lastPosCommandTime < 1000) {
        // Convert current stepper positions from steps to mm.
        float currentX = stepperX.currentPosition() / (float)STEPS_PER_MM;
        float currentY = stepperY.currentPosition() / (float)STEPS_PER_MM;
        float dx = currentX - targetX;
        float dy = currentY - targetY;
        float distance = sqrt(dx * dx + dy * dy);
        // Enable the laser only if within 15 mm of the target.
        if (distance <= 15.0) {
            digitalWrite(LASER_PIN, HIGH);
            if (laser_state == 0) {
                Serial.println("laser is now on");
                laser_state = 1;
            }
            
        } else {
            digitalWrite(LASER_PIN, LOW);
            if (laser_state == 1) {
                Serial.println("laser is now off");
                laser_state = 0;
            }
        }
    } else {
        // No recent pos command; ensure the laser is off.
        digitalWrite(LASER_PIN, LOW);
        if (laser_state == 1) {
            Serial.println("laser is now off");
            laser_state = 0;
        }
    }
}

//--------------------------------------------------------
// Task that moves the steppers toward the target position.
void moveTask(void *pvParameters) {
    while (true) {
        if (newTargetAvailable) {
            long x_steps = targetX * STEPS_PER_MM;
            long y_steps = targetY * STEPS_PER_MM;
            
            // Check that the target is within allowed travel range.
            if (x_steps >= 0 && x_steps <= MAX_STEPS &&
                y_steps >= 0 && y_steps <= MAX_STEPS) {
                stepperX.moveTo(x_steps);
                stepperY.moveTo(y_steps);
            } else {
                Serial.println("error target out of reach");
                newTargetAvailable = false;
                continue;
            }
            
            // Move until both steppers have reached their targets.
            while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
                if (newTargetAvailable) {
                    long newX = targetX * STEPS_PER_MM;
                    long newY = targetY * STEPS_PER_MM;
                    stepperX.moveTo(newX);
                    stepperY.moveTo(newY);
                    newTargetAvailable = false; // Clear the flag after updating
                }
                stepperX.run();
                stepperY.run();
                updateLaserState();
            }

            newTargetAvailable = false;
            Serial.printf("reached %f %f\n", targetX, targetY);
            
            
        }
        updateLaserState();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
}

//--------------------------------------------------------
// Task that processes serial input (only the "pos" command is processed).
void serialTask(void *pvParameters) {
    while (true) {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            
            // Only process commands that start with "pos "
            if (command.startsWith("pos ")) {
                float x, y;
                sscanf(command.c_str(), "pos %f %f", &x, &y);
                targetX = x;
                targetY = y;
                // Update the timestamp so we know a new position came in.
                lastPosCommandTime = millis();
                newTargetAvailable = true;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//--------------------------------------------------------
// Task that checks the homing button. When pressed, it triggers homing on both axes.
void homeButtonTask(void *pvParameters) {
    while (true) {
        // Check if the home button is pressed (assuming active LOW)
        if (digitalRead(HOME_BUTTON_PIN) == LOW) {
            Serial.println("homing triggered");
            // Optionally cancel any active movement
            newTargetAvailable = false;
            // Home both axes
            homeAxis(stepperX, ENDSTOP_X);
            homeAxis(stepperY, ENDSTOP_Y);
            Serial.println("homing complete");
            // Wait until the button is released (with debounce)
            while (digitalRead(HOME_BUTTON_PIN) == LOW) {
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//--------------------------------------------------------
// Setup: initialize pins, steppers, and tasks.
void setup() {
    delay(1000);
    Serial.begin(115200);

    pinMode(ENDSTOP_X, INPUT_PULLUP);
    pinMode(ENDSTOP_Y, INPUT_PULLUP);
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);  // Make sure the laser is off at startup
    pinMode(HOME_BUTTON_PIN, INPUT_PULLUP);  // Home button (active LOW)

    stepperX.setMaxSpeed(25000);
    stepperX.setAcceleration(30000);
    stepperY.setMaxSpeed(25000);
    stepperY.setAcceleration(30000);

    Serial.println("ready");

    // Create tasks on specific cores
    xTaskCreatePinnedToCore(moveTask, "MoveTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(serialTask, "SerialTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(homeButtonTask, "HomeButtonTask", 4096, NULL, 1, NULL, 1);
}

//--------------------------------------------------------
// The main loop does nothing but delay.
void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
