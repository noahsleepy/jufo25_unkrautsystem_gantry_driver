#include <Arduino.h>
#include <AccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <Adafruit_NeoPixel.h>

// ESP32 Pins for X-Axis
#define STEP_PIN_X     33  
#define DIR_PIN_X      23  
#define ENDSTOP_X      32  

// ESP32 Pins for Y-Axis
#define STEP_PIN_Y     26  
#define DIR_PIN_Y      27  
#define ENDSTOP_Y      34  

// Laser and LED Control Pins
#define LASER_PIN      22  
#define LED_PIN        5  // Free pin for the WS2812 strip
#define NUM_LEDS       65  

// Home button pin
#define HOME_BUTTON_PIN 4   

#define MAX_TRAVEL_MM  220
#define STEPS_PER_MM   80
#define MAX_STEPS      (MAX_TRAVEL_MM * STEPS_PER_MM)

AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

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
// LED control function
void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

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
    Serial.printf("moving to %.2f, %.2f\n", centerX, centerY);
    moveToPosition(centerX, centerY);
    updateLaserState(true);
    delay(2000); // Laser ON for 2 sec

    // Return to home position
    updateLaserState(false);
    moveToPosition(0, 0);
    coordCount = 0; // Reset buffer
}

//--------------------------------------------------------
// Serial processing
void serialTask(void *pvParameters) {
    while (true) {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            
            // Coordinate format validation: "123, 456"
            int commaIndex = command.indexOf(',');
            if (commaIndex > 0 && command.length() > commaIndex + 1) {
                bool validFormat = true;
                for (int i = 0; i < command.length(); i++) {
                    if (i != commaIndex && !isDigit(command[i]) && command[i] != ' ') {
                        validFormat = false;
                        break;
                    }
                }
                if (validFormat) {
                    float x, y;
                    sscanf(command.c_str(), "%f, %f", &x, &y);
                    if (!homingActive && !calibrationMode) {
                        coordXBuffer[coordCount] = x;
                        coordYBuffer[coordCount] = y;
                        coordCount++;
                        Serial.printf("[%i/10] added %f, %f\n", coordCount, x, y);

                        if (coordCount == 10) {
                            processBufferedCoordinates();
                        }
                    }
                    continue;
                }
            }

            // LED control command: "laser R, G, B"
            if (command.startsWith("led ")) {
                int r, g, b;
                if (sscanf(command.c_str(), "led %d, %d, %d", &r, &g, &b) == 3) {
                    setLEDColor(r, g, b);
                    Serial.printf("LEDs set to RGB(%d, %d, %d)\n", r, g, b);
                    continue;
                }
            }

            // Invalid command
            Serial.println("Invalid command format!");
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
        
        // Wait for button press before continuing
        Serial.printf("Waiting for button press at position: %.2f, %.2f\n", 
                      positions[calibrationStep][0], positions[calibrationStep][1]);
        
        while (digitalRead(HOME_BUTTON_PIN) == HIGH) {
            vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay to avoid busy waiting
        }

        // Wait for button release
        while (digitalRead(HOME_BUTTON_PIN) == LOW) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        calibrationStep++;  // Move to next position after button press
    } else {
        moveToPosition(0, 0);
        updateLaserState(false);
        calibrationStep = 0;
        calibrationMode = false;
        Serial.println("Calibration complete.");
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
                delay(500);
                updateLaserState(false);
                delay(500);
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


    pinMode(LED_PIN, OUTPUT);
    strip.begin();

    Serial.println("ready");

    xTaskCreatePinnedToCore(serialTask, "SerialTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(homeButtonTask, "HomeButtonTask", 4096, NULL, 1, NULL, 1);
}

//--------------------------------------------------------
// Main loop does nothing
void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
