#include <Arduino.h>
#include <AccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <Adafruit_NeoPixel.h>

//--------------------------------------------------------
// Configuration
//--------------------------------------------------------
#define STEP_PIN_X     33  
#define DIR_PIN_X      23  
#define ENDSTOP_X      32  

#define STEP_PIN_Y     26  
#define DIR_PIN_Y      27  
#define ENDSTOP_Y      34  

#define LASER_PIN      22  
#define HOME_BUTTON_PIN 4   

#define MAX_TRAVEL_MM  220
#define STEPS_PER_MM   80

// Create steppers (DRIVER type)
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

// Global motion control variables
volatile float targetX = 0, targetY = 0;
volatile bool inBlockingMotion = false;  // disables the motion task during blocking routines
uint8_t laser_state = 0;
bool isHoming = false;  // set true during homing
bool isCalibrating = false;
bool isHomed = false;   // remains false until homing is performed

//--------------------------------------------------------
// Laser control
//--------------------------------------------------------
void updateLaserState(bool state) {
  digitalWrite(LASER_PIN, state ? HIGH : LOW);
}

//--------------------------------------------------------
// Blocking move (used in calibration, if needed)
//--------------------------------------------------------
void moveToPositionBlocking(float x, float y) {
  stepperX.moveTo(x * STEPS_PER_MM);
  stepperY.moveTo(y * STEPS_PER_MM);
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    stepperX.run();
    stepperY.run();
  }
}

//--------------------------------------------------------
// Fast homing function for one axis (blocking)
//--------------------------------------------------------
void homeAxis(AccelStepper &stepper, int endstopPin) {
  // Increase speed for homing
  stepper.setSpeed(-3000);
  // Move toward endstop quickly
  while (digitalRead(endstopPin) == LOW) {
    stepper.runSpeed();
  }
  delay(50); // reduced delay
  // Back off a bit and then approach slowly for precision
  stepper.setSpeed(600);
  stepper.move(150);  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(50);
  // Final approach
  stepper.setSpeed(-300);
  while (digitalRead(endstopPin) == LOW) {
    stepper.runSpeed();
  }
  stepper.move(300);
  stepper.runToPosition();
  stepper.setCurrentPosition(0);
}

//--------------------------------------------------------
// Calibration routine (if needed via long button press)
//--------------------------------------------------------
void calibrationRoutine() {
  isCalibrating = true;
  float positions[4][2] = { {0, 0}, {220, 0}, {220, 220}, {0, 220} };
  static int calibrationStep = 0;
  if (calibrationStep < 4) {
    moveToPositionBlocking(positions[calibrationStep][0], positions[calibrationStep][1]);
    while (digitalRead(HOME_BUTTON_PIN) == HIGH) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    while (digitalRead(HOME_BUTTON_PIN) == LOW) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    calibrationStep++;
  } else {
    moveToPositionBlocking(0, 0);
    updateLaserState(false);
    isCalibrating = false;
  }
  
}

//--------------------------------------------------------
// Fast, non-blocking motion task
//--------------------------------------------------------
void motionTask(void *pvParameters) {
  while (true) {
    if (!inBlockingMotion) {
      stepperX.run();
      stepperY.run();
    }
    // Yield control without delaying (vTaskDelay(0) forces a yield)
    vTaskDelay(0);
  }
}

//--------------------------------------------------------
// Serial command processing
//--------------------------------------------------------
void serialTask(void *pvParameters) {
  while (true) {
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.length() == 0)
        continue;
      
      if (command.equalsIgnoreCase("on")) {
        updateLaserState(true);
      } 
      else if (command.equalsIgnoreCase("off")) {
        updateLaserState(false);
      }
      else if (command.equalsIgnoreCase("home")) {
        inBlockingMotion = true;
        isHoming = true;
        Serial.println("Homing triggered");
        homeAxis(stepperX, ENDSTOP_X);
        homeAxis(stepperY, ENDSTOP_Y);
        stepperX.setCurrentPosition(0);
        stepperY.setCurrentPosition(0);
        targetX = 0;
        targetY = 0;
        isHoming = false;
        isHomed = true;
        inBlockingMotion = false;
        Serial.println("Homing complete");
      }
      else { // Expect coordinate command: "x y"
        float x, y;
        int parsed = sscanf(command.c_str(), "%f %f", &x, &y);
        if (parsed == 2) {
          if (x >= 0 && x <= MAX_TRAVEL_MM && y >= 0 && y <= MAX_TRAVEL_MM) {
            targetX = x;
            targetY = y;
            stepperX.moveTo(targetX * STEPS_PER_MM);
            stepperY.moveTo(targetY * STEPS_PER_MM);
          }
          else {
            Serial.println("Coordinates out of range!");
          }
        }
        else {
          Serial.println("Invalid command format!");
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

//--------------------------------------------------------
// Home button task
//--------------------------------------------------------
void homeButtonTask(void *pvParameters) {
  unsigned long pressStartTime = 0;
  bool buttonPressed = false;
  
  while (true) {
    if (digitalRead(HOME_BUTTON_PIN) == LOW) {
      if (!buttonPressed) {
        pressStartTime = millis();
        buttonPressed = true;
      }
      else if (millis() - pressStartTime >= 2000) {
        updateLaserState(true);
        delay(200);
        updateLaserState(false);
        delay(200);
      }
    } 
    else {
      if (buttonPressed) {
        if (millis() - pressStartTime >= 2000) {
          isCalibrating = true;
          inBlockingMotion = true;
          updateLaserState(true);
        } 
        else {
          inBlockingMotion = true;
          isHoming = true;
          Serial.println("Homing triggered");
          homeAxis(stepperX, ENDSTOP_X);
          homeAxis(stepperY, ENDSTOP_Y);
          stepperX.setCurrentPosition(0);
          stepperY.setCurrentPosition(0);
          targetX = 0;
          targetY = 0;
          isHoming = false;
          isHomed = true;
          Serial.println("Homing complete");
          inBlockingMotion = false;
        }
        buttonPressed = false;
      }
    }
    
    if (isCalibrating) {
      calibrationRoutine();
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

//--------------------------------------------------------
// Position feedback task (rounded integers + status)
//--------------------------------------------------------
void positionFeedbackTask(void *pvParameters) {
  while (true) {
    int posX = round(stepperX.currentPosition() / (float)STEPS_PER_MM);
    int posY = round(stepperY.currentPosition() / (float)STEPS_PER_MM);
    if (isHoming || isCalibrating) {
      Serial.printf("%d %d busy\n", posX, posY);
    } else if (!isHomed) {
      Serial.printf("%d %d unhomed\n", posX, posY);
    } else {
      Serial.printf("%d %d\n", posX, posY);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//--------------------------------------------------------
// Setup
//--------------------------------------------------------
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
  xTaskCreatePinnedToCore(positionFeedbackTask, "PositionFeedbackTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(motionTask, "MotionTask", 2048, NULL, 1, NULL, 1);
}

//--------------------------------------------------------
// Main loop (empty: tasks handle everything)
//--------------------------------------------------------
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
