#include <Arduino.h>
#include <HoverboardAPI.h>
#include <Wire.h>
#include "MPU6050_tockn.h"
#include "PIDController.h"

int serialWrapper(unsigned char *data, int len) {
 return (int) Serial2.write(data,len);
}

HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);
MPU6050 mpu(Wire);

// enable debug mode
#define DEBUG

#define FOOTPAD_SENSOR_THRESHOLD 600
#define FOOTPAD_DEACTIVATE_DELAY 100

#define MAX_PWM 600

#define PUSHBACK_THRESHOLD .8
#define PUSHBACK_TIME 2000
#define PUSHBACK_SPEED 10 // degrees per second
#define PUSHBACK_AMOUNT 15 // degrees

// Board States
#define STATE_IDLE 0
#define STATE_RIDING 1
#define STATE_PUSHBACK 2

// PID Controller constants
#define KP 50
#define KI 0.005
#define KD 400
PIDController pid;

bool last_footpad = false;
int last_footpad_change;
int pushback_start_timer, pushback_end_timer;
int state = STATE_IDLE;
int targetAngle, pwm_cmd;
int pushback_start_time = 0;
int pushback_end_time = 0; // time when pushback started and ended

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial2.begin(115200);

  mpu.begin();
  mpu.setGyroOffsets(-1.78, -2.08, 0.94);

  pid.begin();
  pid.tune(KP, KI, KD);

  pinMode(PB4, OUTPUT);
  pinMode(PC14, INPUT_PULLUP);
  pid.limit(-600, 600);
}

void loop() {
  // Loop Performance Timer
  long startMicros = micros();

  //////////////// FOOTPADS //////////////// 
  bool footpad = !digitalRead(PC14);
  bool footpad_change = footpad != last_footpad;

  // record when the footpad last changed state
  if (footpad_change) {
    last_footpad_change = millis();
  }

  // if the footpad is released, and the board is not idle
  if (!footpad && state != STATE_IDLE) {
    // if the footpad has been released for more than FOOTPAD_DEACTIVATE_DELAY ms
    if (millis() - last_footpad_change > FOOTPAD_DEACTIVATE_DELAY) {
      state = STATE_IDLE;
    }
  } 

  // if the footpad is pressed, and the board is idle
  if (footpad && state == STATE_IDLE) {
    state = STATE_RIDING;
  }

  //////////////// PUSHBACK ////////////////

  // if PWM is above threshold 
  if (abs(pwm_cmd) > MAX_PWM*PUSHBACK_THRESHOLD) {
    pushback_end_timer = 0;

    if (state == STATE_RIDING) {
      // record when the PWM first went above threshold
      if (pushback_start_timer == 0) {
        pushback_start_timer = millis();
      }

      // for more than PUSHBACK_TIME ms, initiate pushback
      if (millis() - pushback_start_timer > PUSHBACK_TIME) {
        state = STATE_PUSHBACK;
        pushback_start_time = millis();
      }
    }
  } else if (abs(pwm_cmd) < MAX_PWM*PUSHBACK_THRESHOLD) {
    pushback_start_timer = 0;

    if (state == STATE_PUSHBACK) {
      // record when the PWM first went above threshold
      if (pushback_end_timer == 0) {
        pushback_end_timer = millis();
      }

      // for more than PUSHBACK_TIME ms, end pushback
      if (millis() - pushback_end_timer > PUSHBACK_TIME) {
        state = STATE_RIDING;
        pushback_end_time = millis();
      }
    }
  }

  // fade in and out of pushback 
  if (state == STATE_PUSHBACK) {
    // fade in pushback at PUSHBACK_SPEED deg/s to PUSHBACK_AMOUNT
    int pushback_duration = millis() - pushback_start_time;
    targetAngle = constrain(pushback_duration*(PUSHBACK_SPEED/1000.0), 0, PUSHBACK_AMOUNT);

  } else if (state == STATE_RIDING && pushback_start_time != 0) {
    // fade out pushback at PUSHBACK_SPEED deg/s to 0
    int end_pushback_duration = millis() - pushback_end_time;
    targetAngle = constrain(PUSHBACK_AMOUNT - end_pushback_duration*(PUSHBACK_SPEED/1000.0), 0, PUSHBACK_AMOUNT);
    
  }
  
  ///////////// LEVELING LOGIC /////////////
  // Get MPU Data
  mpu.update();
  float board_tilt = mpu.getAngleX();

  // get PWM command
  pid.setpoint(targetAngle);
  int pid_out = pid.compute(board_tilt);

  if (state != STATE_IDLE) {
    pwm_cmd = pid_out;
  } else {
    // if the board is idle, set the PWM to 0
    pwm_cmd = 0;
  }

  #ifndef DEBUG
    hoverboard.sendDifferentialPWM(pwm_cmd, -pwm_cmd, PROTOCOL_SOM_NOACK);
    // limit loop to 500Hz
  #else
    // put data in Serial Plotter compatable format
    Serial2.println("ANGLE:" + String(board_tilt) + ",TARGET:" + String(targetAngle) + ",PWM:" + String(pwm_cmd) + ",STATE:" + String(state) + ",FOOTPAD:" + String(footpad) + ",LOOP_TIME:" + String(micros() - startMicros));
  #endif

  while (micros() - startMicros < 2000);
}