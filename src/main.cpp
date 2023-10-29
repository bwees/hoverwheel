#include <Arduino.h>
#include <HoverboardAPI.h>
#include <Wire.h>
#include "MPU6050_tockn.h"
#include "PIDController.h"
#include <FastLED.h>

int serialWrapper(unsigned char *data, int len) {
 return (int) Serial2.write(data,len);
}

HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);
MPU6050 mpu(Wire);

// enable debug mode
// #define DEBUG

#define FOOTPAD_SENSOR_THRESHOLD 600
#define FOOTPAD_DEACTIVATE_DELAY 100
#define FOOTPAD_PIN PA4

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
#define KP 300
#define KI 0
#define KD 0
PIDController pid;

bool last_footpad = false;
int last_footpad_change;
int pushback_start_timer, pushback_end_timer;
int state = STATE_IDLE;
int targetAngle = 2;
int pwm_cmd;
int pushback_start_time = 0;
int pushback_end_time = 0; // time when pushback started and ended
int pushback_last_beep = -1;

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat);
float mahony_q[4] = {1.0, 0.0, 0.0, 0.0};
#define FILTER_KP 2.0
#define FILTER_KI 0
int filter_last_update = micros();

#define NUM_LEDS 12
CRGB leds[NUM_LEDS];
float hue = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial2.begin(115200);

  mpu.begin();
  mpu.setGyroOffsets(-0.96, -0.94, 0.47);

  pid.begin();
  pid.tune(KP, KI, KD);

  pinMode(PB4, OUTPUT);
  pinMode(FOOTPAD_PIN, INPUT);
  pid.limit(-600, 600);

  FastLED.addLeds<WS2812B, PC15, GRB>(leds, NUM_LEDS);  // GRB ordering is assumed
  FastLED.setBrightness(100);
}

void loop() {
  // show hue on led strip
  hue += 1;

  // Loop Performance Timer
  long startMicros = micros();

  // Get MPU Data
  mpu.update();
  int now = micros();
  // Serial.println(now - filter_last_update);
  float deltat = (now - filter_last_update) * 1.0e-6; //seconds since last update
  filter_last_update = now;

  // get MPU data
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  float gx = mpu.getGyroX() * (PI/180.0);
  float gy = mpu.getGyroY() * (PI/180.0);
  float gz = mpu.getGyroZ() * (PI/180.0);

  // update filter
  Mahony_update(ax, ay, az, gx, gy, gz, deltat);

  // Serial2.println(deltat);

  // float roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  // float pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  float board_tilt = atan2((mahony_q[0] * mahony_q[1] + mahony_q[2] * mahony_q[3]), 0.5 - (mahony_q[1] * mahony_q[1] + mahony_q[2] * mahony_q[2]));
  board_tilt *= 180.0/PI;


  int battery_voltage = hoverboard.getBatteryVoltage();
  int motor_speed = hoverboard.getSpeed0_kmh();

  //////////////// FOOTPADS //////////////// 
  bool footpad = digitalRead(FOOTPAD_PIN);
  bool footpad_change = footpad != last_footpad;

  // record when the footpad last changed state
  if (footpad_change) {
    last_footpad_change = millis();
  }

  // if the footpad is released, and the board is not idle
  // and the board speed is less than 2 km/h
  if (!footpad && state != STATE_IDLE) {
    // if the footpad has been released for more than FOOTPAD_DEACTIVATE_DELAY ms
    if (millis() - last_footpad_change > FOOTPAD_DEACTIVATE_DELAY) {
      state = STATE_IDLE;
    }
  } 

  // if the footpad is pressed, and the board is idle
  // and the board is +- 10 degrees from the target angle
  if (footpad && state == STATE_IDLE && abs(board_tilt - targetAngle) < 3) {
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
  
  // handle pushback beeps
  if (state == STATE_PUSHBACK) {
    int time_since_beep = millis() - pushback_start_time;

    if (time_since_beep > 250) {
      pushback_start_time = millis();
      hoverboard.sendBuzzer(4, 0, 50, PROTOCOL_SOM_NOACK);
    }
  }

  ///////////// LEVELING LOGIC /////////////

  // get PWM command
  pid.setpoint(targetAngle);
  int pid_out = pid.compute(board_tilt);

  if (state != STATE_IDLE) {
    pwm_cmd = pid_out;
  } else {
    // if the board is idle, set the PWM to 0
    pwm_cmd = 0;
  }


  if (pwm_cmd < 3) {
    FastLED.showColor(CHSV(0, 255, 255));
  } else {
    FastLED.showColor(CHSV(120, 255, 255));
  }


  #ifndef DEBUG
    hoverboard.sendDifferentialPWM(pwm_cmd, pwm_cmd, PROTOCOL_SOM_NOACK);
    // limit loop to 500Hz
  #else
    // put data in Serial Plotter compatable format
    // SerialBT.println(2);
    Serial2.println("ANGLE:" + String(board_tilt) + ",TARGET:" + String(targetAngle) + ",PWM:" + String(pwm_cmd) + ",STATE:" + String(state) + ",FOOTPAD:" + String(footpad) + ",LOOP_TIME:" + String(micros() - startMicros));
  #endif

  while (micros() - startMicros < 2000);
}


void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm, accelNorm;
  float qa, qb, qc;
  float vx, vy, vz;
  float ex, ey, ez;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

  //  tmp = ax * ax + ay * ay + az * az;
  accelNorm = sqrt(ax * ax + ay * ay + az * az);

  // ignore accelerometer if false (tested OK, SJR)
  if (accelNorm > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(accelNorm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = mahony_q[1] * mahony_q[3] - mahony_q[0] * mahony_q[2];
    vy = mahony_q[0] * mahony_q[1] + mahony_q[2] * mahony_q[3];
    vz = mahony_q[0] * mahony_q[0] - 0.5f + mahony_q[3] * mahony_q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (FILTER_KI > 0.0f) {
      gx += FILTER_KI * ex * deltat;  // apply integral feedback
      gy += FILTER_KI * ey * deltat;
      gz += FILTER_KI * ez * deltat;
    }

    // Apply proportional feedback to gyro term
    gx += FILTER_KP * ex;
    gy += FILTER_KP * ey;
    gz += FILTER_KP * ez;
  }

  // Integrate rate of change of mahony_q, given by gyro term
  // rate of change = current orientation mahony_q (qmult) gyro rate

  gx *= deltat/2.f;   // pre-multiply common factors
  gy *= deltat/2.f;
  gz *= deltat/2.f;
  qa = mahony_q[0];
  qb = mahony_q[1];
  qc = mahony_q[2];

  //add qmult*delta_t to current orientation
  mahony_q[0] += (-qb * gx - qc * gy - mahony_q[3] * gz);
  mahony_q[1] += (qa * gx + qc * gz - mahony_q[3] * gy);
  mahony_q[2] += (qa * gy - qb * gz + mahony_q[3] * gx);
  mahony_q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise mahony_q
  recipNorm = 1.0 / sqrt(mahony_q[0] * mahony_q[0] + mahony_q[1] * mahony_q[1] + mahony_q[2] * mahony_q[2] + mahony_q[3] * mahony_q[3]);
  mahony_q[0] = mahony_q[0] * recipNorm;
  mahony_q[1] = mahony_q[1] * recipNorm;
  mahony_q[2] = mahony_q[2] * recipNorm;
  mahony_q[3] = mahony_q[3] * recipNorm;
}