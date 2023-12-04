#include <Arduino.h>
#include <HoverboardAPI.h>
#include <Wire.h>
#include <MPU6050.h>
#include <FastLED.h>
#include <ArduinoBLE.h>
#include <Preferences.h>
#include "config.h"
#include "mahony.h"

int serialWrapper(unsigned char *data, int len) {
  return (int) Serial.write(data,len);
}

HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);
MPU6050 mpu;

//macro that returns +1/-1 depending on the sign of the argument
#define SIGN(x) ((x > 0) - (x < 0))

// Board States
#define STATE_IDLE 0
#define STATE_RIDING 1
#define STATE_PUSHBACK 2

int last_footpad;
int last_footpad_change;
int pushback_start_timer, pushback_end_timer;
int state = STATE_IDLE;
float targetAngle = 0;
int pwm_cmd;
int pushback_start_time = 0;
int last_beep = -1;
int pushback_end_time = 0; // time when pushback started and ended
int pushback_last_beep = -1;
float step_up_start = 0;
int filter_last_update = micros();
int last_ble_update = millis();
int last_param_update = -1;

// Control loop parameters
struct {
  int kp;
  int krp;
  float fkp;
  int footpad_a_thresh;
  int footpad_b_thresh;
  float pushback_threshold;
  int pushback_time;
  float pushback_speed;
  float pushback_amount;
  float ride_angle;
  float step_up_angle;
  float step_up_speed;
  float imu_offset;
} control_params;


CRGB leds[NUM_LEDS];

// BLE
BLEService controlService("19b10000-e8f2-537e-4f6c-d104768a1215"); // create service
BLEService statsService("19a10000-e8f2-537e-4f6c-d104768a1215"); // create service

// create switch characteristic and allow remote device to read and write
BLEIntCharacteristic pChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);
BLEIntCharacteristic rpChar("39b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);
BLEFloatCharacteristic fkpChar("a9b1cc01-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);
BLEIntCharacteristic footpadAThreshChar("1d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEIntCharacteristic footpadBThreshChar("2d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEFloatCharacteristic pushbackThresholdChar("3d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEIntCharacteristic pushbackTimeChar("4d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEFloatCharacteristic pushbackSpeedChar("5d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEFloatCharacteristic pushbackAmountChar("6d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEFloatCharacteristic rideAngleChar("7d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEFloatCharacteristic stepUpAngleChar("8d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEFloatCharacteristic stepUpSpeedChar("9d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);
BLEFloatCharacteristic imuOffsetChar("0d0f689a-8be0-11ee-b9d1-0242ac120002", BLERead | BLEWrite);


BLEIntCharacteristic stateChar("49b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic angleChar("59B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic pwmChar("69b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic footpadAChar("79a10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic footpadBChar("79b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic loopTimeChar("89b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic targetChar("99b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic batteryChar("a9b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

int calc_pid(float currentAngle, float rotationRate) {
  float error = targetAngle - currentAngle;
  float p_comp = error * control_params.kp;
  float rp_comp = rotationRate * control_params.krp;
  return constrain(p_comp + rp_comp, -MAX_PWM, MAX_PWM);
}

int calcPads(int footpad_a, int footpad_b) {
  int footpad = 0;
  if (footpad_a > control_params.footpad_a_thresh) footpad += 1;
  if (footpad_b > control_params.footpad_b_thresh) footpad += 1;

  return footpad;
}

void setupPreferences() {
  Preferences preferences;
  preferences.begin("hoverwheel", false);

  control_params.kp = preferences.getInt("kp", 30);
  control_params.krp = preferences.getInt("krp", 0);
  control_params.fkp = preferences.getFloat("fkp", 2);
  control_params.footpad_a_thresh = preferences.getInt("fa_thresh", 100);
  control_params.footpad_b_thresh = preferences.getInt("fb_thresh", 100);
  control_params.pushback_threshold = preferences.getFloat("pushback_thresh", .8);
  control_params.pushback_time = preferences.getInt("pushback_time", 250);
  control_params.pushback_speed = preferences.getFloat("pushback_speed", 10.f);
  control_params.pushback_amount = preferences.getFloat("pushback_amount", 4.f);
  control_params.ride_angle = preferences.getFloat("ride_angle", 0.f);
  control_params.step_up_angle = preferences.getFloat("step_up_angle", 10.f);
  control_params.step_up_speed = preferences.getFloat("step_up_speed", 90.f);
  control_params.imu_offset = preferences.getFloat("imu_offset", 0.f);

  // write to BLE characteristics
  pChar.writeValue(control_params.kp);
  rpChar.writeValue(control_params.krp);
  fkpChar.writeValue(control_params.fkp);
  footpadAThreshChar.writeValue(control_params.footpad_a_thresh);
  footpadBThreshChar.writeValue(control_params.footpad_b_thresh);
  pushbackThresholdChar.writeValue(control_params.pushback_threshold);
  pushbackTimeChar.writeValue(control_params.pushback_time);
  pushbackSpeedChar.writeValue(control_params.pushback_speed);
  pushbackAmountChar.writeValue(control_params.pushback_amount);
  rideAngleChar.writeValue(control_params.ride_angle);
  stepUpAngleChar.writeValue(control_params.step_up_angle);
  stepUpSpeedChar.writeValue(control_params.step_up_speed);
  imuOffsetChar.writeValue(control_params.imu_offset);

  preferences.end();
}

void writePreferences() {
  Preferences preferences;
  preferences.begin("hoverwheel", false);

  preferences.putInt("kp", control_params.kp);
  preferences.putInt("krp", control_params.krp);
  preferences.putFloat("fkp", control_params.fkp);
  preferences.putInt("fa_thresh", control_params.footpad_a_thresh);
  preferences.putInt("fb_thresh", control_params.footpad_b_thresh);
  preferences.putFloat("pushback_thresh", control_params.pushback_threshold);
  preferences.putInt("pushback_time", control_params.pushback_time);
  preferences.putFloat("pushback_speed", control_params.pushback_speed);
  preferences.putFloat("pushback_amount", control_params.pushback_amount);
  preferences.putFloat("ride_angle", control_params.ride_angle);
  preferences.putFloat("step_up_angle", control_params.step_up_angle);
  preferences.putFloat("step_up_speed", control_params.step_up_speed);
  preferences.putFloat("imu_offset", control_params.imu_offset);

  preferences.end();
}

void charachteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  control_params.kp = pChar.value();
  control_params.fkp = fkpChar.value();
  control_params.krp = rpChar.value();
  control_params.footpad_a_thresh = footpadAThreshChar.value();
  control_params.footpad_b_thresh = footpadBThreshChar.value();
  control_params.pushback_threshold = pushbackThresholdChar.value();
  control_params.pushback_time = pushbackTimeChar.value();
  control_params.pushback_speed = pushbackSpeedChar.value();
  control_params.pushback_amount = pushbackAmountChar.value();
  control_params.ride_angle = rideAngleChar.value();
  control_params.step_up_angle = stepUpAngleChar.value();
  control_params.step_up_speed = stepUpSpeedChar.value();
  control_params.imu_offset = imuOffsetChar.value();

  last_param_update = millis();
}

void setup() {
  Wire.begin(3, 2);
  Wire.setClock(400000);
  Serial.begin(115200);

  mpu.initialize();
  mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_98); // 98Hz

  mpu.setXGyroOffset(145);
  mpu.setYGyroOffset(32);
  mpu.setZGyroOffset(-33);
  mpu.setXAccelOffset(-5558);
  mpu.setYAccelOffset(-2118);
  mpu.setZAccelOffset(1218);


  pinMode(FOOTPAD_A_PIN, INPUT);
  pinMode(FOOTPAD_B_PIN, INPUT);
  // FastLED.addLeds<WS2812, 8, GRB>(leds, NUM_LEDS);  // GRB ordering is assumed

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  BLE.setLocalName("Hoverwheel");
  BLE.setDeviceName("Hoverwheel");

  // BLE
  BLE.setAdvertisedService(controlService);

  // add the characteristic to the service
  controlService.addCharacteristic(pChar);
  controlService.addCharacteristic(rpChar);
  controlService.addCharacteristic(fkpChar);
  controlService.addCharacteristic(footpadAThreshChar);
  controlService.addCharacteristic(footpadBThreshChar);
  controlService.addCharacteristic(pushbackThresholdChar);
  controlService.addCharacteristic(pushbackTimeChar);
  controlService.addCharacteristic(pushbackSpeedChar);
  controlService.addCharacteristic(pushbackAmountChar);
  controlService.addCharacteristic(rideAngleChar);
  controlService.addCharacteristic(stepUpAngleChar);
  controlService.addCharacteristic(stepUpSpeedChar);
  controlService.addCharacteristic(imuOffsetChar);


  statsService.addCharacteristic(stateChar);
  statsService.addCharacteristic(angleChar);
  statsService.addCharacteristic(pwmChar);
  statsService.addCharacteristic(footpadAChar);
  statsService.addCharacteristic(footpadBChar);
  statsService.addCharacteristic(loopTimeChar);
  statsService.addCharacteristic(targetChar);
  statsService.addCharacteristic(batteryChar);


  // add service
  BLE.addService(controlService);
  BLE.addService(statsService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // start advertising
  BLE.advertise();

  // assign event handlers for characteristic
  pChar.setEventHandler(BLEWritten, charachteristicWritten);
  rpChar.setEventHandler(BLEWritten, charachteristicWritten);
  fkpChar.setEventHandler(BLEWritten, charachteristicWritten);

  footpadAThreshChar.setEventHandler(BLEWritten, charachteristicWritten);
  footpadBThreshChar.setEventHandler(BLEWritten, charachteristicWritten);
  pushbackThresholdChar.setEventHandler(BLEWritten, charachteristicWritten);
  pushbackTimeChar.setEventHandler(BLEWritten, charachteristicWritten);
  pushbackSpeedChar.setEventHandler(BLEWritten, charachteristicWritten);
  pushbackAmountChar.setEventHandler(BLEWritten, charachteristicWritten);
  rideAngleChar.setEventHandler(BLEWritten, charachteristicWritten);
  stepUpAngleChar.setEventHandler(BLEWritten, charachteristicWritten);
  stepUpSpeedChar.setEventHandler(BLEWritten, charachteristicWritten);
  imuOffsetChar.setEventHandler(BLEWritten, charachteristicWritten);

  setupPreferences();
}

void loop() {
  // Loop Performance Timer
  long startMicros = micros();
  BLE.poll();

  int now = micros();
  // Serial.println(now - filter_last_update);
  float deltat = (now - filter_last_update) * 1.0e-6; //seconds since last update
  filter_last_update = now;

  // get MPU data
  int16_t rax, ray, raz;
  int16_t rgx, rgy, rgz;
  mpu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);

  // convert to G's
  float ax = ((float) rax) / 16384.0;
  float ay = ((float) ray) / 16384.0;
  float az = ((float) raz) / 16384.0;

  // convert to degrees per second
  float gx = ((float) rgx) / 131.0;
  float gy = ((float) rgy) / 131.0;
  float gz = ((float) rgz) / 131.0;

  // convert to radians per second
  gx *= (PI/180.0);
  gy *= (PI/180.0);
  gz *= (PI/180.0);
  
  // update filter
  Mahony_update(ax, ay, az, gx, gy, gz, deltat, control_params.fkp, 0.f);

  // float roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  // float pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  float board_tilt = atan2((mahony_q[0] * mahony_q[1] + mahony_q[2] * mahony_q[3]), 0.5 - (mahony_q[1] * mahony_q[1] + mahony_q[2] * mahony_q[2]));
  board_tilt *= 180.0/PI;
  board_tilt += control_params.ride_angle;
  board_tilt += control_params.imu_offset;
  board_tilt *= -1; // flip imu

  // int motor_speed = hoverboard.getSpeed0_kmh();

  //////////////// FOOTPADS //////////////// 
  int footpad_a = analogRead(FOOTPAD_A_PIN);
  int footpad_b = analogRead(FOOTPAD_B_PIN);
  footpad_a = footpad_b;
  int footpad = (footpad_a > control_params.footpad_a_thresh) + (footpad_b > control_params.footpad_a_thresh);


  // record when the footpad last changed state
  if (footpad != last_footpad) {
    last_footpad_change = millis();
    last_footpad = footpad;
  }

  // if the board is not idle
  // 
  if (state != STATE_IDLE && (footpad == 0 || (footpad < 2) && abs(pwm_cmd) < 150)) {
    // if the footpad has been released for more than FOOTPAD_DEACTIVATE_DELAY ms
    if (millis() - last_footpad_change > FOOTPAD_DEACTIVATE_DELAY) {
      state = STATE_IDLE;
    }
  } 

  // if the footpad is pressed, and the board is idle
  // and the board is +- 10 degrees from the target angle
  if (footpad == 2 && state == STATE_IDLE && abs(board_tilt - targetAngle) < control_params.step_up_angle) {
    state = STATE_RIDING;
    targetAngle = board_tilt;
    step_up_start = millis();
  }

  // step up fade to target angle
  if (state == STATE_RIDING) {
    float step_up_duration = millis() - step_up_start;
    targetAngle = min(-control_params.step_up_angle + control_params.step_up_speed*(step_up_duration/1000.f), (float) control_params.ride_angle);
  }

  //////////////// PUSHBACK ////////////////

  // if PWM is above threshold 
  if (abs(pwm_cmd) > MAX_PWM*control_params.pushback_threshold) {
    pushback_end_timer = 0;

    if (state == STATE_RIDING) {
      // record when the PWM first went above threshold
      if (pushback_start_timer == 0) {
        pushback_start_timer = millis();
      }

      // for more than PUSHBACK_TIME ms, initiate pushback
      if (millis() - pushback_start_timer > control_params.pushback_time) {
        state = STATE_PUSHBACK;
        pushback_start_time = millis();
      }
    }
  } else if (abs(pwm_cmd) < MAX_PWM*control_params.pushback_threshold) {
    pushback_start_timer = 0;

    if (state == STATE_PUSHBACK) {
      // record when the PWM first went above threshold
      if (pushback_end_timer == 0) {
        pushback_end_timer = millis();
      }

      // for more than PUSHBACK_TIME ms, end pushback
      if (millis() - pushback_end_timer > control_params.pushback_time) {
        state = STATE_RIDING;
        pushback_end_time = millis();
      }
    }
  }

  // fade in and out of pushback 
  if (state == STATE_PUSHBACK) {
    float pushback_duration = millis() - pushback_start_time;
    targetAngle = min(control_params.pushback_speed*(pushback_duration/1000.f), (float) control_params.pushback_amount) * SIGN(pwm_cmd);

  } else if (state == STATE_RIDING && pushback_start_time != 0) {
    float pushback_end_duration = millis() - pushback_end_time;
    targetAngle = max(control_params.pushback_amount - control_params.pushback_speed*(pushback_end_duration/1000.f), 0.f) * SIGN(pwm_cmd);

    // reset pushback timer if target angle is 0, stops animation
    if (targetAngle == 0) {
      pushback_start_time = 0;
    }
  }
  
  // handle pushback beeps
  if (state == STATE_PUSHBACK) {
    int time_since_beep = millis() - last_beep;

    if (time_since_beep > 250) {
      last_beep = millis();
      hoverboard.sendBuzzer(4, 0, 50, PROTOCOL_SOM_NOACK);
    }
  }

  ///////////// LEVELING LOGIC /////////////

  // get PWM command
  int pid_out = calc_pid(board_tilt, gx);

  if (state != STATE_IDLE) {
    pwm_cmd = pid_out;
  } else {
    // if the board is idle, set the PWM to 0
    pwm_cmd = 0;
  }


  if (state == STATE_IDLE) {
    FastLED.showColor(CRGB::Blue);
  }
  if ( state == STATE_RIDING) {
    FastLED.showColor(CRGB::Green);
  } 
  if (state == STATE_PUSHBACK) {
    FastLED.showColor(CRGB::Red);
  }



  if (millis() - last_ble_update > 100) {
    // update BLE
    stateChar.writeValue(state);
    angleChar.writeValue(board_tilt);
    pwmChar.writeValue(pwm_cmd);
    footpadAChar.writeValue(footpad_a);
    footpadBChar.writeValue(footpad_b);
    targetChar.writeValue(targetAngle);
    loopTimeChar.writeValue(micros() - startMicros);
    batteryChar.writeValue(0);

    last_ble_update = millis();
  }


  if (millis() - last_param_update > 1000 && last_param_update != -1) {
    writePreferences();
    last_param_update = -1;
  }

  hoverboard.sendDifferentialPWM(pwm_cmd, pwm_cmd, PROTOCOL_SOM_NOACK);
  // #ifndef DEBUG
  // #else
  //   // Serial.println("ANGLE:" + String(board_tilt) + ",TARGET:" + String(targetAngle) + ",PWM:" + String(pwm_cmd) + ",STATE:" + String(state) + ",FOOTPAD:" + String(footpad) + ",LOOP_TIME:" + String(micros() - startMicros));
  // #endif

  while (micros() - startMicros < 2000);
}
