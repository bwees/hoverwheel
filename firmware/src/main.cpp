#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <FastLED.h>
#include <ArduinoBLE.h>
#include <Preferences.h>
#include "config.h"
#include "mahony.h"
#include "hoverboard_esc.h"

MPU6050 mpu(0x69);
Hoverboard esc;

//macro that returns +1/-1 depending on the sign of the argument
#define SIGN(x) ((x > 0) - (x < 0))

#define MPH_TO_RPM(x) x * ((WHEEL_DIAMETER*3.14*60)/63360.0)

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
int packets = 0;

//footpad
int footpad_a = 0;
int footpad_b = 0;

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
  bool headlightOn;
  int btUpdateInterval;
  int compensatedKpPrescale;
  int compensatedKpPostscale;
  int maxMotorPWM;
} control_params;


CRGB leds[NUM_LEDS];

// BLE SERVICES
BLEService controlService("19b10000-e8f2-537e-4f6c-d104768a1215"); // create service
BLEService statsService("19a10000-e8f2-537e-4f6c-d104768a1215"); // create service

// CONTROL SETTINGS
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
BLEBoolCharacteristic headlightChar("277589d2-cfc2-4d27-937f-f5d299b769b5", BLERead | BLEWrite);
BLEIntCharacteristic btUpdateInterval("377589d2-cfc2-4d27-937f-f5d299b769b5", BLERead | BLEWrite);
BLEIntCharacteristic compensatedKPPrescaleChar("477589d2-cfc2-4d27-937f-f5d299b769b5", BLERead | BLEWrite);
BLEIntCharacteristic compensatedKPPostscaleChar("577589d2-cfc2-4d27-937f-f5d299b769b5", BLERead | BLEWrite);
BLEIntCharacteristic maxMotorPWMChar("677589d2-cfc2-4d27-937f-f5d299b769b5", BLERead | BLEWrite);

// TELEMETRY
BLEIntCharacteristic stateChar("49b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic angleChar("59B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic pwmChar("69b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic footpadAChar("79a10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic footpadBChar("79b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic loopTimeChar("89b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic targetChar("99b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic batteryChar("a9b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic boardTempChar("423ed38f-e856-45e7-9a6e-ec9e51c3aba1", BLERead | BLENotify);
BLEIntCharacteristic wheelSpeedChar("523ed38f-e856-45e7-9a6e-ec9e51c3aba1", BLERead | BLENotify);
BLEFloatCharacteristic compensatedKpChar("623ed38f-e856-45e7-9a6e-ec9e51c3aba1", BLERead | BLENotify);



void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  // Serial.print("Connected event, central: ");
  // Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  // Serial.print("Disconnected event, central: ");
  // Serial.println(central.address());
}

void handleBLE(void* params) {
  while (true) {
    BLE.poll();
    delay(2);
  }
}

void handleFootpads(void* params) {
  while (true) {
    footpad_a = analogRead(FOOTPAD_A_PIN);
    footpad_b = analogRead(FOOTPAD_B_PIN);
    #ifdef SINGLE_FOOTPAD
    footpad_a = footpad_b;
    #endif
    delay(2);
  }
}

int calc_pid(float currentAngle, float rotationRate, float compensatedKp) {
  float error = targetAngle - currentAngle;
  float p_comp = error * compensatedKp;
  float rp_comp = rotationRate * control_params.krp;
  return constrain(p_comp + rp_comp, -control_params.maxMotorPWM, control_params.maxMotorPWM);
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
  control_params.compensatedKpPrescale = preferences.getInt("compensatedKpPrescale", 250);
  control_params.compensatedKpPostscale = preferences.getInt("compensatedKpPostscale", 100);
  control_params.maxMotorPWM = preferences.getInt("maxMotorPWM", 600);
  control_params.headlightOn = false;
  control_params.btUpdateInterval = 500;

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
  headlightChar.writeValue(control_params.headlightOn);
  compensatedKPPrescaleChar.writeValue(control_params.compensatedKpPrescale);
  compensatedKPPostscaleChar.writeValue(control_params.compensatedKpPostscale);
  maxMotorPWMChar.writeValue(control_params.maxMotorPWM);
  btUpdateInterval.writeValue(control_params.btUpdateInterval);

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
  preferences.putInt("compensatedKpPrescale", control_params.compensatedKpPrescale);
  preferences.putInt("compensatedKpPostscale", control_params.compensatedKpPostscale);
  preferences.putInt("maxMotorPWM", control_params.maxMotorPWM);

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
  control_params.headlightOn = headlightChar.value();
  control_params.btUpdateInterval = btUpdateInterval.value();
  control_params.compensatedKpPrescale = compensatedKPPrescaleChar.value();
  control_params.compensatedKpPostscale = compensatedKPPostscaleChar.value();
  control_params.maxMotorPWM = maxMotorPWMChar.value();

  last_param_update = millis();
}

void led_task(void* parameters) {
  FastLED.setBrightness(255);
  while (true) {
    if (control_params.headlightOn == true) {
      FastLED.showColor(CRGB::White);
    }
    else if (state == STATE_IDLE) {
      FastLED.showColor(CRGB::Blue);
    }
    else if ( state == STATE_RIDING) {
      FastLED.showColor(CRGB::Green);
    } 
    if (state == STATE_PUSHBACK) {
      FastLED.showColor(CRGB::Red);
    }
    delay(100);
  }
}

void setup_ble() {
    // begin initialization
  if (!BLE.begin()) {
    while (1);
  }

  BLE.setLocalName("Hoverwheel - bwees");
  BLE.setDeviceName("Hoverwheel - bwees");

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
  controlService.addCharacteristic(headlightChar);
  controlService.addCharacteristic(compensatedKPPrescaleChar);
  controlService.addCharacteristic(compensatedKPPostscaleChar);
  controlService.addCharacteristic(maxMotorPWMChar);
  controlService.addCharacteristic(btUpdateInterval);

  statsService.addCharacteristic(stateChar);
  statsService.addCharacteristic(angleChar);
  statsService.addCharacteristic(pwmChar);
  statsService.addCharacteristic(footpadAChar);
  statsService.addCharacteristic(footpadBChar);
  statsService.addCharacteristic(loopTimeChar);
  statsService.addCharacteristic(targetChar);
  statsService.addCharacteristic(batteryChar);
  statsService.addCharacteristic(boardTempChar);
  statsService.addCharacteristic(wheelSpeedChar);
  statsService.addCharacteristic(compensatedKpChar);

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
  headlightChar.setEventHandler(BLEWritten, charachteristicWritten);
  compensatedKPPrescaleChar.setEventHandler(BLEWritten, charachteristicWritten);
  compensatedKPPostscaleChar.setEventHandler(BLEWritten, charachteristicWritten);
  maxMotorPWMChar.setEventHandler(BLEWritten, charachteristicWritten);
  btUpdateInterval.setEventHandler(BLEWritten, charachteristicWritten);

}

void setup() {
  Wire.begin(32, 33);
  Wire.setClock(400000);

  Serial2.begin(115200, SERIAL_8N1); // redefine RX pin 

  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_98); // 98Hz

  mpu.setXAccelOffset(823);
  mpu.setYAccelOffset(2446);
  mpu.setZAccelOffset(1122);
  mpu.setXGyroOffset(-12);
  mpu.setYGyroOffset(15);
  mpu.setZGyroOffset(-27);

  pinMode(FOOTPAD_A_PIN, INPUT);
  pinMode(FOOTPAD_B_PIN, INPUT);
  FastLED.addLeds<WS2812, LED_FRONT_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is assumed

  setup_ble();

  // start BLE task
  xTaskCreatePinnedToCore(handleBLE, "BLE", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(handleFootpads, "Footpads", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(led_task, "LED", 10000, NULL, 1, NULL, 0);

  setupPreferences();
}

void loop() {
  // Loop Performance Timer
  long startMicros = micros();

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

  // handle sensor being upside down and flipping between 180 and -180
  if (board_tilt > 90) {
    board_tilt = 180 - board_tilt;
  } else if (board_tilt < -90) {
    board_tilt = -180 - board_tilt;
  }

  board_tilt += control_params.ride_angle;
  board_tilt += control_params.imu_offset;
  board_tilt *= -1; // flip imu


  ////////////// ESC TELEMETRY ////////////// 
  esc.receiveTelemetry();

  //////////////// FOOTPADS //////////////// 
  int footpad = (footpad_a > control_params.footpad_a_thresh) + (footpad_b > control_params.footpad_a_thresh);


  // record when the footpad last changed state
  if (footpad != last_footpad) {
    last_footpad_change = millis();
    last_footpad = footpad;
  }

  // if the board is not idle and the footpad is not pressed 
  // or the board speed is less than 2 mph and 1/2 footpad is pressed (heel lift)
  if (state != STATE_IDLE && (footpad == 0 || (footpad < 2 && abs(esc.feedback.speedL_meas) < MPH_TO_RPM(2)))) {
    // if the footpad has been released for more than FOOTPAD_DEACTIVATE_DELAY ms
    if (millis() - last_footpad_change > FOOTPAD_DEACTIVATE_DELAY) {
      // deactivate the board
      state = STATE_IDLE;
    }
  } 

  // if the footpad is completely pressed, and the board is idle
  // and the board is +- 10 degrees from the target angle
  if (footpad == 2 && state == STATE_IDLE && (-5 < (control_params.step_up_angle+board_tilt)) && (0 > (control_params.step_up_angle+board_tilt))) {
    // activate the board at the start angle and record the time
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
  if (abs(pwm_cmd) > control_params.maxMotorPWM*control_params.pushback_threshold) { // pushback activation
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
  } else if (abs(pwm_cmd) < control_params.maxMotorPWM*control_params.pushback_threshold) { // pushback deactivation
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

  ///////////// LEVELING LOGIC /////////////

  // calculate kp compensation
  float kpCompensation = map(abs(esc.feedback.speedL_meas), 0, control_params.compensatedKpPrescale, 0, control_params.compensatedKpPostscale);
  kpCompensation = constrain(kpCompensation, 0, control_params.compensatedKpPostscale);

  // get PWM command
  float compensatedKp = control_params.kp + kpCompensation;
  int pid_out = calc_pid(board_tilt, gx, compensatedKp);

  // only send PWM if the board is not idle
  if (state != STATE_IDLE) {
    pwm_cmd = pid_out;
  } else {
    // if the board is idle, set the PWM to 0
    pwm_cmd = 0;
  }

  //////////////// BLE ////////////////

  // update BLE
  if (millis() - last_ble_update > control_params.btUpdateInterval) {
    stateChar.writeValue(state);
    angleChar.writeValue(board_tilt);
    pwmChar.writeValue(pwm_cmd);
    footpadAChar.writeValue(footpad_a);
    footpadBChar.writeValue(footpad_b);
    targetChar.writeValue(targetAngle);
    batteryChar.writeValue(esc.feedback.batVoltage/100.0);
    boardTempChar.writeValue(esc.feedback.boardTemp);
    wheelSpeedChar.writeValue(esc.feedback.speedL_meas);
    compensatedKpChar.writeValue(compensatedKp);
    loopTimeChar.writeValue(micros() - startMicros);

    last_ble_update = millis();
  }

  // only write parameters to flash if they have not been updated in 1 second
  // they will be updated immediately in control params after they are changed
  if (millis() - last_param_update > 1000 && last_param_update != -1) {
    writePreferences();
    last_param_update = -1;
  }

  //////////////// ESC COMMUNICATION ////////////////

  // update esc data
  esc.speedCmd = pwm_cmd;
  esc.sendCommand();


  while (micros() - startMicros < 2000);
}
