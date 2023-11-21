#include <Arduino.h>
#include <HoverboardAPI.h>
#include <Wire.h>
#include <MPU6050.h>
#include "PIDController.h"
#include <FastLED.h>
#include <ArduinoBLE.h>

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

PIDController pid;

bool last_footpad = false;
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

int kp = P_GAIN;
int fkp = FILTER_KP;
int krp = RP_GAIN;

CRGB leds[NUM_LEDS];

// BLE
BLEService controlService("19b10000-e8f2-537e-4f6c-d104768a1215"); // create service
BLEService statsService("19a10000-e8f2-537e-4f6c-d104768a1215"); // create service

// create switch characteristic and allow remote device to read and write
BLEIntCharacteristic pChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);
BLEIntCharacteristic rpChar("39b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);
BLEFloatCharacteristic fkpChar("a9b1cc01-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);

BLEIntCharacteristic stateChar("49b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic angleChar("59B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic pwmChar("69b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic footpadAChar("79a10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic footpadBChar("79b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEIntCharacteristic loopTimeChar("89b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEFloatCharacteristic targetChar("99b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);

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

void pidCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  kp = pChar.value();
  fkp = fkpChar.value();
  krp = rpChar.value();
}

int calc_pid(float currentAngle, float rotationRate) {
  float error = targetAngle - currentAngle;
  float p_comp = error * kp;
  float rp_comp = rotationRate * krp;
  return constrain(p_comp + rp_comp, -MAX_PWM, MAX_PWM);

}


void setup() {
  Wire.begin(2, 3);
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


  pinMode(FOOTPAD_PIN, INPUT_PULLUP);
  pid.limit(-MAX_PWM, MAX_PWM);
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

  statsService.addCharacteristic(stateChar);
  statsService.addCharacteristic(angleChar);
  statsService.addCharacteristic(pwmChar);
  statsService.addCharacteristic(footpadAChar);
  statsService.addCharacteristic(footpadBChar);
  statsService.addCharacteristic(loopTimeChar);
  statsService.addCharacteristic(targetChar);

  // add service
  BLE.addService(controlService);
  BLE.addService(statsService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  pChar.setEventHandler(BLEWritten, pidCharacteristicWritten);
  rpChar.setEventHandler(BLEWritten, pidCharacteristicWritten);
  fkpChar.setEventHandler(BLEWritten, pidCharacteristicWritten);

  // set an initial value for the characteristic
  pChar.writeValue(P_GAIN);
  rpChar.writeValue(RP_GAIN);
  fkpChar.writeValue(FILTER_KP);
  
  // start advertising
  BLE.advertise();
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
  Mahony_update(ax, ay, az, gx, gy, gz, deltat);

  // float roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  // float pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  float board_tilt = atan2((mahony_q[0] * mahony_q[1] + mahony_q[2] * mahony_q[3]), 0.5 - (mahony_q[1] * mahony_q[1] + mahony_q[2] * mahony_q[2]));
  board_tilt *= 180.0/PI;
  board_tilt += RIDE_ANGLE;


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
  if (!footpad && state != STATE_IDLE && abs(pwm_cmd) < 150) {
    // if the footpad has been released for more than FOOTPAD_DEACTIVATE_DELAY ms
    if (millis() - last_footpad_change > FOOTPAD_DEACTIVATE_DELAY) {
      state = STATE_IDLE;
    }
  } 

  // if the footpad is pressed, and the board is idle
  // and the board is +- 10 degrees from the target angle
  if (footpad && state == STATE_IDLE && abs(board_tilt - targetAngle) < STEP_UP_ANGLE) {
    state = STATE_RIDING;
    targetAngle = board_tilt;
    step_up_start = millis();
  }

  // step up fade to target angle
  if (state == STATE_RIDING) {
    float step_up_duration = millis() - step_up_start;

    
    targetAngle = min(-STEP_UP_ANGLE + STEP_UP_SPEED*(step_up_duration/1000.f), (float) RIDE_ANGLE);
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
        // state = STATE_PUSHBACK;
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
    float pushback_duration = millis() - pushback_start_time;
    targetAngle = min(PUSHBACK_SPEED*(pushback_duration/1000.f), (float) PUSHBACK_AMOUNT) * SIGN(pwm_cmd);

  } else if (state == STATE_RIDING && pushback_start_time != 0) {
    float pushback_end_duration = millis() - pushback_end_time;
    targetAngle = max(PUSHBACK_AMOUNT - PUSHBACK_SPEED*(pushback_end_duration/1000.f), 0.f) * SIGN(pwm_cmd);

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
  pid.setpoint(targetAngle);
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
    footpadAChar.writeValue(footpad);
    footpadBChar.writeValue(footpad);
    targetChar.writeValue(targetAngle);
    loopTimeChar.writeValue(micros() - startMicros);


    last_ble_update = millis();
  }

  #ifndef DEBUG
    hoverboard.sendDifferentialPWM(pwm_cmd, pwm_cmd, PROTOCOL_SOM_NOACK);
  #else
    // Serial.println("ANGLE:" + String(board_tilt) + ",TARGET:" + String(targetAngle) + ",PWM:" + String(pwm_cmd) + ",STATE:" + String(state) + ",FOOTPAD:" + String(footpad) + ",LOOP_TIME:" + String(micros() - startMicros));
  #endif

  while (micros() - startMicros < 2000);
}
