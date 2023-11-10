// enable debug mode
#define DEBUG

// footpads
#define FOOTPAD_DEACTIVATE_DELAY 100
#define FOOTPAD_PIN 9

// motor
#define MAX_PWM 600

// pushback
#define PUSHBACK_THRESHOLD .80 // % of max pwm
#define PUSHBACK_TIME 250 // ms
#define PUSHBACK_SPEED 10.f // degrees per second
#define PUSHBACK_AMOUNT 4.f // degrees
#define RIDE_ANGLE 0.f // degrees

// step up
#define STEP_UP_ANGLE 10.f // degrees
#define STEP_UP_SPEED 90.f // degrees per second

// PID Controller constants
#define KP 1200
#define KI 0
#define KD 0

// Gyro Filter
#define FILTER_KP 2
#define FILTER_KI 0

// LEDS
#define NUM_LEDS 8