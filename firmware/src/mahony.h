#include <Arduino.h>
#include "config.h"

float mahony_q[4] = {1.0, 0.0, 0.0, 0.0};

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat, float filter_kp, float filter_ki) {
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
    if (filter_ki > 0.0f) {
      gx += filter_ki * ex * deltat;  // apply integral feedback
      gy += filter_ki * ey * deltat;
      gz += filter_ki * ez * deltat;
    }

    // Apply proportional feedback to gyro term
    gx += filter_kp * ex;
    gy += filter_kp * ey;
    gz += filter_kp * ez;
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