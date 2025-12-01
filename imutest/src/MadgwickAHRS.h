#pragma once

/* Minimal MadgwickAHRS implementation (single-header)
   - update(gx,gy,gz, ax,ay,az, mx,my,mz)
   - quaternion stored as q0,q1,q2,q3
   - beta (gain) and sampleFreq configurable
*/

class Madgwick {
public:
  Madgwick(float sampleFreq = 100.0f, float beta = 0.1f) : sampleFreq(sampleFreq), beta(beta) {
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
  }

  // gx,gy,gz in rad/s
  // ax,ay,az in g (or normalized)
  // mx,my,mz in uT (or normalized)
  void update(float gx, float gy, float gz,
              float ax, float ay, float az,
              float mx, float my, float mz) {
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Normalize accelerometer measurement
    float norm = ax*ax + ay*ay + az*az;
    if (norm > 0.0f) {
      norm = invSqrt(norm);
      ax *= norm; ay *= norm; az *= norm;
    } else return;

    // Normalize magnetometer measurement
    norm = mx*mx + my*my + mz*mz;
    if (norm > 0.0f) {
      norm = invSqrt(norm);
      mx *= norm; my *= norm; mz *= norm;
    } else {
      // if no valid magnetometer, use IMU only update
      updateIMU(gx,gy,gz,ax,ay,az);
      return;
    }

    // Reference direction of Earth's magnetic field
    float hx = 2.0f*(mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
    float hy = 2.0f*(mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));
    float _2bx = sqrtf(hx*hx + hy*hy);
    float _2bz = 2.0f*(mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

    // Gradient descent algorithm corrective step
    float s0 = -2.0f*(q2*(2.0f*q1q3 - 2.0f*q0q2 - ax) + q3*(2.0f*q0q1 + 2.0f*q2q3 - ay)) + (-_2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz));
    float s1 = 2.0f*(q1*(2.0f*q1q3 - 2.0f*q0q2 - ax) + q3*(2.0f*q0q1 + 2.0f*q2q3 - ay)) + (-_2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q3 - 2.0f*_2bz*q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz));
    float s2 = -2.0f*(q0*(2.0f*q1q3 - 2.0f*q0q2 - ax) + q1*(2.0f*q0q1 + 2.0f*q2q3 - ay)) + ((-2.0f*_2bz*q0 + _2bx*q2)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q0 - _2bz*q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz));
    float s3 = 2.0f*(q0*(2.0f*q1q3 - 2.0f*q0q2 - ax) + q2*(2.0f*q0q1 + 2.0f*q2q3 - ay)) + ((-_2bx*q1 + _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz));

    // Normalize step magnitude
    norm = s0*s0 + s1*s1 + s2*s2 + s3*s3;
    if (norm > 0.0f) {
      norm = invSqrt(norm);
      s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;
    }

    // Compute rate of change of quaternion
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz) - beta * s0;
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy) - beta * s1;
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx) - beta * s2;
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx) - beta * s3;

    // Integrate to yield quaternion
    q0 += qDot0 * (1.0f / sampleFreq);
    q1 += qDot1 * (1.0f / sampleFreq);
    q2 += qDot2 * (1.0f / sampleFreq);
    q3 += qDot3 * (1.0f / sampleFreq);

    // Normalize quaternion
    norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    norm = invSqrt(norm);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
  }

  // IMU-only update (no magnetometer) - basic gradient descent correction
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm = ax*ax + ay*ay + az*az;
    if (norm == 0.0f) return;
    norm = invSqrt(norm);
    ax *= norm; ay *= norm; az *= norm;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    norm = s0*s0 + s1*s1 + s2*s2 + s3*s3;
    if (norm > 0.0f) {
      norm = invSqrt(norm);
      s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;
    }

    // Compute rate of change of quaternion
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz) - beta * s0;
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy) - beta * s1;
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx) - beta * s2;
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx) - beta * s3;

    q0 += qDot0 * (1.0f / sampleFreq);
    q1 += qDot1 * (1.0f / sampleFreq);
    q2 += qDot2 * (1.0f / sampleFreq);
    q3 += qDot3 * (1.0f / sampleFreq);

    norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    norm = invSqrt(norm);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
  }

  // Return heading (yaw) in degrees 0..360
  float getHeading() const {
    // yaw (z-axis rotation)
    float yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
    float deg = yaw * 180.0f / 3.14159265358979323846f;
    if (deg < 0) deg += 360.0f;
    return deg;
  }

  // public state
  float q0, q1, q2, q3;
  float sampleFreq; // Hz
  float beta; // algorithm gain

private:
  static float invSqrt(float x) {
    return 1.0f / sqrtf(x);
  }
};
