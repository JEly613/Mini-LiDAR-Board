/* ---------------------------------------------------------------------------
 * madgwick.c -- 6-DOF Madgwick filter.
 *
 * Structure follows Madgwick's published MadgwickAHRSupdateIMU() closely so
 * the derivation in his paper maps line-for-line onto the code.  Two
 * deliberate changes:
 *
 *   1. dt is passed in per update instead of using a fixed sampleFreq
 *      constant.  Our sample interval comes from the hardware microsecond
 *      timer, so the filter tracks the real IMU rate even if a sample is
 *      dropped.
 *   2. invSqrt() uses the Cortex-M4F's VSQRT.F32 instruction rather than the
 *      Quake fast-inverse-square-root bit hack from the original.  The hack
 *      existed because Madgwick's target had no FPU; here the hardware
 *      instruction is both faster and exact.
 * ------------------------------------------------------------------------- */

#include "madgwick.h"
#include "util.h"

static quat_t s_q;
static float  s_beta;

void madgwick_init(float beta)
{
    s_q.w  = 1.0f;
    s_q.x  = 0.0f;
    s_q.y  = 0.0f;
    s_q.z  = 0.0f;
    s_beta = beta;
}

void  madgwick_set_beta(float beta) { s_beta = beta; }
float madgwick_get_beta(void)       { return s_beta; }

quat_t madgwick_quat(void) { return s_q; }

void madgwick_update_imu(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float q0 = s_q.w, q1 = s_q.x, q2 = s_q.y, q3 = s_q.z;

    /* ---- Rate of change of quaternion from the gyroscope ---------------- */
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* ---- Accelerometer feedback -----------------------------------------
     * Skipped when the accelerometer reads exactly zero, which would make the
     * normalisation blow up.  In free fall the magnitude is near zero too,
     * but the reference implementation only guards the exact case and so do
     * we; the filter simply coasts on the gyro. */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        float recipNorm;

        /* Normalise the measurement: only its direction carries information
         * about which way is down. */
        recipNorm = inv_sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Auxiliary terms, hoisted out of the objective function and its
         * Jacobian to avoid recomputing them (this is Madgwick's own
         * optimisation, kept verbatim). */
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
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        /* Gradient descent step: the direction of steepest descent of the
         * error between the measured gravity vector and the one predicted by
         * the current quaternion. */
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
                 - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                 - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = inv_sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply the feedback step, scaled by the filter gain. */
        qDot0 -= s_beta * s0;
        qDot1 -= s_beta * s1;
        qDot2 -= s_beta * s2;
        qDot3 -= s_beta * s3;
    }

    /* ---- Integrate to yield the new quaternion --------------------------- */
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* ---- Renormalise ----------------------------------------------------
     * Numerical drift makes |q| wander away from 1 over thousands of
     * integrations; the host assumes a unit quaternion. */
    float recipNorm = inv_sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    s_q.w = q0 * recipNorm;
    s_q.x = q1 * recipNorm;
    s_q.y = q2 * recipNorm;
    s_q.z = q3 * recipNorm;
}
