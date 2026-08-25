/* ---------------------------------------------------------------------------
 * madgwick.h -- Madgwick's gradient-descent orientation filter, IMU-only
 * (6-DOF) variant.
 *
 * Port of Sebastian O.H. Madgwick's reference implementation
 * ("An efficient orientation filter for inertial and inertial/magnetic sensor
 * arrays", University of Bristol, April 2010; reference source released
 * under the "do whatever you want" terms in MadgwickAHRS.c).
 *
 * This board has no magnetometer, so only the IMU variant is implemented --
 * the 9-DOF MARG update would have nothing to correct yaw with.  The practical
 * consequence is that roll and pitch are gravity-referenced and drift-free,
 * while yaw is a dead-reckoned integration of gyro Z and WILL drift.  That is
 * expected, not a bug, and is called out in the host UI.
 * ------------------------------------------------------------------------- */

#ifndef MADGWICK_H
#define MADGWICK_H

#include <stdint.h>

typedef struct {
    float w;   /* q0 -- scalar part */
    float x;   /* q1 */
    float y;   /* q2 */
    float z;   /* q3 */
} quat_t;

/* Reset the quaternion to identity and set the filter gain. */
void madgwick_init(float beta);

/* Beta is the algorithm gain: it trades gyro-integration smoothness against
 * how fast accelerometer feedback pulls the estimate back to vertical.
 * Madgwick derives it as sqrt(3/4) * (gyro measurement error in rad/s).
 * Kept as a runtime variable, not a compile-time constant, so the host can
 * retune it live over the command channel without a reflash. */
void  madgwick_set_beta(float beta);
float madgwick_get_beta(void);

/* One filter step.
 *   gx, gy, gz : angular rate in RADIANS per second (sensor frame)
 *   ax, ay, az : acceleration in any consistent unit (it is normalised
 *                internally, so g or m/s^2 both work)
 *   dt         : elapsed time since the previous update, in seconds
 *
 * A zero-magnitude accelerometer reading is ignored (gyro integration only)
 * rather than producing a NaN, which is what the reference implementation
 * does too. */
void madgwick_update_imu(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);

/* Current orientation estimate: rotation from the sensor frame to the earth
 * frame, as a unit quaternion. */
quat_t madgwick_quat(void);

#endif /* MADGWICK_H */
