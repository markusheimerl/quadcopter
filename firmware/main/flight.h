/*
 * Attitude estimate and attitude control, in plain C (no ESP-IDF) so that
 * tools/hover_sim.c flies this same code on a PC.
 *
 * IMU frame = body frame: +X nose, +Y left, +Z up. Angles in degrees:
 * roll > 0 = right side down, pitch > 0 = nose down; rates in dps about
 * +X/+Y/+Z. Motors, viewed from above:
 *
 *      M3 (FL, CW)   M0 (FR, CCW)
 *      M2 (BL, CCW)  M1 (BR, CW)
 */
#pragma once
#include <stdbool.h>

typedef struct {
    float up[3];          /* estimated world "up" seen in the body frame, unit length */
    bool  init;           /* up[] holds an estimate */
    float roll0, pitch0;  /* level calibration: the IMU's angles on a level surface */
    float roll, pitch;    /* attitude relative to that level */
    float i[3];           /* rate-loop integrators (PWM) */
    float d[3], g_prev[3];/* filtered angular acceleration (dps/s), last gyro */
    float g1;             /* accelerometer magnitude at rest (g); 0 = 1 g */
    float lift;           /* low-passed specific force along "up" (g) */
    bool  air;            /* lift-off seen (throttle >= I_THR): integrators get their full range */
} flight_t;

/* Roll and pitch from one IMU sample (acc in g, gyro in dps, bias removed):
 * the gyro carries the attitude, the accelerometer pulls it slowly toward
 * gravity. Clear f->init to restart from the next accelerometer reading. */
void flight_estimate(flight_t *f, const float acc[3], const float gyr[3], float dt);

/* Motor duties (0..255) around the base throttle thr (0..255) that hold
 * roll and pitch at the setpoints (deg) and keep the heading. gain scales
 * the rate loop (1 = the defaults in flight.c). thr = 0 turns the motors
 * off and clears the integrators. */
void flight_control(flight_t *f, float thr, float roll_sp, float pitch_sp,
                    float gain, const float gyr[3], float dt, int duty[4]);
