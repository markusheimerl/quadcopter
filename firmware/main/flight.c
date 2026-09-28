#include <math.h>
#include "flight.h"

/* Accelerometer correction time constant. In flight the accelerometer
 * measures thrust, not gravity, whenever the quad speeds up or slows down:
 * a tilted quad that accelerates reads "level". A fast correction (the old
 * 0.25 s) turns every tilt into a drift the loop no longer sees. */
#define TAU_ACC   2.0f      /* s */
#define ACC_GATE  0.15f     /* g: no correction while |a| is further than this from 1 g */

/* Cascade: angle error -> rotation rate setpoint -> PID on the gyro rate.
 * The integrator trims out an off-centre battery or a weaker motor, which
 * the old angle PD turned into a steady tilt (and a drift); the D term
 * makes up for the motors' lag (70 ms up, 140 ms down). Tuned in
 * simulation (tools/hover_sim.c): a bump dies down from 1/2 to 2x these
 * gains even for motor lags of 50-130 ms, inertia -25..+30 %, thrust
 * -15..+35 % and 3 ms of extra delay together; much wider for the nominal
 * quad. */
#define KA        3.5f      /* angle loop: dps per degree of error */
#define RATE_MAX  180.0f    /* dps */
#define KR        0.3f      /* roll/pitch rate loop: PWM per dps */
#define KI        0.8f      /* PWM per degree (dps x s) */
#define KD        0.008f    /* PWM per dps/s of angular acceleration */
#define D_TAU     0.008f    /* s: low-pass on that derivative */
#define I_MAX     25.0f     /* PWM */
#define KY        0.5f      /* yaw rate loop */
#define KIY       0.5f
#define IY_MAX    30.0f
/* Integrators run from I_THR up, but only to I_GROUND of their limits until
 * lift-off is seen: on its feet (throttle between I_THR and hover) the quad
 * would wind them up and lurch at take-off. Lift-off shows as upward
 * acceleration: the specific force along "up", averaged over LIFT_TAU,
 * above LIFT_G times its value at rest. A brisk climb (about 0.1 m/s gained
 * within a second) sets it; sitting does not (1 % is about 6 sigma of the
 * averaged noise at 0.05 g of vibration). Below I_THR the integrators hold
 * and the latch clears; a touch-down above I_THR goes unnoticed. */
#define I_THR     100.0f
#define I_GROUND  0.3f      /* 7.5 PWM: enough for a battery 2 mm off centre */
#define LIFT_G    1.01f
#define LIFT_TAU  1.0f      /* s */
#define DUTY_MAX  255.0f

#define D2R ((float)M_PI / 180.0f)
#define R2D (180.0f / (float)M_PI)

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

void flight_estimate(flight_t *f, const float a[3], const float g[3], float dt)
{
    float *u = f->up;
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (!f->init) {
        if (n < 0.5f) return;
        for (int k = 0; k < 3; ++k) u[k] = a[k] / n;
        f->lift = n;
        f->init = true;
    } else {
        /* a world-fixed vector seen from the rotating body turns by -w x u */
        float w[3] = { g[0] * D2R * dt, g[1] * D2R * dt, g[2] * D2R * dt };
        float c[3] = { u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0] };
        float k_acc = fabsf(n - 1.0f) < ACC_GATE ? dt / TAU_ACC : 0.0f;
        for (int k = 0; k < 3; ++k) u[k] += c[k] + k_acc * (a[k] / n - u[k]);
        float m = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        for (int k = 0; k < 3; ++k) u[k] /= m;
        f->lift += (a[0] * u[0] + a[1] * u[1] + a[2] * u[2] - f->lift) * dt / (dt + LIFT_TAU);
    }
    f->roll  = atan2f(u[1], u[2]) * R2D - f->roll0;
    f->pitch = atan2f(-u[0], sqrtf(u[1] * u[1] + u[2] * u[2])) * R2D - f->pitch0;
}

void flight_control(flight_t *f, float thr, float roll_sp, float pitch_sp,
                    float gain, const float g[3], float dt, int duty[4])
{
    if (thr <= 0.0f) {
        for (int k = 0; k < 3; ++k) { f->i[k] = f->d[k] = 0.0f; f->g_prev[k] = g[k]; }
        f->air = false;
        duty[0] = duty[1] = duty[2] = duty[3] = 0;
        return;
    }
    if (thr < I_THR) f->air = false;
    else if (f->lift > LIFT_G * (f->g1 > 0.0f ? f->g1 : 1.0f)) f->air = true;
    float sp[3] = {
        clampf(KA * (roll_sp  - f->roll ), -RATE_MAX, RATE_MAX),
        clampf(KA * (pitch_sp - f->pitch), -RATE_MAX, RATE_MAX),
        0.0f,
    };
    float u[3];   /* torque about +X, +Y, +Z, in PWM */
    for (int k = 0; k < 3; ++k) {
        float e = sp[k] - g[k];
        float kp = (k < 2 ? KR : KY) * gain, ki = (k < 2 ? KI : KIY) * gain;
        float im = (k < 2 ? I_MAX : IY_MAX) * (f->air ? 1.0f : I_GROUND);
        float v = f->i[k] + ki * e * dt;   /* past its limit it may only shrink, never jump */
        if (thr >= I_THR && (fabsf(v) <= im || fabsf(v) < fabsf(f->i[k]))) f->i[k] = v;
        f->d[k] += ((g[k] - f->g_prev[k]) / dt - f->d[k]) * dt / (dt + D_TAU);
        f->g_prev[k] = g[k];
        u[k] = kp * e + f->i[k] - (k < 2 ? KD * gain * f->d[k] : 0.0f);
    }
    /* +X torque lifts the left side, +Y the back, +Z (CCW) speeds up the CW props */
    float m[4] = {
        thr - u[0] - u[1] - u[2],   /* M0 FR CCW */
        thr - u[0] + u[1] + u[2],   /* M1 BR CW  */
        thr + u[0] + u[1] - u[2],   /* M2 BL CCW */
        thr + u[0] - u[1] + u[2],   /* M3 FL CW  */
    };
    /* over full power: lower all four alike, which keeps the differences */
    float hi = fmaxf(fmaxf(m[0], m[1]), fmaxf(m[2], m[3]));
    for (int k = 0; k < 4; ++k) {
        if (hi > DUTY_MAX) m[k] -= hi - DUTY_MAX;
        duty[k] = (int)lroundf(clampf(m[k], 0.0f, DUTY_MAX));
    }
}
