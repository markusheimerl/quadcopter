/*
 * hover_sim.c -- flies main/flight.c in a simulated quad before the real
 * one does. From firmware/:
 *
 *   cc -O2 -Imain tools/hover_sim.c main/flight.c -lm -o hover_sim && ./hover_sim
 *
 * The plant: a 32 g quad, 33 mm per-axis arms, 7 mm brushed motors with
 * 55 mm props (19 g of thrust each, thrust ~ 0.57 d + 0.43 d^2 of that,
 * following the duty with a 70 ms lag, 140 ms going down: the flyback diode
 * lets a motor coast), the IMU's 800 Hz sampling and 1.43 ms group delay,
 * the 2 ms loop, sensor noise, rotor drag. Numbers from Crazyflie
 * measurements scaled to this frame. Body = IMU frame (+X nose, +Y left,
 * +Z up), world Z up.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "flight.h"

typedef struct {
    double tau;          /* motor lag, spin-up (s); spin-down is 2x */
    double J;            /* roll/pitch inertia (kg m^2), yaw 1.95x */
    double fmax;         /* thrust per motor at full duty (N) */
    double cg_x;         /* centre of gravity ahead of the motor centre (m) */
    double mis;          /* motor thrust mismatch: M0 +, M1 -, M2 +, M3 - */
    double kick;         /* roll + pitch rate kick at 1 s (dps) */
    double steer;        /* pitch setpoint 2..3 s (deg) */
    double gain;         /* the [ ] gain scale */
    double trim;         /* roll setpoint (deg) */
    int takeoff;         /* start on the ground, throttle +5 every 0.6 s from 100 */
} plant_t;

typedef struct { int crash; double peak, tilt, rms_rate, drift, sat, chatter, ring, t_lift, i_lift, over, pitch; } result_t;

static const plant_t NOMINAL = { 0.07, 2.0e-5, 0.186, 0, 0, 0, 0, 1, 0, 0 };

static unsigned s_seed = 1;
static double noise(void)   /* ~N(0, 1) */
{
    double v = 0;
    for (int i = 0; i < 12; i++) { s_seed = s_seed * 1103515245u + 12345u; v += (s_seed >> 8 & 0xFFFF) / 65536.0; }
    return v - 6;
}

static void mul(const double A[9], const double B[9], double C[9])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
}

static void rotate(double R[9], const double w[3], double dt)   /* R <- R exp([w dt]x) */
{
    double v[3] = { w[0] * dt, w[1] * dt, w[2] * dt }, t2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2], t = sqrt(t2);
    double a = t < 1e-8 ? 1 - t2 / 6 : sin(t) / t, b = t < 1e-8 ? 0.5 - t2 / 24 : (1 - cos(t)) / t2;
    double K[9] = { 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0 }, K2[9], E[9], Rn[9];
    mul(K, K, K2);
    for (int i = 0; i < 9; i++) E[i] = (i % 4 == 0) + a * K[i] + b * K2[i];
    mul(R, E, Rn);
    memcpy(R, Rn, sizeof Rn);
}

static result_t fly(plant_t p, double seconds)
{
    const double m = 0.032, g = 9.81, arm = 0.033, cM = 0.007, kd = 0.007, dt = 1e-4, Q = 0.43;
    const double rx[4] = { arm, -arm, -arm, arm }, ry[4] = { -arm, -arm, arm, arm }, spin[4] = { -1, 1, -1, 1 };
    const double J[3] = { p.J, p.J, 1.95 * p.J };
    double pos[3] = { 0, 0, p.takeoff ? 0 : 1 }, vel[3] = { 0 }, R[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, w[3] = { 0 };
    double hover = m * g / (4 * p.fmax), th[4] = { hover, hover, hover, hover };
    double thr0 = 255 * (-(1 - Q) + sqrt((1 - Q) * (1 - Q) + 4 * Q * hover)) / (2 * Q);
    double gf[3] = { 0 }, af[3] = { 0, 0, 1 }, t_imu = 0.0003, t_ctrl = 0.001, alt_i = 0;
    float acc[3] = { 0, 0, 1 }, gyr[3] = { 0 };
    flight_t f = { .up = { 0, 0, 1 }, .init = true, .lift = 1, .air = !p.takeoff };   /* levelled on the ground; in the air: lifted off before */
    int duty[4] = { 0 }, last[4] = { 0 };
    result_t r = { 0 };
    double sum = 0, tilt = 0, dsum = 0, kick_peak = 0, last_peak = 0;
    long n = 0, nc = 0, nsat = 0;
    int ground = p.takeoff, flying = !p.takeoff;
    s_seed = 1;
    for (double t = 0; t < seconds; t += dt) {
        double F = 0;
        for (int i = 0; i < 4; i++) F += p.fmax * th[i];
        double vb[3], fb[3];   /* body velocity; body force: thrust + rotor drag */
        for (int k = 0; k < 3; k++) vb[k] = R[k] * vel[0] + R[3 + k] * vel[1] + R[6 + k] * vel[2];
        fb[0] = -kd * F / (m * g) * vb[0]; fb[1] = -kd * F / (m * g) * vb[1]; fb[2] = F;
        ground = ground && F * R[8] <= m * g;   /* on its feet until the thrust carries it */
        if (!ground && p.takeoff && !r.t_lift) { r.t_lift = t; r.i_lift = f.i[0]; }
        for (int k = 0; k < 3; k++) {   /* IMU: 1.43 ms group delay, sampled at 800 Hz */
            gf[k] += dt / 0.00143 * (w[k] * 180 / M_PI - gf[k]);   /* on the feet it reads 1 g up */
            af[k] += dt / 0.00143 * ((ground ? m * g * R[6 + k] : fb[k]) / (m * g) - af[k]);
        }
        if (t >= t_imu) {
            t_imu += 0.00125;
            for (int k = 0; k < 3; k++) { gyr[k] = (float)(gf[k] + 0.5 * noise()); acc[k] = (float)(af[k] + 0.05 * noise()); }
        }
        if (t >= t_ctrl) {   /* the firmware's 2 ms loop; the pilot holds 1 m with the throttle */
            const float cdt = 0.002f;
            t_ctrl += cdt;
            double thr;
            if (!flying) {   /* taking off like a pilot on the keys, until it has climbed 0.3 m */
                thr = t < 0.5 ? 0 : fmin(100 + 5 * floor((t - 0.5) / 0.6), 220);
                flying = pos[2] > 0.3;
            } else {
                alt_i += (1 - pos[2]) * cdt;
                thr = fmax(thr0 + 20 * (1 - pos[2]) - 15 * vel[2] + 2 * alt_i, 1);
            }
            flight_estimate(&f, acc, gyr, cdt);
            flight_control(&f, (float)thr, (float)p.trim, (t > 2 && t < 3) ? (float)p.steer : 0, (float)p.gain, gyr, cdt, duty);
            if (t > 1.5) {   /* motors pinned at 0 or full, duty jumping from one loop to the next */
                int pinned = 0;
                for (int i = 0; i < 4; i++) {
                    pinned |= duty[i] == 0 || duty[i] == 255;
                    dsum += (duty[i] - last[i]) * (duty[i] - last[i]);
                }
                nsat += pinned; nc++;
            }
            memcpy(last, duty, sizeof last);
        }
        for (int i = 0; i < 4; i++) {   /* motors */
            double target = (1 + (i % 2 ? -p.mis : p.mis)) * (Q * duty[i] * duty[i] / 65025.0 + (1 - Q) * duty[i] / 255.0);
            th[i] += (target - th[i]) * dt / (target > th[i] ? p.tau : 2 * p.tau);
        }
        if (ground) {   /* sitting level on its feet: no motion */
            memset(vel, 0, sizeof vel); memset(w, 0, sizeof w);
            continue;
        }
        double tq[3] = { 0 };
        for (int i = 0; i < 4; i++) {
            tq[0] += ry[i] * p.fmax * th[i];
            tq[1] -= (rx[i] - p.cg_x) * p.fmax * th[i];
            tq[2] += spin[i] * cM * p.fmax * th[i];
        }
        if (fabs(t - 1) < dt / 2) { w[0] += p.kick * M_PI / 180; w[1] += p.kick * M_PI / 180; }
        double wo[3] = { w[0], w[1], w[2] };
        for (int k = 0; k < 3; k++) {
            int a = (k + 1) % 3, b = (k + 2) % 3;
            w[k] += dt * (tq[k] - (w[a] * J[b] * w[b] - w[b] * J[a] * w[a])) / J[k];
        }
        for (int k = 0; k < 3; k++) {
            vel[k] += dt * ((R[k * 3] * fb[0] + R[k * 3 + 1] * fb[1] + R[k * 3 + 2] * fb[2]) / m - (k == 2 ? g : 0));
            pos[k] += dt * vel[k];
        }
        double wm[3] = { (w[0] + wo[0]) / 2, (w[1] + wo[1]) / 2, (w[2] + wo[2]) / 2 };
        rotate(R, wm, dt);
        double ang = acos(fmin(1, R[8])) * 180 / M_PI;   /* tilt from level */
        if (ang > 60 || pos[2] < -0.05) { r.crash = 1; break; }
        double rate = hypot(w[0], w[1]) * 180 / M_PI;
        if (t > 1) kick_peak = fmax(kick_peak, rate);
        if (t > seconds - 1) last_peak = fmax(last_peak, rate);
        if (r.t_lift > 0 && t < r.t_lift + 2) {   /* after lift-off: roll past its trim, pitch */
            r.over = fmax(r.over, atan2(R[7], R[8]) * 180 / M_PI - p.trim);
            r.pitch = fmax(r.pitch, fabs(asin(R[6]) * 180 / M_PI));
        }
        if (t > 1) r.peak = fmax(r.peak, ang);
        if (t > 1.5) { sum += (w[0] * w[0] + w[1] * w[1]) * 3283; n++; }
        if (t > seconds - 1) tilt = fmax(tilt, ang);
    }
    r.rms_rate = n ? sqrt(sum / n) : 0;
    r.sat = nc ? (double)nsat / nc : 0;
    r.chatter = nc ? sqrt(dsum / (4 * nc)) : 0;
    r.tilt = tilt;
    r.ring = kick_peak > 0 ? last_peak / kick_peak : 0;
    r.drift = hypot(pos[0], pos[1]);
    return r;
}

static int stable(plant_t p)   /* a 60 dps bump dies down within 3 s, no motor chatter */
{
    p.kick = 60;
    result_t r = fly(p, 5);
    return !r.crash && r.ring < 0.1 && r.sat < 0.01 && r.chatter < 20;
}

static void show(const char *name, plant_t p)
{
    result_t r = fly(p, 6);
    if (r.crash) printf("%-38s CRASH\n", name);
    else printf("%-38s peak %4.1f deg, tilt at the end %3.1f deg, rate noise %4.1f dps, duty jitter %4.1f, drift %3.1f m\n",
                name, r.peak, r.tilt, r.rms_rate, r.chatter, r.drift);
}

int main(void)
{
    plant_t p = NOMINAL;
    printf("nominal: 32 g, motor lag %.0f ms up / %.0f ms down, full thrust %.0f g per motor\n\n",
           p.tau * 1000, p.tau * 2000, p.fmax / 9.81 * 1000);
    show("hover", p);
    p.kick = 150; show("bump of 150 dps on roll and pitch", p);
    p = NOMINAL; p.cg_x = 0.002; p.mis = 0.05; show("CG 2 mm forward, motors +-5 %", p);
    p = NOMINAL; p.steer = 4; show("steer 4 deg forward for 1 s", p);
    p = NOMINAL; p.takeoff = 1; p.trim = 2; p.cg_x = 0.002;
    result_t t = fly(p, 8);
    if (t.crash || !t.t_lift) printf("take-off, 2 deg roll trim, CG 2 mm   %s\n", t.crash ? "CRASH" : "never lifted off");
    else printf("take-off, 2 deg roll trim, CG 2 mm     lifts off at %.1f s, roll integrator %.1f then; next 2 s: roll %.1f deg past trim, pitch %.1f deg\n",
                t.t_lift, t.i_lift, t.over, t.pitch);
    printf("\nstable range of the [ ] gain scale (1 = flight.c as is):\n");
    const double taus[] = { 0.05, 0.07, 0.10, 0.13 }, Js[] = { 0.75, 1, 1.3 };
    for (int a = 0; a < 4; a++)
        for (int b = 0; b < 3; b++) {
            p = NOMINAL; p.tau = taus[a]; p.J *= Js[b];
            double lo = 0, hi = 0;
            for (int k = -16; k <= 16; k++) {   /* quarter-octave steps, 1/16 .. 16 */
                p.gain = pow(2, k / 4.0);
                int ok = stable(p);
                if (ok && !lo) lo = p.gain;
                if (ok) hi = p.gain;
                if (!ok && hi) break;
            }
            printf("  motor lag %3.0f ms, inertia x%.2f: %s%.2f .. %.2f\n", taus[a] * 1000, Js[b],
                   lo <= 1 && hi >= 1 ? "" : "NOT STABLE AT 1: ", lo, hi);
        }
    return 0;
}
