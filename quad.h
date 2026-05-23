// Conventions:
//   World frame  W : right-handed, Y axis up. Gravity = (0, -g, 0).
//   Body  frame  B : X forward, Y up (thrust along +Y), Z right.
//   Rotation R_WB  : 3x3 row-major, columns are body axes in W,  v_W = R * v_B.
//
// X-configuration, viewed from above (+Y looking down at the XZ plane):
//
//        +X (forward)
//           ^
//     3 . . | . . 0          rotor   (x,   z )   spin
//      .    |    .             0   ( +a, +a )    CCW
//      .....|.....> +Z         1   ( -a, +a )    CW
//      .    |    .             2   ( -a, -a )    CCW
//     2 . . | . . 1            3   ( +a, -a )    CW
//
//   a = RDIST (per-axis offset; diagonal arm length = a*sqrt(2)).
//
// Each rotor: thrust f_i = K_F * w_i^2 along +Y_B, drag m_i = K_M * w_i^2.

#ifndef QUAD_H
#define QUAD_H

#include <math.h>
#include <string.h>


// ====================================================================
//  Physical constants
// ====================================================================

#define G          9.81
#define MASS       0.10            // kg
#define RDIST      0.0354          // per-axis rotor offset (m); diagonal = 10 cm
#define K_F        1.0e-7          // thrust coeff:  f = K_F * w^2 (N)
#define K_M        1.0e-8          // drag   coeff:  m = K_M * w^2 (N m)
#define OMEGA_MIN  50.0            // rotor rad/s
#define OMEGA_MAX  2200.0
#define IXX        1.2e-4          // roll  inertia
#define IYY        2.0e-4          // yaw   inertia (about thrust axis)
#define IZZ        1.2e-4          // pitch inertia

// Geometric controller gains
#define KP_POS     0.225
#define KP_VEL     0.30
#define KP_ROT     3.0e-3
#define KP_OMG     1.2e-3

// ====================================================================
//  Vector helpers (3-vectors)
// ====================================================================

static void vec_scale(double s, const double a[3], double r[3])
{
    r[0] = s * a[0];
    r[1] = s * a[1];
    r[2] = s * a[2];
}

static double vec_dot(const double a[3], const double b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static double vec_norm(const double a[3])
{
    return sqrt(vec_dot(a, a));
}

static void vec_cross(const double a[3], const double b[3], double r[3])
{
    r[0] = a[1]*b[2] - a[2]*b[1];
    r[1] = a[2]*b[0] - a[0]*b[2];
    r[2] = a[0]*b[1] - a[1]*b[0];
}

// Normalize a into r. Returns 0 if |a| is too small.
static int vec_normalize(const double a[3], double r[3])
{
    double n = vec_norm(a);
    if (n < 1e-12) return 0;
    vec_scale(1.0 / n, a, r);
    return 1;
}


// ====================================================================
//  Matrix helpers (row-major 3x3)
// ====================================================================

// C = A * B
static void mat_mul(const double A[9], const double B[9], double C[9])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i*3 + j] = A[i*3 + 0] * B[0*3 + j]
                       + A[i*3 + 1] * B[1*3 + j]
                       + A[i*3 + 2] * B[2*3 + j];
        }
    }
}

// R = A^T
static void mat_transpose(const double A[9], double R[9])
{
    R[0] = A[0];  R[1] = A[3];  R[2] = A[6];
    R[3] = A[1];  R[4] = A[4];  R[5] = A[7];
    R[6] = A[2];  R[7] = A[5];  R[8] = A[8];
}


// ====================================================================
//  SO(3) helpers
// ====================================================================

// Axial vector of the skew part 0.5 (M - M^T).
static void so3_vee(const double M[9], double w[3])
{
    w[0] = 0.5 * (M[7] - M[5]);
    w[1] = 0.5 * (M[2] - M[6]);
    w[2] = 0.5 * (M[3] - M[1]);
}

// Exact exponential map R = expm([w]_x) via Rodrigues' formula,
// with a Taylor expansion fallback for small |w| to avoid 0/0.
static void so3_exp(const double w[3], double R[9])
{
    double t2 = vec_dot(w, w);
    double t  = sqrt(t2);

    // a = sin(t)/t,  b = (1 - cos(t))/t^2
    double a, b;
    if (t < 1e-8) {
        a = 1.0 - t2 / 6.0;
        b = 0.5 - t2 / 24.0;
    } else {
        a = sin(t) / t;
        b = (1.0 - cos(t)) / t2;
    }

    double K[9] = {
         0.0 , -w[2],  w[1],
         w[2],  0.0 , -w[0],
        -w[1],  w[0],  0.0
    };
    double K2[9];
    mat_mul(K, K, K2);

    for (int i = 0; i < 9; i++) {
        double I_ij = (i % 4 == 0) ? 1.0 : 0.0;   // identity diagonal
        R[i] = I_ij + a * K[i] + b * K2[i];
    }
}


// ====================================================================
//  Quad state and initialization
// ====================================================================

typedef struct {
    double p[3];        // position in W
    double v[3];        // linear  velocity in W
    double R[9];        // attitude R_WB (body -> world)
    double w[3];        // angular velocity in B
    double omega[4];    // rotor speeds (rad/s, >= 0)
    double I[3];        // diagonal inertia in B
} Quad;

static void quad_init(Quad *q, double x, double y, double z, double yaw)
{
    memset(q, 0, sizeof(*q));

    q->p[0] = x;
    q->p[1] = y;
    q->p[2] = z;

    // R = Ry(yaw): rotation about world +Y
    double c = cos(yaw);
    double s = sin(yaw);
    double Ry[9] = {
         c, 0, s,
         0, 1, 0,
        -s, 0, c
    };
    memcpy(q->R, Ry, sizeof(Ry));

    q->I[0] = IXX;
    q->I[1] = IYY;
    q->I[2] = IZZ;

    // Start at hover throttle.
    double w_hover = sqrt(MASS * G / (4.0 * K_F));
    for (int i = 0; i < 4; i++) {
        q->omega[i] = w_hover;
    }
}


// ====================================================================
//  Rigid-body dynamics (semi-implicit / symplectic Euler at ~1 kHz)
// ====================================================================

static void quad_step(Quad *q, const double omega_cmd[4], double dt)
{
    // 0. Saturate and apply the commanded rotor speeds (no rotor dynamics).
    for (int i = 0; i < 4; i++) {
        double w = omega_cmd[i];
        if (w < OMEGA_MIN) w = OMEGA_MIN;
        if (w > OMEGA_MAX) w = OMEGA_MAX;
        q->omega[i] = w;
    }

    // 1. Body-frame thrust (along +Y_B) and torque from rotor speeds.
    //    tau_x = sum -z_i f_i, tau_y = sum s_i m_i, tau_z = sum x_i f_i.
    const double a    = RDIST;
    const double X[4] = { +a, -a, -a, +a };
    const double Z[4] = { +a, +a, -a, -a };
    const double S[4] = { +1, -1, +1, -1 };   // + for CCW spin

    double thrust = 0.0;
    double tau[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < 4; i++) {
        double w2 = q->omega[i] * q->omega[i];
        double f  = K_F * w2;
        double m  = K_M * w2;
        thrust += f;
        tau[0] += -Z[i] * f;
        tau[1] +=  S[i] * m;
        tau[2] +=  X[i] * f;
    }

    // 2a. Translational acceleration in W:  a_W = (1/m) R [0,T,0]^T + [0,-g,0].
    double dv[3];
    dv[0] =  q->R[1] * thrust / MASS;
    dv[1] =  q->R[4] * thrust / MASS - G;
    dv[2] =  q->R[7] * thrust / MASS;

    // 2b. Angular acceleration in B (Euler's equation):  w_dot = I^-1 (tau - w x I w).
    double w_old[3];
    memcpy(w_old, q->w, sizeof(w_old));

    double Iw[3]  = { q->I[0]*w_old[0], q->I[1]*w_old[1], q->I[2]*w_old[2] };
    double wIw[3];
    vec_cross(w_old, Iw, wIw);

    double dw[3];
    dw[0] = (tau[0] - wIw[0]) / q->I[0];
    dw[1] = (tau[1] - wIw[1]) / q->I[1];
    dw[2] = (tau[2] - wIw[2]) / q->I[2];

    // 3. Velocity update (linear and angular).
    for (int i = 0; i < 3; i++) {
        q->v[i] += dv[i] * dt;
        q->w[i] += dw[i] * dt;
    }

    // 4. Position update using the new velocity (symplectic).
    for (int i = 0; i < 3; i++) {
        q->p[i] += q->v[i] * dt;
    }

    // 5. Attitude update:  R <- R * expm([0.5 (w_old + w_new) dt]_x).
    double phi[3];
    for (int i = 0; i < 3; i++) {
        phi[i] = 0.5 * dt * (w_old[i] + q->w[i]);
    }
    double dR[9];
    double Rn[9];
    so3_exp(phi, dR);
    mat_mul(q->R, dR, Rn);
    memcpy(q->R, Rn, sizeof(Rn));

    // 6. Rigid floor at y = 0.
    if (q->p[1] < 0.0) {
        q->p[1] = 0.0;
        if (q->v[1] < 0.0) q->v[1] = 0.0;
    }
}


// ====================================================================
//  Geometric controller (adapted to Y-up body frame)
// ====================================================================

// target = [ px, py, pz, vx, vy, vz, yaw ]
static void quad_control(const Quad *q, const double target[7], double omega_cmd[4])
{
    // 1. Position and velocity errors (current - desired).
    double ep[3], ev[3];
    for (int i = 0; i < 3; i++) {
        ep[i] = q->p[i] - target[i];
        ev[i] = q->v[i] - target[3 + i];
    }

    // 2. Desired force in world frame:  F = -Kp ep - Kv ev + m g e_y.
    double F[3];
    F[0] = -KP_POS * ep[0] - KP_VEL * ev[0];
    F[1] = -KP_POS * ep[1] - KP_VEL * ev[1] + MASS * G;
    F[2] = -KP_POS * ep[2] - KP_VEL * ev[2];

    // 3. Desired body-Y axis = thrust direction.
    double b2d[3];
    if (!vec_normalize(F, b2d)) {
        b2d[0] = 0.0;
        b2d[1] = 1.0;
        b2d[2] = 0.0;
    }

    // Thrust magnitude projected onto current body-Y (decouples tilt lag from thrust).
    double b2[3] = { q->R[1], q->R[4], q->R[7] };
    double thrust = vec_dot(F, b2);
    if (thrust < 0.0) thrust = 0.0;

    // 4. Build desired attitude R_d. Columns are body axes (b1d, b2d, b3d) in W.
    //    Heading vector c = body-X for given yaw, with R = Ry(psi).
    double psi  = target[6];
    double c[3] = { cos(psi), 0.0, -sin(psi) };

    double tmp[3], b3d[3], b1d[3];
    vec_cross(c, b2d, tmp);
    if (!vec_normalize(tmp, b3d)) {
        // Degenerate: c parallel to b2d. Pick a different reference.
        double c2[3] = { 0.0, 0.0, 1.0 };
        vec_cross(c2, b2d, tmp);
        vec_normalize(tmp, b3d);
    }
    vec_cross(b2d, b3d, b1d);   // unit by construction

    double Rd[9] = {
        b1d[0], b2d[0], b3d[0],
        b1d[1], b2d[1], b3d[1],
        b1d[2], b2d[2], b3d[2]
    };

    // 5. Attitude error in body frame:  e_R = 0.5 * vee(Rd^T R - R^T Rd).
    double RdT[9], RT[9], A[9], B[9], E[9];
    mat_transpose(Rd,   RdT);
    mat_transpose(q->R, RT);
    mat_mul(RdT, q->R, A);
    mat_mul(RT,  Rd,   B);
    for (int i = 0; i < 9; i++) {
        E[i] = A[i] - B[i];
    }
    double eR[3];
    so3_vee(E, eR);
    vec_scale(0.5, eR, eR);

    // 6. Angular velocity error (desired body rate = 0).
    double eW[3] = { q->w[0], q->w[1], q->w[2] };

    // 7. Control torque:  tau = -KR eR - Kw eW + w x (I w).
    double Iw[3] = { q->I[0]*q->w[0], q->I[1]*q->w[1], q->I[2]*q->w[2] };
    double wIw[3];
    vec_cross(q->w, Iw, wIw);

    double tau[3];
    for (int i = 0; i < 3; i++) {
        tau[i] = -KP_ROT * eR[i] - KP_OMG * eW[i] + wIw[i];
    }

    // 8. Invert the X-config mixer for rotor speed^2.
    //    Rows of M are [T; tx; ty; tz] = M * [w0^2; w1^2; w2^2; w3^2].
    const double a    = RDIST;
    const double iKf  = 1.0 / (4.0 * K_F);
    const double iKm  = 1.0 / (4.0 * K_M);
    const double iaKf = 1.0 / (4.0 * a * K_F);

    double w_sq[4];
    w_sq[0] = thrust*iKf  - tau[0]*iaKf  + tau[1]*iKm  + tau[2]*iaKf;
    w_sq[1] = thrust*iKf  - tau[0]*iaKf  - tau[1]*iKm  - tau[2]*iaKf;
    w_sq[2] = thrust*iKf  + tau[0]*iaKf  + tau[1]*iKm  - tau[2]*iaKf;
    w_sq[3] = thrust*iKf  + tau[0]*iaKf  - tau[1]*iKm  + tau[2]*iaKf;

    for (int i = 0; i < 4; i++) {
        if (w_sq[i] < 0.0) w_sq[i] = 0.0;
        omega_cmd[i] = sqrt(w_sq[i]);
    }
}

#endif // QUAD_H
