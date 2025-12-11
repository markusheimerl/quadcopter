#ifndef QUAD_H
#define QUAD_H

#include <math.h>
#include <string.h>

// ============================================================================
// VECTOR OPERATIONS (ℝ³)
// ============================================================================

void crossVec3f(const double* a, const double* b, double* result) {
    // result = a × b
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

void multScalVec3f(double s, const double* v, double* result) {
    // result = s * v
    for (int i = 0; i < 3; i++) result[i] = s * v[i];
}

void addVec3f(const double* a, const double* b, double* result) {
    // result = a + b
    for (int i = 0; i < 3; i++) result[i] = a[i] + b[i];
}

void subVec3f(const double* a, const double* b, double* result) {
    // result = a - b
    for (int i = 0; i < 3; i++) result[i] = a[i] - b[i];
}

double dotVec3f(const double* a, const double* b) {
    // return a · b
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

double normVec3f(const double* v) {
    // return ‖v‖
    return sqrt(dotVec3f(v, v));
}

void normalizeVec3f(const double* v, double* result) {
    // result = v / ‖v‖
    double mag = normVec3f(v);
    for (int i = 0; i < 3; i++) result[i] = v[i] / mag;
}

// ============================================================================
// MATRIX OPERATIONS (for inertia tensor and control allocation)
// ============================================================================

void multMatVec3f(const double* M, const double* v, double* result) {
    // result = M * v  (3×3 matrix × 3×1 vector, row-major storage)
    result[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
    result[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
    result[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
}

void diagMat3f(const double* v, double* M) {
    // M = diag(v)  (diagonal matrix from vector)
    for (int i = 0; i < 9; i++) M[i] = 0;
    M[0] = v[0];
    M[4] = v[1];
    M[8] = v[2];
}

void multMatVec4f(const double* M, const double* v, double* result) {
    // result = M * v  (4×4 matrix × 4×1 vector, row-major storage)
    result[0] = M[0]*v[0]  + M[1]*v[1]  + M[2]*v[2]  + M[3]*v[3];
    result[1] = M[4]*v[0]  + M[5]*v[1]  + M[6]*v[2]  + M[7]*v[3];
    result[2] = M[8]*v[0]  + M[9]*v[1]  + M[10]*v[2] + M[11]*v[3];
    result[3] = M[12]*v[0] + M[13]*v[1] + M[14]*v[2] + M[15]*v[3];
}

void inv4Mat4f(const double* m, double* result) {
    // result = m⁻¹  (4×4 matrix inverse using cofactor expansion)
    double s0 = m[0]*m[5] - m[4]*m[1];
    double s1 = m[0]*m[6] - m[4]*m[2];
    double s2 = m[0]*m[7] - m[4]*m[3];
    double s3 = m[1]*m[6] - m[5]*m[2];
    double s4 = m[1]*m[7] - m[5]*m[3];
    double s5 = m[2]*m[7] - m[6]*m[3];

    double c5 = m[10]*m[15] - m[14]*m[11];
    double c4 = m[9]*m[15]  - m[13]*m[11];
    double c3 = m[9]*m[14]  - m[13]*m[10];
    double c2 = m[8]*m[15]  - m[12]*m[11];
    double c1 = m[8]*m[14]  - m[12]*m[10];
    double c0 = m[8]*m[13]  - m[12]*m[9];

    double det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0;
    if (det == 0.0) return;
    double invdet = 1.0 / det;

    result[0]  = ( m[5]*c5 - m[6]*c4 + m[7]*c3) * invdet;
    result[1]  = (-m[1]*c5 + m[2]*c4 - m[3]*c3) * invdet;
    result[2]  = ( m[13]*s5 - m[14]*s4 + m[15]*s3) * invdet;
    result[3]  = (-m[9]*s5 + m[10]*s4 - m[11]*s3) * invdet;
    result[4]  = (-m[4]*c5 + m[6]*c2 - m[7]*c1) * invdet;
    result[5]  = ( m[0]*c5 - m[2]*c2 + m[3]*c1) * invdet;
    result[6]  = (-m[12]*s5 + m[14]*s2 - m[15]*s1) * invdet;
    result[7]  = ( m[8]*s5 - m[10]*s2 + m[11]*s1) * invdet;
    result[8]  = ( m[4]*c4 - m[5]*c2 + m[7]*c0) * invdet;
    result[9]  = (-m[0]*c4 + m[1]*c2 - m[3]*c0) * invdet;
    result[10] = ( m[12]*s4 - m[13]*s2 + m[15]*s0) * invdet;
    result[11] = (-m[8]*s4 + m[9]*s2 - m[11]*s0) * invdet;
    result[12] = (-m[4]*c3 + m[5]*c1 - m[6]*c0) * invdet;
    result[13] = ( m[0]*c3 - m[1]*c1 + m[2]*c0) * invdet;
    result[14] = (-m[12]*s3 + m[13]*s1 - m[14]*s0) * invdet;
    result[15] = ( m[8]*s3 - m[9]*s1 + m[10]*s0) * invdet;
}

// ============================================================================
// QUATERNION OPERATIONS
// Unit quaternion q = [w, x, y, z] ∈ ℍ represents rotation in SO(3)
// Convention: q = [cos(θ/2), sin(θ/2)·n] for rotation of angle θ around axis n
// ============================================================================

void quatMultiply(const double* q1, const double* q2, double* result) {
    // Hamilton product: result = q1 ⊗ q2
    // [w1,v1] ⊗ [w2,v2] = [w1w2 - v1·v2, w1v2 + w2v1 + v1×v2]
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void quatConjugate(const double* q, double* result) {
    // Conjugate: q* = [w, -x, -y, -z]
    // For unit quaternions: q* = q⁻¹
    result[0] =  q[0];
    result[1] = -q[1];
    result[2] = -q[2];
    result[3] = -q[3];
}

void quatNormalize(double* q) {
    // Normalize to unit quaternion: q = q / ‖q‖
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; i++) q[i] /= norm;
}

void quatRotateVec(const double* q, const double* v, double* result) {
    // Rotate vector by quaternion: v' = q ⊗ [0,v] ⊗ q*
    // Equivalent to: v' = R(q) · v where R(q) is the rotation matrix
    double v_quat[4] = {0, v[0], v[1], v[2]};
    double q_conj[4], temp[4], rotated[4];
    quatConjugate(q, q_conj);
    quatMultiply(q, v_quat, temp);
    quatMultiply(temp, q_conj, rotated);
    result[0] = rotated[1];
    result[1] = rotated[2];
    result[2] = rotated[3];
}

void quatRotateVecInverse(const double* q, const double* v, double* result) {
    // Inverse rotation: v' = q* ⊗ [0,v] ⊗ q
    // Equivalent to: v' = R(q)ᵀ · v
    double q_conj[4];
    quatConjugate(q, q_conj);
    quatRotateVec(q_conj, v, result);
}

void quatToRotMat(const double* q, double* R) {
    // Convert quaternion to 3×3 rotation matrix (row-major storage)
    // R(q) = I + 2w[v]ₓ + 2[v]ₓ²  where q = [w, v]
    double w = q[0], x = q[1], y = q[2], z = q[3];
    R[0] = 1 - 2*(y*y + z*z);  R[1] = 2*(x*y - w*z);      R[2] = 2*(x*z + w*y);
    R[3] = 2*(x*y + w*z);      R[4] = 1 - 2*(x*x + z*z);  R[5] = 2*(y*z - w*x);
    R[6] = 2*(x*z - w*y);      R[7] = 2*(y*z + w*x);      R[8] = 1 - 2*(x*x + y*y);
}

void rotMatToQuat(const double* R, double* q) {
    // Convert 3×3 rotation matrix to quaternion (Shepperd's method)
    double trace = R[0] + R[4] + R[8];
    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        q[0] = 0.25 / s;
        q[1] = (R[7] - R[5]) * s;
        q[2] = (R[2] - R[6]) * s;
        q[3] = (R[3] - R[1]) * s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = 2.0 * sqrt(1.0 + R[0] - R[4] - R[8]);
        q[0] = (R[7] - R[5]) / s;
        q[1] = 0.25 * s;
        q[2] = (R[1] + R[3]) / s;
        q[3] = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        double s = 2.0 * sqrt(1.0 + R[4] - R[0] - R[8]);
        q[0] = (R[2] - R[6]) / s;
        q[1] = (R[1] + R[3]) / s;
        q[2] = 0.25 * s;
        q[3] = (R[5] + R[7]) / s;
    } else {
        double s = 2.0 * sqrt(1.0 + R[8] - R[0] - R[4]);
        q[0] = (R[3] - R[1]) / s;
        q[1] = (R[2] + R[6]) / s;
        q[2] = (R[5] + R[7]) / s;
        q[3] = 0.25 * s;
    }
    quatNormalize(q);
}

void quatFromBasis(const double* bx, const double* by, const double* bz, double* q) {
    // Build quaternion from orthonormal basis vectors (body frame axes in world frame)
    // R = [bx | by | bz] where each column is a basis vector
    double R[9] = {
        bx[0], by[0], bz[0],
        bx[1], by[1], bz[1],
        bx[2], by[2], bz[2]
    };
    rotMatToQuat(R, q);
}

// ============================================================================
// QUADCOPTER PHYSICAL CONSTANTS
// ============================================================================

#define K_F 0.0004905       // Thrust coefficient [N/(rad/s)²]
#define K_M 0.00004905      // Moment coefficient [N·m/(rad/s)²]
#define L 0.25              // Arm length [m]
#define GRAVITY 9.81        // Gravitational acceleration [m/s²]
#define MASS 0.5            // Total mass [kg]
#define OMEGA_MIN 30.0      // Minimum rotor speed [rad/s]
#define OMEGA_MAX 70.0      // Maximum rotor speed [rad/s]

// ============================================================================
// QUADCOPTER STATE
// ============================================================================

typedef struct {
    double omega[4];                // Rotor speeds ω_i [rad/s]
    double linear_position_W[3];    // Position p ∈ ℝ³ in world frame [m]
    double linear_velocity_W[3];    // Velocity v ∈ ℝ³ in world frame [m/s]
    double angular_velocity_B[3];   // Angular velocity ω ∈ ℝ³ in body frame [rad/s]
    double q_W_B[4];                // Orientation quaternion q ∈ ℍ (world ← body)
    double inertia[3];              // Principal moments of inertia [kg·m²]
    double omega_next[4];           // Commanded rotor speeds [rad/s]
} Quad;

Quad create_quad(double x, double y, double z, double yaw) {
    Quad quad;
    
    // Initialize rotor speeds to zero
    memcpy(quad.omega, (double[]){0.0, 0.0, 0.0, 0.0}, 4 * sizeof(double));
    
    // Initialize position
    memcpy(quad.linear_position_W, (double[]){x, y, z}, 3 * sizeof(double));
    
    // Initialize velocities to zero
    memcpy(quad.linear_velocity_W, (double[]){0.0, 0.0, 0.0}, 3 * sizeof(double));
    memcpy(quad.angular_velocity_B, (double[]){0.0, 0.0, 0.0}, 3 * sizeof(double));
    
    // Initialize orientation: rotation of 'yaw' around Y-axis (up)
    // q = [cos(yaw/2), 0, sin(yaw/2), 0]
    quad.q_W_B[0] = cos(yaw / 2.0);
    quad.q_W_B[1] = 0.0;
    quad.q_W_B[2] = sin(yaw / 2.0);
    quad.q_W_B[3] = 0.0;
    
    // Initialize inertia tensor (diagonal)
    memcpy(quad.inertia, (double[]){0.01, 0.02, 0.01}, 3 * sizeof(double));
    
    // Initialize commanded rotor speeds
    memcpy(quad.omega_next, (double[]){0.0, 0.0, 0.0, 0.0}, 4 * sizeof(double));
    
    return quad;
}

// ============================================================================
// PHYSICS UPDATE
// Integrates the rigid body dynamics of the quadcopter
// ============================================================================

void update_quad_states(
    // Current state
    const double* omega,                // ω_i: Rotor speeds [4]
    const double* linear_position_W,    // p: Position in world frame [3]
    const double* linear_velocity_W,    // v: Velocity in world frame [3]
    const double* angular_velocity_B,   // ω: Angular velocity in body frame [3]
    const double* q_W_B,                // q: Orientation quaternion [4]
    const double* inertia,              // I: Principal moments of inertia [3]
    const double* omega_next,           // ω_i,cmd: Commanded rotor speeds [4]
    double dt,                          // Δt: Time step [s]
    // Output (new state)
    double* new_linear_position_W,
    double* new_linear_velocity_W,
    double* new_angular_velocity_B,
    double* new_q_W_B,
    double* new_omega
) {
    // -------------------------------------------------------------------------
    // Step 1: Compute rotor forces and moments
    // f_i = k_f · |ω_i| · ω_i  (thrust, positive upward in body frame)
    // m_i = k_m · |ω_i| · ω_i  (reaction torque)
    // -------------------------------------------------------------------------
    double f[4], m[4];
    for (int i = 0; i < 4; i++) {
        double omega_sq = omega[i] * fabs(omega[i]);
        f[i] = K_F * omega_sq;
        m[i] = K_M * omega_sq;
    }

    // -------------------------------------------------------------------------
    // Step 2: Compute total thrust and body-frame torques
    // T = Σ f_i
    // τ_B = Σ (r_i × f_i·ŷ) + τ_drag
    double thrust = f[0] + f[1] + f[2] + f[3];
    
    // Yaw torque from rotor drag: τ_yaw = m_0 - m_1 + m_2 - m_3 (CW positive)
    double tau_B[3] = {0.0, m[0] - m[1] + m[2] - m[3], 0.0};
    
    // Rotor positions in body frame
    const double r[4][3] = {
        {-L, 0.0,  L},   // Rotor 0
        { L, 0.0,  L},   // Rotor 1
        { L, 0.0, -L},   // Rotor 2
        {-L, 0.0, -L}    // Rotor 3
    };
    
    // Add torques from thrust forces: τ += r_i × (f_i · ŷ)
    for (int i = 0; i < 4; i++) {
        double f_vec[3] = {0.0, f[i], 0.0};
        double tau_i[3];
        crossVec3f(r[i], f_vec, tau_i);
        addVec3f(tau_B, tau_i, tau_B);
    }

    // -------------------------------------------------------------------------
    // Step 3: Linear dynamics (Newton's second law in world frame)
    // F_W = R(q) · [0, T, 0]ᵀ  (thrust in world frame)
    // a = F_W/m - g·ŷ
    // -------------------------------------------------------------------------
    double f_thrust_B[3] = {0.0, thrust, 0.0};
    double f_thrust_W[3];
    quatRotateVec(q_W_B, f_thrust_B, f_thrust_W);

    double linear_accel_W[3];
    for (int i = 0; i < 3; i++) {
        linear_accel_W[i] = f_thrust_W[i] / MASS;
    }
    linear_accel_W[1] -= GRAVITY;  // Subtract gravity (Y-up)

    // -------------------------------------------------------------------------
    // Step 4: Integrate linear dynamics (semi-implicit Euler)
    // v(t+Δt) = v(t) + Δt · a(t)
    // p(t+Δt) = p(t) + Δt · v(t+Δt)
    // -------------------------------------------------------------------------
    for (int i = 0; i < 3; i++) {
        new_linear_velocity_W[i] = linear_velocity_W[i] + dt * linear_accel_W[i];
        new_linear_position_W[i] = linear_position_W[i] + dt * new_linear_velocity_W[i];
    }
    
    // Ground constraint
    if (new_linear_position_W[1] < 0.0) new_linear_position_W[1] = 0.0;

    // -------------------------------------------------------------------------
    // Step 5: Angular dynamics (Euler's equation in body frame)
    // I·ω̇ = τ_B - ω × (I·ω)
    // ω̇ = I⁻¹ · (τ_B - ω × (I·ω))
    // -------------------------------------------------------------------------
    double I_mat[9];
    diagMat3f(inertia, I_mat);
    
    double h_B[3];      // Angular momentum: h = I·ω
    double w_cross_h[3]; // Gyroscopic term: ω × h
    multMatVec3f(I_mat, angular_velocity_B, h_B);
    crossVec3f(angular_velocity_B, h_B, w_cross_h);

    // -------------------------------------------------------------------------
    // Step 6: Integrate angular dynamics (explicit Euler)
    // ω(t+Δt) = ω(t) + Δt · ω̇(t)
    // -------------------------------------------------------------------------
    for (int i = 0; i < 3; i++) {
        double angular_accel = (tau_B[i] - w_cross_h[i]) / inertia[i];
        new_angular_velocity_B[i] = angular_velocity_B[i] + dt * angular_accel;
    }

    // -------------------------------------------------------------------------
    // Step 7: Quaternion kinematics
    // q̇ = ½ · q ⊗ [0, ω]
    // q(t+Δt) = q(t) + Δt · q̇(t)
    // -------------------------------------------------------------------------
    double omega_quat[4] = {0.0, angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2]};
    double q_dot[4];
    quatMultiply(q_W_B, omega_quat, q_dot);
    
    for (int i = 0; i < 4; i++) new_q_W_B[i] = q_W_B[i] + 0.5 * dt * q_dot[i];
    quatNormalize(new_q_W_B);  // Re-normalize to maintain unit quaternion

    // -------------------------------------------------------------------------
    // Step 8: Update rotor speeds with saturation
    // ω_i(t+Δt) = clamp(ω_i,cmd, ω_min, ω_max)
    // -------------------------------------------------------------------------
    for (int i = 0; i < 4; i++) {
        new_omega[i] = fmax(OMEGA_MIN, fmin(OMEGA_MAX, omega_next[i]));
    }
}

// ============================================================================
// GEOMETRIC CONTROLLER
// SE(3) geometric tracking controller using quaternion representation
// ============================================================================

void control_quad_commands(
    // Current state (from sensors/estimator)
    const double* position,         // p: Current position [3]
    const double* velocity,         // v: Current velocity [3]
    const double* q_W_B,            // q: Current orientation quaternion [4]
    const double* omega,            // ω: Current angular velocity [3]
    const double* inertia,          // I: Inertia tensor (diagonal) [3]
    // Desired state
    const double* target,           // [p_d(3), v_d(3), ψ_d(1)]
    // Controller gains
    double K_P, double K_V, double K_R, double K_W,
    // Output
    double* omega_cmd               // Commanded rotor speeds [4]
) {
    // -------------------------------------------------------------------------
    // Step 1: Extract target values
    // -------------------------------------------------------------------------
    double p_d[3] = {target[0], target[1], target[2]};  // Desired position
    double v_d[3] = {target[3], target[4], target[5]};  // Desired velocity
    double yaw_d  = target[6];                          // Desired yaw angle

    // -------------------------------------------------------------------------
    // Step 2: Position and velocity errors
    // e_p = p - p_d
    // e_v = v - v_d
    // -------------------------------------------------------------------------
    double e_p[3], e_v[3];
    subVec3f(position, p_d, e_p);
    subVec3f(velocity, v_d, e_v);

    // -------------------------------------------------------------------------
    // Step 3: Desired force vector (in world frame)
    // F_d = -K_p·e_p - K_v·e_v + m·g·ŷ + m·a_d
    // This determines the desired thrust direction
    // -------------------------------------------------------------------------
    double F_d[3], temp[3];
    
    multScalVec3f(-K_P, e_p, F_d);           // -K_p · e_p
    multScalVec3f(-K_V, e_v, temp);          // -K_v · e_v
    addVec3f(F_d, temp, F_d);
    
    double gravity_comp[3] = {0.0, MASS * GRAVITY, 0.0};
    addVec3f(F_d, gravity_comp, F_d);        // + m·g·ŷ
    
    // Note: a_d = 0 for setpoint tracking (could add feedforward for trajectories)

    // -------------------------------------------------------------------------
    // Step 4: Thrust magnitude
    // T = F_d · (R·ŷ) = F_d · z_B
    // where z_B is the body Y-axis (thrust direction) expressed in world frame
    // -------------------------------------------------------------------------
    double y_body[3] = {0.0, 1.0, 0.0};
    double z_B_W[3];  // Body Y-axis in world frame
    quatRotateVec(q_W_B, y_body, z_B_W);
    
    double thrust = dotVec3f(F_d, z_B_W);

    // -------------------------------------------------------------------------
    // Step 5: Desired orientation
    // Construct desired body frame from F_d direction and desired yaw
    // 
    // z_d = F_d / ‖F_d‖
    // x̃_d = [sin(ψ_d), 0, cos(ψ_d)]
    // x_d = (z_d × x̃_d) × z_d
    // y_d = z_d × x̃_d
    // -------------------------------------------------------------------------
    double x_tilde_d[3] = {sin(yaw_d), 0.0, cos(yaw_d)};
    
    double temp1[3], temp2[3];
    double b_x[3], b_y[3], b_z[3];  // Desired body frame axes in world frame
    
    crossVec3f(F_d, x_tilde_d, temp1);       // z_d × x̃_d
    crossVec3f(temp1, F_d, temp2);           // (z_d × x̃_d) × z_d
    
    normalizeVec3f(temp2, b_z);              // Body Z-axis (normalized)
    normalizeVec3f(temp1, b_x);              // Body X-axis (normalized)
    normalizeVec3f(F_d, b_y);                // Body Y-axis (thrust direction)
    
    // Convert desired frame to quaternion
    double q_W_d[4];
    quatFromBasis(b_x, b_y, b_z, q_W_d);

    // -------------------------------------------------------------------------
    // Step 6: Attitude error (quaternion formulation)
    // q_err = q_d* ⊗ q  (error quaternion: rotation from desired to actual)
    // 
    // The attitude error vector is:
    // e_R = 2·w_err·[x_err, y_err, z_err]
    // 
    // This is equivalent to the SO(3) error:
    // e_R = ½·vee(R_d^T·R - R^T·R_d)
    // -------------------------------------------------------------------------
    double q_d_conj[4], q_err[4];
    quatConjugate(q_W_d, q_d_conj);
    quatMultiply(q_d_conj, q_W_B, q_err);
    
    double e_R[3] = {
        2.0 * q_err[0] * q_err[1],
        2.0 * q_err[0] * q_err[2],
        2.0 * q_err[0] * q_err[3]
    };

    // -------------------------------------------------------------------------
    // Step 7: Angular velocity error
    // e_ω = ω - R^T·R_d·ω_d
    // For setpoint tracking with ω_d = 0: e_ω = ω
    // -------------------------------------------------------------------------
    double omega_d[3] = {0.0, 0.0, 0.0};
    double omega_d_B[3];
    
    // Transform ω_d to body frame: ω_d_B = R^T·R_d·ω_d
    double q_B_conj[4], q_rel[4];
    quatConjugate(q_W_B, q_B_conj);
    quatMultiply(q_B_conj, q_W_d, q_rel);    // q_rel represents R^T·R_d
    quatRotateVec(q_rel, omega_d, omega_d_B);
    
    double e_omega[3];
    subVec3f(omega, omega_d_B, e_omega);

    // -------------------------------------------------------------------------
    // Step 8: Control torque (PD control + feedforward)
    // τ = -K_R·e_R - K_ω·e_ω + ω × I·ω - I·(ω × R^T·R_d·ω_d - R^T·R_d·ω̇_d)
    // 
    // For setpoint tracking (ω_d = 0, ω̇_d = 0):
    // τ = -K_R·e_R - K_ω·e_ω + ω × I·ω
    // -------------------------------------------------------------------------
    double tau_ctrl[3];
    
    // PD terms
    multScalVec3f(-K_R, e_R, tau_ctrl);      // -K_R · e_R
    multScalVec3f(-K_W, e_omega, temp);      // -K_ω · e_ω
    addVec3f(tau_ctrl, temp, tau_ctrl);
    
    // Gyroscopic compensation: ω × I·ω
    double I_mat[9], I_omega[3], gyro[3];
    diagMat3f(inertia, I_mat);
    multMatVec3f(I_mat, omega, I_omega);
    crossVec3f(omega, I_omega, gyro);
    addVec3f(tau_ctrl, gyro, tau_ctrl);
    
    // Feedforward terms (for trajectory tracking with ω_d ≠ 0)
    double omega_d_dot[3] = {0.0, 0.0, 0.0};  // Desired angular acceleration
    double ff_term1[3], ff_term2[3], ff_total[3];
    
    quatRotateVec(q_W_d, omega_d_dot, temp);
    quatRotateVecInverse(q_W_B, temp, ff_term1);  // R^T·R_d·ω̇_d
    
    quatRotateVec(q_W_d, omega_d, temp);
    quatRotateVecInverse(q_W_B, temp, temp1);     // R^T·R_d·ω_d
    crossVec3f(omega, temp1, ff_term2);           // ω × R^T·R_d·ω_d
    
    subVec3f(ff_term2, ff_term1, ff_total);
    multMatVec3f(I_mat, ff_total, temp);
    subVec3f(tau_ctrl, temp, tau_ctrl);

    // -------------------------------------------------------------------------
    // Step 9: Control allocation (thrust and torques → rotor speeds)
    // [T, τ_x, τ_y, τ_z]^T = F · [ω_0², ω_1², ω_2², ω_3²]^T
    // 
    // Solve: [ω_0², ω_1², ω_2², ω_3²]^T = F⁻¹ · [T, τ_x, τ_y, τ_z]^T
    // -------------------------------------------------------------------------
    
    // Build control effectiveness matrix F
    double F[16] = {
        K_F,  K_F,  K_F,  K_F,   // Thrust coefficients
        0.0,  0.0,  0.0,  0.0,   // Roll torque coefficients (computed below)
        K_M, -K_M,  K_M, -K_M,   // Yaw torque coefficients
        0.0,  0.0,  0.0,  0.0    // Pitch torque coefficients (computed below)
    };
    
    // Compute roll and pitch torque coefficients from rotor geometry
    const double rotor_pos[4][3] = {
        {-L, 0.0,  L},   // Rotor 0
        { L, 0.0,  L},   // Rotor 1
        { L, 0.0, -L},   // Rotor 2
        {-L, 0.0, -L}    // Rotor 3
    };
    
    for (int i = 0; i < 4; i++) {
        double pos_scaled[3], torque[3];
        multScalVec3f(K_F, rotor_pos[i], pos_scaled);
        crossVec3f(pos_scaled, y_body, torque);  // r_i × ŷ · k_f
        F[4 + i]  = torque[0];   // Roll (X) coefficient
        F[12 + i] = torque[2];   // Pitch (Z) coefficient
    }
    
    // Invert control effectiveness matrix
    double F_inv[16];
    inv4Mat4f(F, F_inv);
    
    // Compute squared rotor speeds
    double ctrl_input[4] = {thrust, tau_ctrl[0], tau_ctrl[1], tau_ctrl[2]};
    double omega_sq[4];
    multMatVec4f(F_inv, ctrl_input, omega_sq);
    
    // Extract rotor speeds (handle negative values from numerical issues)
    for (int i = 0; i < 4; i++) {
        omega_cmd[i] = sqrt(fabs(omega_sq[i]));
    }
}

#endif // QUAD_H