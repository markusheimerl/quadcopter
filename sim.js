// 3x3 Matrix Operations
function multMat3f(a, b) {
    const m = [];
    for(let i = 0; i < 3; i++)
        for(let j = 0; j < 3; j++)
            m[i*3 + j] = a[i*3]*b[j] + a[i*3+1]*b[j+3] + a[i*3+2]*b[j+6];
    return m;
}

function multMatVec3f(m, v) {
    return [
        m[0]*v[0] + m[1]*v[1] + m[2]*v[2],
        m[3]*v[0] + m[4]*v[1] + m[5]*v[2],
        m[6]*v[0] + m[7]*v[1] + m[8]*v[2]
    ];
}

function vecToDiagMat3f(v) {
    return [v[0],0,0, 0,v[1],0, 0,0,v[2]];
}

function invMat3f(m) {
    const det = m[0]*(m[4]*m[8] - m[7]*m[5]) - 
                m[1]*(m[3]*m[8] - m[5]*m[6]) + 
                m[2]*(m[3]*m[7] - m[4]*m[6]);
    
    if (det === 0) throw new Error("Matrix is not invertible");
    
    const invDet = 1/det;
    return [
        invDet*(m[4]*m[8] - m[7]*m[5]), invDet*(m[2]*m[7] - m[1]*m[8]), invDet*(m[1]*m[5] - m[2]*m[4]),
        invDet*(m[5]*m[6] - m[3]*m[8]), invDet*(m[0]*m[8] - m[2]*m[6]), invDet*(m[3]*m[2] - m[0]*m[5]),
        invDet*(m[3]*m[7] - m[6]*m[4]), invDet*(m[6]*m[1] - m[0]*m[7]), invDet*(m[0]*m[4] - m[3]*m[1])
    ];
}

function transpMat3f(m) {
    return [m[0],m[3],m[6], m[1],m[4],m[7], m[2],m[5],m[8]];
}

function identMat3f() {
    return [1,0,0, 0,1,0, 0,0,1];
}

function rotMat3f(axis, rads) {
    const s = Math.sin(rads), c = Math.cos(rads);
    switch(axis) {
        case 'x': return [1,0,0, 0,c,-s, 0,s,c];
        case 'y': return [c,0,s, 0,1,0, -s,0,c];
        case 'z': return [c,-s,0, s,c,0, 0,0,1];
    }
}

const xRotMat3f = rads => rotMat3f('x', rads);
const yRotMat3f = rads => rotMat3f('y', rads);
const zRotMat3f = rads => rotMat3f('z', rads);

function so3hat(v) {
    return [0,-v[2],v[1], v[2],0,-v[0], -v[1],v[0],0];
}

function so3vee(m) {
    return [m[7], m[2], m[3]];
}

// Matrix arithmetic
const addMat3f = (a, b) => a.map((v, i) => v + b[i]);
const subMat3f = (a, b) => a.map((v, i) => v - b[i]);
const multScalMat3f = (s, m) => m.map(v => v * s);

// 4x4 Matrix Operations
function multMat4f(a, b) {
    const m = [];
    for(let i = 0; i < 4; i++)
        for(let j = 0; j < 4; j++)
            m[i*4 + j] = a[i*4]*b[j] + a[i*4+1]*b[j+4] + 
                         a[i*4+2]*b[j+8] + a[i*4+3]*b[j+12];
    return m;
}

// Vector Operations
const crossVec3f = (a, b) => [
    a[1]*b[2] - a[2]*b[1],
    a[2]*b[0] - a[0]*b[2],
    a[0]*b[1] - a[1]*b[0]
];

const multScalVec3f = (s, v) => v.map(x => x * s);
const addVec3f = (a, b) => a.map((v, i) => v + b[i]);
const subVec3f = (a, b) => a.map((v, i) => v - b[i]);
const dotVec3f = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);

function normVec3f(v) {
    const mag = Math.sqrt(dotVec3f(v, v));
    return v.map(x => x/mag);
}

function inv4Mat4f(m) {
    const s0 = m[0]*m[5] - m[4]*m[1];
    const s1 = m[0]*m[6] - m[4]*m[2];
    const s2 = m[0]*m[7] - m[4]*m[3];
    const s3 = m[1]*m[6] - m[5]*m[2];
    const s4 = m[1]*m[7] - m[5]*m[3];
    const s5 = m[2]*m[7] - m[6]*m[3];

    const c5 = m[10]*m[15] - m[14]*m[11];
    const c4 = m[9]*m[15] - m[13]*m[11];
    const c3 = m[9]*m[14] - m[13]*m[10];
    const c2 = m[8]*m[15] - m[12]*m[11];
    const c1 = m[8]*m[14] - m[12]*m[10];
    const c0 = m[8]*m[13] - m[12]*m[9];

    const det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0;
    
    if (det === 0) throw new Error("Matrix is not invertible");

    const invdet = 1/det;

    return [
        (m[5]*c5 - m[6]*c4 + m[7]*c3)*invdet,
        (-m[1]*c5 + m[2]*c4 - m[3]*c3)*invdet,
        (m[13]*s5 - m[14]*s4 + m[15]*s3)*invdet,
        (-m[9]*s5 + m[10]*s4 - m[11]*s3)*invdet,

        (-m[4]*c5 + m[6]*c2 - m[7]*c1)*invdet,
        (m[0]*c5 - m[2]*c2 + m[3]*c1)*invdet,
        (-m[12]*s5 + m[14]*s2 - m[15]*s1)*invdet,
        (m[8]*s5 - m[10]*s2 + m[11]*s1)*invdet,

        (m[4]*c4 - m[5]*c2 + m[7]*c0)*invdet,
        (-m[0]*c4 + m[1]*c2 - m[3]*c0)*invdet,
        (m[12]*s4 - m[13]*s2 + m[15]*s0)*invdet,
        (-m[8]*s4 + m[9]*s2 - m[11]*s0)*invdet,

        (-m[4]*c3 + m[5]*c1 - m[6]*c0)*invdet,
        (m[0]*c3 - m[1]*c1 + m[2]*c0)*invdet,
        (-m[12]*s3 + m[13]*s1 - m[14]*s0)*invdet,
        (m[8]*s3 - m[9]*s1 + m[10]*s0)*invdet
    ];
}

function multMatVec4f(m, v) {
    return [
        m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3]*v[3],
        m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7]*v[3],
        m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3],
        m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3]
    ];
}

// Constants
const k_f = 0.0004905, k_m = 0.00004905, L = 0.25;
const l = L / Math.sqrt(2);
const I = [0.01, 0.02, 0.01];
const g = 9.81, m = 0.5, dt = 0.01;
const omega_min = 30, omega_max = 70, omega_stable = 50;

// State variables
let [omega_1, omega_2, omega_3, omega_4] = Array(4).fill(omega_stable);
let angular_velocity_B = [0, 0, 0];
let linear_velocity_W = [0, 0, 0];
let linear_position_W = [0, 1, 0];
let R_W_B = multMat3f(multMat3f(xRotMat3f(0), yRotMat3f(0)), zRotMat3f(0));

setInterval(() => {
    // Limit motor speeds
    omega_1 = Math.max(Math.min(omega_1, omega_max), omega_min);
    omega_2 = Math.max(Math.min(omega_2, omega_max), omega_min);
    omega_3 = Math.max(Math.min(omega_3, omega_max), omega_min);
    omega_4 = Math.max(Math.min(omega_4, omega_max), omega_min);

    // Calculate individual rotor forces and moments
    let f1 = k_f * omega_1 * Math.abs(omega_1);
    let f2 = k_f * omega_2 * Math.abs(omega_2);
    let f3 = k_f * omega_3 * Math.abs(omega_3);
    let f4 = k_f * omega_4 * Math.abs(omega_4);

    let m1 = k_m * omega_1 * Math.abs(omega_1);
    let m2 = k_m * omega_2 * Math.abs(omega_2);
    let m3 = k_m * omega_3 * Math.abs(omega_3);
    let m4 = k_m * omega_4 * Math.abs(omega_4);
    
    // Total thrust force in body frame
    let f_B_thrust = [0, f1 + f2 + f3 + f4, 0];
    
    // Torques from drag and thrust
    let tau_B_drag = [0, m1 - m2 + m3 - m4, 0];
    
    // Torques from thrust forces
    let tau_1 = crossVec3f([-L, 0, L], [0, f1, 0]);
    let tau_2 = crossVec3f([L, 0, L], [0, f2, 0]);
    let tau_3 = crossVec3f([L, 0, -L], [0, f3, 0]);
    let tau_4 = crossVec3f([-L, 0, -L], [0, f4, 0]);
    
    let tau_B_thrust = [
        tau_1[0] + tau_2[0] + tau_3[0] + tau_4[0],
        tau_1[1] + tau_2[1] + tau_3[1] + tau_4[1],
        tau_1[2] + tau_2[2] + tau_3[2] + tau_4[2]
    ];

    let tau_B = [
        tau_B_drag[0] + tau_B_thrust[0],
        tau_B_drag[1] + tau_B_thrust[1],
        tau_B_drag[2] + tau_B_thrust[2]
    ];

    // Transform thrust to world frame and add gravity
    let f_thrust_W = multMatVec3f(R_W_B, f_B_thrust);
    let f_gravity_W = [0, -g * m, 0];
    
    // Calculate accelerations
    let linear_acceleration_W = [
        (f_thrust_W[0] + f_gravity_W[0]) / m,
        (f_thrust_W[1] + f_gravity_W[1]) / m,
        (f_thrust_W[2] + f_gravity_W[2]) / m
    ];

    // Angular momentum terms
    let h_B = multMatVec3f(vecToDiagMat3f(I), angular_velocity_B);
    let w_cross_h = crossVec3f(multScalVec3f(-1, angular_velocity_B), h_B);
    
    let angular_acceleration_B = [
        (w_cross_h[0] + tau_B[0]) / I[0],
        (w_cross_h[1] + tau_B[1]) / I[1],
        (w_cross_h[2] + tau_B[2]) / I[2]
    ];

    // Update state variables
    linear_velocity_W = [
        linear_velocity_W[0] + dt * linear_acceleration_W[0],
        linear_velocity_W[1] + dt * linear_acceleration_W[1],
        linear_velocity_W[2] + dt * linear_acceleration_W[2]
    ];

    linear_position_W = [
        linear_position_W[0] + dt * linear_velocity_W[0],
        linear_position_W[1] + dt * linear_velocity_W[1],
        linear_position_W[2] + dt * linear_velocity_W[2]
    ];

    angular_velocity_B = [
        angular_velocity_B[0] + dt * angular_acceleration_B[0],
        angular_velocity_B[1] + dt * angular_acceleration_B[1],
        angular_velocity_B[2] + dt * angular_acceleration_B[2]
    ];

    // Update rotation matrix
    let w_hat = so3hat(angular_velocity_B);
    let R_dot = multMat3f(R_W_B, w_hat);
    R_W_B = addMat3f(R_W_B, multScalMat3f(dt, R_dot));
}, dt);

// ----------------------------------- CONTROL PARAMETERS -----------------------------------
let linear_position_d_W = [2, 2, 2];
let linear_velocity_d_W = [0, 0, 0];
let linear_acceleration_d_W = [0, 0, 0];
let angular_velocity_d_B = [0, 0, 0];
let angular_acceleration_d_B = [0, 0, 0];
let yaw_d = 0.0;

const k_p = 0.05;
const k_v = 0.5;
const k_R = 0.5;
const k_w = 0.5;

// ----------------------------------- CONTROL LOOP -----------------------------------
setInterval(function () {
    // --- LINEAR CONTROL ---
    let error_p = subVec3f(linear_position_W, linear_position_d_W);
    let error_v = subVec3f(linear_velocity_W, linear_velocity_d_W);

    let z_W_d = multScalVec3f(-k_p, error_p);
    z_W_d = addVec3f(z_W_d, multScalVec3f(-k_v, error_v));
    z_W_d = addVec3f(z_W_d, [0, m * g, 0]);
    z_W_d = addVec3f(z_W_d, multScalVec3f(m, linear_acceleration_d_W));
    let z_W_B = multMatVec3f(R_W_B, [0, 1, 0]);
    let f_z_B_control = dotVec3f(z_W_d, z_W_B);

    // --- ATTITIDUE CONTROL ---
    let x_tilde_d_W = [Math.sin(yaw_d), 0, Math.cos(yaw_d)];
    let R_W_d_column_0 = normVec3f(crossVec3f(crossVec3f(z_W_d, x_tilde_d_W), z_W_d));
    let R_W_d_column_1 = normVec3f(crossVec3f(z_W_d, x_tilde_d_W));
    let R_W_d_column_2 = normVec3f(z_W_d);
    let R_W_d = [
        R_W_d_column_1[0], R_W_d_column_2[0], R_W_d_column_0[0],
        R_W_d_column_1[1], R_W_d_column_2[1], R_W_d_column_0[1],
        R_W_d_column_1[2], R_W_d_column_2[2], R_W_d_column_0[2]
    ];

    let error_r = multScalVec3f(0.5, so3vee(subMat3f(multMat3f(transpMat3f(R_W_d), R_W_B), multMat3f(transpMat3f(R_W_B), R_W_d))));
    let error_w = subVec3f(angular_velocity_B, multMatVec3f(multMat3f(transpMat3f(R_W_d), R_W_B), angular_velocity_d_B));

    let tau_B_control = multScalVec3f(-k_R, error_r);
    tau_B_control = addVec3f(tau_B_control, multScalVec3f(-k_w, error_w));
    tau_B_control = addVec3f(tau_B_control, crossVec3f(angular_velocity_B, multMatVec3f(vecToDiagMat3f(I), angular_velocity_B)));
    let term_0 = multMatVec3f(transpMat3f(R_W_B), multMatVec3f(R_W_d, angular_acceleration_d_B));
    let term_1 = crossVec3f(angular_velocity_B, multMatVec3f(transpMat3f(R_W_B), multMatVec3f(R_W_d, angular_velocity_d_B)));
    tau_B_control = subVec3f(tau_B_control, multMatVec3f(vecToDiagMat3f(I), subVec3f(term_1, term_0)));

    // --- ROTOR SPEEDS ---
    let F_bar_column_0 = addVec3f([0, k_m, 0], crossVec3f(multScalVec3f(k_f, [-L, 0, L]), [0, 1, 0]));
    let F_bar_column_1 = addVec3f([0, -k_m, 0], crossVec3f(multScalVec3f(k_f, [L, 0, L]), [0, 1, 0]));
    let F_bar_column_2 = addVec3f([0, k_m, 0], crossVec3f(multScalVec3f(k_f, [L, 0, -L]), [0, 1, 0]));
    let F_bar_column_3 = addVec3f([0, -k_m, 0], crossVec3f(multScalVec3f(k_f, [-L, 0, -L]), [0, 1, 0]));
    let F_bar = [
        k_f, k_f, k_f, k_f,
        F_bar_column_0[0], F_bar_column_1[0], F_bar_column_2[0], F_bar_column_3[0],
        F_bar_column_0[1], F_bar_column_1[1], F_bar_column_2[1], F_bar_column_3[1],
        F_bar_column_0[2], F_bar_column_1[2], F_bar_column_2[2], F_bar_column_3[2]
    ];
    let F_bar_inv = inv4Mat4f(F_bar);
    let omega_sign_square = multMatVec4f(F_bar_inv, [f_z_B_control, tau_B_control[0], tau_B_control[1], tau_B_control[2]]);

    omega_1 = Math.sqrt(Math.abs(omega_sign_square[0]));
    omega_2 = Math.sqrt(Math.abs(omega_sign_square[1]));
    omega_3 = Math.sqrt(Math.abs(omega_sign_square[2]));
    omega_4 = Math.sqrt(Math.abs(omega_sign_square[3]));

}, dt * 10);

// Print drone state every 100ms
setInterval(() => {
    console.log(`Time: ${new Date().toISOString()}
Linear Position: ${linear_position_W}
Linear Velocity: ${linear_velocity_W}
Angular Velocity: ${angular_velocity_B}
Rotor Speeds: ${omega_1}, ${omega_2}, ${omega_3}, ${omega_4}
`);
}, 100);

// Check position and stop conditions
setInterval(() => {
    const isAtTarget = linear_position_W.every((pos, i) => 
        Math.abs(pos - linear_position_d_W[i]) < 0.1
    );
    if (isAtTarget) process.exit();
}, dt * 1000);

// Timeout after 10 seconds
setTimeout(() => process.exit(), 10000);