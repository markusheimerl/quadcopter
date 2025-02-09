#!/usr/bin/env python3
"""
A "perfectly-correct" sympy implementation of the quadrotor dynamics
that mirrors the C code in quad.h. In particular, rotor forces are computed as

   f_i = Kf · (ω_i²)

and rotor drag moments as

   m_i = Km · (ω_i²).

Then the net thrust is
   thrust = f₀ + f₁ + f₂ + f₃,
and the net body-torque is built by adding a direct yaw moment
   τ_yaw = m₀ - m₁ + m₂ - m₃
plus the “thrust-induced” moments via the cross-product
   r_i x (0, f_i, 0)
using rotor positions
   r₀ = ( -L, 0,  L )
   r₁ = (  L, 0,  L )
   r₂ = (  L, 0, -L )
   r₃ = ( -L, 0, -L ).
Thus the total torque τ_B becomes:
   τ_B[0] = L * ( -f₀ - f₁ + f₂ + f₃ )
   τ_B[1] = (m₀ - m₁ + m₂ - m₃)
   τ_B[2] = L * ( -f₀ + f₁ + f₂ - f₃ ).
The linear acceleration is computed from transforming the thrust (applied along the body-y axis)
to world coordinates using the rotation matrix R, and subtracting gravity.
The angular acceleration is computed from
   ω̇ = I⁻¹ · ( τ_B - ω x (Iω) ).
Finally, the state is integrated with a simple Euler step.
This script then differentiates p_next, v_next, and w_next with respect to ω (rotor speeds)
and also computes the other requested Jacobians.
"""

import sympy as sp

# Define symbols and matrices
dt, m, Kf, Km, L, g = sp.symbols('dt m Kf Km L g', real=True)
Ix, Iy, Iz = sp.symbols('Ix Iy Iz', real=True, positive=True)
# State variables:
px, py, pz = sp.symbols('px py pz', real=True)
vx, vy, vz = sp.symbols('vx vy vz', real=True)
# Angular velocity vector in body frame
wx, wy, wz = sp.symbols('wx wy wz', real=True)
# Rotor speeds (assume they are positive so that ω_i * |ω_i| = ω_i^2)
w1, w2, w3, w4 = sp.symbols('w1 w2 w3 w4', real=True)

# Create vectors and matrices
p = sp.Matrix([px, py, pz])
v = sp.Matrix([vx, vy, vz])
w_vec = sp.Matrix([wx, wy, wz])
omega_vec = sp.Matrix([w1, w2, w3, w4])

# Rotation matrix R (3x3) whose 9 elements we label R0 ... R8.
R_syms = sp.symbols('R0:9', real=True)
R = sp.Matrix(3, 3, list(R_syms))
# Note: here we assume "column-major" order matching the C code use:
#   R[0], R[1], R[2]   -> first column,
#   R[3], R[4], R[5]   -> second column,
#   R[6], R[7], R[8]   -> third column.
# (Ensure that this ordering is consistent with your C indexing.)

# Inertia matrix (diagonal)
I_mat = sp.diag(Ix, Iy, Iz)

# --- 1. Compute rotor forces and moments ---
# For each rotor i: f_i = Kf * omega_i^2, and m_i = Km * omega_i^2.
f0 = Kf * omega_vec[0]**2
f1 = Kf * omega_vec[1]**2
f2 = Kf * omega_vec[2]**2
f3 = Kf * omega_vec[3]**2
# Similarly for drag moments:
m0 = Km * omega_vec[0]**2
m1 = Km * omega_vec[1]**2
m2 = Km * omega_vec[2]**2
m3 = Km * omega_vec[3]**2

# Total thrust (acts along the body y-axis)
thrust = f0 + f1 + f2 + f3

# --- 2. Net rotor torque assembly ---
# Define rotor positions (exactly as in C):
r0 = sp.Matrix([-L, 0,  L])
r1 = sp.Matrix([ L, 0,  L])
r2 = sp.Matrix([ L, 0, -L])
r3 = sp.Matrix([-L, 0, -L])
# Define a function to compute cross product r x F, with F having only y component:
def rotor_cross(r, f):
    # f_vector = [0, f, 0]
    f_vec = sp.Matrix([0, f, 0])
    # r x f_vec = ( - r_z * f, 0, r_x * f )
    return sp.Matrix([ - r[2]*f, 0, r[0]*f ])

# Sum the cross-product contributions
tau_cross = ( rotor_cross(r0, f0) +
              rotor_cross(r1, f1) +
              rotor_cross(r2, f2) +
              rotor_cross(r3, f3) )
# Now add the direct yaw moments from drag:
tau_yaw = sp.Matrix([0, m0 - m1 + m2 - m3, 0])
# Total body torque:
tau_B = tau_cross + tau_yaw
# For clarity, you can simplify the components:
tau_B = sp.simplify(tau_B)
# It should yield:
#   tau_B[0] = L*(-f0 - f1 + f2 + f3)
#   tau_B[1] = m0 - m1 + m2 - m3
#   tau_B[2] = L*(-f0 + f1 + f2 - f3)

# --- 3. Linear dynamics ---
# In the C code the thrust force is applied as f_B_thrust = [0, thrust, 0] in body frame.
f_B_thrust = sp.Matrix([0, thrust, 0])
# Transform to world frame: f_thrust_W = R * f_B_thrust.
f_thrust_W = R * f_B_thrust
# Linear acceleration in world frame (note gravity subtracts in y direction):
a_lin = f_thrust_W/m - sp.Matrix([0, g, 0])
# Euler integration:
v_next = v + dt * a_lin
p_next = p + dt * v_next

# --- 4. Angular dynamics ---
# Compute angular momentum in body frame: h = I * w.
h_B = I_mat * w_vec
# Compute cross product ω x h:
w_cross_h = w_vec.cross(h_B)
# Angular acceleration:
w_dot = I_mat.inv() * (tau_B - w_cross_h)
w_next = w_vec + dt * w_dot

# (Optionally you could also add the rotation matrix integration,
#  but for the purpose of computing gradients with respect to ω this is sufficient.)

# --- 5. Jacobians (backward pass) ---
# We now compute the Jacobians (gradients) of interest.
# Gradient of next position with respect to rotor speeds:
dp_next_domega = sp.simplify(p_next.jacobian(omega_vec))

# Gradient of next angular velocity with respect to rotor speeds:
dw_next_domega = sp.simplify(w_next.jacobian(omega_vec))

# Additionally, gradients with respect to current state:
dp_next_dp = sp.simplify(p_next.jacobian(p))
dp_next_dv = sp.simplify(p_next.jacobian(v))
dv_next_dp = sp.simplify(v_next.jacobian(p))
dv_next_dv = sp.simplify(v_next.jacobian(v))
dw_next_dw = sp.simplify(w_next.jacobian(w_vec))

# --- 6. Print results ---
print("// Gradient of next position with respect to rotor speeds (dp_next/domega):")
#sp.pprint(dp_next_domega)
print(dp_next_domega)
print("\n// Gradient of next angular velocity with respect to rotor speeds (dw_next/domega):")
#sp.pprint(dw_next_domega)
print(dw_next_domega)
print("\n// Gradient of next position with respect to current position (dp_next/dp):")
#sp.pprint(dp_next_dp)
print(dp_next_dp)
print("\n// Gradient of next position with respect to current velocity (dp_next/dv):")
#sp.pprint(dp_next_dv)
print(dp_next_dv)
print("\n// Gradient of next velocity with respect to current position (dv_next/dp):")
#sp.pprint(dv_next_dp)
print(dv_next_dp)
print("\n// Gradient of next velocity with respect to current velocity (dv_next/dv):")
#sp.pprint(dv_next_dv)
print(dv_next_dv)
print("\n// Gradient of next angular velocity with respect to current angular velocity (dw_next/dw):")
#sp.pprint(dw_next_dw)
print(dw_next_dw)

# End of script.