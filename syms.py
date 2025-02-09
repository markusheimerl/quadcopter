import sympy as sp

# First, let's define our symbolic variables
# State variables
p = sp.Matrix(sp.symbols('px py pz'))  # position
v = sp.Matrix(sp.symbols('vx vy vz'))  # velocity
R = sp.Matrix(3, 3, sp.symbols('R:9'))  # rotation matrix
w = sp.Matrix(sp.symbols('wx wy wz'))  # angular velocity
omega = sp.Matrix(sp.symbols('w1 w2 w3 w4'))  # rotor speeds

# Constants
dt = sp.Symbol('dt')
mass = sp.Symbol('m')
Kf = sp.Symbol('Kf')
Km = sp.Symbol('Km')
L = sp.Symbol('L')
g = sp.Symbol('g')
I = sp.diag(*sp.symbols('Ix Iy Iz'))  # inertia matrix

# Simplified rotor forces and moments (assume omega is positive)
f = Kf * omega.multiply_elementwise(omega)  # Using ω² directly instead of |ω|ω
m = Km * omega.multiply_elementwise(omega)

# Net thrust
thrust = sum(f)

# Rotor positions
rotor_positions = [
    sp.Matrix([-L, 0, L]),  # Rotor 0
    sp.Matrix([L, 0, L]),   # Rotor 1
    sp.Matrix([L, 0, -L]),  # Rotor 2
    sp.Matrix([-L, 0, -L])  # Rotor 3
]

# Torques from rotors
# Simplify the torque calculation
tau_B = sp.Matrix([
    L * Kf * (omega[3]**2 + omega[0]**2 - omega[1]**2 - omega[2]**2),
    Km * (omega[0]**2 - omega[1]**2 + omega[2]**2 - omega[3]**2),
    L * Kf * (omega[1]**2 + omega[0]**2 - omega[2]**2 - omega[3]**2)
])

# Linear dynamics
f_B_thrust = sp.Matrix([0, thrust, 0])
f_thrust_W = R * f_B_thrust

linear_acc = f_thrust_W/mass - sp.Matrix([0, g, 0])

# State evolution equations
v_next = v + dt * linear_acc
p_next = p + dt * v_next

# Angular dynamics
w_hat = sp.Matrix([
    [0, -w[2], w[1]],
    [w[2], 0, -w[0]],
    [-w[1], w[0], 0]
])

h_B = I * w
w_cross_h = w.cross(h_B)
w_dot = I.inv() * (tau_B - w_cross_h)
w_next = w + dt * w_dot

# Now compute gradients
# Gradient of p_next with respect to omega:
dp_next_domega = p_next.jacobian(omega)

# Gradient of w_next with respect to omega:
dw_next_domega = w_next.jacobian(omega)

# Other important gradients
dp_next_dp = p_next.jacobian(p)
dp_next_dv = p_next.jacobian(v)
dv_next_dp = v_next.jacobian(p)
dv_next_dv = v_next.jacobian(v)
dw_next_dw = w_next.jacobian(w)

# Simplify expressions
dp_next_domega = sp.simplify(dp_next_domega)
dw_next_domega = sp.simplify(dw_next_domega)
dp_next_dp = sp.simplify(dp_next_dp)
dp_next_dv = sp.simplify(dp_next_dv)
dv_next_dp = sp.simplify(dv_next_dp)
dv_next_dv = sp.simplify(dv_next_dv)
dw_next_dw = sp.simplify(dw_next_dw)

print("// Gradient of position with respect to rotor speeds:")
sp.pprint(dp_next_domega)

print("\n// Gradient of angular velocity with respect to rotor speeds:")
sp.pprint(dw_next_domega)

print("\n// Gradient of next position with respect to current position:")
sp.pprint(dp_next_dp)

print("\n// Gradient of next position with respect to current velocity:")
sp.pprint(dp_next_dv)

print("\n// Gradient of next velocity with respect to current position:")
sp.pprint(dv_next_dp)

print("\n// Gradient of next velocity with respect to current velocity:")
sp.pprint(dv_next_dv)

print("\n// Gradient of next angular velocity with respect to current angular velocity:")
sp.pprint(dw_next_dw)