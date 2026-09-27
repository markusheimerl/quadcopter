# quadcopter
A quadcopter simulation & hardware design

Consider a quadcopter modeled as a rigid body with state $(\mathbf{p}, \mathbf{v}, R, \boldsymbol{\omega})$, where $\mathbf{p}, \mathbf{v} \in \mathbb{R}^3$ are the position and linear velocity in the world frame $W$, $R \in SO(3)$ is the body-to-world rotation, and $\boldsymbol{\omega} \in \mathbb{R}^3$ is the angular velocity in the body frame $B$. The world frame is right-handed with $\mathbf{e}_y$ pointing up, and the body frame is defined with $\mathbf{x}_B$ forward, $\mathbf{y}_B$ along the thrust axis, and $\mathbf{z}_B$ to the right. The four rotors sit in an X-configuration at body positions $(x_i, 0, z_i) = (\pm a, 0, \pm a)$, with alternating CCW/CW spin directions $s_i \in \{+1, -1\}$. Each rotor produces a thrust $f_i = k_F \omega_i^2$ along $+\mathbf{y}_B$ and an aerodynamic drag torque $m_i = k_M \omega_i^2$ about $\mathbf{y}_B$. The total body-frame thrust magnitude $T$ and torque $\boldsymbol{\tau} \in \mathbb{R}^3$ are:

$$
\begin{align*}
T &= \sum_{i=0}^{3} k_F \omega_i^2 \\
\tau_x &= \sum_{i=0}^{3} -z_i \, k_F \omega_i^2 \\
\tau_y &= \sum_{i=0}^{3} s_i \, k_M \omega_i^2 \\
\tau_z &= \sum_{i=0}^{3} x_i \, k_F \omega_i^2
\end{align*}
$$

The rigid-body equations of motion are Newton's second law in the world frame and Euler's equation in the body frame. With mass $m$, gravitational acceleration $g$, and diagonal inertia $I = \mathrm{diag}(I_{xx}, I_{yy}, I_{zz})$:

$$
\begin{align*}
\dot{\mathbf{p}} &= \mathbf{v} \\
\dot{\mathbf{v}} &= \tfrac{T}{m} R \mathbf{e}_y - g \mathbf{e}_y \\
\dot{R} &= R [\boldsymbol{\omega}]_\times \\
I \dot{\boldsymbol{\omega}} &= \boldsymbol{\tau} - \boldsymbol{\omega} \times I \boldsymbol{\omega}
\end{align*}
$$

Here $[\boldsymbol{\omega}]_\times$ is the skew-symmetric matrix associated with $\boldsymbol{\omega}$. The simulator integrates these equations at $1\,\mathrm{kHz}$ using a semi-implicit (symplectic) Euler scheme: velocities are advanced first, then positions and attitude are updated with the new velocities. The attitude update uses the exact exponential map on $SO(3)$ with the midpoint angular velocity, evaluated via Rodrigues' formula:

$$
\begin{align*}
\boldsymbol{\phi} &= \tfrac{1}{2} \Delta t (\boldsymbol{\omega}_k + \boldsymbol{\omega}_{k+1}) \\
\mathbf{v}_{k+1} &= \mathbf{v}_k + \Delta t \left( \tfrac{T}{m} R_k \mathbf{e}_y - g \mathbf{e}_y \right) \\
\boldsymbol{\omega}_{k+1} &= \boldsymbol{\omega}_k + \Delta t \, I^{-1} \left( \boldsymbol{\tau} - \boldsymbol{\omega}_k \times I \boldsymbol{\omega}_k \right) \\
\mathbf{p}_{k+1} &= \mathbf{p}_k + \Delta t \, \mathbf{v}_{k+1} \\
R_{k+1} &= R_k \exp\!\left( [\boldsymbol{\phi}]_\times \right) \\
\exp\!\left( [\boldsymbol{\phi}]_\times \right) &= I_3 + \tfrac{\sin\|\boldsymbol{\phi}\|}{\|\boldsymbol{\phi}\|} [\boldsymbol{\phi}]_\times + \tfrac{1 - \cos\|\boldsymbol{\phi}\|}{\|\boldsymbol{\phi}\|^2} [\boldsymbol{\phi}]_\times^2
\end{align*}
$$

For small $\|\boldsymbol{\phi}\|$ the coefficients are replaced by their Taylor expansions $\sin(t)/t \approx 1 - t^2/6$ and $(1-\cos t)/t^2 \approx 1/2 - t^2/24$ to avoid the $0/0$ singularity. A rigid floor constraint clamps $p_y \ge 0$ and zeros downward velocity on contact.

The flight controller is a geometric tracking controller on $SO(3)$ running at $100\,\mathrm{Hz}$, given a target state $\mathbf{x}\_d = (\mathbf{p}\_d, \mathbf{v}\_d, \psi\_d)$. Position and velocity errors drive a desired world-frame force, from which a desired thrust direction $\mathbf{b}\_{2,d}$ is extracted. The thrust magnitude is projected onto the current body-Y axis $\mathbf{b}\_2 = R \mathbf{e}\_y$ to decouple thrust from tilt error:

$$
\begin{align*}
\mathbf{e}_p &= \mathbf{p} - \mathbf{p}_d \\
\mathbf{e}_v &= \mathbf{v} - \mathbf{v}_d \\
\mathbf{F} &= -K_p \mathbf{e}_p - K_v \mathbf{e}_v + m g \mathbf{e}_y \\
\mathbf{b}_{2,d} &= \frac{\mathbf{F}}{\|\mathbf{F}\|} \\
T &= \max\!\left(0,\; \mathbf{F} \cdot \mathbf{b}_2 \right)
\end{align*}
$$

A heading reference $\mathbf{c} = (\cos\psi_d,\, 0,\, -\sin\psi_d)$ defines the desired yaw, and the remaining body axes are obtained by orthogonalization:

$$
\begin{align*}
\mathbf{b}_{3,d} &= \frac{\mathbf{c} \times \mathbf{b}_{2,d}}{\|\mathbf{c} \times \mathbf{b}_{2,d}\|} \\
\mathbf{b}_{1,d} &= \mathbf{b}_{2,d} \times \mathbf{b}_{3,d} \\
R_d &= [\mathbf{b}_{1,d} \;\; \mathbf{b}_{2,d} \;\; \mathbf{b}_{3,d}]
\end{align*}
$$

The attitude and angular-velocity errors in the body frame, together with the feedback-linearized control torque, follow:

$$
\begin{align*}
\mathbf{e}_R &= \tfrac{1}{2} \left( R_d^T R - R^T R_d \right)^{\vee} \\
\mathbf{e}_\omega &= \boldsymbol{\omega} \\
\boldsymbol{\tau} &= -K_R \mathbf{e}_R - K_\omega \mathbf{e}_\omega + \boldsymbol{\omega} \times I \boldsymbol{\omega}
\end{align*}
$$

where $(\cdot)^{\vee}$ is the inverse of $[\cdot]_\times$, extracting the axial vector from a skew-symmetric matrix. Finally, the desired thrust and torque are mapped to squared rotor speeds by inverting the linear X-configuration mixer $[T, \tau_x, \tau_y, \tau_z]^T = M [\omega_0^2, \omega_1^2, \omega_2^2, \omega_3^2]^T$:

$$
\begin{align*}
\omega_0^2 &= \tfrac{T}{4 k_F} - \tfrac{\tau_x}{4 a k_F} + \tfrac{\tau_y}{4 k_M} + \tfrac{\tau_z}{4 a k_F} \\
\omega_1^2 &= \tfrac{T}{4 k_F} - \tfrac{\tau_x}{4 a k_F} - \tfrac{\tau_y}{4 k_M} - \tfrac{\tau_z}{4 a k_F} \\
\omega_2^2 &= \tfrac{T}{4 k_F} + \tfrac{\tau_x}{4 a k_F} + \tfrac{\tau_y}{4 k_M} - \tfrac{\tau_z}{4 a k_F} \\
\omega_3^2 &= \tfrac{T}{4 k_F} + \tfrac{\tau_x}{4 a k_F} - \tfrac{\tau_y}{4 k_M} + \tfrac{\tau_z}{4 a k_F} \\
\omega_i &= \mathrm{clip}\!\left( \sqrt{\max(0, \omega_i^2)},\; \omega_{\min},\; \omega_{\max} \right)
\end{align*}
$$

The commanded rotor speeds are clamped to $[\omega_{\min}, \omega_{\max}]$ before being applied to the dynamics. The implementation runs physics at $1\,\mathrm{kHz}$, control at $100\,\mathrm{Hz}$, and renders frames through a CPU raytracer at $24\,\mathrm{fps}$.

## Hardware

The flight controller / frame PCB lives in [pcb/](pcb/) as a [tscircuit](https://tscircuit.com) design with a JLCPCB-ready build (`npm run fab`). Firmware for it is in [firmware/](firmware/). Older EasyEDA designs are kept in [hardware/](hardware/).

## How to run
### Ubuntu
```bash
sudo apt update
sudo apt install -y clang make git
git clone https://github.com/markusheimerl/quadcopter && cd quadcopter/
make run -j 6
```
