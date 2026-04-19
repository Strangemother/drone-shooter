# Flight Dynamics — Equations Reference

> **Audience:** engineers, contributors, and AI agents implementing flight
> simulation components in this project.
>
> **Goal:** a self-contained mathematical reference covering the physics of
> rotary-wing and fixed-wing flight as applied in this simulator.  Every
> equation is expressed in SI units and Godot's coordinate conventions.
> Use this as the starting point when computing component parameters, checking
> physical plausibility, or designing new controller logic.
>
> **Convention:** Godot uses a right-handed coordinate system with $+Y$ up,
> $-Z$ forward, $+X$ right.  Rotations follow the right-hand rule.  All vectors
> are in the body frame unless otherwise stated.  See
> [flight-controller-guide.md §3](flight-controller-guide.md) for a full axis
> table.
>
> **Notation**

| Symbol | Meaning | SI unit |
|---|---|---|
| $m$ | Mass | kg |
| $g$ | Gravitational acceleration (9.8) | m/s² |
| $\mathbf{F}$ | Force vector | N |
| $\mathbf{T}$ | Thrust vector | N |
| $\boldsymbol{\tau}$ | Torque vector | N·m |
| $I$ | Moment of inertia (scalar about yaw axis) | kg·m² |
| $\boldsymbol{\omega}$ | Angular velocity | rad/s |
| $\mathbf{v}$ | Velocity | m/s |
| $\rho$ | Air density (sea level ≈ 1.225) | kg/m³ |
| $A$ | Reference area (planform) | m² |
| $C_D$ | Drag coefficient (dimensionless) | — |
| $C_L$ | Lift coefficient (dimensionless) | — |
| $r$ | Lever arm length from body centre | m |
| $R$ | Propeller radius | m |
| $n$ | Rotational speed | rev/s |
| $k_Q$ | Proportionality coefficient (torque per unit force) | m |
| $\Delta t$ | Physics time step | s |

---

## 1. Newtonian foundation

### 1.1 Newton's second law — translation

The net force on the body equals its mass times acceleration:

$$\mathbf{F}_\text{net} = m \, \mathbf{a} = m \, \dot{\mathbf{v}}$$

In the simulator, every `apply_force` call accumulates into $\mathbf{F}_\text{net}$
for the current physics step.  Godot integrates this automatically.

### 1.2 Newton's second law — rotation

$$\boldsymbol{\tau}_\text{net} = I \, \boldsymbol{\alpha} = I \, \dot{\boldsymbol{\omega}}$$

Every `apply_torque` call accumulates into $\boldsymbol{\tau}_\text{net}$.

### 1.3 Equations of motion

Over one time step $\Delta t$:

$$\mathbf{v}_{t+1} = \mathbf{v}_t + \frac{\mathbf{F}_\text{net}}{m} \, \Delta t$$

$$\mathbf{x}_{t+1} = \mathbf{x}_t + \mathbf{v}_{t+1} \, \Delta t$$

$$\boldsymbol{\omega}_{t+1} = \boldsymbol{\omega}_t + \frac{\boldsymbol{\tau}_\text{net}}{I} \, \Delta t$$

Godot's `RigidBody3D` uses a semi-implicit Euler integrator, which matches
these equations.

---

## 2. Gravity

### 2.1 Gravitational force

$$\mathbf{F}_g = m \, \mathbf{g} = m \begin{pmatrix} 0 \\ -9.8 \\ 0 \end{pmatrix} \text{ N}$$

This force is applied automatically by Godot to every `RigidBody3D` unless
`gravity_scale = 0`.

### 2.2 Minimum thrust to hover

For a hover, vertical thrust must exactly cancel gravity:

$$F_\text{hover} = m \, g$$

With $N$ thrusters each at throttle $\theta$:

$$N \, F_\text{max} \, \theta_\text{hover} = m \, g \implies \theta_\text{hover} = \frac{m \, g}{N \, F_\text{max}}$$

At a target TWR of 2.0, hover occurs at $\theta_\text{hover} = 0.5$ (50%
throttle).

### 2.3 Thrust-to-weight ratio

$$\text{TWR} = \frac{N \, F_\text{max}}{m \, g}$$

| TWR | Meaning |
|---|---|
| < 1.0 | Cannot lift off |
| 1.0 | Full throttle holds altitude; cannot climb |
| 2.0 | Hover at 50% throttle; consumer drone feel |
| 4.0 | Hover at 25%; FPV sport feel |
| 8 – 12 | Racing; nearly vertical climb possible |

---

## 3. Thrust vectors

### 3.1 Single thruster

A thruster at world position $\mathbf{p}$ with local force axis $\hat{\mathbf{a}}$
(expressed in the *body* frame) applies:

$$\mathbf{F}_\text{world} = \mathbf{R}_\text{body} \, \hat{\mathbf{a}} \, F_\text{max} \, \theta$$

where $\mathbf{R}_\text{body}$ is the body's rotation matrix (i.e.
`global_transform.basis`).  `FlightThruster._physics_process` performs this
multiplication implicitly via `global_transform.basis * normalized_axis`.

### 3.2 Force application point and induced torque

When a force $\mathbf{F}$ is applied at offset $\mathbf{r}$ from the centre of
mass, it produces an additional torque:

$$\boldsymbol{\tau}_\text{arm} = \mathbf{r} \times \mathbf{F}$$

This is the origin of pitch/roll coupling in an asymmetric thruster layout and
the desired effect in a differential-thrust pitch/roll controller.

For a symmetric quad, all $\boldsymbol{\tau}_\text{arm}$ contributions about
the vertical axis cancel at equal throttle.  The net is non-zero only during
pitch or roll manoeuvres.

### 3.3 Collective thrust (quadcopter)

For $N$ symmetric thrusters all pointing along the body $+Y$ axis at equal
throttle $\theta$:

$$\mathbf{T}_\text{total} = N \, F_\text{max} \, \theta \, \hat{Y}_\text{world}$$

Net vertical acceleration:

$$a_y = \frac{T_\text{total}}{m} - g = \left(\frac{N \, F_\text{max} \, \theta}{m} - g\right)$$

---

## 4. Aerodynamic drag

### 4.1 Velocity-squared drag

Real aerodynamic drag scales with the square of airspeed:

$$\mathbf{F}_\text{drag} = -\frac{1}{2} \rho \, C_D \, A \, v^2 \, \hat{\mathbf{v}}$$

where $\hat{\mathbf{v}}$ is the unit velocity direction (opposing drag).

**In the simulator:** this can be approximated by Godot's built-in
`linear_damp`, which applies a linear drag $\mathbf{F}_\text{drag} = -d \, m
\, \mathbf{v}$.  Linear drag is a simplification but is numerically stable
and produces a natural terminal-velocity feel without requiring explicit force
computation.

The linear-equivalent damping coefficient for a target terminal velocity
$v_\text{term}$ at full thrust $T$:

$$d = \frac{T}{m \, v_\text{term}}$$

Example: 0.5 kg drone, 9.8 N total thrust, target terminal horizontal speed
20 m/s:

$$d = \frac{9.8}{0.5 \times 20} = 0.98 \approx 1.0$$

### 4.2 Angular drag

Rotational drag from aerodynamic resistance of the spinning body is modelled
by Godot's `angular_damp`:

$$\boldsymbol{\tau}_{\text{angular drag}} = -d_\omega \, I \, \boldsymbol{\omega}$$

This causes exponential decay:

$$\boldsymbol{\omega}(t) = \boldsymbol{\omega}_0 \, e^{-d_\omega \, t}$$

---

## 5. Reactive torque (propeller yaw couple)

### 5.1 Aerodynamic drag torque from a spinning prop

A propeller spinning at angular velocity $\Omega$ (rad/s) produces both thrust
$T$ and a reaction torque $Q$ on the motor shaft:

$$Q = k_Q \, T$$

where $k_Q$ is the motor's torque-to-thrust ratio (approximately 0.01–0.05 m
for typical drone props).  The body receives $-Q$ about the prop's spin axis.

### 5.2 Net yaw torque from a quadcopter

For a layout with $N_\text{CW}$ clockwise and $N_\text{CCW}$ counter-clockwise
motors:

$$\tau_\text{yaw} = k_Q \left( \sum_{i \in \text{CCW}} T_i - \sum_{j \in \text{CW}} T_j \right)$$

At equal throttle this sum is zero.  During a yaw manoeuvre, the differential
$\Delta T = T_\text{CCW} - T_\text{CW}$ produces:

$$\tau_\text{yaw} = k_Q \, \Delta T \, N_\text{pairs}$$

**In the simulator:** `apply_torque(body.basis.y * -yaw * yaw_reaction_torque)`
encodes $k_Q \, \Delta T \, N_\text{pairs}$ as the single tunable scalar
`yaw_reaction_torque`.  It is applied in the body's local $Y$ direction, so it
correctly rotates the drone about its own up axis regardless of attitude.

### 5.3 Steady-state yaw rate

At steady state, input torque equals damping torque:

$$\tau_\text{yaw} = d_\omega \, I \, \omega_\text{yaw}$$

$$\omega_\text{yaw} = \frac{\tau_\text{yaw}}{d_\omega \, I}$$

This gives the formula used in [units-reference.md §5.3](units-reference.md)
to solve for `yaw_reaction_torque` from a target yaw rate.

---

## 6. Moment of inertia

### 6.1 Uniform sphere (Godot default)

When no collision shape is assigned, Godot derives $I$ as a uniform solid
sphere:

$$I_\text{sphere} = \frac{2}{5} m r^2$$

For a 0.5 kg drone with effective radius 0.15 m:

$$I \approx \frac{2}{5} \times 0.5 \times 0.15^2 = 0.0045~\mathrm{kg \cdot m^2}$$

### 6.2 Thin rod / arm (more realistic for a quad frame)

The moment of inertia of a quad frame about the yaw axis ($+Y$) is dominated
by the arm mass at distance $r$ from centre:

$$I_\text{arm} = \sum_i m_i r_i^2$$

For four equal arms of mass $m_\text{arm}$ at radius $r$:

$$I_\text{frame} = 4 \, m_\text{arm} \, r^2$$

### 6.3 Parallel axis theorem

To shift an inertia from the centroid to an offset point at distance $d$:

$$I_\text{offset} = I_\text{cm} + m \, d^2$$

Useful when motors have non-negligible mass and are offset from the body
centre.

---

## 7. Centre of mass and centroid

### 7.1 Composite body centre of mass

For a drone body with $n$ components each of mass $m_i$ at position
$\mathbf{x}_i$:

$$\mathbf{x}_\text{cm} = \frac{\sum_i m_i \, \mathbf{x}_i}{\sum_i m_i}$$

If the centre of mass does not coincide with the `RigidBody3D` origin, all
four motors apply asymmetric lever arms, causing pitch/roll coupling under
throttle.

### 7.2 Practical check in Godot

Godot always treats the `RigidBody3D` origin as the centre of mass (it does
not support offset CoM natively in 4.x without code).  To verify symmetry:

- Sum the motor positions: $\sum \mathbf{p}_i$ should equal $\mathbf{0}$
- Ensure motor forces are equal at hover throttle

A non-zero sum indicates a CoM offset and will cause a pitch/roll tilt under
collective throttle.

---

## 8. Ground effect and prop wash

### 8.1 Ground effect — lift augmentation near a surface

When a rotor operates within one rotor diameter of the ground, the airflow
below the prop is partially blocked, creating a cushion of elevated pressure
that augments lift.  The thrust increase is approximately:

$$T_\text{IGE} = T_\text{OGE} \left(1 + \frac{k_\text{ge}}{(h / R)^2}\right)^{-1}$$

where:
- $T_\text{IGE}$ = thrust in ground effect
- $T_\text{OGE}$ = thrust out of ground effect (free air)
- $h$ = height above ground
- $R$ = rotor radius
- $k_\text{ge} \approx 0.25$ (empirical constant, varies by rotor design)

Practical result: near the ground ($h < R$) the drone requires *less* throttle
to hover.  Pilots experience this as the drone "floating" near landing.

**Implementation note:** not yet modelled in `FlightThruster`.  To add it,
cast a ray downward from each thruster and modulate `max_force` by the formula
above when the ray distance is less than `2R`.

### 8.2 Fountain effect (multi-rotor interaction)

On multi-rotor craft, downwash from adjacent propellers meets below the
frame.  The colliding jets are redirected upward in a "fountain" between the
rotors, then curve back down through the props.  Net effect:

- Reduces effective thrust by **5 – 15%** compared to isolated rotors
- Increases vibration and turbulence at the frame
- Scales with how close the propellers are spaced

Approximation for a quadcopter (empirical):

$$T_\text{effective} \approx T_\text{isolated} \times (1 - \epsilon_\text{fountain})$$

where $\epsilon_\text{fountain} \approx 0.05 – 0.15$ depending on prop spacing
relative to diameter.  For drones with large prop-to-frame-distance ratios
this effect is small and can be absorbed into a fixed `max_force` reduction.

### 8.3 Prop wash

Downwash velocity directly below a hovering rotor:

$$v_\text{wash} = \sqrt{\frac{T}{2 \, \rho \, A_\text{disk}}}$$

where $A_\text{disk} = \pi R^2$ is the prop disk area.

For a 5-inch prop ($R \approx 0.064$ m) at 15 N thrust:

$$v_\text{wash} = \sqrt{\frac{15}{2 \times 1.225 \times \pi \times 0.064^2}} \approx 24~\text{m/s}$$

This is the air velocity below the drone — relevant for dust, particle
effects, and prop wash on nearby objects.

---

## 9. Lift (fixed-wing and gyroplane)

For completeness, the thin-aerofoil lift equation used by fixed-wing
controllers:

$$L = \frac{1}{2} \rho \, v^2 \, C_L \, A$$

where $A$ is the wing planform area and $C_L$ depends on angle of attack $\alpha$:

$$C_L \approx 2 \pi \sin(\alpha) \approx 2 \pi \alpha \quad \text{(small angles, thin aerofoil)}$$

The corresponding induced drag:

$$D_\text{induced} = \frac{L^2}{\frac{1}{2} \rho v^2 \pi A R_\text{aspect}}$$

where $R_\text{aspect} = b^2 / A$ is the aspect ratio ($b$ = wingspan).

---

## 10. Pitch and roll mixing (differential thrust)

### 10.1 Torque from a pair of thrusters

Two thrusters at equal but opposite offsets $\pm r$ from the body centre,
each producing thrust $T_1$ and $T_2$:

$$\tau_\text{axis} = (T_1 - T_2) \, r$$

This is the fundamental equation behind pitch and roll control in a
quadcopter.  To pitch nose-down, reduce front-motor thrust and increase
rear-motor thrust by the same amount.

### 10.2 Quadcopter full mixing matrix

For an X-quad with motors at $(\pm r, 0, \pm r)$:

$$\begin{pmatrix} T_\text{FL} \\ T_\text{FR} \\ T_\text{RR} \\ T_\text{RL} \end{pmatrix}
= \begin{pmatrix}
1 & -1 & -1 & +1 \\
1 & +1 & -1 & -1 \\
1 & +1 & +1 & +1 \\
1 & -1 & +1 & -1
\end{pmatrix}
\begin{pmatrix}
\text{throttle} \\ \text{roll} \\ \text{pitch} \\ \text{yaw}
\end{pmatrix}
\times \frac{F_\text{max}}{4}$$

Each output is clamped to $[0, F_\text{max}]$.  This matrix is the target
for a fully mixed stabilised controller; the current `FlightQuadController`
handles yaw only — pitch and roll mixing come from the attitude/PID layer
(see [flight-controller-guide.md](flight-controller-guide.md)).

---

## 11. PID control (stabilisation)

### 11.1 PID formula

The standard discrete PID with derivative on measurement (avoids derivative
kick on setpoint change):

$$u(t) = K_P \, e(t) + K_I \sum_t e(t) \, \Delta t - K_D \frac{e(t) - e(t-1)}{\Delta t}$$

where:
- $u(t)$ = controller output
- $e(t) = \theta_\text{target} - \theta_\text{measured}$ = error
- $K_P, K_I, K_D$ = proportional, integral, derivative gains

### 11.2 Integral windup guard

When the controller output saturates (motor at min/max), the integral
accumulates error it cannot act on.  Guard against this by clamping the
integral state:

$$\int e \leftarrow \text{clamp}\left(\int e,\; -I_\text{max},\; +I_\text{max}\right)$$

Typically $I_\text{max} \approx 1.0$ (same range as the output [−1, 1]).

---

## 12. Euler angles and attitude representation

### 12.1 Roll, pitch, yaw from basis

Given the body's `Basis` $\mathbf{R}$:

- **Roll** (rotation about $-Z$ forward axis): $\phi = \operatorname{atan2}(R_{YX}, R_{XX})$
- **Pitch** (rotation about $+X$ right axis): $\theta = \arcsin(-R_{ZX})$
- **Yaw** (rotation about $+Y$ up axis): $\psi = \operatorname{atan2}(R_{ZZ}, R_{ZY})$

In GDScript: `body.rotation` gives Euler angles directly (in radians, XYZ
order).  Prefer `body.basis` for vector operations to avoid gimbal lock.

### 12.2 Quaternion interpolation

For smooth orientation interpolation (e.g. waypoint facing direction):

$$\mathbf{q}_\text{interp} = \text{slerp}(\mathbf{q}_a, \mathbf{q}_b, t)$$

In GDScript: `Quaternion.slerp(target_quat, t)`.  Prefer slerp over
converting through Euler angles for any interpolation that crosses $\pm 90°$
pitch.

---

## 13. Quick-reference formula card

| Goal | Formula | GDScript |
|---|---|---|
| Force to hover | $F = mg$ | `mass * 9.8` |
| TWR | $\frac{NF_\text{max}}{mg}$ | `n_motors * max_force / (mass * 9.8)` |
| Hover throttle | $\theta = \frac{mg}{NF_\text{max}}$ | `(mass * 9.8) / (n_motors * max_force)` |
| Terminal velocity | $v = F / (d \cdot m)$ | `total_thrust / (linear_damp * mass)` |
| Steady yaw rate | $\omega = \tau / (d_\omega I)$ | `yaw_torque / (angular_damp * inertia)` |
| Lever torque | $\tau = r \times F$ | `r.cross(F)` |
| Drag force | $F_d = \frac{1}{2}\rho C_D A v^2$ | (manual; approx by `linear_damp`) |
| Wash velocity | $v_w = \sqrt{T / (2\rho A_d)}$ | see §8.3 |
| CoM | $\bar{x} = \sum m_i x_i / \sum m_i$ | sum of weighted positions |
