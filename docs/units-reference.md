# Physical Units Reference

> **Audience:** contributors tuning flight-controller exports and drone body
> properties in the Godot editor.
>
> **Goal:** a single place to look up what unit every inspector value is
> expressed in, what realistic real-world ranges look like for the drone
> scale used in this project, and how each value connects to the flight
> controller exports.
>
> **Ground truth:** Godot 4 uses SI units throughout its physics engine.
> 1 Godot unit = 1 metre.  1 unit of mass = 1 kilogram.  1 unit of force =
> 1 Newton.  1 unit of torque = 1 Newton-metre.  There are no hidden scale
> factors.

---

## 1. Spatial scale

| Quantity | Godot unit | = SI unit |
|---|---|---|
| Position, offset | 1 unit | 1 metre (m) |
| Velocity | 1 unit/s | 1 m/s |
| Acceleration | 1 unit/s² | 1 m/s² |

### 1.1 Reference objects in this project

The initial drone scene is approximately **30 cm** (0.3 m) in length along
its longest axis.  Use this as a calibration anchor when placing motors and
checking arm lengths.

| Object | Approximate size |
|---|---|
| Initial drone body | 0.30 m long × 0.20 m wide |
| Motor arm length from centre | 0.12 – 0.15 m |
| Full-size FPV 5-inch quad | ~0.25 m motor-to-motor diagonal |
| DJI Mavic 3 (folded) | ~0.28 m long |
| Human figure | 1.75 m tall |

---

## 2. Mass — `RigidBody3D.mass`

| Property | Unit | Path in editor |
|---|---|---|
| `mass` | kg | Inspector → `RigidBody3D` → `Mass` |

### 2.1 Typical drone masses

| Drone class | Real mass | Suggested simulator mass |
|---|---|---|
| Indoor micro (1S Tinywhoop) | 20 – 30 g | 0.025 kg |
| 3-inch toothpick | 60 – 90 g | 0.075 kg |
| 5-inch freestyle quad | 400 – 700 g | 0.500 kg |
| DJI Mavic 3 | 895 g | 0.900 kg |
| Heavy payload / cinematic | 2 – 4 kg | 2.000 kg |

The initial drone scene is recommended at **0.3 – 0.5 kg** to sit in the
DJI/sport class where hover at ~50% throttle feels natural.

---

## 3. Force — `FlightThruster.max_force`

| Property | Unit | Path in editor |
|---|---|---|
| `max_force` | N (Newtons) | Thruster node → Inspector → `Max Force` |

### 3.1 Thrust-to-weight formula

$$\text{TWR} = \frac{N_\text{thrusters} \times F_\text{max}}{m \times g}$$

where $g = 9.8~\text{m/s}^2$ (Godot's default gravity).

For a stable hover at half-throttle, target **TWR = 2.0**.

### 3.2 Solving for `max_force`

$$F_\text{max} = \frac{\text{TWR} \times m \times g}{N_\text{thrusters}}$$

| Drone mass | TWR | Motors | `max_force` per thruster |
|---|---|---|---|
| 0.3 kg | 2.0 | 4 | **1.47 N** |
| 0.5 kg | 2.0 | 4 | **2.45 N** |
| 0.9 kg | 2.0 | 4 | **4.41 N** |
| 0.5 kg | 4.0 | 4 | **4.90 N** (racing) |

### 3.3 Real motor thrust reference

Thrust values are *per motor* at full throttle on a typical battery.  Values
converted from manufacturer thrust data (gram-force → Newtons via
$1~\mathrm{gf} \approx 0.00981~\mathrm{N}$).

| Motor class | Approximate thrust at full power |
|---|---|
| 0603 / brushed 1S Tinywhoop | 0.15 – 0.35 N (≈ 15 – 35 gf) |
| 1103 / 1S brushless | 0.35 – 0.55 N (≈ 35 – 55 gf) |
| 1404 / 3-inch 4S | 3 – 5 N (≈ 300 – 500 gf) |
| 2306 / 5-inch freestyle 6S | 15 – 25 N (≈ 1500 – 2500 gf) |
| DJI Mavic 3 (per motor) | ~6 – 8 N |

Note: a 5-inch freestyle quad's 2306 motor typically produces 1.5 – 2.5 kgf
per motor on 6S, giving a four-motor total of ~60 – 100 N against a ~700 g
body (TWR ≈ 8 – 14).  Match this pattern when configuring the simulator for
racing-feel drones.

---

## 4. Gravity

| Property | Value | Path in editor |
|---|---|---|
| Default gravity magnitude | 9.8 m/s² | Project Settings → Physics → 3D → `Default Gravity` |
| Default gravity direction | `Vector3(0, -1, 0)` | same |

Changing the default gravity affects *all* `RigidBody3D` nodes globally.  For
zero-gravity (space) environments, set `gravity_scale = 0.0` on the individual
body instead of changing the project setting.

---

## 5. Torque and angular motion

### 5.1 Torque

| Property | Unit | Where |
|---|---|---|
| `apply_torque(τ)` argument | N·m | RigidBody3D method |
| `prop_torque_coefficient` export | m (= $\kappa R$) | `FlightQuadController` inspector |

Torque is angular force.  It causes angular acceleration according to:

$$\tau = I \cdot \alpha$$

where $I$ is moment of inertia (kg·m²) and $\alpha$ is angular acceleration
(rad/s²).

Godot's `RigidBody3D` computes its inertia tensor from the attached
`CollisionShape3D` and the body's `mass`.  The formula used depends on the
shape type:

| Shape | Inertia about principal axis |
|---|---|
| `SphereShape3D` (radius $r$) | $I = \frac{2}{5} m r^2$ |
| `BoxShape3D` (size $a \times b \times c$) | $I_y = \frac{1}{12} m (a^2 + c^2)$ |
| `CapsuleShape3D` (radius $r$, height $h$) | complex; see Godot source |
| `CylinderShape3D` (radius $r$, height $h$) | $I_y = \frac{1}{2} m r^2$ (about Y) |

If you need an explicit inertia tensor independent of the collision shape,
set `RigidBody3D.inertia` to a non-zero `Vector3` (components give inertia
about each body axis).  Setting `inertia = Vector3.ZERO` (default) falls back
to the shape-derived value.

Example — a 0.5 kg drone using a `BoxShape3D` of size 0.30 × 0.10 × 0.30 m:

$$I_y = \frac{1}{12} \times 0.5 \times (0.30^2 + 0.30^2) = 0.0075~\mathrm{kg \cdot m^2}$$

If the collision shape is significantly larger or smaller than the visible
drone body, the resulting $I$ will over- or under-estimate yaw inertia — one
of the most common sources of "drone feels too heavy / too twitchy" once
basic forces are correct.

### 5.2 Angular velocity

| Property | Unit | Where |
|---|---|---|
| `angular_velocity` | rad/s | RigidBody3D (`body.angular_velocity`) |
| `angular_damp` | s⁻¹ (damping coefficient) | RigidBody3D Inspector |

`angular_damp` in Godot 4 is a linear drag coefficient applied every physics
step such that:

$$\boldsymbol{\omega}_{t+1} = \boldsymbol{\omega}_t \times \max(0,\; 1 - d \, \Delta t)$$

For small $d \, \Delta t$ this approximates exponential decay
$\omega(t) = \omega_0 e^{-d t}$.  At the default physics step of
$\Delta t = 1/60$ s, the per-step decay factor is $1 - d/60$, meaning
`angular_damp = 60` would fully stop rotation in a single tick.

To convert observed angular velocity to human-readable:

$$\omega_\text{deg/s} = \omega_\text{rad/s} \times \frac{180}{\pi}$$

Typical real-quad yaw rates:

| Style | Max yaw rate |
|---|---|
| Cinematic / DJI | 100 – 200 °/s |
| Sport mode | 200 – 500 °/s |
| FPV freestyle | 500 – 900 °/s |
| Racing full deflection | > 1000 °/s |

### 5.3 Tuning `prop_torque_coefficient` to a target yaw rate

At steady state (angular velocity constant) yaw torque equals damping
torque:

$$\tau_\text{yaw} = \text{angular\_damp} \times I \times \omega_\text{target}$$

Substituting the physical torque model
$\tau_\text{yaw} = \kappa R \sum_i s_i T_i$ and rearranging:

$$\kappa R = \frac{\text{angular\_damp} \times I \times \omega_\text{target}}{\sum_i s_i T_i}$$

You need a value for $\sum_i s_i T_i$ at full yaw deflection.  For a
symmetric four-motor quad at idle-only thrust (no lift/pitch/roll input),
with `yaw_motor_idle = m`, `yaw_differential = d`, `max_force = F`, and
full yaw stick:

$$\sum_i s_i T_i \approx 2 F \left( \text{clamp}(m - d, 0, 1) - \text{clamp}(m + d, 0, 1) \right)$$

(The factor 2 accounts for the two CW + two CCW motors contributing to
the signed sum.)

Example — 0.5 kg drone, $I \approx 0.0045$, `angular_damp = 4.0`,
`max_force = 4.0 N`, `yaw_motor_idle = 0.1`, `yaw_differential = 0.25`,
target 400 °/s (6.98 rad/s):

$$\sum_i s_i T_i = 2 \times 4.0 \times (0 - 0.35) = -2.8~\mathrm{N}$$

$$|\kappa R| = \frac{4.0 \times 0.0045 \times 6.98}{2.8} \approx 0.045~\mathrm{m}$$

Start there and tune upward if yaw feels weak.

### 5.4 `angular_damp` — physical meaning

`angular_damp` applies exponential decay to `angular_velocity` every physics
step:

$$\omega_{t+1} = \omega_t \times e^{-d \cdot \Delta t}$$

Higher values = faster spin-down after releasing the stick.

| `angular_damp` | Per-second decay | Feel |
|---|---|---|
| 0 | 0% | No resistance — drone spins forever once torqued |
| 1 | ~63% in 1 s | Light — agile, minimal aerodynamic damping |
| 3 | ~95% in 1 s | Moderate — consumer drone feel (recommended start) |
| 5 – 8 | ~99% in 1 s | Heavy — cinematic / DJI-stabilised feel |
| > 10 | > 99% in < 0.5 s | Strongly over-damped — yaw stops nearly instantly |

These are starting points, not calibrated to real aerodynamics.  The correct
value depends on the body's moment of inertia, yaw-torque magnitude, and
target maximum yaw rate — see §5.3 for the physics-based calculation.

---

## 6. Linear damping — `linear_damp`

| Property | Unit | Where |
|---|---|---|
| `linear_damp` | s⁻¹ | RigidBody3D Inspector |

Simulates aerodynamic drag on translation.  Real quads have significant drag
at high speeds.  A value of **0.5 – 2.0** adds a natural-feeling speed cap
without requiring explicit drag forces in the controller.

---

## 7. Controller export cheat-sheet

Complete reference of every flight-controller export with its unit and a
realistic starting range for the initial 0.3–0.5 kg drone.

### FlightControllerBase (inherited by all controllers)

*(Input action names — no physical unit.)*

### FlightFpsController

| Export | Unit | Realistic range | Notes |
|---|---|---|---|
| `power_authority` | dimensionless [0–1] | 0.8 – 1.0 | Output ceiling |
| `yaw_tilt_angle` | radians | 0.1 – 0.5 rad | Tilt-rotor model only |
| `yaw_idle_thrust` | dimensionless [0–1] | 0.10 – 0.20 | Tilt-rotor idle |
| `yaw_throttle_attenuation` | dimensionless [0–1] | 0.4 – 0.8 | Tilt-rotor blend |

### FlightQuadController

| Export | Unit | Realistic range | Notes |
|---|---|---|---|
| `yaw_differential` | dimensionless [0–1] | 0.15 – 0.35 | Motor Δthrottle |
| `prop_torque_coefficient` | m (= $\kappa R$) | 0.01 – 0.05 | Physical $\approx$ 0.02 for 5-inch props; game drones may need more |
| `yaw_motor_idle` | dimensionless [0–1] | 0.05 – 0.15 | Idle floor |

### RigidBody3D (set on drone body node, not the controller)

| Property | Unit | Realistic range | Notes |
|---|---|---|---|
| `mass` | kg | 0.025 – 4.0 | See §2.1 |
| `linear_damp` | s⁻¹ | 0.5 – 2.0 | Air drag substitute |
| `angular_damp` | s⁻¹ | 3.0 – 8.0 | Gyroscopic resistance |
| `gravity_scale` | dimensionless | 1.0 (nominal) | 0 for space craft |

### FlightThruster (set on each thruster node)

| Property | Unit | Realistic range (0.5 kg quad) | Notes |
|---|---|---|---|
| `max_force` | N | 2.0 – 5.0 | See §3.2 formula |
| `throttle` | dimensionless [0–1] | set by controller | Do not set manually |
| `force_axis` | unit vector | `Vector3.UP` | Set by `set_thrust` |
