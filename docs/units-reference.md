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

| Motor class | Approximate thrust at full power |
|---|---|
| 1103 / 1S Tinywhoop | 0.05 – 0.15 N |
| 1404 / 3-inch | 0.30 – 0.50 N |
| 2306 / 5-inch freestyle | 8 – 14 N (≈ 850 – 1450 gf) |
| DJI Mavic 3 prop | ~20 N total (≈ 5 N per motor) |

---

## 4. Gravity

| Property | Value | Path in editor |
|---|---|---|
| Default gravity magnitude | 9.8 m/s² | Project Settings → Physics → 3D → `Default Gravity` |
| Default gravity direction | `Vector3(0, −1, 0)` | same |

Changing the default gravity affects *all* `RigidBody3D` nodes globally.  For
zero-gravity (space) environments, set `gravity_scale = 0.0` on the individual
body instead of changing the project setting.

---

## 5. Torque and angular motion

### 5.1 Torque

| Property | Unit | Where |
|---|---|---|
| `apply_torque(τ)` argument | N·m | RigidBody3D method |
| `yaw_reaction_torque` export | N·m | `FlightQuadController` inspector |

Torque is angular force.  It causes angular acceleration according to:

$$\tau = I \cdot \alpha$$

where $I$ is moment of inertia (kg·m²) and $\alpha$ is angular acceleration
(rad/s²).

Godot's `RigidBody3D` calculates $I$ automatically from `mass` when no
`PhysicsMaterial` shape is assigned; the default approximates a uniform sphere
of the given mass.  The effective $I$ for small drones is approximately:

$$I \approx \frac{2}{5} m r^2$$

where $r$ is the body's collision-shape radius.  For a 0.5 kg drone with a
~0.15 m effective radius: $I \approx 0.0045~\text{kg·m}^2$.

### 5.2 Angular velocity

| Property | Unit | Where |
|---|---|---|
| `angular_velocity` | rad/s | RigidBody3D Inspector (read in code) |
| `angular_damp` | s⁻¹ (damping coefficient) | RigidBody3D Inspector |

To convert observed angular velocity to human-readable:

$$\omega_\text{deg/s} = \omega_\text{rad/s} \times \frac{180}{\pi}$$

Typical real-quad yaw rates:

| Style | Max yaw rate |
|---|---|
| Cinematic / DJI | 100 – 200 °/s |
| Sport mode | 200 – 500 °/s |
| FPV freestyle | 500 – 900 °/s |
| Racing full deflection | > 1000 °/s |

### 5.3 Tuning `yaw_reaction_torque` to a target yaw rate

At steady state (angular velocity constant) torque equals damping force:

$$\tau = \text{angular\_damp} \times I \times \omega_\text{target}$$

Rearranging:

$$\text{yaw\_reaction\_torque} = \text{angular\_damp} \times I \times \omega_\text{target}$$

Example — 0.5 kg drone, $I \approx 0.0045$, `angular_damp = 4.0`, target
400 °/s (6.98 rad/s):

$$\tau = 4.0 \times 0.0045 \times 6.98 \approx 0.13~\text{N·m}$$

Start there and tune upward if yaw feels weak.

### 5.4 `angular_damp` — physical meaning

`angular_damp` applies exponential decay to `angular_velocity` every physics
step:

$$\omega_{t+1} = \omega_t \times e^{-d \cdot \Delta t}$$

Higher values = faster spin-down after releasing the stick.

| `angular_damp` | Feel |
|---|---|
| 0 | No resistance — drone spins forever |
| 1 – 2 | Light / over-powered feel |
| 3 – 5 | Consumer drone feel (recommended start) |
| 6 – 10 | Heavy / sluggish yaw |
| > 10 | Drone stops yawing almost instantly |

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
| `yaw_reaction_torque` | N·m | 0.05 – 0.5 | Scale with mass × 0.3 |
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
