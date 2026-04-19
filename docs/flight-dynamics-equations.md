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
| $\kappa$ | Rotor torque coefficient (dimensionless) | — |
| $\alpha$ | Angle of attack (fixed wing) / angular acceleration (dynamics) | rad or rad/s² |
| $\Delta t$ | Physics time step | s |

---

## 0. Raw physics ↔ Godot approximation

> A recurring confusion when working across "real physics" references and a
> game engine's physics API is that the two speak different dialects.  Every
> section below gives the equations in two forms where they differ:
>
> - **Raw physics** — the textbook form, valid for analysis, sizing, and
>   sanity-checking.  These are what you'd find in an aerospace or
>   classical-mechanics textbook.  Used when reasoning about what the
>   simulator *should* produce.
> - **Godot approximation / idiom** — what the engine actually computes each
>   physics tick given the available API (`apply_force`, `apply_torque`,
>   `linear_damp`, `angular_damp`, `RigidBody3D.inertia`, etc.).  These are
>   what you'd type in GDScript.
>
> Where the two match exactly (Newton's laws, gravity, lever-arm torque), only
> one form is given.  Where they diverge (drag, ground effect, prop wash) the
> raw form is labelled first, then the approximation used in-engine.

### 0.1 Side-by-side reference

Common quantities, in both languages.

| Quantity | Raw physics | Godot idiom |
|---|---|---|
| Force on body | $\mathbf{F}_\text{net} = m\,\mathbf{a}$ | `body.apply_force(F, offset)` |
| Torque on body | $\boldsymbol{\tau} = I\,\boldsymbol{\alpha}$ | `body.apply_torque(T)` |
| Gravity | $\mathbf{F}_g = m\mathbf{g}$ | Applied automatically; scale with `gravity_scale` |
| Off-centre thrust → torque | $\boldsymbol{\tau} = \mathbf{r} \times \mathbf{F}$ | `body.apply_force(F, r)` — Godot computes the cross product internally |
| Quadratic drag | $\mathbf{F}_d = -\tfrac{1}{2}\rho C_D A v^2 \hat{\mathbf{v}}$ | approximated by `linear_damp` (linear in $v$, not $v^2$) |
| Rotational drag | $\boldsymbol{\tau}_d = -c_\omega\,\omega^2\,\hat{\boldsymbol{\omega}}$ (or linear: $-d\,I\,\boldsymbol{\omega}$) | `angular_damp` (per-tick linear) |
| Moment of inertia | $\int r^2\,dm$ over body | Derived from `CollisionShape3D`; overridable via `RigidBody3D.inertia` |
| Attitude | Rotation matrix $\mathbf{R}$ or quaternion $\mathbf{q}$ | `body.basis` (3 column vectors) or `body.quaternion` |
| Angular velocity | $\boldsymbol{\omega}$ (rad/s) | `body.angular_velocity` (Vector3, rad/s) |
| Integration method | Semi-implicit Euler | Godot's built-in integrator (also semi-implicit Euler) |

### 0.2 Where the approximations hurt

Being explicit about where Godot's cheap approximations diverge from real
aerodynamics helps when the simulator *feels* wrong despite correct-looking
numbers:

1. **Drag is linear, not quadratic.**  Real drag grows as $v^2$ — doubling
   airspeed quadruples drag.  `linear_damp` grows only as $v$, so terminal
   velocity in-engine is proportional to thrust rather than to $\sqrt{\text{thrust}}$.
   Practical effect: an engine-tuning change by 2× doubles terminal speed
   in-sim, where it would raise it by only ~1.4× in reality.
2. **Angular damp is uniform across axes.**  Real drone yaw has much higher
   aerodynamic damping than pitch/roll (the frame is broad and flat in plan,
   thin in side view).  Godot uses a single `angular_damp` scalar across all
   three axes unless you apply per-axis damping in code.
3. **No inertia tensor off-diagonals.**  The engine treats the body as
   having its principal axes aligned with the body frame — valid for
   symmetric drones, wrong for asymmetric payloads (hanging camera, offset
   battery).  For those cases, simulate with a symmetric body and apply the
   mass offset via `center_of_mass`.
4. **No propeller gyroscopic effect.**  Real spinning rotors resist attitude
   changes like gyroscopes.  `FlightThruster` models thrust only, not rotor
   angular momentum — a racing quad's "tilt feels like pushing through
   syrup" character is absent.  Can be added later by attaching a small
   `angular_velocity` on each rotor `Node3D` and applying the resulting
   precession torque manually.
5. **No air velocity field.**  Wind, downwash between rotors, and ground
   effect all assume still air at every thruster's position.  Any of these
   effects must be added as explicit forces (see §8).

Treat these as known *feature gaps*, not bugs in the existing controllers.
The quadcopter *feel* is already correct for its approximation class; adding
the items above is the path toward higher-fidelity flight.

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

**Raw physics.**  Aerodynamic drag on a body moving through a fluid scales
with the square of airspeed:

$$\mathbf{F}_\text{drag} = -\frac{1}{2} \rho \, C_D \, A \, v^2 \, \hat{\mathbf{v}}$$

where $\hat{\mathbf{v}}$ is the unit velocity direction (opposing drag).
This is the correct form for any surface moving through air faster than a
few cm/s.  The coefficient $C_D$ depends on shape and Reynolds number; typical
values range from $\sim 0.04$ (streamlined body) through $\sim 1.0$ (flat
plate broadside to flow) to $\sim 1.3$ (cube).

**Godot approximation.**  `linear_damp` applies a *linear* drag proportional
to velocity:

$$\mathbf{F}_\text{drag,Godot} = -d \, m \, \mathbf{v}$$

This is a simplification but is numerically stable and produces a natural
terminal-velocity feel without per-tick force computation.  The two forms
give identical terminal velocity when sized correctly, and diverge only in
the *transient* approach to that terminal velocity (linear damp decelerates
slower at high $v$, faster at low $v$, compared to quadratic drag).

To size `linear_damp` for a target terminal velocity $v_\text{term}$ at full
thrust $T$, set net force to zero at $v = v_\text{term}$:

$$T = d \, m \, v_\text{term} \implies d = \frac{T}{m \, v_\text{term}}$$

Example: 0.5 kg drone, 9.8 N total thrust, target terminal horizontal speed
20 m/s:

$$d = \frac{9.8}{0.5 \times 20} = 0.98 \approx 1.0$$

For a scene requiring quadratic-drag behaviour (e.g. a fixed-wing aircraft
where drag rise with airspeed matters), set `linear_damp = 0` and apply the
raw-physics form as an explicit `apply_force` each `_physics_process`.

### 4.2 Angular drag

**Raw physics.**  Rotational aerodynamic drag on a bluff body scales with
$\omega^2$ (same reasoning as linear drag, applied tangentially to the
rotating surface):

$$\boldsymbol{\tau}_\text{drag,raw} = -\tfrac{1}{2} \rho \, C_{D,\omega} \, A_\text{ref} \, r^3 \, \omega^2 \, \hat{\boldsymbol{\omega}}$$

**Godot approximation.**  `angular_damp` applies a linear decay to angular
velocity each physics step:

$$\boldsymbol{\omega}_{t+1} = \boldsymbol{\omega}_t \times \max(0,\; 1 - d_\omega \, \Delta t)$$

which for small $d_\omega \Delta t$ approximates exponential decay
$\boldsymbol{\omega}(t) = \boldsymbol{\omega}_0 \, e^{-d_\omega t}$, equivalent
to a torque of:

$$\boldsymbol{\tau}_\text{drag,Godot} = -d_\omega \, I \, \boldsymbol{\omega}$$

Applied uniformly to all three body axes — a limitation discussed in §0.2.

---

## 5. Reactive torque (propeller yaw couple)

### 5.1 Aerodynamic drag torque from a spinning prop

A propeller spinning at angular velocity $\Omega$ (rad/s) produces thrust $T$
and a reaction torque $Q$ on the motor shaft.  For a rotor of radius $R$
operating in hover, momentum theory gives:

$$Q = \kappa \, T \, R$$

where $\kappa$ is a dimensionless efficiency factor (≈ 0.05 – 0.15 for
typical small drone props; varies with prop pitch, Reynolds number, and
induced-flow state).  Hence $Q$ has units $[\mathrm{N} \cdot \mathrm{m}]$.

The body receives the *reaction* torque $-Q$ about the prop's spin axis —
this is the mechanism that yaws a quadcopter.

### 5.2 Net yaw torque from a quadcopter

For a layout with CW and CCW motors (spin signs $s_i \in \{+1, -1\}$),
the total reaction torque on the body about the yaw axis is:

$$\tau_\text{yaw} = -\sum_i s_i \, Q_i = -\kappa R \sum_i s_i \, T_i$$

At equal thrust across a symmetric layout ($\sum s_i = 0$), $\tau_\text{yaw}
= 0$.  During a yaw manoeuvre, speeding one diagonal pair by $+\Delta T$ and
slowing the other by $-\Delta T$ gives:

$$\tau_\text{yaw} = 2 \kappa R \, \Delta T \quad \text{(quadcopter, single pair)}$$

**In the simulator:** `FlightQuadController` exposes $\kappa R$ directly
as the `prop_torque_coefficient` export (units of metres).  The per-motor
thrust $T_i = \text{max\_force}_i \cdot \text{motor\_throttle}_i$ is
already available from the yaw-differential mixing step, so the torque
is applied as
`apply_torque(body.basis.y * prop_torque_coefficient * Σ sᵢ·Tᵢ)`.
The sign falls out of the thrust split — no hand-tuned negation is
needed.  This torque acts about the body's local Y axis, so it correctly
rotates the drone about its own up axis regardless of attitude, and it
scales naturally with throttle (unlike the earlier constant-torque
implementation).

### 5.3 Steady-state yaw rate

At steady state, input torque equals damping torque:

$$\tau_\text{yaw} = d_\omega \, I \, \omega_\text{yaw}$$

$$\omega_\text{yaw} = \frac{\tau_\text{yaw}}{d_\omega \, I}$$

This gives the formula used in [units-reference.md §5.3](units-reference.md)
to solve for `prop_torque_coefficient` from a target yaw rate.

---

## 6. Moment of inertia

Godot 4's `RigidBody3D` derives its inertia tensor from the attached
`CollisionShape3D` and the body's `mass`.  The following are the standard
closed-form inertias about a principal axis through the centre of mass for
common shapes.

### 6.1 Solid sphere

$$I_\text{sphere} = \frac{2}{5} m r^2$$

Appropriate for roughly spherical bodies (a compact drone hull modelled as a
`SphereShape3D`).

### 6.2 Solid box (rectangular cuboid)

For a box of edge lengths $a, b, c$ aligned with the body axes:

$$I_x = \frac{1}{12} m (b^2 + c^2), \quad I_y = \frac{1}{12} m (a^2 + c^2), \quad I_z = \frac{1}{12} m (a^2 + b^2)$$

For a typical 0.5 kg quadcopter with a $0.30 \times 0.10 \times 0.30$ m
`BoxShape3D`:

$$I_y = \frac{1}{12} \times 0.5 \times (0.30^2 + 0.30^2) = 0.0075~\mathrm{kg \cdot m^2}$$

### 6.3 Thin rod / arm (realistic quad frame estimate)

The yaw-axis inertia of an idealised quad frame is dominated by the masses
at the ends of the arms:

$$I_\text{arm model} = \sum_i m_i r_i^2$$

For four equal motors of mass $m_\text{motor}$ at arm radius $r$:

$$I_y \approx 4 \, m_\text{motor} \, r^2$$

This tends to *underestimate* real yaw inertia (it ignores the central hub
and battery) by 20 – 40 %.

### 6.4 Parallel-axis theorem

To shift an inertia from an object's own centroid to a parallel axis at
distance $d$:

$$I_\text{offset} = I_\text{cm} + m \, d^2$$

Used when composing total inertia from separately-inertia'd subcomponents
(motors, battery, FC stack) each offset from the body centre.

### 6.5 Overriding inertia in Godot

If the shape-derived inertia is wrong for your model (e.g. you want to match
a measured real drone rather than the collision hull), set
`RigidBody3D.inertia` to a non-zero `Vector3`.  Components give the principal
inertias about each body axis.  `Vector3.ZERO` (default) falls back to the
shape-derived value.

## 7. Centre of mass and centroid

### 7.1 Composite body centre of mass

For a drone body with $n$ components each of mass $m_i$ at position
$\mathbf{x}_i$:

$$\mathbf{x}_\text{cm} = \frac{\sum_i m_i \, \mathbf{x}_i}{\sum_i m_i}$$

If the centre of mass does not coincide with the `RigidBody3D` origin, all
four motors apply asymmetric lever arms, causing pitch/roll coupling under
throttle.

### 7.2 Practical check in Godot

Godot 4 supports offset centre of mass via two `RigidBody3D` properties:

- `center_of_mass_mode` — `AUTO` (derived from shapes) or `CUSTOM`.
- `center_of_mass` — a `Vector3` offset, used when mode is `CUSTOM`.

In `AUTO` mode with a single symmetric `CollisionShape3D`, the CoM coincides
with the body origin.  For a drone with heavy mass concentrations (battery,
camera gimbal) that are offset from the hull centre, use `CUSTOM` mode and
supply the weighted-average offset.

Verification check for a symmetric quad:

- Sum of motor positions: $\sum \mathbf{p}_i = \mathbf{0}$ (body frame)
- At hover throttle, body attitude remains level with no pitch/roll drift

A non-zero sum or a persistent tilt under collective throttle indicates a
CoM/thrust misalignment.  Fix by moving thrusters, adjusting `center_of_mass`,
or rebalancing component masses.

---

## 8. Ground effect and prop wash

### 8.1 Ground effect — lift augmentation near a surface

When a rotor operates within roughly one rotor diameter of the ground, the
downwash is partially blocked, recirculating into an elevated-pressure cushion
that augments lift.  The classical Cheeseman–Bennett relation for a single
rotor in hover is:

$$\frac{T_\text{IGE}}{T_\text{OGE}} = \frac{1}{1 - \left(\dfrac{R}{4 z}\right)^2}$$

where:
- $T_\text{IGE}$ = thrust in ground effect at the same power setting
- $T_\text{OGE}$ = thrust out of ground effect (free air)
- $R$ = rotor radius
- $z$ = hub height above ground ($z > R/4$ for the formula to be physical)

Behaviour: as $z \to \infty$, ratio $\to 1$ (no effect).  At $z = R$, ratio
$\approx 1.07$ (7 % lift gain).  At $z = R/2$, ratio $\approx 1.33$ (33 %
gain).  Pilots experience this as the drone "floating" just before landing.

**Implementation note:** not currently modelled by `FlightThruster`.  To add
it, cast a ray downward from each rotor, measure $z$, and multiply the
thruster's effective `max_force` by the Cheeseman–Bennett ratio when
$z < 2R$ (above that the effect is negligible).

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

For completeness, the steady-flow lift equation used by fixed-wing
controllers:

$$L = \frac{1}{2} \rho \, v^2 \, C_L \, A$$

where $A$ is the wing planform area and $C_L$ depends on angle of attack
$\alpha$.  For a thin symmetric aerofoil, inviscid theory gives:

$$C_L \approx 2 \pi \, \alpha \quad \text{(radians, } |\alpha| \lesssim 10\degree\text{)}$$

This linear relationship holds only up to the **stall angle** (typically
$\alpha_\text{stall} \approx 12 \degree \text{ to } 18\degree$ depending on
aerofoil and Reynolds number).  Beyond stall, $C_L$ drops abruptly as flow
separates from the upper surface.  Any fixed-wing simulation must either
clamp $\alpha$ below stall or model the post-stall drop explicitly.

The corresponding lift-induced drag (ignoring parasitic drag):

$$D_\text{induced} = \frac{L^2}{\frac{1}{2} \rho v^2 \pi A R_\text{aspect}}$$

where $R_\text{aspect} = b^2 / A$ is the aspect ratio ($b$ = wingspan).
Higher aspect ratios (gliders, $R_\text{aspect} > 10$) produce much less
induced drag than low aspect ratios (delta wings, $R_\text{aspect} < 3$).

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

For an X-quad with motors at body-frame positions
$\text{FL}(-r, 0, -r)$, $\text{FR}(+r, 0, -r)$, $\text{RR}(+r, 0, +r)$,
$\text{RL}(-r, 0, +r)$ and spin signs (+1 = CW, −1 = CCW) FL/RR = +1,
FR/RL = −1 (see
[quad-flight-controller-guide.md §3](quad-flight-controller-guide.md)):

$$\begin{pmatrix} T_\text{FL} \\ T_\text{FR} \\ T_\text{RR} \\ T_\text{RL} \end{pmatrix}
= \frac{F_\text{max}}{4} \begin{pmatrix}
1 & +1 & +1 & +1 \\
1 & -1 & +1 & -1 \\
1 & -1 & -1 & +1 \\
1 & +1 & -1 & -1
\end{pmatrix}
\begin{pmatrix}
\text{throttle} \\ \text{roll} \\ \text{pitch} \\ \text{yaw}
\end{pmatrix}$$

Each row is the per-motor contribution of each command channel.  Reading
columns:
- **throttle** — all motors equal (collective lift).
- **roll** (positive = roll right) — left-side motors (FL, RL) up, right-side
  (FR, RR) down → body rolls right.
- **pitch** (positive = pitch up / nose up) — front motors (FL, FR) down,
  rear (RR, RL) up → body pitches nose-up.
- **yaw** (positive = yaw left / CCW from above, matching Godot's +Y rotation
  convention) — CCW-spinning motors (FR, RL, spin sign −1) raised, CW motors
  (FL, RR, spin sign +1) lowered → net CCW drag torque on body.

Each output is clamped to $[0, F_\text{max}]$.  This matrix is the target for
a fully mixed stabilised controller.  The current `FlightQuadController`
handles yaw only via this column plus an explicit `apply_torque` call; pitch
and roll mixing come from the attitude / PID layer (see
[flight-controller-guide.md](flight-controller-guide.md)).

---

## 11. PID control (stabilisation)

### 11.1 Textbook PID

The standard discrete PID on the *error* signal $e(t) = \theta_\text{target}
- \theta_\text{measured}$:

$$u(t) = K_P \, e(t) + K_I \sum_k e(k) \, \Delta t + K_D \frac{e(t) - e(t-1)}{\Delta t}$$

where $K_P, K_I, K_D$ are proportional, integral, and derivative gains.

### 11.2 Derivative on measurement (preferred for flight control)

The textbook form above suffers from **derivative kick** — when
$\theta_\text{target}$ changes suddenly (e.g. player moves the stick), the
abrupt step in $e$ causes a large spurious $D$ output.  The common fix is to
take the derivative of the *measurement* $\theta$ rather than the error
(their derivatives differ only in sign when the setpoint is constant):

$$u(t) = K_P \, e(t) + K_I \sum_k e(k) \, \Delta t - K_D \frac{\theta(t) - \theta(t-1)}{\Delta t}$$

For drone attitude stabilisation this is equivalent to using the gyro angular
rate directly as the $D$ term — hence the common naming "PD on rate" in
flight-control literature.

### 11.3 Integral windup guard

When the controller output saturates (motor at 0 or $F_\text{max}$), the
integral continues accumulating error the controller cannot act on, causing
overshoot when saturation releases.  Clamp the integral state:

$$\int e \leftarrow \text{clamp}\!\left(\int e,\; -I_\text{max},\; +I_\text{max}\right)$$

Typical $I_\text{max}$: large enough that $K_I \cdot I_\text{max}$ equals the
full output range (e.g. $\pm 1$).  Better alternatives include *conditional
integration* (only integrate when output is unsaturated) and *back-calculation*
(reduce integral when output is clamped).

---

## 12. Euler angles and attitude representation

### 12.1 Reading attitude from a Godot `Basis`

Godot's `Basis` stores the body's local axes as its three columns:
`basis.x` is body-right, `basis.y` is body-up, `basis.z` is body-back
(opposite of forward, which is $-Z$).  **Prefer reading attitude directly
from these vectors rather than via Euler decomposition** — Euler forms are
ambiguous across the $\pm 90\degree$ pitch singularity (gimbal lock) and
order-dependent, whereas the column vectors are unambiguous.

Useful attitude queries in terms of the basis:

| Query | Expression |
|---|---|
| Body-forward vector (world) | $-\text{basis}.z$ |
| Body-up vector (world) | $\text{basis}.y$ |
| Body-right vector (world) | $\text{basis}.x$ |
| Tilt angle from vertical | $\arccos(\text{basis}.y \cdot \hat{Y}_\text{world})$ |
| Heading (yaw about world +Y) | $\operatorname{atan2}(-\text{basis}.z.x,\; -\text{basis}.z.z)$ |

If Euler angles are required (e.g. for a stabilised controller comparing
target pitch/roll to current), Godot provides `basis.get_euler()`, which
returns a `Vector3` using the default YXZ Tait–Bryan order.  Components:

- `.y` — yaw (rotation about world Y), range $[-\pi, \pi]$.
- `.x` — pitch (rotation about intermediate X), range $[-\pi/2, \pi/2]$.
- `.z` — roll (rotation about body Z), range $[-\pi, \pi]$.

Gimbal lock occurs when pitch $\approx \pm 90\degree$ and yaw/roll become
indistinguishable — do not rely on Euler angles for any manoeuvre that may
cross vertical.

### 12.2 Quaternion interpolation

For smooth orientation interpolation (e.g. waypoint facing direction):

$$\mathbf{q}_\text{interp} = \text{slerp}(\mathbf{q}_a, \mathbf{q}_b, t)$$

In GDScript: `quat_a.slerp(quat_b, t)`, or
`basis_a.get_rotation_quaternion().slerp(basis_b.get_rotation_quaternion(), t)`
if you have bases.  Always prefer slerp over interpolating through Euler
angles, which produces non-uniform angular velocity and misbehaves near
gimbal lock.

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
