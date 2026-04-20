# Physics Accuracy Review — Drone Model

> Review of the current drone flight model against a "physically accurate
> quadcopter" target.  Two sections:
>
> 1. **Maths to tighten** — existing code that is implemented, but where the
>    formula, variable, or sign is either wrong, inconsistent, or using a
>    cruder approximation than the codebase docs already claim.
> 2. **Missing physics** — effects that real quads exhibit and the
>    simulator currently does not model at all.
>
> Each item links to the offending file / line range (where applicable) and
> is tagged with a rough **impact** (how strongly a player would feel it)
> and **effort** (how much code it takes to add).
>
> This document is a *review*, not a spec — treat it as a to-do list to
> work through, not a contract.  Items may be dropped, deferred, or
> re-scoped once we start implementing.

---

## 1. Maths to tighten

### 1.1 Yaw reaction torque: throttle-vs-RPM ambiguity  ✅ **Done**
**File:** [QuadFlightController.gd](../Drones/Scripts/FlightControllers/QuadFlightController.gd) (reaction-torque block, `τ_yaw = κR · Σ sᵢ·Tᵢ`)
**Impact:** medium · **Effort:** low (documentation + optional square)

Resolved together with §1.2 — `throttle` is now explicitly RPM
fraction, and the reaction-torque sum uses `rpm_frac²` so
$T_i = F_\text{max}\cdot\text{rpm}^2$ feeds
$\tau_\text{yaw} = \kappa R \sum s_i T_i$ consistently.

### 1.2 Spool-up filter applied to thrust, not RPM  ✅ **Done**
**File:** [DroneThrusterScript.gd](../Drones/Scripts/DroneThrusterScript.gd) (`_physics_process` ramp)
**Impact:** medium · **Effort:** low

`throttle` is now semantically RPM fraction; the PowerRamp LPF
operates on it, and the emitted force is
`max_force · throttle² · ground_multiplier`.  Hover stick position
is therefore $\sqrt{mg/(NF_\text{max})}$ rather than the linear form
— see [flight-dynamics-equations.md §2.2](flight-dynamics-equations.md)
for the updated formula and the tuning note.

### 1.3 Ground-effect height uses slant distance, not vertical  ✅ **Done**
**File:** [DroneThrusterScript.gd](../Drones/Scripts/DroneThrusterScript.gd) — `_compute_ground_effect_multiplier`
**Impact:** low-medium · **Effort:** trivial

Now uses the world-Y component of the offset from thruster to
collision point, matching Cheeseman–Bennett's $z$.  Cushion strength
is invariant under body tilt, and the tilt-induced "cushion cuts out
when banked past some angle" failure mode falls out naturally from
the ray missing the ground.

### 1.4 Cheeseman–Bennett domain comment is misleading  ✅ **Done**
**File:** [DroneThrusterScript.gd](../Drones/Scripts/DroneThrusterScript.gd) — same function
**Impact:** cosmetic · **Effort:** trivial

Comment rewritten alongside §1.3 to frame the `h < R/4` clamp
honestly as guarding the formula's singularity at $h \to R/4$, and
to cover the $h \le 0$ edge case.

### 1.5 Per-axis angular drag is linear, not quadratic
**File:** [QuadFlightController.gd](../Drones/Scripts/FlightControllers/QuadFlightController.gd) — angular-drag block
**Impact:** low · **Effort:** low

`τ = −k · ω` matches Godot's built-in `angular_damp`, but real
aerodynamic rotational drag scales with $\omega^2$.  The docs
([flight-dynamics-equations.md §4.2](flight-dynamics-equations.md))
already acknowledge this.  Adding an optional
`τ = −k · ω · |ω|` term gives "heavy air" feel at high yaw rates.

### 1.6 Yaw idle floor produces unwanted collective lift
**File:** [QuadFlightController.gd](../Drones/Scripts/FlightControllers/QuadFlightController.gd) — `yaw_motor_idle` block
**Impact:** medium · **Effort:** low

When yaw is deflected, collective is raised to a minimum floor so the
differential has RPM to act on.  Side effect: pure yaw at hover causes
the drone to climb.  Real pilots expect pure yaw to hold altitude.
Fix: bias the yaw differential *around* the current collective (add to
CW, subtract from CCW) instead of lifting the floor.

### 1.7 Mixing uses `signf(position)`, ignores magnitude  ✅ **Done**
**File:** [QuadFlightController.gd](../Drones/Scripts/FlightControllers/QuadFlightController.gd) — X-quad mix loop
**Impact:** medium (only for non-square frames) · **Effort:** low

Per-motor roll/pitch weights are now $-x_i / r_{\max,x}$ and
$+z_i / r_{\max,z}$ using the existing `get_max_arm_length` helper,
giving the physically correct form

$$T_i = \text{col} + k_r \cdot \tfrac{x_i}{r_{\max,x}}\cdot\text{roll} + k_p \cdot \tfrac{z_i}{r_{\max,z}}\cdot\text{pitch} + k_y \cdot s_i \cdot \text{yaw}$$

For symmetric X-quads every weight is $\pm 1$, so behaviour is
unchanged.  H-frames, stretched-X, and deadcat layouts now receive
the correct per-motor throttle perturbations (a motor twice as far
from the CoM gets half the delta for the same torque).  Degenerate
colinear layouts (all motors on a single axis) fall back to an
unnormalised signed coordinate; the final per-motor clamp keeps this
safe.

---

## 2. Missing physics

### 2.A Rotor gyroscopic / precession torque
**Impact:** low-medium (racing feel) · **Effort:** medium

Each spinning rotor has angular momentum $\mathbf{L} = I_\text{rotor}\,\Omega\,\hat{\mathbf{s}}$.
Body angular velocity $\boldsymbol{\omega}_\text{body}$ produces a
precession torque $\boldsymbol{\tau} = \boldsymbol{\omega}_\text{body} \times \mathbf{L}$.
This is the "pitches under yaw / yaws under pitch" cross-coupling real
quads feel, and is listed in the doc's known-gaps section.

### 2.B Blade-flapping / H-force / translational lift
**Impact:** high · **Effort:** medium-high

Horizontal motion makes the advancing blade faster than the retreating
blade → rotor disc tilts back ("flapback"), producing a body-frame drag
in the direction of motion *plus* a pitch-up torque.  Without this, the
drone has no physical reason to "settle into" forward flight; you must
fake it with damp.

### 2.C Induced-velocity / vortex ring state (VRS)
**Impact:** medium (only during descents) · **Effort:** medium

Thrust is currently `max_force · throttle` regardless of airflow through
the disc.  In a fast descent the rotor ingests its own wake, thrust
collapses, and the drone falls.  A simple "thrust falls 20 % when
descending faster than `v_wash/2`" is cheap and recognisable.

### 2.D Thrust curve: $T \propto \Omega^2$, not linear in command  ✅ **Done**
**Impact:** medium · **Effort:** low
Implemented as part of §1.2.

### 2.E Battery sag / thrust derating
**Impact:** low (mission feel) · **Effort:** low

TWR drops with voltage.  Cheapest model: a `thrust_multiplier` that
decays with integrated throttle-seconds.

### 2.F Air density / altitude
**Impact:** low (unless altitude sims) · **Effort:** low

$T \propto \rho$, $F_\text{drag} \propto \rho$.  Currently implicit at
sea-level $\rho$.

### 2.G Fountain effect / inter-rotor interference
**Impact:** low · **Effort:** trivial (constant factor)

Doc §8.2 notes a 5–15 % loss on close-spaced quads.  Cheapest model: a
per-controller `fountain_loss` multiplier on collective.

### 2.H Quadratic linear drag (terminal velocity)
**Impact:** high · **Effort:** medium

Godot's `linear_damp` is linear in $v$, so terminal velocity scales as
$T/d$ (doubling thrust doubles top speed).  Real drag $\propto v^2$, so
top speed scales as $\sqrt{T/k}$.  Apply explicit
$-\tfrac{1}{2}\rho C_D A |v| \mathbf{v}$ per tick and zero `linear_damp`.

### 2.I Per-axis linear drag
**Impact:** medium · **Effort:** low (depends on 2.H)

A quad has much higher drag laterally (frame + belly) than vertically
(prop discs are nearly transparent to vertical airflow).  Isotropic
damp erases this.  Requires explicit drag (2.H) to expose the axes.

### 2.J Wind / ambient air velocity field
**Impact:** medium · **Effort:** medium

All aero currently assumes still air at the thruster position.  The fix
is to compute drag and prop airflow against $\mathbf{v}_\text{body} - \mathbf{v}_\text{air}$ where `v_air` is sampled from a wind field / function.

### 2.K CoM / thrust-alignment verification
**Impact:** low (quality-of-life) · **Effort:** low

A startup assertion — "sum of motor local positions is approximately
zero w.r.t. CoM" — catches asymmetric builds before they produce
mysterious drift at hover.

### 2.L Inertia tensor override  ✅ **Done**
**Impact:** medium (affects attitude feel) · **Effort:** low

Opt-in `use_motor_point_inertia` flag on
[DroneScript.gd](../Drones/Scripts/DroneScript.gd).  When enabled,
`_ready()` computes the body-local inertia tensor as four point
masses at the thruster positions and writes the result directly to
`RigidBody3D.inertia`, overriding the shape-derived default.

$$m_\text{motor} = \frac{m \cdot f}{N},\quad I_{xx} = \sum m_\text{motor}(y_i^2 + z_i^2),\; I_{yy} = \sum m_\text{motor}(x_i^2 + z_i^2),\; I_{zz} = \sum m_\text{motor}(x_i^2 + y_i^2)$$

where $f$ is the exposed `motor_mass_fraction` (default 0.75 — the
remaining mass is treated as a point at the body origin, contributing
nothing to rotational inertia).  Left off by default so existing
scenes are unaffected.

### 2.M Mixer saturation / desaturation strategy
**Impact:** high · **Effort:** medium

Currently each motor is clamped to [0, 1] independently.  When one
saturates, the commanded roll/pitch/yaw is silently truncated and the
drone loses attitude authority.  Real flight stacks (Betaflight
"airmode", PX4 mixer) use *priority-based desaturation*: scale
collective down to preserve attitude authority.

### 2.N IMU / gyro simulation (for stabilised controllers)
**Impact:** low (PID realism) · **Effort:** medium

PID / angle-hold controllers currently read `body.angular_velocity`
directly.  A real stack measures a gyro with noise, bias drift, and
quantisation.  Only matters if we want the PID tune to transfer to real
hardware.

### 2.O ESC / motor minimum-RPM and active braking
**Impact:** low · **Effort:** low

Motors stall below some command threshold; 3D-capable ESCs can
actively brake.  Cheapest: a `motor_min_command` floor and a
`brake_torque` coefficient on the spool-down path.

---

## Suggested implementation order

Grouped by "biggest physical improvement per line of code":

1. ~~**Ground-effect vertical height** (§1.3)~~ ✅
2. ~~**RPM-vs-thrust clarification + `thrust = cmd²`** (§1.2, §2.D)~~ ✅
3. ~~**Arm-length-aware mixing** (§1.7)~~ ✅ — non-square frames now
   supported.
4. **Quadratic + per-axis linear drag** (§2.H, §2.I) — biggest felt
   change for cruise flight. **← next**
5. **Mixer desaturation** (§2.M) — prevents "loss of authority" bugs when
   stabilisation layer lands on top.
6. **Rotor gyroscopic torque** (§2.A) — cheap realism boost for racing feel.
7. **Blade-flapping / H-force** (§2.B) — gives the drone a physical reason
   to settle into forward flight.

Everything below that (VRS, battery sag, air density, wind field, IMU
simulation) is refinement territory.

---

## Out-of-band additions (not in the original review)

Changes made while resolving review items above, worth recording:

- **Attitude idle floor on pitch/roll** (`attitude_motor_idle`, default
  0.10) — mirror of `yaw_motor_idle`, added after the RPM² thrust
  change.  Without it, pure pitch/roll at zero throttle produces only
  ~6 % thrust on one side (and zero on the other after the clamp),
  killing the classic FPV "stick alive at idle" feel.
- **Per-axis stick expo** (`throttle_expo` / `roll_expo` / `pitch_expo`
  / `yaw_expo` on `FlightQuadController`, plus a shared `apply_expo`
  helper on `FlightControllerBase`) — Betaflight-style cubic shaping
  `(1−e)·x + e·x³`.  Softens the centre-stick region that the T ∝ Ω²
  curve made twitchier, without touching authority endpoints.
