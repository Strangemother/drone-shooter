# Quad Flight Controller — Implementation Guide

> **Audience:** future agents (Copilot, Claude, etc.) and human contributors
> working on the CW/CCW differential-yaw flight controller in this project —
> primarily
> [`Drones/Scripts/FlightControllers/QuadFlightController.gd`](../Drones/Scripts/FlightControllers/QuadFlightController.gd).
>
> **Goal:** document the full X-quad mixing matrix, motor spin model, yaw
> reaction-torque layer, per-axis aerodynamic drag, motor spool dynamics
> (PowerRamp), and the tuning knobs for each.  Also catalogue the sign and
> convention issues that come up on import.
>
> **Prerequisite:** read [fps-flight-controller-guide.md](fps-flight-controller-guide.md)
> first — `FlightQuadController` extends `FlightFpsController` for input
> action names and `power_authority`.  **Translation behaviour is NOT
> inherited** from the parent: the quad controller implements its own
> four-channel mixer (§4) and the parent's motor-tilt yaw model is
> deliberately unused.  Read [units-reference.md](units-reference.md) for
> physical unit context before tuning `prop_torque_coefficient`.
>
> **Status:** reflects the working implementation of `FlightQuadController`
> after the full X-quad mixing matrix, PowerRamp-on-thruster, per-axis
> angular drag, and pilot-orientation inversion exports were added
> (April 2026 session).

---

## 1. The single most important rule

> **A quadcopter yaws through angular momentum, not tilt.  The yaw couple
> exists because CW and CCW props produce equal and opposite drag at equal RPM —
> differential RPM breaks that balance.  Do not add motor tilt to this
> controller; that is a separate model with incompatible assumptions.**

The tilt-rotor yaw of the parent `FlightFpsController` and the differential-RPM
yaw of `FlightQuadController` are mutually exclusive.  Both can produce a body
rotation, but they behave differently under load and at idle, and mixing them
produces unintuitive interaction.  The quad controller deliberately never
populates `_motor_rest_basis` and never calls the tilt path.

---

## 2. How a real quadcopter yaws

Understanding the physical model prevents sign and tuning errors.

### 2.1 Propeller drag torque

Every spinning propeller exerts a torque on the air equal and opposite to the
torque the motor shaft exerts on the prop.  In still air, the reaction is borne
by the drone's frame.  A CW prop produces a CCW reaction torque on the body; a
CCW prop produces a CW reaction torque.

In a symmetric quadcopter, two CW and two CCW props are arranged so that at
equal RPM the four drag torques sum to zero — no yaw.

### 2.2 Differential RPM yaw

To yaw, the flight controller speeds up one diagonal pair (say, CCW motors)
while slowing the opposite diagonal pair (CW motors) by the same amount:

- Faster CCW motors → larger CW drag torque on body
- Slower CW motors → smaller CCW drag torque on body
- Net result: CW torque on body → body yaws CW (right)

Reversing the differential gives a CCW (left) yaw.  Thrust output is conserved
because the speed-up on one pair compensates the slow-down on the other.

### 2.3 Why this is different from tilt-rotor yaw

A tilt-rotor yaw works by redirecting motor thrust tangentially.  It requires
motor nodes to physically rotate (see `FpsFlightController` commit history) and
needs a minimum `yaw_idle_thrust` because a zero-thrust motor has nothing to
redirect.

Differential-RPM yaw has no such requirement — prop drag exists even at high
throttle, and the torque scales with RPM rather than with the direction of
thrust.  The practical difference: differential yaw is usually *more* authority
at high throttle, while tilt yaw is more authority at low/idle.

---

## 3. Spin-sign assignment — the standard X-quad layout

The controller determines whether each motor is CW or CCW by looking at its
position in the body's local XZ plane.

For the standard X-quad layout viewed from above:

```
        -Z (front)
     FL  ·  FR
  -X  ·     ·  +X
     RL  ·  RR
        +Z (rear)

  FL (x < 0, z < 0) → x·z > 0 → CW  (+1)
  FR (x > 0, z < 0) → x·z < 0 → CCW (−1)
  RR (x > 0, z > 0) → x·z > 0 → CW  (+1)
  RL (x < 0, z > 0) → x·z < 0 → CCW (−1)
```

The sign rule: a motor at position $(x, z)$ in the body frame has spin sign

$$\text{spin} = \text{sgn}(x \cdot z)$$

At equal throttle, FL+RR (CW) and FR+RL (CCW) drag torques cancel → no yaw.
This is the same layout used by DJI, Betaflight, and virtually every X-frame
commercial or racing quadcopter.

### 3.1 Non-standard layouts

For hex frames, Y-frames, V-tails, or any layout that does not follow the sign
rule above, populate the `spin_sign_override` dictionary before the first
physics tick:

```gdscript
func _ready() -> void:
    var controller := $QuadFlightController as FlightQuadController
    controller.spin_sign_override[$Thruster_FL] =  1  # CW
    controller.spin_sign_override[$Thruster_FR] = -1  # CCW
    # … etc.
```

A warning is pushed to the console if any motor lies exactly on the X or Z
body axis (indeterminate position) — assign it manually via the override.

---

## 4. The full X-quad mixing matrix

### 4.1 Channel layout

`FlightQuadController` implements the classic four-channel mix from
[flight-dynamics-equations.md §10.2](flight-dynamics-equations.md).
Each motor's throttle is a linear sum of four command channels:

$$\begin{pmatrix} T_\text{FL} \\ T_\text{FR} \\ T_\text{RR} \\ T_\text{RL} \end{pmatrix}
= \begin{pmatrix}
1 & +1 & +1 & +1 \\
1 & -1 & +1 & -1 \\
1 & -1 & -1 & +1 \\
1 & +1 & -1 & -1
\end{pmatrix}
\begin{pmatrix}
\text{throttle} \\ \text{roll} \\ \text{pitch} \\ \text{yaw}
\end{pmatrix}$$

The controller reads the four pilot inputs via `get_axis_value` and scales
each column by its own authority export:

| Channel | Stick | Authority export | Role |
|---|---|---|---|
| throttle | `throttle_up`/`_down` | `power_authority` | collective lift |
| roll | `roll_right`/`_left` | `roll_authority` | differential L↔R thrust |
| pitch | `pitch_forward`/`_backward` | `pitch_authority` | differential front↔rear thrust |
| yaw | `yaw_right`/`_left` | `yaw_differential` | differential CW↔CCW spin |

### 4.2 Per-motor formula

Given body-local position $(x_i, z_i)$ and spin sign $s_i \in \{+1, -1\}$:

$$T_i = \text{clamp}\!\bigl(
c
\; {-}\; \operatorname{sgn}(x_i) \cdot r \cdot a_r
\; {+}\; \operatorname{sgn}(z_i) \cdot p \cdot a_p
\; {-}\; s_i \cdot y \cdot a_y,
\; 0,\; 1\bigr)$$

where $c$ is the collective throttle after `power_authority` and the
`yaw_motor_idle` floor (§4.4), and $r, p, y$ are the three stick inputs
in $[-1, 1]$.  Sign choices (so you can audit them later):

| Axis | Stick > 0 means | Matrix sign | This controller's sign | Why |
|---|---|---|---|---|
| roll | body rolls right | $-\operatorname{sgn}(x_i)$ | $-\operatorname{sgn}(x_i)$ | matches matrix directly |
| pitch | nose down | $-\operatorname{sgn}(z_i)$ | $+\operatorname{sgn}(z_i)$ | flipped — input is nose-down, matrix is nose-up |
| yaw | body yaws CW | $+s_i$ | $-s_i$ | flipped — input is CW, matrix is CCW |

The pitch/yaw flips are a *convention* adjustment, not a bug.  They keep
the pilot's mental model (forward stick → nose drops → forward flight,
right yaw stick → right turn) aligned with Godot's coordinate system
without having to relabel input actions.

### 4.3 Attitude-based flight (ACRO-mode behaviour)

All motors push straight along their own local UP axis.  Lateral flight
happens because the pitch/roll channels **tilt the body**, and the tilted
thrust vector then has a horizontal component.  This is the standard
ACRO-mode behaviour of real quadcopters:

- Push roll stick right → right motors slow, left motors speed up → body
  rolls right → straight-up thrust now points partially to the right →
  drone accelerates right.
- Push pitch stick forward → front motors slow, rear motors speed up →
  nose drops → thrust vector points partially forward → drone accelerates
  forward.

Consequences the pilot will notice:

- **Drone tilts to fly** — there is no "strafe flat" motion in this
  controller.  A stabilised successor (angle-mode / horizon-mode) will
  sit on top of this mixer later and convert stick into *target attitude*.
- **Lateral stops take longer** — the body must rotate back to level
  before translation stops.  This inertia is correct; do not eliminate
  it with damping.
- **Pitch/roll authority scales with throttle** — at idle the motors
  can't differentiate below zero, so attitude authority is weak.  Unlike
  `yaw_motor_idle`, there is deliberately no pitch/roll idle floor
  (matches real-drone feel).

### 4.4 Budget conservation

The total throttle budget across four motors is conserved when each
differential column sums to zero across motors:

$$\sum_i T_i = 4 c
- a_r \cdot r \cdot \underbrace{\sum \operatorname{sgn}(x_i)}_{=0}
+ a_p \cdot p \cdot \underbrace{\sum \operatorname{sgn}(z_i)}_{=0}
- a_y \cdot y \cdot \underbrace{\sum s_i}_{=0}$$

For a symmetric X-quad the three sums are all zero (two motors on each
side, two of each spin), so the summed throttle is independent of the
roll/pitch/yaw sticks.  Attitude manoeuvres do not reduce total lift —
matching real-world quadcopter behaviour.  Clamping per motor to $[0, 1]$
can break this at the saturation edges; that's the same effect real
pilots know as "yaw/roll washout at full throttle".

### 4.5 Yaw motor idle floor

At zero throttle the differential changes RPM from zero — still zero.  The
`yaw_motor_idle` export raises all motors to a minimum floor proportional to
yaw input:

$$\theta_\text{floor} = \text{yaw\_motor\_idle} \times |\text{yaw}| \times \text{power\_authority}$$

$$\theta_\text{effective} = \max(\theta_\text{base},\; \theta_\text{floor})$$

The differential is then applied on top of `effective_base`, so idle thrust
inherits the correct motor split.

---

## 5. The reaction-torque layer

### 5.1 Why a separate `apply_torque` call

`FlightThruster` applies a linear force via `apply_force`.  It does not model
rotational inertia of the propeller or the aerodynamic drag couple that
spinning props produce.  Without an additional torque, the in-simulator yaw
authority would come *only* from the off-centre force differential (which is
weak for a symmetric quad with short arms).

The controller therefore injects the missing angular momentum directly.
Crucially, the magnitude is **derived from the per-motor thrust you already
computed**, not from a separate tuning constant — so yaw authority scales
naturally with throttle:

```gdscript
# inside update_mix, accumulated across motors in the mixing loop:
#   signed_thrust_sum += spin * motor.get_max_force() * motor_thr

body.apply_torque(body.basis.y * prop_torque_coefficient * signed_thrust_sum)
```

This is the direct implementation of the momentum-theory torque model
$\tau_\text{yaw} = \kappa R \sum_i s_i T_i$ from
[flight-dynamics-equations.md](flight-dynamics-equations.md) §5.  The
export `prop_torque_coefficient` *is* $\kappa R$ (in metres).

`body.basis.y` is the drone's own *local* up axis expressed in world space.
This ensures the torque always rotates the drone about its own yaw axis,
regardless of roll or pitch attitude.

### 5.2 Sign convention

Godot's right-hand coordinate system: a **positive** torque about `+Y`
rotates the body **CCW** (left) when viewed from above.

With our spin convention (`spin = +1` CW, `−1` CCW), positive yaw input
raises CCW motors and lowers CW motors, so $\sum s_i T_i$ becomes
*negative*.  Applying `+κR · sum` along `body.basis.y` therefore produces a
−Y torque → CW body rotation from above → "yaw right stick → drone yaws
right" ✓.

Unlike the earlier constant-torque implementation, **no explicit sign flip
is needed** — the sign falls out of the differential RPM split directly.
This also removes the two-layers-fighting class of bug (the differential
loop and the torque layer cannot disagree because they share the same
thrust values).

### 5.3 Tuning `prop_torque_coefficient`

`apply_torque` is called every physics tick — it is an angular *acceleration*,
not an impulse.  The body's angular velocity grows each frame the stick is
held.  With no damping, the drone accelerates indefinitely.

The physical analogue is gyroscopic inertia and aerodynamic drag on the
spinning body.  In Godot this is controlled by `angular_damp` on the
`RigidBody3D` node (Inspector → `RigidBody3D` section).

Real 5-inch quad props have $\kappa R \approx 0.01\text{–0.03}$ m
($\kappa \approx 0.05\text{–0.15}$, $R \approx 0.0635$ m).  Game drones
with unrealistic mass or moment of inertia may need larger values.

**Recommended tuning sequence:**

1. Set `angular_damp` on the `RigidBody3D` to **3.0–5.0**.  This simulates
   the prop-mass gyroscopic resistance real quads have and prevents runaway
   spin.
2. Start `prop_torque_coefficient` at **0.02** (mid of physical range).
3. Raise until yaw rate at full stick feels responsive but not twitchy.
   Because torque now scales with throttle, low-throttle yaw will still
   feel softer than high-throttle yaw — raise `yaw_motor_idle` if low-
   throttle authority is too weak.
4. If the drone continues spinning after releasing the stick, raise
   `angular_damp` rather than lowering `prop_torque_coefficient`.

See [units-reference.md](units-reference.md) §5 for the physics-based
formula that converts a target yaw rate (deg/s) into a value for
`prop_torque_coefficient`.

---

## 6. Integration — attaching to a drone

`FlightQuadController` is a drop-in replacement for `FlightFpsController` on
any drone scene that follows the standard layout.

### 6.1 Scene tree

```
RigidBody3D  (drone body)
├── FlightQuadController   ← attach this script
├── Thruster_FL            ← FlightThruster, positioned at (-arm, 0, -arm)
├── Thruster_FR            ← FlightThruster, positioned at (+arm, 0, -arm)
├── Thruster_RR            ← FlightThruster, positioned at (+arm, 0, +arm)
└── Thruster_RL            ← FlightThruster, positioned at (-arm, 0, +arm)
```

All thrusters should have `force_axis = Vector3.UP` and identical `max_force`
values for a symmetric quad.  Asymmetric `max_force` values will skew the
effective CG of thrust and cause unwanted pitch/roll under throttle.

### 6.2 Inspector exports to set on first use

| Export | Start value | Notes |
|---|---|---|
| `power_authority` | 1.0 | Inherited from parent; collective gain |
| `pitch_authority` | 0.25 | Peak pitch-column delta at full pitch stick |
| `roll_authority` | 0.25 | Peak roll-column delta at full roll stick |
| `yaw_differential` | 0.25 | Peak yaw-column delta at full yaw stick |
| `prop_torque_coefficient` | 0.02 | m (= $\kappa R$); tune with angular drag |
| `yaw_motor_idle` | 0.10 | Idle floor when yawing at zero throttle |
| `angular_drag_pitch` | 0.05 | N·m·s/rad on body-local X (§8) |
| `angular_drag_roll` | 0.05 | N·m·s/rad on body-local Z (§8) |
| `angular_drag_yaw` | 0.25 | N·m·s/rad on body-local Y (§8); ≈ 5× pitch/roll |
| `invert_pitch` | `false` | Toggle if pitch stick produces reversed tilt (§9) |
| `invert_roll` | `false` | Toggle if roll stick produces reversed tilt (§9) |
| `spool_up_time` | 0.08 s | On each `FlightThruster`, not the controller (§7) |
| `spool_down_time` | 0.12 s | On each `FlightThruster`, not the controller (§7) |
| `angular_damp` (on RigidBody3D) | 0.0 | **Set to 0** when using per-axis drag (§8) |

### 6.3 Input map requirements

Inherited from `FlightControllerBase`.  All standard actions apply.
`yaw_left_action` and `yaw_right_action` **must** be bound — without them
`get_axis_value` returns 0 and yaw is non-functional.

Recommended default bindings:

| Action | Binding |
|---|---|
| `yaw_left` | Joypad Left Stick X, negative / `Q` key |
| `yaw_right` | Joypad Left Stick X, positive / `E` key |

---

## 7. Motor spool — PowerRamp

### 7.1 Why a motor can't follow the stick instantly

Real brushless motors have rotational inertia: the rotor + prop has a
moment of inertia of $~10^{-5}$ kg·m², and the ESC can only deliver
finite current, so RPM takes ~50–150 ms to reach a new commanded
setpoint.  Turbines and piston engines are much slower (seconds).

A controller that writes commanded throttle straight into the force
output skips this lag, producing "arcade" feel — the body responds
to stick input on the same tick.  `FlightThruster` therefore
implements a **first-order low-pass filter** on throttle:

$$\alpha = 1 - e^{-\Delta t / \tau}$$

$$\theta \;\leftarrow\; \theta + (\theta_\text{cmd} - \theta) \cdot \alpha$$

where $\theta_\text{cmd}$ is the controller's commanded throttle,
$\theta$ is the *actual* applied throttle this tick, and $\tau$ is
the time constant from the thruster's exports (`spool_up_time` when
$\theta_\text{cmd} > \theta$, `spool_down_time` otherwise).  After
one $\tau$ the filter has closed 63 % of the gap, after 3 $\tau$ it
has closed 95 %.

### 7.2 Asymmetric spool

Real motors spool up and down at different rates.  Spool-up is
limited by ESC current delivery; spool-down is limited by prop
inertia free-wheeling without drive torque.  Defaults reflect this:

| Export (on `FlightThruster`) | Default | Meaning |
|---|---|---|
| `spool_up_time` | 0.08 s | Time constant when throttle is rising |
| `spool_down_time` | 0.12 s | Time constant when throttle is falling |

Set either to 0 to disable ramping in that direction.  Both at 0
recovers the old "instant response" behaviour.

### 7.3 Interaction with yaw reaction torque

Because the reaction-torque layer (§5) reads each thruster's
*actual* (ramped) `throttle` — not the just-commanded value — yaw
authority lags motor RPM in the same way the linear thrust does.
Pilots will feel the correct "wind-up" on yaw input, not an
instant snap to the target angular acceleration.

This is why the sum inside `update_mix` looks like:

```gdscript
var actual_thr: float = motor_thr
if "throttle" in t:
    actual_thr = t.throttle          # ramped, not commanded
signed_thrust_sum += float(spin) * motor_max * actual_thr
```

### 7.4 When to disable PowerRamp

- Testing or unit checks that need deterministic per-tick output —
  set both time constants to 0 on the relevant thruster.
- Direct assignment `thruster.throttle = x` bypasses the ramp and
  snaps both the command and the actual throttle to `x`.  Use for
  hard resets, not for normal flight control.
- Non-motor thrusters (ion drives, force effectors for gameplay
  gadgets) — set both to 0 so they respond instantly.

---

## 8. Per-axis aerodynamic angular drag

### 8.1 Why Godot's `angular_damp` is wrong for drones

`RigidBody3D.angular_damp` applies the same damping coefficient to
all three rotational axes.  Real drones are not isotropic: the four
props present a large flat disc to yaw rotation but only edge-on to
pitch and roll, so yaw aerodynamic damping is typically **3–5× the
pitch/roll damping**.

Symptoms of isotropic damping:

- If you tune `angular_damp` high enough to stop yaw oscillation,
  pitch/roll feel over-damped and "swimmy".
- If you tune it low enough for snappy pitch/roll, yaw will keep
  going after you release the stick.

### 8.2 Implementation

`FlightQuadController` exposes three separate drag coefficients
and applies them each tick as a body-local torque:

$$\boldsymbol{\tau}_\text{drag,local} = -\begin{pmatrix} k_\text{pitch} \cdot \omega_x \\ k_\text{yaw} \cdot \omega_y \\ k_\text{roll} \cdot \omega_z \end{pmatrix}$$

The torque is then rotated into world space via `body.basis` and
handed to `apply_torque`.  Units are **N·m per rad/s**, so a value
of 0.25 means "0.25 N·m of resistance per rad/s of rotation" on
that axis.

| Export | Axis (body-local) | Typical value | Role |
|---|---|---|---|
| `angular_drag_pitch` | X — nose up/down | 0.05 | low (drones pitch easily) |
| `angular_drag_roll` | Z — wings level | 0.05 | low (similar to pitch) |
| `angular_drag_yaw` | Y — heading | 0.25 | high (prop-disc drag) |

### 8.3 Workflow

1. Set `angular_damp` on the `RigidBody3D` to **0**.  This cedes
   full angular damping control to the per-axis values.  (If
   `angular_damp` is non-zero both effects stack — harmless but
   makes the yaw-rate formula in
   [units-reference.md §5.3](units-reference.md) inexact.)
2. Set the three per-axis coefficients.  Start from the table
   above and tune.
3. If the drone "wobbles" after a stick input, raise
   `angular_drag_pitch`/`_roll`.
4. If yaw keeps going after releasing the stick, raise
   `angular_drag_yaw`.

### 8.4 Steady-state yaw rate (revised)

With per-axis drag, the steady-state yaw rate satisfies
$\tau_\text{yaw} = k_\text{yaw} \cdot \omega_\text{yaw}$, so:

$$\omega_\text{yaw,steady} = \frac{\kappa R \sum s_i T_i}{k_\text{yaw}}$$

Inertia no longer appears in the steady-state expression because
the drag torque is modelled as *linear in $\omega$* directly,
rather than as exponential decay of $\omega$ (Godot's
`angular_damp`).  This makes tuning deterministic: the yaw rate is
simply the ratio of driving torque to drag coefficient.

---

## 9. Pilot-orientation inversions

### 9.1 Why sign flips happen on import

`FlightQuadController` assumes the Godot convention: body +X =
right, +Y = up, −Z = forward (camera looks along −Z).  Scenes
imported from modelling tools sometimes land with rotations baked
into the drone mesh or camera — commonly a 180° yaw about Y,
which flips both the visible "right" and the visible "forward".

From the pilot's viewpoint both roll and pitch then appear
inverted, even though the mixing-matrix math is numerically
correct.

### 9.2 The inversion exports

Rather than edit the scene (risking breaking other references),
flip the signs at the controller:

| Export | Effect when `true` |
|---|---|
| `invert_pitch` | Negates the pitch-stick input before mixing |
| `invert_roll` | Negates the roll-stick input before mixing |
| `invert_yaw` (parent) | Negates the yaw-stick input (tilt-rotor legacy; on QuadController, prefer swapping `yaw_left_action`/`yaw_right_action` bindings) |

These live as early as possible in `update_mix`, before any of the
authority scalars or per-motor sign derivations.  They are a
cosmetic layer over the mixer — the underlying sign derivation in
§4.2 is unchanged.

### 9.3 Which fix to use

- **Temporary / single-drone fix:** tick the inversion export(s)
  on the `FlightController` node and move on.
- **Permanent fix:** rotate the imported mesh 180° about Y, remove
  the camera's compensating rotation, leave the inversions `false`.
  This restores agreement with Godot's default convention and
  makes *all* controllers on that drone behave as documented
  without per-scene flags.

---

## 10. Bug catalogue

### 10.1 "Yaw is backwards — spinning left when I push right"

**Cause (most likely):** `spin_sign_override` entries are inverted, or the
drone's motor layout does not follow the X-quad convention.  Use the
override dict to correct individual motors.

**Alternative cause:** all four motors are on a body axis (x or z ≈ 0),
so `_get_spin_sign` defaulted every one to CW.  Check the console for
the "motor … is on a body axis" warning and assign overrides explicitly.

*(The old "`apply_torque` sign mismatch" bug no longer applies — the
torque sign now derives from the thrust split directly; see §5.2.)*

### 10.2 "Yaw feels abrupt / twitchy even at low torque values"

**Cause:** `apply_torque` is called every physics tick with no angular
damping on the body.  The drone accelerates continuously.

**Fix:** raise `angular_drag_yaw` (§8) or `angular_damp` on the
`RigidBody3D` if you haven't switched to per-axis drag yet.  Do not
compensate by reducing `prop_torque_coefficient` to near-zero — that
removes yaw authority in proportion across the whole throttle range,
which the differential alone cannot replace.

### 10.3 "Drone doesn't yaw at all at low throttle"

**Cause A:** `yaw_motor_idle` is 0 and the differential has nothing to act
on.  **Fix:** Set `yaw_motor_idle` to 0.05–0.15.

**Cause B:** `prop_torque_coefficient` is 0 and `yaw_differential` is too
small for the drone's inertia.  **Fix:** raise `prop_torque_coefficient`.

### 10.4 "Drone drifts to one side while yawing"

**Cause:** motor positions are not symmetric about the body centre, so the
throttle differential applies unequal lift to opposite sides.

**Fix:** verify all four motors are at equal ±X, ±Z offsets from the body
origin (local Y offset is fine — it changes lever arm for lift torque but not
the horizontal symmetry check).

### 10.5 "Yaw keeps going after I release the stick"

**Cause:** insufficient angular drag.  The body has accumulated
angular momentum and there is no physical mechanism to shed it.

**Fix:** raise `angular_drag_yaw` (recommended) or `angular_damp` on
the `RigidBody3D`.  Values of 0.2–0.5 for the per-axis yaw drag are
typical for a small quadcopter.  Above ~1.0 the drone begins to feel
over-damped (stops instantly, no inertia feel).

### 10.6 "Roll/pitch stick produces the opposite direction"

**Cause:** the drone's mesh or camera was imported with a 180° yaw
about Y, so body-frame "right" and "forward" are mirrored from the
pilot's viewpoint.

**Fix (cosmetic):** tick `invert_pitch` and/or `invert_roll` on the
controller node — see §9.2.

**Fix (permanent):** rotate the mesh 180° about Y, remove any
compensating rotation from the `Camera3D`, leave the inversions
`false`.  See §9.3.

### 10.7 "Motor response feels instant / arcade-y despite tuning"

**Cause:** `spool_up_time` and `spool_down_time` are both 0 on the
`FlightThruster` nodes (not the controller).  The motors snap to the
commanded throttle without the first-order filter.

**Fix:** set `spool_up_time ≈ 0.08`, `spool_down_time ≈ 0.12` on
each thruster (see §7.2).  Values in the 0.05–0.15 s range feel
"drone-like"; 0.5–1.0 s feels like a turbine.

### 10.8 "Drone tips over at small pitch/roll inputs"

**Cause:** `pitch_authority` or `roll_authority` is too high relative
to the drone's inertia and `angular_drag_pitch`/`_roll`.  Small stick
deflections are saturating the mixer and the drone accumulates
attitude rate faster than it can shed.

**Fix:** lower `pitch_authority`/`roll_authority` first; if the drone
then feels too sluggish, raise `angular_drag_pitch`/`_roll` to bleed
attitude rate more quickly instead.
