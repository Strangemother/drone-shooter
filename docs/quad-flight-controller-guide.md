# Quad Flight Controller — Implementation Guide

> **Audience:** future agents (Copilot, Claude, etc.) and human contributors
> working on the CW/CCW differential-yaw flight controller in this project —
> primarily
> [`Drones/Scripts/FlightControllers/QuadFlightController.gd`](../Drones/Scripts/FlightControllers/QuadFlightController.gd).
>
> **Goal:** document the motor spin model, yaw math, reaction-torque layer, and
> tuning knobs for the quadcopter-style yaw mechanism.  Also catalogue the
> sensitivity and sign issues encountered during initial implementation.
>
> **Prerequisite:** read [fps-flight-controller-guide.md](fps-flight-controller-guide.md)
> first — `FlightQuadController` extends `FlightFpsController` and all
> translation (throttle, pitch, strafe) behaviour is unchanged from that guide.
> Read [units-reference.md](units-reference.md) for physical unit context
> before tuning `yaw_reaction_torque`.
>
> **Status:** reflects the working implementation of `FlightQuadController`
> after the reaction-torque sign fix.

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

## 4. The throttle differential — math

### 4.1 Per-motor throttle formula

Given:
- `base_scalar` — the player's commanded translation throttle ∈ [0, 1]
- `yaw` — the yaw axis input ∈ [−1, 1]
- `yaw_differential` — peak delta ∈ [0, 1] (export)
- `spin` — motor spin sign ∈ {−1, +1}

Each motor's throttle is:

$$\theta_\text{motor} = \text{clamp}\!\left(\theta_\text{base} - \text{yaw} \times \Delta_\text{yaw} \times \text{spin},\; 0,\; 1\right)$$

### 4.2 Sign derivation (yaw right — positive yaw input)

| Motor | spin | Formula | Effect |
|---|---|---|---|
| FL / RR (CW) | +1 | `base − yaw × Δ × (+1)` → lowered | CW motors slow → less CW drag |
| FR / RL (CCW) | −1 | `base − yaw × Δ × (−1)` → raised | CCW motors speed up → more CW drag |

Net prop drag on body: CW > CCW → **body yaws CW (right)** ✓

### 4.3 Budget conservation

The total throttle budget across four motors is conserved when deltas cancel:

$$\sum \theta_i = 4 \theta_\text{base} - \Delta_\text{yaw} \cdot \text{yaw} \cdot \underbrace{\sum \text{spin}}_{ = 0}$$

Because a symmetric quad has two CW and two CCW motors, $\sum \text{spin} = 0$,
so the summed throttle is independent of `yaw`.  This means yaw manoeuvres do
not reduce total lift — matching real-world quadcopter behaviour.

### 4.4 Yaw motor idle floor

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

The `apply_torque` call directly injects the missing angular momentum:

```gdscript
body.apply_torque(body.basis.y * -yaw * yaw_reaction_torque)
```

`body.basis.y` is the drone's own *local* up axis expressed in world space.
This ensures the torque always rotates the drone about its own yaw axis,
regardless of roll or pitch attitude.

### 5.2 Sign convention

Godot's right-hand coordinate system: a **positive** torque about `+Y` rotates
the body **CCW** (left) when viewed from above.

| yaw input | desired rotation | torque sign |
|---|---|---|
| +1 (right) | CW | negative → `−yaw` ✓ |
| −1 (left) | CCW | positive → `−yaw` ✓ |

Hence the negation: `body.basis.y * -yaw * yaw_reaction_torque`.

The differential loop uses the *opposite* convention (positive yaw right is
directly handled by the `spin` term in the formula), so the two layers agree.

### 5.3 Tuning `yaw_reaction_torque`

`apply_torque` is called every physics tick — it is an angular *acceleration*,
not an impulse.  The body's angular velocity grows each frame the stick is
held.  With no damping, the drone accelerates indefinitely.

The physical analogue is gyroscopic inertia and aerodynamic drag on the
spinning body.  In Godot this is controlled by `angular_damp` on the
`RigidBody3D` node (Inspector → `RigidBody3D` section).

**Recommended tuning sequence:**

1. Set `angular_damp` on the `RigidBody3D` to **3.0–5.0**.  This simulates
   the prop-mass gyroscopic resistance real quads have and prevents runaway
   spin.
2. Start `yaw_reaction_torque` at `mass × 2.0` (for a 0.5 kg drone: start
   at ~1 N·m).
3. Raise until yaw rate at full stick feels responsive but not twitchy.
4. If the drone continues spinning after releasing the stick, raise
   `angular_damp` rather than lowering `yaw_reaction_torque`.

See [units-reference.md](units-reference.md) §5 for how to convert between
an observed angular velocity (deg/s) and the torque and damping values
required to produce it.

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
| `power_authority` | 1.0 | Inherited from parent |
| `yaw_differential` | 0.25 | Motor throttle delta at full yaw |
| `yaw_reaction_torque` | `mass × 2` | Tune with `angular_damp` |
| `yaw_motor_idle` | 0.10 | Idle floor when yawing at zero throttle |
| `angular_damp` (on RigidBody3D) | 3.0–5.0 | Set on the *drone body*, not the controller |

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

## 7. Bug catalogue

### 7.1 "Yaw is backwards — spinning left when I push right"

**Cause (most likely):** `apply_torque` sign is mismatched.  Godot's `+Y`
torque is CCW — without negating `yaw`, pushing right produced a CCW rotation.

**Fix:** ensure the torque line reads `body.basis.y * -yaw * yaw_reaction_torque`.
*(Resolved in the current implementation.)*

**Alternative cause:** `spin_sign_override` entries are inverted, or the
drone's motor layout does not follow the X-quad convention.  Use the override
dict to correct individual motors.

### 7.2 "Yaw feels abrupt / twitchy even at low torque values"

**Cause:** `apply_torque` is called every physics tick with no angular
damping on the body.  The drone accelerates continuously.

**Fix:** raise `angular_damp` on the `RigidBody3D` (see §5.3).  Do not
compensate by reducing `yaw_reaction_torque` to near-zero — that removes
low-throttle yaw authority, which the differential alone cannot replace.

### 7.3 "Drone doesn't yaw at all at low throttle"

**Cause A:** `yaw_motor_idle` is 0 and the differential has nothing to act
on.  **Fix:** Set `yaw_motor_idle` to 0.05–0.15.

**Cause B:** `yaw_reaction_torque` is 0 and `yaw_differential` is too small
for the drone's inertia.  **Fix:** raise `yaw_reaction_torque`.

### 7.4 "Drone drifts to one side while yawing"

**Cause:** motor positions are not symmetric about the body centre, so the
throttle differential applies unequal lift to opposite sides.

**Fix:** verify all four motors are at equal ±X, ±Z offsets from the body
origin (local Y offset is fine — it changes lever arm for lift torque but not
the horizontal symmetry check).

### 7.5 "Yaw keeps going after I release the stick"

**Cause:** insufficient `angular_damp`.  The body has accumulated angular
momentum and there is no physical mechanism to shed it.

**Fix:** raise `angular_damp` on the `RigidBody3D`.  Values of 3–8 are
typical for a small quadcopter.  Above ~10 the drone begins to feel
over-damped (stops instantly, no inertia feel).
