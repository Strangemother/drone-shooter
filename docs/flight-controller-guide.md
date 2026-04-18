# Drone Flight Controller — Implementation Guide

> **Audience:** future agents (Copilot, Claude, etc.) and human contributors working on the drone flight stack in this project.
>
> **Goal:** prevent the entire class of tuning, sign, and feedback-loop bugs that were worked through during the `FlightStabilisedController` / `FlightWaypointController` development. If you are here because the drone is spinning, flipping, diving, drifting, or refusing to track a waypoint — read this *before* editing gains or adding new controllers.
>
> **Status:** reflects the working implementation of
> [`Drones/Scripts/FlightControllers/StabilisedFlightController.gd`](../Drones/Scripts/FlightControllers/StabilisedFlightController.gd)
> and
> [`Drones/Scripts/FlightControllers/WaypointFlightController.gd`](../Drones/Scripts/FlightControllers/WaypointFlightController.gd).

---

## 1. The single most important rule

> **Before you change any gain, any sign, or any vector operation, write out the sign convention for every axis on paper. Verify that measurement, target, PID output, thruster mix, and physics body frame all use the *same* sign convention.**

Almost every catastrophic failure below (runaway spin, flip, waypoint reverse-tracking) came from one quantity being expressed with the opposite sign from another quantity it was combined with. PID loops are exponential amplifiers of sign errors — a 30° tilt in the wrong direction doesn't stay at 30°, it accelerates.

---

## 2. Target behaviour — what "correct" flight feels like

The drone must:

1. **Hover in place** with no player input, no visible drift, no wobble.
2. **Return to level** smoothly and quickly when disturbed (bumped, pushed, or released from stick).
3. **Accept stick input** as a *target attitude angle* (angle mode), not a raw torque.
4. **Follow waypoints** by tilting toward them, accelerating, and levelling off on arrival.
5. **Never flip** even under full stick deflection, saturated motors, or aggressive waypoint changes.
6. **Feel drone-like** — light, responsive, tilts to move, uses differential thrust not lateral motors.

If any of these are violated, you have either a sign bug (1, 4, 5), a saturation bug (5), or a gain problem (2, 3, 6). Use this document to tell them apart before making changes.

---

## 3. Godot 4 frame & sign conventions (ground truth)

This section is **reference material**. Every other section assumes these are correct.

### 3.1 World axes

Godot uses a **right-handed** coordinate system:

| Axis | Meaning |
|------|---------|
| `+X` | World right |
| `+Y` | World up |
| `+Z` | World "south" / out-of-screen toward the camera in default view |
| `-Z` | World "north" / into-screen / **forward by convention** |

### 3.2 Body axes on a drone

By Godot convention (and matching this project's drone scene), when the drone is level and facing forward:

| Body axis | Direction in body frame | Meaning |
|-----------|--------------------------|---------|
| `+X` | Right wing | Lateral |
| `+Y` | Up (above the drone) | Thrust direction |
| `+Z` | Tail (rear) | Opposite of forward |
| `-Z` | Nose (forward) | Flight direction |

A thruster pointing "up" from the drone's body has `force_axis = Vector3.UP` which is `+Y`.

### 3.3 Rotation sign (right-hand rule)

For a right-handed system, a **positive** rotation about an axis follows the right-hand rule: point thumb along `+axis`, fingers curl in the positive direction.

| Axis | Positive rotation |
|------|-------------------|
| `+X` (right wing) | Nose-**up** (`+Y` tips toward `+Z`, i.e. the nose tips backward — hence positive X-rotation = nose up) |
| `+Y` (up) | Yaw **left** (`+X`/right wing swings toward `-Z`/nose, rotating the body counter-clockwise viewed from above) |
| `+Z` (tail) | Roll **left** (`+X`/right wing swings toward `+Y`/up, i.e. right-wing-up = left-roll) |

**This is subtle and easy to get wrong.** Consequences:

- Drone nose-**down** (flying forward) → **negative** `omega_body.x` → if you want D to oppose nose-down motion, use `-omega_body.x` as the rate.
- Drone rolling **right** (right wing down) → **negative** `omega_body.z` → use `-omega_body.z` as the rate for roll PID.
- Drone yawing **right** (clockwise from above) → **negative** `omega_body.y`.

### 3.4 Project's canonical "positive" conventions

Throughout the controllers we deliberately use:

| Quantity | Positive means |
|----------|----------------|
| `target_pitch` (player input) | Nose-down (stick forward = fly forward) |
| `current_pitch` (measurement) | Nose-down (must match target) |
| `pitch_output` (PID result) | Rear motors push harder → nose-down |
| `target_roll` | Right-wing-down (stick right = roll right) |
| `current_roll` | Right-wing-down (must match target) |
| `roll_output` | Left motors push harder → right-wing-down |
| `wp_pitch_target` (waypoint) | Waypoint is ahead (accelerate forward) |

**All five in each group must match, or the PID goes positive-feedback and the drone flips.**

### 3.5 Deriving `current_pitch` and `current_roll`

The measurement is the direction of world-up expressed in body frame:

```gdscript
var basis   := body.global_transform.basis
var basis_t := basis.transposed()          # orthonormal inverse: world → body
var world_up_body := basis_t * Vector3.UP  # (x, y, z) of world-up in body frame
```

Interpretation of `world_up_body` components:

- `.y` → how much of world-up is still aligned with body-up. Near 1 means level.
- `.z` → if **positive**, body-up is tilted "forward" from world-up, meaning the drone is **nose-down** (body-forward `-Z` tips toward `+Y`).
- `.x` → if **positive**, body-up is tilted "right" from world-up, meaning the drone is rolled with **right-wing-up**. For our convention (positive = right-wing-**down**) we negate this.

Correct formulas in this project:

```gdscript
var current_pitch := atan2(world_up_body.z, world_up_body.y)   # + = nose-down
var current_roll  := atan2(-world_up_body.x, world_up_body.y)  # + = right-wing-down
```

Miss the negation on roll and the drone spins on initial input — this was the bug that caused the "flip + spin" symptom even after pitch was working.

---

## 4. The PID architecture actually used in this project

### 4.1 Angle-mode control

Stick input is interpreted as a **target tilt angle** in radians, clamped by `max_tilt_angle` (default 0.52 ≈ 30°). A PID loop drives the *actual* tilt to match the target. This matches Betaflight's "angle mode" and is what real consumer drones use.

### 4.2 D term = body angular velocity, **not** finite difference

Old approach (removed — caused noise and flip):

```gdscript
var derivative := (error - prev_error) / dt   # DO NOT USE on angle
```

Problems with this approach:

- **Derivative kick:** the moment the stick moves, `target` changes → `error` jumps → derivative spikes → motor commands spike → drone jolts and can flip.
- **Angle wrap:** if an angle ever crosses ±π the finite difference produces a 2π spike.
- **Noise amplification:** any sub-frame measurement jitter is divided by `dt`, producing enormous D values.

Correct approach — read angular velocity directly from Godot's physics engine:

```gdscript
var omega_body := basis_t * body.angular_velocity
# pitch D input: -omega_body.x  (nose-down motion should produce negative; negate so D opposes)
# roll  D input: -omega_body.z  (right-wing-down = negative ω_z; negate to match current_roll sign)
```

Helper: `_pid_rate(target, current, rate, kp, ki, kd, dt, integral)` takes the rate as an explicit parameter and computes `output = kp*error + ki*integral − kd*rate`. No finite difference, no kick, no wrap problems.

### 4.3 Attitude authority clamp

```gdscript
var pitch_output := clampf(raw_output, -attitude_authority, attitude_authority)
```

This is a **safety valve**. It bounds the magnitude of the per-motor attitude differential regardless of how large the PID output gets. It does **not** fix sign bugs (positive feedback saturates the clamp and still flips), but it prevents gain-tuning mistakes from producing runaway accelerations.

**Sizing rule:** `attitude_authority` should be *well above* hover throttle, so the PID can brake hard rotation by pushing motors near saturation. For this project (hover ≈ 17%, TWR ≈ 5.7), `0.6` works. If TWR is higher, raise it.

### 4.4 Airmode mix — the anti-flip safeguard

The naive mix

```gdscript
throttle = clampf(collective + pitch_mix + roll_mix, 0.0, 1.0)
```

has a fatal failure: when any motor clamps to 0 (or 1), the *differential* between motors is destroyed. Example with hover=0.17 and pitch_output=0.6 on a quad:

```
front motor: 0.17 − 0.6 = −0.43 → clamped to 0.0
rear motor:  0.17 + 0.6 =  0.77 → fine
differential: 0.77 − 0.0 = 0.77  (wanted 1.2)
```

The rear motors push hard, the front motors can't push back down, net torque is less than commanded, and the drone keeps rotating past vertical → flips.

**Solution: `_apply_mix()` in `StabilisedFlightController.gd`.** It performs a three-stage process:

1. Compute each motor's attitude contribution `atti[i] = pitch_mix + roll_mix`.
2. If the attitude span `(max_atti − min_atti) > 1.0`, scale all `atti[i]` by `1.0 / span` — preserves ratios between motors.
3. Clamp `collective` into the window `[-min_atti, 1.0 - max_atti]` so no motor is pushed outside [0, 1] *by the collective*.

The net effect: **attitude is preserved at the cost of altitude.** When the drone must choose between "hold altitude" and "don't flip", it chooses "don't flip".

### 4.5 Yaw damping — deliberately absent

The quad in this project has all thrusters firing along body-UP. There is **no way** to produce yaw torque from force-differential mixing alone (all forces share the same axis). Attempts to apply yaw via `body.apply_torque()` proved fragile at low body inertia — they compete with the solver and can oscillate.

**Current solution:** leave yaw to `RigidBody3D.angular_damp`. Set it to ~3.0 on the drone's root body in the editor, and the physics engine damps residual yaw for free.

**If real yaw control is needed later:** add a dedicated yaw actuator (tilting thrusters or reaction wheel), not more `apply_torque` layering.

---

## 5. Frame of reference — body vs world

### 5.1 The rule

> **Everything in the attitude controller lives in the body frame.**
> Position errors, angular velocities, tilt angles, thruster offsets, torque vectors — all body-local.

### 5.2 Why

Thrusters are arranged in body-local positions (a front-left thruster is always front-left regardless of yaw). Their torque contribution is a body-local quantity. If you measure tilt in world frame, pitch/roll cross-couple through yaw the instant the drone faces anywhere other than `-Z`.

### 5.3 World → body conversion

For an orthonormal basis (which RigidBody3D always has), `basis.inverse() == basis.transposed()`. Use `transposed()` — it's cheaper and numerically exact.

```gdscript
var basis_t := body.global_transform.basis.transposed()
var vec_body := basis_t * vec_world
var vec_world := body.global_transform.basis * vec_body
```

### 5.4 Waypoint position error must be body-relative

```gdscript
var error_world := target_pos - body.global_position
var error_body  := basis_t * error_world     # ← critical
```

If you feed `error_world` directly into a pitch/roll PID that works in body frame, "forward" to the PID means "world -Z" not "nose-forward" — so the drone flies toward world-north regardless of where it's facing. This was one of the waypoint bugs.

---

## 6. Thruster force model (what the mix is actually commanding)

Each `FlightThruster` in `Drones/Scripts/DroneThrusterScript.gd`:

- Has a `max_force` (Newtons) and `throttle` in [0, 1].
- Fires along `force_axis` (default `Vector3.UP`, i.e. body-local +Y).
- Applies `force = basis * axis * max_force * throttle` at the thruster's world position (via `apply_force`, which produces a torque automatically if offset from CoM).

Consequences:

1. Motors **cannot push in reverse** (throttle is clamped to [0, 1], not [-1, 1]). This is the source of the saturation problem that airmode solves.
2. Torque comes from *offset between thruster and body CoM*, not from a torque axis. Thrusters must be positioned away from CoM on at least two axes (X and Z) for pitch and roll authority. No offset = no torque = no control.
3. Any thruster not aligned with body-UP produces *unintended* torques. Keep `force_axis = Vector3.UP` unless you're deliberately modelling a tilt-rotor.

---

## 7. Physical parameters — why gains depend on the drone

PID gains are **not universal**. The same controller code with the same gains will behave completely differently on drones of different mass, thrust, or arm length. Always ask / measure:

| Parameter | Symbol | Effect on gains |
|-----------|--------|-----------------|
| Mass | `m` (kg) | Higher mass → higher `pitch_p`, `roll_p` |
| Thrust-to-weight ratio | TWR = `n·F_max / (m·g)` | Higher TWR → **lower** gains (more authority per throttle %) |
| Arm length | `r` (m) | Shorter arms → higher `pitch_p`, `roll_p` |
| Moment of inertia | `I ≈ m·r²` | Higher `I` → higher D gain to achieve the same damping |

### 7.1 Symptoms of wrongly-sized gains

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Violent spin on any input | P too high for a high-TWR drone | Lower `pitch_p`, `roll_p` ~10× |
| Drone sluggish, drifts off level | P too low | Raise `pitch_p`, `roll_p` |
| Oscillates (wobbles) when settling | D too low, or P too high | Raise D first; if still oscillating lower P |
| Flips past vertical on hard input | Not airmode (section 4.4), or D+authority too low | Enable airmode + raise authority |
| Fine at hover, flips during waypoint | Sign bug in waypoint error conversion (sec 5.4) | Check body-frame conversion |
| Spins up unbounded at rest | Sign bug in current_pitch or current_roll | Re-derive per section 3.5 |

### 7.2 The reference tuning

For this project's drone (0.5 kg, 4 × 7 N thrusters, 0.4 m arms, TWR 5.7):

```
pitch_p / roll_p     = 0.4
pitch_i / roll_i     = 0.0  (enable only if persistent lean appears)
pitch_d / roll_d     = 0.35
attitude_authority   = 0.6
max_tilt_angle       = 0.52   (~30°)
alt_p / alt_i / alt_d = 0.4 / 0.02 / 0.5
```

If you scale the drone: roughly, double mass → double `pitch_p`. Halve the arms → quadruple `pitch_p` (because I ∝ r²).

---

## 8. The bug catalogue — what went wrong, and why

This is the list of every real failure mode encountered during development and the root cause of each. Future agents should check this list *first* when a new drone-flight symptom appears.

### 8.1 "Parser Error: Class X hides a global script class"

**Cause:** two `.gd` files both declare `class_name X`. Godot's class registry is flat and global.

**Fix:** rename one with a prefix (e.g. `FlightThruster` instead of duplicating `DroneThrusterScript`). See the rename table in the project history if in doubt.

**Prevention:** before introducing a `class_name`, grep the repo for existing declarations.

### 8.2 Violent spin on activation

**Cause (our first occurrence):** `apply_torque` layered on top of existing force-differential thruster mixing. Two controllers commanded the same axis → fighting → compound spin per tick.

**Fix:** only ever control pitch/roll via one mechanism. In this project that's thruster force mix. `apply_torque` is reserved for axes with no mechanical authority (yaw), or removed entirely in favour of `angular_damp`.

### 8.3 Flip-to-ground on forward input

**Multiple possible causes — work through all three:**

1. **Pitch sign mismatch.** `current_pitch` computed as nose-up positive but `target_pitch` positive-on-forward-stick. Error grows as drone pitches down → PID commands more pitch down → overshoot past vertical. Section 3.5 formula is the correct one.
2. **No airmode.** Front motors clamp to 0, differential dies, PID can't arrest rotation. Section 4.4 fixes this.
3. **Authority too tight.** `attitude_authority` < hover throttle means the PID can't produce differential thrust large enough to oppose gravity-induced rotation. Raise to 0.5–0.7.

### 8.4 Waypoint flies the drone away from the target

**Cause:** position error computed in world frame, fed into a body-frame PID. The drone flies toward world-`-Z` regardless of its actual heading.

**Fix:** `error_body = basis.transposed() * error_world` before feeding into pitch/roll reference. Section 5.4.

### 8.5 Waypoint flies straight into the ground

**Cause:** sign error on the pitch reference derived from body-frame position error. `-error_body.z` commanded nose-up for a forward waypoint.

**Fix:** body-forward is `-Z` so a target ahead has `error_body.z < 0`. Feeding `error_body.z` (without negation) as `current` to `_pid(0, current)` gives output proportional to `−error_body.z`, which is positive for forward targets → correct nose-down command.

### 8.6 Roll axis explodes on waypoint engagement

**Cause:** `current_roll` missing sign negation (positive in math but our convention wants positive = right-wing-down, which is negative-Z rotation). Tiny lateral waypoint error → positive `wp_roll_target` → runaway.

**Fix:** `atan2(-world_up_body.x, world_up_body.y)`. Section 3.5.

### 8.7 Derivative kick on stick movement

**Cause:** D term using `(error − prev_error) / dt`. Moving the stick changes `target` abruptly → huge fake derivative → motor spike → drone jolts and can flip.

**Fix:** D term uses body angular velocity directly (section 4.2), which only responds to *actual* motion.

### 8.8 Drone feels "mushy" and can't hold altitude during manoeuvres

**Cause:** airmode gives up altitude to preserve attitude. This is **by design** — you cannot have both if the motors saturate.

**Fix:** if the drone is losing too much altitude during normal manoeuvres, the drone is under-powered for the task. Raise `max_force` per thruster, or reduce `max_tilt_angle` so less commanded pitch means less attitude demand means more headroom for collective.

### 8.9 Drone bounces forever when trying to settle

**Cause:** P too high or D too low. Classic under-damped response.

**Fix:** raise D (0.35 → 0.5) or lower P (0.4 → 0.3). Never raise both P and I simultaneously while tuning — one at a time.

### 8.10 Integral wind-up after long held input

**Cause:** held stick creates a persistent error; I accumulates beyond usefulness; on release drone overshoots strongly in the opposite direction.

**Fix:** `integral_limit` clamps the stored integral magnitude. Default 0.3. Also: keep `pitch_i / roll_i = 0` unless you observe a steady-state lean.

---

## 9. Workflow for editing flight code — a checklist

If you find yourself modifying attitude logic, go through this list:

- [ ] Before changing anything, have you reproduced the bug and confirmed the symptom matches the section in this doc?
- [ ] Are you about to change a **sign**? Derive it on paper first. Note which of `target`, `current`, `output`, `rate`, `body-frame axis` you're modifying and verify all five remain consistent.
- [ ] Are you adding a new control path (e.g. `apply_torque`)? Check that no existing path (thruster mix) already controls that axis. Never stack two controllers on the same DOF.
- [ ] Are you changing gains? Note the drone's mass, TWR, and arm length. Is this change justified by a change in one of those, or are you guessing?
- [ ] Does the change survive three stress tests: (a) full stick deflection in each direction, (b) rapid stick reversal, (c) waypoint at distance > 20m and angle > 90° from current heading?
- [ ] Did you break airmode? If `_apply_mix` is replaced with naive per-motor clamping, the drone will flip under hard input. This is not optional.
- [ ] Is the waypoint controller still converting position error to body frame?

---

## 10. Reference: concrete mental models

### 10.1 "What does pitch_output = 0.6 actually mean?"

Every motor's commanded throttle gets `± (offset.z / max_arm_z) * pitch_output` added to the collective. For a symmetric quad with arm length 0.4 m:

- Front motors (`offset.z = -0.66`): contribution = `(-0.66 / 0.66) * 0.6 = -0.6`. Motor is asked to *reduce* throttle by 60 percentage points.
- Rear motors (`offset.z = +0.66`): contribution = `+0.6`. Motor is asked to *increase* throttle by 60pp.
- Net: 1.2 spread between front and rear. Produces nose-down torque of roughly `1.2 * F_max * r = 1.2 * 7 * 0.66 ≈ 5.5 N·m`.
- On a body with `I_x ≈ m·r² ≈ 0.5 * 0.66² ≈ 0.22 kg·m²`, that's `α = τ / I ≈ 25 rad/s²`.
- After one 16ms physics tick → `ω ≈ 0.4 rad/s`. After 100 ms → 2.5 rad/s. At 90° (π/2 rad), drone has passed vertical.

This illustrates why D is critical: without damping 2.5 rad/s of angular velocity, the drone *will* overshoot. With `pitch_d = 0.35` the D contribution is `-0.35 * 2.5 = -0.87` — large negative value subtracted from output, braking the rotation.

### 10.2 "Why does the drone lose altitude during waypoint tracking?"

Because airmode prioritises attitude. While commanding a 20° nose-down tilt:
- `pitch_output ≈ 0.16` (rescaled by attitude_authority)
- Attitude span on motors = ~0.32
- Collective windows shrinks: `collective_effective ∈ [0.16, 0.84]`
- If hover demands 0.17 and altitude PID wants to push to 0.3, it *can* — up to 0.84. Fine.
- But if collective demand exceeds 0.84 (drone is below target altitude and rising hard), it clamps. Drone sags until attitude relaxes.

This is correct and expected behaviour. Real FPV quads do this too.

### 10.3 "Why is roll measurement different from pitch measurement?"

Geometry. Body-right is `+X` and body-forward is `-Z`. When the drone rolls right by θ:

- Body-up rotates about `+Z` by `-θ` (right-hand rule: right-wing-down = negative Z rotation).
- In body frame, the world-up vector acquires a `+X` component (it tilts "rightward" from body-up's perspective, because the body has rolled left relative to world-up).
- So `world_up_body.x` is **positive** when we've rolled right. Our convention wants positive = right-wing-down, so we negate.

Pitch:

- Body-forward `-Z`, so pitching nose-down rotates body-up about `+X` by `-θ`.
- World-up in body frame acquires a `-Z` component? No — let's trace: rotating body-up (which was `+Y`) by `-θ` about `+X` gives `(0, cos θ, -sin θ)` in body frame? Wait — rotating the *body* tilts the *world-up vector* the opposite way in body frame. So world-up, when body pitches nose-down (body +Z rises), acquires a `+Z` component in body frame.
- Positive `world_up_body.z` = nose-down. No negation needed. ✅

Do this derivation for every new axis before trusting the signs.

---

## 11. Files involved

| Path | Role |
|------|------|
| [Drones/Scripts/DroneScript.gd](../Drones/Scripts/DroneScript.gd) | Root `RigidBody3D`. Discovers thrusters, finds flight controller, calls `update_mix` each physics tick. |
| [Drones/Scripts/DroneThrusterScript.gd](../Drones/Scripts/DroneThrusterScript.gd) | Individual thruster. Applies force at its world position along `force_axis * throttle * max_force`. |
| [Drones/Scripts/FlightControllers/DroneFlightController.gd](../Drones/Scripts/FlightControllers/DroneFlightController.gd) | Base class `FlightController`. Defines input action names and shared helpers. |
| [Drones/Scripts/FlightControllers/StabilisedFlightController.gd](../Drones/Scripts/FlightControllers/StabilisedFlightController.gd) | `FlightStabilisedController`. Angle-mode PID with airmode mixing. The heart of the stack. |
| [Drones/Scripts/FlightControllers/WaypointFlightController.gd](../Drones/Scripts/FlightControllers/WaypointFlightController.gd) | `FlightWaypointController`. Extends stabilised mode with body-frame position tracking. |
| [Drones/Scripts/FlightControllers/AcroFlightController.gd](../Drones/Scripts/FlightControllers/AcroFlightController.gd) | `FlightAcroController`. Rate-mode (raw torque) — does **not** use airmode or angle PID. |
| [Drones/Scripts/FlightControllers/AngleFlightController.gd](../Drones/Scripts/FlightControllers/AngleFlightController.gd) | `FlightAngleController`. Simpler angle mode without altitude hold. |

---

## 12. Signals a future agent should treat as red flags

When the user reports any of the following, **stop** and consult this doc before editing:

- "it spins"
- "it flips"
- "it flies the wrong way"
- "it worked at hover but broke during X"
- "the waypoint moves it away from the target"
- "fine on one axis, broken on the other"

These almost always map to a sign error or a saturation failure documented above. Jumping straight to changing gains is how the development process ended up with ten iterations instead of two.

---

## 13. When in doubt

1. **Reproduce the bug first.** Don't edit speculatively.
2. **Instrument before patching.** `print("pitch: ", current_pitch, " target: ", target_pitch, " omega_x: ", omega_body.x)` for one tick reveals sign bugs instantly.
3. **Change one thing at a time.** Gains + signs + mix + mode simultaneously is how hours are lost.
4. **Prefer reading physics state over computing it.** `body.angular_velocity` beats finite difference. `body.linear_velocity` beats differencing positions. `body.global_transform.basis` beats manually tracked rotations.
5. **Airmode is not optional** for motorless-torque quadcopter designs.

---

*This document was written after a long and painful debugging session. Its purpose is for no one — human or agent — to repeat that session. If you discover a new failure mode not listed in section 8, add it.*

---

## 14. Extended notes — things the first pass missed

The rest of this document is extension material added after the initial write-up. Sections above are the core curriculum; these are deeper dives on topics that caused real confusion but weren't spelled out.

### 14.1 Physics ticks, `_process` vs `_physics_process`, and why it matters

**Godot runs physics on a fixed timestep** (default 60 Hz). `body.angular_velocity`, `body.linear_velocity`, and `apply_force` only behave deterministically inside `_physics_process`. Calling them from `_process` mixes render-framerate samples with physics-framerate state and produces jittery or FPS-dependent behaviour.

- `DroneScript._physics_process` calls `flight_controller.update_mix()` every physics tick. ✅
- `FlightThruster._physics_process` calls `apply_force` every physics tick. ✅
- **Do not** call `update_mix` from `_process`. Flight controller state assumes fixed `dt`.

The `dt` used for PID integration is `get_physics_process_delta_time()`, not a passed-in `_delta`. This is intentional — mixing the two sources is how integration drift happens.

### 14.2 `can_sleep = false` on the drone body

The drone scene sets `can_sleep = false` on its `RigidBody3D`. This is **required**. If the body sleeps:

- `angular_velocity` reads as zero even if something is trying to rotate it.
- `apply_force` still wakes it, but there's a one-tick delay before the sim is live.
- The PID runs, commands thrust, but the body doesn't respond → apparent control failure on first input.

If you're designing a new drone scene, preserve this setting.

### 14.3 Gravity, `gravity_scale`, and the TWR trap

`RigidBody3D.gravity_scale` multiplies the project's default gravity. The combination that matters is *effective gravity* (`default_gravity * gravity_scale`), which is what `_hover_throttle()` computes with.

If `_hover_throttle()` uses `ProjectSettings.get_setting("physics/3d/default_gravity")` but the body has `gravity_scale != 1.0`, the hover baseline will be **wrong**. Our current implementation does not correct for this — it assumes `gravity_scale = 1.0`.

**Fix for non-unity scales:**

```gdscript
func _hover_throttle(body: RigidBody3D, total_max: float) -> float:
    var g := float(ProjectSettings.get_setting("physics/3d/default_gravity")) * body.gravity_scale
    g = maxf(g - anti_gravity, 0.0)
    return clampf((body.mass * g) / total_max, 0.0, 1.0)
```

If the user reports "drone falls from hover" or "drone rockets upward at rest", check `gravity_scale` and the hover calculation first.

### 14.4 `linear_damp` and `angular_damp` on the RigidBody3D

These are the physics engine's built-in drag, not part of the flight controller. They have two valid uses:

| Setting | Use |
|---------|-----|
| `linear_damp = 0.3–0.5` | Mild air resistance. Makes the drone feel "weighted". Without this the drone slides forever after a push. |
| `angular_damp = 3.0–5.0` | Yaw damping (section 4.5). Also catches any runaway rotation as a safety net, regardless of what the controller is doing. |

Overusing damping produces a mushy, floaty feel — the drone responds sluggishly because every input is fighting invisible drag. Tune these in the editor, not in code.

### 14.5 The `reset_state()` contract

`FlightController.reset_state()` is called by `DroneScript.reset_flight_controller()` after a position reset (spawn, teleport, respawn). Every subclass must:

- Zero all integrators (`_*_integral = 0`)
- Clear all captured set-points (`_target_altitude = NAN`)
- Reset any cached previous-frame state

Failing to implement this → drone respawns with I-term wound up from its pre-respawn lean → immediately pitches hard in one direction. Subtle and easy to miss in testing.

### 14.6 Debug printing — what's actually useful

If a drone misbehaves and you're about to guess at gains, **stop and print**. A single well-chosen `print()` reveals almost every sign bug instantly.

For sign verification (run with the drone at rest, then tilt it manually in the editor):

```gdscript
print("tilt_deg: pitch=%.1f roll=%.1f | omega: x=%.2f y=%.2f z=%.2f | target_p=%.2f target_r=%.2f" % [
    rad_to_deg(current_pitch), rad_to_deg(current_roll),
    omega_body.x, omega_body.y, omega_body.z,
    target_pitch, target_roll
])
```

Expected behaviour:
- Drone level → `current_pitch ≈ 0`, `current_roll ≈ 0`
- Drone nose-down by hand → `current_pitch > 0` (positive = nose-down in our convention)
- Drone right-wing-down by hand → `current_roll > 0`
- Stick forward → `target_pitch > 0` and matches `current_pitch`'s sign after a moment

For PID authority checks:

```gdscript
print("P=%.2f I=%.2f D=%.2f out=%.2f clamped=%.2f" % [
    pitch_p * error, pitch_i * _pitch_integral, -pitch_d * rate,
    raw_output, pitch_output
])
```

If `clamped` is frequently at `±attitude_authority`, the authority cap is being hit — either authority is too low or gains are too high.

### 14.7 `apply_force` vs `apply_central_force` vs `apply_torque`

These are distinct and non-interchangeable:

| Call | What it does |
|------|--------------|
| `apply_force(force, offset)` | Force at a point offset from CoM. Produces both linear acceleration AND torque proportional to `cross(offset, force)`. **This is what thrusters use.** |
| `apply_central_force(force)` | Force at CoM only. No torque. For uniform body acceleration (e.g. player "push" hit). |
| `apply_torque(torque)` | Pure torque, no linear effect. Useful for simulated reaction wheels or designing a yaw actuator. |
| `apply_impulse`/`apply_torque_impulse` | One-shot version of the above, instantaneously adds momentum. For hit reactions. |

The `offset` argument to `apply_force` is a world-space vector from the body CoM to the application point, **not** a local-space thruster offset. `FlightThruster` computes this correctly as `global_position - target_body.global_position`.

### 14.8 Controller hierarchy and extension

```
Node (FlightController)
  ↳ FlightAngleController         (simple angle mode)
  ↳ FlightAcroController          (rate mode, uses apply_torque directly)
  ↳ FlightStabilisedController    (angle mode + PID + airmode + altitude hold)
        ↳ FlightWaypointController (adds body-frame position tracking)
```

Rules for adding a new controller:

1. **Extend `FlightController`** if you need a completely new control strategy (e.g. orbital, formation, tethered).
2. **Extend `FlightStabilisedController`** if you want airmode and PID attitude for free. `WaypointFlightController` is the template.
3. **Override `update_mix(body, thrusters)`** — this is the per-tick entry point.
4. **Override `reset_state()`** — call `super.reset_state()` then zero your own state.
5. **Never override `_physics_process`** on the controller — `DroneScript` drives the call. If you need per-tick logic independent of the body, do it inside `update_mix`.
6. **Always call `super.update_mix(body, thrusters)` when falling back** — `WaypointFlightController` does this when no waypoint is set, which is why the drone hovers cleanly even with the waypoint controller attached.

### 14.9 Why not just use `Basis.get_euler()`?

Because Euler extraction has **gimbal lock**. At ±90° pitch the yaw and roll become degenerate and `get_euler().z` flips wildly. A drone at 89° pitch that rolls through 90° would see its roll measurement jump by 180° → massive fake error → PID spikes.

`atan2(world_up_body.*, world_up_body.y)` avoids this by reading the body-up vector directly. It still degenerates at true vertical (both args tiny), but at that point the drone is upside down and attitude control is moot — the airmode mix and angular damp catch it.

### 14.10 Performance & memory

Typical per-tick cost of `update_mix`:

- ~4 thrusters per drone
- 2 × `atan2`, 1 × basis transpose, ~10 float operations in PIDs
- 1 loop over thrusters (~4 iterations) for `_apply_mix`
- Total: well under 10µs per drone on modern hardware

There is no reason to optimise the controller further unless you're spawning hundreds of drones. If you are, consider:

- Cache `max_arm_x` and `max_arm_z` in `_ready()` rather than recomputing every tick.
- Pool the `PackedFloat32Array` in `_apply_mix` instead of allocating.

Both are trivial; neither matters for < 50 drones.

### 14.11 Things to never do

A partial list of seductive-but-wrong approaches that were tried or considered and rejected:

| Anti-pattern | Why it fails |
|--------------|--------------|
| Finite-difference D on angle | Noise amplification, angle wrap, derivative kick (section 4.2, 8.7) |
| World-frame attitude measurement | Cross-couples through yaw, causes runaway spin when drone isn't facing -Z (section 5.2) |
| `apply_torque` on pitch/roll | Fights the force-differential mix, produces compounding spin (section 8.2) |
| Naive `clampf(collective + mix, 0, 1)` | Motor saturation destroys PID authority, causes flips (section 4.4, 8.3) |
| Raising P without first raising D | Under-damped oscillation or divergence |
| Non-zero I on a drone that already holds level | Wind-up → overshoot on release (section 8.10) |
| Reading `angular_velocity` from `_process` | Samples between physics ticks, not deterministic |
| Using Euler angles (`basis.get_euler()`) for attitude | Gimbal lock (section 14.9) |
| Mixing world and body frame in the same PID loop | Signs diverge with yaw, drone flies in wrong direction |

### 14.12 Suggested stress tests for any new flight code

Before declaring a drone controller "working":

1. **Hover test.** Spawn drone at 10m altitude, zero input. Must stay within 0.2m of spawn position and 2° of level for 30 seconds.
2. **Settling test.** Apply 5 rad/s angular impulse on each axis. Must return to level within 1.5s with no more than one overshoot of 10°.
3. **Full-stick test.** Hold full pitch-forward for 3 seconds, release. Must reach commanded tilt, hold it, and recover to level without flipping.
4. **Reversal test.** Alternate full forward and full backward stick at 2Hz for 10 seconds. Drone should track without flipping; altitude may sag.
5. **Yaw-then-forward test.** Yaw the drone 180° (manually or via `rotate_y`), then apply full forward stick. Drone must fly in its *current* forward direction, not the world's.
6. **Distant waypoint test.** Place waypoint 50m ahead and 20m up. Drone must reach within `arrival_radius` in under 15s without overshooting past the target horizontally.
7. **Angled waypoint test.** Place waypoint 20m away but 135° from current heading. Drone must yaw is absent (acceptable) or reorient, then approach.
8. **Abuse test.** Mid-flight, set `mass` to 2× its spawn value via code. Controller should stabilise at the new hover throttle within 2 seconds (not flip).

If any of these fail, go back to section 8.

---

## 15. Glossary

| Term | Meaning |
|------|---------|
| **Airmode** | A mixing strategy that preserves attitude differential when motors saturate. Sacrifices altitude to keep the drone oriented. |
| **Angle mode** | Stick input represents a target *tilt angle*; PID drives the drone to match. Opposite of rate mode. |
| **Body frame** | Coordinate system attached to the drone's root `RigidBody3D`. Rotates with the drone. |
| **Collective** | The throttle value applied uniformly to all motors (for altitude/lift), before attitude differential is added. |
| **D term / derivative** | The part of a PID response that opposes rate-of-change. In this project, fed from `body.angular_velocity` directly. |
| **Derivative kick** | A spurious D-term spike caused by a sudden target change (stick movement). Eliminated by derivative-on-measurement or explicit-rate input. |
| **Gimbal lock** | Degenerate orientation where Euler angle extraction becomes ambiguous. Avoided by using basis vectors. |
| **I term / integral** | The part of a PID response that accumulates over time to eliminate steady-state offset. Kept at 0 by default here. |
| **Mixer** | The code that translates PID outputs (pitch, roll, yaw, collective) into per-motor throttles. |
| **Moment of inertia** | Rotational equivalent of mass. Determines how much torque is needed to produce a given angular acceleration. |
| **P term / proportional** | The part of a PID response proportional to the current error. The main restoring force in a PID. |
| **Rate mode** | Stick input represents a target *angular rate* (rad/s), not an angle. Used in acro / racing. |
| **TWR** | Thrust-to-weight ratio. Total max thrust divided by weight. Above ~4 means the drone can fly aggressively; below 2 it can barely climb. |
| **Wind-up** | Uncontrolled growth of the I term when the plant can't respond. Clamped with `integral_limit`. |

---

*End of document. Total length is deliberate — the iterative debugging cost far more time than reading this will.*
