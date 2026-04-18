# Waypoint Flight Controller — Implementation Guide

> **Audience:** future agents (Copilot, Claude, etc.) and human contributors working on
> the drone's waypoint-following stack in this project.
>
> **Goal:** prevent the class of *position-tracking, heading, and pacing* bugs that were
> worked through while getting `FlightWaypointController` to actually follow a waypoint.
> The attitude/airmode foundation is covered in
> [flight-controller-guide.md](flight-controller-guide.md); **read that first** if you
> are debugging anything below the waypoint layer (spin, flip, tilt). This document
> assumes attitude control is working.
>
> **Status:** reflects the working implementation of
> [`Drones/Scripts/FlightControllers/WaypointFlightController.gd`](../Drones/Scripts/FlightControllers/WaypointFlightController.gd).

---

## 1. The single most important rule

> **The waypoint controller is a *cascaded* loop that sits on top of the attitude
> controller. Every stage of that cascade has its own sign convention, its own
> reference frame, and its own failure mode. Changing one stage without verifying the
> others is how a "one-line fix" turns into a day-long debug.**

The full cascade, in evaluation order, is:

```
waypoint world-pos  ─►  world error  ─►  body-frame error  ─►  forward-aligned error
                                                                    │
                                                                    ▼
                                                             target velocity
                                                                    │
                                                             (velocity error)
                                                                    │
                                                                    ▼
                                                              tilt target
                                                                    │
                                                        (align-gate by heading)
                                                                    │
                                                                    ▼
                                                   final tilt target (+ player stick)
                                                                    │
                                                                    ▼
                                           FlightStabilisedController attitude PID
                                                                    │
                                                                    ▼
                                                      airmode mix → thrusters
```

In parallel (and *independently*):

```
forward-aligned error (XZ)  ─►  heading error  ─►  yaw PD  ─►  body.apply_torque()
```

**If the drone "flies the wrong way" the bug is almost always before the tilt target.
If the drone "won't stop" or "overshoots" the bug is almost always at target-velocity
or velocity-error. If the drone "rotates forever" the bug is in the yaw loop. Diagnose
by stage, not by gain.**

---

## 2. Target behaviour — what "correct" waypoint tracking feels like

With `set_waypoint(node)` called on a stationary drone in open space, the drone must:

1. **Yaw to face the waypoint** before accelerating toward it — not fly tail-first or
   sideways.
2. **Accelerate smoothly** up to `max_approach_speed`, not lurch forward at saturated
   tilt.
3. **Cruise** at `max_approach_speed` while far from the target, with `wp_pitch ≈ 0`
   and only small correction roll.
4. **Begin braking** at `dist ≈ max_approach_speed / pos_p` out from the waypoint, with
   `wp_pitch` going negative (nose up) to slow the drone.
5. **Arrive** at the waypoint with `vel_body.z → 0`, pitch/roll → 0, and hover at the
   waypoint's altitude. The controller disengages its position loop inside
   `arrival_radius` and becomes equivalent to `FlightStabilisedController` again.
6. **Clearing the waypoint** (`clear_waypoint()`) must return the drone to normal
   stabilised hover with no residual motion or integral wind-up.

If any of these are violated, walk the cascade in section 1 and identify which stage's
output has gone wrong. **Do not** start by changing gains. Gains are the last lever to
pull, not the first.

---

## 3. Reference frames involved, and why they are not interchangeable

The waypoint stack juggles three distinct frames. Conflating them is the single biggest
source of bugs.

### 3.1 World frame

The scene's global coordinate system.
* `waypoint_target.global_position` is in world frame.
* `body.global_position` is in world frame.
* Their difference `error_world` is in world frame.

World-frame vectors mean nothing to a controller that reasons about "ahead of the
drone", because "ahead" depends on the drone's yaw.

### 3.2 Body frame

The drone's local coordinate system, with `+Y` always up relative to the chassis, and
`+X` / `+Z` rotating with the body's yaw.

* `basis_t = body.global_transform.basis.transposed()` converts world → body (the
  basis is orthonormal so transpose = inverse; see
  [flight-controller-guide.md §5.3](flight-controller-guide.md)).
* `error_body = basis_t * error_world` is the waypoint's position *relative to the
  drone's current orientation*.

**In Godot's standard convention, body-forward is `-Z`.** A waypoint ahead of the drone
has `error_body.z < 0`. This is the convention the stabilised controller and
`_apply_mix` assume.

### 3.3 Forward-aligned frame (the waypoint controller's private frame)

Some meshes (FBX exports, models authored "+Z forward") have their visible nose at
body `+Z` instead of body `-Z`. When that happens, `basis_t * error_world` will give
`error_body.z > 0` for a waypoint the player can *see* directly ahead of the drone.

To avoid forking every sign downstream, the waypoint controller introduces a
**forward-aligned frame**: the body frame with X and Z optionally mirrored so that the
drone's *visible* nose is always at `-Z` of the aligned frame.

```gdscript
var fwd_sign: float = -1.0 if invert_forward else 1.0
var err_fwd := Vector3(fwd_sign * error_body.x, error_body.y, fwd_sign * error_body.z)
var vel_fwd := Vector3(fwd_sign * vel_body.x, vel_body.y, fwd_sign * vel_body.z)
```

All the waypoint math below (`target_vel`, `vel_err`, `heading_error`, tilt commands)
is done in this aligned frame. The final `wp_pitch_target` and `wp_roll_target` are
multiplied by `fwd_sign` on the way out so the stabilised controller receives them in
its own native frame and keeps working unmodified.

Y is not mirrored. Altitude doesn't care about yaw-plane mirroring.

### 3.4 Why we don't just "fix the mesh"

Tempting, but:

* The mesh orientation is content-authored and changing it breaks whatever else
  consumes the mesh (other scripts, camera rigs, thruster positions referenced by
  offset).
* The rest of the flight stack — attitude, stick input, thruster mix — is correct
  under the Godot convention and should stay that way.
* The mismatch is *only* a question of how to interpret "the waypoint is in front of
  the drone". A single boolean contains it.

Keep `invert_forward` as the only place this mismatch is expressed.

---

## 4. The cascaded position → velocity controller

### 4.1 The problem with direct P-on-position

The first working waypoint loop used direct P-on-position:

```gdscript
pitch_raw = -pos_p * err_body.z + pos_d * vel_body.z   # DO NOT USE
```

With any reasonable `pos_p` (e.g. 0.15) and a distant waypoint (`err_body.z = -180`),
the P term demands `0.15 × 180 = 27 rad` of tilt. `pos_d * vel_body.z` at 11 m/s is
only `0.325 × 11 = 3.6 rad`. Net demand ~23 rad, clamped to `max_waypoint_tilt`.

**Consequence:** the pitch command is *pinned* at saturation from the moment the
waypoint is engaged until the drone is within roughly `max_waypoint_tilt / pos_p`
metres (~2 m). D never gets a chance to brake because P dominates it by a factor of
~6. The drone accelerates unchecked to whatever speed drag caps it at, then punches
through the target into the far wall.

This failure is not fixable by tuning gains. It's architectural.

### 4.2 The fix: cascade position into a velocity target

The working controller uses the structure real flight controllers use (ArduPilot,
PX4, Betaflight's "position hold" modes):

```gdscript
# Stage 1: position loop produces a *target velocity*, clamped.
var target_vel_z := clampf(pos_p * err_fwd.z, -max_approach_speed, max_approach_speed)
var target_vel_x := clampf(pos_p * err_fwd.x, -max_approach_speed, max_approach_speed)

# Stage 2: velocity loop produces a tilt command.
var vel_err_z := vel_fwd.z - target_vel_z
var vel_err_x := vel_fwd.x - target_vel_x

var pitch_raw :=  pos_d * vel_err_z + pos_i * _pos_z_integral
var roll_raw  := -pos_d * vel_err_x - pos_i * _pos_x_integral
```

Behavioural regimes:

| Distance to target       | `target_vel`           | What the drone does                                        |
|--------------------------|------------------------|------------------------------------------------------------|
| Far (> brake distance)   | Clamped to ±`max_approach_speed` | Tilts to accelerate, then cruises at `max_approach_speed`. |
| Inside brake distance    | Scales down with `dist`| `vel_err` goes negative → pitch reverses → drone brakes.    |
| Inside `arrival_radius`  | —                      | Position loop disengages; hovers.                           |

**Brake distance** is the single most intuitive tuning parameter:

```
brake_distance ≈ max_approach_speed / pos_p
```

At the defaults (`max_approach_speed = 6 m/s`, `pos_p = 0.25`) → 24 m. Lower `pos_p`
for a gentler, earlier brake. Raise for a harder, later stop.

**Tilt gain** is `pos_d`. This is now the only tilt-generating term; name is
historical. Raise for snappier response to velocity errors (acceleration and braking
both). Too high → wobble around the target velocity.

### 4.3 Why D is on *measured velocity*, not finite-differenced error

The same principle as
[flight-controller-guide.md §4.2](flight-controller-guide.md): reading
`body.linear_velocity` directly is noise-free and kick-free.

The first post-cascade implementation used `(err - prev_err) / dt` as the D input.
Symptom from the log:

```
wp_roll=-0.350  →  wp_roll=+0.350  →  wp_roll=-0.350  →  wp_roll=+0.350
```

A sub-metre lateral error jittering by a few millimetres between ticks, divided by
`dt ≈ 1/60`, produced full-saturation roll commands that flipped sign every tick. The
fix is to use `body.linear_velocity` exactly as the attitude loop uses
`body.angular_velocity`.

### 4.4 Sign derivation, step by step

This is the derivation you must check *on paper* before touching any sign.

Conventions (in the forward-aligned frame):
* Body-forward = `-Z`.
* Body-right   = `+X`.
* Positive `wp_pitch_target` = nose-down = accelerate along body `-Z` = forward.
* Positive `wp_roll_target`  = right-wing-down = accelerate along body `+X` = right.

Case A — waypoint 100 m directly ahead, drone at rest:
* `err_fwd.z = -100`, `err_fwd.x = 0`.
* `target_vel_z = clamp(0.25 × -100) = -6` (capped at `-max_approach_speed`). ✓
* `vel_fwd.z = 0` → `vel_err_z = 0 - (-6) = +6`. Drone is 6 m/s *slower than desired*
  in the -Z direction. We need to accelerate forward, i.e. produce +pitch.
* `pitch_raw = +pos_d × 6 = +0.9` → clamp to `+max_waypoint_tilt`. ✓

Case B — cruising at `-6 m/s`, waypoint still ahead:
* `vel_fwd.z = -6`, `target_vel_z = -6` → `vel_err_z = 0`. `pitch_raw = 0`. ✓ (cruise).

Case C — entering brake zone at 12 m out, still cruising `-6`:
* `err_fwd.z = -12`, `target_vel_z = clamp(0.25 × -12) = -3`.
* `vel_err_z = -6 - (-3) = -3`. Drone is *faster than desired* in -Z direction.
* `pitch_raw = +pos_d × -3 = -0.45` → clamp to `-max_waypoint_tilt`. Nose up = brake. ✓

Case D — waypoint directly to the right, drone at rest:
* `err_fwd.x = +50`, `target_vel_x = clamp(0.25 × 50) = +6`.
* `vel_err_x = 0 - 6 = -6`. Drone needs to accelerate in +X.
* For positive roll (right-wing-down) to accelerate in +X, we want `roll_raw > 0`.
* `roll_raw = -pos_d × vel_err_x = -pos_d × -6 = +0.9`. ✓

The leading minus on the roll term is **load-bearing**. If you remove it, roll
command will point the drone's left wing down on a waypoint to the right, which
accelerates left, which makes the error grow, which commands more roll the same way —
positive-feedback runaway.

---

## 5. The yaw loop — the missing piece that caused "flies away"

### 5.1 Why the stabilised controller has no yaw

See [flight-controller-guide.md §4.5](flight-controller-guide.md). Briefly: all
thrusters fire along body-UP, so differential thrust cannot produce yaw torque. The
stabilised controller simply doesn't touch yaw and relies on the body's
`angular_damp` to kill residual spin.

### 5.2 Why waypoint mode *needs* yaw

Without active yaw, if the drone's nose is pointed at world `-Z` but the waypoint is
at world `+X`, the controller will correctly compute `err_body.x = +180, err_body.z ≈ 0`
and command *full right roll*. The drone will fly sideways to the waypoint. This is
mathematically valid and visually horrible.

It gets worse at 180°: if the waypoint is directly behind the drone, `err_body.x ≈ 0`,
`err_body.z ≈ +180`, and the controller commands full *nose-up* pitch — the drone
reverses into the waypoint. This was one of the recurring bug modes.

### 5.3 The yaw controller

```gdscript
heading_error = atan2(err_fwd.x, -err_fwd.z)   # 0 = ahead, ±π = behind, + = yaw right
yaw_torque_local = -yaw_p * heading_error - yaw_d * omega_body.y
yaw_torque_world = basis * Vector3(0, yaw_torque_local, 0)
body.apply_torque(yaw_torque_world)
```

Notes:

* `atan2(err_fwd.x, -err_fwd.z)` returns the angle from body-forward (`-Z`) to the
  horizontal waypoint direction. Positive means "waypoint is to the right" which
  requires yawing right, which in Godot's right-hand convention is a *negative*
  rotation about body `+Y`.
* That's why the formula starts with `-yaw_p`: positive heading error needs negative
  yaw torque.
* The torque is expressed in body-local coordinates (about body `+Y`) but
  `apply_torque` takes world-frame vectors. The `basis * Vector3(0, yaw_torque_local, 0)`
  rotates it into world space.
* `-yaw_d * omega_body.y` damps rotation in either direction.

### 5.4 The `apply_torque` caveat

[flight-controller-guide.md §8.2](flight-controller-guide.md) warns strongly against
`apply_torque`. That warning is about **stacking** `apply_torque` on top of axes that
are already being controlled by thruster-force mixing. Yaw is not such an axis —
nothing else controls it.

The waypoint controller is allowed to use `apply_torque` on yaw because:

1. It is the *only* control path touching yaw.
2. The body has `angular_damp ≈ 3` set in the editor, which damps the response
   naturally.
3. The commanded torque is bounded by `yaw_p × π + yaw_d × ω_max` which on this
   drone is ~5 N·m — small relative to the body's inertia.

**Do not extend this pattern to pitch/roll.** Those are force-mixed and adding a
torque path on top will cause the "violent spin" failure mode documented in
[flight-controller-guide.md §8.2](flight-controller-guide.md).

### 5.5 The yaw-damping trap (lesson from the log)

Initial yaw gains were `yaw_p = 1.5, yaw_d = 0.4`. On a 180° engagement the log
showed:

```
heading_err = 1.95 → -0.10 → -1.18 → -0.96 → -0.15 → 0.40 → 0.43
```

A 2 rad initial error overshooting to -1.18 rad and ringing for several seconds.
Classic under-damped PD — the body has low yaw inertia and `angular_damp = 3` alone
cannot brake a fast-spinning response. Lesson:

> **Yaw D is cheap. Over-damping yaw makes the turn a bit slower; under-damping
> creates a multi-second ringing catastrophe that completely masks whether the rest
> of the controller is working.**

Default was raised to `yaw_d = 1.2` (three times `yaw_p`). Response is critically
damped: turn-and-settle with no overshoot.

### 5.6 Align-gating: don't accelerate before you're facing the target

While the drone is still rotating toward the waypoint, its *current* body-forward
direction is pointed at something that isn't the waypoint. If the pitch/roll loop is
allowed to accelerate in that direction during the turn, the drone flies off-course,
yaw integrates out, but drifts.

The fix is to fade the pitch/roll commands to zero when the heading error is large:

```gdscript
align_factor = clamp(1.0 - abs(heading_error) / yaw_align_tolerance, 0.0, 1.0)
wp_pitch_target *= align_factor
wp_roll_target  *= align_factor
```

With `yaw_align_tolerance = 0.7 rad (~40°)`:
* On-heading (`heading_err ≈ 0`) → `align = 1.0`, full tilt authority.
* 20° off  → `align ≈ 0.5`, half authority.
* 40°+ off → `align = 0.0`, no horizontal acceleration; yaw alone rotates the drone.

This makes engagements from arbitrary orientations look smooth: rotate first,
accelerate once aligned. The yaw loop is unaffected (yaw torque isn't gated by
`align_factor` — only the translation commands are).

---

## 6. Altitude handling during waypoint following

Altitude hold is inherited from `FlightStabilisedController`. The waypoint controller
simply retargets it every tick:

```gdscript
_target_altitude = target_pos.y
```

This produces a continuous "climb to waypoint Y then hold" behaviour. Player
throttle input still layers on top (nudge up/down while tracking). Inside
`arrival_radius` the position loop disengages but the altitude hold keeps running —
the drone hovers at the waypoint's Y.

**Gotcha:** large vertical errors at long range will saturate the altitude PID's
collective demand against airmode's attitude priority. If the drone is 50 m below its
target while cruising at full pitch, the motors will clip and it will climb slowly.
This is correct airmode behaviour (attitude wins over altitude) — if you need faster
climbs, raise thruster `max_force` or reduce `max_waypoint_tilt` so attitude demand
leaves more headroom.

---

## 7. Tuning — speed, aggression, and feel

This is the *most common* edit: "the drone is too smooth, speed it up" or vice versa.
The controller exposes four numbers that matter.

### 7.1 The four dials

| Property              | Default | Units   | What it controls |
|-----------------------|---------|---------|------------------|
| `max_approach_speed`  | 6       | m/s     | Top cruise speed the drone will target. |
| `pos_p`               | 0.25    | 1/s     | Brake aggressiveness; determines brake distance = `max_approach_speed / pos_p`. |
| `pos_d`               | 0.15    | rad / (m/s) | Tilt produced per m/s of velocity error. Drives both acceleration ramp-up and braking stiffness. |
| `max_waypoint_tilt`   | 0.35    | rad     | Hard cap on tilt the waypoint loop can command. ~20°. |

And two secondary:

| Property                | Default | What it controls |
|-------------------------|---------|------------------|
| `arrival_radius`        | 0.3 m   | Where the position loop disengages. Raise if the drone hovers-oscillates near the target. |
| `yaw_align_tolerance`   | 0.7 rad | How off-heading the drone is allowed before pitch/roll are gated off. Lower = stricter align-first. |

### 7.2 "Too smooth / make it faster" recipe

Order matters. Change one at a time and test.

1. **Raise `max_approach_speed`.** This is the cruise cap. Going from 6 → 10 m/s is a
   60% top-speed increase. Recheck braking — brake distance rises proportionally at a
   fixed `pos_p`, so the drone needs more room to stop.
2. **Raise `pos_d`.** This makes the drone accelerate harder from rest (larger
   `vel_err` during ramp-up) and brake harder during arrival. 0.15 → 0.25 noticeably
   punchier. Above ~0.5 the drone starts wobbling about the target velocity.
3. **Raise `max_waypoint_tilt`** only if `pos_d × max_approach_speed > max_waypoint_tilt`
   on ramp-up and the drone saturates pitch. Otherwise it's irrelevant. Cap at ~0.5
   rad (~28°) — beyond that airmode will start trading altitude for attitude badly.
4. **Adjust `pos_p`** *last*, to tune the brake distance. Lower `pos_p` = longer, gentler
   stop; higher = shorter, harder. Hard stops need a correspondingly high `pos_d`.

### 7.3 "Too twitchy / make it smoother" recipe

Reverse of the above, but the #1 quickest win is usually:

1. **Lower `max_approach_speed`.** Slower cruise = more relaxed arrival.
2. **Lower `pos_d`.** Gentler acceleration, less abrupt braking.
3. **Raise `arrival_radius`** if the drone hunts around the target at low speed.

### 7.4 The brake-distance rule of thumb

```
brake_distance = max_approach_speed / pos_p
```

Cross-check at your configured speeds: this should be *at least* twice the physical
stopping distance the drone achieves at `max_approach_speed` with full nose-up tilt.
If brake distance is less, the drone cannot decelerate in time and will overshoot.

For this project's drone (0.5 kg, 4 × 7 N thrusters, TWR 5.7) at `max_tilt_angle =
0.52` rad:
* Horizontal acceleration at full tilt ≈ `g × sin(0.52) ≈ 5 m/s²`.
* Stopping from 6 m/s ≈ `6 / 5 = 1.2 s` × average 3 m/s ≈ 3.6 m.
* Brake distance at defaults ≈ 24 m.
* 24 / 3.6 = 6.6× margin. Comfortable.

At `max_approach_speed = 20 m/s`, stopping distance would be ~40 m. Defaults would
give brake distance 80 m — still safe. At `max_approach_speed = 20 m/s` *and* `pos_p
= 1.0`, brake distance is 20 m and the drone *will* overshoot. Always check.

### 7.5 Yaw gains are mostly independent

`yaw_p` / `yaw_d` / `yaw_align_tolerance` do not interact with position-loop tuning.
Set them once to get smooth engagements and leave them.

* Yaw ringing on engagement → raise `yaw_d`.
* Yaw overshoot of small heading errors → raise `yaw_d`.
* Yaw slow to turn (drone drifts while turning) → raise `yaw_p` or, better, raise
  `yaw_align_tolerance` so translation is allowed earlier.
* Drone flies sideways during turns → lower `yaw_align_tolerance`.

---

## 8. The bug catalogue — what went wrong, and why

Check this list *first* when any waypoint-specific symptom appears. Attitude bugs
belong in [flight-controller-guide.md §8](flight-controller-guide.md).

### 8.1 "Drone flies tail-first" / "Drone flies sideways to the waypoint"

**Cause:** no yaw control. The position loop has faithfully decomposed the waypoint
error into body axes and commanded the drone to accelerate toward it, without ever
changing the drone's heading. A waypoint behind the drone becomes "accelerate in body
`+Z` = reverse", and one to the right becomes "accelerate in body `+X` = strafe".

**Fix:** enable the yaw-to-waypoint loop. See §5.

**Prevention:** the debug print shows `heading_err`. If this is large (> 0.5 rad) and
the drone is already moving, yaw isn't doing its job.

### 8.2 "Drone rotates 180° and flies away from a waypoint placed in front"

**Cause:** mesh authored with `+Z forward` instead of `-Z forward`. The controller
sees the waypoint at `err_body.z > 0` (behind) for a waypoint the user visually placed
in front, and correctly decides to fly tail-first.

**Fix:** `invert_forward = true`. See §3.3.

**Prevention:** when adding a new drone model, test with a close waypoint directly in
the drone's visible front. If the drone turns away from it, flip `invert_forward`.

### 8.3 "Drone blasts past the waypoint at cruise speed"

**Cause:** direct P-on-position without velocity cascade; or cascade present but
`pos_p` so small that `brake_distance` exceeds the scene.

**Fix:** confirm the cascade is in place (§4.2). Recompute `brake_distance =
max_approach_speed / pos_p`. Raise `pos_p` until the drone begins decelerating at
least 2× its raw stopping distance from the target.

**Prevention:** whenever you change `max_approach_speed`, recheck brake distance.

### 8.4 "Roll command oscillates ±max every tick"

**Cause:** D term using finite-differenced position error. Sub-frame jitter divided
by `dt` produces enormous D output with random sign.

**Fix:** use `body.linear_velocity` as the D input, not `(err - prev_err) / dt`. See
§4.3.

**Prevention:** there is exactly one correct source for the velocity-loop rate:
`body.linear_velocity` (rotated into the aligned body frame). Reach for
`body.linear_velocity` *before* you reach for finite differences.

### 8.5 "Yaw overshoots 180° engagement and rings for 5 seconds"

**Cause:** `yaw_d` too low relative to `yaw_p`.

**Fix:** raise `yaw_d` to at least `yaw_p`; `yaw_d = 1.2, yaw_p = 1.2` is a safe
starting point on this project's drone.

**Prevention:** test every new drone with a 180° engagement. If heading_err
oscillates through zero even once, crank yaw D until it doesn't.

### 8.6 "Drone flies toward the waypoint but doesn't slow down in time / misses it
by 20 m"

**Cause:** `pos_d` too low — velocity-loop D cannot produce enough reverse tilt to
brake the drone during the brake zone. Saturates at `-max_waypoint_tilt` and the
drone decelerates at a fixed rate, which may not be enough to stop.

**Fix:** raise `pos_d`, or lower `max_approach_speed`, or raise `max_waypoint_tilt`.
Preferred order is the first.

**Prevention:** see §7.4's brake-distance cross-check.

### 8.7 "Drone hovers forever inside `arrival_radius` instead of stopping"

**Cause:** `arrival_radius` too small for the drone's approach-speed × reaction time.
The drone overshoots the radius in the direction of travel, the position loop
re-engages on the other side of the target, it turns around, overshoots the radius
again, forever.

**Fix:** raise `arrival_radius` to at least `max_approach_speed × physics_dt × 2` ≈
20 cm at 6 m/s and 60 Hz. 0.3 m is fine at defaults; consider 0.5 m if the drone
hunts.

**Prevention:** the log inside the arrival radius shows pitch/roll oscillating sign
with `dist < arrival_radius`. That's the hunt signature.

### 8.8 "Drone flies perfectly, but for 'too long'"

**Cause:** the waypoint is much further away than it looks. The scene's default
camera rig / visual scale can make distances hard to read by eye.

**Fix:** from the script that calls `set_waypoint`, print
`(wp.global_position - drone.global_position).length()` before the call. If
it's big, either move the waypoint or raise `max_approach_speed`.

**Prevention:** add a temporary `print()` of distances when any "doesn't seem to
work" report comes in. Confirm perception matches world state before touching the
controller.

This is the bug that caused the longest misdiagnosis in this document's history.
Always confirm the waypoint's real-world distance first.

### 8.9 "Drone lurches violently when `set_waypoint` is called"

**Cause:** integral state from a previous waypoint / previous manoeuvre is still
wound up. `pos_i` × stale `_pos_*_integral` produces instant tilt at engagement.

**Fix:** `set_waypoint(target)` zeros the position integrals. Confirm no path into
the controller bypasses this (e.g. direct writes to `waypoint_target`).

**Prevention:** `set_waypoint` and `reset_state` are the only sanctioned entry points
for activating the waypoint loop. Never write `waypoint_target` from outside.

### 8.10 "Heading loop fights the stabilised controller"

**Cause:** a second script also applying yaw torque (e.g. a turret-like aiming
controller, a player yaw stick passthrough stacked on top).

**Fix:** only one control path on any DOF. The waypoint controller's yaw loop is
allowed; other yaw paths must be disabled whenever `yaw_to_waypoint = true`.

**Prevention:** grep the project for `apply_torque` before adding another control
layer. See [flight-controller-guide.md §8.2](flight-controller-guide.md).

---

## 9. Workflow for editing the waypoint controller — a checklist

- [ ] Have you read [flight-controller-guide.md](flight-controller-guide.md) in full?
      Most "waypoint" bugs are actually attitude-stage bugs exposed by waypoint
      engagement.
- [ ] Before changing any code, have you turned on `debug_print` and captured a log
      of the failure?
- [ ] Does the log show the controller reaching the stage you think is failing? (e.g.
      a "doesn't brake" bug should show a large negative `err_fwd.z` with
      `target_vel_z` saturating — not the controller not being called at all.)
- [ ] Are you about to change a sign? Redo the derivation in §4.4 for the axis you're
      editing. Verify `target_vel`, `vel_err`, and tilt output signs all match.
- [ ] Are you about to change yaw gains? Check the ringing signature (heading_err
      oscillating through zero) *before* concluding yaw is the problem.
- [ ] Are you about to add `apply_torque` on a new axis? Re-read §5.4. Don't.
- [ ] Did you confirm the *real* distance to the waypoint? §8.8 exists for a reason.
- [ ] Have you tested (a) a 180° engagement from rest, (b) a lateral waypoint, (c) a
      waypoint placed at the same position as the drone (null case), (d) clearing
      the waypoint mid-flight?

---

## 10. Reference: the `debug_print` log format

When `debug_print = true`, the controller prints this line ~2× per second:

```
[waypoint] err_fwd=(...) vel_fwd=(...) target_vel=(tx,?,tz) heading_err=h align=a wp_pitch=p wp_roll=r dist=d
```

Interpreting fields:

| Field          | Expected values (approx)                             |
|----------------|------------------------------------------------------|
| `err_fwd.x`    | Small (±1 m) during cruise, grows during turns.      |
| `err_fwd.z`    | Large negative = waypoint ahead (aligned frame).     |
| `err_fwd.y`    | Altitude delta; drops to ~0 as drone reaches target altitude. |
| `vel_fwd.z`    | Ramps from 0 to `-max_approach_speed` during cruise.  |
| `vel_fwd.x`    | Small (±1 m/s) during straight flight.                |
| `target_vel_z` | Clamped to `-max_approach_speed` far, ramps to 0 near.|
| `heading_err`  | Large (±π) on engagement, → 0 within ~1 s.            |
| `align`        | 0 during initial turn, → 1 once heading locked.       |
| `wp_pitch`     | Positive during ramp-up, ~0 at cruise, negative in brake zone, ~0 on arrival. |
| `wp_roll`      | Small (±0.05) except during turns or lateral corrections. |
| `dist`         | Monotonically decreasing toward `arrival_radius`.     |

If any field deviates strongly from these expectations, the bug is usually at that
stage of the cascade (§1). **Stop and investigate that stage** rather than
speculatively changing gains.

### 10.1 Signatures of specific bugs in the log

| Log signature                                                | Diagnosis                                 | See   |
|--------------------------------------------------------------|-------------------------------------------|-------|
| `wp_pitch` saturated (`±0.35`) the entire flight.            | Direct P-on-position, or `pos_d` too low. | §4    |
| `wp_roll` flipping between `+0.35` / `-0.35` every sample.   | Finite-differenced D.                     | §4.3  |
| `heading_err` oscillating through 0 at engagement.           | Yaw D too low.                            | §5.5  |
| `heading_err ≈ ±π` staying flat, `wp_pitch` negative large.  | Mesh `+Z` forward with `invert_forward=false`. | §3.3 |
| `dist` decreasing *very slowly*, `wp_pitch ≈ 0`, long-range. | Everything is fine, waypoint is just far. | §8.8  |
| `dist` bouncing around < `arrival_radius`.                   | Arrival radius too small.                 | §8.7  |
| `vel_fwd.z` continues to grow after passing `max_approach_speed`. | `pos_d` too low — velocity loop can't hold target. | §7.2 |

---

## 11. Files involved

| Path                                                                                             | Role |
|--------------------------------------------------------------------------------------------------|------|
| [Drones/Scripts/FlightControllers/WaypointFlightController.gd](../Drones/Scripts/FlightControllers/WaypointFlightController.gd) | The controller itself. Inherits from `FlightStabilisedController`. |
| [Drones/Scripts/FlightControllers/StabilisedFlightController.gd](../Drones/Scripts/FlightControllers/StabilisedFlightController.gd) | Attitude PID + airmode mix. Consumes the tilt targets this controller produces. |
| [Drones/Scripts/DroneScript.gd](../Drones/Scripts/DroneScript.gd)                                | Drone root; forwards `set_waypoint` / `clear_waypoint` calls into the flight controller. |
| [addons/Maps/Scripts/template_map_scene.gd](../addons/Maps/Scripts/template_map_scene.gd)        | Reference consumer — binds key `P` to `set_waypoint`. |

---

## 12. Signals a future agent should treat as red flags

When the user reports any of the following, **stop** and consult this doc before
editing:

- "the drone flies past the waypoint"
- "the drone flies away from the waypoint"
- "the drone spins forever / rotates the wrong way"
- "it flies fine manually but the waypoint doesn't work"
- "it's too fast / too slow" — still consult this doc; tuning has an order (§7.2).
- "it tracked fine at 5m but not at 50m"

These almost always map to one of the bugs in §8.

---

## 13. When in doubt

1. **Reproduce the bug with `debug_print = true`.** One log line beats ten guesses.
2. **Diagnose by cascade stage, not by gain.** §1's flowchart is the map.
3. **Verify the waypoint's real-world distance first.** §8.8.
4. **One change at a time.** Cascade stages interact; simultaneous edits are
   untraceable.
5. **Trust the cascade, tune the dials.** If the architecture is right, the four
   numbers in §7.1 are all you need.
6. **Prefer reading physics state over computing it.** `body.linear_velocity` and
   `body.angular_velocity` are your first-class inputs. Finite differences of
   positions or angles are a last resort.

---

*This document was written after the "drone flies away from the waypoint" debugging
session. Its purpose is for no one — human or agent — to repeat that session. If you
discover a new waypoint-stage failure mode not listed in section 8, add it.*
