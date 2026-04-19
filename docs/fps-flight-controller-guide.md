# FPS Flight Controller — Implementation Guide

> **Audience:** future agents (Copilot, Claude, etc.) and human contributors
> working on raw / direct-input flight controllers in this project — primarily
> [`Drones/Scripts/FlightControllers/FpsFlightController.gd`](../Drones/Scripts/FlightControllers/FpsFlightController.gd)
> and any successor that maps player input directly to thrusters without a PID
> stack.
>
> **Goal:** prevent the class of *input scaling, magnitude, and thrust-to-weight*
> bugs that were worked through getting the FPS controller to feel right. The
> attitude/PID layer has its own document
> ([flight-controller-guide.md](flight-controller-guide.md)) — read that if the
> drone is spinning or refusing to hover. This document covers the specifically
> *non-stabilised* feel layer.
>
> **Prerequisite:** read [thruster-features.md](thruster-features.md) to
> understand the `FlightThruster` model the controller talks to.
>
> **Status:** reflects the working implementation of `FlightFpsController` after
> the magnitude-preservation fix.

---

## 1. The single most important rule

> **The player's analog input is a magnitude, not just a direction. If you
> normalise the input vector, you have *thrown the magnitude away* and every
> non-zero stick deflection becomes full throttle.**

This is the single bug that occupied an entire debug session. It is invisible
to "press to fly" testing — the drone responds to input, so the controller
"works." It only shows up when the player tries to *modulate* throttle (e.g.
hold a hover with the right trigger half-pulled). Every analog control system
in the project — current or future — must preserve player magnitude.

---

## 2. Target behaviour — what "correct" FPS flight feels like

With an analog input device (Xbox/PS pad triggers, gamepad sticks, HOTAS
throttle), the drone must:

1. **Throttle is proportional.** Right trigger at 30% must produce roughly 30%
   of the lift the trigger does at 100%. There must be no non-linearity the
   player did not opt into via curve settings.
2. **Combined inputs do not exceed 100%.** Pulling forward and right
   simultaneously cannot somehow exceed the maximum thrust either axis would
   produce alone. Otherwise diagonal inputs feel "boosted" and cardinals feel
   weak.
3. **Releasing all input cuts engines instantly.** No latched throttle, no
   residual creep. The drone falls under gravity the moment the stick centres.
4. **Feels physical, not scripted.** A heavy drone (low TWR) sluggishly climbs;
   a light drone (high TWR) leaps. The controller should not paper over mass
   with input scaling — the developer should feel mass through the controls.
5. **Direction is in the body's local frame.** Pitching forward sends thrust
   toward whatever the body considers "forward" — including after the body has
   yawed/rolled. The player never has to think in world coordinates.

Failures map to predictable causes:

| Symptom | Almost certainly caused by |
|---|---|
| Trigger at 1% acts like trigger at 100% | Input vector normalised (§3) |
| Trigger feels "on/off", no analog feel | Same as above |
| Diagonal input feels stronger than cardinals | No clamp after summing axes (§3) |
| Drone barely climbs at full throttle | Thrust-to-weight ratio too low (§4) |
| Drone hovers stationary at 50% throttle | TWR ≈ 2.0 — *correct* (§4) |
| Drone keeps thrusting after stick centred | Missing zero-input branch (§5) |

---

## 3. The magnitude-preservation contract

The FPS controller composes a single thrust vector from N input axes:

```
direction = Vector3(strafe, lift, -forward)
```

Each component is in `[-1, 1]` (or `[0, 1]` for analog triggers). The naive
"normalise then scale" pattern is **wrong**:

```gdscript
# ❌ WRONG — discards player magnitude.
var thrust := direction.normalized() * power_authority
```

`direction.normalized()` returns a unit vector for *any* non-zero input. A
player input of `(0, 0.01, 0)` and `(0, 1.0, 0)` both normalise to
`(0, 1, 0)` — the trigger is effectively on/off.

The correct pattern keeps the magnitude and only clamps it so a worst-case
diagonal corner cannot exceed unity:

```gdscript
# ✅ CORRECT — preserves player magnitude, clamps overflow.
var magnitude := minf(direction.length(), 1.0)
var thrust := direction.normalized() * magnitude * power_authority
```

### 3.1 Why both `normalized()` *and* `magnitude`?

`direction.normalized() * magnitude` recovers `direction` itself (clamped).
The pattern is written this way to make the intent clear: "use the input
direction, scaled to the input magnitude, capped at 1.0." Equivalent to:

```gdscript
var thrust := direction.limit_length(1.0) * power_authority
```

`limit_length` is the more idiomatic Godot 4 spelling — both work.

### 3.2 Worst-case input geometry

A simultaneous max-deflection on all three Cartesian axes produces
`Vector3(1, 1, 1)`, length `√3 ≈ 1.73`. Without clamping, the thruster sees
a throttle of 1.73, which `set_thrust` would silently clamp to 1.0 anyway —
**but** the *direction* would be skewed: lift, strafe, and forward all
applied at full throttle simultaneously, instead of "share the budget."
Always clamp the vector length before splitting per-thruster.

### 3.3 Two-axis case (just throttle on a trigger)

When only one axis is non-zero (e.g. trigger pulled, no stick), `length() ==
abs(component)`. The `min(length, 1.0)` is a no-op for triggers (already
`[0, 1]`) and protects symmetric-axis inputs (a stick at +1 plus a trigger
at +0.6 yields length `√1.36 ≈ 1.17` → clamped to 1.0).

---

## 4. Thrust-to-weight ratio (TWR) — what makes a drone "feel right"

The FPS controller has no auto-hover, no anti-gravity term, no PID. Every
newton of lift comes from the player. So the *physics* of the drone alone
determines whether `throttle = 1.0` feels like a hop, a jump, or a launch.

### 4.1 The formula

Net upward acceleration at full throttle:

$$a_\text{up} = \frac{N_\text{thrusters} \cdot F_\text{max}}{m_\text{body}} - g$$

Thrust-to-weight ratio:

$$\text{TWR} = \frac{N_\text{thrusters} \cdot F_\text{max}}{m_\text{body} \cdot g}$$

A **TWR of 1.0** means full throttle exactly cancels gravity — drone hovers
but cannot climb. **TWR < 1.0** means the drone falls even at full power.
**TWR > 1.0** is the only flyable configuration.

### 4.2 Recommended TWR by drone style

| Style | TWR | Feel |
|---|---|---|
| Heavy hauler / cinematic | 1.3 – 1.5 | Sluggish climb, dignified descent |
| Consumer / DJI-like | 1.8 – 2.2 | Brisk climb, hover at ~50% throttle |
| Sport / FPV freestyle | 2.5 – 4.0 | Snappy, hover at ~25-40% throttle |
| Racing | 5.0 – 12.0 | Vertical wall climb, tap-throttle feel |

A real DJI Mavic 3 has TWR ≈ 2.0. A Tinywhoop has TWR ≈ 4. A 5-inch
freestyle quad has TWR ≈ 8-12. Configure for the experience you want.

### 4.3 Tuning recipe

To target a specific TWR:

$$F_\text{max per thruster} = \frac{\text{TWR} \cdot m \cdot g}{N_\text{thrusters}}$$

Example: 1.0 kg drone, 4 thrusters, target TWR = 2.0, gravity = 9.8 m/s²:

$$F_\text{max} = \frac{2.0 \cdot 1.0 \cdot 9.8}{4} = 4.9 \text{ N per thruster}$$

Set every thruster's `max_force` export to ~5 N. The drone now hovers at
50% throttle and accelerates upward at ~9.8 m/s² at full throttle.

### 4.4 Checking your current TWR in the editor

1. Note the drone `RigidBody3D`'s `mass`.
2. Sum every thruster's `max_force`.
3. Divide by `mass × 9.8`.
4. Compare to the table.

If the drone "barely lifts at full throttle," TWR is almost certainly < 1.5.
The fix is **never** to scale `power_authority` above 1.0 (it's clamped) —
it's to raise `max_force` per thruster, or reduce body mass.

---

## 5. The zero-input branch

At the top of `update_mix`, before doing any math:

```gdscript
if direction == Vector3.ZERO:
    _silence_all(thrusters)
    return
```

This is not just an optimisation. Without it, releasing the stick leaves
`thrust = Vector3.ZERO * power_authority = Vector3.ZERO`, which `set_thrust`
correctly interprets as "engine off" — but only if the implementation treats
`length() < epsilon` as zero. Belt-and-braces: emit `Vector3.ZERO` (or call
`set_throttle(0.0)`) explicitly so future thruster implementations cannot
latch the last commanded direction.

The `_silence_all` helper also covers thrusters that lack `set_thrust` and
only support the older `set_throttle` API — keeps the controller forward-
and backward-compatible with the thruster surface.

---

## 6. Input device rules of thumb

This controller relies on the base class's `get_axis_value(neg, pos)` which
returns `_action_strength(pos) - _action_strength(neg)`. `Input.get_action_strength`
returns:

- `0.0` or `1.0` for digital buttons (keyboard / gamepad face buttons),
- continuous `0.0..1.0` for analog axes (triggers, sticks).

### 6.1 Analog triggers (recommended for throttle)

Bind in **Project Settings → Input Map → Add → Joypad Axis**, pick the
trigger by name (e.g. "Right Trigger, Joystick 0 Axis 5" — axis numbers
vary by platform/pad). The action then returns continuous `0..1` and
`get_axis_value` will report it directly.

Pair with the opposite trigger for symmetric throttle (`-1..1`), or leave
the negative action unbound (`&""`) for trigger-only lift (positive only).
Both work — the empty-StringName guard in `_action_strength` ensures unbound
actions contribute zero.

### 6.2 Sticks

Same procedure — bind each stick axis to a Joypad Axis input. A stick
naturally provides bipolar `-1..1` motion, so a single axis usually maps
straight onto a single `get_axis_value(neg=&"", pos=stick_action)` or
similar. If the engine reports values outside `[-1, 1]` (rare; some HOTAS
devices), clamp at the input layer.

### 6.3 Deadzones

Configured per-action in the Input Map (default 0.5, usually too high for
flight — try 0.1). The controller does not need to deadzone again.

### 6.4 Curves & expo

Not implemented in the base controller — and intentionally so. Curve shaping
belongs in a derived class or an input-preprocessing node so the base remains
the linear, "what you push is what you get" reference. If a future controller
adds expo:

```gdscript
var raw := get_axis_value(neg, pos)
var curved := signf(raw) * pow(absf(raw), expo_exponent)
```

`expo_exponent > 1` softens the centre and sharpens the edges (typical FPV
feel); `< 1` does the opposite (sluggish at full deflection — usually wrong).

---

## 7. The bug catalogue — anticipated and observed failures

Forward-looking and historical failures the FPS controller is likely to hit
or has hit during development. Add to it as new failures appear.

### 7.1 "Trigger acts like an on/off switch"

**Cause:** `direction.normalized() * power_authority` — the input magnitude
was thrown away. *(Resolved: the controller now uses `magnitude *
normalized()`.)*

**Fix:** the pattern in §3. Test: log `lift` and `thrust.length()` and
verify they track linearly across the trigger range.

### 7.2 "Drone climbs slowly even at full throttle"

**Cause:** thrust-to-weight ratio too low. See §4. Common in scenes
inherited from the stabilised-controller setup, where `max_force` was tuned
to *hover* (TWR ≈ 1.0) rather than *fly*.

**Fix:** raise per-thruster `max_force` per the §4.3 formula, or reduce
body mass. Do **not** raise `power_authority` above 1.0 (it's clamped).

### 7.3 "Drone won't stop climbing when I release the trigger"

**Cause:** the trigger's input action has a high deadzone, or the analog
binding never returns to exactly zero (some gamepads report `0.001..0.01`
at rest). Result: `direction != Vector3.ZERO`, the silence-all branch never
fires, the controller commands a tiny but non-zero thrust — and *because of
§7.1's old normalise-bug behaviour*, that tiny direction was scaled to full
throttle.

**Fix:** with the §3 magnitude fix in place, the residual is now also tiny
and gravity wins. If you still see it, lower the action's deadzone or add
an explicit deadzone check before the zero comparison:

```gdscript
if direction.length() < 0.02:
    _silence_all(thrusters)
    return
```

### 7.4 "Diagonal input is stronger than cardinal input"

**Cause:** missing `magnitude` clamp; `(1, 1, 0)` sums to `√2 ≈ 1.41` length
with no normalisation. The thruster clamps internally, but the *direction*
remains skewed.

**Fix:** §3.

### 7.5 "Drone tips forward as soon as I throttle up"

**Cause:** thrusters mounted asymmetrically about the body centre. Lift
applied at points off the centre of mass produces torque — and the FPS
controller does not compensate (by design — see §1 of `flight-controller-
guide.md` for why stabilisation belongs in a separate controller).

**Fix:** either (a) reposition thrusters so their lever arms about each
horizontal axis sum to zero, (b) move the body's centre of mass, or (c)
graduate to a stabilised controller. **Never** add a torque-compensation
term to the FPS controller — that defeats its purpose as the raw baseline.

### 7.6 "Trigger value reads negative on rest"

**Cause:** some platforms report triggers as a `-1..1` axis where `-1` is
"released." The Input Map's "Joypad Axis" picker has separate entries for
positive/negative axis halves; choose the one labelled "+" for triggers.

**Fix:** rebind in Project Settings → Input Map. Verify with
`print(Input.get_action_strength(...))` at rest — should read `0.0`.

### 7.7 "Climbing feels like the drone is pushing through water"

**Cause:** the `RigidBody3D` has high `linear_damp` (>= 1.0). Damping
proportionally subtracts from velocity each tick, capping the drone's
top speed regardless of thrust.

**Fix:** drop `linear_damp` to ~0.05 for a snappy drone, ~0.2 for a
floaty one. There is no "air density" knob — damping is the closest
equivalent. Document the chosen value in the drone scene.

### 7.8 "Forward stick sends drone backward (or vice versa)"

**Cause:** sign of the `forward` term in the direction vector does not
match the body's forward axis. Godot's convention is forward = -Z; the
controller writes `Vector3(strafe, lift, -forward)` to map "positive
input → -Z motion."

**Fix:** if the drone's mesh is authored with nose at +Z (common for FBX
imports), either flip the controller's sign or rotate the mesh under the
body. Consistent with `flight-controller-guide.md` §3 — verify the body's
"forward" before signing the input.

---

## 8. The yaw model (tilt-rotor with idle)

> **Status:** reflects the working implementation after the *yaw-feel* pass.
> Covers the choices behind `yaw_tilt_angle`, `yaw_idle_thrust`, and
> `yaw_throttle_attenuation` in `FpsFlightController.gd`.
>
> **Prerequisite:** §3 (magnitude preservation) and
> [AGENTS.md](../AGENTS.md) — specifically *"prefer physical manipulation
> over mathematical equivalence"*. The yaw model is a direct application of
> that rule.

### 8.1 What the code actually does

Yaw is produced by *physically rotating each motor node* around its own
arm axis (the horizontal vector from body centre out to the motor). The
thruster's `force_axis` is local `UP`, so tilting the motor's `Basis`
tilts the thrust vector with it. The controller sets the transform; the
thruster does its normal job. No per-motor vector math is performed in
the controller — it just mutates node transforms and calls `set_thrust`
with one common vector.

All four motors receive the *same signed tilt angle* around their
outward-pointing hinge. Because each hinge points radially outward from
centre, the four tangential pushes reinforce around body-Y (pure yaw
couple) and cancel translationally.

Rest bases are cached on first sight per motor and the tilted basis is
rebuilt fresh from rest every tick, so there is no rotation drift.

### 8.2 Why "mechanical yaw" happens without care

This is a **tilt-rotor** model, not a **quadcopter** model. The
difference matters because:

- A real quad produces yaw from **differential reaction torque** between
  its CW and CCW propellers. Its motors *must* spin to hover, so yaw
  authority is always "hot" — you never notice the coupling to throttle.
- A tilt-rotor produces yaw by **redirecting existing thrust** tangent
  to the arm. With *zero* base thrust (player not commanding throttle,
  pitch, or roll), there is nothing to redirect, and yaw does nothing.

An FPS controller has no hover assist — at rest the thrusters really are
off — so the raw tilt model only yaws *while the player is also
commanding translation*. That coupling is what felt "mechanical."

### 8.3 The idle-thrust fix

`yaw_idle_thrust` raises the `y` component of `base_thrust` to a small
floor whenever the yaw stick is deflected:

```
idle = yaw_idle_thrust * |yaw| * (power_authority * idle_scale)
base_thrust.y = max(base_thrust.y, idle)
```

Key properties (do not break these when refactoring):

- **Only raises the floor.** If the player is already commanding more
  throttle than the idle, their input wins. The idle never fights the
  player's own throttle curve.
- **Scales with `|yaw|`.** A barely-touched yaw stick adds a barely-audible
  idle. Binary "on/off" idle feels like a motor jump.
- **Independent of throttle direction.** Idle only touches `y`; strafe
  and pitch pass through untouched.
- **Set to 0 to disable** and revert to strict-tilt behaviour for
  testing.

Physical interpretation: the ESCs keep the props spinning whenever the
player asks to yaw, so the tilt has airflow/thrust to redirect. This is
a *feel mimic*, not a real-quad yaw model — both real behaviours (prop
idle, reaction torque) happen to feel similar from the pilot's seat.

### 8.4 Throttle attenuation — biasing yaw toward the idle path

With idle thrust alone, yaw authority at full throttle becomes *larger*
than at hover (the same tilt angle redirects a larger vector). That
stacks and over-rotates. `yaw_throttle_attenuation` scales the tilt
angle down as player-commanded translation magnitude rises:

```
yaw_angle *= 1.0 - yaw_throttle_attenuation * magnitude
```

Important: `magnitude` here is the **pre-idle** base-thrust magnitude —
the raw player translation input. Using the post-idle magnitude would
make the yaw idle self-cancel against its own attenuation.

Tuning shape:

- `0.0` — original behaviour; tilt constant regardless of throttle.
- `1.0` — tilt fades to zero at full translation; nearly all yaw
  authority flows through the idle-thrust path.
- `0.6` (default) — tilt at full throttle is 40% of its hover value;
  yaw rate feels consistent across the throttle range.

### 8.5 What **not** to do (yaw-specific)

- **Do not compute a rotated thrust vector and hand it to an
  unrotated motor.** That is mathematically equivalent but violates
  the AGENTS.md directive, hides the mechanism from the editor
  viewport, and makes future additions (gimbal, variable-pitch rotor,
  animation) harder than they should be. Rotate the *motor node*.
- **Do not accumulate rotations tick-over-tick.** Always rebuild each
  motor's basis from its cached rest basis. Applying `rotated()` to
  the previous tick's basis will drift on any non-commutative
  combination of rotations.
- **Do not idle-thrust whenever yaw is non-zero — idle-thrust *scaled
  by `|yaw|`*.** A constant idle at all times is `yaw_idle_thrust > 0`
  behaviour *by itself*, but the scaling is what keeps the stick feel
  proportional.
- **Do not normalise away the yaw axis input.** Same magnitude rule as
  §3 applies — `yaw` must reach the tilt calculation unflattened.
- **Do not try to solve the feel with `power_authority`.** It's a
  ceiling; raising it amplifies *everything* including the coupling
  problem. The three yaw exports are the correct lever.

### 8.6 When the true quad yaw model arrives

The tilt-rotor model is the intended foundation for a later
reaction-torque yaw implementation (tracked separately). When that
lands:

- Each thruster will need a CW/CCW spin group.
- Yaw input will become differential throttle across the groups rather
  than a tilt.
- `yaw_idle_thrust` and `yaw_throttle_attenuation` will likely become
  irrelevant to the new controller and should stay exclusive to this
  FPS controller — keep them as part of the tilt-rotor contract, not
  as shared `FlightControllerBase` state.

Do not retrofit reaction-torque yaw into `FpsFlightController`. Build
a sibling controller (the rule from §11.1 still applies).

---

## 9. Workflow checklist for the next session

- [ ] Have you read this document and `thruster-features.md`?
- [ ] If you're implementing analog input: are you preserving magnitude
      (§3, not normalise-only)?
- [ ] Have you computed the drone's TWR and confirmed it's in the playable
      range for the desired feel (§4)?
- [ ] Does releasing all input return the drone to engine-off (§5)?
- [ ] Have you bound triggers as Joypad Axis (not Joypad Button) and
      verified rest-state reads `0.0` (§6, §7.6)?
- [ ] Are the body's mass, gravity, and `linear_damp` documented in the
      scene so the next agent doesn't tune blind (§7.7)?
- [ ] Are you adding stabilisation (PID, anti-gravity, torque cancel) to
      the FPS controller? **Don't.** Inherit `FlightControllerBase` and
      build a new sibling controller — keep the FPS one as the raw baseline.

---

## 10. Glossary

| Term | Meaning |
|---|---|
| **FPS controller** | Direct-input flight controller — player input maps 1:1 to thrust. No PID, no auto-hover. |
| **Magnitude preservation** | The property that the *length* of the player's input vector survives the math pipeline; trigger at 0.5 stays at 0.5, not snapped to 1.0. |
| **TWR** | Thrust-to-weight ratio. Total thrust at full throttle divided by `mass × g`. |
| **`set_thrust(Vector3)`** | The thruster API the FPS controller targets. Vector length = throttle, direction = local force axis. See [thruster-features.md](thruster-features.md). |
| **`power_authority`** | Inspector ceiling on the magnitude the FPS controller will command. `[0, 1]`. Lowers max throttle without rebuilding thrusters. |
| **Local frame** | The thruster's own coordinate system, which inherits the parent body's orientation. Input is interpreted in this frame so "forward" follows the drone. |

---

## 11. When in doubt

1. **Don't add stabilisation to the FPS controller.** Build a new controller
   that inherits `FlightControllerBase` instead. The FPS controller's value
   is being the unchanged reference baseline.
2. **Don't tune `power_authority` to fix lift problems.** It's a ceiling, not
   a multiplier. Adjust thruster `max_force` or body `mass` instead.
3. **Don't normalise an input vector and call it "smoothing."** You have
   destroyed the player's magnitude. Use `limit_length(1.0)` or the
   `magnitude * normalized()` pattern from §3.
4. **Verify TWR before tuning anything else.** Half the "feel" complaints
   are physics, not controller — and physics is a one-line spreadsheet
   calculation away.
