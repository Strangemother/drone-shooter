# Ground Effect

Per-thruster in-ground-effect (IGE) lift augmentation, implemented in
[DroneThrusterScript.gd](../Drones/Scripts/DroneThrusterScript.gd) and
wired through the `FootRayCast3D` child in
[thruster_collision_shape.tscn](../Drones/Scenes/thruster_collision_shape.tscn).

Each thruster measures its own height above the terrain with a
downward-facing `RayCast3D` and multiplies its output thrust by a boost
factor that grows as the motor approaches the ground. Because every
thruster measures independently, off-centre motors receive different
multipliers — tilting the drone toward the ground produces a stronger
push on the lower motor, which is the self-righting "cushion" a real
quad feels during landing.

## Why it exists

A rotor close to the ground pushes against air that cannot escape
downward as freely as it would in free flight. The recirculation
compresses a thin high-pressure cushion under the disc, raising
effective thrust by 10–30 % in the last rotor-radius or so of
altitude. Game-feel-wise this gives:

+ visible buoyancy on landing and takeoff,
+ automatic self-levelling when descending (low motor pushes harder),
+ a characteristic "soft touchdown" feel that's absent from naive
  propulsion models,
+ a clear cue that the drone is near something underneath — useful
  when the camera can't see the ground.

## The model

Cheeseman–Bennett, the standard closed-form approximation used in
helicopter aerodynamics:

$$
\frac{T_{\mathrm{IGE}}}{T_{\mathrm{OGE}}}
= \frac{1}{1 - \left(\dfrac{R}{4h}\right)^{2}}
$$

where

+ $T_{\mathrm{OGE}}$ — out-of-ground-effect thrust (normal flight),
+ $T_{\mathrm{IGE}}$ — in-ground-effect thrust (boosted),
+ $R$ — effective rotor radius (the `ground_effect_radius` export),
+ $h$ — measured height from the motor to the ground hit point.

Sampled values for $R = 0.3$ m:

| $h$ | $R/4h$ | Multiplier |
|---|---|---|
| 2.0 m | 0.0375 | 1.0014 |
| 1.0 m | 0.075 | 1.0057 |
| 0.6 m ≈ 2R | 0.125 | 1.016 |
| 0.3 m ≈ R | 0.25 | 1.067 |
| 0.15 m ≈ R/2 | 0.5 | 1.333 |

The formula diverges as $h \to R$, so the implementation clamps to
`ground_effect_max` before returning. A zero-radius divide is
prevented by treating any $h$ below `R / 4` as instantly capped.

### Overdrive: gain knob

The raw formula is gentle at realistic game altitudes. A gain term
lets developers amplify just the *boost* portion while keeping the
"no effect" floor at exactly 1.0:

$$
M_{\mathrm{final}}
= \operatorname{clamp}\bigl(1 + g \cdot (M_{\mathrm{raw}} - 1),\ 1,\ M_{\max}\bigr)
$$

Setting $g = 1$ gives physically-accurate Cheeseman–Bennett.
Higher values exaggerate the cushion for visible testing or
arcade feel. $g = 0$ disables the effect entirely.

## Architecture

```
Thruster (Node3D, FlightThruster script)
├── MeshInstance3D           — cosmetic prop disc
└── FootRayCast3D            — downward ray, hit tested each tick
```

The thruster's own `global_transform.basis` rotates the local
`force_axis` into world space each `_physics_process` — so thrust
follows the drone's current orientation for free, with no per-node
maths. The ground-effect ray is a sibling of the mesh and rotates
with the thruster node, meaning the ray always points "down from the
motor's own perspective". For a level quad that's world-down; for a
steeply banked drone it shortens horizontally. This is correct: a
hovering quad banked 45° over a cliff edge *shouldn't* feel full
ground effect, because the rotor is no longer parallel to the ground
beneath it.

Measurement is the **vertical** (world-Y) offset from the thruster's
`global_position` to the ray's collision point.  This matches $z$ in
the Cheeseman–Bennett formula — defined as hub height above the
ground plane, not slant distance along the ray.  Using vertical
height makes the cushion strength independent of body tilt, so a
rolled or pitched drone at the same *altitude* feels the same
ground effect regardless of ray angle.

## Exports

All on [DroneThrusterScript.gd](../Drones/Scripts/DroneThrusterScript.gd),
grouped under the standard thruster exports:

| Export | Units | Default | Meaning |
|---|---|---|---|
| `ground_effect_enabled` | bool | `true` | Master switch. Off → multiplier always 1. |
| `ground_ray_path` | NodePath | `FootRayCast3D` | Child RayCast3D used for height measurement. |
| `ground_effect_radius` | m | `0.3` | Rotor radius $R$ in Cheeseman–Bennett. |
| `ground_effect_max` | × | `1.5` | Hard cap on the multiplier. |
| `ground_effect_gain` | × | `1.0` | Boost amplifier; 1 = physical, >1 = overdriven. |
| `ground_effect_debug` | bool | `false` | Per-tick print of ray state, h, multiplier. |

The ray itself is configured inside
[thruster_collision_shape.tscn](../Drones/Scenes/thruster_collision_shape.tscn):

+ `target_position = (0, -6, 0)` — 6 m of detection reach.
+ `collision_mask` — must match the ground layer (see gotcha below).
+ `enabled = true` — otherwise `is_colliding()` is permanently false.

## Tuning recipes

### Physically realistic (default)

```
ground_effect_radius = 0.3
ground_effect_gain   = 1.0
ground_effect_max    = 1.5
```

You will barely feel it. That's real life — ground effect is subtle.

### Perceptible cushion (recommended for this game)

```
ground_effect_radius = 0.6
ground_effect_gain   = 3.0
ground_effect_max    = 2.0
```

+ Cushion reaches up to ~1.2 m.
+ ~1.5× thrust at 30 cm, 2× near touchdown.
+ Self-righting torque on tilted descent feels natural.

### Overdriven dev/testing cushion

```
ground_effect_radius = 2.0
ground_effect_gain   = 10.0
ground_effect_max    = 10.0
```

+ Cushion reaches high, obviously buoyant below ~3 m.
+ Use to confirm the system is wired and responding.
+ Dial back before shipping — the drone becomes uncontrollably floaty.

## Development log and bug catalogue

The path from "draft implementation" to "working ground effect" hit
several issues worth recording.

### 1. Nested `RigidBody3D` in the thruster sub-scene

**Symptom:** drone spiralled elastically on roll/yaw inputs, motors
appeared not to respect current body rotation.

**Cause:** the thruster sub-scene initially contained
`Node3D → RigidBody3D → CollisionShape3D`. Instancing this under
`DroneScene` (itself a `RigidBody3D`) produced four nested rigid
bodies inside the drone's body. Nested `RigidBody3D`s are undefined
behaviour in Godot — they push each other around via contact.

Additionally, `DroneThrusterScript._enter_tree()` registers into the
`vehicle_thrusters` group; with a script attached to both the outer
`Node3D` and the inner `CollisionShape3D`, the drone had **eight**
registered thrusters instead of four, half of which targeted the
inner rigid body (via `NodePath("..")`).

**Fix:** flatten to `Node3D → CollisionShape3D` or
`Node3D → [mesh, FootRayCast3D]` with the drone's own collision
shape as a direct child of `DroneScene`. One script per arm, on the
`Node3D` root. Nested `CollisionShape3D`s still contribute to their
nearest `CollisionObject3D` ancestor through any number of plain
`Node3D`s — the editor warning is cosmetic, not a runtime rule.

### 2. Asymmetric pitch/roll response

**Symptom:** roll felt far more responsive than pitch at equal
authority values.

**Cause:** initial motor positions `(±1.1, 0.25, ±0.66)` gave a
1.1 m roll arm against a small roll inertia (narrow body) but only
a 0.66 m pitch arm against a much larger pitch inertia (long body).

$$
\text{angular accel} \propto \frac{\text{arm}}{I}
$$

gave roll ~4× more responsive than pitch.

**Fix:** move the motors to a symmetric layout
`(±1.1, 0.25, ±1.1)`. If keeping an asymmetric layout is
intentional (racing quad feel), compensate with asymmetric
`pitch_authority` / `roll_authority` values in the
`FlightController` inspector.

### 3. Ray never hit the ground — silent null effect

**Symptom:** ground effect felt like it did nothing even at
`gain = 10`, `max = 10`. Stepping through the code looked fine.

**Cause:** two stacked issues.

First, the `RayCast3D` had its default `collision_mask = 1`, but the
ground CSG in the scene was on a different collision layer. The ray
exited the drone's box from beneath and carried on to infinity
without ever calling `is_colliding() == true`.

Second, the node had been renamed from `RayCast3D` (script default)
to `FootRayCast3D`, so even after fixing the mask, the script
couldn't locate it and fell through the `_ground_ray == null`
branch.

**Fix (diagnostic):** the `ground_effect_debug` export prints one of

+ `no RayCast3D at <path>` — path mismatch,
+ `ray not colliding (enabled=… mask=…)` — disabled or wrong mask,
+ `h=… raw=… boosted=… mult=… hit=…` — working, read the numbers.

**Fix (causes):** set `ground_ray_path` to the actual child name
(now `FootRayCast3D` by default) and set the ray's `collision_mask`
to match the ground layer. Godot's `RayCast3D.collision_mask`
default of 1 is a silent trap — worth remembering for future
sensors.

### 4. `ground_effect_max` being interpreted as distance

**Symptom:** cranking `ground_effect_max` to its top value produced
no visible change.

**Cause:** confusion between the cap (a unitless multiplier, e.g.
1.5×) and a distance. The cap only limits the *output* multiplier;
if the raw formula isn't being evaluated at all (see bug #3), the
cap has nothing to act on.

**Fix (documentation):** clearly label both `ground_effect_max` and
`ground_effect_gain` as unitless multipliers in the export tooltips,
and steer users toward `ground_effect_radius` when they want the
cushion to *reach further* (that's a distance).

## Future extensions

Replacing the single downward ray with a richer sensor is the
natural next step. The public API of
`_compute_ground_effect_multiplier()` can stay the same; only its
internals need to change.

+ **Multi-ray sampling.** Four rays at the corners of the rotor
  disc, averaged. Handles thin pipes / edges that a single centre
  ray misses. Still cheap.
+ **Cone of influence.** `ShapeCast3D` with a short cylinder or
  cone, using the closest contact. More robust over uneven terrain.
+ **Material-sensitive cushion.** Strong over concrete, weaker over
  grass, near-zero over water mist. Read the hit collider's
  `physics_material_override` or a custom `ground_type` script.
+ **Translating-flight attenuation.** Real ground effect weakens
  once the aircraft is moving horizontally faster than a rotor
  diameter per second. Scale the multiplier by
  `1 / (1 + k · v_horizontal)` where `k ≈ 1 / R`.
+ **Pressure-wall interaction.** Same formula, same ray, but
  pointing at nearby walls for "wall cushion" when flying in
  corridors — common in FPV drone cinematography.

## Related documents

+ [thruster-features.md](thruster-features.md) — general thruster
  architecture and PowerRamp.
+ [flight-dynamics-equations.md](flight-dynamics-equations.md) — the
  underlying equations of motion; §5/§10 cover mixing and reaction
  torque, where ground-effect-modified thrust still plugs in cleanly.
+ [quad-flight-controller-guide.md](quad-flight-controller-guide.md) —
  the mixer that commands each thruster; ground effect is applied
  *after* the mix and is invisible to the controller.
