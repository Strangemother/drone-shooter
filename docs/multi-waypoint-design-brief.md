# Multi-Waypoint Path Following — Design Brief

> **Audience:** the next session's agent (and future human contributors) picking up
> multi-waypoint flight for the drone.
>
> **Status:** design brief. No code in the repo yet. This document captures the
> architecture, order of operations, and pitfalls discussed at the end of the
> single-waypoint session so the next session can start from the right foot.
>
> **Prerequisites:** read these first.
> - [flight-controller-guide.md](flight-controller-guide.md) — attitude control.
> - [waypoint-controller-guide.md](waypoint-controller-guide.md) — single-waypoint
>   following, the controller this brief builds *on top of*.
>
> **Non-goal:** this brief does **not** modify
> [`FlightWaypointController`](../Drones/Scripts/FlightControllers/WaypointFlightController.gd).
> That controller's one job is "fly to the Node3D I'm currently told to fly to"
> and it stays that way. All multi-waypoint logic lives in a new node.

---

## 1. The single most important rule

> **Keep the flight controller ignorant of paths. It follows one `Node3D` target
> at a time. The "path walker" is a separate node that *moves* the target.**

Every gameplay feature (auto-drone, racing, patrol, cinematic flythrough) decomposes
into the same question: "which single point should the drone be chasing *right now*?"
A dedicated path-walker node answers that question; the existing flight controller
just does the flying. This separation is why the staged plan in §5 is cheap — each
stage changes only *how the virtual target moves*, not the controller.

---

## 2. Target behaviour — what "correct" path following feels like

Given an ordered list of waypoints A → B → C → D, the drone must:

1. **Not stop at each waypoint.** Hard arrival at every point looks like a bus, not a
   drone. The drone should sweep an arc through each interior waypoint.
2. **Slow for tight corners.** A 90° corner at cruise speed is physically impossible
   given the controller's bank-angle limits — the drone needs to pre-brake or it
   overshoots and spirals. A smooth-flying drone slows proportionally to path
   curvature.
3. **Feel responsive**, not dragged. The drone should look like it's choosing to
   fly the path, not being yanked by an invisible tether.
4. **Handle the degenerate cases** gracefully: empty list, single waypoint, duplicate
   points, path cleared mid-flight, drone starting past the first point, a U-turn
   (two waypoints back-to-back that require 180° of heading change).
5. **Emit meaningful signals** — `waypoint_reached(index)`, `path_completed`,
   `path_cancelled`. Gameplay code shouldn't have to poll.
6. **Be authorable.** A level designer should be able to place the path in the
   editor, visually, without writing script — at least for the common case.

If any of these fail in testing, consult §8.

---

## 3. The three architectures and why we pick A first

The single-session analysis came out to three genuinely different path storage
options. Each has a clean migration path to the next.

### 3.1 Option A — List of Node3D in the scene

Store waypoints as real scene nodes under a parent. The walker iterates its
children.

| Pros | Cons |
|------|------|
| Edit visually in the editor. Snap to geometry. | Scene clutter at high waypoint counts. |
| Moving/animating a waypoint node Just Works — the existing `set_waypoint` tracks any live `Node3D`. | Hard to generate at runtime (random courses, procedural patrols). |
| Attach metadata as children (triggers, VFX). | Paths can't be shared between drones without duplication. |
| Debug-friendly: you can *see* the path. | — |

**Best for:** hand-authored tracks, tutorials, fixed mission layouts. This is the
starting point.

### 3.2 Option B — `Resource`-based `WaypointPath`

A custom `Resource` (`.tres`) holding `Array[Vector3]` plus optional per-point
metadata (`speed_override`, `yaw_override`, `tolerance`, `is_loop`). The walker
consumes a resource; no scene nodes required.

| Pros | Cons |
|------|------|
| Reusable across drones and scenes. | Not visible in the 3D viewport without a `@tool` gizmo. |
| Clean version control (small text files). | World-space positions — loses "snap to moving elevator" without extra machinery. |
| Easy procedural generation. | More indirection than option A for simple tasks. |
| Small and fast. | — |

**Best for:** reusable patrol patterns, procedurally generated paths, mission
design in a separate tool.

### 3.3 Option C — `Path3D` / `Curve3D`

Use Godot's built-in spline. Sample points along it; drive the drone by curve
parameter.

| Pros | Cons |
|------|------|
| Curve editor built in. | Conceptually a *curve*, not a *list*. "Did I pass waypoint 3" becomes "did the parameter cross 0.45". |
| Naturally smooth between points. | Less intuitive for "fly to nearest enemy". |
| Free tangent / curvature vectors. | — |
| Same concept as cameras, pickups. | — |

**Best for:** race tracks, cinematic flythroughs, anything where the *shape* of
the path matters more than individual waypoints.

### 3.4 Migration story

All three can share a walker that exposes a single `VirtualTarget : Node3D` whose
position is updated each frame. The *source of motion* changes; the consumer
(the flight controller) never notices. Start with A, add B support when you need
reusable paths, reach for C only when you need true spline smoothness.

---

## 4. Two techniques you need; neither is bezier

The ask was "swap waypoints early so the drone doesn't stop at corners, then maybe
bezier through them for smooth turns." Both are good instincts but the standard
solution is simpler and more robust.

### 4.1 Early-swap — necessary but insufficient

Swap the current target as soon as the drone is within `swap_radius` of it, rather
than waiting to arrive. This breaks the stop-at-every-point behaviour.

Alone, this produces *straight-line segments with cut corners*. The drone flies
straight toward A, then as it gets close, starts flying toward B. There's a
discontinuity in commanded direction at the swap — visible as a tiny jink.

### 4.2 Lookahead (pure pursuit) — the 80%-smoothness-for-5%-complexity trick

Instead of giving the controller the *current* waypoint, give it a **virtual target
that moves continuously along the path ahead of the drone**:

```
1. Maintain a scalar `s` = arc-length progress along the path.
2. Each tick: advance `s` by (current_cruise_speed * dt).
3. Move the VirtualTarget Node3D to the point at arc-length (s + lookahead_distance).
4. The drone chases the VirtualTarget — which is always smoothly ahead of it.
```

Properties:

- No "swap" event — the target sweeps smoothly around corners.
- One tuneable: `lookahead_distance`. Longer = smoother but cuts corners more.
- Robust to the drone lagging behind (it just picks up where it is).
- Works identically on straight lines, splines, and dense waypoint lists.

This is called **pure pursuit** in robotics. It's what most AAA game AI uses for
vehicles because it degrades gracefully under speed and orientation changes.

### 4.3 Swap + lookahead compose

You don't choose between them — you use both when it helps:

- Sparse waypoints + swap-only → straight segments, cut corners, visible jinks.
- Sparse waypoints + dense resampling + lookahead → smooth path. No swap needed.
- Dense waypoints + lookahead alone → smooth path. No swap needed.
- Spline + arc-length parameterisation + lookahead → smoothest option. No swap.

**The right choice for your system is lookahead over a densely-sampled path.** Swap
is implicit: when `s` crosses a waypoint's arc-length position, that waypoint is
"done".

### 4.4 Why not bezier-through-the-waypoints?

Cubic bezier **does not pass through its control points** (only the endpoints). If
your waypoints are *authoritative* ("fly through this door"), bezier drags the
path off them. That's usually wrong for gameplay.

Two better choices when you want real spline smoothing:

| Method | Passes through points? | Smoothness | Godot support |
|--------|------------------------|------------|---------------|
| **Catmull-Rom** | Yes | C¹ (continuous tangent) | Manual but trivial math. |
| **`Curve3D` with in/out handles** | At control points, yes | C² if handles are symmetric | Built in (`set_point_in`, `set_point_out`). |

For your case, `Curve3D` with auto-computed handles (set them to the average
direction of incoming/outgoing segments) gives you Catmull-Rom-equivalent behaviour
with Godot doing the sampling. That's stage 3 of the plan below.

### 4.5 Speed-along-curve is not what you think

A parametric curve `r(t)`, `t ∈ [0,1]` does **not** have constant speed when `t`
is advanced linearly. Tight corners have high `|dr/dt|`, straights have low.
Advancing `t` at a fixed rate produces *slower* motion in tight corners, which is
the opposite of what a drone wants.

**The solution is arc-length parameterisation.** `Curve3D.sample_baked(offset)`
gives it to you for free — `offset` is in metres along the curve, not a parametric
`t`. Use the baked sampler, never the parametric one.

### 4.6 Curvature-based speed (bonus)

Once you're arc-length-parameterised, you can slow the drone in tight corners:

```
kappa = |sample_baked(s + eps).normalized() - sample_baked(s - eps).normalized()| / (2*eps)
target_speed = max_cruise / (1 + kappa_weight * kappa)
```

Five lines. Makes the drone look like a pilot reading the track. Optional — add
it only if tight turns feel wrong.

---

## 5. Staged plan — build it in this order

Each stage is independently shippable, independently testable, and independently
reversible. Resist the temptation to jump to stage 3 on day one. The early stages
reveal controller quirks (especially in stage 1) that are much easier to diagnose
against a simple baseline than inside a spline system.

### Stage 1 — Straight lines + early swap

**Goal:** multi-waypoint flight works end to end, even if corners are jinky.

- `WaypointPathFollower` node: `@export var waypoints: Array[NodePath]` + index.
- Each physics tick: if no current target, set one; if current target is within
  `swap_radius`, advance index; if end reached, emit `path_completed`.
- Uses `drone.set_waypoint(waypoints[index])` exactly as a human would from code.
- Exposes `bind_drone(drone)`, `clear_path()`, `restart()`.

This stage gives you: pathological-case testing, signal plumbing, editor workflow,
and a baseline for every subsequent visual comparison.

### Stage 2 — Virtual target with lookahead (straight-segment path)

**Goal:** corners become smooth; the drone stops jinking at swap points.

- Compute a polyline from the Node3D positions. Precompute cumulative arc lengths
  per segment.
- Maintain scalar `s`. Each tick: `s += cruise_speed * dt`.
- `VirtualTarget.global_position = sample_polyline(s + lookahead_distance)`.
- `drone.set_waypoint(VirtualTarget)` once at bind time — never again.

Corners are now *rounded* because the virtual target sweeps the corner while the
drone is still approaching it. No code change in the flight controller.

### Stage 3 — `Curve3D`-backed path (smooth interpolation)

**Goal:** the path itself becomes curved, not just the drone's pursuit of it.

- Build a `Curve3D` from the waypoints' positions, optionally with handles.
- Replace the polyline sampler with `curve.sample_baked(s + lookahead_distance)`.
- Everything else unchanged.

If you use `@tool` on the path scene, Godot's editor will draw the `Path3D`
visualisation for free — no custom gizmo code.

### Stage 4 — Curvature-limited speed

**Goal:** the drone self-limits in tight corners.

- Sample curvature at the lookahead point (or a short window ahead).
- Scale `cruise_speed` down proportionally.
- Now the drone slows for corners and accelerates out, like a pilot.

### Stage 5 — Per-waypoint overrides

**Goal:** "stop here and face that door" kinds of scripted moments.

- `WaypointPath` resource with `Array[Waypoint]` where `Waypoint` holds position,
  optional `yaw`, optional `speed`, optional `wait_time`.
- Follower reads per-point metadata when `s` crosses each original control point's
  arc-length.
- The flight controller's `align_factor` gate already handles "rotate before
  flying"; to support *arrival-yaw* you'd need a new input on the flight
  controller (a target yaw), which is the one controller-side change this whole
  system needs — and it's localised. Defer until a design actually needs it.

---

## 6. Architecture — concrete node structure

```
TestScene
 ├── DroneScene            (existing RigidBody3D)
 │    └── FlightController  (existing FlightWaypointController)
 └── WaypointPath           (new scene/node)
      ├── Point0   (Node3D)
      ├── Point1   (Node3D)
      ├── Point2   (Node3D)
      ├── ...
      ├── VirtualTarget     (Node3D, internal — the node the drone actually follows)
      └── Follower          (new script, attached to WaypointPath)
```

`Follower` responsibilities:

- Discover its `Point*` children (or read an assigned `WaypointPath` resource).
- Own the `VirtualTarget` Node3D.
- Run in `_physics_process`:
  1. Advance `s` by `cruise_speed * dt`.
  2. Move `VirtualTarget` to the path position at `s + lookahead_distance`.
  3. Detect waypoint-crossed events, emit signals.
  4. At end-of-path, either loop, ping-pong, hold, or emit `path_completed`
     depending on export setting.
- `bind_drone(drone)` calls `drone.set_waypoint(VirtualTarget)` exactly once.
- `clear_path()` calls `drone.clear_waypoint()` and halts.

**The flight controller does not change.** This is non-negotiable — it's how the
stages stay cheap.

### 6.1 Signals to emit

```gdscript
signal waypoint_reached(index: int, waypoint: Node3D)
signal path_started
signal path_completed
signal path_looped         # end reached, restarting
signal path_cancelled      # clear_path() called mid-flight
```

### 6.2 Exports to consider

| Export | Type | Default | Purpose |
|--------|------|---------|---------|
| `waypoints` | `Array[NodePath]` | `[]` | Option A source. |
| `path_resource` | `WaypointPath` | `null` | Option B source (later). |
| `cruise_speed` | `float` | `6.0` | Arc-length advance rate. |
| `lookahead_distance` | `float` | `2.0` | Pure pursuit lookahead. |
| `loop_mode` | enum `NONE / LOOP / PING_PONG` | `NONE` | End-of-path behaviour. |
| `auto_bind_drone` | `NodePath` | `""` | Optional — bind at `_ready`. |
| `swap_radius` | `float` | `2.0` | Stage 1 only. Discard in stage 2+. |

### 6.3 Separation of concerns

```
WaypointPathFollower     ← "which point should the drone chase right now?"
FlightWaypointController ← "how do I fly to the point I'm currently chasing?"
FlightStabilisedController ← "how do I hold attitude while flying?"
```

Each tier knows nothing about the tier above it. Each tier can be replaced
independently. This is the architectural pay-off of keeping the controller
single-waypoint.

---

## 7. Decision points the next session should settle upfront

These need to be answered before coding, because changing them later requires
reworking state.

### 7.1 What counts as "arrived" at the last waypoint?

Options:
1. `dist < arrival_radius` (classic). Risk: the drone enters the radius at speed
   and exits the other side — brief "arrived" flicker followed by hover.
2. `dist < radius AND speed < threshold` (proper stop). Safer but slower.
3. `s >= path_length` (arc-length based). Never flickers. Recommended when using
   the lookahead approach — it aligns naturally with how the virtual target
   moves.

Recommended: (3) for the path-ended event; (1) for per-waypoint-reached events
during traversal.

### 7.2 What happens when the path is cleared mid-flight?

- Must call `drone.clear_waypoint()` — otherwise the drone chases a stale
  `VirtualTarget`.
- Should the drone hover in place, or return to some safe state? Depends on game
  design. Default to "hover in place" (the existing single-waypoint-cleared
  behaviour).

### 7.3 What happens if `cruise_speed` exceeds the drone's physical max speed?

The `VirtualTarget` runs away from the drone; lookahead grows unbounded; the
drone lags further and further behind. Options:

1. Clamp `s` so `VirtualTarget` never gets more than `max_lag` metres ahead of
   the drone.
2. Advance `s` by `min(cruise_speed, drone_actual_speed * 1.1) * dt` — the
   virtual target leads by a bounded factor.

Recommended: (2). It's self-tuning and produces natural-looking behaviour when
the drone is disturbed or under-powered.

### 7.4 Yaw at arrival

- Default: tangent-follow (the drone faces the direction of travel). Cheap —
  give the flight controller a lookahead-plus-extra target for yaw computation.
- Override: per-waypoint `target_yaw`. Stage 5 only.

### 7.5 Multiple drones on one path

Option A waypoints are shared nodes — fine for multiple drones. Each drone
needs its own `s` (progress scalar) though, which means each drone needs its
own `Follower` instance (or the Follower indexes `s` by drone).

Simplest architecture: one Follower per drone. Reuse the same waypoints.

---

## 8. The bug catalogue — anticipated failure modes

This is a *forward-looking* catalogue — bugs this system is likely to hit
during implementation, based on first-principles analysis. Add to it as real
failures appear.

### 8.1 "Drone drives past the final waypoint and flies away"

**Cause:** `VirtualTarget.s` kept advancing past path length; path wraps or resets
silently.

**Fix:** clamp `s ≤ path_length` in stage 1-4. In stage 5, add explicit loop
mode handling. Test empty and single-point paths (section 7).

### 8.2 "Drone lags further and further behind the virtual target"

**Cause:** `cruise_speed` too high for the drone's physical capability (or for
its current gains). Virtual target outruns the drone. Lookahead balloons.

**Fix:** see §7.3. Clamp `s` advance to a bounded multiple of
`drone.linear_velocity.length()`.

### 8.3 "Drone jinks at every waypoint"

**Cause:** stage 1 implementation. Straight-line segments, discrete target swap.

**Fix:** advance to stage 2.

### 8.4 "Drone 'cheats' and cuts corners too aggressively"

**Cause:** `lookahead_distance` too large relative to waypoint spacing. The
virtual target skips ahead to waypoint N+1 before the drone has come close to
waypoint N.

**Fix:** shrink `lookahead_distance` (rule of thumb: `~cruise_speed * 0.3 s`).
Or: space waypoints further apart. Or: use curvature-limited speed (stage 4).

### 8.5 "Drone slows inexplicably on long straights"

**Cause:** using parametric `t` instead of baked arc length on a `Curve3D`.

**Fix:** use `curve.sample_baked(offset)`, never `curve.sample(t)`. See §4.5.

### 8.6 "Drone flies to the correct points but in the wrong order"

**Cause:** `waypoints` export is `Array[NodePath]` but the order resolves to
tree-traversal order, not array order. Or the child-discovery code uses
`get_children()` without sorting.

**Fix:** sort explicitly by name, by export array index, or by a per-point
`@export var sequence_index: int`. Document which.

### 8.7 "Drone doesn't start moving when `bind_drone` is called"

**Cause:** `set_waypoint` passed the `WaypointPath` node itself instead of the
child `VirtualTarget`.

**Fix:** always bind the internal `VirtualTarget` — never an external node. The
`VirtualTarget` is the integration point.

### 8.8 "Drone gets stuck at one waypoint forever"

**Cause:** waypoint tagged as an "arrival" point (wait / stop) but the follower's
resume condition never fires. Or duplicate waypoints — the drone arrives, index
advances, lands on the same position, decides it's already arrived, advances
again, ad infinitum — or worse, one waypoint is *behind* the drone so distance
grows.

**Fix:** test the degenerate cases (§2.4). Reject duplicate consecutive
waypoints, or treat them as a single point.

### 8.9 "Path follower restarts at distance zero after clear"

**Cause:** `clear_path` didn't reset `s`. Next `bind_drone` picks up at the
old `s`.

**Fix:** `clear_path` zeroes `s` and clears the drone's waypoint. `restart()`
zeroes `s` explicitly.

### 8.10 "Moving waypoint parents warp the virtual target"

**Cause:** in stage 1 the flight controller follows `waypoints[i]` directly —
if the waypoint moves, the drone tracks it, which is correct. In stage 2+,
the virtual target is computed from *current* waypoint positions each tick. If a
waypoint jumps, the polyline jumps, `s` doesn't change, but the sampled point
does — producing an apparent teleport.

**Fix:** decide whether your waypoints are static (bake arc lengths at bind
time) or dynamic (recompute each tick). Dynamic is slower but correct; static
is faster but breaks if a waypoint moves. Document either way.

---

## 9. Workflow checklist for the next session

- [ ] Have you read `waypoint-controller-guide.md` and `flight-controller-guide.md`?
- [ ] Are you clear on §1 — the flight controller stays single-waypoint?
- [ ] Which stage (§5) are you targeting? Start at stage 1 even if the goal is
      stage 3 — the baseline is needed for comparison.
- [ ] Have you sorted the decisions in §7 with the user?
- [ ] Are you testing the degenerate cases from §2.4 *this stage*, not "later"?
- [ ] Is every change additive? Nothing in `Drones/Scripts/FlightControllers/` should
      change for stages 1–4. (Stage 5's target-yaw input is the sole exception.)
- [ ] Are the signals in §6.1 hooked up before you add gameplay consumers?

---

## 10. Glossary

| Term | Meaning |
|------|---------|
| **Waypoint** | An authored point in space the path should honour. |
| **Path** | The ordered set of waypoints plus the interpolation between them. |
| **Virtual target** | The `Node3D` the flight controller actually chases. Its position is computed, not authored. |
| **`s` (arc-length progress)** | Distance travelled along the path from its start. Units: metres. |
| **Lookahead** | The distance beyond `s` at which the virtual target is placed. Controls pursuit smoothness. |
| **Early swap** | Changing the flight controller's target to the next waypoint before the drone has arrived at the current one. Obsoleted by lookahead. |
| **Pure pursuit** | The algorithm: follow a virtual target that moves continuously ahead of you along the path. |
| **Arc-length parameterisation** | Advancing along a curve by metres rather than by parametric `t`. `Curve3D.sample_baked` provides this. |
| **Tangent-follow** | The drone's yaw tracks the direction of travel (the path's tangent vector). |

---

## 11. When in doubt

1. **Don't modify the flight controller.** If you need to, stop and confirm with
   the user first. 95% of the time the answer is "move the logic to the
   walker".
2. **Prefer sampling over simulating.** `Curve3D.sample_baked(s)` beats any
   hand-rolled interpolation for paths.
3. **One source of truth for `s`.** If two parts of the code both advance it,
   you will desync and spend an hour hunting.
4. **Test at high speed and high curvature.** Anything that works at 3 m/s on a
   gentle arc will deceive you. Test at 15 m/s with a 90° corner.
5. **The staged plan is the plan.** Don't compress it. Each stage's output is
   useful on its own and reveals different bugs.

---

*This brief was written at the end of the single-waypoint session to bootstrap
the multi-waypoint session. If the next session reaches a decision point not
covered here, add it to §7 and §8 so the session after doesn't have to
rediscover it.*
