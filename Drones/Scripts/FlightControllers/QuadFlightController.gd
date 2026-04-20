## Quadcopter-style flight controller — full X-quad mixing matrix.
##
## Implements the classic four-channel mix
## (flight-dynamics-equations.md §10.2):
##
##     ⎡T_FL⎤     ⎡ 1  +1  +1  +1 ⎤   ⎡ throttle ⎤
##     ⎢T_FR⎥  =  ⎢ 1  -1  +1  -1 ⎥ · ⎢   roll   ⎥
##     ⎢T_RR⎥     ⎢ 1  -1  -1  +1 ⎥   ⎢   pitch  ⎥
##     ⎣T_RL⎦     ⎣ 1  +1  -1  -1 ⎦   ⎣    yaw   ⎦
##
## Each motor's throttle is a linear sum of the four command
## channels, with per-motor coefficients derived from the motor's
## position in the body's local XZ plane and its CW/CCW spin sign.
## Output is clamped to [0, 1] per motor.
##
## ── Behavioural change from earlier revisions ──────────────────────
## Earlier versions of this controller inherited translation (lift /
## strafe / pitch) from `FlightFpsController`, which composed those
## inputs into a single thrust vector and let off-centre application
## produce pitch/roll torques as a side effect.  That works for a
## loose "arcade" feel but produces noisy pitch/roll authority and
## cannot represent correct attitude-rate responses.
##
## This version interprets the inputs the way a real quad pilot does:
##   • throttle (lift)    → collective thrust (all motors equal)
##   • roll   (strafe)    → differential thrust left↔right (attitude rate)
##   • pitch  (forward)   → differential thrust front↔rear (attitude rate)
##   • yaw    (yaw)       → differential CW↔CCW thrust (prop-drag couple)
##
## The drone therefore **tilts** to fly laterally rather than
## strafing flat.  This is standard ACRO-mode behaviour and is a
## prerequisite for a PID-stabilised angle/horizon controller on top.
##
## ── Reaction torque layer (unchanged) ──────────────────────────────
## The physically-derived yaw torque from
## flight-dynamics-equations.md §5.2,
##
##     τ_yaw = κR · Σ sᵢ · Tᵢ
##
## is still applied on top of the mixing matrix.  Tᵢ is read from
## each thruster's *ramped* `throttle` property so the reaction
## torque lags RPM exactly like the linear thrust does (PowerRamp).
##
## ── Motor spin-sign convention ────────────────────────────────────
## Spin signs are derived automatically from motor position in the
## body's local XZ plane — the standard X-quad layout:
##
##     Front-Left  (x < 0, z < 0) → x·z > 0 → CW  (+1)
##     Front-Right (x > 0, z < 0) → x·z < 0 → CCW (−1)
##     Rear-Right  (x > 0, z > 0) → x·z > 0 → CW  (+1)
##     Rear-Left   (x < 0, z > 0) → x·z < 0 → CCW (−1)
##
## Override per-motor signs via `spin_sign_override` for non-standard
## layouts (e.g. hex, Y-frame, V-tail).
##
## ── Input convention ──────────────────────────────────────────────
##   • throttle_up   stick = +lift   = collective up
##   • roll_right    stick = +roll   = body rolls right (right side down)
##   • pitch_forward stick = +pitch  = nose drops (forward flight)
##   • yaw_right     stick = +yaw    = body yaws CW viewed from above
##
## (Note: "pitch_forward = nose down" is the standard RC sim
## convention.  Godot's matrix convention uses "positive pitch =
## nose up"; the sign is flipped internally so the user input
## matches pilot expectation.)
##
## ── What is NOT used from the parent ──────────────────────────────
## The following FlightFpsController exports are inherited but have
## no effect in this controller:
##   • `yaw_tilt_angle`         — motor nodes are never tilted.
##   • `yaw_idle_thrust`        — replaced by `yaw_motor_idle`.
##   • `yaw_throttle_attenuation` — not applicable to this model.
##   • `_motor_rest_basis`       — internal cache, never populated.
##
## `power_authority` still applies, but now as the *collective* gain
## rather than a whole-thrust-vector gain.
extends FlightFpsController
class_name FlightQuadController


## ── Stick-shaping (expo) ──────────────────────────────────────────
##
## Betaflight-style RC expo, per-axis.  Formula (see `apply_expo` on
## FlightControllerBase for the full doc):
##
##     out = (1 − e)·x + e·x³
##
## Endpoints are preserved (|stick|=1 always gives full authority);
## only the slope around stick=0 is reduced.  This is the right knob
## to reach for when the drone feels twitchy around centre stick —
## particularly with the T ∝ Ω² thrust curve, which inherently adds
## gain with throttle and makes small-input response snappier than
## pilots tuned to a linear plant expect.
##
## Recommended starting values:
##   • angle / stabilised controllers — 0.0 – 0.3
##   • ACRO / FPV racing feel         — 0.5 – 0.7
##   • cinematic slow flying          — 0.7 – 0.9
##
## Set all four to 0 to disable.  Throttle expo is usually left at 0
## or kept small — a softened throttle makes hover trim finicky.

## Expo on the lift / throttle stick.  Usually 0 (linear) — softening
## the throttle makes hover-trim feel mushy.  Raise only if you want
## a dead zone in the low-throttle region for cinematic flying.
@export_range(0.0, 1.0, 0.001) var throttle_expo: float = 0.0

## Expo on the roll stick.  Softens small deflections so tiny wiggles
## don't produce twitchy roll commands.  0.5 is a gentle default;
## 0.7 matches a common FPV ACRO tune.
@export_range(0.0, 1.0, 0.001) var roll_expo: float = 0.5

## Expo on the pitch stick.  Usually matched to `roll_expo` for
## symmetric feel; set separately if pitch feels more or less
## sensitive than roll on your drone.
@export_range(0.0, 1.0, 0.001) var pitch_expo: float = 0.5

## Expo on the yaw stick.  Often set a little higher than roll/pitch
## (0.6–0.8) because yaw is typically used for fine heading
## corrections rather than rapid full-stick manoeuvres.
@export_range(0.0, 1.0, 0.001) var yaw_expo: float = 0.6


## Peak yaw throttle delta applied at full yaw-stick deflection.
## CW motors are raised by this amount and CCW motors lowered (or vice
## versa), clamped to [0, 1] per motor.
##
## Increasing this value sharpens the motor-speed difference visible
## during a yaw manoeuvre and, because τ_yaw is derived from the
## resulting thrust split, also increases the body's yaw authority.
## If yaw feels weak at low throttle, raise `yaw_motor_idle` so the
## differential has some baseline RPM to act on.
@export_range(0.0, 1.0, 0.001) var yaw_differential: float = 0.25

## Peak pitch throttle delta applied at full pitch-stick deflection.
## Front motors (z < 0) are raised by this amount and rear motors
## (z > 0) lowered (or vice versa, depending on stick sign), clamped
## to [0, 1] per motor.
##
## This is the "pitch" column of the X-quad mixing matrix
## (flight-dynamics-equations.md §10.2).  Larger values produce a
## higher attitude rate at full stick; start small and raise until
## the drone tips at a pleasant speed without saturating motors on
## a gentle stick push.
##
## The default (0.08) is tuned for a "typical" 5"–10" quad with arm
## length ~0.15–0.3 m, mass ~0.5 kg, and angular drag at the default
## 0.10 N·m/(rad/s).  Attitude rate scales **linearly with arm
## length** (τ = ΔT · r), so a scene with much longer or shorter
## arms will need this raised or lowered proportionally — a drone
## with 1.0 m arms is ~5× more reactive than one with 0.2 m arms for
## the same value here.
@export_range(0.0, 1.0, 0.001) var pitch_authority: float = 0.08

## Peak roll throttle delta applied at full roll-stick deflection.
## Left motors (x < 0) raised, right motors (x > 0) lowered (or
## vice versa), clamped to [0, 1] per motor.
##
## This is the "roll" column of the X-quad mixing matrix.  Usually
## set equal to `pitch_authority` for symmetric feel; for drones
## with non-square frames (inertia different on each axis) the two
## can be tuned independently.  Roll inertia is typically lower than
## pitch inertia on a longer-than-wide airframe, so the default is
## left slightly above `pitch_authority` to give a tighter roll feel.
@export_range(0.0, 1.0, 0.001) var roll_authority: float = 0.10

## Flip the roll response if the drone rolls the wrong way when you
## push the roll stick.  Defaults to the Godot convention (body +X =
## right, camera looking along −Z); a scene whose forward-facing
## camera is rotated 180° about Y (a common content-import artefact)
## will see roll appear reversed from the pilot's viewpoint — set
## this to `true` to correct it rather than editing the scene.
@export var invert_roll: bool = false

## Flip the pitch response.  Same story as `invert_roll`: useful when
## the scene's visual "forward" is body +Z rather than the Godot
## default −Z, which inverts the pilot's pitch perception.
@export var invert_pitch: bool = false

## Propeller torque coefficient (metres) — combined $\kappa R$ from
## the momentum-theory rotor torque model $Q = \kappa\,T\,R$
## (flight-dynamics-equations.md §5.1).  The net yaw torque on the
## body is then
##
##     τ_yaw = κR · Σ sᵢ · Tᵢ
##
## where sᵢ is each motor's spin sign and Tᵢ is its current thrust
## in Newtons.  Thrust per motor is `max_force · rpm_frac²` because
## `throttle` on the thruster is RPM fraction and real props give
## $T \propto \Omega^2$ (see DroneThrusterScript's `throttle` doc).
## Because Tᵢ already encodes the yaw differential computed below,
## yaw authority scales naturally with throttle — there is no
## separate "constant torque" knob to tune.
##
## Physical values for a 5-inch quad are roughly 0.01–0.03 m
## (κ ≈ 0.05–0.15, R ≈ 0.0635 m).  Game drones with unrealistic mass
## or moment of inertia may need larger values for a responsive feel;
## the range is widened accordingly.  If yaw is too twitchy, lower
## this; if it feels sluggish, raise it.
@export_range(0.0, 1.0, 0.001) var prop_torque_coefficient: float = 0.02

## Minimum throttle floor applied to *all* motors when the yaw stick
## is deflected, scaled by |yaw|.  Keeps props "spinning" during a
## pure-yaw input so the differential has something to act on.
##
## Analogous to `yaw_idle_thrust` in the parent tilt-rotor controller.
## Set to 0 to rely entirely on the player-commanded base thrust; at
## zero base throttle and zero idle, all Tᵢ are zero and the derived
## τ_yaw collapses to zero as well (a zero-RPM prop produces no
## drag — physically correct, but means yaw authority disappears).
@export_range(0.0, 1.0, 0.001) var yaw_motor_idle: float = 0.10

## Minimum collective floor applied to *all* motors when the pitch
## or roll stick is deflected, scaled by the larger of |pitch| and
## |roll|.  Exists for the same reason as `yaw_motor_idle` — with
## the RPM-fraction thrust curve (T ∝ Ω²) a differential at zero
## collective produces minuscule torque, because a motor asked for
## e.g. 0.25 RPM only outputs 6.25 % thrust and the other side
## clamps at zero.  A small idle floor gives the differential
## working RPM so attitude authority persists at idle throttle —
## the classic "stick alive even with no throttle" FPV feel.
##
## **Throttle-punch caveat.**  Because thrust is quadratic in RPM,
## holding a roll/pitch input while the pilot climbs from idle to
## hover multiplies the attitude torque by (col_hover/col_idle)².
## A large idle floor therefore produces a "snap" as the drone
## transitions through mid-throttle.  The default (0.03) is small
## enough to keep the stick feeling alive without a noticeable punch;
## raise toward 0.10–0.15 for a more mechanical idle feel, or set to
## 0 for strictly physical behaviour (stick goes dead at zero
## throttle).
@export_range(0.0, 1.0, 0.001) var attitude_motor_idle: float = 0.03

## ── Per-axis aerodynamic angular drag ──────────────────────────────
##
## Godot's built-in `angular_damp` on RigidBody3D is *isotropic* — it
## applies the same resistance to pitch, roll, and yaw.  Real drones
## are not isotropic: a quad's four props present a large flat disc
## to yaw rotation but only edge-on to pitch and roll, so yaw
## aerodynamic damping is typically 3–5× the pitch/roll damping.
##
## These exports apply an explicit drag torque in the body's local
## frame each physics tick:
##
##     τ_drag_local = −Vector3(k_pitch · ω_x,  k_yaw · ω_y,  k_roll · ω_z)
##
## where ω is the body's angular velocity expressed in its own local
## frame.  The torque is then rotated back to world space and handed
## to `apply_torque`.  Units are N·m per rad/s (linear drag), so
## doubling ω doubles the opposing torque (matches low-speed
## aerodynamic drag on an immersed disc/plate).
##
## Recommended workflow: **set the RigidBody3D's `angular_damp` to 0**
## and tune these three values instead.  If `angular_damp` is left
## non-zero it will stack on top of these — both are additive — which
## is fine for quick tuning but makes the yaw-rate calculations in
## units-reference.md §5.3 inexact.
##
## A good starting point for the 0.5 kg / 10-inch-prop baseline:
##   angular_drag_pitch = 0.10, angular_drag_roll = 0.10,
##   angular_drag_yaw   = 0.20  (yaw ≈ 2× pitch/roll).
##
## Defaults to a modest non-zero value so the drone has a terminal
## roll/pitch rate without the pilot having to dial in drag first.
## Set all three to 0 (and tune the RigidBody3D's `angular_damp`
## instead) to disable this explicit drag path.
@export_range(0.0, 10.0, 0.001) var angular_drag_pitch: float = 0.10

## See `angular_drag_pitch`.  Roll drag around the drone's local Z
## axis (forward/back axis).  Usually similar to pitch drag.
@export_range(0.0, 10.0, 0.001) var angular_drag_roll: float = 0.10

## See `angular_drag_pitch`.  Yaw drag around the drone's local Y
## (up) axis.  Typically 2–5× the pitch/roll values because the prop
## discs act like flat plates resisting yaw rotation.
@export_range(0.0, 10.0, 0.001) var angular_drag_yaw: float = 0.20

## ── Per-axis aerodynamic linear drag (quadratic) ───────────────────
##
## Godot's built-in `linear_damp` on RigidBody3D applies a **linear**
## drag $\mathbf{F} = -d\,\mathbf{v}$.  Real aerodynamic drag is
## **quadratic** in airspeed, $\mathbf{F} = -\tfrac{1}{2}\rho C_D A
## |\mathbf{v}|\mathbf{v}$, which makes terminal velocity scale with
## $\sqrt{F/k}$ rather than $F/d$ — doubling thrust only gives you
## $\sqrt{2}$× the top speed, matching real drones.  It also makes
## slow flight feel much freer (drag collapses quadratically near
## hover) while keeping a firm speed ceiling at cruise.
##
## These exports apply an explicit quadratic drag force in the body's
## local frame each physics tick:
##
##     F_drag_local.x = −k_lateral  · v_local.x · |v_local.x|
##     F_drag_local.y = −k_vertical · v_local.y · |v_local.y|
##     F_drag_local.z = −k_forward  · v_local.z · |v_local.z|
##
## where `v_local = body.basis⁻¹ · body.linear_velocity`.  Units are
## kg/m (i.e. N per (m/s)²), absorbing the $\tfrac{1}{2}\rho C_D A$
## constants into a single per-axis number.  Per-axis lets the drone
## have a low-drag forward profile and a draggier side/belly profile
## (review §2.I) — a real quad has noticeably more drag sliding
## sideways than nose-first.
##
## Rough sizing: terminal speed along an axis under a horizontal
## force $F$ (e.g. $mg$ at a 45° tilt) is $v_\text{term} = \sqrt{F/k}$.
## A 0.5 kg quad with $k_\text{forward} = 0.05$ and 45° tilt has
## $v_\text{term} \approx \sqrt{4.9/0.05} \approx 9.9$ m/s — a modest
## cinelifter.  Racing quads with TWR 5–8 want lower k (0.02–0.03)
## to reach 25+ m/s.
##
## Recommended workflow: **set the RigidBody3D's `linear_damp` to 0**
## and tune these three values instead.  They stack additively if you
## leave `linear_damp` non-zero, which is fine for quick tuning but
## makes the $v^2$ term inexact at low speed.
##
## Defaults to non-zero values so drones have a finite terminal
## velocity without the pilot having to opt in.  Set all three to 0
## (and restore `linear_damp`) to disable this explicit drag path.

## Forward/back drag (body-local Z axis).  Low relative to lateral —
## a drone's nose-first profile is the slipperiest direction.  For a
## 0.5 kg / 20 N-thrust quad, 0.05 kg/m gives a ~10 m/s terminal at
## 45° tilt; 0.02 for racers, 0.10 for cinematic drones.
@export_range(0.0, 10.0, 0.001) var linear_drag_forward: float = 0.05

## Left/right drag (body-local X axis).  Typically 1.5–2× the
## forward value because the drone's side profile (motors, arms,
## camera pod) is much draggier than the nose profile.
@export_range(0.0, 10.0, 0.001) var linear_drag_lateral: float = 0.10

## Up/down drag (body-local Y axis).  Usually the *lowest* of the
## three because the rotor discs are nearly transparent to axial
## airflow (air flows through them rather than round them) and the
## belly profile is still small versus the sides.  Mostly matters
## for descent rate — falling belly-first through air.
@export_range(0.0, 10.0, 0.001) var linear_drag_vertical: float = 0.03

## Per-thruster spin-sign overrides.
##   Key:   the Node3D thruster reference (assign at runtime or from
##          a ready function with `spin_sign_override[get_node(...)] = 1`).
##   Value: +1 (CW) or −1 (CCW).
##
## When empty (default) spin signs are auto-derived from motor position.
var spin_sign_override: Dictionary = {}


func update_mix(body: RigidBody3D, thrusters: Array[Node]) -> void:
	if thrusters.is_empty():
		return

	# ── Read pilot stick inputs ─────────────────────────────────────
	# All in [−1, 1].  Inherited action names from FlightControllerBase.
	var lift:  float = get_axis_value(throttle_down_action,  throttle_up_action)
	var roll:  float = get_axis_value(roll_left_action,      roll_right_action)   # + = roll right
	var pitch: float = get_axis_value(pitch_backward_action, pitch_forward_action) # + = pitch forward / nose down
	var yaw:   float = get_axis_value(yaw_left_action,       yaw_right_action)    # + = yaw right (CW)

	# Apply per-axis expo (centre-stick softening).  Endpoints are
	# preserved so max authority is unchanged — only the slope near
	# stick=0 is reduced.  See `roll_expo` export doc.
	lift  = apply_expo(lift,  throttle_expo)
	roll  = apply_expo(roll,  roll_expo)
	pitch = apply_expo(pitch, pitch_expo)
	yaw   = apply_expo(yaw,   yaw_expo)

	# Apply pilot-orientation inversions (mirror-image scenes, camera
	# rotated 180°, etc.).  `invert_yaw` is on the parent and handled
	# by the existing yaw sign flow; pitch/roll are local here.
	if invert_roll:
		roll = -roll
	if invert_pitch:
		pitch = -pitch

	# ── Collective channel ──────────────────────────────────────────
	var collective: float = lift * power_authority

	# Raise collective to a floor when the yaw stick is deflected so
	# the yaw differential has non-zero motor RPM to work with — see
	# `yaw_motor_idle` doc for the rationale.
	var yaw_abs: float = absf(yaw)
	if yaw_motor_idle > 0.0 and yaw_abs > 0.0:
		var _floor: float = yaw_motor_idle * yaw_abs * power_authority
		if collective < _floor:
			collective = _floor

	# Same idea for pitch/roll: with T ∝ Ω², pure-differential thrust
	# at zero collective is vanishingly weak (the "pushed" motor is
	# squared low, the "pulled" motor clamps at zero).  Raising the
	# floor when the pilot is tilting the stick keeps attitude alive
	# without throttle — standard FPV ACRO feel.  See
	# `attitude_motor_idle` doc.
	var attitude_abs: float = maxf(absf(pitch), absf(roll))
	if attitude_motor_idle > 0.0 and attitude_abs > 0.0:
		var _floor_att: float = attitude_motor_idle * attitude_abs * power_authority
		if collective < _floor_att:
			collective = _floor_att

	# ── Per-motor X-quad mix ────────────────────────────────────────
	# motor_thr = clamp(
	#   collective
	#   − (x_local / r_max_x) · roll  · roll_authority   (right side down on +roll)
	#   + (z_local / r_max_z) · pitch · pitch_authority  (nose down on +pitch)
	#   − spin                · yaw   · yaw_differential, (CW body on +yaw)
	#   0, 1)
	#
	# Arm-length weighting (review §1.7):
	#   Each motor's pitch/roll contribution is scaled by its moment
	#   arm relative to the longest arm on that axis.  For a symmetric
	#   X-quad every motor is at the same distance so all weights are
	#   ±1 and behaviour is identical to the previous sign-only form.
	#   For H-frames, stretched-X, deadcats, etc. this gives the
	#   physically correct mix: a motor twice as far from the CoM
	#   produces twice the torque for the same thrust delta, so it
	#   should only receive half the throttle perturbation to deliver
	#   the same attitude rate — weighting by x/r_max does exactly that.
	#
	# Sign derivation (cross-reference §10.2):
	#   • roll  — matrix coeff = −sign(x); we use the same convention
	#             because user "roll right" matches matrix "positive roll".
	#   • pitch — matrix coeff = −sign(z) with matrix "positive pitch"
	#             meaning nose-UP; our "pitch_forward" is nose-DOWN, so
	#             the coefficient flips to +sign(z).
	#   • yaw   — matrix coeff = +spin with matrix "positive yaw"
	#             meaning CCW body rotation; our "yaw_right" is CW, so
	#             the coefficient flips to −spin.  (Unchanged from the
	#             prior yaw-only version of this controller.)
	var max_arm_x: float = get_max_arm_length(body, thrusters, "x")
	var max_arm_z: float = get_max_arm_length(body, thrusters, "z")
	# Guard against degenerate / colinear layouts where every motor
	# sits on one axis.  Falling back to 1.0 makes `x / r_max` behave
	# like the raw body-local coordinate (i.e. still signed, just
	# un-normalised), which is harmless — no motor contribution
	# exceeds the authority knob because roll/pitch inputs are in
	# [−1, 1] and the final motor throttle is clamped below.
	if max_arm_x <= 0.0:
		max_arm_x = 1.0
	if max_arm_z <= 0.0:
		max_arm_z = 1.0

	var signed_thrust_sum: float = 0.0
	var any_active := false

	# ── Desaturation pass 1: compute per-motor deltas (no clamp) ────
	#
	# Priority-based desaturation (review §2.M).  The naïve mixer
	# clamps each motor to [0, 1] independently, which silently
	# truncates the attitude differential whenever a motor saturates.
	# A real flight stack (Betaflight airmode, PX4 mixer) preserves
	# the *attitude* command and gives up *collective* first, yaw
	# second, attitude last:
	#
	#   1. Shift collective up/down to absorb the most-over and
	#      most-under motor — every motor moves by the same amount,
	#      so the attitude/yaw split is unchanged.  This is free
	#      (no information is lost) as long as the motor-delta
	#      spread (max − min of attitude + yaw contributions across
	#      motors) ≤ 1.0.
	#   2. If the spread exceeds 1.0, scale **yaw** down just enough
	#      to bring it back to ≤ 1.0.  The pilot loses yaw authority
	#      at this extreme, but roll/pitch attitude stays honest.
	#   3. If even yaw=0 leaves the spread > 1.0, scale attitude
	#      proportionally.  This is the "drone physically cannot do
	#      what you asked" fallback.
	#
	# Everything is a no-op in normal flight — triggers only at/near
	# motor saturation.  The yaw/attitude idle floors above are still
	# applied (they're additive to the physical model); desaturation
	# just prevents the clamp from silently eating commands the pilot
	# put in.
	var n_motors: int = thrusters.size()
	var att_deltas := PackedFloat32Array()
	var yaw_deltas := PackedFloat32Array()
	var spin_cache := PackedInt32Array()
	att_deltas.resize(n_motors)
	yaw_deltas.resize(n_motors)
	spin_cache.resize(n_motors)

	var min_d: float = INF
	var max_d: float = -INF
	var min_a: float = INF   # attitude-only extent, for fallback scaling
	var max_a: float = -INF

	for i in n_motors:
		var t = thrusters[i]
		if not (t is Node3D):
			att_deltas[i] = 0.0
			yaw_deltas[i] = 0.0
			spin_cache[i] = 0
			continue

		var motor := t as Node3D
		var spin: int = _get_spin_sign(motor, body)
		spin_cache[i] = spin

		var local_pos: Vector3
		if body != null:
			local_pos = body.to_local(motor.global_position)
		else:
			local_pos = motor.transform.origin

		var roll_weight:  float = -local_pos.x / max_arm_x
		var pitch_weight: float =  local_pos.z / max_arm_z

		var att: float = roll_weight * roll * roll_authority + pitch_weight * pitch * pitch_authority
		var ydelta: float = -float(spin) * yaw * yaw_differential

		att_deltas[i] = att
		yaw_deltas[i] = ydelta

		var combined: float = att + ydelta
		if combined < min_d: min_d = combined
		if combined > max_d: max_d = combined
		if att < min_a: min_a = att
		if att > max_a: max_a = att

	# ── Desaturation pass 2: scale channels if spread > 1.0 ─────────
	var spread: float = max_d - min_d
	if spread > 1.0:
		var att_spread: float = max_a - min_a
		if att_spread >= 1.0:
			# Step 3: attitude alone doesn't fit.  Scale it to exactly
			# fill the motor range and drop yaw entirely.
			var att_scale: float = 1.0 / att_spread
			min_d = INF
			max_d = -INF
			for i in n_motors:
				var d: float = att_deltas[i] * att_scale
				att_deltas[i] = d
				yaw_deltas[i] = 0.0
				if d < min_d: min_d = d
				if d > max_d: max_d = d
		else:
			# Step 2: attitude fits, yaw pushes it over.  Bisect
			# yaw_scale so combined spread = 1.0.  The spread is
			# piecewise-linear in yaw_scale so a dozen iterations
			# give > 4 decimal digits of precision, well below
			# anything a pilot could feel.
			var lo: float = 0.0
			var hi: float = 1.0
			for _it in 12:
				var mid: float = 0.5 * (lo + hi)
				var _min: float = INF
				var _max: float = -INF
				for i in n_motors:
					var d: float = att_deltas[i] + mid * yaw_deltas[i]
					if d < _min: _min = d
					if d > _max: _max = d
				if (_max - _min) > 1.0:
					hi = mid
				else:
					lo = mid
			# Final yaw scale = lo (the largest value that still fits).
			min_d = INF
			max_d = -INF
			for i in n_motors:
				var d2: float = att_deltas[i] + lo * yaw_deltas[i]
				att_deltas[i] = d2  # fold yaw into att_deltas for pass 3
				if d2 < min_d: min_d = d2
				if d2 > max_d: max_d = d2
	else:
		# Fits as-is.  Fold yaw into att_deltas for a uniform pass 3.
		for i in n_motors:
			att_deltas[i] = att_deltas[i] + yaw_deltas[i]

	# Step 1: shift collective into the window that keeps every motor
	# inside [0, 1].  After scaling, `spread ≤ 1.0` so col_lo ≤ col_hi
	# and this always finds a valid collective.
	var col_lo: float = -min_d
	var col_hi: float = 1.0 - max_d
	collective = clampf(collective, col_lo, col_hi)

	# ── Desaturation pass 3: write final per-motor commands ─────────
	for i in n_motors:
		var t = thrusters[i]
		if not (t is Node3D):
			continue

		# The `clampf` is a safety net — after desaturation the sum
		# is already inside [0, 1] to floating-point tolerance.
		var motor_thr: float = clampf(collective + att_deltas[i], 0.0, 1.0)

		# All motors push straight UP in their own local frame; the
		# body's tilt does the lateral-flight work (ACRO-mode style).
		# `set_thrust_directed` writes the command; the thruster's
		# PowerRamp will lag the actual applied throttle.
		if t.has_method("set_thrust_directed"):
			t.set_thrust_directed(Vector3.UP, motor_thr)
		elif t.has_method("set_thrust"):
			t.set_thrust(Vector3.UP * motor_thr)

		# Tᵢ = max_force · rpm_frac².  `throttle` on the thruster is
		# RPM fraction (§5.1 — T ∝ Ω²), so square it here to get the
		# real Newtons of thrust entering Σ sᵢ·Tᵢ.  Reading the
		# *ramped* value keeps yaw authority lagged identically to the
		# linear thrust.  Falls back to the just-commanded `motor_thr`
		# for exotic thrusters that don't expose a `throttle`
		# property — also squared, same reasoning.
		var motor_max: float = 1.0
		if t.has_method("get_max_force"):
			motor_max = t.get_max_force()
		var rpm_frac: float = motor_thr
		if "throttle" in t:
			rpm_frac = t.throttle
		signed_thrust_sum += float(spin_cache[i]) * motor_max * rpm_frac * rpm_frac

		if motor_thr > 0.0:
			any_active = true

	# ── Yaw reaction torque (physically derived) ────────────────────
	# τ_yaw = κR · Σ sᵢ · Tᵢ   (flight-dynamics-equations.md §5.2)
	# Sign falls out of the thrust split — with spin=+1 CW and
	# yaw>0 CW-desired, CW motors are lowered so Σ sᵢ·Tᵢ < 0, and
	# `+κR · sum` along body-Y gives a −Y torque → CW rotation ✓.
	if body != null and prop_torque_coefficient > 0.0 and absf(signed_thrust_sum) > 0.0001:
		body.apply_torque(body.basis.y * prop_torque_coefficient * signed_thrust_sum)

	# ── Per-axis aerodynamic angular drag ───────────────────────────
	# Apply τ_drag = −k · ω component-wise in the body's local frame,
	# then rotate the resulting torque back into world space before
	# handing it to `apply_torque`.  This lets yaw be more damped
	# than pitch/roll (see export doc on `angular_drag_pitch`).
	#
	# Skipped when all three coefficients are zero — cheap early-out
	# so users who haven't opted in don't pay for the transform math.
	if body != null and (angular_drag_pitch > 0.0 or angular_drag_roll > 0.0 or angular_drag_yaw > 0.0):
		# RigidBody3D bases are orthonormal, so transposed() == inverse().
		var w_local: Vector3 = body.basis.transposed() * body.angular_velocity
		var drag_local := Vector3(
			-w_local.x * angular_drag_pitch,  # local-X = pitch axis
			-w_local.y * angular_drag_yaw,    # local-Y = yaw   axis
			-w_local.z * angular_drag_roll,   # local-Z = roll  axis
		)
		# Rotate back to world frame for apply_torque, which takes
		# a world-space torque vector.
		body.apply_torque(body.basis * drag_local)

	# ── Per-axis aerodynamic linear drag (quadratic) ────────────────
	# F_drag_local = −k · v_local · |v_local|   (component-wise)
	#
	# v² rather than v means drag is negligible near hover and rises
	# sharply at cruise — matching real aero behaviour.  Per-axis
	# coefficients let the drone have a slippery nose and a draggy
	# side/belly (review §2.I).  See the `linear_drag_forward` export
	# doc for sizing guidance.
	#
	# Skipped when all three coefficients are zero so drones that
	# rely on Godot's `linear_damp` (or have none) pay nothing for
	# the transform math.
	if body != null and (linear_drag_forward > 0.0 or linear_drag_lateral > 0.0 or linear_drag_vertical > 0.0):
		var v_local: Vector3 = body.basis.transposed() * body.linear_velocity
		var fd_local := Vector3(
			-linear_drag_lateral  * v_local.x * absf(v_local.x),
			-linear_drag_vertical * v_local.y * absf(v_local.y),
			-linear_drag_forward  * v_local.z * absf(v_local.z),
		)
		body.apply_central_force(body.basis * fd_local)

	# Silence all engines when player releases every input.
	if not any_active and yaw_abs < 0.001:
		_silence_all(thrusters)


## Returns the CW/CCW spin sign (+1 or −1) for a single motor node.
##
## Checks `spin_sign_override` first.  If no override is registered,
## deduces the sign from the motor's position in the body's local XZ
## plane using the standard X-quad rule (see class header).
func _get_spin_sign(motor: Node3D, body: RigidBody3D) -> int:
	# Manual override takes precedence over position detection.
	if spin_sign_override.has(motor):
		var s: int = spin_sign_override[motor]
		return 1 if s >= 0 else -1

	# Resolve local position relative to the body origin.
	# Using body.to_local handles motors nested under arm sub-nodes
	# as well as the common case of motors as direct body children.
	var local_pos: Vector3
	if body != null:
		local_pos = body.to_local(motor.global_position)
	else:
		local_pos = motor.transform.origin

	# X-quad rule: same-sign quadrant → CW, opposite-sign → CCW.
	var xz: float = local_pos.x * local_pos.z
	if absf(xz) < 0.0001:
		# Motor lies on the X or Z axis (indeterminate).  Default CW.
		push_warning(
			"QuadFlightController: motor '%s' is on a body axis (x·z ≈ 0); "
			% motor.name +
			"spin sign defaults to CW (+1).  Set spin_sign_override to silence."
		)
		return 1

	return 1 if xz > 0.0 else -1
