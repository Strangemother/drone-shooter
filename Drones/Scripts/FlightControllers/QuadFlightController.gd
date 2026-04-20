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
@export_range(0.0, 1.0, 0.001) var pitch_authority: float = 0.25

## Peak roll throttle delta applied at full roll-stick deflection.
## Left motors (x < 0) raised, right motors (x > 0) lowered (or
## vice versa), clamped to [0, 1] per motor.
##
## This is the "roll" column of the X-quad mixing matrix.  Usually
## set equal to `pitch_authority` for symmetric feel; for drones
## with non-square frames (inertia different on each axis) the two
## can be tuned independently.
@export_range(0.0, 1.0, 0.001) var roll_authority: float = 0.25

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
##   angular_drag_pitch = 0.05, angular_drag_roll = 0.05,
##   angular_drag_yaw   = 0.25  (yaw ≈ 5× pitch/roll).
##
## Left at 0 by default so existing scenes are not silently changed.
@export_range(0.0, 10.0, 0.001) var angular_drag_pitch: float = 0.0

## See `angular_drag_pitch`.  Roll drag around the drone's local Z
## axis (forward/back axis).  Usually similar to pitch drag.
@export_range(0.0, 10.0, 0.001) var angular_drag_roll: float = 0.0

## See `angular_drag_pitch`.  Yaw drag around the drone's local Y
## (up) axis.  Typically 3–5× the pitch/roll values because the prop
## discs act like flat plates resisting yaw rotation.
@export_range(0.0, 10.0, 0.001) var angular_drag_yaw: float = 0.0

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
	# `yaw_motor_idle` doc for the rationale.  Only yaw gets an idle
	# floor; pitch/roll authority is expected to scale with throttle
	# (matches real-drone feel at low RPM).
	var yaw_abs: float = absf(yaw)
	if yaw_motor_idle > 0.0 and yaw_abs > 0.0:
		var _floor: float = yaw_motor_idle * yaw_abs * power_authority
		if collective < _floor:
			collective = _floor

	# ── Per-motor X-quad mix ────────────────────────────────────────
	# motor_thr = clamp(
	#   collective
	#   − sign(x_local) · roll  · roll_authority     (right side down on +roll)
	#   + sign(z_local) · pitch · pitch_authority    (nose down on +pitch)
	#   − spin          · yaw   · yaw_differential,  (CW body on +yaw)
	#   0, 1)
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
	var signed_thrust_sum: float = 0.0
	var any_active := false

	for t in thrusters:
		if not (t is Node3D):
			continue

		var motor := t as Node3D
		var spin: int = _get_spin_sign(motor, body)

		# Position in body-local frame (handles nested motors).
		var local_pos: Vector3
		if body != null:
			local_pos = body.to_local(motor.global_position)
		else:
			local_pos = motor.transform.origin

		var roll_sign:  float = -signf(local_pos.x)
		var pitch_sign: float = signf(local_pos.z)

		var motor_thr: float = clampf(
			collective
			+ roll_sign  * roll  * roll_authority
			+ pitch_sign * pitch * pitch_authority
			- float(spin) * yaw   * yaw_differential,
			0.0, 1.0
		)

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
		signed_thrust_sum += float(spin) * motor_max * rpm_frac * rpm_frac

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
