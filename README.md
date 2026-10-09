# Fork Documentation: Kenaiwolf/ardupilot-4.7.Custom — Rover Vectored-Thrust Drift Compensation  
  
**Fork base:** upstream ArduPilot **Rover 4.7.1** (release line ending at `dbe79216`, Sep 2026). Fork work begins at `baab9fab` ("nutne minimum V1") and culminates at `9f09d8bf` (this document).  
  
**Target vehicle:** 250 kg vectored-thrust boat (single steerable thruster, GPS + compass, no wheel encoders).  
  
**Modified files:** `Rover/GCS_MAVLink_Rover.cpp`, `Rover/mode.cpp`, `Rover/mode.h`, `Rover/mode_guided.cpp`, `Rover/mode_loiter.cpp`, `libraries/APM_Control/AR_AttitudeControl.cpp/.h`, `libraries/AR_Motors/AP_MotorsUGV.cpp/.h`, baro driver replace `AP_BARO_LPS2XH`→`AP_BARO_MS5611` (`0320e405`), param files `Tools/Frame_params/Deset-mapping-boat.param` + `Tools/autotest/default_params/rover-vectored.parm`, new file `extra_hwdef.dat` (repo root), `README.md` (this file).  
  
---  
  
## 1. Feature Overview  
  
Three layered features on top of upstream Rover:  
  
1. **Current/wind drift estimation & compensation** — two independent estimators (nav-source while driving, loiter-source while holding position) feed crab-angle/speed corrections into Auto, Guided and Loiter, with per-source gains, age-weighted seeding across mode switches, and a kill-switch at gain=0.  
2. **Steering thrust floor + throttle management** — guarantees minimum thrust whenever a large heading correction is commanded, so a vectored thruster always has authority; plus cosine throttle reduction and speed-PID I-freeze during pivot turns.  
3. **Vectored-thrust angle blending** — blends direct and atan-based steering-angle computation based on filtered throttle, with hysteresis deadband.  
  
Supporting infrastructure: MAVLink named-float diagnostics, an `ATC_SPD_EXPO` nonlinear speed↔throttle curve, and a slim `extra_hwdef.dat` build.  
  
---  
  
## 2. `libraries/AR_Motors/AP_MotorsUGV.cpp/.h`  
  
### 2.1 Vectored-thrust angle blending (`MOT_VEC_BLEND_THR`, `MOT_VEC_DEADBAND`, `MOT_VEC_RESID_TC`)  
  
In the vectored-thrust output path, the commanded (steering, throttle) pair is converted to a thruster deflection angle `steering_angle_rad`. Two mappings exist:  
  
- **Direct mapping** — angle proportional to steering demand; best at low/zero throttle.  
- **Legacy `atan()` mapping** — `atan2(steer, throttle)`; best at cruise.  
  
The fork blends them with weight `w`:  
  
```cpp  
w = 1.0f - constrain_float(fabsf(_vec_throttle_filt) / _vec_blend_thr, 0.0f, 1.0f);  
steering_angle_rad = w * direct_angle_rad + (1.0f - w) * atan_angle_rad;  
// AP_MotorsUGV.cpp ~1030-1038  
```  
  
`w=1` (near-zero throttle) → pure direct mapping; `w=0` (throttle ≥ blend threshold) → pure atan mapping. The same `w` blends the boost scaler:  
  
```cpp  
const float throttle_scaler_inv = w + (1.0f - w) * cosf(steering_angle_rad);  
if (!is_zero(throttle_scaler_inv)) {  
    throttle /= throttle_scaler_inv;  
}  
// AP_MotorsUGV.cpp ~1057-1060  
```  
  
**Physics:** `throttle` is the commanded *forward* thrust component. The boost guarantees the commanded forward component `T·cos(θ) = throttle` is deliverable — `T = throttle/cos(θ)` — which simultaneously makes the lateral component `T·sin(θ) = throttle·tan(θ)` large enough to turn the boat at speed. Without it, the boat loses steering authority exactly when moving fast. The 1/cos spike near 90° is intentional (see §10).  
  
`MOT_VEC_DEADBAND` freezes `_vec_last_steering_angle_rad` when both steering and throttle demands are near zero — prevents servo hunting. `_vec_resid_tc` sets the EMA time constant on `_vec_throttle_filt` so the blend weight doesn't chatter with throttle noise.  
  
### 2.2 Drift estimator state store  
  
Four slots of persistent state per estimate (nav + loiter):  
  
```cpp  
void AP_MotorsUGV::set_nav_estimate_ne(const Vector2f &drift_ne)  
{  
    const float gain = constrain_float(_drift_comp_gain_nav, 0.0f, 2.0f);  
    _nav_estimate_ne = drift_ne * gain;      // gain applied at WRITE time  
    _nav_estimate_ms = AP_HAL::millis();  
    _nav_estimate_is_seeded = false;  
}  
// AP_MotorsUGV.cpp ~301-307  
```  
  
```cpp  
void AP_MotorsUGV::seed_loiter_estimate_ne(const Vector2f &drift_ne, uint32_t source_ms)  
{  
    _loiter_estimate_ne = drift_ne;  
    _loiter_estimate_ms = source_ms;         // ORIGINAL timestamp preserved  
    _loiter_estimate_is_seeded = true;  
}  
// AP_MotorsUGV.cpp ~310-315  
```  
  
Write-time gain means a stored value is PID-ready; changing the gain parameter doesn't retroactively rescale stored estimates (they refresh on the next sample). The `is_seeded` flag distinguishes measured samples from handoff placeholders.  
  
### 2.3 Source arbitration — `get_current_estimate_ne`  
  
```cpp  
const bool loiter_real = (loiter_age_ms != UINT32_MAX) && !_loiter_estimate_is_seeded;  
const bool nav_real    = (nav_age_ms    != UINT32_MAX) && !_nav_estimate_is_seeded;  
// a genuinely-measured estimate always wins over a seeded one, regardless of timestamp;  
// between two seeds or two reals, newer wins  
// AP_MotorsUGV.cpp ~357-370  
```  
  
Then a shared staleness cutoff `DRIFT_MAXAGE` (default 1800 s, 0 = never trust) rejects both. Priority rule: **real > seeded**, then **newest wins**.  
  
### 2.4 Parameter group  
  
All new params live in `AP_MotorsUGV` under the `MOT_`/`LOIT_`/`DRIFT_`/`SFL_` prefixes — see §9 table.  
  
---  
  
## 3. `libraries/APM_Control/AR_AttitudeControl.cpp/.h`  
  
### 3.1 Nonlinear speed→throttle feed-forward (`ATC_SPD_EXPO`)  
  
```cpp  
if (is_positive(cruise_speed) && is_positive(cruise_throttle)) {  
    const float speed_ratio = fabsf(_desired_speed) / cruise_speed;  
    const float expo = (_speed_thr_expo > 0.0f) ? _speed_thr_expo : 1.0f;  
    throttle_base = cruise_throttle * powf(speed_ratio, expo);  
    if (is_negative(_desired_speed)) {  
        throttle_base = -throttle_base;  
    }  
}  
// AR_AttitudeControl.cpp ~800-807  
```  
  
Boats have quadratic drag — a linear FF underestimates thrust at low speed ratios and overestimates above cruise. `expo` shapes the calibrated curve through the (cruise_speed, cruise_throttle) point. Value 1.0 = stock behavior; ~2.0 recommended for displacement hulls.  
  
**Important for estimation:** the same `powf(ratio, expo)` curve is the basis for loiter method-2's inverse mapping (I-term → equivalent speed) and the drift feed-forward — one parameter governs all three, keeping them consistent.  
  
### 3.2 Throttle-limit flag plumbing  
  
`get_throttle_out_speed` / `get_throttle_out_stop` already took `motor_limit_low`/`motor_limit_high`; the fork OR's `steer_i_freeze` into both at the call site (`mode.cpp:352-356`). Inside, all four collapse to one bool:  
  
```cpp  
float throttle_out = _throttle_speed_pid.update_all(_desired_speed, speed, dt,  
    (motor_limit_low || motor_limit_high || _throttle_limit_low || _throttle_limit_high));  
throttle_out += _throttle_speed_pid.get_ff();  
// AR_AttitudeControl.cpp ~810-811  
```  
  
See §4.4 for why the collapsed direction-agnostic limit is correct.  
  
### 3.3 Normalized steering output contract (`get_steering_out_rate`) — FIX PENDING  
  
`get_steering_out_rate` currently returns `P+D+I+FF` **unconstrained** (upstream behavior):  
  
```cpp  
float output = _steer_rate_pid.update_all(_desired_turn_rate, AP::ahrs().get_yaw_rate_earth(), dt, (motor_limit_left || motor_limit_right));  
output += _steer_rate_pid.get_ff();  
return output;  
// AR_AttitudeControl.cpp:719-722  
```  
  
All Rover callers multiply by 4500 and treat the result as normalized ±1. Upstream stays within range only because PID gains keep the sum below ~7.3; this fork's tuning routinely produces −14…−19, which exposed the downstream `(int16_t)` wrap documented in §4.6.  
  
**Fix (prepared, pending commit):**  
  
```cpp  
return constrain_float(output, -1.0f, 1.0f);  
```  
  
Safe to apply: turn-rate overshoot shaping happens on the *input* (`_steer_decel_max`/`_steer_accel_max` sqrt-limits `desired_rate` at `AR_AttitudeControl.cpp:645-651`), no Rover caller relies on |output|>1, and ArduPlane uses a different class (`AP_SteerController`), so the change is Rover-only.  
  
---  
  
## 4. `Rover/mode.cpp` and `Rover/mode.h`  
  
### 4.1 `update_drift_estimator(raw_heading_cd, desired_speed)` — nav-source estimator  
  
Called from `navigate_to_waypoint` and Guided HeadingAndSpeed with the **uncompensated** commanded vector. Accumulates heading+speed samples only while:  
  
```cpp  
const bool yaw_rate_ok = fabsf(degrees(ahrs.get_yaw_rate_earth())) < drift_est_yaw_rate_dps;  
// mode.cpp ~585-587 — earth-frame yaw rate (not gyro.z), so wave-induced  
// roll/pitch rates don't contaminate the gate  
```  
  
`MOT_DRIFT_EST_YAWR` (default 15 °/s, 0 = off; promoted from compile-time constant `DRIFT_EST_MAX_YAW_RATE_DPS` in `mode.h`) is the yaw-rate gate. Samples collected during steady straight motion are converted into an NE drift vector and written via `set_nav_estimate_ne`. The estimator window is reset on mode entry so stale partial windows don't leak across modes.  
  
### 4.2 `apply_drift_compensation(heading_cd, speed)` — crab-angle correction  
  
Computes the required own-motion vector = desired track − estimated drift, then:  
  
```cpp  
const float speed_max = calc_speed_max(g.speed_cruise, 1.0f);  
desired_speed = MIN(own_required_ne.length(), speed_max);  
// mode.cpp ~561 — clamp magnitude only; direction stays optimal (up-current)  
// even if required speed exceeds capability: pointing up-current at max speed  
// is the best achievable response  
```  
  
A reversal branch handles `desired_speed < 0` (heading +180°, negative speed). In-place at the caller — which is why Guided passes a **local copy** of `_desired_yaw_cd` (see §6).  
  
### 4.3 Seeding on mode entry — `Mode::enter()`  
  
```cpp  
// one-shot seed from Loiter, age-weighted; avoids cold (0,0) start.  
// seeded sources accepted: the seed carries the ORIGINAL sample timestamp,  
// so ping-pong between slots is safe - each bounce multiplies by seed_weight <= 1  
// and the value keeps aging toward DRIFT_MAXAGE (converges to zero, never amplifies).  
// guard: never overwrite a real (measured) destination estimate with a weighted  
// seed copy - a real sample has higher priority in get_current_estimate_ne().  
Vector2f nav_ne; uint32_t nav_age_ms = 0; bool nav_is_seeded = false;  
const bool nav_has_real =  
    g2.motors.get_nav_estimate_ne(nav_ne, nav_age_ms, nav_is_seeded) &&  
    !nav_is_seeded &&  
    (nav_age_ms < uint32_t(g2.motors.get_drift_max_age_s() * 1000.0f));  
if (!nav_has_real &&  
    g2.motors.get_loiter_estimate_ne(loiter_ne, loiter_age_ms, loiter_is_seeded) &&  
    (loiter_age_ms >= DRIFT_SEED_MIN_AGE_MS) &&  
    (loiter_age_ms < uint32_t(g2.motors.get_drift_max_age_s() * 1000.0f))) {  
    // seed nav slot from loiter value, weighted by source age  
}  
// mode.cpp ~59-80  
```  
  
`DRIFT_SEED_MIN_AGE_MS = 50` (`mode.h`) blocks same-tick echo (Guided::_enter → start_loiter → ModeLoiter::_enter). Seeded sources are deliberately accepted so bridge `LOITER_WP_RESET` bounces (Guided→Loiter→Guided in ~1.5 s) don't cold-start the nav estimator — the loiter window almost never completes in that window.  
  
### 4.4 `calc_throttle` — cosine reduction, drift FF, steering floor, I-freeze  
  
Order of operations in the vectored-thrust + heading-fresh block:  
  
```cpp  
// 1. Cosine Throttle Reduction: reduce forward throttle at large heading error.  
// only scale forward thrust - a negative (braking) PID demand must pass  
// through unchanged, otherwise a heavy boat cannot decelerate during a  
// large heading change  
if (is_positive(throttle_out)) {  
    throttle_out *= MAX(0.0f, cosf(yaw_error_rad));  
}  
// mode.cpp ~390-399  
```  
  
Then drift feed-forward is **added after** the cosine scaling (previously the FF could be zeroed out by cos() at large crab angles, defeating its purpose of holding position):  
  
```cpp  
// FF = full nonlinear curve evaluated at |drift_body.x|, NOT a local derivative —  
// at target_speed=0 (loiter) dThrottle/dv ≈ 0 for expo>1 and FF would vanish  
// exactly where it is needed. computed inline: get_throttle_out_speed() is a  
// stateful PID loop and a second call per tick would corrupt I/D terms  
if (is_positive(drift_body.x)) {  
    const float drift_ratio = drift_body.x / cruise_speed;  
    throttle_out += 100.0f * throttle_cruise_frac * powf(drift_ratio, expo);  
}  
// mode.cpp ~401-422  
```  
  
Then the steering thrust floor (applied last, so nothing downstream can remove it):  
  
```cpp  
// 2. Steering Thrust Floor: force minimum throttle to allow rotation  
if (yaw_error_deg > g2.motors.get_steer_floor_deadband_deg()) {  
    const float steer_throttle_floor = constrain_float(  
        (yaw_error_deg - g2.motors.get_steer_floor_deadband_deg()) * g2.motors.get_steer_floor_gain(),  
        0.0f, g2.motors.get_steer_floor_max_pct());  
    // do not override a throttle that is already stronger than the floor in  
    // the same direction; only raise the magnitude, never flip its sign  
    if (fabsf(throttle_out) < steer_throttle_floor) {  
        throttle_out = is_negative(throttle_out) ? -steer_throttle_floor : steer_throttle_floor;  
    }  
}  
```  
  
`SFL_DB=10°`, `SFL_GAIN=0.7`, `SFL_MAX=20%`, `SFL_IFRZ=45°`. Yaw error is measured against `_steering_target_yaw_cd` (the target *before* crab correction — fixing compounding where the corrected heading fed back into the error).  
  
**Freshness gate:** `steering_heading_fresh` = `_steering_heading_active_ms` written by `calc_steering_to_heading()` within 50 ms (`mode.h`). This scopes floor + cosine reduction to heading-driven modes — Auto, Guided, Loiter, **and Simple and Follow** (which also call `calc_steering_to_heading`).  
  
**I-freeze (`SFL_IFRZ`) — reviewed, intentionally kept:**  
  
`steer_i_freeze` is OR'd into both limit flags and collapses to one bool passed to `AC_PID::update_i`. With `limit=true`, `update_i` permits integrator change only when error opposes integrator sign — **hold-or-decay, never grow**.  
  
This was originally flagged as a "direction-undifferentiated freeze" defect with a proposed hard-hold fix. On deeper analysis the current behavior is correct:  
  
1. **The speed-PID integrator is a body-frame quantity** — "forward thrust needed to hold commanded speed", valid only relative to the heading at which it was learned. Freeze only activates above `SFL_IFRZ=45°`, i.e. always during large reorientations. After a 90° turn, the current that required forward thrust becomes lateral (invisible to the speed PID); after 180°, the required thrust flips sign. A held integrator would be systematically *stale* — worse than zero.  
2. **Decay acts as implicit invalidation.** During a pivot, speed error typically opposes the integrator (current/inertia carries the boat past target), so the sign rule lets the integrator bleed toward zero — a reasonable reset of a body-frame estimate. True negative windup is already blocked: same-sign growth is never allowed, and the decay branch dead-ends at exactly zero.  
  
The world-frame memory of current lives in the drift estimator (NE coordinates), heading-invariant and unaffected by this freeze — nothing that matters is forgotten. Kept consequence: brief speed undershoot after a large turn while I re-learns drag — small at typical `ATC_SPD_I` and pivot durations.  
  
### 4.5 `mode.h` additions  
  
New members: `_steer_floor_active`, `_steering_heading_active_ms`, `_drift_rising_count`, `_drift_zero_count`, `_drift_i_filt`/`_drift_i_filt_valid`, `_inside_loiter_circle`, `_throttle_nav_pct`; constants `DRIFT_SEED_MIN_AGE_MS`, `LOITER_DRIFT_RISING_TICKS`, `LOITER_DRIFT_ZERO_TICKS=5`; helpers `update_drift_estimator`, `apply_drift_compensation`, `get_drift_compensation_body`, `calc_steering_to_heading`, `start_loiter` (Guided), `get_speed_thr_expo` accessor.  
  
### 4.6 `Mode::set_steering` — `(int16_t)` overflow wrap (BUG IDENTIFIED, FIX PENDING)  
  
**Symptom (log 00000105.BIN, t≈10337–10473):** Loiter held a ~135–180° heading error for 137 s while `PIDS` showed the steering PID saturated (P+I+FF ≈ −14.6 every tick). `STER.SteerOut` however oscillated only ±1400 and the boat turned ~1°/s instead of ~272°/s. `WpDist` diverged 1.7→39 m.  
  
**Root cause:** `Mode::set_steering` (`mode.cpp:827`) casts the demand through `int16_t` inside the stick-mixing branch:  
  
```cpp  
void Mode::set_steering(float steering_value)  
{  
    if (allows_stick_mixing() && g2.stick_mixing > 0) {  
        steering_value = channel_steer->stick_mixing((int16_t)steering_value);  // WRAP  
    }  
    g2.motors.set_steering(steering_value);  
}  
```  
  
A saturated demand of −18.77×4500 = **−84465** wraps mod 65536 to **−18929** — observed SteerOut −18840 (delta = unlogged D-term). Verified on 6/6 sampled points; wrapped output sits almost exactly on the −65536 boundary, so ±0.3-unit PID jitter sweeps the servo output through zero → average deflection ≈ 0 → self-locking failure. Active because `ModeLoiter::is_autopilot_mode() = true` (`mode.h`) → `allows_stick_mixing()` is true whenever `STICK_MIXING > 0`.  
  
**Fix (prepared, pending commit):**  
  
```cpp  
steering_value = channel_steer->stick_mixing((int16_t)constrain_float(steering_value, -4500.0f, 4500.0f));  
```  
  
Plus the defense-in-depth clamp in §3.3. Affects **every** autopilot mode (`is_autopilot_mode()=true`) whenever `STICK_MIXING>0` and the heading PID saturates — not just Loiter. Interaction with fork features is positive: §5.3's `get_steering()/4500` normalization and §2.1's `steering_norm` blend both assume ±4500 and currently receive wrapped garbage.  
  
### 4.7 Related — `stop_vehicle` bypass  
  
`Mode::stop_vehicle` (`mode.cpp:475`) calls `g2.motors.set_steering(steering_out * 4500.0)` **directly**, bypassing `Mode::set_steering`. Harmless today (servo output is clamped at `AP_MotorsUGV.cpp:1130`), but worth unifying so all steering output flows through one path.  
  
---  
  
## 5. `Rover/mode_loiter.cpp`  
  
Loiter is the second drift source and the main beneficiary of compensation.  
  
### 5.1 Anti-drift heading (bow-into-current)  
  
```cpp  
if (g2.motors.get_current_estimate_ne(drift_ne) &&  
    drift_ne.length() > g2.motors.get_loit_drift_min_mps()) {  
// mode_loiter.cpp ~79-80 — LOIT_DRIFT_MIN default 0.03 m/s  
```  
  
When drift exceeds the minimum, loiter steers the bow into the estimated current instead of holding the entry heading — reduces the lateral component the thruster must continuously fight.  
  
### 5.2 Method 1 — coast sampling  
  
While inside the loiter circle, the controller watches `_throttle_nav_pct` (snapshot taken before the floor/FF block — `mode.cpp:367`). When drift pushes the boat and nav throttle keeps rising for `LOITER_DRIFT_RISING_TICKS` consecutive ticks, a **coast** window opens: throttle drops below `LOIT_COAST_THR` (1 %) and the GPS velocity during the coast is sampled as a drift measurement → `set_loiter_estimate_ne`.  
  
**Rotation gate:** sampling is rejected while the boat is actively rotating:  
  
```cpp  
const float yaw_rate_degs = fabsf(degrees(ahrs.get_yaw_rate_earth()));  
// gated by LOIT_ROT_ANG (15°) and LOIT_ROT_RATE (20°/s), 0 = gate off  
// rationale: a deflected bow thruster rotates the hull about its pivot point  
// (~0.6 m radius at 90°), generating lateral IMU velocity r·L_pivot up to  
// ~0.5 m/s that is NOT drift  
```  
  
Uses `get_yaw_rate_earth()` (earth-frame) so wave-induced roll/pitch rates don't false-trigger the gate — consistent with the nav estimator's gate.  
  
**`_steer_floor_active` gate:** coast sampling is suppressed while the floor is forcing throttle (the boat isn't coasting).  
  
**Zero-drift sample (`_drift_zero_count`, `LOITER_DRIFT_ZERO_TICKS=5`):** if the equilibrium window holds but the filtered I-term stays below the `LOIT_I_MIN` noise floor for 5 ticks, a **zero** drift estimate is accepted — without it, a ceased current would leave the last nonzero estimate pinned until `DRIFT_MAXAGE` expiry.  
  
### 5.3 Method 2 — equilibrium I-term inference  
  
When the loiter is in steady state (position error small, throttle stable), the speed-PID I-term is a direct measure of the thrust holding position against drift:  
  
```cpp  
// assumes bow-pull thruster convention: thrust dir = yaw + steer angle.  
// for a stern-push or inverted-output thruster add +M_PI to thrust_rad.  
const float steer_ang_rad = (g2.motors.get_steering() / 4500.0f)  
                            * radians(g2.motors.get_vector_angle_max());  
const float thrust_rad = ahrs.get_yaw_rad() + steer_ang_rad;  
// drift vector = opposite of the force that holds us in place  
meas_dir_ne = Vector2f{-cosf(thrust_rad), -sinf(thrust_rad)};  
// mode_loiter.cpp ~192-196  
```  
  
```cpp  
// clamp: if (I + FF) exceeds cruise throttle the powf() above would  
// extrapolate beyond the calibrated curve - cap at the full-throttle speed  
// ceiling, not cruise_speed, so currents stronger than cruise are still  
// representable  
const float drift_mag_mps = MIN(cruise_speed * powf((_drift_i_filt + ff_frac) / cruise_thr, 1.0f / expo),  
                                calc_speed_max(g.speed_cruise, 1.0f));  
// mode_loiter.cpp ~188-193  
```  
  
> **Depends on §4.6 fix:** the `/4500` normalization assumes `_steering ∈ ±4500`. Until the fix is committed, this estimator receives wrapped values whenever the heading PID saturates.  
  
Tunables: `LOIT_I_EQERR` (throttle-stability window 0.10), `LOIT_I_MIN` (noise floor 0.02), `LOIT_I_ALPHA` (EMA 0.10), `LOIT_I_DISAG` (method-1/method-2 disagreement handling 0.5). The inverse `powf(…, 1/expo)` maps I-term-fraction → equivalent speed using the same `ATC_SPD_EXPO` curve as the forward direction.  
  
### 5.4 Seeding on loiter entry  
  
Symmetric to `Mode::enter` (§4.3): seeds the loiter slot from the nav estimate, age-weighted, accepting seeded sources, guarded against overwriting a real loiter measurement (`loiter_has_real`).  
  
---  
  
## 6. `Rover/mode_guided.cpp`  
  
Two changes:  
  
**1. Boat stops via loiter, not throttle-cut.** Every "stop" path for boats routes through `start_loiter()` (falling back to `start_stop()`/`stop_vehicle()` if loiter init fails):  
  
```cpp  
if (rover.is_boat()) {  
    if (!start_loiter()) {  
        start_stop();  
    }  
} else {  
    start_stop();  
}  
// mode_guided.cpp ~7-13 (same pattern at destination reach, timeouts, etc.)  
```  
  
A 250 kg boat carries significant momentum and is drift-dominated — cutting throttle leaves it adrift; loiter actively holds position.  
  
**2. HeadingAndSpeed feeds estimator + applies compensation:**  
  
```cpp  
if (have_attitude_target) {  
    // feed the estimator with the *uncompensated* commanded vector - using  
    // corrected_heading_cd here would subtract our own correction out of the  
    // measured drift, systematically under-estimating it  
    update_drift_estimator(_desired_yaw_cd, _desired_speed);  
  
    // apply drift compensation to a local copy of the heading only -  
    // _desired_yaw_cd must stay clean (persistent commanded target, re-read  
    // every tick), otherwise the crab-angle correction would compound  
    // cycle-over-cycle since apply_drift_compensation() writes its result  
    // back into whatever variable is passed in  
    float corrected_heading_cd = _desired_yaw_cd;  
    float corrected_speed = _desired_speed;  
    apply_drift_compensation(corrected_heading_cd, corrected_speed);  
  
    calc_steering_to_heading(corrected_heading_cd);  
    calc_throttle(calc_speed_nudge(corrected_speed, is_negative(corrected_speed)), true);  
}  
// mode_guided.cpp ~60-77  
```  
  
The uncompensated-in / local-copy-out discipline is the key correctness invariant of the whole feature.  
  
---  
  
## 7. `Rover/GCS_MAVLink_Rover.cpp`  
  
`send_named_float` diagnostics on `MAVLINK_COMM_0` only (avoids N× duplication per channel): `DRIFTN`/`DRIFTE`/`DRIFTSPD` current estimate, `DRIFTNAGE`/`DRIFTNSED`/`DRIFTLAGE`/`DRIFTLSED` sample age (s) and seeded flag per slot — essential for A/B verification on water.  
  
---  
  
## 8. `extra_hwdef.dat` (repo root, new)  
  
Slim Rover build for a low-flash FC: ~470 feature `undef`s plus explicit `define … 0` — copter/plane modes, most rangefinder/mount/camera/OSD/CAN bindings, EKF2 (`HAL_NAVEKF2_AVAILABLE 0`), airspeed, and most GPS/compass drivers removed (kept: uBlox, NMEA, IST8310). Baro driver switched `AP_BARO_LPS2XH`→`AP_BARO_MS5611` in commit `0320e405`.  
  
---  
  
## 9. Parameter summary  
  
| Param | Location | Default | Purpose |  
|---|---|---|---|  
| `MOT_VEC_BLEND_THR` | AP_MotorsUGV | 0.3 | throttle fraction where angle blend switches direct→atan |  
| `MOT_VEC_DEADBAND` | AP_MotorsUGV | 0.03 | steering deadband, freezes last angle |  
| `MOT_VEC_RESID_TC` | AP_MotorsUGV | 0.5 s | throttle filter TC for blend weight |  
| `ATC_SPD_EXPO` | AR_AttitudeControl | 1.0 | speed↔throttle curve exponent (shared by FF, method-2, clamp) |  
| `DRIFT_GAIN_NAV` / `DRIFT_GAIN_LOIT` | AP_MotorsUGV | 1.0 | write-time gains; both 0 = full kill-switch |  
| `DRIFT_MAXAGE` | AP_MotorsUGV | 1800 s | shared staleness cutoff; 0 = never trust |  
| `MOT_DRIFT_EST_YAWR` | AP_MotorsUGV | 15 °/s | nav estimator yaw-rate gate; 0 = off |  
| `SFL_DB`/`SFL_GAIN`/`SFL_MAX`/`SFL_IFRZ` | AP_MotorsUGV | 10°/0.7/20%/45° | steering floor + I-freeze |  
| `LOIT_DRIFT_MIN` | AP_MotorsUGV | 0.03 m/s | anti-drift heading activation |  
| `LOIT_COAST_THR` | AP_MotorsUGV | 1 % | coast gate (method 1) |  
| `LOIT_I_EQERR`/`I_MIN`/`I_ALPHA`/`I_DISAG` | AP_MotorsUGV | 0.10/0.02/0.10/0.5 | method-2 window, noise floor, EMA, disagreement |  
| `LOIT_ROT_ANG`/`LOIT_ROT_RATE` | AP_MotorsUGV | 15°/20 °/s | rotation gate (method 1); 0 = off |  
  
---  
  
## 10. Architectural notes for reviewers  
  
- **Kill-switch:** `DRIFT_GAIN_NAV=0 && DRIFT_GAIN_LOIT=0` restores exact stock behavior — `get_drift_compensation_body()` returns false and `calc_throttle` runs the legacy path. Estimators keep filling (useful for A/B logging comparison).  
- **Estimator feedback hygiene:** every estimator input is the uncompensated command; every compensation output is applied to a local copy. Violating either rule creates a subtraction loop that under-estimates drift.  
- **Seed safety:** seeds carry original timestamps → guaranteed aging; `seed_weight ≤ 1` → ping-pong decays; real estimates always outrank seeds; same-tick echo blocked by `DRIFT_SEED_MIN_AGE_MS`; real-destination overwrite blocked by the `*_has_real` guard.  
- **Units discipline:** speeds m/s, headings centi-degrees, throttle 0–100 % at `calc_throttle` level vs −1..+1 inside `AR_AttitudeControl`, drift estimates always earth-frame NE.  
- **Normalized steering contract (post-fix):** once §3.3 + §4.6 are committed, `get_steering_out_rate` returns ±1 and `_steering` is always ±4500 centidegrees. Every consumer of `_steering` — vectored blend (§2.1), method-2 estimator (§5.3), `STER.SteerOut` logging — may rely on that range. Until then, treat `_steering` as untrusted under PID saturation.  
  
## 11. Known-open items (deliberate)  
  
| Item | Status |  
|---|---|  
| Speed-PID I-freeze hold-vs-decay | reviewed — current hold-or-decay is **correct**, see §4.4 |  
| `MOT_VEC_ANGLEMAX=90` boost spike near 89° | kept — clamping removes turn authority at speed |  
| Follow mode: no estimator feed | Follow→Loiter starts cold; documented |  
| Slew-rate limiting on vectored angle | optional future improvement if water tests show a throttle stumble |  
| Nonlinear thrust effectiveness in rotation | at δ=90° full throttle, first ~10 deg/s overshoot then G-limits clamp at ~45; future allocator should dose N-moment via T·sin(δ), not flat throttle shaping |  
| SFL_PIVOT semantics | meant as full-throttle pivot capability (default ~100%), not scaled off SFL_MAX; rotation limit should come from G-limit + physics only |  
| Yaw rate cap during pivot | at δ=90°, T=100% the turn-rate loop has no lever left — rate is set by hull drag/G only; verify water test shows acceptable peak deg/s | 
| N→T spill allocation | when δ saturates (e.g. ANGLEMAX=90 pivot), residual  
  yaw demand is dropped by limit flags today; allocator should convert it  
  into throttle (N = T·sinδ). Inverse: when yaw rate is already high,  
  allocator should close δ and return cosδ to forward thrust. This is the  
  missing axis of the vectored control — current atan-boost approximates  
  it indirectly via 1/cos(δ) scaling |  
| Nonlinear thrust effectiveness (rotation) | first ~10 deg/s at δ=90°  
  is a spike (same physics as the 1.8-expo fix for forward accel);  
  allocator should dose yaw moment via T·sin(δ) ramp, not flat T |
  | Rotation braking | T<0 at held saturated δ (never δ-flip a running  
  thruster). Scale by |ω| demand. Followed by T→0, δ close, T>0 exit.  
  nav_mode must NOT block this: braking ≠ stern travel |  
| Reverse-gate definition | stern travel = sustained T<0 with |v|<eps  
  AND |δ|<eps. T<0 opposing v or ω is braking — always legal |  
| `(int16_t)` wrap in `Mode::set_steering` | **root-caused, fix prepared — pending commit** (§4.6 + §3.3). Upstream line: re-apply the constrain after any rebase. |  
| `stop_vehicle` bypasses `Mode::set_steering` | harmless today (output clamp at `:1130`); unify after §4.6 lands (§4.7) |  
  
---  
  
*Caveat: line numbers were taken from git HEAD blame/diffs; the repo search index lags HEAD so exact lines may drift ±a few. `extra_hwdef.dat` and parameter default values were summarized rather than read line-by-line — verify defaults against `AP_MotorsUGV.cpp` `AP_GROUPINFO` table before publishing externally.*  
  
[![Discord](https://img.shields.io/discord/674039678562861068.svg)](https://ardupilot.org/discord)
