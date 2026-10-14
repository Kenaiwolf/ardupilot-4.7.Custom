/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsUGV.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_AHRS/AP_AHRS.h>

#define SERVO_MAX 4500  // This value represents 45 degrees and is just an arbitrary representation of servo max travel.

extern const AP_HAL::HAL& hal;

// singleton instance
AP_MotorsUGV *AP_MotorsUGV::_singleton;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsUGV::var_info[] = {
    // @Param: PWM_TYPE
    // @DisplayName: Motor Output PWM type
    // @Description: This selects the output PWM type as regular PWM, OneShot, Brushed motor support using PWM (duty cycle) with separated direction signal, Brushed motor support with separate throttle and direction PWM (duty cycle)
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:BrushedWithRelay,4:BrushedBiPolar,5:DShot150,6:DShot300,7:DShot600,8:DShot1200
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 1, AP_MotorsUGV, _pwm_type, PWMType::NORMAL),

    // @Param: PWM_FREQ
    // @DisplayName: Motor Output PWM freq for brushed motors
    // @Description: Motor Output PWM freq for brushed motors
    // @Units: kHz
    // @Range: 1 20
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_FREQ", 2, AP_MotorsUGV, _pwm_freq, 16),

    // @Param: SAFE_DISARM
    // @DisplayName: Motor PWM output disabled when disarmed
    // @Description: Disables motor PWM output when disarmed
    // @Values: 0:PWM enabled while disarmed, 1:PWM disabled while disarmed
    // @User: Advanced
    AP_GROUPINFO("SAFE_DISARM", 3, AP_MotorsUGV, _disarm_disable_pwm, 0),

    // @Param: THR_MIN
    // @DisplayName: Throttle minimum
    // @Description: Throttle minimum percentage the autopilot will apply. This is useful for handling a deadzone around low throttle and for preventing internal combustion motors cutting out during missions. Must be less than MOT_THR_MAX.
    // @Units: %
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MIN", 5, AP_MotorsUGV, _throttle_min, 0),

    // @Param: THR_MAX
    // @DisplayName: Throttle maximum
    // @Description: Throttle maximum percentage the autopilot will apply. This can be used to prevent overheating an ESC or motor on an electric rover
    // @Units: %
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX", 6, AP_MotorsUGV, _throttle_max, 100),

    // @Param: SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: Throttle slew rate as a percentage of total range per second. A value of 100 allows the motor to change over its full range in one second.  A value of zero disables the limit.  Note some NiMH powered rovers require a lower setting of 40 to reduce current demand to avoid brownouts.
    // @Units: %/s
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SLEWRATE", 8, AP_MotorsUGV, _slew_rate, 100),

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Thrust curve exponent (-1 to +1 with 0 being linear)
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("THST_EXPO", 9, AP_MotorsUGV, _thrust_curve_expo, 0.0f),

    // 10 was VEC_THR_BASE

    // @Param: SPD_SCA_BASE
    // @DisplayName: Motor speed scaling base speed
    // @Description: Speed above which steering is scaled down when using regular steering/throttle vehicles.  zero to disable speed scaling
    // @Units: m/s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("SPD_SCA_BASE", 11, AP_MotorsUGV, _speed_scale_base, 1.0f),

    // @Param: STR_THR_MIX
    // @DisplayName: Motor steering vs throttle prioritisation
    // @Description: Steering vs Throttle priorisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid for Skid Steering vehicles
    // @Range: 0.2 1.0
    // @User: Advanced
    AP_GROUPINFO("STR_THR_MIX", 12, AP_MotorsUGV, _steering_throttle_mix, 0.5f),

    // @Param: VEC_ANGLEMAX
    // @DisplayName: Vector thrust angle max
    // @Description: The angle between steering's middle position and maximum position when using vectored thrust (boats only)
    // @Units: deg
    // @Range: 0 90
    // @User: Standard
    AP_GROUPINFO("VEC_ANGLEMAX", 13, AP_MotorsUGV, _vector_angle_max, 0.0f),

    // @Param: THST_ASYM
    // @DisplayName: Motor Thrust Asymmetry
    // @Description: Thrust Asymmetry. Used for skid-steering. 2.0 means your motors move twice as fast forward than they do backwards.
    // @Range: 1.0 10.0
    // @User: Advanced
    AP_GROUPINFO("THST_ASYM", 14, AP_MotorsUGV, _thrust_asymmetry, 1.0f),

    // @Param: REV_DELAY
    // @DisplayName: Motor reversal delay
    // @Description: For reversible motors that need a delay before they can change direction. When greater than zero the throttle will go to zero for this amount of time before outputting the new throttle when the demanded motor direction changes.
    // @Units: s
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("REV_DELAY", 15, AP_MotorsUGV, _reverse_delay, 0),

#if AP_BATTERY_WATT_MAX_ENABLED
    // @Param: BAT_WATT_TC
    // @DisplayName: Battery power limiting time constant
    // @Description: Time constant used to limit the smooth power throttling.
    // @Range: 0 10
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("BAT_WATT_TC", 16, AP_MotorsUGV, _batt_power_time_constant, 2.0f),
#endif

    // @Param: VEC_DEADBAND
    // @DisplayName: Vectored thrust deadband
    // @Description: Commanded steering/throttle magnitude below which the vectored angle is frozen (anti-jitter near zero throttle)
    // @Range: 0.0 0.2
    // @User: Advanced
    AP_GROUPINFO("VEC_DEADBAND", 17, AP_MotorsUGV, _vec_deadband, 0.03f),

    // @Param: VEC_BLND_THR
    // @DisplayName: Vectored thrust blend throttle
    // @Description: Filtered throttle below which the vectored angle blends from direct steering-proportional angle into full atan(); 0 = legacy atan() always
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("VEC_BLND_THR", 18, AP_MotorsUGV, _vec_blend_thr, 0.15f),

    // @Param: VEC_RESID_TC
    // @DisplayName: Vectored thrust throttle filter time constant
    // @Description: Time constant of the low-pass filter applied to throttle before it selects/blends the vectored-thrust regime
    // @Units: s
    // @Range: 0.0 5.0
    // @User: Advanced
    AP_GROUPINFO("VEC_RESID_TC", 19, AP_MotorsUGV, _vec_resid_tc, 0.5f),

    // not reused - members _cur_est_tc/_cur_est_blend no longer exist (replaced
    // by per-source write-time gain + shared max-age below). Reusing a param
    // index would silently reinterpret old EEPROM values as a different param.

    // @Param: DRIFT_G_LOIT
    // @DisplayName: Loiter-sourced drift compensation gain
    // @Description: Gain applied to Loiter-sourced drift samples at write-time. 0 disables this source, 1 = full, 2 = double compensation
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("DRIFT_G_LOIT", 23, AP_MotorsUGV, _drift_comp_gain_loiter, 1.0f),

    // @Param: DRIFT_G_NAV
    // @DisplayName: Nav-sourced drift compensation gain
    // @Description: Gain applied to nav-mode (Auto/Guided/RTL) drift samples at write-time. 0 disables this source, 1 = full, 2 = double compensation
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("DRIFT_G_NAV", 24, AP_MotorsUGV, _drift_comp_gain_nav, 1.0f),

    // @Param: DRIFT_MAXAGE
    // @DisplayName: Drift estimate maximum age
    // @Description: Staleness cutoff (s) shared by both drift sources. If no estimate was updated within this time, correction expires (safety for dead estimators / long manual). 0 = never trust any estimate
    // @Units: s
    // @Range: 0 3600
    // @User: Advanced
    AP_GROUPINFO("DRIFT_MAXAGE", 25, AP_MotorsUGV, _drift_max_age_s, 1800.0f),

    // @Param: SFL_DB
    // @DisplayName: Steering-floor deadband
    // @Description: Heading error (deg) below which the vectored-thrust steering-to-throttle floor injects no throttle
    // @Units: deg
    // @Range: 0 45
    // @User: Advanced
    AP_GROUPINFO("SFL_DB", 26, AP_MotorsUGV, _sfl_deadband_deg, 10.0f),

    // @Param: SFL_GAIN
    // @DisplayName: Steering-floor gain
    // @Description: Floor throttle percent per degree of heading error above the deadband [%/deg]
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("SFL_GAIN", 27, AP_MotorsUGV, _sfl_gain, 0.7f),

    // @Param: SFL_MAX
    // @DisplayName: Steering-floor maximum
    // @Description: Maximum throttle percent the steering-floor is allowed to inject
    // @Units: %
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("SFL_MAX", 28, AP_MotorsUGV, _sfl_max_pct, 20.0f),

    // @Param: SFL_IFRZ
    // @DisplayName: Steering-floor I-term freeze threshold
    // @Description: Heading error (deg) above which the speed PID integrator is frozen to prevent windup while rotating in place
    // @Units: deg
    // @Range: 10 90
    // @User: Advanced
    AP_GROUPINFO("SFL_IFRZ", 29, AP_MotorsUGV, _sfl_ifreeze_deg, 45.0f),

    // @Param: LOIT_DRF_MIN
    // @DisplayName: Loiter drift minimum magnitude
    // @Description: Drift estimate (m/s) below which the loiter anti-drift heading is not activated; deadband against GPS/estimate noise
    // @Units: m/s
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("LOIT_DRF_MIN", 30, AP_MotorsUGV, _loit_drift_min, 0.03f),

    // @Param: LOIT_CST_THR
    // @DisplayName: Loiter coasting throttle threshold
    // @Description: Pre-floor navigation throttle demand (%) below which the vehicle is considered coasting for method-1 drift sampling
    // @Units: %
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("LOIT_CST_THR", 31, AP_MotorsUGV, _loit_coast_thr, 1.0f),

    // @Param: LOIT_I_EQERR
    // @DisplayName: Loiter drift method-2 equilibrium error
    // @Description: Speed-PID error (m/s) below which the I-term is considered at drift equilibrium during the decel-to-stop window
    // @Units: m/s
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("LOIT_I_EQERR", 32, AP_MotorsUGV, _loit_i_eq_err, 0.10f),

    // @Param: LOIT_I_MIN
    // @DisplayName: Loiter drift method-2 minimum I-term
    // @Description: Minimum speed-PID I-term magnitude (0-1 throttle fraction) accepted as a valid drift sample; below this there is nothing to compensate
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("LOIT_I_MIN", 33, AP_MotorsUGV, _loit_i_min, 0.02f),

    // @Param: LOIT_I_ALPHA
    // @DisplayName: Loiter drift method-2 EMA alpha
    // @Description: EMA weight applied to each I-term sample during the equilibrium window; lower = smoother but slower
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("LOIT_I_ALPHA", 34, AP_MotorsUGV, _loit_i_alpha, 0.10f),

    // @Param: LOIT_I_DISAG
    // @DisplayName: Loiter drift method-2 disagreement gate
    // @Description: If a new estimate differs from the stored one by more than this fraction of its magnitude, blend only 25% of the way (NAV-style trend check)
    // @Range: 0.1 2.0
    // @User: Advanced
    AP_GROUPINFO("LOIT_I_DISAG", 35, AP_MotorsUGV, _loit_i_disagree, 0.5f),

    // @Param: LOIT_ROT_ANG
    // @DisplayName: Loiter drift method-1 rotation angle gate
    // @Description: Vectored-thruster deflection (deg) above which coast drift sampling is blocked. Rotating the hull about its pivot point produces real lateral IMU velocity (v = yaw_rate * pivot_radius) that would be misread as drift. 0 disables the gate
    // @Units: deg
    // @Range: 0 90
    // @User: Advanced
    AP_GROUPINFO("LOIT_ROT_ANG", 36, AP_MotorsUGV, _loit_rot_ang_deg, 15.0f),

    // @Param: LOIT_ROT_RAT
    // @DisplayName: Loiter drift method-1 yaw rate gate
    // @Description: Yaw rate (deg/s) above which coast drift sampling is blocked. During rotation the IMU orbits the pivot point (~0.6m) - e.g. 45deg/s = ~0.5m/s false drift. 0 disables the gate
    // @Units: deg/s
    // @Range: 0 90
    // @User: Advanced
    AP_GROUPINFO("LOIT_ROT_RAT", 37, AP_MotorsUGV, _loit_rot_rate_dps, 20.0f),

    // @Param: DRIFT_EST_YR
    // @DisplayName: Nav drift estimator yaw rate gate
    // @Description: Yaw rate (deg/s) above which the shared drift-estimator window (navigate_to_waypoint/Guided) is invalidated. Keep tighter than LOIT_ROT_RAT - the estimator needs long clean windows, not sample count
    // @Units: deg/s
    // @Range: 0 45
    // @User: Advanced
    AP_GROUPINFO("DRIFT_EST_YR", 38, AP_MotorsUGV, _drift_est_yaw_rate_dps, 15.0f),

    // @Param: VEC_FOSSEN
    // @DisplayName: Fossen-style vectored thrust unified allocator
    // @Description: NEED MOT_VEC_ANGLEMAX=90!!! 0 = stock blended atan/direct mapping in output_regular, legacy steering-thrust floor (SFL_*) and cosine throttle reduction remain active; 1 = unified allocator mapping (surge force, yaw moment) to thruster angle+throttle. Braking-only reverse policy for autonomous modes is enforced inside the allocator. Mutually exclusive with SFL floor/cosine reduction - enabling this automatically disables them to avoid double throttle compensation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("VEC_FOSSEN", 39, AP_MotorsUGV, _vec_fossen, 0),

    // @Param: SFL_PIVOT
    // @DisplayName: Steering-floor pivot level
    // @Description: Minimum floor throttle (%) while heading error exceeds SFL_IFRZ. At pivot-scale errors the regular floor (~11%) only yields a few deg/s; during an in-place turn all thruster authority is available for rotation anyway. 0 disables
    // @Units: %
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("SFL_PIVOT", 40, AP_MotorsUGV, _sfl_pivot_pct, 0.0f),

    // @Param: VEC_BRK
    // @DisplayName: Vectored rotation brake scale
    // @Description: Scaling of the rotation-brake reverse thrust (%) in the unified allocator. 100 = nominal (brake magnitude = |n| x this scale, further multiplied by VEC_BRK_ASYM); 0 disables the brake; >100 allows stronger-than-nominal counter-thrust. Only active when the servo is at full deflection and the PID demands opposite rotation
    // @Units: %
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("VEC_BRK", 41, AP_MotorsUGV, _vec_brk_pct, 100.0f),

    // @Param: VEC_YAW_EXP
    // @DisplayName: Vectored yaw thrust linearization exponent
    // @Description: Inverse exponent (1/n) applied to the yaw-dominant throttle demand in the unified allocator to compensate measured nonlinear throttle->turn-rate response (fitted from log data, TurnRate_frac = ThrOut_frac^n). 1.0 = no linearization (raw demand passed through)
    // @Range: 1.0 3.0
    // @User: Advanced
    AP_GROUPINFO("VEC_YAW_EXP", 42, AP_MotorsUGV, _vec_yaw_lin_expo, 1.67f),

    // @Param: VEC_BRK_ASYM
    // @DisplayName: Vectored braking asymmetry multiplier
    // @Description: Multiplier applied to counter-thrust (braking) demand in the unified allocator, both rotation-brake and linear surge braking, to compensate reverse thrust being less efficient than forward thrust for the same deceleration. 1.0 = symmetric/off
    // @Range: 1.0 2.0
    // @User: Advanced
    AP_GROUPINFO("VEC_BRK_ASYM", 43, AP_MotorsUGV, _vec_brk_asym, 1.24f),

    // @Param: VEC_COS_FLR
    // @DisplayName: Vectored cosine-singularity floor angle
    // @Description: Angular margin (deg) from the mechanical 90 deg stop within which the cos(delta)-based magnitude floor engages, preventing the division/magnitude blow-up as delta approaches 90 deg. 0 disables (legacy 1e-3f hard threshold behavior)
    // @Units: deg
    // @Range: 0.0 30.0
    // @User: Advanced
    AP_GROUPINFO("VEC_COS_FLR", 44, AP_MotorsUGV, _vec_cos_flr_deg, 10.0f),

	// @Param: VEC_BLD_STRT
    // @DisplayName: Vectored throttle-blend start angle
    // @Description: Steering angle (deg) below which throttle is pure surge/yaw geometric decomposition (VEC_YAW_EXP has zero effect).
    // @Units: deg
    // @Range: 0.0 90.0
    // @User: Advanced
    AP_GROUPINFO("VEC_BLD_STRT", 45, AP_MotorsUGV, _vec_bld_strt_deg, 70.0f),

	// @Param: YAW_CRS_RATE
    // @DisplayName: Yaw pivot cruise rate reference
    // @Description: Reference yaw rate (deg/s) used to calibrate pivot-mode throttle in the unified allocator, analogous to SPEED_CRUISE for forward motion. Paired with YAW_CRS_THR: at this rate, pivot throttle = YAW_CRS_THR before VEC_YAW_EXP shaping. Should be a rate comfortably inside normal PID authority, not ATC_STR_RAT_MAX
    // @Units: deg/s
    // @Range: 1.0 60.0
    // @User: Advanced
    AP_GROUPINFO("YAW_CRS_RATE", 46, AP_MotorsUGV, _yaw_crs_rate_dps, 16.0f),

    // @Param: YAW_CRS_THR
    // @DisplayName: Yaw pivot cruise throttle reference
    // @Description: Throttle (%) required to sustain YAW_CRS_RATE in pivot mode, analogous to THR_CRUISE. Paired with YAW_CRS_RATE and shaped by VEC_YAW_EXP to derive pivot throttle for other commanded turn rates
    // @Units: %
    // @Range: 1.0 100.0
    // @User: Advanced
    AP_GROUPINFO("YAW_CRS_THR", 47, AP_MotorsUGV, _yaw_crs_thr_pct, 49.0f),

    // @Param: VEC_TRN_MS
    // @DisplayName: Vectored servo-transit suppression window
    // @Description: Throttle suppression window (ms) covering the servo sweep from one +-90deg stop through center to the opposite stop, started after a yaw-agreement brake release or a demanded-side flip from saturation. Any thrust during the sweep lands partly forward and contaminates the pivot. Set from measured full-to-full servo transit time. 0 disables suppression entirely
    // @Units: ms
    // @Range: 0 3000
    // @User: Advanced
    AP_GROUPINFO("VEC_TRN_MS", 48, AP_MotorsUGV, _vec_transit_ms, 800),


    AP_GROUPEND
};

AP_MotorsUGV::AP_MotorsUGV(AP_WheelRateControl& rate_controller) :
    _rate_controller(rate_controller)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
    _loiter_estimate_ne.zero();
    _loiter_estimate_ms = 0;
    _nav_estimate_ne.zero();
    _nav_estimate_ms = 0;
    _loiter_estimate_is_seeded = false;
    _nav_estimate_is_seeded = false;
    _vec_throttle_filt = 0.0f;
    _vec_steering_filt = 0.0f;
    _vec_last_steering_angle_rad = 0.0f;
    _vec_last_w = 1.0f;
}

// set a Loiter-sourced current+wind drift estimate (m/s, North/East).  gain
// (DRIFT_G_LOIT) applied here, at write-time, so the stored
// value is already the final, PID-ready corrected vector
void AP_MotorsUGV::set_loiter_estimate_ne(const Vector2f &drift_ne)
{
    const float gain = constrain_float(_drift_comp_gain_loiter, 0.0f, 2.0f);
    _loiter_estimate_ne = drift_ne * gain;
    _loiter_estimate_ms = AP_HAL::millis();
    _loiter_estimate_is_seeded = false;
}

// set a Guided-sourced current+wind drift estimate (m/s, North/East).  gain
// (DRIFT_G_NAV) applied here, at write-time, so the stored
// value is already the final, PID-ready corrected vector
void AP_MotorsUGV::set_nav_estimate_ne(const Vector2f &drift_ne)
{
    const float gain = constrain_float(_drift_comp_gain_nav, 0.0f, 2.0f);
    _nav_estimate_ne = drift_ne * gain;
    _nav_estimate_ms = AP_HAL::millis();
    _nav_estimate_is_seeded = false;
}

// seed the Loiter slot - source_ms preserves the original measurement time
void AP_MotorsUGV::seed_loiter_estimate_ne(const Vector2f &drift_ne, uint32_t source_ms)
{
    _loiter_estimate_ne = drift_ne;
    _loiter_estimate_ms = source_ms;
    _loiter_estimate_is_seeded = true;
}

// seed the nav/Guided slot - source_ms preserves the original measurement time
void AP_MotorsUGV::seed_nav_estimate_ne(const Vector2f &drift_ne, uint32_t source_ms)
{
    _nav_estimate_ne = drift_ne;
    _nav_estimate_ms = source_ms;
    _nav_estimate_is_seeded = true;
}

bool AP_MotorsUGV::get_loiter_estimate_ne(Vector2f &drift_ne, uint32_t &age_ms, bool &is_seeded) const
{
    if (_loiter_estimate_ms == 0) {
        return false;
    }
    age_ms = AP_HAL::millis() - _loiter_estimate_ms;
    drift_ne = _loiter_estimate_ne;
    is_seeded = _loiter_estimate_is_seeded;
    return true;
}

// raw Guided-sourced estimate + age (ms). false if never set
bool AP_MotorsUGV::get_nav_estimate_ne(Vector2f &drift_ne, uint32_t &age_ms, bool &is_seeded) const
{
    if (_nav_estimate_ms == 0) {
        return false;
    }
    age_ms = AP_HAL::millis() - _nav_estimate_ms;
    drift_ne = _nav_estimate_ne;
    is_seeded = _nav_estimate_is_seeded;
    return true;
}

// return the more recently-updated of the two per-source drift estimates
// (gain already applied at write-time in set_loiter_estimate_ne()/
// set_nav_estimate_ne()).  returns false if neither source has ever been
// set, or if the newer of the two is older than DRIFT_MAXAGE seconds
// (correction fully off - e.g. wind/current stopped being measured)
bool AP_MotorsUGV::get_current_estimate_ne(Vector2f &current_ne) const
{
    const uint32_t now_ms = AP_HAL::millis();

    const uint32_t loiter_age_ms = (_loiter_estimate_ms == 0) ? UINT32_MAX : (now_ms - _loiter_estimate_ms);
    const uint32_t nav_age_ms = (_nav_estimate_ms == 0) ? UINT32_MAX : (now_ms - _nav_estimate_ms);

    if ((loiter_age_ms == UINT32_MAX) && (nav_age_ms == UINT32_MAX)) {
        // neither source has ever provided a sample
        return false;
    }

    // a seeded estimate is only a handoff placeholder, not a real measurement -
    // a genuinely-measured estimate always wins over a seeded one regardless
    // of timestamp. between two seeds or two real samples, newer wins.
    const bool loiter_real = (loiter_age_ms != UINT32_MAX) && !_loiter_estimate_is_seeded;
    const bool nav_real    = (nav_age_ms    != UINT32_MAX) && !_nav_estimate_is_seeded;

    bool use_loiter;
    if (loiter_real && !nav_real) {
        use_loiter = true;
    } else if (nav_real && !loiter_real) {
        use_loiter = false;
    } else {
        use_loiter = loiter_age_ms <= nav_age_ms;
    }
    const uint32_t chosen_age_ms = use_loiter ? loiter_age_ms : nav_age_ms;

    if (!is_positive(_drift_max_age_s) || (chosen_age_ms > uint32_t(_drift_max_age_s * 1000.0f))) {
        // shared staleness cutoff: neither source is recent enough to trust
        return false;
    }

    current_ne = use_loiter ? _loiter_estimate_ne : _nav_estimate_ne;
    return true;
}

// unified vectored-thrust allocator (VEC_FOSSEN=1). continuous allocation:
// the (fx_req, n_req) demand is one thrust vector - its direction selects the
// thruster angle, its magnitude and the dominant axis select throttle. no
// per-regime states: atan2 handles 0/45/90 deg continuously, saturation folds
// residual demand into the other channel instead of dropping it.
void AP_MotorsUGV::vectored_allocate(float fx_req, float n_req, float ground_speed,
                                     bool nav_mode, float &steering_cd, float &throttle_pct, float dt)
{
    // measured yaw rate (rad/s, earth frame = body yaw rate on a boat)
    const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();

    const float fx = constrain_float(fx_req, -1.0f, 1.0f);
    const float n  = constrain_float(n_req, -1.0f, 1.0f);
    const float vector_angle_max_rad = radians(constrain_float(_vector_angle_max, 0.0f, 90.0f));

    // low-pass filter the demand vector to suppress atan2 noise amplification
    // near zero throttle (same purpose as the legacy filter). scoped to the
    // same low-throttle band as VEC_BLND_THR/the atan-fix blend below: full
    // VEC_RESID_TC near zero throttle, fading to zero (no filtering, no
    // phase lag) at/above VEC_BLND_THR. uses raw fx (not yet filtered this
    // tick) since _vec_throttle_filt is what this block is computing
    float resid_tc_scaled = _vec_resid_tc;
    if (is_positive(_vec_blend_thr)) {
        // S-shaped ramp (same sin^2 shape as the pivot blend below), not
        // linear: flatter near both ends, steeper mid-transition, so the
        // filter strength doesn't kink at the VEC_BLND_THR boundary
        const float tc_ratio = constrain_float(fabsf(fx) / _vec_blend_thr, 0.0f, 1.0f);
        const float tc_scale = sq(cosf(tc_ratio * M_PI_2));    // smooth S-shape: slope=0 at both ends, no kink
        resid_tc_scaled *= tc_scale;
    }
    if (is_positive(resid_tc_scaled)) {
        const float alpha = constrain_float(dt / (resid_tc_scaled + dt), 0.0f, 1.0f);
        _vec_throttle_filt += (fx - _vec_throttle_filt) * alpha;
        _vec_steering_filt += (n - _vec_steering_filt) * alpha;
    } else {
        _vec_throttle_filt = fx;
        _vec_steering_filt = n;
    }

    const float vec_mag = sqrtf(sq(_vec_steering_filt) + sq(_vec_throttle_filt));

    float steering_angle_rad;
    float throttle_norm;

    {
        // desired thrust direction in (surge, yaw) space
        float delta = atan2f(_vec_steering_filt, _vec_throttle_filt);

        // fold demands pointing aft into the +/-90 deg envelope with negative
        // thrust (reverse). reverse reachable only via this fold: fx<0 while
        // moving = braking, and the nav_mode gate above zeroes it at rest.
        // hysteresis on the fold boundary: without it, a demand hovering at
        // ~90deg jitters delta between +89.9/-89.9, which swings the servo
        // ~180deg and flips throttle sign every tick. fold in only past
        // 90+2deg, fold out only below 90-2deg; a pure geometric boundary,
        // so unlike the rotation brake the hysteresis cannot delay a real
        // response - worst-case angle quantization error is 2deg, and it is
        // the only protection here when VEC_RESID_TC=0 (no input filtering)
        const float fold_eps_rad = radians(2.0f);
        if (_vec_folded) {
            // unfold test on the RAW delta: demand must return inside
            // +/-88deg (forward hemisphere) before we stop folding
            if (fabsf(delta) < M_PI_2 - fold_eps_rad) {
                _vec_folded = false;
            } else {
                delta = is_positive(delta) ? (delta - M_PI) : (delta + M_PI);
            }
        } else if (fabsf(delta) > M_PI_2 + fold_eps_rad) {
            delta = is_positive(delta) ? (delta - M_PI) : (delta + M_PI);
            _vec_folded = true;
        }
        const float atan_angle_rad = constrain_float(delta, -vector_angle_max_rad, vector_angle_max_rad);

        // ported legacy atan-fix (formerly output_regular()-only, VEC_FOSSEN=0
        // path): near zero throttle, atan2's angle is dominated by noise on
        // the small steering/throttle ratio (hull roll/pitch, sensor noise).
        // blend toward a direct angle proportional to steering demand only,
        // same VEC_BLND_THR threshold/mechanism as the legacy blend, now
        // applied inside the Fossen allocator instead of being unavailable
        float direct_angle_rad = constrain_float(_vec_steering_filt, -1.0f, 1.0f) * vector_angle_max_rad;
        if (is_negative(_vec_throttle_filt)) {
            direct_angle_rad = -direct_angle_rad;
        }
        float w_lowthr;
        if (is_positive(_vec_blend_thr)) {
            // same sin^2 S-curve as the pivot blend (and the VEC_RESID_TC
            // scaling above), replacing the previous linear ramp
            const float lowthr_ratio = constrain_float(fabsf(_vec_throttle_filt) / _vec_blend_thr, 0.0f, 1.0f);
            w_lowthr = sq(cosf(lowthr_ratio * M_PI_2));    // smooth S-shape, same family as the pivot blend
        } else {
            w_lowthr = 0.0f;
        }
        steering_angle_rad = w_lowthr * direct_angle_rad + (1.0f - w_lowthr) * atan_angle_rad;

        // magnitude: fx/cos(delta) generalizes vec_mag to the case where the
        // angle clamp above actually engaged (VEC_ANGLEMAX<90): if delta was
        // NOT clamped, fx = vec_mag*cos(delta) exactly (atan2 construction),
        // so this collapses to vec_mag with zero behavior change at the
        // current MOT_VEC_ANGLEMAX=90 default. cos_safe prevents the
        // division singularity as |delta| -> 90 deg; VEC_COS_FLR=0
        // reproduces the legacy 1e-3f hard-threshold behavior.
        const float cos_d = cosf(steering_angle_rad);
        const float cos_floor = sinf(radians(_vec_cos_flr_deg));  // cos(90-floor_deg)
        const float cos_safe = fmaxf(fabsf(cos_d), is_positive(_vec_cos_flr_deg) ? cos_floor : 1e-3f);
        const float t_vector = _vec_throttle_filt / copysignf(cos_safe, cos_d);

        // VEC_YAW_EXP reattachment: two-zone blend weight w. Below
        // VEC_BLD_START, w=0 (pure surge-cos(delta) geometry, VEC_YAW_EXP
        // has zero effect). Between VEC_BLD_STRT and the derived blend-end
        // (90-VEC_COS_FLR -- the same angle where the cosine floor itself
        // starts engaging), w ramps 0->1 via sin^2 (no slope discontinuity
        // at either boundary). Above blend-end, w=1 (full pivot-style
        // throttle). Blend-end is derived, not a separate tunable, so the
        // two angles the floor and the blend key off cannot drift apart.
        const float bld_end_deg = 90.0f - _vec_cos_flr_deg;
        const float bld_start_deg = MIN(_vec_bld_strt_deg, bld_end_deg - 0.1f);
        const float angle_deg = degrees(fabsf(steering_angle_rad));
        const float angle_ratio = (bld_end_deg > bld_start_deg) ?
            constrain_float((angle_deg - bld_start_deg) / (bld_end_deg - bld_start_deg), 0.0f, 1.0f) : 1.0f;
        const float w = sq(sinf(angle_ratio * M_PI_2));
        const float pivot_expo = MAX(_vec_yaw_lin_expo, 1.0f);
        float t_pivot;
        if (is_positive(_yaw_crs_rate_dps) && is_positive(_yaw_crs_thr_pct)) {
            // rate_ratio: n is already a normalized [-1,1] fraction of full
            // yaw demand (constrained above), so n=1 IS the YAW_CRS_RATE
            // calibration point by definition - no unit conversion needed.
            // (previous version incorrectly divided n by _yaw_crs_rate_dps,
            // a deg/s value, collapsing rate_ratio toward ~0 even at n=1
            // and effectively defeating the YAW_CRS_THR ceiling)
            const float rate_ratio = fabsf(n);
            t_pivot = copysignf((_yaw_crs_thr_pct * 0.01f) * powf(rate_ratio, pivot_expo), n);
        } else {
            t_pivot = vec_mag;
        }
        throttle_norm = t_vector * (1.0f - w) + t_pivot * w;

        // braking-demand asymmetry: reverse thrust is less efficient than
        // forward for the same deceleration, so counter-thrust demand needs
        // to be scaled UP (not down) by VEC_BRK_ASYM, measured from log data.
        // applies identically to rotation brake (below) and linear surge
        // braking (this block) - same physical motor, same asymmetry
        const float brk_asym = MAX(_vec_brk_asym, 1.0f);

        // linear surge brake-asymmetry: not at full deflection (surge-dominant,
        // not yaw-dominant) and reverse thrust is being commanded -> apply the
        // same forward/reverse efficiency correction as the rotation brake.
        // no need to distinguish "braking" from "actually reversing": the
        // motor-efficiency asymmetry is identical in both cases, so gating on
        // ground_speed sign-mismatch only created a dead zone (while cruising
        // in reverse, ground_speed and throttle_norm are both negative, so the
        // old condition never fired even though the same correction applies).
        // same autopilot-only gate as rotation brake
        if ((nav_mode || _pid_steering) &&
            fabsf(steering_angle_rad) < vector_angle_max_rad - 1e-4f &&
            throttle_norm < 0.0f) {
            throttle_norm *= brk_asym;
        }

        // drop any latched brake / coast / debounce state when running
        // outside autopilot/pid-steering context (e.g. just after a mode
        // switch) - stale state must not survive a mode change
        if (!nav_mode && !_pid_steering) {
            _vec_rot_brake_active = false;
            _vec_rot_brake_cnt = 0;
            _vec_pivot_coast_ms = 0;
        }

        // rotation brake (Option B, angle-locked): flipping the angle to the
        // opposite side while the hull is still overshooting the commanded
        // rotation is the actual failure mode this guards against - a ~15kg
        // thruster traversing ~1.5s under full load, at the exact moment a
        // fast corrective reaction is needed, is a net-negative response.
        // instead: the instant the measured yaw rate opposes the commanded
        // direction (n) while at/near full deflection, LOCK steering_angle_rad
        // at its current value - immune to the atan2/direct-angle blend above
        // and to whatever the steering PID asks for - and brake with reverse
        // thrust proportional to |n| alone. n (not an approximated rad/s
        // turn rate) is used deliberately: n is a ratio quantity with no
        // reliable conversion to a physical rate without pulling in
        // assumptions (ATC_STR_RAT_MAX, YAW_CRS_RATE/YAW_CRS_THR) that were
        // calibrated for acceleration, not braking, and n alone is already
        // sufficient as the proportional weight. the lock releases only once
        // the measured yaw rate's sign agrees with n again (error crossed
        // back through zero) - normal steering then resumes from the
        // then-current n, with no intermediate state to unwind. brk_asym
        // always applies here: with the angle locked, this thrust can never
        // land on the "accidentally forward" side the way an unlocked flip
        // could - it is always genuine counter-rotation reverse thrust.
        // MOT_VEC_BRK scales the magnitude (0 = brake off). active in
        // autopilot modes (nav_mode) and in Acro (pid_steering); excluded in
        // Manual, which must stay a pure RC passthrough.
        // vec_mag >= _vec_deadband guard: without it, pure sensor/filter
        // noise at near-zero demand could set steering_angle_rad near +-90
        // via atan2 for one tick and latch the brake on noise alone - the
        // deadband S-blend below will suppress the *output* either way, but
        // the latch itself would persist into the next tick without this
        // side-flip coast trigger: demand flips sign while the previously
        // commanded angle sat at a stop (or during an active coast window -
        // restart covers a double-flip mid-transit). fires only with a pure
        // rotation demand: fx>0.03 means the pivot ended and suppression is
        // the wrong tool. gated by !_vec_rot_brake_active: during a lock the
        // servo is held, not transiting, and zeroing throttle would kill the
        // running brake. pivot-from-rest is safe: last angle is ~0, not a stop
        if (!_vec_rot_brake_active &&
            (nav_mode || _pid_steering) &&
            vec_mag >= _vec_deadband &&
            fabsf(n) > 0.05f &&
            fabsf(fx) <= 0.03f &&
            ((fabsf(_vec_last_steering_angle_rad) >= vector_angle_max_rad - 1e-4f &&
              is_positive(n) != is_positive(_vec_last_steering_angle_rad)) ||
             (_vec_pivot_coast_ms != 0 && is_positive(n) != is_positive(_vec_last_steering_angle_rad)))) {
            _vec_pivot_coast_ms = AP_HAL::millis();
        }

        // brake latch condition: yaw rate measurably OPPOSES n. the old
        // (n*yaw_rate) < +0.01 latched on any near-zero product - including
        // yaw~0 at pivot start from rest - producing latch->release->800ms
        // dead-throttle coast on every pivot from rest. now:
        // |yaw_rate| > ~0.5deg/s AND sign-opposed to n, for 3 consecutive
        // ticks (~60ms at 50Hz), rejecting single-sample gyro noise and
        // hull-wobble sign oscillation around zero. release stays on
        // n*yaw_rate >= 0: entry needs 3 opposed ticks, exit needs 1
        // agreeing tick - asymmetric hysteresis against chatter. n flipping
        // sign releases implicitly: during lock yaw opposes the old n, so
        // after the flip it agrees with the new n (product >= 0)
        const bool rot_brake_cond = (nav_mode || _pid_steering) &&
            vec_mag >= _vec_deadband &&
            fabsf(steering_angle_rad) >= vector_angle_max_rad - 1e-4f &&
            fabsf(n) > 0.05f &&
            fabsf(yaw_rate_rads) > radians(0.5f) &&
            (is_positive(yaw_rate_rads) != is_positive(n));

        if (rot_brake_cond) {
            _vec_rot_brake_cnt++;
        } else {
            _vec_rot_brake_cnt = 0;
        }

        if (!_vec_rot_brake_active && _vec_rot_brake_cnt >= 3) {
            _vec_rot_brake_cnt = 0;
            // re-lock on the SAME side as the previous lock proves the earlier
            // release was premature (servo never crossed center) - end the
            // transit suppression instantly so the brake can work. a re-lock
            // on the OPPOSITE side does NOT clear it: the servo still needs
            // the full ~0.8s transit window from the original release.
            if (_vec_pivot_coast_ms != 0 &&
                is_positive(steering_angle_rad) == is_positive(_vec_rot_brake_angle_rad)) {
                _vec_pivot_coast_ms = 0;
            }
            _vec_rot_brake_active = true;
            // lock onto the side the hull is rotating FROM (the overshoot
            // side) - reverse thrust through this angle produces genuine
            // counter-torque. with the 3-tick debounce, steering_angle_rad
            // may already sit on the post-flip (new n) side by latch time;
            // locking that side would make the "brake" accelerate the
            // overshoot. yaw_rate_rads is the authoritative side indicator
            _vec_rot_brake_angle_rad = copysignf(vector_angle_max_rad, yaw_rate_rads);
        } else if (_vec_rot_brake_active &&
                   ((n * yaw_rate_rads) >= 0.0f || fabsf(fx) > 0.03f)) {
            // release on EITHER: (a) measured rotation agrees with commanded
            // direction again - including implicitly after n changes sign,
            // since during the lock yaw opposed the old n and therefore
            // agrees with the new one - or (b) forward/aft
            // demand (fx) has reappeared - pivot has ended, atan2(n,fx) no
            // longer sits at/near the +-90deg envelope, so re-triggering the
            // instant this clears is not possible. fx=0.03 is a wide margin:
            // even at n=1 it already pulls delta to ~88 deg, well clear of
            // the vector_angle_max_rad-1e-4 trigger threshold. deliberately
            // instantaneous - no hysteresis/filter on this fx check, since a
            // real fx demand reappearing means the PID needs the angle back
            // immediately, and lock-release delay is the larger risk here
            _vec_rot_brake_active = false;
			_vec_rot_brake_cnt = 0;  // stale count during the lock would bypass
                                     // the debounce on re-entry (angle stays
                                     // pinned at the stop so cond keeps counting)
                                     // - each re-latch must re-earn its 3 ticks
            // start the servo-transit suppression window only on a
            // yaw-agreement release with no forward demand - the servo is
            // about to traverse +-90deg through center and any thrust during
            // that sweep lands partly forward. window length is VEC_TRN_MS;
            // a side-flip from saturation can also arm it (above)
            if (fabsf(fx) <= 0.03f) {
                _vec_pivot_coast_ms = AP_HAL::millis();
            }
        }

        if (_vec_rot_brake_active) {
            steering_angle_rad = _vec_rot_brake_angle_rad;
            const float brake_scale = _vec_brk_pct * 0.01f;
            const float brake_frac = fabsf(n) * brake_scale;
            throttle_norm = copysignf(brake_frac * brk_asym, n);
        }

        // servo-transit suppression: for VEC_TRN_MS after a yaw-agreement
        // brake release or a demanded-side flip from saturation, force
        // throttle to zero - the servo is still sweeping to
        // the opposite stop and any thrust through intermediate angles lands
        // partly forward, contaminating the pivot. placed AFTER the brake
        // block so an opposite-side re-lock inside the window is also held
        // at zero. cleared early only by a real fx demand (pivot over) or a
        // same-side re-lock (handled in the trigger branch above)
        if (_vec_pivot_coast_ms != 0) {
            if (fabsf(fx) > 0.03f ||
                _vec_transit_ms <= 0 ||
                (AP_HAL::millis() - _vec_pivot_coast_ms) >= (uint32_t)_vec_transit_ms.get()) {
                _vec_pivot_coast_ms = 0;
            } else {
                throttle_norm = 0.0f;
            }
        }

        // deadband S-curve blend: fade smoothly from the frozen hold-state
        // (last steering angle, zero throttle) up to the fully-computed
        // output above as vec_mag crosses VEC_DEADBAND, instead of the old
        // hard on/off switch - same sin^2 family as every other blend in
        // this function. w_db=0 well inside the deadband (pure hold, no
        // jitter reaches the servo/throttle), w_db=1 at/above VEC_DEADBAND
        // (identical to current behavior there, zero added lag)
        // gated by the rotation-brake lock: once _vec_rot_brake_active is
        // true, the brake block above has already pinned steering_angle_rad
        // to the locked angle and set its own throttle_norm independently -
        // the deadband blend must not re-touch either, or it would fade the
        // brake's reverse throttle/angle toward the frozen hold-state using
        // a magnitude (vec_mag) that has nothing to do with the brake's own
        // release condition (yaw-rate-error sign / fx). w_db=1 here is a
        // pass-through, not a judgment that the demand is large - it simply
        // defers entirely to the brake lock's own state.
        float w_db = 1.0f;
        if (!_vec_rot_brake_active && is_positive(_vec_deadband)) {
            const float db_ratio = constrain_float(vec_mag / _vec_deadband, 0.0f, 1.0f);
            w_db = sq(sinf(db_ratio * M_PI_2));
        }
        steering_angle_rad = w_db * steering_angle_rad + (1.0f - w_db) * _vec_last_steering_angle_rad;
        throttle_norm *= w_db;

        if (fabsf(steering_angle_rad) >= vector_angle_max_rad) {
            limit.steer_right = true;
            limit.steer_left = true;
        }
        if (fabsf(throttle_norm) > 1.0f) {
            throttle_norm = constrain_float(throttle_norm, -1.0f, 1.0f);
            limit.throttle_lower = true;
            limit.throttle_upper = true;
        }

        _vec_last_steering_angle_rad = steering_angle_rad;
    }

    // convert steering angle to steering output (centidegrees)
    steering_cd = steering_angle_rad / vector_angle_max_rad * 4500.0f;
    throttle_pct = throttle_norm * 100.0f;
}

void AP_MotorsUGV::init(uint8_t frtype)
{
    _frame_type = frame_type(frtype);

    // setup for omni vehicles
    if (_frame_type != FRAME_TYPE_UNDEFINED) {
        setup_omni();
    }

    // setup servo output
    setup_servo_output();

    // setup pwm type
    setup_pwm_type();

    // set safety output
    setup_safety_output();

}

bool AP_MotorsUGV::get_legacy_relay_index(int8_t &index1, int8_t &index2, int8_t &index3, int8_t &index4) const
{
    if (_pwm_type != PWMType::BRUSHED_WITH_RELAY) {
        // Relays only used if PWM type is set to brushed with relay
        return false;
    }

    // First relay is always used, throttle, throttle left or motor 1
    index1 = 0;

    // Second relay is used for right throttle on skid steer and motor 2 for omni
    if (have_skid_steering()) {
        index2 = 1;
    }

    // Omni can have a variable number of motors
    if (is_omni()) {
        // Omni has at least 3 motors
        index2 = 2;
        if (_motors_num >= 4) {
            index2 = 3;
        }
    }

    return true;
}

// setup output in case of main CPU failure
void AP_MotorsUGV::setup_safety_output()
{
    if (_pwm_type == PWMType::BRUSHED_WITH_RELAY) {
        // set trim to min to set duty cycle range (0 - 100%) to servo range
        // ignore servo revese flag, it is used by the relay
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttle, true);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleLeft, true);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleRight, true);
    }

    // stop sending pwm if main CPU fails
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
}

// setup servo output ranges
void AP_MotorsUGV::setup_servo_output()
{
    // k_steering are limited to -45;45 degree
    SRV_Channels::set_angle(SRV_Channel::k_steering, SERVO_MAX);

    // k_throttle are in power percent so -100 ... 100
    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);

    // skid steering left/right throttle as -1000 to 1000 values
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  1000);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 1000);

    // omni motors set in power percent so -100 ... 100
    for (uint8_t i=0; i<AP_MOTORS_NUM_MOTORS_MAX; i++) {
        SRV_Channel::Function function = SRV_Channels::get_motor_function(i);
        SRV_Channels::set_angle(function, 100);
    }

    // mainsail range from 0 to 100
    SRV_Channels::set_range(SRV_Channel::k_mainsail_sheet, 100);
    // wing sail -100 to 100
    SRV_Channels::set_angle(SRV_Channel::k_wingsail_elevator, 100);
    // mast rotation -100 to 100
    SRV_Channels::set_angle(SRV_Channel::k_mast_rotation, 100);

}

// set steering as a value from -4500 to +4500
//   apply_scaling should be set to false for manual modes where
//   no scaling by speed or angle should be performed
void AP_MotorsUGV::set_steering(float steering, bool apply_scaling)
{
    _steering = steering;
    _scale_steering = apply_scaling;
}

// set throttle as a value from -100 to 100
void AP_MotorsUGV::set_throttle(float throttle)
{
    // only allow setting throttle if armed
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // check throttle is between -_throttle_max and  +_throttle_max
    _throttle = constrain_float(throttle, -_throttle_max, _throttle_max);
}

// set lateral input as a value from -100 to +100
void AP_MotorsUGV::set_lateral(float lateral)
{
    _lateral = constrain_float(lateral, -100.0f, 100.0f);
}

// set roll input as a value from -1 to +1
void AP_MotorsUGV::set_roll(float roll)
{
    _roll = constrain_float(roll, -1.0f, 1.0f);
}

// set pitch input as a value from -1 to +1
void AP_MotorsUGV::set_pitch(float pitch)
{
    _pitch = constrain_float(pitch, -1.0f, 1.0f);
}

// set walking_height input as a value from -1 to +1
void AP_MotorsUGV::set_walking_height(float walking_height)
{
    _walking_height = constrain_float(walking_height, -1.0f, 1.0f);
}

// set mainsail input as a value from 0 to 100
void AP_MotorsUGV::set_mainsail(float mainsail)
{
    _mainsail = constrain_float(mainsail, 0.0f, 100.0f);
}

// set wingsail input as a value from -100 to 100
void AP_MotorsUGV::set_wingsail(float wingsail)
{
    _wingsail = constrain_float(wingsail, -100.0f, 100.0f);
}

// set mast rotation input as a value from -100 to 100
void AP_MotorsUGV::set_mast_rotation(float mast_rotation)
{
    _mast_rotation = constrain_float(mast_rotation, -100.0f, 100.0f);
}

// get slew limited throttle
// used by manual mode to avoid bad steering behaviour during transitions from forward to reverse
// same as private slew_limit_throttle method (see below) but does not update throttle state
float AP_MotorsUGV::get_slew_limited_throttle(float throttle, float dt) const
{
    if (_slew_rate <= 0) {
        return throttle;
    }

    const float throttle_change_max = static_cast<float>(_slew_rate) * dt;
    return constrain_float(throttle, _throttle_prev - throttle_change_max, _throttle_prev + throttle_change_max);
}

/*
  work out if skid steering is available
 */
bool AP_MotorsUGV::have_skid_steering() const
{
    return (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) && SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) || is_omni();
}

// true if the vehicle has a mainsail
bool AP_MotorsUGV::has_sail() const
{
    return SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet) || SRV_Channels::function_assigned(SRV_Channel::k_wingsail_elevator) || SRV_Channels::function_assigned(SRV_Channel::k_mast_rotation);
}

void AP_MotorsUGV::output(bool armed, float ground_speed, float dt)
{
    // soft-armed overrides passed in armed status
    if (!hal.util->get_soft_armed()) {
        armed = false;
        _throttle = 0.0f;
    }

    // clear limit flags
    // output_ methods are responsible for setting them to true if required on each iteration
    limit.steer_left = limit.steer_right = limit.throttle_lower = limit.throttle_upper = false;

    // sanity check parameters
    sanity_check_parameters();

    // slew limit throttle
    slew_limit_throttle(dt);

    // apply power limiting to throttle
#if AP_BATTERY_WATT_MAX_ENABLED
    power_limit_throttle(dt);
#endif

    // output for regular steering/throttle style frames
    output_regular(armed, ground_speed, _steering, _throttle, dt);

    // output for skid steering style frames
    output_skid_steering(armed, _steering, _throttle, dt);

    // output for omni frames
    output_omni(armed, _steering, _throttle, _lateral);

    // output to sails
    output_sail();

    // send values to the PWM timers for output
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();

    // nav context is per-cycle: a mode must re-assert it every update via
    // Mode::set_steering(); clearing it here (after all output dispatch)
    // means a stale true can never persist into a manual-mode frame
    _nav_context = false;
    _pid_steering = false;
}

// test steering or throttle output as a percentage of the total (range -100 to +100)
// used in response to DO_MOTOR_TEST mavlink command
bool AP_MotorsUGV::output_test_pct(motor_test_order motor_seq, float pct)
{
    // check if the motor_seq is valid
    if (motor_seq >= MOTOR_TEST_LAST) {
        return false;
    }
    pct = constrain_float(pct, -100.0f, 100.0f);

    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor1)) {
                output_throttle(SRV_Channel::k_motor1, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                output_throttle(SRV_Channel::k_throttle, pct);
            }
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor2)) {
                output_throttle(SRV_Channel::k_motor2, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_steering, pct * 45.0f);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor3)) {
                output_throttle(SRV_Channel::k_motor3, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                output_throttle(SRV_Channel::k_throttleLeft, pct);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor4)) {
                output_throttle(SRV_Channel::k_motor4, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                output_throttle(SRV_Channel::k_throttleRight, pct);
            }
            break;
        }
        case MOTOR_TEST_MAINSAIL: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_mainsail_sheet, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_wingsail_elevator)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_wingsail_elevator, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_mast_rotation)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_mast_rotation, pct);
            }
            break;
        }
        case MOTOR_TEST_LAST:
            return false;
    }
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
    return true;
}

// test steering or throttle output using a pwm value
bool AP_MotorsUGV::output_test_pwm(motor_test_order motor_seq, float pwm)
{
    // check if the motor_seq is valid
    if (motor_seq > MOTOR_TEST_THROTTLE_RIGHT) {
        return false;
    }
    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor1)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, pwm);
            }
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor2)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_steering, pwm);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor3)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor4)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            }
            break;
        }
        case MOTOR_TEST_MAINSAIL: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_mainsail_sheet, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_wingsail_elevator)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_wingsail_elevator, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_mast_rotation)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_mast_rotation, pwm);
            }
            break;
        }
        default:
            return false;
    }
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
    return true;
}

//  returns true if checks pass, false if they fail.  report should be true to send text messages to GCS
bool AP_MotorsUGV::pre_arm_check(bool report) const
{
    const bool have_throttle = SRV_Channels::function_assigned(SRV_Channel::k_throttle);
    const bool have_throttle_left = SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft);
    const bool have_throttle_right = SRV_Channels::function_assigned(SRV_Channel::k_throttleRight);

    // check that there's defined outputs, inc scripting and sail
    if(!have_throttle_left &&
       !have_throttle_right &&
       !have_throttle &&
       !SRV_Channels::function_assigned(SRV_Channel::k_steering) &&
       !SRV_Channels::function_assigned(SRV_Channel::k_scripting1) &&
       !has_sail() &&
       !is_omni()) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: no motor, sail or scripting outputs defined");
        }
        return false;
    }
    // check if only one of skid-steering output has been configured
    if (have_throttle_left != have_throttle_right) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: check skid steering config");
        }
        return false;
    }
    // check if only one of throttle or steering outputs has been configured, if has a sail allow no throttle
    if ((has_sail() || have_throttle) != SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: check steering and throttle config");
        }
        return false;
    }
    // check all omni motor outputs have been configured
    for (uint8_t i=0; i<_motors_num; i++) {
        SRV_Channel::Function function = SRV_Channels::get_motor_function(i);
        if (!SRV_Channels::function_assigned(function)) {
            if (report) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: servo function %u unassigned", function);
            }
            return false;
        }
    }

    // Check relays are configured for brushed with relay outputs
#if AP_RELAY_ENABLED
    AP_Relay*relay = AP::relay();
    if ((_pwm_type == PWMType::BRUSHED_WITH_RELAY) && (relay != nullptr)) {
        // If a output is configured its relay must be enabled
        struct RelayTable {
            bool output_assigned;
            AP_Relay_Params::FUNCTION fun;
        };

        const RelayTable relay_table[] = {
            { have_throttle || have_throttle_left || (SRV_Channels::function_assigned(SRV_Channel::k_motor1) && (_motors_num >= 1)), AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_1 },
            { have_throttle_right || (SRV_Channels::function_assigned(SRV_Channel::k_motor2) && (_motors_num >= 2)),                 AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_2 },
            { SRV_Channels::function_assigned(SRV_Channel::k_motor3) && (_motors_num >= 3),                                          AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_3 },
            { SRV_Channels::function_assigned(SRV_Channel::k_motor4) && (_motors_num >= 4),                                          AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_4 },
        };

        for (uint8_t i=0; i<ARRAY_SIZE(relay_table); i++) {
            if (relay_table[i].output_assigned && !relay->enabled(relay_table[i].fun)) {
                if (report) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: relay function %u unassigned", uint8_t(relay_table[i].fun));
                }
                return false;
            }
        }
    }
#endif

    return true;
}

// sanity check parameters
void AP_MotorsUGV::sanity_check_parameters()
{
    _throttle_max.set(constrain_int16(_throttle_max, 5, 100));
    _throttle_min.set(constrain_int16(_throttle_min, 0, MIN(20, _throttle_max)));
    _vector_angle_max.set(constrain_float(_vector_angle_max, 0.0f, 90.0f));
}

// setup pwm output type
void AP_MotorsUGV::setup_pwm_type()
{
    _motor_mask = 0;

    hal.rcout->set_dshot_esc_type(SRV_Channels::get_dshot_esc_type());

    // work out mask of channels assigned to motors
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttle);
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttleLeft);
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttleRight);
    for (uint8_t i=0; i<_motors_num; i++) {
        _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channels::get_motor_function(i));
    }

    switch (_pwm_type) {
    case PWMType::ONESHOT:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        break;
    case PWMType::ONESHOT125:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT125);
        break;
    case PWMType::BRUSHED_WITH_RELAY:
    case PWMType::BRUSHED_BIPOLAR:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        hal.rcout->set_freq(_motor_mask, uint16_t(_pwm_freq * 1000));
        break;
    case PWMType::DSHOT150:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT150);
        break;
    case PWMType::DSHOT300:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT300);
        break;
    case PWMType::DSHOT600:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT600);
        break;
    case PWMType::DSHOT1200:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT1200);
        break;
    default:
        // do nothing
        break;
    }
}

// setup for frames with omni motors
void AP_MotorsUGV::setup_omni()
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_NUM_MOTORS_MAX; i++) {
        clear_omni_motors(i);
    }

    // hard coded factor configuration
    switch (_frame_type) {

    //   FRAME TYPE NAME
    case FRAME_TYPE_UNDEFINED:
        break;

    case FRAME_TYPE_OMNI3:
        _motors_num = 3;
        add_omni_motor(0, 1.0f, -1.0f, -1.0f);
        add_omni_motor(1, 0.0f, -1.0f, 1.0f);
        add_omni_motor(2, 1.0f, 1.0f, 1.0f);
        break;

    case FRAME_TYPE_OMNIX:
        _motors_num = 4,
        add_omni_motor(0, 1.0f, -1.0f, -1.0f);
        add_omni_motor(1, 1.0f, -1.0f, 1.0f);
        add_omni_motor(2, 1.0f, 1.0f, -1.0f);
        add_omni_motor(3, 1.0f, 1.0f, 1.0f);
        break;

    case FRAME_TYPE_OMNIPLUS:
        _motors_num = 4;
        add_omni_motor(0, 0.0f, 1.0f, 1.0f);
        add_omni_motor(1, 1.0f, 0.0f, 0.0f);
        add_omni_motor(2, 0.0f, -1.0f, 1.0f);
        add_omni_motor(3, 1.0f, 0.0f, 0.0f);
        break;

    case FRAME_TYPE_OMNI3MECANUM:
        _motors_num = 3;
        add_omni_motor(0,  -1.0f,    1.0f,  -0.26795f);
        add_omni_motor(1,  0.73205f, 1.0f,  -0.73205f);
        add_omni_motor(2,  0.26795f, 1.0f,   1.0f);
        break;
    }
}

// add omni motor using separate throttle, steering and lateral factors
void AP_MotorsUGV::add_omni_motor(int8_t motor_num, float throttle_factor, float steering_factor, float lateral_factor)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {

        // set throttle, steering and lateral factors
        _throttle_factor[motor_num] = throttle_factor;
        _steering_factor[motor_num] = steering_factor;
        _lateral_factor[motor_num] = lateral_factor;

        add_omni_motor_num(motor_num);
    }
}

// add an omni motor and set up default output function
void AP_MotorsUGV::add_omni_motor_num(int8_t motor_num)
{
    // ensure a valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {
        uint8_t chan;
        SRV_Channel::Function function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);
        if (!SRV_Channels::find_channel(function, chan)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
        }
    }
}

// disable omni motor and remove all throttle, steering and lateral factor for this motor
void AP_MotorsUGV::clear_omni_motors(int8_t motor_num)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {
        // disable the motor and set factors to zero
        _throttle_factor[motor_num] = 0;
        _steering_factor[motor_num] = 0;
        _lateral_factor[motor_num] = 0;
    }
}

// output to regular steering and throttle channels
void AP_MotorsUGV::output_regular(bool armed, float ground_speed, float steering, float throttle, float dt)
{
    // output to throttle channels
    if (armed) {
        if (_scale_steering) {
            // vectored thrust handling
            if (have_vectored_thrust()) {
                if (_vec_fossen > 0) {
                    // unified allocator path (VEC_FOSSEN=1): mapping lives in vectored_allocate() — math differs from the legacy blend below (fold hysteresis, cos-floor, blend curve, brakes)
                    vectored_allocate(throttle * 0.01f, steering * (1.0f/4500.0f), ground_speed,
                                      _nav_context, steering, throttle, dt);
                } else {
                // normalise desired steering and throttle to ease calculations
                const float steering_norm = steering / 4500.0f;
                const float throttle_norm = throttle * 0.01f;
                const float vector_angle_max_rad = radians(constrain_float(_vector_angle_max, 0.0f, 90.0f));

                // low-pass filter throttle to distinguish sudden step commands from
                // steady-state near-zero throttle, where atan() amplifies noise
                if (is_positive(_vec_resid_tc)) {
                    const float alpha = constrain_float(dt / (_vec_resid_tc + dt), 0.0f, 1.0f);
                    _vec_throttle_filt += (throttle_norm - _vec_throttle_filt) * alpha;
                    _vec_steering_filt += (steering_norm - _vec_steering_filt) * alpha;
                } else {
                    _vec_throttle_filt = throttle_norm;
                    _vec_steering_filt = steering_norm;
                }

                // steering can never be more than filtered-throttle * tan(_vector_angle_max)
                const float steering_norm_lim = fabsf(_vec_throttle_filt * tanf(vector_angle_max_rad));
                float atan_steering_norm = steering_norm;
                if (fabsf(atan_steering_norm) > steering_norm_lim) {
                    if (is_positive(atan_steering_norm)) {
                        atan_steering_norm = steering_norm_lim;
                    } else {
                        atan_steering_norm = -steering_norm_lim;
                    }
                    limit.steer_right = true;
                    limit.steer_left = true;
                }

                const float vec_mag = sqrtf(sq(_vec_steering_filt) + sq(_vec_throttle_filt));

                float w = 1.0f;   // blend weight: 1 = pure direct/quadrant mapping, 0 = legacy full atan()
                float steering_angle_rad;

                if (vec_mag < _vec_deadband) {
                    // both steering and throttle demand are negligible: freeze angle
                    // (and blend weight) to suppress jitter instead of letting atan()
                    // amplify noise, and so the throttle-boost stays consistent with
                    // whatever regime (direct vs. atan) was active just before freezing
                    steering_angle_rad = _vec_last_steering_angle_rad;
                    w = _vec_last_w;
                } else {
                    // direct/quadrant-based angle: proportional to steering demand only,
                    // sign-folded for reverse throttle so the thruster points the correct way
					float direct_angle_rad = constrain_float(steering_norm, -1.0f, 1.0f) * vector_angle_max_rad;
					// use the raw (unfiltered) throttle sign so the vector flips at the same instant
					// the actual commanded throttle crosses zero, not delayed by the low-pass filter
					if (is_negative(throttle_norm)) {
					direct_angle_rad = -direct_angle_rad;
					}

                    // legacy atan()-based angle, guarded against divide-by-zero
					float atan_angle_rad = 0.0f;
					if (!is_zero(_vec_throttle_filt)) {
						// magnitude from the filtered throttle (noise suppression), but sign forced
						// to match the raw/unfiltered throttle so reversal timing isn't lagged
						const float atan_denom = is_negative(throttle_norm) ? -fabsf(_vec_throttle_filt) : fabsf(_vec_throttle_filt);
						atan_angle_rad = atanf(atan_steering_norm / atan_denom);
					}

                    // blend weight from filtered throttle magnitude: w=1 near zero throttle
                    // (direct mapping), w=0 at/above VEC_BLND_THR (legacy atan() mapping)
                    if (is_positive(_vec_blend_thr)) {
                        w = 1.0f - constrain_float(fabsf(_vec_throttle_filt) / _vec_blend_thr, 0.0f, 1.0f);
                    } else {
                        w = 0.0f;
                    }

                    steering_angle_rad = w * direct_angle_rad + (1.0f - w) * atan_angle_rad;

                    if (fabsf(steering_angle_rad) > vector_angle_max_rad) {
                        steering_angle_rad = constrain_float(steering_angle_rad, -vector_angle_max_rad, vector_angle_max_rad);
                        limit.steer_right = true;
                        limit.steer_left = true;
                    }

                    _vec_last_steering_angle_rad = steering_angle_rad;
                    _vec_last_w = w;
                }

                // convert steering angle to steering output
                steering = steering_angle_rad / vector_angle_max_rad * 4500.0f;

                // scale up throttle to compensate for steering angle, blended by the same w
                // computed above: w=1 -> no boost (no forward-thrust component to preserve
                // when driven purely by a position/speed PID at near-zero throttle),
                // w=0 -> full cos() boost, matching original legacy behaviour
                const float throttle_scaler_inv = w + (1.0f - w) * cosf(steering_angle_rad);
                if (!is_zero(throttle_scaler_inv)) {
                    throttle /= throttle_scaler_inv;
                }
                }
            } else {
                // scale steering down as speed increase above MOT_SPD_SCA_BASE (1 m/s default)
                if (is_positive(_speed_scale_base) && (fabsf(ground_speed) > _speed_scale_base)) {
                    steering *= (_speed_scale_base / fabsf(ground_speed));
                } else {
                    // regular steering rover at low speed so set limits to stop I-term build-up in controllers
                    if (!have_skid_steering()) {
                        limit.steer_left = true;
                        limit.steer_right = true;
                    }
                }
                // reverse steering direction when backing up
                if (is_negative(ground_speed)) {
                    steering *= -1.0f;
                }
            }
        } else {
            // reverse steering direction when backing up
            if (is_negative(throttle)) {
                steering *= -1.0f;
            }
        }
    output_throttle(SRV_Channel::k_throttle, throttle, dt);
    } else {
        // handle disarmed case
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
        }
    }

    // clear and set limits based on input
    // we do this here because vectored thrust or speed scaling may have reduced steering request
    set_limits_from_input(armed, steering, throttle);

    // constrain steering
    steering = constrain_float(steering, -4500.0f, 4500.0f);

    // always allow steering to move
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering);
}

// output to skid steering channels
void AP_MotorsUGV::output_skid_steering(bool armed, float steering, float throttle, float dt)
{
    if (!have_skid_steering()) {
        return;
    }

    // clear and set limits based on input
    set_limits_from_input(armed, steering, throttle);

    // constrain steering
    steering = constrain_float(steering, -4500.0f, 4500.0f);

    // handle simpler disarmed case
    if (!armed) {
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
        }
        return;
    }

    // skid steering mixer
    float steering_scaled = steering / 4500.0f; // steering scaled -1 to +1
    float throttle_scaled = throttle * 0.01f;  // throttle scaled -1 to +1

    // sanitize values for asymmetry of thrust, mixer assumes forward thrust is always larger than reverse
    const float thrust_asymmetry = MAX(_thrust_asymmetry, 1.0);
    const float lower_throttle_limit = -1.0 / thrust_asymmetry;

    // Maximum steering is half way between upper and lower limits
    const float best_steering_throttle = (1.0 + lower_throttle_limit) * 0.5;
    float steering_range;
    if (throttle_scaled < best_steering_throttle) {
        // steering range is reduced as throttle will never be increased by mixer
        steering_range = MAX(throttle_scaled,0.0) - lower_throttle_limit;
    } else {
        // full range available, throttle can always be lowered down to best_steering_throttle
        steering_range = 1 - best_steering_throttle;
    }

    // apply constraints
    if (steering_scaled > steering_range) {
        limit.steer_right = true;
        steering_scaled = steering_range;
    } else if (steering_scaled < -steering_range) {
        limit.steer_left = true;
        steering_scaled = -steering_range;
    }
    if (throttle_scaled > 1.0) {
        limit.throttle_upper = true;
        throttle_scaled = 1.0;
    } else if (throttle_scaled < lower_throttle_limit) {
        limit.throttle_lower = true;
        throttle_scaled = lower_throttle_limit;
    }

    // All throttle or all steering will now fit, check if they will both fit together
    const float max_output = throttle_scaled + fabsf(steering_scaled);
    const float min_output = throttle_scaled - fabsf(steering_scaled);

    // check for saturation and scale back throttle and steering proportionally
    const float saturation_value = MAX(max_output, min_output / lower_throttle_limit);
    if (saturation_value > 1.0f) {
        // store pre-scaled values so we can set limit flags afterwards
        const float steering_scaled_orig = steering_scaled;
        const float throttle_scaled_orig = throttle_scaled;

        const float str_thr_mix = constrain_float(_steering_throttle_mix, 0.0f, 1.0f);
        const float fair_scaler = 1.0f / saturation_value;
        if (str_thr_mix >= 0.5f) {
            // prioritise steering over throttle
            steering_scaled *= linear_interpolate(fair_scaler, 1.0f, str_thr_mix, 0.5f, 1.0f);
            if (throttle_scaled >= best_steering_throttle) {
                // constrained by upper limit
                throttle_scaled = 1.0 - fabsf(steering_scaled);
            } else {
                // constrained by lower limit
                throttle_scaled = fabsf(steering_scaled) + lower_throttle_limit;
            }

        } else {
            // prioritise throttle over steering
            throttle_scaled *= linear_interpolate(fair_scaler, 1.0f, 0.5f - str_thr_mix, 0.0f, 0.5f);
            const float steering_sign = is_positive(steering_scaled) ? 1.0 : -1.0;
            if (throttle_scaled >= best_steering_throttle) {
                // constrained by upper limit
                steering_scaled = (1.0 - throttle_scaled) * steering_sign;
            } else {
                // constrained by lower limit
                steering_scaled = (throttle_scaled - lower_throttle_limit) * steering_sign;
            }
        }

        // update limits if either steering or throttle has been reduced
        if (fabsf(steering_scaled) < fabsf(steering_scaled_orig)) {
            limit.steer_left |= is_negative(steering_scaled_orig);
            limit.steer_right |= is_positive(steering_scaled_orig);
        }
        if (fabsf(throttle_scaled) < fabsf(throttle_scaled_orig)) {
            limit.throttle_lower |= is_negative(throttle_scaled_orig);
            limit.throttle_upper |= is_positive(throttle_scaled_orig);
        }
    }

    // add in throttle and steering
    float motor_left = throttle_scaled + steering_scaled;
    float motor_right = throttle_scaled - steering_scaled;

    // Apply asymmetry correction
    if (is_negative(motor_right)) {
        motor_right *= thrust_asymmetry;
    }
    if (is_negative(motor_left)) {
        motor_left *= thrust_asymmetry;
    }

    // send pwm value to each motor
    output_throttle(SRV_Channel::k_throttleLeft, 100.0f * motor_left, dt);
    output_throttle(SRV_Channel::k_throttleRight, 100.0f * motor_right, dt);
}

// output for omni frames
void AP_MotorsUGV::output_omni(bool armed, float steering, float throttle, float lateral)
{
    // exit immediately if the vehicle is not omni
    if (!is_omni()) {
        return;
    }

    if (armed) {
        // clear and set limits based on input
        set_limits_from_input(armed, steering, throttle);

        // constrain steering
        steering = constrain_float(steering, -4500.0f, 4500.0f);

        // scale throttle, steering and lateral inputs to -1 to 1
        const float scaled_throttle = throttle * 0.01f;
        const float scaled_steering = steering / 4500.0f;
        const float scaled_lateral = lateral * 0.01f;

        // sized to the maximum rather than the runtime motor count: a
        // variable-length array is non-standard C++, and setup_omni() only
        // ever sets _motors_num to 3 or 4
        float thr_str_ltr_out[AP_MOTORS_NUM_MOTORS_MAX] {};
        float thr_str_ltr_max = 1;
        for (uint8_t i=0; i<_motors_num; i++) {
            // Each motor outputs throttle + steering + lateral
            thr_str_ltr_out[i] = (scaled_throttle * _throttle_factor[i]) +
                              (scaled_steering * _steering_factor[i]) +
                              (scaled_lateral * _lateral_factor[i]);
            // record the largest output above 1
            if (fabsf(thr_str_ltr_out[i]) > thr_str_ltr_max) {
                thr_str_ltr_max = fabsf(thr_str_ltr_out[i]);
            }
        }
        // Scale all outputs back evenly such that the largest fits
        const float output_scale = 1 / thr_str_ltr_max;
        for (uint8_t i=0; i<_motors_num; i++) {
            // send output for each motor
            output_throttle(SRV_Channels::get_motor_function(i), thr_str_ltr_out[i] * 100.0f * output_scale);
        }
        if (output_scale < 1.0) {
            // can't tell which command resulted in the scale back, so limit all
            limit.steer_left = true;
            limit.steer_right = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;
        }
    } else {
        // handle disarmed case
        if (_disarm_disable_pwm) {
            for (uint8_t i=0; i<_motors_num; i++) {
                SRV_Channels::set_output_limit(SRV_Channels::get_motor_function(i), SRV_Channel::Limit::ZERO_PWM);
            }
        } else {
            for (uint8_t i=0; i<_motors_num; i++) {
                SRV_Channels::set_output_limit(SRV_Channels::get_motor_function(i), SRV_Channel::Limit::TRIM);
            }
        }
    }
}

// return power_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
#if AP_BATTERY_WATT_MAX_ENABLED
float AP_MotorsUGV::get_power_limit_max_throttle(float dt)
{
    AP_BattMonitor &battery = AP::battery();

    float _batt_current;
    const float watt_max = battery.get_watt_max();
    const float _batt_voltage = battery.voltage();

    if (watt_max <= 0 ||  // return maximum if power limiting is disabled
        !hal.util->get_soft_armed() ||   // remove throttle limit if disarmed
        !battery.current_amps(_batt_current) || // no current monitoring is available
        is_zero(_batt_voltage)) {  // no voltage monitoring is available
        _throttle_limit = 1.0f;
        return 1.0f;
    }

    const float power = _batt_voltage * _batt_current;
    const float power_ratio = power / watt_max;

    _throttle_limit += (dt / (dt + _batt_power_time_constant)) * (1.0f - power_ratio);

    // ensure throttle limit is within min and max throttle limits
    _throttle_limit = constrain_float(_throttle_limit, _throttle_min * 0.01f, _throttle_max * 0.01f);

    return _throttle_limit;
}
#endif

// output throttle value to main throttle channel, left throttle or right throttle.  throttle should be scaled from -100 to 100
void AP_MotorsUGV::output_throttle(SRV_Channel::Function function, float throttle, float dt)
{
    // sanity check servo function
    if (function != SRV_Channel::k_throttle && function != SRV_Channel::k_throttleLeft && function != SRV_Channel::k_throttleRight && function != SRV_Channel::k_motor1 && function != SRV_Channel::k_motor2 && function != SRV_Channel::k_motor3 && function!= SRV_Channel::k_motor4) {
        return;
    }

    // constrain and scale output
    throttle = get_scaled_throttle(throttle);

    // apply rate control
    throttle = get_rate_controlled_throttle(function, throttle, dt);

    // set relay if necessary
#if AP_RELAY_ENABLED
    AP_Relay*relay = AP::relay();
    if ((_pwm_type == PWMType::BRUSHED_WITH_RELAY) && (relay != nullptr)) {

        // find the output channel, if not found return
        const SRV_Channel *out_chan = SRV_Channels::get_channel_for(function);
        if (out_chan == nullptr) {
            return;
        }
        const int8_t reverse_multiplier = out_chan->get_reversed() ? -1 : 1;
        bool relay_high = is_negative(reverse_multiplier * throttle);

        AP_Relay_Params::FUNCTION relay_function;
        switch (function) {
            case SRV_Channel::k_throttle:
            case SRV_Channel::k_throttleLeft:
            case SRV_Channel::k_motor1:
            default:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_1;
                break;
            case SRV_Channel::k_throttleRight:
            case SRV_Channel::k_motor2:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_2;
                break;
            case SRV_Channel::k_motor3:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_3;
                break;
            case SRV_Channel::k_motor4:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_4;
                break;
        }
        relay->set(relay_function, relay_high);

        // invert the output to always have positive value calculated by calc_pwm
        throttle = reverse_multiplier * fabsf(throttle);
    }
#endif  // AP_RELAY_ENABLED

    if (_reverse_delay > 0) {
        switch (function) {
        case SRV_Channel::k_throttle:
            rev_delay_throttle.output(function, throttle, _reverse_delay);
            return;
        case SRV_Channel::k_throttleLeft:
            rev_delay_throttleLeft.output(function, throttle * 10, _reverse_delay);
            return;
        case SRV_Channel::k_throttleRight:
            rev_delay_throttleRight.output(function, throttle * 10, _reverse_delay);
            return;
        default:
            // fall through to other non-delayed outputs
            break;
        }
    }

    // output to servo channel
    switch (function) {
        case SRV_Channel::k_throttle:
        case SRV_Channel::k_motor1:
        case SRV_Channel::k_motor2:
        case SRV_Channel::k_motor3:
        case SRV_Channel::k_motor4:
            SRV_Channels::set_output_scaled(function,  throttle);
            break;
        case SRV_Channel::k_throttleLeft:
        case SRV_Channel::k_throttleRight:
            SRV_Channels::set_output_scaled(function,  throttle * 10.0f);
            break;
        default:
            // do nothing
            break;
    }
}

// output for sailboat's sails
void AP_MotorsUGV::output_sail()
{
    if (!has_sail()) {
        return;
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_mainsail_sheet, _mainsail);
    SRV_Channels::set_output_scaled(SRV_Channel::k_wingsail_elevator, _wingsail);
    SRV_Channels::set_output_scaled(SRV_Channel::k_mast_rotation, _mast_rotation);
}

// slew limit throttle for one iteration
void AP_MotorsUGV::slew_limit_throttle(float dt)
{
    const float throttle_orig = _throttle;
    _throttle = get_slew_limited_throttle(_throttle, dt);
    if (throttle_orig > _throttle) {
        limit.throttle_upper = true;
    } else if (throttle_orig < _throttle) {
        limit.throttle_lower = true;
    }
    _throttle_prev = _throttle;
}

// apply power limiting to throttle for one iteration
#if AP_BATTERY_WATT_MAX_ENABLED
void AP_MotorsUGV::power_limit_throttle(float dt)
{
    const float power_limit_max = get_power_limit_max_throttle(dt);

    const float throttle_max = 100.0f * power_limit_max;
    const float throttle_min = -100.0f * power_limit_max;

    if (_throttle > throttle_max) {
        _throttle = throttle_max;
        limit.throttle_upper = true;
    } else if (_throttle < throttle_min) {
        _throttle = throttle_min;
        limit.throttle_lower = true;
    }
}
#endif

// set limits based on steering and throttle input
void AP_MotorsUGV::set_limits_from_input(bool armed, float steering, float throttle)
{
    // set limits based on inputs
    limit.steer_left |= !armed || (steering <= -4500.0f);
    limit.steer_right |= !armed || (steering >= 4500.0f);
    limit.throttle_lower |= !armed || (throttle <= -_throttle_max);
    limit.throttle_upper |= !armed || (throttle >= _throttle_max);
}

// scale a throttle using the _throttle_min and _thrust_curve_expo parameters.  throttle should be in the range -100 to +100
float AP_MotorsUGV::get_scaled_throttle(float throttle) const
{
    // exit immediately if throttle is zero
    if (is_zero(throttle)) {
        return throttle;
    }

    // scale using throttle_min
    if (_throttle_min > 0) {
        if (is_negative(throttle)) {
            throttle = -_throttle_min + (throttle * ((100.0f - _throttle_min) * 0.01f));
        } else {
            throttle = _throttle_min + (throttle * ((100.0f - _throttle_min) * 0.01f));
        }
    }

    // skip further scaling if thrust curve disabled or invalid
    if (is_zero(_thrust_curve_expo) || (_thrust_curve_expo > 1.0f) || (_thrust_curve_expo < -1.0f)) {
        return throttle;
    }

    // calculate scaler
    const float sign = (throttle < 0.0f) ? -1.0f : 1.0f;
    const float throttle_pct = constrain_float(throttle, -100.0f, 100.0f) * 0.01f;
    return 100.0f * sign * ((_thrust_curve_expo - 1.0f) + safe_sqrt((1.0f - _thrust_curve_expo) * (1.0f - _thrust_curve_expo) + 4.0f * _thrust_curve_expo * fabsf(throttle_pct))) / (2.0f * _thrust_curve_expo);
}

// use rate controller to achieve desired throttle
float AP_MotorsUGV::get_rate_controlled_throttle(SRV_Channel::Function function, float throttle, float dt)
{
    // require non-zero dt
    if (!is_positive(dt)) {
        return throttle;
    }

    // attempt to rate control left throttle
    if ((function == SRV_Channel::k_throttleLeft) && _rate_controller.enabled(0)) {
        return _rate_controller.get_rate_controlled_throttle(0, throttle, dt);
    }

    // rate control right throttle
    if ((function == SRV_Channel::k_throttleRight) && _rate_controller.enabled(1)) {
        return _rate_controller.get_rate_controlled_throttle(1, throttle, dt);
    }

    // return throttle unchanged
    return throttle;
}

// return true if motors are moving
bool AP_MotorsUGV::active() const
{
    // if soft disarmed, motors not active
    if (!hal.util->get_soft_armed()) {
        return false;
    }

    // check throttle is active
    if (!is_zero(get_throttle())) {
        return true;
    }

    // skid-steering vehicles active when steering
    if (have_skid_steering() && !is_zero(get_steering())) {
        return true;
    }

    return false;
}

// returns true if the configured PWM type is digital and should have fixed endpoints
bool AP_MotorsUGV::is_digital_pwm_type() const
{
    switch (_pwm_type) {
    case PWMType::DSHOT150:
    case PWMType::DSHOT300:
    case PWMType::DSHOT600:
    case PWMType::DSHOT1200:
        return true;
    case PWMType::NORMAL:
    case PWMType::ONESHOT:
    case PWMType::ONESHOT125:
    case PWMType::BRUSHED_WITH_RELAY:
    case PWMType::BRUSHED_BIPOLAR:
        break;
    }
    return false;
}

/*
  handle delay on reversal for a throttle
 */
void AP_MotorsUGV::ReverseThrottle::output(SRV_Channel::Function function, float throttle, float delay)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (is_zero(throttle)) {
        // pass through, no change, don't update the last throttle
    } else if (throttle * last_throttle < 0 &&
        now_ms - last_output_ms < delay*1000) {
        // sign change, add pause
        throttle = 0;
    } else {
        last_output_ms = now_ms;
        last_throttle = throttle;
    }
    SRV_Channels::set_output_scaled(function, throttle);
}

namespace AP {
    AP_MotorsUGV *motors_ugv()
    {
        return AP_MotorsUGV::get_singleton();
    }
}

