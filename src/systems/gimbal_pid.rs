use bevy::prelude::*;
use bevy::transform::helper::TransformHelper;

use crate::components::{Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset};
use crate::config::{GimbalAxisPidConfig, GimbalPidConfig, SimulationConfig};

/// Absolute muzzle-frame aim target produced by an external auto-aim solver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GimbalAimTarget {
    pub yaw: f32,
    pub pitch: f32,
}

impl GimbalAimTarget {
    /// Solver commands arrive in degrees:
    ///
    /// - `yaw_deg` is the muzzle azimuth in the **map frame** (degrees),
    /// - `pitch_deg` is the elevation above the **horizon** (degrees,
    ///   **negative = downwards**).
    ///
    /// This convention is verified against the benchmark stack: a command of
    /// `-0.5` produces a projectile launch elevation of `-0.5°` (measured from
    /// the `/sim/events/fire` velocity vector), and the shipped
    /// `hit_static_single` recordings send `pitch ∈ [-5.0, -3.0]` while their
    /// shots leave at `-3.05°`. The `- 90.0` below folds in the gimbal
    /// **mount** rest pitch; it is *not* part of the API convention.
    ///
    /// (An earlier version of this comment claimed "pitch measured from the
    /// vertical axis". That is wrong: following it makes the gimbal saturate
    /// against `gimbal_pitch_limit` and fires ~70° upwards.)
    pub fn from_solver_degrees(yaw_deg: f32, pitch_deg: f32) -> Self {
        Self {
            yaw: yaw_deg.to_radians(),
            pitch: (pitch_deg - 90.0).to_radians(),
        }
    }

    fn rotation(self) -> Quat {
        Quat::from_euler(EulerRot::YXZ, self.yaw, self.pitch, 0.0)
    }
}

/// One independent single-axis PID loop.
#[derive(Clone, Copy, Debug, Default)]
struct AxisPid {
    integral: f32,
    previous_error: f32,
}

impl AxisPid {
    /// Angular rate command (rad/s) for this axis given the current error.
    fn step(&mut self, error: f32, config: &GimbalAxisPidConfig, dt: f32) -> f32 {
        self.integral =
            (self.integral + error * dt).clamp(-config.integral_limit, config.integral_limit);
        let derivative = (error - self.previous_error) / dt;
        self.previous_error = error;

        let rate = error * config.kp + self.integral * config.ki + derivative * config.kd;
        rate.clamp(-config.max_rate, config.max_rate)
    }
}

/// Closed-loop state for tracking a solver target. Its presence on the gimbal *is*
/// the "auto-aim has a target" state: no tracker means no target and no PID output,
/// and a freshly inserted tracker starts from clean integrator/derivative state.
#[derive(Component, Clone, Copy, Debug)]
pub struct GimbalAimTracker {
    yaw_model: super::yaw_reference::YawReference,
    model_diagnostic: [f64;5],
    model_limited: bool,
    model_effective_ns: Option<i128>,
    target: GimbalAimTarget,
    yaw: AxisPid,
    pitch: AxisPid,
    command_stamp_ns: Option<i128>,
    accepted_wall_ns: i128,
    last_command_at: Option<crate::clock::Instant>,
    feedforward: Vec2,
    temporal: Option<(i128, Vec2)>,
    control_steps: u64,
    last_dt: f32,
    last_muzzle_yaw: f32,
    last_error: Vec2,
    last_feedback: Vec2,
    last_feedforward: Vec2,
}

impl GimbalAimTracker {
    pub fn new(target: GimbalAimTarget) -> Self {
        Self {
            yaw_model: Default::default(),
            model_diagnostic: [0.0;5],
            model_limited: false,
            model_effective_ns: None,
            target,
            yaw: AxisPid::default(),
            pitch: AxisPid::default(),
            command_stamp_ns: None,
            accepted_wall_ns: 0,
            last_command_at: None,
            feedforward: Vec2::ZERO,
            temporal: None,
            control_steps: 0,
            last_dt: 0.0,
            last_muzzle_yaw: 0.0,
            last_error: Vec2::ZERO,
            last_feedback: Vec2::ZERO,
            last_feedforward: Vec2::ZERO,
        }
    }

    pub fn reset_yaw_model(&mut self) {
        self.yaw_model=Default::default();self.model_diagnostic=[0.0;5];
        self.model_limited=false;self.model_effective_ns=None;
    }

    /// Shared production yaw actuator path, also exercised by the component harness.
    fn advance_yaw(&mut self,id:&str,rate:f32,dt:f32,limit:f32,angle_before:f32)->f32 {
        match id {
        "legacy" => rate * dt,
        id => {
            let model=super::yaw_reference::ReferenceModel::from_id(id)
                .unwrap_or_else(||panic!("Unknown gimbal dynamics: {}",id));
            let a=model.advance(&mut self.yaw_model,rate as f64,dt as f64,limit as f64);
            self.model_diagnostic=[rate as f64,a.raw_rate,a.rate,angle_before as f64,a.angle];
            if self.model_effective_ns.is_none() {self.model_effective_ns=Some(crate::clock::now_ns());}
            self.model_limited=a.limited;
            a.angle as f32
        },
        }
    }

    pub fn target(&self) -> GimbalAimTarget {
        self.target
    }

    /// Update the setpoint while keeping the loop state, so a stream of commands
    /// drives one continuous controller instead of restarting every message.
    pub fn retarget(&mut self, target: GimbalAimTarget) {
        self.target = target;
    }

    /// Feedforward uses only timestamped solver angles, never simulator truth.
    pub fn retarget_stamped(&mut self, target: GimbalAimTarget, stamp_ns: i128, config: &GimbalPidConfig) {
        let delta = Vec2::new(wrap_angle(target.yaw-self.target.yaw), wrap_angle(target.pitch-self.target.pitch));
        if let Some(previous) = self.command_stamp_ns {
            let dt = (stamp_ns-previous) as f32 * 1e-9;
            if dt > 0.001 && dt <= config.feedforward_reset_gap_s
                && delta.abs().max_element() <= config.feedforward_max_step_rad {
                let limit = config.feedforward_max_rate.max(0.0);
                let raw = (delta/dt).clamp(Vec2::splat(-limit), Vec2::splat(limit));
                let alpha = 1.0-(-dt/config.feedforward_tau_s.max(0.001)).exp();
                self.feedforward += (raw-self.feedforward)*alpha;
            } else {
                self.feedforward = Vec2::ZERO;
            }
        }
        self.command_stamp_ns = Some(stamp_ns);
        self.accepted_wall_ns = crate::clock::now_ns();
        self.model_effective_ns=None;
        self.last_command_at = Some(crate::clock::Instant::now());
        self.temporal=None;
        self.retarget(target);
    }

    pub fn temporal_setpoint(&mut self, enabled: bool, stamp_ns: i64, yaw_rate: f64, pitch_rate: f64) {
        self.temporal=None;
        let now=crate::clock::now_ns();
        let age=(now-i128::from(stamp_ns)) as f64*1e-9;
        if enabled && age>=-0.05 && age<=0.1 && yaw_rate.is_finite() && pitch_rate.is_finite() && yaw_rate.abs()<=0.6 && pitch_rate.abs()<=0.6 {
            self.temporal=Some((i128::from(stamp_ns),Vec2::new(yaw_rate as f32,pitch_rate as f32)));
            self.feedforward=Vec2::new(yaw_rate as f32,pitch_rate as f32);
        }
    }
    fn effective_target(&self)->Option<GimbalAimTarget> {
        let now=crate::clock::now_ns();
        self.effective_target_at(now)
    }
    fn effective_target_at(&self,now:i128)->Option<GimbalAimTarget> {
        let Some((stamp,rate))=self.temporal else {return Some(self.target);};
        let dt=(now-stamp) as f32*1e-9;
        if !(-0.05..=0.1).contains(&dt) {return None;}
        Some(GimbalAimTarget {yaw:self.target.yaw+rate.x*dt,pitch:self.target.pitch+rate.y*dt})
    }

    pub fn diagnostic(&self) -> serde_json::Value {
        serde_json::json!({"model_state":self.yaw_model.x,"model_input_rad_s":self.model_diagnostic[0],
            "model_raw_output_rad_s":self.model_diagnostic[1],"model_output_rad_s":self.model_diagnostic[2],
            "model_angle_before_rad":self.model_diagnostic[3],"model_angle_delta_rad":self.model_diagnostic[4],
            "model_first_effective_ns":self.model_effective_ns,"model_limited":self.model_limited,
            "command_stamp_ns":self.command_stamp_ns,
            "accepted_wall_ns":self.accepted_wall_ns,"target_yaw_rad":self.target.yaw,
            "target_pitch_rad":self.target.pitch,"control_steps":self.control_steps,
            "temporal_target":self.temporal.is_some(),"aim_stamp_ns":self.temporal.map(|p|p.0),
            "dt_s":self.last_dt,"muzzle_yaw_before_control_rad":self.last_muzzle_yaw,"error_local_rad":self.last_error.to_array(),
            "feedback_local_rad_s":self.last_feedback.to_array(),
            "feedforward_world_rad_s":self.feedforward.to_array(),
            "feedforward_local_rad_s":self.last_feedforward.to_array()})
    }
}

fn wrap_angle(angle: f32) -> f32 {
    let wrapped = (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU);
    wrapped - std::f32::consts::PI
}

/// Tracking error as `(yaw, pitch)` in the gimbal's own local frame, i.e. in the same
/// coordinates as `InfantryGimbal`. The solver target aims the *muzzle* in world space,
/// so the muzzle's fixed mount offset and the chassis rotation are divided out first —
/// measuring the error in world space instead flips the pitch sign for any mount whose
/// local axes disagree with the world axes.
fn tracking_error(
    target_rotation: Quat,
    gimbal_local: Quat,
    gimbal_world: Quat,
    muzzle_world: Quat,
) -> Vec2 {
    // World-space correction that would put the muzzle on target right now.
    let correction = target_rotation * muzzle_world.inverse();
    // Same correction expressed as the gimbal's desired local rotation.
    let desired_local = gimbal_local * gimbal_world.inverse() * correction * gimbal_world;

    let (desired_yaw, desired_pitch, _) = desired_local.to_euler(EulerRot::YXZ);
    let (current_yaw, current_pitch, _) = gimbal_local.to_euler(EulerRot::YXZ);

    Vec2::new(
        wrap_angle(desired_yaw - current_yaw),
        wrap_angle(desired_pitch - current_pitch),
    )
}

/// Replaces the direct pose snap that auto-aim used to apply: the solver target is
/// tracked by a rate-limited PID loop so the gimbal moves like an actuated axis.
pub fn gimbal_pid_controls(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    ids: Query<Entity, (With<Controlled>, With<InfantryGimbal>)>,
    mut transforms: ParamSet<(
        TransformHelper,
        Query<
            (
                &mut Transform,
                &GlobalTransform,
                &mut InfantryGimbal,
                &mut GimbalAimTracker,
            ),
            (
                With<Controlled>,
                Without<InfantryChassis>,
                Without<InfantryLaunchOffset>,
            ),
        >,
    )>,
    muzzle: Option<Single<(Entity, &GlobalTransform), (With<InfantryLaunchOffset>, With<Controlled>)>>,
) {
    let (Ok(entity), Some(muzzle)) = (ids.single(), muzzle) else { return; };
    let (muzzle_entity, muzzle_global) = muzzle.into_inner();
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    let fresh = if config.vehicle.gimbal_pid.fixed_update {
        let helper = transforms.p0();
        let (Ok(gimbal), Ok(muzzle)) = (helper.compute_global_transform(entity), helper.compute_global_transform(muzzle_entity)) else { return; };
        Some((gimbal.rotation(), muzzle.rotation()))
    } else { None };
    let mut query = transforms.p1();
    let Ok((mut gimbal_transform, gimbal_global, mut gimbal_data, mut tracker)) = query.get_mut(entity) else { return; };
    // Fixed ticks must not use stale GlobalTransforms from the previous render.
    let (gimbal_rotation, muzzle_rotation) = fresh.unwrap_or((gimbal_global.rotation(), muzzle_global.rotation()));
    if config.vehicle.gimbal_pid.fixed_update && tracker.last_command_at.is_some_and(|t| t.elapsed().as_secs_f32() > 0.25) {
        tracker.feedforward = Vec2::ZERO;
        return;
    }

    // Error is measured on the muzzle, so chassis motion shows up as tracking error
    // instead of being cancelled out by a one-shot correction.
    let Some(effective_target)=tracker.effective_target() else {return;};
    let error = tracking_error(
        effective_target.rotation(),
        gimbal_transform.rotation,
        gimbal_rotation,
        muzzle_rotation,
    );
    let (yaw_error, pitch_error) = (error.x, error.y);
    let pid = &config.vehicle.gimbal_pid;
    let yaw_feedback = tracker.yaw.step(yaw_error, &pid.yaw, dt);
    let pitch_feedback = tracker.pitch.step(pitch_error, &pid.pitch, dt);
    let mut feedforward = Vec2::ZERO;
    if pid.feedforward_gain != 0.0 && tracker.last_command_at.is_some_and(|t| t.elapsed().as_secs_f32() <= pid.feedforward_reset_gap_s) {
        let h = 0.001;
        let next = GimbalAimTarget { yaw:effective_target.yaw+tracker.feedforward.x*h,
            pitch:effective_target.pitch+tracker.feedforward.y*h };
        let next_error = tracking_error(next.rotation(),gimbal_transform.rotation,gimbal_rotation,muzzle_rotation);
        // Express the target's world angular velocity in the local actuator axes.
        feedforward = Vec2::new(wrap_angle(next_error.x-error.x),wrap_angle(next_error.y-error.y)) / h * pid.feedforward_gain;
    }
    let yaw_rate = (yaw_feedback+feedforward.x).clamp(-pid.yaw.max_rate,pid.yaw.max_rate);
    let pitch_rate = (pitch_feedback+feedforward.y).clamp(-pid.pitch.max_rate,pid.pitch.max_rate);
    tracker.control_steps += 1;
    tracker.last_dt = dt;
    tracker.last_muzzle_yaw=muzzle_rotation.to_euler(EulerRot::YXZ).0;
    tracker.last_error = error;
    tracker.last_feedback = Vec2::new(yaw_feedback,pitch_feedback);
    tracker.last_feedforward = feedforward;

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);
    let yaw_delta=tracker.advance_yaw(&pid.dynamics,yaw_rate,dt,pid.yaw.max_rate,gimbal_data.local_yaw);
    gimbal_data.local_yaw += yaw_delta;
    gimbal_data.pitch = (gimbal_data.pitch + pitch_rate * dt).clamp(
        -config.vehicle.gimbal_pitch_limit,
        config.vehicle.gimbal_pitch_limit,
    );

    gimbal_transform.rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);
}

#[cfg(feature = "bench")]
pub fn publish_gimbal_control_trace(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    stamp: Res<crate::ros2::plugin::RosFrameStamp>,
    publisher: Res<crate::ros2::topic::TopicPublisher<crate::ros2::topic::bench_topics::SimGimbalTraceTopic>>,
    gimbal: Query<&GimbalAimTracker, (With<Controlled>, With<InfantryGimbal>)>,
    mut previous: Local<f64>,
) {
    if time.elapsed_secs_f64()-*previous < 0.01 { return; }
    *previous = time.elapsed_secs_f64();
    let Ok(tracker) = gimbal.single() else { return; };
    let mut value = tracker.diagnostic();
    value["stamp_ns"] = serde_json::json!(i64::from(stamp.0.sec)*1_000_000_000+i64::from(stamp.0.nanosec));
    value["gimbal_dynamics"] = serde_json::json!(config.vehicle.gimbal_pid.dynamics);
    value["fixed_update"] = serde_json::json!(config.vehicle.gimbal_pid.fixed_update);
    value["feedforward_gain"] = serde_json::json!(config.vehicle.gimbal_pid.feedforward_gain);
    publisher.publish(r2r::std_msgs::msg::String { data:value.to_string() });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corrected_production_selection_retarget_reset_and_raw_gain() {
        for (id,expected_dc) in [("rm2017-yaw-corrected-raw",1.1117948717948718),
            ("rm2017-yaw-corrected-unitdc",1.0)] {
            let mut tracker=GimbalAimTracker::new(GimbalAimTarget{yaw:0.0,pitch:0.0});
            tracker.yaw.integral=0.25;tracker.feedforward=Vec2::new(0.2,0.1);
            let first=tracker.advance_yaw(id,1.0,0.001,20.0,0.0);
            assert!(first<0.0 && tracker.model_diagnostic[2]<0.0);
            let state=tracker.yaw_model.x;let command=GimbalAimTarget{yaw:0.02,pitch:0.0};
            tracker.retarget(command);assert_eq!(tracker.yaw_model.x,state);
            tracker.advance_yaw(id,1.0,5.0,20.0,first);
            assert!((tracker.model_diagnostic[2]-expected_dc).abs()<1e-12);
            assert!(tracker.model_effective_ns.is_some());
            tracker.reset_yaw_model();assert_eq!(tracker.yaw_model.x,[0.0,0.0]);
            assert_eq!(tracker.model_diagnostic,[0.0;5]);assert!(tracker.model_effective_ns.is_none());
            assert_eq!(tracker.yaw.integral,0.25);assert_eq!(tracker.feedforward,Vec2::new(0.2,0.1));
            assert_eq!(tracker.target,command);
            assert!(tracker.advance_yaw(id,1.0,0.001,20.0,0.0)<0.0);
        }
        let mut legacy=GimbalAimTracker::new(GimbalAimTarget{yaw:0.0,pitch:0.0});
        assert_eq!(legacy.advance_yaw("legacy",0.3,0.002,20.0,0.0),0.3_f32*0.002_f32);
        assert_eq!(legacy.yaw_model.x,[0.0,0.0]);
    }

    #[test]
    fn corrected_production_history_dispatch_is_bit_exact() {
        let mut tracker=GimbalAimTracker::new(GimbalAimTarget{yaw:0.0,pitch:0.0});
        let mut original=super::super::yaw_reference::YawReference::default();
        for k in 0..1000 {
            let input=if k<300 {0.3}else if k<700 {-20.0}else{10.0};
            let expected=original.advance(input as f64,0.002_f32 as f64,0.7_f32 as f64,true);
            let actual=tracker.advance_yaw("rm2017-yaw-unitdc",input,0.002,0.7,0.0);
            assert_eq!(actual,expected.angle as f32);assert_eq!(tracker.yaw_model.x,original.x);
            assert_eq!(tracker.model_diagnostic[1],expected.raw_rate);
            assert_eq!(tracker.model_diagnostic[2],expected.rate);
            assert_eq!(tracker.model_limited,expected.limited);
        }
    }

    /// Export with `cargo test ... corrected_angle_loop_csv -- --ignored --nocapture`.
    /// This yaw-only component harness uses production AxisPid, stamped target
    /// feedforward, angle wrapping and actuator dispatch. The local/world yaw
    /// frames are identical. It does not simulate ECS transforms, ROS timing,
    /// chassis motion or camera/vision; it is not a full vehicle qualification.
    #[test]
    #[ignore = "CSV component diagnostic; run explicitly with --nocapture"]
    fn corrected_angle_loop_csv() {
        println!("model,signal,ff_gain,end_t_s,target_sample_t_s,target_rad,yaw_rad,error_rad,feedback_rad_s,ff_rad_s,input_rad_s,raw_output_rad_s,output_rad_s,delta_rad,limited");
        for id in ["legacy","rm2017-yaw-unitdc","rm2017-yaw-corrected-raw","rm2017-yaw-corrected-unitdc"] {
            for signal in ["step","ramp","chirp"] {for gain in [0.0,1.0] {
                let mut config=GimbalPidConfig::default();config.feedforward_gain=gain;
                let mut tracker=GimbalAimTracker::new(GimbalAimTarget{yaw:0.0,pitch:0.0});let mut angle=0.0;
                let dt=0.001;let mut observed_ff=false;
                for k in 0..4000 {
                    let t=k as f32*dt;
                    let requested=match signal {"step"=>if t>=0.1 {0.01}else{0.0},
                        "ramp"=>0.002*t,
                        _=>0.001*(std::f32::consts::TAU*(0.5*t+2.4375*t*t)).sin()};
                    // Timestamped commands at 100 Hz; actuator at 1000 Hz.
                    // 1 ms commands would hit the estimator's >1 ms guard and
                    // accidentally disable the feedforward diagnostic.
                    if k%10==0 {tracker.retarget_stamped(GimbalAimTarget{yaw:requested,pitch:0.0},1_000_000_000+k as i128*1_000_000,&config);}
                    let target=tracker.target.yaw;
                    let error=wrap_angle(target-angle);let feedback=tracker.yaw.step(error,&config.yaw,dt);
                    // In the yaw-only aligned frame, target world angular rate
                    // maps directly to local yaw; same timestamped estimator.
                    let ff=tracker.feedforward.x*config.feedforward_gain;
                    observed_ff|=ff.abs()>1e-8;
                    let input=(feedback+ff).clamp(-config.yaw.max_rate,config.yaw.max_rate);
                    let delta=tracker.advance_yaw(id,input,dt,config.yaw.max_rate,angle);angle+=delta;
                    assert!(angle.is_finite() && delta.is_finite());
                    let (raw,output)=if id=="legacy" {(input as f64,input as f64)}else{(tracker.model_diagnostic[1],tracker.model_diagnostic[2])};
                    assert!(output.abs()<=config.yaw.max_rate as f64);
                    println!("{id},{signal},{gain},{},{},{target},{angle},{error},{feedback},{ff},{input},{raw},{output},{delta},{}",t+dt,(k/10*10) as f32*dt,tracker.model_limited);
                }
                if gain!=0.0 {assert!(observed_ff,"feedforward case must actually activate");}
            }}
        }
    }

    #[test]
    fn reference_reset_retarget_wrap_and_long_gap() {
        let mut t=GimbalAimTracker::new(GimbalAimTarget {yaw:std::f32::consts::PI-0.001,pitch:0.0});
        t.yaw.integral=0.25;t.feedforward=Vec2::new(0.2,0.1);
        t.yaw_model.advance(1.0,0.1,20.0,true);let state=t.yaw_model.x;
        t.retarget(GimbalAimTarget{yaw:-std::f32::consts::PI+0.001,pitch:0.0});
        assert_eq!(t.yaw_model.x,state);
        assert!((wrap_angle(t.target.yaw-(std::f32::consts::PI-0.001))-0.002).abs()<1e-6);
        let a=t.yaw_model.advance(-1.0,5.0,20.0,true);
        assert!((a.rate+1.0).abs()<1e-12 && a.angle.is_finite());
        t.reset_yaw_model();assert_eq!(t.yaw_model.x,[0.0,0.0]);
        assert_eq!(t.yaw.integral,0.25);assert_eq!(t.feedforward,Vec2::new(0.2,0.1));
        assert!(t.yaw_model.advance(1.0,0.001,20.0,true).rate<0.0);
    }

    #[test]
    fn temporal_target_extrapolates_and_expires() {
        let mut tracker=GimbalAimTracker::new(GimbalAimTarget {yaw:0.2,pitch:-0.1});
        tracker.temporal=Some((1_000_000_000,Vec2::new(0.5,-0.2)));
        let target=tracker.effective_target_at(1_020_000_000).unwrap();
        assert!((target.yaw-0.21).abs()<1e-6);
        assert!((target.pitch+0.104).abs()<1e-6);
        assert!(tracker.effective_target_at(1_101_000_000).is_none());
        assert!(tracker.effective_target_at(949_000_000).is_none());
        tracker.temporal=None;
        assert_eq!(tracker.effective_target_at(9_000_000_000).unwrap().yaw,0.2);
    }

    #[test]
    fn temporal_target_rejects_nonfinite_rates_and_stale_stamps() {
        let mut tracker=GimbalAimTracker::new(GimbalAimTarget {yaw:0.,pitch:0.});
        let now=crate::clock::now_ns() as i64;
        tracker.temporal_setpoint(true,now,0.5,0.);
        assert!(tracker.temporal.is_some());
        tracker.temporal_setpoint(true,now,f64::NAN,0.);
        assert!(tracker.temporal.is_none());
        tracker.temporal_setpoint(true,now-200_000_000,0.5,0.);
        assert!(tracker.temporal.is_none());
        tracker.temporal_setpoint(true,now,0.7,0.);
        assert!(tracker.temporal.is_none());
    }

    fn axis(kp: f32, max_rate: f32) -> GimbalAxisPidConfig {
        GimbalAxisPidConfig {
            kp,
            ki: 0.0,
            kd: 0.0,
            integral_limit: 1.0,
            max_rate,
        }
    }

    #[test]
    fn proportional_rate_follows_error_sign_and_magnitude() {
        let mut pid = AxisPid::default();

        assert!((pid.step(0.2, &axis(10.0, 20.0), 0.01) - 2.0).abs() < 1e-5);
        assert!((pid.step(-0.1, &axis(10.0, 20.0), 0.01) + 1.0).abs() < 1e-5);
    }

    #[test]
    fn each_axis_uses_its_own_gains_and_rate_limit() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let yaw_config = axis(10.0, 20.0);
        let pitch_config = axis(4.0, 1.0);

        let yaw_rate = tracker.yaw.step(0.5, &yaw_config, 0.01);
        let pitch_rate = tracker.pitch.step(0.5, &pitch_config, 0.01);

        assert!((yaw_rate - 5.0).abs() < 1e-5);
        assert_eq!(pitch_rate, 1.0);
    }

    #[test]
    fn axis_loop_state_is_not_shared_between_yaw_and_pitch() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let config = axis(0.0, 20.0);

        tracker.yaw.step(1.0, &config, 0.5);

        assert_eq!(tracker.yaw.integral, 0.5);
        assert_eq!(tracker.pitch.integral, 0.0);
        assert_eq!(tracker.pitch.previous_error, 0.0);
    }

    #[test]
    fn integral_term_is_bounded_against_windup() {
        let mut config = axis(0.0, 20.0);
        config.ki = 1.0;
        let mut pid = AxisPid::default();

        for _ in 0..1000 {
            pid.step(1.0, &config, 0.01);
        }

        assert_eq!(pid.integral, config.integral_limit);
    }

    #[test]
    fn retarget_keeps_loop_state_so_command_stream_is_continuous() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        tracker.yaw.step(0.1, &axis(10.0, 20.0), 0.01);
        let integral = tracker.yaw.integral;

        tracker.retarget(GimbalAimTarget {
            yaw: 0.5,
            pitch: 0.1,
        });

        assert_eq!(tracker.yaw.integral, integral);
        assert_eq!(tracker.target().yaw, 0.5);
    }

    #[test]
    fn error_points_the_same_way_as_the_local_angles_on_a_bare_gimbal() {
        let target = Quat::from_euler(EulerRot::YXZ, 0.3, -0.2, 0.0);
        let error = tracking_error(target, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY);

        assert!((error.x - 0.3).abs() < 1e-5);
        assert!((error.y + 0.2).abs() < 1e-5);
    }

    /// Guards the pitch sign. The gimbal's pitch axis is its *local* X, which points
    /// against world X once the gimbal has yawed past 90 degrees; an error read off the
    /// world-space correction is inverted there and drives pitch away from the target.
    #[test]
    fn pitch_error_keeps_its_sign_when_the_gimbal_faces_backwards() {
        let local = Quat::from_rotation_y(std::f32::consts::PI);
        let target = local * Quat::from_rotation_x(0.1);
        let error = tracking_error(target, local, local, local);

        assert!(error.x.abs() < 1e-5, "yaw leaked {}", error.x);
        assert!((error.y - 0.1).abs() < 1e-5, "pitch error was {}", error.y);
    }

    #[test]
    fn closed_loop_converges_through_a_mount_offset_and_chassis_yaw() {
        let chassis = Quat::from_rotation_y(2.8);
        let mount = Quat::from_euler(EulerRot::YXZ, 0.4, 0.25, 0.15);
        let goal = Quat::from_euler(EulerRot::YXZ, 0.3, -0.2, 0.0);
        let target = chassis * goal * mount;

        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let yaw_config = axis(10.0, 20.0);
        let pitch_config = axis(6.0, 20.0);
        let mut local = Quat::IDENTITY;
        let dt = 1.0 / 120.0;

        for _ in 0..600 {
            let gimbal_world = chassis * local;
            let error = tracking_error(target, local, gimbal_world, gimbal_world * mount);
            let (mut yaw, mut pitch, _) = local.to_euler(EulerRot::YXZ);
            yaw += tracker.yaw.step(error.x, &yaw_config, dt) * dt;
            pitch += tracker.pitch.step(error.y, &pitch_config, dt) * dt;
            local = Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
        }

        let (yaw, pitch, _) = local.to_euler(EulerRot::YXZ);
        assert!((yaw - 0.3).abs() < 1e-3, "yaw settled at {yaw}");
        assert!((pitch + 0.2).abs() < 1e-3, "pitch settled at {pitch}");
    }

    #[test]
    fn wrap_angle_takes_the_short_way_around() {
        assert!((wrap_angle(std::f32::consts::TAU + 0.2) - 0.2).abs() < 1e-5);
        assert!((wrap_angle(std::f32::consts::PI + 0.1) + std::f32::consts::PI - 0.1).abs() < 1e-5);
    }
}
