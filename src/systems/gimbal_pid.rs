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
    target: GimbalAimTarget,
    yaw: AxisPid,
    pitch: AxisPid,
    command_stamp_ns: Option<i128>,
    accepted_wall_ns: i128,
    last_command_at: Option<std::time::Instant>,
    feedforward: Vec2,
    control_steps: u64,
    last_dt: f32,
    last_error: Vec2,
    last_feedback: Vec2,
    last_feedforward: Vec2,
}

impl GimbalAimTracker {
    pub fn new(target: GimbalAimTarget) -> Self {
        Self {
            target,
            yaw: AxisPid::default(),
            pitch: AxisPid::default(),
            command_stamp_ns: None,
            accepted_wall_ns: 0,
            last_command_at: None,
            feedforward: Vec2::ZERO,
            control_steps: 0,
            last_dt: 0.0,
            last_error: Vec2::ZERO,
            last_feedback: Vec2::ZERO,
            last_feedforward: Vec2::ZERO,
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
        self.accepted_wall_ns = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default().as_nanos() as i128;
        self.last_command_at = Some(std::time::Instant::now());
        self.retarget(target);
    }

    pub fn diagnostic(&self) -> serde_json::Value {
        serde_json::json!({"command_stamp_ns":self.command_stamp_ns,
            "accepted_wall_ns":self.accepted_wall_ns,"target_yaw_rad":self.target.yaw,
            "target_pitch_rad":self.target.pitch,"control_steps":self.control_steps,
            "dt_s":self.last_dt,"error_local_rad":self.last_error.to_array(),
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
    let error = tracking_error(
        tracker.target().rotation(),
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
        let next = GimbalAimTarget { yaw:tracker.target.yaw+tracker.feedforward.x*h,
            pitch:tracker.target.pitch+tracker.feedforward.y*h };
        let next_error = tracking_error(next.rotation(),gimbal_transform.rotation,gimbal_rotation,muzzle_rotation);
        // Express the target's world angular velocity in the local actuator axes.
        feedforward = Vec2::new(wrap_angle(next_error.x-error.x),wrap_angle(next_error.y-error.y)) / h * pid.feedforward_gain;
    }
    let yaw_rate = (yaw_feedback+feedforward.x).clamp(-pid.yaw.max_rate,pid.yaw.max_rate);
    let pitch_rate = (pitch_feedback+feedforward.y).clamp(-pid.pitch.max_rate,pid.pitch.max_rate);
    tracker.control_steps += 1;
    tracker.last_dt = dt;
    tracker.last_error = error;
    tracker.last_feedback = Vec2::new(yaw_feedback,pitch_feedback);
    tracker.last_feedforward = feedforward;

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);
    gimbal_data.local_yaw += yaw_rate * dt;
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
    value["fixed_update"] = serde_json::json!(config.vehicle.gimbal_pid.fixed_update);
    value["feedforward_gain"] = serde_json::json!(config.vehicle.gimbal_pid.feedforward_gain);
    publisher.publish(r2r::std_msgs::msg::String { data:value.to_string() });
}

#[cfg(test)]
mod tests {
    use super::*;

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
