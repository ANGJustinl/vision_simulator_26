use avian3d::prelude::*;
use bevy::input::gamepad::{GamepadRumbleIntensity, GamepadRumbleRequest};
use bevy::prelude::*;
use core::{f32::consts::PI, time::Duration};

use crate::components::{
    Controlled, DartLaunch, DartProjectile, DartSetting, GameLayer, Infantry, InfantryChassis,
    InfantryGimbal, InfantryLaunchOffset, ProjectileCooldown, ProjectileLifetime,
    ProjectileSetting,
};
use crate::config::SimulationConfig;
use crate::robomaster::prelude::Projectile;
use crate::statistic::ProjectileStatistics;
use crate::systems::{ControllerState, request_controller_rumble};

/// Granted only around a fresh ROS launch attempt; local inputs cannot use it.
#[derive(Resource, Default)]
pub(crate) struct IntegrationShotPermit {
    pub authorized: bool,
    pub stamp_ns: i128,
    pub last_launch: Option<crate::clock::Instant>,
}

pub fn setup_projectile(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(ProjectileSetting(
        meshes.add(Sphere::new(config.projectile.diameter / 2.0)),
        materials.add(StandardMaterial {
            base_color: Color::srgba(0.132866, 1.0, 0.132869, 0.85),
            emissive: LinearRgba::new(0.132866, 1.0, 0.132869, 0.85),
            emissive_exposure_weight: -1.0,
            alpha_mode: AlphaMode::Opaque,
            ..default()
        }),
    ));
    commands.insert_resource(DartSetting(
        asset_server.load(GltfAssetLabel::Scene(0).from_asset("DART.glb")),
    ));
}

pub fn projectile_launch(
    time: Res<Time>,
    mut cooldown: ResMut<ProjectileCooldown>,
    mut stats: ResMut<ProjectileStatistics>,
    config: Res<SimulationConfig>,
    _asset_server: Res<AssetServer>,
    mut commands: Commands,
    controller: Option<Res<ControllerState>>,
    mut rumble_requests: MessageWriter<GamepadRumbleRequest>,
    setting: Res<ProjectileSetting>,
    infantry: Single<
        (&Transform, &LinearVelocity, &AngularVelocity),
        (With<Infantry>, With<Controlled>),
    >,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<Controlled>, Without<InfantryChassis>),
    >,
    launch_offset: Single<&Transform, (With<Controlled>, With<InfantryLaunchOffset>)>,
    muzzle_world: Single<&GlobalTransform, (With<Controlled>, With<InfantryLaunchOffset>, Without<InfantryGimbal>)>,
    permit: Option<ResMut<IntegrationShotPermit>>,
    #[cfg(feature = "bench")] scenario: Option<Res<crate::bench::scenario::Scenario>>,
) {
    if std::env::var("DAEDALUS_INTEGRATION").is_ok_and(|value| value == "1") {
        let Some(mut permit) = permit else { return; };
        let authorized = std::mem::take(&mut permit.authorized);
        let now_ns = crate::clock::now_ns();
        let rate = std::env::var("DAEDALUS_INTEGRATION_FIRE_RATE_HZ").ok()
            .and_then(|value| value.parse::<f64>().ok()).unwrap_or(2.0);
        if !rate.is_finite() || rate <= 0.0 || rate > 5.0 { return; }
        if !authorized || std::env::var("DAEDALUS_INTEGRATION_FIRE").as_deref() != Ok("auto")
            || now_ns - permit.stamp_ns > 250_000_000 || permit.stamp_ns > now_ns + 10_000_000
            || permit.last_launch.is_some_and(|last| last.elapsed() < Duration::from_secs_f64(1.0/rate)) {
            return;
        }
        #[cfg(feature = "bench")]
        if !scenario.as_ref().is_some_and(|s| s.phase == crate::bench::scenario::Phase::Running) {
            return;
        }
        // The ROS path already advances the mechanical cooldown each frame.
        if !cooldown.is_finished() { return; }
        permit.last_launch = Some(crate::clock::Instant::now());
    }
    cooldown.tick(time.delta());
    if !cooldown.is_finished() {
        return;
    }
    cooldown.reset();

    stats.increase_launch();
    let direction = (gimbal.0.rotation() * launch_offset.rotation)
        .mul_vec3(Vec3::Y)
        .normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }
    let vel = infantry.1.0 + direction * config.projectile.speed;
    let mut projectile = commands.spawn((
        RigidBody::Dynamic,
        Collider::sphere(config.projectile.diameter / 2.0),
        Mass(config.projectile.mass),
        Friction::new(config.projectile.friction),
        Restitution::new(0.3),
        LinearDamping(config.projectile.linear_damping),
        GameLayer::projectile_collision_layers(true),
        if std::env::var("DAEDALUS_PROJECTILE_VISIBLE").as_deref()==Ok("off") {Visibility::Hidden} else {Visibility::Inherited},
        Mesh3d(setting.0.clone()),
        MeshMaterial3d(setting.1.clone()),
        LinearVelocity(vel),
        AngularVelocity(infantry.2.0),
        Transform::IDENTITY.with_translation(muzzle_world.translation()),
        ProjectileLifetime(Timer::from_seconds(
            config.projectile.lifetime,
            TimerMode::Once,
        )),
        Projectile,
    ));
    if std::env::var("DAEDALUS_INTEGRATION").is_ok_and(|value| value == "1") {
        projectile.insert((Sensor, SpeculativeMargin::ZERO, CollisionMargin(0.0)));
    }
    request_controller_rumble(
        controller.as_deref(),
        &mut rumble_requests,
        GamepadRumbleIntensity {
            strong_motor: 0.45,
            weak_motor: 0.2,
        },
        Duration::from_millis(80),
    );
}

pub fn projectile_aerodynamics(
    config: Res<SimulationConfig>,
    mut projectiles: Query<Forces, (With<Projectile>, Without<DartProjectile>)>,
) {
    let aero = &config.projectile.aerodynamics;
    if !aero.enabled {
        return;
    }

    let diameter = config.projectile.diameter;
    if diameter <= 0.0 {
        return;
    }
    let air_density = aero.air_density.max(0.0);
    let drag_coefficient = aero.drag_coefficient.max(0.0);
    if air_density == 0.0 || drag_coefficient == 0.0 {
        return;
    }

    let area = PI * (diameter * 0.5).powi(2);
    let wind = Vec3::new(aero.wind[0], aero.wind[1], aero.wind[2]);
    let k = 0.5 * air_density * drag_coefficient * area;

    for mut forces in projectiles.iter_mut() {
        let v_rel = forces.linear_velocity() - wind;
        let speed = v_rel.length();
        if speed <= 1e-3 {
            continue;
        }
        forces.apply_force(-k * speed * v_rel);
    }
}

pub fn dart_launch(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    mut stats: ResMut<ProjectileStatistics>,
    controller: Option<Res<ControllerState>>,
    mut rumble_requests: MessageWriter<GamepadRumbleRequest>,
    setting: Res<DartSetting>,
    launchers: Query<&GlobalTransform, With<DartLaunch>>,
) {
    const DART_FORWARD: Vec3 = Vec3::Y;
    const DART_MODEL_FORWARD: Vec3 = Vec3::NEG_Y;
    const DART_SPEED_MPS: f32 = 17.0;
    const DART_MASS_KG: f32 = 0.25;
    const DART_COLLIDER_RADIUS_M: f32 = 0.001;
    const DART_COLLIDER_LENGTH_M: f32 = 0.001;
    const DART_SPAWN_OFFSET_M: f32 = 0.00;

    let Ok(launcher) = launchers.single() else {
        return;
    };

    let direction = launcher
        .rotation()
        .mul_vec3(DART_FORWARD)
        .normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }

    stats.increase_launch();

    let transform =
        Transform::from_translation(launcher.translation() + direction * DART_SPAWN_OFFSET_M)
            .with_rotation(
                launcher.rotation() * Quat::from_rotation_arc(DART_MODEL_FORWARD, DART_FORWARD),
            );
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
        .with_default_layers(GameLayer::projectile_collision_layers(true))
    };
    commands.spawn((
        RigidBody::Dynamic,
        voxel(0.005),
        Mass(DART_MASS_KG),
        Friction::new(config.projectile.friction),
        Restitution::new(0.55),
        LinearDamping(config.projectile.linear_damping),
        GameLayer::projectile_collision_layers(true),
        WorldAssetRoot(setting.0.clone()),
        transform,
        LinearVelocity(direction * DART_SPEED_MPS),
        ProjectileLifetime(Timer::from_seconds(
            config.projectile.lifetime,
            TimerMode::Once,
        )),
        Projectile,
        DartProjectile,
    ));
    request_controller_rumble(
        controller.as_deref(),
        &mut rumble_requests,
        GamepadRumbleIntensity {
            strong_motor: 0.65,
            weak_motor: 0.35,
        },
        Duration::from_millis(140),
    );
}

pub fn cleanup_projectiles(
    time: Res<Time>,
    mut commands: Commands,
    mut projectiles: Query<(Entity, &mut ProjectileLifetime)>,
    #[cfg(feature = "bench")] tags: Query<&crate::bench::events::ShotTag>,
    #[cfg(feature = "bench")] sequence: Res<crate::bench::events::ShotSequence>,
    #[cfg(feature = "bench")] mut settlement: ResMut<crate::bench::settlement::Settlement>,
) {
    for (entity, mut lifetime) in &mut projectiles {
        lifetime.tick(time.delta());
        if lifetime.is_finished() {
            #[cfg(feature = "bench")]
            if crate::bench::strict_hits::enabled() {
                if let Ok(tag)=tags.get(entity) {
                    settlement.finish(&sequence,tag.shot_seq,"expired");
                }
            }
            commands.entity(entity).despawn();
        }
    }
}
