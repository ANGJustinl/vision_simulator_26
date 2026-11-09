//! Explicit bench-only continuous firing, independent of vision eligibility.
//! Uses the normal projectile launcher and its 2 Hz integration limit.
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use crate::components::ProjectileCooldown;
use crate::ros2::topic::{TopicSubscriber, bench_topics::SimContinuousFireTopic};
use crate::systems::{IntegrationShotPermit, projectile_launch};

#[derive(Resource, Default)]
pub struct ContinuousFire {
    stamp_ns: i128,
    enabled: bool,
}

pub fn fire(world: &mut World) {
    if std::env::var("DAEDALUS_INTEGRATION").as_deref() != Ok("1")
        || std::env::var("DAEDALUS_BENCH_FIRE_POLICY").as_deref() != Ok("continuous")
        || std::env::var("DAEDALUS_INTEGRATION_FIRE").as_deref() != Ok("auto") { return; }
    let mut latest = None;
    let subscriber = world.resource::<TopicSubscriber<SimContinuousFireTopic>>();
    while let Some(message) = subscriber.try_recv() { latest = Some(message); }
    if let Some(message) = latest {
        let parsed = serde_json::from_str::<serde_json::Value>(&message.data).ok();
        let mut state = world.resource_mut::<ContinuousFire>();
        state.enabled = parsed.as_ref().and_then(|v| v["enabled"].as_bool()).unwrap_or(false);
        state.stamp_ns = parsed.as_ref().and_then(|v| v["stamp_ns"].as_i64()).unwrap_or(0) as i128;
    }
    let now_ns = crate::clock::now_ns();
    let state = world.resource::<ContinuousFire>();
    if !state.enabled || now_ns-state.stamp_ns > 250_000_000 || state.stamp_ns > now_ns+10_000_000
        || !world.get_resource::<super::scenario::Scenario>()
            .is_some_and(|s| s.phase == super::scenario::Phase::Running) { return; }
    let delta = world.resource::<Time>().delta();
    world.resource_mut::<ProjectileCooldown>().tick(delta);
    if world.resource::<IntegrationShotPermit>().last_launch
        .is_some_and(|last| last.elapsed() < std::time::Duration::from_millis(500)) { return; }
    {
        let mut permit = world.resource_mut::<IntegrationShotPermit>();
        permit.authorized = true;
        permit.stamp_ns = now_ns;
    }
    world.run_system_once(projectile_launch).unwrap();
    world.resource_mut::<IntegrationShotPermit>().authorized = false;
}
