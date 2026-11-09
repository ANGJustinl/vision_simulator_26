//! Physical phase boundaries, independent of the human-facing 10 Hz status.
use bevy::prelude::*;
use serde_json::{Value,json};
use crate::ros2::topic::{TopicPublisher,bench_topics::SimScenarioBoundaryTopic};

#[derive(Resource,Default)]
pub struct PhaseBoundary {
    pub snapshot: Option<Value>,
    key: Option<(String,u64)>,
    previous: Option<u8>,
    start: Option<i64>,
    end: Option<i64>,
    published: Option<(String,u64,u8)>,
    last_publish: Option<i128>,
}

pub fn sample(
    scenario: Option<Res<super::scenario::Scenario>>,
    shots: Res<super::events::ShotSequence>,
    mut boundary: ResMut<PhaseBoundary>,
) {
    let Some(scene)=scenario else {return;};
    let generation=shots.generation();let key=(scene.id.clone(),generation);
    let now=crate::clock::now_ns() as i64;let phase=scene.phase.wire();
    if boundary.key.as_ref()!=Some(&key) {
        boundary.key=Some(key);boundary.previous=None;boundary.start=None;boundary.end=None;
    }
    if boundary.previous!=Some(phase) {
        if phase==1 {boundary.start=Some(now);boundary.end=None;}
        if phase==2 {boundary.end=Some(now);}
        if phase==0 {boundary.start=None;boundary.end=None;}
        boundary.previous=Some(phase);
    }
    boundary.snapshot=Some(json!({"version":1,"id":scene.id,"generation":generation,
        "phase":phase,"stamp_ns":now,"measurement_start_stamp_ns":boundary.start,
        "measurement_end_stamp_ns":boundary.end}));
}

pub fn publish(mut boundary: ResMut<PhaseBoundary>,publisher: Res<TopicPublisher<SimScenarioBoundaryTopic>>) {
    let Some(value)=boundary.snapshot.clone() else {return;};
    let key=(value["id"].as_str().unwrap().to_string(),value["generation"].as_u64().unwrap(),value["phase"].as_u64().unwrap() as u8);
    let now=crate::clock::now_ns();
    if boundary.published.as_ref()==Some(&key) && boundary.last_publish.is_some_and(|t|now-t<100_000_000)
        && !crate::clock::observing() {return;}
    publisher.publish(r2r::std_msgs::msg::String{data:value.to_string()});
    boundary.published=Some(key);boundary.last_publish=Some(now);
}
