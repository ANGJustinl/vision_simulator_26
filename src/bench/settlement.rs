//! Cumulative, reliable evaluation-only settlement. Never an algorithm input.
use bevy::prelude::*;
use std::collections::BTreeMap;
use crate::bench::events::ShotSequence;
use crate::ros2::{plugin::RosFrameStamp, topic::{TopicPublisher, bench_topics::SimSettlementTopic}};

#[derive(Resource, Default)]
pub struct Settlement {
    generation: u64,
    terminals: BTreeMap<u32, &'static str>,
}

impl Settlement {
    fn sync(&mut self, generation: u64) {
        if self.generation != generation {
            self.generation = generation;
            self.terminals.clear();
        }
    }

    pub fn finish(&mut self, sequence: &ShotSequence, shot: u32, reason: &'static str) {
        self.sync(sequence.generation());
        self.terminals.entry(shot).or_insert(reason);
    }
}

pub fn publish(
    sequence: Res<ShotSequence>,
    mut ledger: ResMut<Settlement>,
    time: Res<Time>,
    stamp: Res<RosFrameStamp>,
    publisher: Res<TopicPublisher<SimSettlementTopic>>,
    mut previous: Local<f64>,
) {
    ledger.sync(sequence.generation());
    if time.elapsed_secs_f64()-*previous < 0.05 { return; }
    *previous=time.elapsed_secs_f64();
    let terminals: Vec<_> = ledger.terminals.iter().map(|(shot,reason)|
        serde_json::json!({"shot_seq":shot,"reason":reason})).collect();
    publisher.publish(r2r::std_msgs::msg::String { data: serde_json::json!({
        "version":1,"method":"swept_actual_segment_v3",
        "stamp_ns":i64::from(stamp.0.sec)*1_000_000_000+i64::from(stamp.0.nanosec),
        "generation":sequence.generation(),"allocated":sequence.allocated(),
        "terminals":terminals}).to_string() });
}
