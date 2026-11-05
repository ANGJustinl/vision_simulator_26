//! Explicit, capture-indexed sensor randomness. The native lens shader is kept.
use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use std::sync::{Arc,Mutex};

#[derive(Resource,Clone,Default)]
pub struct SensorSequence(pub Arc<Mutex<SensorFrame>>);

#[derive(Resource,Clone,Default,ExtractResource)]
pub struct SensorFrame {
    pub seed: Option<u32>,
    pub index: u64,
    pub reset_id: String,
    pub phase: f32,
}

pub fn phase(seed:u32,index:u64)->f32 {
    let mut x=seed.wrapping_add((index as u32).wrapping_mul(0x9e3779b9))
        ^ ((index>>32) as u32).wrapping_mul(0x85ebca6b);
    x^=x>>16;x=x.wrapping_mul(0x7feb352d);x^=x>>15;
    x=x.wrapping_mul(0x846ca68b);x^=x>>16;
    (x & 0x00ffffff) as f32 / 4096.0
}

impl SensorSequence {
    pub fn reset(&self,id:String,seed:u32) {
        *self.0.lock().unwrap()=SensorFrame{seed:Some(seed),index:0,reset_id:id,phase:phase(seed,0)};
    }
    pub fn current(&self)->SensorFrame {self.0.lock().unwrap().clone()}
    pub fn captured(&self,frame:&SensorFrame) {
        let mut state=self.0.lock().unwrap();
        // An old render can finish after a reset. Its metadata stays old and
        // it must never advance the new sequence.
        if state.reset_id==frame.reset_id && state.index==frame.index {
            state.index+=1;
            state.phase=phase(state.seed.unwrap(),state.index);
        }
    }
}

#[cfg(all(feature="ros2",feature="bench"))]
pub fn poll_control(
    config: Res<crate::config::SimulationConfig>,
    scene: Option<Res<crate::bench::scenario::Scenario>>,
    sequence: Res<SensorSequence>,
    input: Option<Res<crate::ros2::topic::TopicSubscriber<crate::ros2::topic::bench_topics::SimSensorControlTopic>>>,
    output: Option<Res<crate::ros2::topic::TopicPublisher<crate::ros2::topic::bench_topics::SimSensorStateTopic>>>,
) {
    // The lens plugin can run without ROS. Do not consume a reset unless its
    // acknowledgement channel exists as well.
    let (Some(input), Some(output)) = (input, output) else { return; };
    while let Some(message)=input.try_recv() {
        let Ok(value)=serde_json::from_str::<serde_json::Value>(&message.data) else {continue;};
        let Some(id)=value["reset_id"].as_str().filter(|id|!id.is_empty() && id.len()<=64) else {continue;};
        let allowed=scene.as_ref().is_some_and(|s|s.phase==crate::bench::scenario::Phase::Idle);
        if let Some(seed)=config.capture.effect.sensor_seed.filter(|_|allowed) {
            if sequence.current().reset_id!=id {sequence.reset(id.to_string(),seed);}
            output.publish(r2r::std_msgs::msg::String{data:serde_json::json!({"version":1,"kind":"reset",
                "reset_id":id,"seed":seed,"stamp_ns":crate::clock::now_ns() as i64}).to_string()});
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_and_duplicate_captures_cannot_advance_a_new_session() {
        let sequence = SensorSequence::default();
        sequence.reset("old".into(), 1);
        let old = sequence.current();
        sequence.reset("new".into(), 2);
        sequence.captured(&old);
        assert_eq!(sequence.current().index, 0);
        let new = sequence.current();
        sequence.captured(&new);
        sequence.captured(&new);
        let current = sequence.current();
        assert_eq!(current.reset_id, "new");
        assert_eq!(current.seed, Some(2));
        assert_eq!(current.index, 1);
        assert_eq!(current.phase, phase(2, 1));
    }
}
