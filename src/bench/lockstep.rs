//! A single acknowledged main-loop step. Inference never runs inside Bevy.
use bevy::prelude::*;
use bevy::render::extract_resource::{ExtractResource,ExtractResourcePlugin};
use bevy::time::{TimeSystems,TimeUpdateStrategy};
use bevy::ecs::system::RunSystemOnce;
use serde_json::{Value,json};
use std::io::{BufRead,BufReader,Write};
use std::os::unix::net::UnixListener;
use std::sync::{Mutex,mpsc};
use std::time::Duration;

#[derive(Resource,Clone,ExtractResource,Default)]
pub struct StepFrame { pub capture: bool, pub stamp_ns:i64, pub id:u64 }

struct Request { value:Value, reply:mpsc::SyncSender<Value> }
#[derive(Resource)]
struct Driver { requests:Mutex<mpsc::Receiver<Request>>, active:Option<Request>, id:u64 }

pub struct LockstepPlugin;
impl Plugin for LockstepPlugin {
    fn build(&self,app:&mut App) {
        if !crate::clock::enabled() {return;}
        let path=std::env::var("DAEDALUS_STEP_SOCKET").unwrap_or_else(|_|"/run/rm26-sim/lockstep.sock".into());
        let _=std::fs::remove_file(&path);
        let listener=UnixListener::bind(&path).expect("bind private lockstep socket");
        let (sender,receiver)=mpsc::sync_channel::<Request>(1);
        std::thread::spawn(move || {
            let mut last_id=0;let mut last_request=Value::Null;let mut last_reply=Value::Null;
            for stream in listener.incoming() {
                let Ok(mut stream)=stream else {continue;};
                let mut reader=BufReader::new(stream.try_clone().unwrap());
                loop {
                    let mut line=String::new();
                    if reader.read_line(&mut line).unwrap_or(0)==0 {break;}
                    let reply=match serde_json::from_str::<Value>(&line) {
                        Ok(value) if line.len()<=65536 => {
                            let id=value["id"].as_u64().unwrap_or(0);
                            if id==last_id && value==last_request {last_reply.clone()}
                            else if id!=last_id+1 {json!({"ok":false,"error":"step id must be consecutive"})}
                            else {
                                let (tx,rx)=mpsc::sync_channel(1);
                                if sender.send(Request {value:value.clone(),reply:tx}).is_err() {break;}
                                let Ok(response)=rx.recv() else {break;};
                                if response["ok"]==true {last_id=id;last_request=value;last_reply=response.clone();}
                                response
                            }
                        },
                        _=>json!({"ok":false,"error":"invalid step request"}),
                    };
                    if writeln!(stream,"{}",reply).is_err() {break;}
                }
            }
        });
        app.insert_resource(Driver {requests:Mutex::new(receiver),active:None,id:0})
            .init_resource::<StepFrame>()
            .add_plugins(ExtractResourcePlugin::<StepFrame>::default())
            .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::ZERO))
            .add_systems(First,begin.before(TimeSystems))
            .add_systems(Last,finish.after(super::events::publish_fire_events).after(super::settlement::publish));
    }
}

fn begin(world:&mut World) {
    loop {
        let request=world.resource::<Driver>().requests.lock().unwrap().recv().expect("step coordinator disconnected");
        let value=&request.value;
        if value["observe_only"].as_bool().unwrap_or(false) {
            if value["delta_ns"].as_u64()!=Some(0) || value.get("command").is_some()
                || value.get("sensor_reset").is_some()
                || ["reset_scene","begin_scene","align_epoch","capture"].iter().any(|k|value[*k].as_bool().unwrap_or(false)) {
                let _=request.reply.send(json!({"ok":false,"error":"observation request cannot mutate the world"}));
                continue;
            }
            crate::clock::set_observing(true);
            world.run_system_once(crate::ros2::plugin::capture_rune).expect("snapshot TF");
            world.run_system_once(super::gt::publish_armor_gt).expect("snapshot armor evidence");
            world.run_system_once(super::gt::publish_robot_gt).expect("snapshot robot evidence");
            world.run_system_once(super::scenario::publish_scenario_status).expect("snapshot scenario status");
            crate::clock::set_observing(false);
            let generation=world.resource::<super::events::ShotSequence>().generation();
            world.run_system_once(super::phase::publish).expect("snapshot physical phase");
            let phase=world.resource::<super::phase::PhaseBoundary>().snapshot.clone();
            let _=request.reply.send(json!({"ok":true,"id":value["id"],"stamp_ns":crate::clock::now_ns() as i64,"generation":generation,"observation_only":true,"scenario_boundary":phase}));
            continue;
        }
        let delta=value["delta_ns"].as_u64().filter(|n|*n>0 && *n<=100_000_000);
        let generation=world.get_resource::<super::events::ShotSequence>().map(|s|s.generation()).unwrap_or(0);
        let valid_generation=value.get("generation").and_then(Value::as_u64).is_none_or(|g|g==generation);
        let parsed=value.get("command").map(|cmd|serde_json::from_value::<r2r::rm_interfaces::msg::GimbalCmd>(cmd.clone())).transpose();
        let valid_command=parsed.as_ref().is_ok_and(|cmd|cmd.as_ref().is_none_or(|c|c.yaw.is_finite() && c.pitch.is_finite() && c.distance.is_finite()));
        let begin=value["begin_scene"].as_bool().unwrap_or(false);
        let reset=value["reset_scene"].as_bool().unwrap_or(false);
        let sensor_reset=value.get("sensor_reset").and_then(Value::as_str);
        let sensor_seed=world.resource::<crate::config::SimulationConfig>().capture.effect.sensor_seed;
        let valid_sensor=value.get("sensor_reset").is_none() || (sensor_reset.is_some_and(|s|!s.is_empty() && s.len()<=64)
            && sensor_seed.is_some() && !begin && world.get_resource::<super::scenario::Scenario>().is_some_and(|s|s.phase==super::scenario::Phase::Idle));
        let valid_begin=!begin || world.get_resource::<super::scenario::Scenario>().is_some_and(|s|s.phase==super::scenario::Phase::Idle);
        let valid_reset=!reset || (!begin && !value["capture"].as_bool().unwrap_or(false)
            && world.get_resource::<super::scenario::Scenario>().is_some_and(|s|s.phase==super::scenario::Phase::Idle));
        if delta.is_none() || !valid_generation || !valid_command || !valid_begin || !valid_reset || !valid_sensor {
            let _=request.reply.send(json!({"ok":false,"error":"invalid step delta or scenario generation","generation":generation}));
            continue;
        }
        let delta=delta.unwrap();let id=value["id"].as_u64().unwrap();
        if let Some(reset_id)=sensor_reset {
            world.resource::<crate::capture::sensor::SensorSequence>().reset(reset_id.to_string(),sensor_seed.unwrap());
        }
        if begin {
            let mut scenario=world.resource_mut::<super::scenario::Scenario>();
            scenario.pending=super::scenario::ScenarioCommand::Begin;
        }
        if reset {
            // Reuse the existing physical reset; no new motion or physics
            // model. ROS readiness can arrive between arbitrary ticks, so
            // settling starts from this acknowledged reset boundary instead.
            world.resource_mut::<super::scenario::Scenario>().pending=super::scenario::ScenarioCommand::Reset;
        }
        let now=crate::clock::now_ns() as i64+delta as i64;
        crate::clock::set_ns(now);
        if value["align_epoch"].as_bool().unwrap_or(false) {
            crate::clock::reset_origin();
            let overstep=world.resource::<Time<Fixed>>().overstep();
            world.resource_mut::<Time<Fixed>>().discard_overstep(overstep);
            world.resource_mut::<bevy::diagnostic::FrameCount>().0=0;
        }
        *world.resource_mut::<TimeUpdateStrategy>()=TimeUpdateStrategy::ManualDuration(Duration::from_nanos(delta));
        *world.resource_mut::<StepFrame>()=StepFrame {id,capture:value["capture"].as_bool().unwrap_or(false),stamp_ns:now};
        if let Some(cmd)=parsed.unwrap() {
            world.resource_mut::<crate::ros2::plugin::CoordinatedCommands>().0.push_back(cmd);
        }
        if let Some(gate)=value.get("eligibility") {
            crate::ros2::plugin::inject_lockstep_eligibility(world,gate);
        }
        let mut driver=world.resource_mut::<Driver>();driver.id=id;driver.active=Some(request);
        break;
    }
}

fn finish(world:&mut World) {
    let generation=world.resource::<super::events::ShotSequence>().generation();
    let frame=world.resource::<StepFrame>().clone();
    if let Some(request)=world.resource_mut::<Driver>().active.take() {
        let mut reply=json!({"ok":true,"id":frame.id,"stamp_ns":frame.stamp_ns,"generation":generation});
        reply["scenario_boundary"]=world.resource::<super::phase::PhaseBoundary>().snapshot.clone().unwrap_or(Value::Null);
        if let Some(id)=request.value.get("sensor_reset") {reply["sensor_reset_id"]=id.clone();}
        if request.value["reset_scene"].as_bool().unwrap_or(false) {reply["reset_scene"]=json!(true);}
        let _=request.reply.send(reply);
    }
}
