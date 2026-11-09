//! Integration impacts from actual, bounded physics-step sphere sweeps.
//! Avian integrates gravity/drag. Integration projectiles are sensors so an
//! approximate contact solver cannot redirect them before this exact sweep.
use avian3d::prelude::*;
use bevy::prelude::*;
use r2r::std_msgs::msg::String as RosString;
use crate::bench::{events::{self, ShotTag, ShotResolved, ShotContact}, scenario::Scenario};
use crate::components::{DartProjectile, RobotId, GameLayer};
use crate::robomaster::prelude::{Armor, ArmorRoot, Projectile};
use crate::ros2::{plugin::RosFrameStamp, topic::{TopicPublisher, bench_topics::{SimHitEventTopic, SimProjectileTraceTopic}}};
use crate::statistic::ProjectileStatistics;
use crate::util::gt::{to_ros_vec3, plate_size};
use crate::util::entity_query::HierarchyQuery;

pub fn enabled() -> bool {
    std::env::var("DAEDALUS_INTEGRATION").is_ok_and(|v| v == "1")
}

// Shape-cast TOI witnesses can be millimetres apart on a grazing mesh edge.
// Refine against exact point-to-mesh distance before accepting the sphere hit.
fn refine_contact(shape: &Collider, position: Vec3, rotation: Quat,
    origin: Vec3, delta: Vec3, guess: f32, radius: f32) -> Option<(f32,Vec3)> {
    let reach = (radius*3.0+0.001)/delta.length().max(1e-8);
    let lo = (guess-reach).max(0.0);
    let hi = (guess+reach).min(1.0);
    let sample = |u: f32| {
        let centre = origin+delta*u;
        let (point,_) = shape.project_point(position,rotation,centre,false);
        (centre.distance(point),point)
    };
    let mut best = (0usize,f32::INFINITY);
    for i in 0..=24 {
        let distance = sample(lo+(hi-lo)*i as f32/24.0).0;
        if distance<best.1 { best=(i,distance); }
    }
    let mut a = lo+(hi-lo)*best.0.saturating_sub(1) as f32/24.0;
    let mut b = lo+(hi-lo)*(best.0+1).min(24) as f32/24.0;
    for _ in 0..24 {
        let u = a+(b-a)/3.0;
        let v = b-(b-a)/3.0;
        if sample(u).0<sample(v).0 { b=v; } else { a=u; }
    }
    let minimum = (a+b)*0.5;
    if sample(minimum).0>radius+0.00001 { return None; }
    let mut start = lo;
    let mut end = minimum;
    if sample(start).0<=radius { end=start; }
    else {
        for _ in 0..24 {
            let middle=(start+end)*0.5;
            if sample(middle).0<=radius { end=middle; } else { start=middle; }
        }
    }
    Some((end,sample(end).1))
}

pub fn contacts(
    spatial: SpatialQuery,
    mut commands: Commands,
    clocks: (Res<Time>, Res<Time<Fixed>>),
    scenario: Option<Res<Scenario>>,
    stamp: Res<RosFrameStamp>,
    hit_pub: Res<TopicPublisher<SimHitEventTopic>>,
    trace_pub: Res<TopicPublisher<SimProjectileTraceTopic>>,
    mut projectiles: Query<(Entity, &Position, &LinearVelocity, &Collider, &ShotTag, &mut ShotContact),
        (With<Projectile>, Without<DartProjectile>, Without<ShotResolved>)>,
    colliders: Query<(Entity,&Position,&Rotation,&Collider,&CollisionLayers,Option<&Armor>)>,
    roots: Query<&ArmorRoot>,
    geometry: (Query<&GlobalTransform>, HierarchyQuery),
    robot_ids: Query<&RobotId>,
    parents: Query<&ChildOf>,
    mut stats: ResMut<ProjectileStatistics>,
    mut previous_plates: Local<std::collections::HashMap<Entity,Vec3>>,
    settling: (Res<events::ShotSequence>, ResMut<crate::bench::settlement::Settlement>),
) {
    let (time,fixed)=clocks;
    let (sequence,mut settlement)=settling;
    let (globals,hierarchy) = geometry;
    let layers = GameLayer::projectile_collision_layers(true);
    let filter = SpatialQueryFilter::default();
    let mut points = Vec::new();
    for (projectile,pose,velocity,shape,shot,mut trace) in &mut projectiles {
        let previous = trace.prev;
        let travelled = pose.0-previous;
        let distance = travelled.length();
        // (fraction of this actual step, collider, actual contact, sphere centre)
        let mut first: Option<(f32,Entity,Vec3,Vec3)> = None;
        if let Ok(direction) = Dir3::new(travelled) {
            let config = ShapeCastConfig { max_distance:distance,target_distance:0.0,
                compute_contact_on_penetration:true,ignore_origin_penetration:false };
            // Static scenery. Armor is handled separately in its moving frame.
            let hit = spatial.cast_shape_predicate(shape,previous,Quat::IDENTITY,direction,
                &config,&filter,&|entity| colliders.get(entity).is_ok_and(|(_,_,_,_,other,armor)|
                    armor.is_none() && entity!=projectile && layers.interacts_with(*other)));
            if let Some(hit) = hit {
                if let Ok((_,p,r,collider,_,_)) = colliders.get(hit.entity) {
                    if let Some((u,point)) = refine_contact(collider,p.0,r.0,previous,travelled,
                        (hit.distance/distance).clamp(0.0,1.0),0.0085) {
                        first = Some((u,hit.entity,point,previous+travelled*u));
                    }
                }
            }
        }
        // Transform the sphere's ACTUAL segment into each plate's end-of-step
        // translating frame. No extension, predicted future or reverse ray.
        // Rotation is sampled each 120 Hz step; at 0.5 rad/s the remaining
        // orientation interpolation error is below 0.4 mm over a 135 mm plate.
        for (entity,position,rotation,collider,other,armor) in &colliders {
            if armor.is_none() || !layers.interacts_with(*other) { continue; }
            let movement = position.0-previous_plates.get(&entity).copied().unwrap_or(position.0);
            let relative = travelled-movement;
            let length = relative.length();
            let Ok(direction) = Dir3::new(relative) else { continue; };
            let config = ShapeCastConfig { max_distance:length,target_distance:0.0,
                compute_contact_on_penetration:true,ignore_origin_penetration:false };
            if let Some(hit) = spatial.cast_shape_predicate(shape,previous+movement,
                Quat::IDENTITY,direction,&config,&filter,&|candidate| candidate==entity) {
                if let Some((u,point)) = refine_contact(collider,position.0,rotation.0,previous+movement,relative,
                    (hit.distance/length).clamp(0.0,1.0),0.0085) {
                    if first.as_ref().is_none_or(|old| u<old.0) {
                        first = Some((u,entity,point-movement*(1.0-u),previous+travelled*u));
                    }
                }
            }
        }
        let actual = first.as_ref().map(|hit| hit.3).unwrap_or(pose.0);
        points.push(serde_json::json!({"shot_seq":shot.shot_seq,
            "previous":to_ros_vec3(previous).to_array(),
            "position":to_ros_vec3(actual).to_array(),
            "velocity":to_ros_vec3(velocity.0).to_array()}));
        trace.prev = actual;
        let Some((fraction,other,contact,centre)) = first else { continue; };
        let chain: Vec<_> = std::iter::once(other).chain(parents.iter_ancestors(other)).collect();
        let root = chain.iter().find_map(|e| roots.get(*e).ok());
        let target = chain.iter().find_map(|e| robot_ids.get(*e).ok()).map(|r| r.0);
        let armor = colliders.get(other).is_ok_and(|(_,_,_,_,_,a)| a.is_some());
        let sim_ns = scenario.as_deref().map(Scenario::sim_time_ns).unwrap_or(0);
        let elapsed = (time.elapsed_secs_f64()-shot.fire_elapsed_s
            - f64::from((1.0-fraction)*fixed.delta_secs())).max(0.0) as f32;
        if armor {
            let (Some(root),Some(target)) = (root,target) else { continue; };
            let mut event = events::hit_armor_event(&stamp,sim_ns,shot.shot_seq,
                shot.shooter_id,target,root.id.as_usize() as u32,
                to_ros_vec3(centre),Some(to_ros_vec3(contact)),to_ros_vec3(velocity.0),elapsed);
            // Read the model centre in collider-local coordinates, then use the
            // SAME physics pose and translation fraction as the accepted sweep.
            if let Ok((_,position,rotation,_,_,Some(armor_data))) = colliders.get(other) {
                let model_root = chain.iter().copied().find(|entity| roots.get(*entity).is_ok());
                if let (Some(root_entity), Ok(global)) = (model_root, globals.get(other)) {
                    let model_center = hierarchy.of(root_entity).suffix("CENTER").any().one().unwrap_or(root_entity);
                    if let (Ok(center_global), Ok(root_global)) = (globals.get(model_center), globals.get(root_entity)) {
                        let offset = global.affine().inverse().transform_point3(center_global.translation());
                        let scale = global.compute_transform().scale;
                        let movement = position.0-previous_plates.get(&other).copied().unwrap_or(position.0);
                        let plate_center = position.0+rotation.0*(offset*scale)-movement*(1.0-fraction);
                        // Model -Y is the normal, +X width, +Z height.
                        let pose_delta = rotation.0*global.rotation().inverse();
                        let right = to_ros_vec3(pose_delta*root_global.rotation()*Vec3::X).normalize();
                        let up = to_ros_vec3(pose_delta*root_global.rotation()*Vec3::Z).normalize();
                        let size = plate_size(armor_data.spec.armor_type());
                        event.impact_geometry_valid = true;
                        event.impact_sim_time_ns = sim_ns.saturating_sub(((1.0-fraction)*fixed.delta_secs()*1e9) as u64);
                        event.plate_center = to_ros_vec3(plate_center).to_array().map(f64::from).to_vec();
                        event.plate_right = right.to_array().map(f64::from).to_vec();
                        event.plate_up = up.to_array().map(f64::from).to_vec();
                        event.plate_size = vec![size.0,size.1];
                    }
                }
            }
            hit_pub.publish(event);
            stats.increase_accurate();
        } else {
            hit_pub.publish(events::hit_env_event(&stamp,sim_ns,shot.shot_seq,
                shot.shooter_id,to_ros_vec3(centre),to_ros_vec3(velocity.0),elapsed));
        }
        settlement.finish(&sequence,shot.shot_seq,"collision");
        commands.entity(projectile).insert(ShotResolved).despawn();
    }
    previous_plates.clear();
    previous_plates.extend(colliders.iter().filter(|(_,_,_,_,_,a)| a.is_some()).map(|(e,p,_,_,_,_)|(e,p.0)));
    if !points.is_empty() {
        trace_pub.publish(RosString { data: serde_json::json!({
            "stamp_ns":i64::from(stamp.0.sec)*1_000_000_000+i64::from(stamp.0.nanosec),
            "physics_time_s":fixed.elapsed_secs_f64(),"step_s":fixed.delta_secs(),
            "method":"swept_actual_segment_v3","projectiles":points}).to_string() });
    }
}
