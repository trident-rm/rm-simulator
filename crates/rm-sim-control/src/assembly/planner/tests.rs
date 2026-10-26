// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Planner behavior and recorded source diagnostics.
use super::*;
use sha2::{Digest, Sha256};
#[test]
fn coarse_runtime_keyframes_preserve_duration_and_reject_invalid_ticks() {
    let mut rig = super::super::tests::mechanism_fixture();
    rig.config.engineer_jaws = (0..2)
        .map(|i| Jaw {
            name: format!("jaw-{i}"),
            origin_m: [0.; 3],
            axis: [0., 0., 1.],
            open_rad: 0.3,
            closed_rad: 0.,
        })
        .collect();
    rig.jaw_angles = vec![0.3; 2];
    let mut frame = sample_frame(&rig, [0.09, 0.2, 0.6], [0.; 3], Stage::Approach, true);
    frame.ticks = 8;
    let plan = Plan {
        frames: vec![frame.clone()],
        statistics: Statistics::default(),
    };
    let samples = rig.runtime_limited_frames(&plan).unwrap();
    assert_eq!(samples.len(), 8);
    let mut remembered = rig.clone();
    remembered.remember_runtime_plan(&plan);
    assert_eq!(remembered.ik_hints.len(), 2);
    assert!(
        samples
            .iter()
            .all(|f| f.joints_rad == rig.engineer_q.map(|v| (v * 1e6).round() / 1e6))
    );
    for ticks in [0, 9] {
        let mut invalid = frame.clone();
        invalid.ticks = ticks;
        let mut copy = rig.clone();
        let before = copy.report();
        assert!(copy.follow_plan(&invalid).is_err());
        assert_eq!(copy.report(), before);
    }
    // The clock stays bounded by time represented by the keyframe.
    frame.rod_spin_rad = rig.rod_spin_rad + 0.02;
    assert!(rig.clone().follow_plan(&frame).is_ok());
    frame.ticks = 1;
    assert!(rig.clone().follow_plan(&frame).is_err());
}
#[test]
fn preloaded_payload_retiming_preserves_the_actual_grasp() {
    let mut rig = super::super::tests::mechanism_fixture();
    rig.config.engineer_jaws = (0..2)
        .map(|i| Jaw {
            name: format!("jaw-{i}"),
            origin_m: [0.; 3],
            axis: [0., 0., 1.],
            open_rad: 0.,
            closed_rad: 0.2,
        })
        .collect();
    let mut arm = ArmPose::neutral(&rig.runtime_model());
    arm.jaws_rad = [0.2; 2];
    rig.set_live_entry(&arm);
    let carried = rig.carried_start(&arm);
    assert_eq!(carried.engineer_q, arm.joints_rad);
    assert!(carried.gripped);
    let mut goal = arm.joints_rad;
    goal[0] += 0.7;
    let next = at(carried.clone(), goal);
    let frame = sample_frame(&next, [0.09, 0.2, 0.6], [0.; 3], Stage::Approach, true);
    let plan = Plan {
        frames: vec![frame],
        statistics: Statistics::default(),
    };
    let samples = carried.runtime_limited_frames(&plan).unwrap();
    assert!(samples.len() > 1);
    for f in samples {
        assert!(f.gripped);
        assert_eq!(f.guide_m, [0.09, 0.2, 0.6]);
        let expected = at(carried.clone(), f.joints_rad).tool_unit_pose();
        assert!(position(expected).distance(position(f.unit)) < 2e-6);
        assert!(quaternion(expected).angle_between(quaternion(f.unit).normalize()) < 2e-6);
    }
}
#[test]
#[ignore = "requires source CAD and a completed recording"]
fn audit_recorded_source_contacts() {
    let package = std::env::var("ASSEMBLY_CAD_PACKAGE").unwrap();
    let area = std::env::var("ASSEMBLY_CAD_AREA").unwrap();
    let input =
        std::fs::read_to_string(std::env::var("ASSEMBLY_CONTACT_RECORDING").unwrap()).unwrap();
    let mut rows = input.lines();
    let header: serde_json::Value = serde_json::from_str(rows.next().unwrap()).unwrap();
    let requested = serde_json::from_value(header["requested_pose"].clone()).unwrap();
    let mut rig = Rig::load_in_area(Path::new(&package), requested, Path::new(&area)).unwrap();
    for row in rows {
        let row: serde_json::Value = serde_json::from_str(row).unwrap();
        let cad = &row["cad"];
        rig.tool = serde_json::from_value(cad["tool"].clone()).unwrap();
        rig.unit_pose = serde_json::from_value(cad["unit_pose"].clone()).unwrap();
        rig.core_q = serde_json::from_value(cad["core_q_rad"].clone()).unwrap();
        rig.gripped = cad["gripped"].as_bool().unwrap();
        rig.grip_requested = cad["grip_requested"].as_bool().unwrap();
        rig.grasp_roll_rad = cad["grasp_roll_rad"].as_f64().unwrap();
        rig.grasp_axial_m = cad["grasp_axial_m"].as_f64().unwrap();
        rig.lift_m = row["guide_m"][2].as_f64().unwrap() - 0.6;
        rig.p_rad = row["angles_rad"][0].as_f64().unwrap();
        rig.q_rad = row["angles_rad"][1].as_f64().unwrap();
        let frames = serde_json::from_value(row["frames_world_flu"].clone()).unwrap();
        let hits = rig
            .collision
            .as_ref()
            .unwrap()
            .contacts(&frames, &rig.allowed_contacts(), 0.0005)
            .unwrap();
        assert!(hits.is_empty(), "tick {}: {hits:?}", row["tick"]);
    }
}
/// Isolate the dominant collision to original source leaves. Diagnostic only:
/// neither ownership nor collision exclusions are changed by this audit.
#[test]
#[ignore = "requires source CAD; isolates source-leaf collision pairs"]
fn audit_chassis_shoulder_collision_leaves() {
    let package = PathBuf::from(std::env::var("ASSEMBLY_CAD_PACKAGE").unwrap());
    let rig = if let Ok(area) = std::env::var("ASSEMBLY_CAD_AREA") {
        Rig::load_in_area(
            &package,
            TargetPose {
                position_m: [-0.05, 0.2, 0.5],
                theta_rad: 0.,
                phi_rad: std::f64::consts::FRAC_PI_2,
                alpha_rad: 0.,
            },
            Path::new(&area),
        )
        .unwrap()
    } else {
        Rig::load(&package).unwrap()
    };
    let asset: rm_sim_physics::manipulator_collision::CollisionAsset =
        serde_json::from_slice(&std::fs::read(package.join("collision.json")).unwrap()).unwrap();
    let pair = std::env::var("ASSEMBLY_COLLISION_PAIR")
        .unwrap_or_else(|_| "engineer/body,engineer/j2".into());
    let (first, second) = pair
        .split_once(',')
        .expect("pair must name two frames separated by comma");
    let body = asset.bodies.iter().find(|b| b.frame == first).unwrap();
    let shoulder = asset.bodies.iter().find(|b| b.frame == second).unwrap();
    let mut cases = vec![
        ("source-zero".to_string(), [0.; 6]),
        ("loaded".into(), rig.engineer_q),
    ];
    cases.extend(
        ik_candidates(
            &rig.config.engineer,
            compose(
                inverse(rig.engineer_body),
                rig.target([0.09, 0.2, 0.6], [0.; 3]),
            ),
            branch_seeds(&rig.config.engineer),
            12,
        )
        .into_iter()
        .enumerate()
        .map(|(i, q)| (format!("initial-branch-{i}"), q)),
    );
    if let Ok(path) = std::env::var("ASSEMBLY_COLLISION_REPLAY") {
        let old: serde_json::Value = serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        for case in old["cases"].as_array().unwrap() {
            if case["label"]
                .as_str()
                .unwrap()
                .starts_with("initial-branch-")
            {
                cases.push((
                    format!("replay-{}", case["label"].as_str().unwrap()),
                    serde_json::from_value(case["q_rad"].clone()).unwrap(),
                ));
            }
        }
    }
    let mut cases: Vec<_> = cases
        .into_iter()
        .map(|(label, q)| {
            let candidate = at(rig.clone(), q);
            (label, q, candidate.world_frames())
        })
        .collect();
    if let Ok(path) = std::env::var("ASSEMBLY_COLLISION_PLANNER") {
        let old: serde_json::Value = serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        let witnesses = &old["planner"]["failure"]["statistics"]["rejected_milestones"];
        for (key, witness) in witnesses.as_object().unwrap() {
            let hit = &witness["hit"];
            if (hit["first"] == first && hit["second"] == second)
                || (hit["first"] == second && hit["second"] == first)
            {
                cases.push((
                    format!("planner-{key}"),
                    serde_json::from_value(witness["q_rad"].clone()).unwrap(),
                    serde_json::from_value(witness["frames"].clone()).unwrap(),
                ));
            }
        }
    }
    if std::env::var_os("ASSEMBLY_COLLISION_DEPLOYED").is_some() {
        let deployed: Vec<_> = cases
            .iter()
            .map(|(label, q, frames)| {
                let mut frames = frames.clone();
                let forearm = compose(
                    frames["engineer/j3"],
                    inverse(translation(rig.config.engineer.axes[2].origin_m)),
                );
                for aux in rig
                    .config
                    .engineer_auxiliary
                    .iter()
                    .filter(|a| a.name == "camera-lift")
                {
                    frames.insert(
                        format!("engineer/{}", aux.name),
                        compose(forearm, aux.reference),
                    );
                }
                (format!("{label}-aux-deployed"), *q, frames)
            })
            .collect();
        cases.extend(deployed);
    }
    let mut report = vec![];
    for (label, q, frames) in cases {
        let bounds = |part: &rm_sim_physics::manipulator_collision::CollisionPart, frame: Pose| {
            let points_m = match part {
                rm_sim_physics::manipulator_collision::CollisionPart::Mesh { points_m, .. }
                | rm_sim_physics::manipulator_collision::CollisionPart::Convex { points_m } => {
                    points_m
                }
                _ => panic!("source audit requires mesh or convex source leaves"),
            };
            points_m
                .iter()
                .map(|p| position(frame) + quaternion(frame) * glam::DVec3::from_array(*p))
                .fold(
                    (
                        glam::DVec3::splat(f64::INFINITY),
                        glam::DVec3::splat(f64::NEG_INFINITY),
                    ),
                    |(lo, hi), p| (lo.min(p), hi.max(p)),
                )
        };
        let aa: Vec<_> = body
            .parts
            .iter()
            .map(|p| bounds(p, frames[&body.frame]))
            .collect();
        let bb: Vec<_> = shoulder
            .parts
            .iter()
            .map(|p| bounds(p, frames[&shoulder.frame]))
            .collect();
        let mut pairs = vec![];
        let mut broad = 0;
        for (i, (body_low, body_high)) in aa.iter().enumerate() {
            for (j, (arm_low, arm_high)) in bb.iter().enumerate() {
                if !body_low.cmple(*arm_high).all() || !arm_low.cmple(*body_high).all() {
                    continue;
                }
                broad += 1;
                let scene =
                    CollisionScene::new(rm_sim_physics::manipulator_collision::CollisionAsset {
                        schema: 1,
                        excluded_pairs: vec![],
                        bodies: vec![
                            rm_sim_physics::manipulator_collision::CollisionBody {
                                frame: body.frame.clone(),
                                parts: vec![body.parts[i].clone()],
                            },
                            rm_sim_physics::manipulator_collision::CollisionBody {
                                frame: shoulder.frame.clone(),
                                parts: vec![shoulder.parts[j].clone()],
                            },
                        ],
                    })
                    .unwrap();
                let hits = scene.contacts(&frames, &[], 0.0005).unwrap();
                if !hits.is_empty() {
                    pairs.push(serde_json::json!({"body_part":i,"j2_part":j,"first_frame":first,"second_frame":second,"overlap_bounds_m":[body_low.max(*arm_low).to_array(),body_high.min(*arm_high).to_array()],"hits":hits}));
                }
            }
        }
        report.push(serde_json::json!({"label":label,"q_rad":q,"frames":frames,"broad_pairs":broad,"intersecting_leaves":pairs}));
    }
    let out = std::env::var("ASSEMBLY_COLLISION_AUDIT").unwrap();
    std::fs::write(out,serde_json::to_vec_pretty(&serde_json::json!({"schema":1,"first_frame":first,"second_frame":second,"manifest_sha256":format!("{:x}",Sha256::digest(std::fs::read(package.join("manifest.json")).unwrap())),"collision_sha256":format!("{:x}",Sha256::digest(std::fs::read(package.join("collision.json")).unwrap())),"cases":report})).unwrap()).unwrap();
}
#[test]
#[ignore = "requires generated CAD; reverse IK diagnostic"]
fn audit_level_three_reverse_ik() {
    let package = std::env::var("ASSEMBLY_CAD_PACKAGE")
        .unwrap_or_else(|_| "../../local-assets/assembly-cad/package".into());
    let rig = Rig::load(Path::new(&package)).unwrap();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut witness = None;
    let mut first_kinematic = None;
    let offsets = if std::env::var_os("ASSEMBLY_AUDIT_NEAR").is_some() {
        vec![-0.4, -0.5, -0.6]
    } else {
        vec![-0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3]
    };
    'setups: for dx in offsets {
        for dy in [0., -0.2, 0.2] {
            for yaw in (0..24).map(|i| i as f64 * std::f64::consts::TAU / 24.) {
                for (roll, axial) in (0..8).flat_map(|i| {
                    [0., 0.02].map(move |a| (i as f64 * std::f64::consts::TAU / 8., a))
                }) {
                    let mut trial = rig.clone();
                    trial.engineer_body.translation_m[0] += dx;
                    trial.engineer_body.translation_m[1] += dy;
                    trial.engineer_body = pose(
                        position(trial.engineer_body),
                        quaternion(rig.engineer_body) * DQuat::from_rotation_z(yaw),
                    );
                    trial.grasp_roll_rad = roll;
                    trial.grasp_axial_m = axial;
                    let milestones =
                        goals(3, std::f64::consts::FRAC_PI_6, [0.09, 0.2, 0.6], [0.; 3]);
                    let (_, end, angles) = *milestones.last().unwrap();
                    let straight_flip = ik_candidates(
                        &trial.config.engineer,
                        compose(
                            inverse(trial.engineer_body),
                            trial.target(end, [std::f64::consts::FRAC_PI_2, 0., 0.]),
                        ),
                        branch_seeds(&trial.config.engineer),
                        1,
                    );
                    *counts
                        .entry("straight_flip_reachable_setups".into())
                        .or_default() += usize::from(!straight_flip.is_empty());
                    if counts.get("setups").copied().unwrap_or(0) == 0 {
                        println!(
                            "TASK_AUDIT {}",
                            serde_json::json!({"body":trial.engineer_body,"socket":trial.socket,
                                "lift":trial.target(end,[0.;3]),"flip":trial.target(end,[std::f64::consts::FRAC_PI_2,0.,0.]),
                                "align":trial.target(end,angles),"axes":trial.config.engineer.axes})
                        );
                    }
                    let candidates = ik_candidates(
                        &trial.config.engineer,
                        compose(inverse(trial.engineer_body), trial.target(end, angles)),
                        branch_seeds(&trial.config.engineer),
                        8,
                    );
                    *counts.entry("setups".into()).or_default() += 1;
                    for q in candidates {
                        *counts.entry("end_solutions".into()).or_default() += 1;
                        let mut probe = at(trial.clone(), q);
                        let mut previous = (end, angles);
                        let mut viable = true;
                        for (stage, p, a) in milestones.iter().rev().copied().chain(
                            std::iter::once((Stage::Approach, [0.09, 0.2, 0.6], [0.; 3])),
                        ) {
                            let count = (0..3)
                                .map(|i| (p[i] - previous.0[i]).abs() / 0.005)
                                .chain((0..3).map(|i| (a[i] - previous.1[i]).abs() / 0.02))
                                .fold(0., f64::max)
                                .ceil()
                                .max(1.) as usize;
                            for i in 1..=count {
                                let t = i as f64 / count as f64;
                                let x = std::array::from_fn(|j| {
                                    previous.0[j] + (p[j] - previous.0[j]) * t
                                });
                                let r = std::array::from_fn(|j| {
                                    previous.1[j] + (a[j] - previous.1[j]) * t
                                });
                                if probe.solve_tool(x, r).is_err() {
                                    *counts.entry(format!("reverse {stage:?} IK")).or_default() +=
                                        1;
                                    viable = false;
                                    break;
                                }
                            }
                            if !viable {
                                break;
                            }
                            previous = (p, a);
                        }
                        if viable {
                            *counts.entry("continuous_ik".into()).or_default() += 1;
                            let setup = serde_json::json!({"dx":dx,"dy":dy,"yaw":yaw,"roll":roll,"axial":axial,"root_q":probe.engineer_q});
                            first_kinematic.get_or_insert_with(|| setup.clone());
                            if counts.get("guarded_paths").copied().unwrap_or(0) >= 32 {
                                break 'setups;
                            }
                            *counts.entry("guarded_paths".into()).or_default() += 1;
                            let mut root = at(trial.clone(), probe.engineer_q);
                            root.initial_engineer_q = root.engineer_q;
                            root.unit_pose = root.tool_unit_pose();
                            let mut search = Search {
                                stats: Statistics::default(),
                                budget: 8000,
                                ik_table: IkTable::default(),
                                transitions: TransitionTable::default(),
                                last: None,
                                last_checked: None,
                                stage: Stage::Approach,
                                signal: None,
                            };
                            match build(
                                &rig,
                                &root,
                                3,
                                std::f64::consts::FRAC_PI_6,
                                [0.09, 0.2, 0.6],
                                [0.; 3],
                                true,
                                &mut search,
                                ChassisRoute::default(),
                            ) {
                                Ok((_, frames)) => {
                                    witness = Some(
                                        serde_json::json!({"setup":setup,"frames":frames.len()}),
                                    );
                                    break 'setups;
                                }
                                Err(e) => {
                                    *counts
                                        .entry(format!(
                                            "guarded {e}: {:?}",
                                            search.last.as_ref().map(|h| (&h.first, &h.second))
                                        ))
                                        .or_default() += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    println!(
        "REVERSE_AUDIT {}",
        serde_json::json!({"counts":counts,"first_kinematic_witness":first_kinematic,"guarded_witness":witness})
    );
}
/// Explicit source-CAD calibration experiment; never changes runtime guards.
#[test]
#[ignore = "requires generated CAD; bounded searches can take minutes"]
fn audit_level_three_joint_guards() {
    let package = std::env::var("ASSEMBLY_CAD_PACKAGE")
        .unwrap_or_else(|_| "../../local-assets/assembly-cad/package".into());
    let mode = std::env::var("ASSEMBLY_AUDIT_LIMITS").unwrap_or_else(|_| "original".into());
    assert!(["original", "j4", "winding", "all"].contains(&mode.as_str()));
    let mut rig = Rig::load(Path::new(&package)).unwrap();
    for (i, axis) in rig.config.engineer.axes.iter_mut().enumerate() {
        if i == 3 && ["j4", "all"].contains(&mode.as_str()) {
            axis.range_rad = [-std::f64::consts::PI, std::f64::consts::PI];
        } else if i != 3 && ["winding", "all"].contains(&mode.as_str()) {
            axis.range_rad = [-2. * std::f64::consts::PI, 2. * std::f64::consts::PI];
        }
    }
    let result = match rig.plan_assembly(
        3,
        std::f64::consts::FRAC_PI_6,
        [0.09, 0.2, 0.6],
        [0.; 3],
        true,
        8000,
    ) {
        Ok((mut start, plan)) => {
            let mut ranges = [[f64::INFINITY, f64::NEG_INFINITY]; 6];
            for f in &plan.frames {
                start.follow_plan(f).unwrap();
                for (i, range) in ranges.iter_mut().enumerate() {
                    range[0] = range[0].min(f.q[i]);
                    range[1] = range[1].max(f.q[i]);
                }
            }
            serde_json::json!({"status":"ready","statistics":plan.statistics,
                    "joint_ranges_rad":ranges,"final":start.report()})
        }
        Err(failure) => serde_json::json!({"status":"failed","failure":failure}),
    };
    println!(
        "GUARD_AUDIT {}",
        serde_json::json!({"mode":mode,"result":result})
    );
}
#[test]
fn segment_boundaries_join_exactly_and_subdivision_preserves_the_guide() {
    let start = ([0.09, 0.2, 0.6], [0.; 3]);
    let task = segments(3, 0.5, start.0, start.1);
    assert_eq!(task.first().unwrap().from, start);
    for pair in task.windows(2) {
        assert_eq!(pair[0].to, pair[1].from);
    }
    assert_eq!(
        task.last().unwrap().to,
        ([-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0.5, 0.])
    );
    for segment in task {
        assert_eq!(segment.point(0.), segment.from);
        assert_eq!(segment.point(1.), segment.to);
        let n = segment.samples(0.005, 0.02);
        let mut previous = segment.from;
        for i in 1..=n {
            let point = segment.point(i as f64 / n as f64);
            for j in 0..3 {
                assert!((point.0[j] - previous.0[j]).abs() <= 0.005 + 1e-12);
                assert!((point.1[j] - previous.1[j]).abs() <= 0.02 + 1e-12);
            }
            previous = point;
        }
    }
}
/// Controlled IK comparison from exactly the same successful grasp and parking.
#[test]
#[ignore = "requires generated CAD and a completed twisting level-three recording"]
fn audit_twist_counterfactual() {
    let package = std::env::var("ASSEMBLY_CAD_PACKAGE").unwrap();
    let area = std::env::var("ASSEMBLY_CAD_AREA").unwrap();
    let motion =
        std::fs::read_to_string(std::env::var("ASSEMBLY_SPIN_REFERENCE").unwrap()).unwrap();
    let records: Vec<serde_json::Value> = motion
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(
        records[0]["manifest"],
        serde_json::from_slice::<serde_json::Value>(
            &std::fs::read(Path::new(&package).join("manifest.json")).unwrap()
        )
        .unwrap()
    );
    assert_eq!(records.last().unwrap()["completed"], true);
    let cad = &records[1]["cad"];
    let requested = serde_json::from_value(cad["requested_pose"].clone()).unwrap();
    let mut rig = Rig::load_in_area(Path::new(&package), requested, Path::new(&area)).unwrap();
    assert_eq!(rig.report()["area"], cad["area"]);
    rig.engineer_body = serde_json::from_value(cad["engineer_body"].clone()).unwrap();
    rig.engineer_q = serde_json::from_value(cad["engineer_q_rad"].clone()).unwrap();
    rig.camera_deployed = cad["camera_deployed"].as_bool().unwrap();
    rig.grasp_roll_rad = cad["grasp_roll_rad"].as_f64().unwrap();
    rig.grasp_axial_m = cad["grasp_axial_m"].as_f64().unwrap();
    let mut results = vec![];
    for twist in [false, true] {
        let mut probe = rig.clone();
        let route = ChassisRoute {
            flip_spin_rad: if twist {
                std::f64::consts::FRAC_PI_2
            } else {
                0.
            },
            ..Default::default()
        };
        let mut failure = None;
        'segments: for segment in
            segments(3, std::f64::consts::FRAC_PI_6, [0.09, 0.2, 0.6], [0.; 3])
        {
            let n = route.samples(segment, 0.005, 0.02);
            for i in 1..=n {
                let (p, a) = segment.point(i as f64 / n as f64);
                if let Err(reason) = route_step(&mut probe, rig.engineer_body, route, p, a) {
                    failure = Some(
                        serde_json::json!({"stage":segment.stage,"angles_rad":a,"reason":reason}),
                    );
                    break 'segments;
                }
            }
        }
        results.push(
            serde_json::json!({"twist":twist,"complete_ik":failure.is_none(),"failure":failure}),
        );
    }
    assert_eq!(results[1]["complete_ik"], true);
    let path = std::env::var("ASSEMBLY_SPIN_AUDIT").unwrap();
    std::fs::write(path,serde_json::to_vec_pretty(&serde_json::json!({"purpose":"same initial configuration; IK-only counterfactual; full swept success is in the recording","cases":results})).unwrap()).unwrap();
}
/// Reproduce the completed real-area lift, then compare locked and free rod clocking.
#[test]
#[ignore = "requires generated CAD and a completed level-two area recording"]
fn audit_rod_spin_reachability() {
    let package = std::env::var("ASSEMBLY_CAD_PACKAGE").unwrap();
    let area = std::env::var("ASSEMBLY_CAD_AREA").unwrap();
    let input = std::env::var("ASSEMBLY_SPIN_REFERENCE").unwrap();
    let reference: serde_json::Value =
        serde_json::from_slice(&std::fs::read(input).unwrap()).unwrap();
    assert_eq!(reference["planner"]["status"], "ready");
    assert_eq!(reference["cores"][0]["step"], "Confirmed");
    assert_eq!(reference["difficulty"], 2);
    let cad = &reference["cad"];
    let requested = serde_json::from_value(cad["requested_pose"].clone()).unwrap();
    let mut rig = Rig::load_in_area(Path::new(&package), requested, Path::new(&area)).unwrap();
    assert_eq!(
        rig.report()["area"],
        cad["area"],
        "reference terrain/configuration must match"
    );
    rig.engineer_body = serde_json::from_value(cad["engineer_body"].clone()).unwrap();
    rig.engineer_q = serde_json::from_value(cad["engineer_q_rad"].clone()).unwrap();
    rig.camera_deployed = cad["camera_deployed"].as_bool().unwrap();
    rig.grasp_roll_rad = cad["grasp_roll_rad"].as_f64().unwrap();
    rig.grasp_axial_m = cad["grasp_axial_m"].as_f64().unwrap();
    let mut seeds = vec![rig.engineer_q];
    seeds.extend(branch_seeds(&rig.config.engineer));
    let mut rows = vec![];
    for p_degrees in [0., 30., 60., 90.] {
        for q_degrees in if p_degrees == 90. {
            vec![0., 30.]
        } else {
            vec![0.]
        } {
            let mut reachable = vec![];
            let mut clear = vec![];
            let mut rejected = BTreeMap::<String, usize>::new();
            for spin_degrees in (-8..8).map(|i| i as f64 * 22.5) {
                rig.rod_spin_rad = spin_degrees.to_radians();
                let logical = [-0.05, 0.2, 0.7];
                let angles = [
                    p_degrees * std::f64::consts::PI / 180.,
                    q_degrees * std::f64::consts::PI / 180.,
                    0.,
                ];
                let solutions = ik_candidates(
                    &rig.config.engineer,
                    compose(inverse(rig.engineer_body), rig.target(logical, angles)),
                    seeds.clone(),
                    8,
                );
                if !solutions.is_empty() {
                    reachable.push(spin_degrees);
                }
                let mut has_clear = false;
                for q in solutions {
                    let mut probe = at(rig.clone(), q);
                    probe.gripped = true;
                    probe.grip_requested = true;
                    probe.lift_m = 0.1;
                    probe.p_rad = angles[0];
                    probe.q_rad = angles[1];
                    probe.unit_pose = probe.tool_unit_pose();
                    match probe.collision_contacts().unwrap().first() {
                        None => has_clear = true,
                        Some(hit) => {
                            *rejected
                                .entry(format!("{} / {}", hit.first, hit.second))
                                .or_default() += 1
                        }
                    }
                }
                if has_clear {
                    clear.push(spin_degrees);
                }
            }
            rows.push(serde_json::json!({"p_degrees":p_degrees,"q_degrees":q_degrees,
                    "reachable_spin_degrees":reachable,"clear_spin_degrees":clear,"rejected_pairs":rejected}));
        }
    }
    let body = rig.engineer_body;
    let mut placements = vec![];
    for dx in [-0.3, -0.15, 0., 0.15] {
        for dy in [-0.15, 0., 0.15] {
            for yaw_degrees in [-30., 0., 30.] {
                rig.engineer_body = pose(
                    position(body) + glam::DVec3::new(dx, dy, 0.),
                    quaternion(body)
                        * DQuat::from_rotation_z(yaw_degrees * std::f64::consts::PI / 180.),
                );
                if !rig.support_parking() {
                    continue;
                }
                let mut reachable = vec![];
                let mut clear = vec![];
                let mut witness = None;
                let mut rejected = BTreeMap::<String, usize>::new();
                for spin_degrees in [0., -45., 45., -90., 90., -135., 135., 180.] {
                    rig.rod_spin_rad = spin_degrees * std::f64::consts::PI / 180.;
                    let angles = [std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_6, 0.];
                    let solutions = ik_candidates(
                        &rig.config.engineer,
                        compose(
                            inverse(rig.engineer_body),
                            rig.target([-0.05, 0.2, 0.7], angles),
                        ),
                        seeds.clone(),
                        4,
                    );
                    if !solutions.is_empty() {
                        reachable.push(spin_degrees);
                    }
                    for q in solutions {
                        let mut probe = at(rig.clone(), q);
                        probe.gripped = true;
                        probe.grip_requested = true;
                        probe.lift_m = 0.1;
                        probe.p_rad = angles[0];
                        probe.q_rad = angles[1];
                        probe.unit_pose = probe.tool_unit_pose();
                        let hits = probe.collision_contacts().unwrap();
                        for hit in &hits {
                            *rejected
                                .entry(format!("{} / {}", hit.first, hit.second))
                                .or_default() += 1;
                        }
                        if hits.is_empty() {
                            clear.push(spin_degrees);
                            witness.get_or_insert_with(|| Witness::at(&probe));
                            break;
                        }
                    }
                }
                placements.push(serde_json::json!({"dx_m":dx,"dy_m":dy,"yaw_delta_degrees":yaw_degrees,
                        "reachable_spin_degrees":reachable,"clear_spin_degrees":clear,"witness":witness,"rejected_pairs":rejected}));
            }
        }
    }
    let output = serde_json::json!({"purpose":"static milestone comparison, not a connected executable plan", "reference":cad, "rows":rows,"placements_at_p90_q30":placements});
    let path = std::env::var("ASSEMBLY_SPIN_AUDIT").unwrap();
    std::fs::write(path, serde_json::to_vec_pretty(&output).unwrap()).unwrap();
}
#[test]
fn axial_spin_preserves_rod_position_axis_and_rigid_grasp_through_flip() {
    let mut rig = super::super::tests::mechanism_fixture();
    rig.grasp_axial_m = 0.02;
    for angles in [
        [0.; 3],
        [0.7, 0., 0.],
        [std::f64::consts::FRAC_PI_2, 0.5, 0.],
    ] {
        let logical = [-0.05, 0.2, 0.7];
        rig.rod_spin_rad = 0.;
        rig.tool = rig.target(logical, angles);
        let zero = rig.tool_unit_pose();
        let grip = compose(inverse(rig.tool), zero);
        let axis = quaternion(zero) * glam::DVec3::Z;
        for spin in [-3., -1.5, 0.4, 2.8] {
            rig.rod_spin_rad = spin;
            rig.tool = rig.target(logical, angles);
            let unit = rig.tool_unit_pose();
            assert!(position(unit).distance(position(zero)) < 1e-12);
            assert!((quaternion(unit) * glam::DVec3::Z - axis).length() < 1e-12);
            assert!(quaternion(unit).angle_between(quaternion(zero)) > 0.3);
            let actual_grip = compose(inverse(rig.tool), unit);
            assert!(position(actual_grip).distance(position(grip)) < 1e-12);
            assert!(quaternion(actual_grip).angle_between(quaternion(grip)) < 1e-7);
        }
    }
}
#[test]
fn rod_spin_route_is_continuous_and_replay_enforces_its_rate() {
    let route = ChassisRoute {
        lift_spin_rad: 1.2,
        flip_spin_rad: -2.,
        ..Default::default()
    };
    let at_insert = route.spin([-0.05, 0.2, 0.6], [0.; 3]);
    assert_eq!(at_insert, 0.);
    let at_lift = route.spin([-0.05, 0.2, 0.7], [0.; 3]);
    assert!((at_lift - 1.2).abs() < 1e-12);
    let at_flip = route.spin([-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0., 0.]);
    assert!((at_flip + 0.8).abs() < 1e-12);
    assert_eq!(
        at_flip,
        route.spin([-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0.5, 0.])
    );
    let mut rig = super::super::tests::mechanism_fixture();
    let previous = rig.report();
    let mut frame = sample_frame(&rig, [-0.05, 0.2, 0.7], [0.; 3], Stage::Lift, true);
    for spin in [0.1, f64::NAN] {
        frame.rod_spin_rad = spin;
        assert!(rig.follow_plan(&frame).is_err());
        assert_eq!(rig.report(), previous);
    }
    rig.rod_spin_rad = 0.4;
    rig.restart();
    assert_eq!(rig.rod_spin_rad, 0.);
}
#[test]
fn axial_freedom_covers_every_moving_stage_and_negative_q() {
    let route = ChassisRoute {
        approach_spin_rad: 0.1,
        insert_spin_rad: 0.2,
        lift_spin_rad: 0.3,
        flip_spin_rad: 0.4,
        align_spin_rad: 0.5,
        alignment_target_rad: -0.6,
        ..Default::default()
    };
    let task = segments(3, -0.6, [0.09, 0.2, 0.6], [0.; 3]);
    let mut previous = 0.;
    for (segment, delta) in task.into_iter().zip([0.1, 0.2, 0.3, 0.4, 0.5]) {
        assert!((route.spin(segment.from.0, segment.from.1) - previous).abs() < 1e-12);
        let spin = route.spin(segment.to.0, segment.to.1);
        assert!((spin - previous - delta).abs() < 1e-12);
        let dt = rm_sim_physics::tick_ns() as f64 * 1e-9;
        let n = route.samples(segment, super::super::SPEED * dt, super::super::RATE * dt);
        let mut last = previous;
        for i in 1..=n {
            let (p, a) = segment.point(i as f64 / n as f64);
            let v = route.spin(p, a);
            assert!((v - last).abs() <= super::super::RATE * dt + 1e-9);
            last = v;
        }
        previous = spin;
    }
    assert!((previous - 1.5).abs() < 1e-12);
}
#[test]
fn resumed_axial_profiles_preserve_current_clocking_at_every_stage() {
    let route = ChassisRoute {
        approach_spin_rad: 0.2,
        insert_spin_rad: -0.3,
        lift_spin_rad: 0.4,
        flip_spin_rad: 0.5,
        align_spin_rad: -0.6,
        ..Default::default()
    };
    for segment in segments(3, -0.6, [0.09, 0.2, 0.6], [0.; 3]) {
        let (p, a) = segment.point(0.4);
        let anchored = route.anchored(0.75, p, a, -0.6);
        assert!((anchored.spin(p, a) - 0.75).abs() < 1e-12);
    }
}
#[test]
fn nearby_ik_table_reuses_seeds_but_resolves_the_new_pose_and_model() {
    let mut arm = super::super::tests::mechanism_fixture().config.engineer;
    arm.tool = Pose::at([0.2, 0., 0.]);
    let q = [0.05; 6];
    let old = arm.tool_pose(q);
    let target = arm.tool_pose([0.052; 6]);
    let mut table = IkTable::default();
    table.insert(&arm, old, vec![q]);
    let (warm, exact) = table.seeds(&arm, target);
    assert!(!exact);
    assert_eq!(warm, vec![q]);
    let solved = arm.solve(target, warm[0]).unwrap();
    assert_ne!(solved, q);
    assert!(position(arm.tool_pose(solved)).distance(position(target)) < 1e-5);
    assert!(quaternion(arm.tool_pose(solved)).angle_between(quaternion(target)) < 1e-4);
    assert!(table.seeds(&arm, old).1);
    arm.axes[0].range_rad = [-1., 1.];
    assert!(table.seeds(&arm, target).0.is_empty());
}
#[test]
fn exact_ik_cache_keeps_failures_but_separates_targets_seeds_and_models() {
    let arm = super::super::tests::mechanism_fixture().config.engineer;
    let mut search = Search {
        stats: Statistics::default(),
        budget: 8,
        ik_table: IkTable::default(),
        transitions: TransitionTable::default(),
        last: None,
        last_checked: None,
        stage: Stage::Approach,
        signal: None,
    };
    let target = Pose::at([100., 0., 0.]);
    assert!(search.ik(&arm, target, [[0.; 6]], 1).is_empty());
    assert!(search.ik(&arm, target, [[0.; 6]], 1).is_empty());
    assert_eq!(search.stats.ik_cache_hits, 1);
    let mut changed = arm.clone();
    changed.axes[0].range_rad = [-1., 1.];
    search.ik(&changed, target, [[0.; 6]], 1);
    search.ik(&arm, Pose::at([101., 0., 0.]), [[0.; 6]], 1);
    search.ik(&arm, target, [[0.1; 6]], 1);
    assert_eq!(search.stats.ik_cache_hits, 1);
    let q = [0.; 6];
    let target = arm.tool_pose(q);
    assert_eq!(search.ik(&arm, target, [q], 1), vec![q]);
    assert_eq!(search.ik(&arm, target, [q], 1), vec![q]);
    assert!(search.stats.ik_cache_hits >= 2);
}
#[test]
fn exact_transition_cache_rejects_mutated_state_configuration_and_geometry() {
    let rig = super::super::tests::mechanism_fixture();
    let mut search = Search {
        stats: Statistics::default(),
        budget: 8,
        ik_table: IkTable::default(),
        transitions: TransitionTable::default(),
        last: None,
        last_checked: None,
        stage: Stage::Approach,
        signal: None,
    };
    let checked = search.check(&rig, rig.clone()).unwrap();
    search.check(&rig, rig.clone()).unwrap();
    assert_eq!(search.stats.collision_table_hits, 1);
    let frame = search.frame(&checked, [0.09, 0.2, 0.6], [0.; 3], Stage::Approach, true);
    let mut replay = rig.clone();
    replay.follow_plan(&frame).unwrap();
    assert_eq!(replay.cached_guard_replays, 1);
    let mut changed = frame.clone();
    changed.q[0] = 0.001;
    let mut replay = rig.clone();
    replay.follow_plan(&changed).unwrap();
    assert_eq!(replay.cached_guard_replays, 0);
    let mut changed = rig.clone();
    changed.config.engineer.axes[0].origin_m[0] += 0.001;
    assert!(
        !frame
            .validation
            .as_ref()
            .unwrap()
            .matches(&changed, &checked)
    );
    changed = rig.clone();
    changed.core_q[0] = 0.001;
    assert!(
        !frame
            .validation
            .as_ref()
            .unwrap()
            .matches(&changed, &checked)
    );
    changed = rig.clone();
    changed.grasp_roll_rad = 0.1;
    assert!(
        !frame
            .validation
            .as_ref()
            .unwrap()
            .matches(&changed, &checked)
    );
    use rm_sim_physics::manipulator_collision::{CollisionAsset, CollisionBody, CollisionPart};
    let mut obstructed = rig.clone();
    obstructed.collision = Some(std::sync::Arc::new(
        CollisionScene::new(CollisionAsset {
            schema: 1,
            excluded_pairs: vec![],
            bodies: vec![
                CollisionBody {
                    frame: "unit/unit".into(),
                    parts: vec![CollisionPart::Box { half_m: [0.01; 3] }],
                },
                CollisionBody {
                    frame: "floor".into(),
                    parts: vec![CollisionPart::Box { half_m: [0.1; 3] }],
                },
            ],
        })
        .unwrap(),
    ));
    assert!(obstructed.follow_plan(&frame).is_err());
    assert_eq!(obstructed.cached_guard_replays, 0);
    assert!(obstructed.last_collision.is_some());
}
#[test]
fn fallback_reserves_room_for_playback_validation_in_small_sweeps() {
    assert_eq!(linear_search_budget(8000), 6400);
    assert_eq!(linear_search_budget(2500), 1300);
    assert_eq!(linear_search_budget(4000), 2800);
    assert!(linear_search_budget(2500) + 808 < 2500);
}
#[test]
fn nonlinear_routes_preserve_linear_setup_order() {
    let mut old: Vec<_> = (0..100)
        .flat_map(|i| (0..18).map(move |j| (i, j)))
        .collect();
    old.sort_by_key(|(i, j)| route_order(*i, *j, 18, 18));
    let mut expanded: Vec<_> = (0..100)
        .flat_map(|i| (0..26).map(move |j| (i, j)))
        .collect();
    expanded.sort_by_key(|(i, j)| route_order(*i, *j, 18, 26));
    assert_eq!(&expanded[..old.len()], old.as_slice());
    assert!(expanded[old.len()..].iter().all(|(_, j)| *j >= 18));
}
#[test]
fn twist_then_unwind_sampling_respects_rate_despite_zero_net_spin() {
    let route = ChassisRoute {
        flip_mid_spin_rad: Some(std::f64::consts::FRAC_PI_2),
        ..Default::default()
    };
    let segment = Segment {
        stage: Stage::Flip,
        from: ([-0.05, 0.2, 0.7], [0.; 3]),
        to: ([-0.05, 0.2, 0.7], [std::f64::consts::FRAC_PI_2, 0., 0.]),
    };
    let dt = rm_sim_physics::tick_ns() as f64 * 1e-9;
    let n = route.samples(segment, super::super::SPEED * dt, super::super::RATE * dt);
    let mut previous = 0.;
    let mut maximum: f64 = 0.;
    for i in 1..=n {
        let (p, a) = segment.point(i as f64 / n as f64);
        let spin = route.spin(p, a);
        assert!((spin - previous).abs() <= super::super::RATE * dt + 1e-9);
        maximum = maximum.max(spin);
        previous = spin;
    }
    assert!(maximum > 1.56);
    assert_eq!(previous, 0.);
}
#[test]
fn seated_spin_playback_moves_the_unit_and_gripper_together() {
    let mut rig = super::super::tests::mechanism_fixture();
    let logical = [-0.05, 0.2, 0.7];
    rig.grasp_axial_m = 0.02;
    rig.tool = rig.target(logical, [0.; 3]);
    rig.unit_pose = rig.tool_unit_pose();
    rig.config.engineer.tool = rig.tool;
    let unit = rig.unit_pose;
    let normal = quaternion(unit) * glam::DVec3::Z;
    for axis in &mut rig.config.engineer.axes {
        axis.origin_m = unit.translation_m;
        axis.axis = normal.to_array();
    }
    rig.gripped = true;
    rig.grip_requested = true;
    let grip = compose(inverse(rig.tool), unit);
    let mut frame = sample_frame(&rig, logical, [0.; 3], Stage::Lift, false);
    frame.rod_spin_rad = 0.001;
    frame.q[0] = 0.001;
    rig.follow_plan(&frame).unwrap();
    assert!(position(rig.unit_pose).distance(position(unit)) < 1e-12);
    assert!((quaternion(rig.unit_pose) * glam::DVec3::Z - normal).length() < 1e-12);
    let spun_grip = compose(inverse(rig.tool), rig.unit_pose);
    assert!(position(spun_grip).distance(position(grip)) < 1e-12);
    assert!(quaternion(spun_grip).angle_between(quaternion(grip)) < 1e-7);
    assert!((quaternion(rig.unit_pose).angle_between(quaternion(unit)) - 0.001).abs() < 1e-9);
}
#[test]
fn chassis_route_is_continuous_and_playback_rejects_teleporting() {
    let route = ChassisRoute {
        lift_offset_m: [0.05, 0.],
        flip_offset_m: [0.1, 0.],
        flip_yaw_rad: 0.2,
        ..Default::default()
    };
    let origin = Pose::yawed([1., 0., 0.3], 0.7);
    assert_eq!(route.body(origin, [-0.05, 0.2, 0.6], [0.; 3]), origin);
    let lifted = route.body(origin, [-0.05, 0.2, 0.7], [0.; 3]);
    assert!((lifted.translation_m[0] - 1.05).abs() < 1e-12);
    let final_body = route.body(
        origin,
        [-0.05, 0.2, 0.7],
        [std::f64::consts::FRAC_PI_2, 0., 0.],
    );
    assert_eq!(
        final_body,
        route.body(
            origin,
            [-0.05, 0.2, 0.7],
            [std::f64::consts::FRAC_PI_2, 0.5, 0.]
        )
    );
    let mut rig = super::super::tests::mechanism_fixture();
    let previous = rig.report();
    let mut frame = sample_frame(&rig, [0.09, 0.2, 0.6], [0.; 3], Stage::Approach, true);
    frame.chassis.translation_m[0] = 0.1;
    assert!(rig.follow_plan(&frame).is_err());
    assert_eq!(rig.report(), previous);
    frame.chassis.translation_m[0] = f64::NAN;
    assert!(rig.follow_plan(&frame).is_err());
    assert_eq!(rig.report(), previous);
}
#[test]
fn moving_base_playback_keeps_the_held_gripper_on_its_world_guide() {
    let mut rig = super::super::tests::mechanism_fixture();
    let logical = [0.09, 0.2, 0.6];
    let expected = rig.target(logical, [0.; 3]);
    rig.config.engineer.tool = expected;
    rig.tool = expected;
    rig.unit_pose = rig.tool_unit_pose();
    rig.gripped = true;
    rig.grip_requested = true;
    let unit = rig.unit_pose;
    let mut frame = sample_frame(&rig, logical, [0.; 3], Stage::Lift, false);
    frame.chassis = Pose::yawed([0.; 3], 0.001);
    frame.q[0] = -0.001;
    rig.follow_plan(&frame).unwrap();
    assert_eq!(rig.engineer_body, frame.chassis);
    assert!(position(rig.tool).distance(position(expected)) < 1e-12);
    assert!(quaternion(rig.tool).angle_between(quaternion(expected)) < 1e-7);
    assert!(position(rig.unit_pose).distance(position(unit)) < 1e-12);
    assert!(rig.gripped);
}
#[test]
fn chassis_motion_sweeps_an_obstacle_and_restart_restores_initial_parking() {
    use rm_sim_physics::manipulator_collision::{CollisionAsset, CollisionBody, CollisionPart};
    let mut rig = super::super::tests::mechanism_fixture();
    rig.engineer_body = Pose::at([-0.05, 0., 0.]);
    rig.initial_engineer_body = rig.engineer_body;
    rig.collision = Some(std::sync::Arc::new(
        CollisionScene::new(CollisionAsset {
            schema: 1,
            excluded_pairs: vec![],
            bodies: vec![
                CollisionBody {
                    frame: "engineer/body".into(),
                    parts: vec![CollisionPart::Box { half_m: [0.01; 3] }],
                },
                CollisionBody {
                    frame: "floor".into(),
                    parts: vec![CollisionPart::Box {
                        half_m: [0.005, 0.2, 0.2],
                    }],
                },
            ],
        })
        .unwrap(),
    ));
    let origin = rig.engineer_body;
    let mut next = rig.clone();
    next.engineer_body.translation_m[0] = 0.05;
    assert!(rig.collision_contacts().unwrap().is_empty());
    assert!(next.collision_contacts().unwrap().is_empty());
    assert!(rig.accept_candidate(next).is_err());
    assert_eq!(rig.engineer_body, origin);
    assert!(rig.last_collision.is_some());
    rig.engineer_body.translation_m[0] = 0.4;
    rig.restart();
    assert_eq!(rig.engineer_body, origin);
}
#[test]
fn camera_preparation_changes_only_the_camera_link_and_is_preserved_by_clone() {
    let mut rig = super::super::tests::mechanism_fixture();
    rig.config.engineer_auxiliary.push(Auxiliary {
        name: "camera-lift".into(),
        reference: Pose::at([0.1, 0., 0.3]),
        stowed_delta: Pose::at([0., 0., -0.07]),
    });
    rig.engineer_q[0] = 0.7;
    let stowed = rig.world_frames();
    rig.camera_deployed = true;
    let deployed = rig.clone().world_frames();
    assert!(
        (position(deployed["engineer/camera-lift"])
            .distance(position(stowed["engineer/camera-lift"]))
            - 0.07)
            .abs()
            < 1e-12
    );
    for (name, frame) in &stowed {
        if name != "engineer/camera-lift" {
            assert_eq!(*frame, deployed[name]);
        }
    }
    assert_eq!(rig.report()["camera_deployed"], true);
}
#[test]
fn reverse_branch_connects_a_junction_and_can_continue_forwards() {
    let mut rig = super::super::tests::mechanism_fixture();
    let from = ([0.09, 0.2, 0.6], [0.; 3]);
    let segment = Segment {
        stage: Stage::Align,
        from,
        to: (from.0, [0., 0.15, 0.]),
    };
    rig.config.engineer.tool = rig.target(from.0, from.1);
    let goal = rig
        .config
        .engineer
        .solve(rig.target(segment.to.0, segment.to.1), [0.025; 6])
        .unwrap();
    let mut stats = Statistics::default();
    let roots = reverse_roots(&rig, &[segment], &[goal], &mut stats);
    assert_eq!(roots.len(), 1);
    assert_eq!(stats.connected_segments["Align reverse IK"], 1);
    let mut probe = at(rig, roots[0]);
    let n = segment.samples(0.005, 0.02);
    for i in 0..=n {
        let (p, a) = segment.point(i as f64 / n as f64);
        probe.solve_tool(p, a).unwrap();
        assert!(probe.position_error_m <= 1e-5);
        assert!(probe.angle_error_rad <= 1e-4);
    }
}
#[test]
fn reverse_segments_reject_unreachable_junction_without_mutating_the_fixture() {
    let rig = super::super::tests::mechanism_fixture();
    let before = rig.report();
    let task = segments(3, 0.5, [0.09, 0.2, 0.6], [0.; 3]);
    let mut stats = Statistics::default();
    assert!(reverse_roots(&rig, &task, &[[0.; 6]], &mut stats).is_empty());
    assert_eq!(stats.reverse_roots, 0);
    assert_eq!(rig.report(), before);
}
#[test]
fn task_goals_preserve_order_and_resume_without_withdrawing_a_seated_unit() {
    let full = goals(3, 0.5, [0.09, 0.2, 0.6], [0.; 3]);
    assert_eq!(
        full.iter().map(|g| g.0).collect::<Vec<_>>(),
        vec![
            Stage::Approach,
            Stage::Insert,
            Stage::Lift,
            Stage::Flip,
            Stage::Align
        ]
    );
    let resumed = goals(3, 0.5, [-0.05, 0.2, 0.7], [0.3, 0., 0.]);
    assert_eq!(
        resumed.iter().map(|g| g.0).collect::<Vec<_>>(),
        vec![Stage::Flip, Stage::Align]
    );
    assert!(goals(1, 0.5, [-0.05, 0.2, 0.6], [0.; 3]).is_empty());
}
#[test]
fn failed_planning_and_invalid_frames_never_mutate_the_live_rig() {
    let mut rig = super::super::tests::mechanism_fixture();
    let before = rig.report();
    assert!(
        rig.plan_assembly(3, 0., [0.09, 0.2, 0.6], [0.; 3], false, 0)
            .is_err()
    );
    assert_eq!(rig.report(), before);
    let frame = Frame {
        ticks: 1,
        validation: None,
        chassis: rig.engineer_body,
        rod_spin_rad: rig.rod_spin_rad,
        q: [99.; 6],
        logical: [0.09, 0.2, 0.6],
        angles: [0.; 3],
        close: true,
        stage: Stage::Grasp,
        free: false,
    };
    assert!(rig.follow_plan(&frame).is_err());
    assert_eq!(rig.report(), before);
}
#[test]
fn grasp_transform_roundtrips_clocking_and_axial_offsets() {
    let mut rig = super::super::tests::mechanism_fixture();
    rig.config.engineer.tool = rig.target([0.09, 0.2, 0.6], [0.; 3]);
    let original = rig.config.engineer.tool;
    for roll in [0., 1.2, -1.5] {
        for axial in [0., 0.02] {
            rig.grasp_roll_rad = roll;
            rig.grasp_axial_m = axial;
            rig.tool = rig.target([0.09, 0.2, 0.6], [0.; 3]);
            let unit = rig.tool_unit_pose();
            let expected = pose(
                position(original),
                quaternion(original) * DQuat::from_rotation_z(-GRASP_ROLL_RAD),
            );
            assert!(position(unit).distance(position(expected)) < 1e-12);
            assert!(quaternion(unit).angle_between(quaternion(expected)) < 1e-7);
        }
    }
}
