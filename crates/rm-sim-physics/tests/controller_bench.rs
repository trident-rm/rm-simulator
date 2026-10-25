// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
#[allow(dead_code)]
#[path = "../examples/controller_bench.rs"]
mod bench;
use rm_sim_physics::{PhysicalRuntime, StepDuration, runtime::session::PhysicalSession};

#[test]
fn positioning_robot_tracks_both_targets_and_replays_after_reset_at_both_rates() {
    let prepared = bench::prepare().unwrap();
    for duration in [StepDuration::HZ_128, StepDuration::HZ_1000] {
        let recipe =
            || PhysicalRuntime::from_recipe(duration, |runtime| bench::install(runtime, &prepared));
        let mut session = PhysicalSession::new(recipe).unwrap();
        let mut reference = Vec::new();
        for episode in 0..2 {
            let joints = bench::bindings(session.runtime()).unwrap();
            let ticks_per_phase = 2_000_000_000 / duration.as_ns();
            for phase in 0..2 {
                let target = bench::target(session.stamp().time_ns);
                for _ in 0..ticks_per_phase {
                    bench::control(session.runtime_mut(), joints, target).unwrap();
                    session.step(1).unwrap();
                    for joint in joints {
                        assert!(
                            session.runtime().joint_applied_effort(joint).unwrap().abs() <= 20.
                        );
                    }
                }
                let state = bench::observe(session.runtime(), joints).unwrap();
                for axis in 0..2 {
                    assert!(
                        (state[axis].position - target[axis]).abs() < 0.001,
                        "{duration:?}: {state:?} vs {target:?}"
                    );
                    assert!(state[axis].velocity.abs() < 0.005, "{state:?}");
                }
                if episode == 0 {
                    reference.push(state);
                } else {
                    assert_eq!(state, reference[phase]);
                }
            }
            session.reset().unwrap();
            assert!(session.runtime().joint_state(joints[0]).is_err());
            assert_eq!(session.stamp().time_ns, 0);
        }
    }
}
