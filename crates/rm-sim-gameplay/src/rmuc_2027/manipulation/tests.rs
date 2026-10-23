// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use super::*;
fn sensors() -> [Sensors; 2] {
    [Sensors {
        attached: true,
        ..Default::default()
    }; 2]
}
fn sample(a: &mut Attempt, t: u64, s: [Sensors; 2]) {
    a.observe(t, s, true, true).unwrap();
}
fn prepare(d: u8) -> (Attempt, [Sensors; 2]) {
    let mut a = Attempt::new(d, [0.5, -0.5], 0).unwrap();
    let mut s = sensors();
    sample(&mut a, 0, s);
    for v in &mut s {
        v.insertion_m = TRAVEL_M;
    }
    sample(&mut a, 1, s);
    if d >= 2 {
        for v in &mut s {
            v.lift_m = TRAVEL_M;
        }
        sample(&mut a, 2, s);
    }
    if d >= 3 {
        for v in &mut s {
            v.p_rad = FRAC_PI_2;
        }
        sample(&mut a, 3, s);
        for (i, v) in s.iter_mut().enumerate() {
            v.q_rad = a.target_q_rad[i];
        }
        sample(&mut a, 4, s);
        sample(&mut a, 4 + ALIGN_NS, s);
    }
    (a, s)
}
#[test]
fn every_difficulty_requires_its_sequence_and_confirmation() {
    for d in 1..=4 {
        let (mut a, _) = prepare(d);
        assert!(!a.completed());
        for i in 0..a.core_count() {
            assert_eq!(a.cores[i].step, Step::Ready);
            a.confirm(i).unwrap();
        }
        assert!(a.completed());
        assert!(a.confirm(0).is_err());
    }
}
#[test]
fn cannot_skip_entrance_insertion_lift_or_raise() {
    let mut a = Attempt::new(3, [0.; 2], 0).unwrap();
    let mut s = sensors();
    s[0].insertion_m = TRAVEL_M;
    s[0].lift_m = TRAVEL_M;
    s[0].p_rad = FRAC_PI_2;
    sample(&mut a, 0, s);
    assert_eq!(a.cores[0].step, Step::Approach);
    s = sensors();
    sample(&mut a, 1, s);
    s[0].insertion_m = TRAVEL_M;
    s[0].lift_m = TRAVEL_M;
    sample(&mut a, 2, s);
    assert_eq!(a.cores[0].step, Step::Insert);
    s[0].lift_m = 0.;
    sample(&mut a, 3, s);
    s[0].p_rad = FRAC_PI_2;
    sample(&mut a, 4, s);
    assert_eq!(a.cores[0].step, Step::Lift);
}
#[test]
fn misalignment_does_not_insert_and_q_dwell_resets() {
    let mut a = Attempt::new(1, [0.; 2], 0).unwrap();
    let mut s = sensors();
    s[0].axis_error_rad = ANGLE_RAD + 0.001;
    sample(&mut a, 0, s);
    assert_eq!(a.cores[0].step, Step::Approach);
    let (mut a, mut s) = prepare(3);
    s[0].q_rad += ANGLE_RAD + 0.001;
    sample(&mut a, ALIGN_NS + 5, s);
    assert_eq!(a.cores[0].step, Step::Align);
    s[0].q_rad = 0.5;
    sample(&mut a, ALIGN_NS + 6, s);
    sample(&mut a, 2 * ALIGN_NS + 5, s);
    assert_eq!(a.cores[0].step, Step::Align);
    sample(&mut a, 2 * ALIGN_NS + 6, s);
    assert_eq!(a.cores[0].step, Step::Ready);
}
#[test]
fn failed_confirmation_locks_for_one_second() {
    let mut a = Attempt::new(1, [0.; 2], 0).unwrap();
    let mut s = sensors();
    sample(&mut a, 0, s);
    assert!(a.confirm(0).is_err());
    s[0].insertion_m = TRAVEL_M;
    sample(&mut a, 1, s);
    sample(&mut a, RETRY_NS - 1, s);
    assert!(a.confirm(0).is_err());
    sample(&mut a, RETRY_NS, s);
    a.confirm(0).unwrap();
}
#[test]
fn removal_death_and_continuous_rfid_loss_fail() {
    let (mut a, mut s) = prepare(2);
    s[0].attached = false;
    sample(&mut a, 3, s);
    assert_eq!(a.failure, Some(Failure::Removed));
    let (mut a, s) = prepare(2);
    a.observe(3, s, true, false).unwrap();
    assert_eq!(a.failure, Some(Failure::Defeated));
    let (mut a, s) = prepare(2);
    a.observe(3, s, false, true).unwrap();
    a.observe(CONTACT_NS + 2, s, true, true).unwrap();
    assert_eq!(a.failure, None);
    a.observe(CONTACT_NS + 3, s, false, true).unwrap();
    // A renewal at exactly the expiry loses to the old interval.
    a.observe(2 * CONTACT_NS + 3, s, true, true).unwrap();
    assert_eq!(a.failure, Some(Failure::ContactLost));
}
#[test]
fn pair_window_inclusive_and_level_four_deadline_exclusive() {
    for extra in [0, 1] {
        let mut a = Attempt::new(4, [0.; 2], 0).unwrap();
        let mut s = sensors();
        sample(&mut a, 0, s);
        s[0].insertion_m = TRAVEL_M;
        sample(&mut a, 1, s);
        s[1].insertion_m = TRAVEL_M;
        sample(&mut a, 1 + PAIR_NS + extra, s);
        assert_eq!(
            a.failure,
            if extra == 0 {
                None
            } else {
                Some(Failure::PairTimeout)
            }
        );
    }
    let mut a = Attempt::new(4, [0.; 2], 0).unwrap();
    sample(&mut a, super::super::assembly::ATTEMPT_NS, sensors());
    assert_eq!(a.failure, Some(Failure::Timeout));
    // Lower levels have no mechanism timeout; the old referee prototype remains separate.
    let mut a = Attempt::new(1, [0.; 2], 0).unwrap();
    sample(&mut a, 90_000_000_000, sensors());
    assert_eq!(a.failure, None);
}
#[test]
fn replay_restore_and_tick_partition_agree() {
    let (a, s) = prepare(3);
    let bytes = serde_json::to_vec(&a).unwrap();
    let mut one: Attempt = serde_json::from_slice(&bytes).unwrap();
    let mut many = a;
    sample(&mut one, 1_000_000_000, s);
    for t in (500_000_000..=1_000_000_000).step_by(7_812_500) {
        sample(&mut many, t, s);
    }
    assert_eq!(one, many);
    one.confirm(0).unwrap();
    many.confirm(0).unwrap();
    assert_eq!(one, many);
}
#[test]
fn invalid_observations_are_transactional() {
    let mut a = Attempt::new(1, [0.; 2], 5).unwrap();
    let before = a.clone();
    assert!(a.observe(4, sensors(), true, true).is_err());
    assert_eq!(a, before);
    let mut s = sensors();
    s[0].q_rad = f64::NAN;
    assert!(a.observe(6, s, true, true).is_err());
    assert_eq!(a, before);
    assert!(a.confirm(2).is_err());
    assert_eq!(a, before);
}

#[test]
fn confirmed_partner_cannot_be_removed_or_moved() {
    let (mut a, mut s) = prepare(4);
    a.confirm(0).unwrap();
    s[0].p_rad += 2. * ANGLE_RAD;
    sample(&mut a, ALIGN_NS + 5, s);
    assert_eq!(a.failure, Some(Failure::Removed));
    let (mut a, s) = prepare(4);
    a.confirm(0).unwrap();
    sample(&mut a, ALIGN_NS + 4 + PAIR_NS, s);
    a.confirm(1).unwrap();
    assert!(a.completed());
    let (mut a, s) = prepare(4);
    a.confirm(0).unwrap();
    sample(&mut a, ALIGN_NS + 5 + PAIR_NS, s);
    assert_eq!(a.failure, Some(Failure::PairTimeout));
    assert!(a.confirm(1).is_err());
}

#[test]
fn off_axis_rotation_revokes_readiness_at_all_levels() {
    for d in 1..=3 {
        let (mut a, mut s) = prepare(d);
        s[0].axis_error_rad = ANGLE_RAD + 0.001;
        sample(&mut a, ALIGN_NS + 5, s);
        assert!(a.confirm(0).is_err());
    }
}
