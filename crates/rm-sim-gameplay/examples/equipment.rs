// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Native equipment experiments without physics, a complete match or wall time.
use rm_sim_gameplay::{
    Team,
    rmuc_2027::{
        CAPTURE_NS, Control,
        equipment::{Bases, Outposts},
        rune::{Hit, Rune},
    },
};

fn main() {
    let mut control = Control::new();
    let mut outpost_hp = [1000; 2];
    let mut rotor_stopped_ns = [None; 2];
    let mut base_hp = [10_000; 2];
    let mut base_lowest_hp = base_hp;

    control.advance_ns(1_000_000_000);
    Outposts::new(&mut control, &mut outpost_hp, &mut rotor_stopped_ns).damage(Team::Red, 1000);
    assert!(!control.base_protected(Team::Red));
    assert_eq!(
        Bases::new(&control, &mut base_hp, &mut base_lowest_hp).damage(Team::Red, 20),
        20
    );

    // Control remains the only owner/territory authority. Its rebuild event
    // changes HP without restarting the rotor.
    let events = control.advance_ns(CAPTURE_NS);
    Outposts::new(&mut control, &mut outpost_hp, &mut rotor_stopped_ns).apply_events(&events);
    assert_eq!(outpost_hp[0], 500);
    assert_eq!(rotor_stopped_ns[0], Some(1_000_000_000));
    assert_eq!(control.snapshot().outposts[0].owner, Team::Blue);

    // Rune already is an independent native equipment authority. The caller
    // owns the explicit timestamp and supplies eligible detected strikes.
    let mut rune = Rune::new(-1, 42);
    let mut now_ns = 0;
    rune.settle(now_ns);
    rune.activate(now_ns).unwrap();
    loop {
        let blade = rune.targets.iter().position(|&lit| lit).unwrap() as u8;
        if rune.hit(now_ns, blade, 10) == Hit::Completed {
            break;
        }
        now_ns += 100_000_000;
        rune.settle(now_ns);
    }
    println!(
        "outpost HP={outpost_hp:?}, Base HP={base_hp:?}, Rune benefit={:?}",
        rune.buff
    );
}
