// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
use super::*;
use std::collections::BTreeSet;

#[test]
fn complete_manual_catalogs_roundtrip_and_keep_ambiguities_visible() {
    // Independent inventory from Table 1-5, including VTM and Radar wireless.
    let expected = [
        0x0001, 0x0002, 0x0003, 0x0101, 0x0104, 0x0105, 0x0201, 0x0202, 0x0203, 0x0204, 0x0206,
        0x0207, 0x0208, 0x0209, 0x020a, 0x020b, 0x020c, 0x020d, 0x020e, 0x0301, 0x0302, 0x0303,
        0x0305, 0x0306, 0x0307, 0x0308, 0x0309, 0x0310, 0x0311, 0x0a01, 0x0a02, 0x0a03, 0x0a04,
        0x0a05, 0x0a06,
    ];
    assert_eq!(
        serial::CATALOG
            .iter()
            .map(|d| d.command_id)
            .collect::<Vec<_>>(),
        expected
    );
    let expected_topics = [
        "KeyboardMouseControl",
        "CustomControl",
        "GameStatus",
        "GlobalUnitStatus",
        "GlobalLogisticsStatus",
        "GlobalSpecialMechanism",
        "Event",
        "RobotInjuryStat",
        "RobotRespawnStatus",
        "RobotStaticStatus",
        "RobotDynamicStatus",
        "RobotModuleStatus",
        "RobotPosition",
        "Buff",
        "PenaltyInfo",
        "RobotPathPlanInfo",
        "MapClickInfo",
        "MapClickCmd",
        "RadarInfoToClient",
        "CustomByteBlock",
        "AssemblyCommand",
        "TechCoreMotionStateSync",
        "RobotPerformanceSelectionCommand",
        "RobotPerformanceSelectionSync",
        "CommonCommand",
        "HeroDeployModeEventCommand",
        "DeployModeStatusSync",
        "RuneActivateCommand",
        "RuneStatusSync",
        "SentryStatusSync",
        "DartCommand",
        "DartSelectTargetStatusSync",
        "SentryCtrlCommand",
        "SentryCtrlResult",
        "AirSupportCommand",
        "AirSupportStatusSync",
    ];
    assert_eq!(
        client::CATALOG.iter().map(|d| d.topic).collect::<Vec<_>>(),
        expected_topics
    );
    for descriptor in client::CATALOG {
        let tags = descriptor
            .fields
            .iter()
            .map(|f| f.tag)
            .collect::<BTreeSet<_>>();
        assert!(!tags.contains(&0));
        assert_eq!(tags.len(), descriptor.fields.len());
    }
    let mock = MockData::default();
    mock.validate().unwrap();
    assert_eq!(mock.serial.len(), 35);
    assert_eq!(mock.client.len(), 36);
    assert_eq!(
        mock.interaction
            .iter()
            .map(|d| d.content_id())
            .collect::<Vec<_>>(),
        [
            0x100, 0x101, 0x102, 0x103, 0x104, 0x110, 0x120, 0x121, 0x200
        ]
    );
    for interaction in &mock.interaction {
        assert!(interaction.data_len() + 6 <= 118);
    }
    let json = serde_json::to_vec(&mock).unwrap();
    assert_eq!(serde_json::from_slice::<MockData>(&json).unwrap(), mock);
    let conflicts = serial::CATALOG
        .iter()
        .filter(|d| d.modeled_len.is_some_and(|n| n != d.overview_len))
        .map(|d| (d.command_id, d.overview_len, d.modeled_len.unwrap()))
        .collect::<Vec<_>>();
    assert_eq!(
        conflicts,
        [(0x0203, 16, 12), (0x0303, 15, 12), (0x0307, 103, 105)]
    );
    assert_eq!(
        client::RobotPosition::DESCRIPTOR.fields.last().unwrap().tag,
        5
    );
    assert_eq!(client::RobotInjuryStat::DESCRIPTOR.fields[1].tag, 2);
    assert_eq!(
        client::RadarInfoToClient::DESCRIPTOR.fields[0].proto_type,
        "repeated RadarSingleRobotInfo"
    );
}
#[test]
fn absent_zero_and_unknown_codes_remain_distinct_without_claiming_wire_layout() {
    let none = client::Message::GameStatus(client::GameStatus::default());
    let zero = client::Message::GameStatus(client::GameStatus {
        current_stage: Some(0),
        ..Default::default()
    });
    assert_ne!(none, zero);
    let unknown = client::Message::SentryCtrlResult(client::SentryCtrlResult {
        command_id: Some(65535),
        result_code: Some(999),
    });
    unknown.validate().unwrap();
    let bytes = serde_json::to_vec(&unknown).unwrap();
    assert_eq!(
        serde_json::from_slice::<client::Message>(&bytes).unwrap(),
        unknown
    );
    let word = 0xfedc_ba98_7654_3210;
    let changed = serial::with_bits(word, 20, 3, 5).unwrap();
    assert_eq!(serial::bits(changed, 20, 3).unwrap(), 5);
    assert_eq!(changed & !(7 << 20), word & !(7 << 20));
    assert_eq!(serial::with_bits(word, 0, 64, u64::MAX).unwrap(), u64::MAX);
    assert!(serial::with_bits(word, 63, 2, 0).is_err());
    assert!(serial::with_bits(word, 0, 3, 8).is_err());
    assert!(serial::bits(word, 0, 0).is_err());
}
#[test]
fn reject_malformed_lengths_nonfinite_data_and_unencodable_figures() {
    let mut path = serial::MapPath::default();
    path.delta_y_dm.pop();
    assert!(serial::Message::MapPath(path).validate().is_err());
    let mut data = serial::RobotCustomData2::default();
    data.data.push(0);
    assert!(serial::Message::RobotCustomData2(data).validate().is_err());
    let figure = serial::Figure {
        start_x: 2048,
        ..Default::default()
    };
    assert!(serial::InteractionData::Figure(figure).validate().is_err());
    assert!(
        serial::InteractionData::RobotData {
            content_id: 0x0120,
            data: vec![0; 4]
        }
        .validate()
        .is_err()
    );
    assert!(
        serial::InteractionData::RobotData {
            content_id: 0x0200,
            data: vec![0; 113]
        }
        .validate()
        .is_err()
    );
    assert!(
        client::Message::MapClickCmd(client::MapClickCmd {
            robot_id: Some(vec![1; 8]),
            ..Default::default()
        })
        .validate()
        .is_err()
    );
    assert!(
        client::Message::RobotPosition(client::RobotPosition {
            x: Some(f32::NAN),
            ..Default::default()
        })
        .validate()
        .is_err()
    );
    assert!(
        client::Message::RobotPathPlanInfo(client::RobotPathPlanInfo {
            offset_x: vec![128; 49],
            offset_y: vec![0; 49],
            ..Default::default()
        })
        .validate()
        .is_err()
    );
    assert!(
        client::Message::GlobalSpecialMechanism(client::GlobalSpecialMechanism {
            mechanism_id: vec![1],
            mechanism_time_sec: vec![]
        })
        .validate()
        .is_err()
    );
}
#[test]
fn own_projection_follows_native_state_and_requires_explicit_identity_and_frame() {
    use rm_sim_world::{RefereeCommand, Team};
    let (mut simulation, ids) = crate::workload::simulation(2);
    let red = ids[0];
    let blue = ids[1];
    let before = simulation.snapshot();
    let own = projection::own_robot(&before, false, red, 3, None).unwrap();
    assert!(own.position.is_none());
    assert!(own.robot_static.field_state.is_none());
    assert!(own.robot_static.max_chassis_energy.is_none());
    assert!(own.robot_dynamic.current_chassis_energy.is_none());
    assert!(own.robot_dynamic.total_projectiles_fired.is_none());
    assert!(projection::own_robot(&before, false, red, 103, None).is_err());
    assert!(projection::own_robot(&before, false, red, 7, None).is_err());
    assert!(projection::own_robot(&before, false, blue, 103, None).is_ok());
    let frame = projection::PositionFrame {
        arena_origin_protocol_m: [10., 20., 0.],
        arena_yaw_protocol_rad: std::f64::consts::FRAC_PI_2,
    };
    assert_eq!(frame.position([1., 2., 3.]).unwrap(), [8., 21., 3.]);
    assert!(frame.position([f64::MAX, 0., 0.]).is_err());
    let data = projection::own_robot(&before, true, red, 3, Some(frame)).unwrap();
    assert_eq!(data.game.is_paused, Some(true));
    assert!(data.position.unwrap().yaw.is_none());
    simulation
        .apply(&crate::protocol::Command::Referee(
            RefereeCommand::StartMatch,
        ))
        .unwrap();
    simulation.step(640).unwrap();
    simulation
        .apply(&crate::protocol::Command::Referee(
            RefereeCommand::SetRobotHp { robot: red, hp: 0 },
        ))
        .unwrap();
    let snapshot = simulation.snapshot();
    let data = projection::own_robot(&snapshot, false, red, 3, None).unwrap();
    assert_eq!(data.game.current_stage, Some(4));
    assert_eq!(data.robot_dynamic.current_health, Some(0));
    assert_eq!(data.robot_static.alive_state, Some(2));
    assert_eq!(data.time_ns, snapshot.time_ns);
    assert_eq!(
        snapshot.chassis.iter().find(|b| b.id == red).unwrap().team,
        Team::Red
    );
}
