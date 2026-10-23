// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! RMUL authority adapter on explicit practice geometry, never RMUC CAD bindings.
//!
//! UL §3.2.1/Figure 3-5 specifies the 12 × 8 m footprint, but no verified
//! field package is supplied. The flat practice rectangles below deliberately
//! approximate RFID at ground height; they do not model elevated highlands,
//! tunnels, workbench hardware or the official antenna layout.
use crate::gameplay::rmul_2027::{Format, Match, Phase, Role, Zone};
use crate::{ChassisPlacement, Field, FieldError, RefereeCommand, RobotKind, referee};

impl Field {
    /// Distinct RMUL 2027 V1.0.0 authority, absent for RMUC and bare worlds.
    ///
    /// ```
    /// use rm_sim_world::{Field, FieldConfig, gameplay::rmul_2027::Format};
    /// let field = Field::new(&FieldConfig {
    ///     rmul_2027: Some(Format::Infantry), ..FieldConfig::default()
    /// }).unwrap();
    /// assert!(field.rmul_2027().is_some());
    /// assert!(field.rmuc_2027().is_none());
    /// ```
    pub fn rmul_2027(&self) -> Option<&Match> {
        self.rmul_2027.as_ref()
    }

    pub(crate) fn rmul_role(kind: RobotKind) -> Result<Role, FieldError> {
        match kind {
            RobotKind::Engineer => Ok(Role::Heavy),
            RobotKind::Infantry => Ok(Role::Infantry),
            RobotKind::Sentry => Ok(Role::Sentry),
            _ => Err(FieldError::Referee("unsupported RMUL 2027 ground role")),
        }
    }

    pub(crate) fn check_rmul_placement(
        &self,
        game: &Match,
        p: &ChassisPlacement,
    ) -> Result<(), FieldError> {
        let role = Self::rmul_role(p.kind)?;
        if game.phase != Phase::Idle {
            return Err(FieldError::Referee(
                "RMUL deployment requires an idle round",
            ));
        }
        if game.format == Format::Infantry && role != Role::Infantry {
            return Err(FieldError::Referee("RMUL Infantry admits only Infantry 3"));
        }
        if game
            .robots
            .iter()
            .any(|r| r.team == referee::game_team(p.team) && r.role == role)
        {
            return Err(FieldError::Referee(
                "RMUL permits one robot of each role per team",
            ));
        }
        if p.performance.is_some_and(|choice| {
            !matches!(
                (game.format, role, choice),
                (
                    Format::ThreeVsThree,
                    Role::Infantry,
                    crate::gameplay::Performance::Infantry { .. }
                )
            )
        }) {
            return Err(FieldError::Referee(
                "performance choices apply only to RMUL 3v3 Infantry",
            ));
        }
        Ok(())
    }

    pub(crate) fn rmul_command(&mut self, command: RefereeCommand) -> Result<(), FieldError> {
        let game = self.rmul_2027.as_mut().unwrap();
        match command {
            RefereeCommand::StartMatch => {
                if game.phase != Phase::Idle {
                    return Err(FieldError::Referee("reset RMUL before starting again"));
                }
                game.begin_countdown().map_err(FieldError::Referee)?;
            }
            RefereeCommand::ResetMatch => {
                game.reset();
            }
            RefereeCommand::FinishMatch => {
                if !matches!(game.phase, Phase::Countdown | Phase::Running) {
                    return Err(FieldError::Referee("no active RMUL round"));
                }
                game.finish();
            }
            RefereeCommand::SetGold { team, gold } if game.format == Format::ThreeVsThree => {
                game.teams[team.index()].gold = gold;
            }
            RefereeCommand::BuyAmmo {
                robot,
                caliber,
                amount,
            } => {
                let large = caliber == crate::Caliber::Mm42;
                let unit = if large { 1 } else { 10 };
                if amount == 0 || amount % unit != 0 {
                    return Err(FieldError::Referee("RMUL ammo uses whole ten-gold batches"));
                }
                game.buy_ammo(robot, large, amount / unit)
                    .map_err(FieldError::Referee)?;
            }
            RefereeCommand::SetPerformance { robot, performance } => {
                let crate::gameplay::Performance::Infantry { chassis, launcher } = performance
                else {
                    return Err(FieldError::Referee(
                        "RMUL supports Infantry profile choices only",
                    ));
                };
                game.configure_infantry(
                    robot,
                    chassis == crate::gameplay::InfantryChassis::PowerFocused,
                    launcher == crate::gameplay::InfantryLauncher::BurstFocused,
                )
                .map_err(FieldError::Referee)?;
            }
            RefereeCommand::SetRobotModules {
                robot,
                main_online,
                speed_online,
                offline_armor,
                supercap_online,
            } => {
                if offline_armor > 4
                    || !game.observe_modules(
                        robot,
                        main_online,
                        speed_online,
                        offline_armor + u8::from(!supercap_online),
                    )
                {
                    return Err(FieldError::Referee("invalid RMUL module observation"));
                }
            }
            RefereeCommand::SetRobotHp { robot, hp } => {
                game.set_robot_hp(robot, hp).map_err(FieldError::Referee)?
            }
            RefereeCommand::DamageRobot { robot, amount } => {
                if !game.robots.iter().any(|r| r.id == robot) {
                    return Err(FieldError::Referee("unknown RMUL robot"));
                }
                game.penalty_damage(robot, amount);
            }
            RefereeCommand::ReviveRobot { robot } => game
                .set_robot_hp(robot, u32::MAX)
                .map_err(FieldError::Referee)?,
            _ => {
                return Err(FieldError::Referee(
                    "command is unavailable in this RMUL competition",
                ));
            }
        }
        self.sync_defeats();
        Ok(())
    }

    pub(crate) fn convert_rmul_heavy(&mut self, id: u32) -> Result<(), FieldError> {
        let game = self.rmul_2027.as_ref().unwrap();
        let robot = game
            .robots
            .iter()
            .find(|r| r.id == id && r.role == Role::Heavy)
            .ok_or(FieldError::Referee(
                "only RMUL Heavy can install a launcher",
            ))?;
        if robot.hp == 0 || !robot.online || robot.heavy_installed {
            return Err(FieldError::Referee(
                "RMUL Heavy is unavailable or already has a launcher",
            ));
        }
        let mut body = self
            .physics
            .chassis_snapshot(id)
            .ok_or(FieldError::Referee("unknown chassis"))?;
        let sign = if robot.team == crate::gameplay::Team::Red {
            1.
        } else {
            -1.
        };
        let [x, y, z] = body.pose.translation_m;
        if (x - 4.75 * sign).abs() > 1.1
            || (y - 2.75 * sign).abs() > 1.
            || !(-0.1..=0.5).contains(&z)
        {
            return Err(FieldError::Referee(
                "RMUL launcher installation requires the practice supply/workbench",
            ));
        }
        if body
            .velocity_m_s
            .iter()
            .chain(&body.angular_velocity_rad_s)
            .any(|v| v.abs() > 0.1)
        {
            return Err(FieldError::Referee(
                "stop before installing the RMUL launcher",
            ));
        }
        // UL §3.3.6: the install action is virtual; the referee-confirmed shot
        // remains necessary for upgraded HP/heat. This is a practice body proxy.
        self.rmul_2027
            .as_mut()
            .unwrap()
            .set_heavy_installed(id, true)
            .map_err(FieldError::Referee)?;
        let config = crate::ChassisConfig::folding_hero();
        body.pose.translation_m[2] += config.rest_height_m() - body.config.rest_height_m();
        body.config = config;
        body.command = crate::ChassisCommand::default();
        body.fold_angle_rad = 0.;
        body.placement_revision = body.placement_revision.saturating_add(1);
        self.physics
            .replace_chassis(&body)
            .map_err(FieldError::Chassis)?;
        Ok(())
    }

    pub(crate) fn advance_rmul(&mut self, duration_ns: u64) {
        if let Some(game) = &mut self.rmul_2027 {
            for r in game.robots.clone() {
                let p = self.physics.chassis_position_m(r.id);
                let sign = if r.team == crate::gameplay::Team::Red {
                    1.
                } else {
                    -1.
                };
                let contains = |center: [f64; 2], half: [f64; 2]| {
                    p.is_some_and(|p| {
                        (p[0] - center[0]).abs() <= half[0]
                            && (p[1] - center[1]).abs() <= half[1]
                            && (-0.1..=0.5).contains(&p[2])
                    })
                };
                game.observe_supply(r.id, contains([4.75 * sign, 2.75 * sign], [1.1, 1.]));
                for (zone, center, half) in [
                    (Zone::RedHighland, [2.3, 0.8], [0.65, 0.65]),
                    (Zone::BlueHighland, [-2.3, -0.8], [0.65, 0.65]),
                    (Zone::Center, [0., 0.], [1.3, 1.3]),
                ] {
                    game.observe_zone(r.id, zone, contains(center, half));
                }
                game.observe_power(r.id, self.physics.chassis_drive_power_w(r.id).unwrap_or(0.));
            }
            game.advance(duration_ns);
            for r in &game.robots {
                self.physics.set_chassis_power(
                    r.id,
                    f64::from(r.stats().chassis_power_w),
                    !game.chassis_enabled(r.id),
                );
            }
        }
        self.sync_defeats();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ArmorTarget, Caliber, ChassisConfig, Contact, FieldConfig, Pose, Shot, StaticGeometry,
        Team, tick_ns,
    };

    fn field(format: Format) -> Field {
        let mut f = Field::new(&FieldConfig {
            rmul_2027: Some(format),
            ..Default::default()
        })
        .unwrap();
        for (i, team) in Team::BOTH.into_iter().enumerate() {
            f.add_chassis(&ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([if i == 0 { 1.8 } else { -1.8 }, 2., 0.16]),
                team,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        }
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(5_000_000_000 / tick_ns()).unwrap();
        f
    }

    #[test]
    fn infantry_admission_is_atomic_and_rejects_other_roles() {
        let mut f = field(Format::Infantry);
        f.referee_command(RefereeCommand::ResetMatch).unwrap();
        let before = f.snapshot();
        for kind in [
            RobotKind::Infantry,
            RobotKind::Sentry,
            RobotKind::Engineer,
            RobotKind::Hero,
        ] {
            assert!(
                f.add_chassis(&ChassisPlacement {
                    config: ChassisConfig::default(),
                    spawn: Pose::at([0., 2., 0.16]),
                    team: Team::Red,
                    kind,
                    performance: None,
                })
                .is_err()
            );
            assert_eq!(before, f.snapshot());
        }
    }

    #[test]
    fn heavy_installation_requires_the_ground_level_practice_workbench() {
        let mut f = Field::new(&FieldConfig {
            rmul_2027: Some(Format::ThreeVsThree),
            ..Default::default()
        })
        .unwrap();
        let id = f
            .add_chassis(&ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([4.75, 2.75, 1.0]),
                team: Team::Red,
                kind: RobotKind::Engineer,
                performance: None,
            })
            .unwrap();
        let before = f.snapshot();
        assert!(f.convert_heavy_to_hero(id).is_err());
        assert_eq!(f.snapshot(), before);
        f.place_chassis(id, Pose::at([4.75, 2.75, 0.16])).unwrap();
        f.convert_heavy_to_hero(id).unwrap();
        let game = f.rmul_2027().unwrap();
        assert!(game.robots[0].heavy_installed);
        assert!(!game.robots[0].heavy_upgraded);
        assert_eq!(game.robots[0].max_hp, 100);
    }

    #[test]
    fn infantry_contacts_use_native_damage_lifesteal_and_detection_interval() {
        let mut f = field(Format::Infantry);
        f.referee_command(RefereeCommand::SetRobotHp { robot: 0, hp: 100 })
            .unwrap();
        let contact = Contact {
            projectile: 0,
            shooter: Some(0),
            caliber: Caliber::Mm17,
            target: ArmorTarget::Chassis {
                chassis: 1,
                plate: 0,
            },
            position_m: [0.; 3],
            local_m: [0.; 3],
            normal_speed_m_s: 12.,
        };
        assert_eq!(f.score(f.time_ns(), contact.clone()).unwrap().damage, 0);
        let contact = Contact {
            normal_speed_m_s: 25.,
            ..contact
        };
        assert_eq!(f.score(f.time_ns(), contact.clone()).unwrap().damage, 20);
        let game = f.rmul_2027().unwrap();
        assert_eq!(game.robots[0].hp, 110);
        assert_eq!(game.robots[1].hp, 180);
        assert_eq!(f.score(f.time_ns(), contact.clone()).unwrap().damage, 0);
        assert_eq!(
            f.score(f.time_ns() + 50_000_000, contact).unwrap().damage,
            20
        );
    }

    #[test]
    fn full_checkpoint_preserves_rmul_identity_and_launch_state() {
        let mut f = field(Format::Infantry);
        f.fire(
            Pose::at([0., 0., 2.]),
            Shot::at_limit(Caliber::Mm17),
            Some(0),
        )
        .unwrap();
        let s = f.snapshot();
        let mut restored = Field::restore(&s, &f.static_geometry_snapshot(), 0.).unwrap();
        assert_eq!(restored.rmul_2027(), f.rmul_2027());
        f.step(10).unwrap();
        restored.step(10).unwrap();
        assert_eq!(restored.rmul_2027(), f.rmul_2027());
        let mut broken = s.clone();
        broken.rmul_2027 = None;
        assert!(Field::restore(&broken, &StaticGeometry::default(), 0.).is_err());
        let mut broken = s;
        broken
            .restore
            .as_mut()
            .unwrap()
            .rmul_2027
            .as_mut()
            .unwrap()
            .format = Format::ThreeVsThree;
        assert!(Field::restore(&broken, &StaticGeometry::default(), 0.).is_err());
    }

    #[test]
    fn running_authority_keeps_explicit_clock_partition_independent() {
        let config = FieldConfig {
            rmul_2027: Some(Format::ThreeVsThree),
            ..Default::default()
        };
        let mut a = Field::new(&config).unwrap();
        let mut b = Field::new(&config).unwrap();
        a.referee_command(RefereeCommand::StartMatch).unwrap();
        b.referee_command(RefereeCommand::StartMatch).unwrap();
        a.step(1024).unwrap();
        for ticks in [1, 127, 256, 640] {
            b.step(ticks).unwrap();
        }
        assert_eq!(a.snapshot(), b.snapshot());
    }
    #[test]
    fn practice_supply_profiles_and_control_use_the_native_authority() {
        let mut f = Field::new(&FieldConfig {
            rmul_2027: Some(Format::ThreeVsThree),
            ..Default::default()
        })
        .unwrap();
        for (team, sign) in [(Team::Red, 1.), (Team::Blue, -1.)] {
            f.add_chassis(&ChassisPlacement {
                config: ChassisConfig::default(),
                spawn: Pose::at([4.75 * sign, 2.75 * sign, 0.16]),
                team,
                kind: RobotKind::Infantry,
                performance: None,
            })
            .unwrap();
        }
        f.referee_command(RefereeCommand::SetPerformance {
            robot: 0,
            performance: crate::gameplay::Performance::Infantry {
                chassis: crate::gameplay::InfantryChassis::PowerFocused,
                launcher: crate::gameplay::InfantryLauncher::BurstFocused,
            },
        })
        .unwrap();
        assert_eq!(f.rmul_2027().unwrap().robots[0].max_hp, 300);
        f.referee_command(RefereeCommand::StartMatch).unwrap();
        f.step(6_000_000_000 / tick_ns()).unwrap();
        f.buy_ammo(0, Caliber::Mm17).unwrap();
        assert_eq!(f.rmul_2027().unwrap().robots[0].allowance, 10);
        assert_eq!(f.rmul_2027().unwrap().teams[0].gold, 190);
        f.place_chassis(0, Pose::at([0., 0., 0.16])).unwrap();
        f.step(5_000_000_000 / tick_ns()).unwrap();
        let game = f.rmul_2027().unwrap();
        assert_eq!(
            game.zones[Zone::Center.index()].controller,
            Some(crate::gameplay::Team::Red)
        );
        assert!(game.teams[1].control < 600);
        let mut restored =
            Field::restore(&f.snapshot(), &f.static_geometry_snapshot(), 0.).unwrap();
        f.step(128).unwrap();
        restored.step(128).unwrap();
        assert_eq!(f.rmul_2027(), restored.rmul_2027());
    }
}
