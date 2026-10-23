// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Field layout records; construction and validation remain in `Field`.
use crate::{
    BaseConfig, ChassisConfig, Pose, RefereeConfig, RobotKind, RuneKind, StepDuration, Team,
    gameplay, projectile, rmuc_2027, zones,
};
use serde::{Deserialize, Serialize};

/// One Power Rune in the layout: which mode it starts in and where its hub sits.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RuneConfig {
    /// Rune mode. The referee's three-minute stage change converts every rune
    /// to Big (section 5.5.2).
    pub kind: RuneKind,
    /// Face centre; local +x points into the wheel, away from the shooter.
    /// The scoring front faces local -x; local +z is up.
    pub hub_pose: Pose,
    /// Use the measured 698.5 mm CAD target orbit instead of the nominal 700 mm.
    pub cad_orbit: bool,
}
impl Default for RuneConfig {
    fn default() -> Self {
        Self {
            kind: RuneKind::Small,
            hub_pose: Pose::at([6.0, 0.0, 1.6]),
            cad_orbit: false,
        }
    }
}

/// One outpost in the layout: where the tower base stands, how fast it turns
/// and, when the asset supplied one, its verified rotor pivot.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OutpostConfig {
    /// Optional verified pivot in CAD glTF metres; legacy configurations use the fitted pivot.
    #[serde(default)]
    pub pivot_cad_m: Option<[f64; 3]>,
    /// Tower base pose: translation is the base on the floor, rotation maps the
    /// tower's local FLU frame (forward = CAD +x) into the world.
    pub origin: Pose,
    /// Rotor speed counter-clockwise about the tower's up axis. Section 5.5.1
    /// gives 0.8π rad/s; the constructor rejects magnitudes above 10 rad/s.
    pub speed_rad_s: f64,
}

/// A driven chassis, whose team it plays for and where it starts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChassisPlacement {
    /// Preset dimensions, suspension, armor geometry and drivetrain budget.
    pub config: ChassisConfig,
    /// Body centre pose; the wheels find the ground on the first tick.
    pub spawn: Pose,
    /// Team whose referee record, base and buffs this chassis belongs to.
    pub team: Team,
    /// Robot class the referee records for it; a placement that names none
    /// is an infantry, which is what older serialized layouts hold.
    #[serde(default)]
    pub kind: RobotKind,
    /// Section 5.4.2 performance type the robot selected; `None` takes the
    /// class default (a long-range Hero, an HP-focused cooling-focused Infantry).
    #[serde(default)]
    pub performance: Option<rm_sim_gameplay::Performance>,
}

/// Static field layout. Everything here is placed once; motion comes from the clock.
///
/// ```rust
/// use rm_sim_world::FieldConfig;
///
/// // Defaults provide bare physics; native bindings opt into match rules.
/// let config = FieldConfig::default();
/// assert!(config.runes.is_empty());
/// assert!(config.outposts.is_empty());
/// assert_eq!(config.floor_height_m, 0.0);
/// assert!(config.bases.is_empty());
/// ```
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldConfig {
    /// Immutable physical/rules step. Custom rates are API-only; server sessions
    /// accept only the default until protocol prediction/planning supports them.
    #[serde(default)]
    pub step_duration: StepDuration,
    /// Native RMUC 2027 rules and source geometry; absent for bare physics.
    #[serde(default)]
    pub rmuc_2027: Option<rmuc_2027::Config>,
    /// RMUL 2027 competition on explicitly approximate practice geometry.
    /// Mutually exclusive with native RMUC structure bindings.
    #[serde(default)]
    pub rmul_2027: Option<gameplay::rmul_2027::Format>,
    /// Historical layout record; must be empty. Native bases use `rmuc_2027`.
    #[serde(default)]
    pub bases: Vec<BaseConfig>,
    /// Historical layout record; must be empty. Native Runes use `rmuc_2027`.
    pub runes: Vec<RuneConfig>,
    /// Historical layout record; must be empty. Native towers use `rmuc_2027`.
    pub outposts: Vec<OutpostConfig>,
    /// Height of the flat floor plane. Zero is the playing floor; a field
    /// that supplies its own ground mesh lowers it to a catch surface.
    pub floor_height_m: f64,
    /// Chassis present from the start, in id order; more can join and
    /// leave with [`crate::Field::add_chassis`] and [`Field::remove_chassis`].
    #[serde(default)]
    pub chassis: Vec<ChassisPlacement>,
    /// Historical configuration record; must be `None`.
    #[serde(default)]
    pub referee: Option<RefereeConfig>,
    /// Ball restitution, hard flight limit and low-speed retirement rule. The
    /// default is the simulator's control behaviour. A host and every client
    /// predicting for it must use the same value, so it travels in the
    /// snapshot's [`FieldRestore`] as well.
    #[serde(default)]
    pub projectile_policy: projectile::ProjectilePolicy,
    /// Historical zone records; must be empty. Native zones use `rmuc_2027`.
    #[serde(default)]
    pub zones: Vec<zones::ZoneArea>,
}

impl Default for FieldConfig {
    fn default() -> Self {
        Self {
            step_duration: StepDuration::default(),
            rmuc_2027: None,
            rmul_2027: None,
            zones: Vec::new(),
            bases: Vec::new(),
            floor_height_m: 0.0,
            chassis: Vec::new(),
            referee: None,
            projectile_policy: projectile::ProjectilePolicy::default(),
            runes: Vec::new(),
            outposts: Vec::new(),
        }
    }
}
