// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Manual review of rigid groups and explicit joints; never inferred from names.
use super::{Panel, importer::*};
use rxui::{Element, prelude::*};
use std::path::PathBuf;

pub struct Review {
    pub project: Project,
    pub selected_part: usize,
    pub selected_group: u64,
    pub proxies: bool,
    pub revision: u64,
    pub save: PathBuf,
    pub export: PathBuf,
    pub status: String,
    pub pivot: String,
    pub axis: String,
    pub limits: String,
    pub density: String,
    pub preview: f64,
    pub explicit_mass: String,
    pub explicit_com: String,
    pub explicit_tensor: String,
}
impl Review {
    pub fn new(project: Project, save: PathBuf) -> Self {
        let export = save.with_file_name(format!(
            "{}.recipe.json",
            save.file_stem().unwrap_or_default().to_string_lossy()
        ));
        let selected_group = project.groups[0].id;
        Self {
            project,
            selected_part: 0,
            selected_group,
            proxies: false,
            revision: 1,
            save,
            export,
            status:
                "Review native frames/configuration geometry, groups and mass basis before export."
                    .into(),
            pivot: "0, 0, 0".into(),
            axis: "0, 0, 1".into(),
            limits: "-1, 1".into(),
            density: "2700".into(),
            preview: 0.,
            explicit_mass: String::new(),
            explicit_com: "0, 0, 0".into(),
            explicit_tensor: String::new(),
        }
    }
    fn select(&mut self, group: u64) {
        self.selected_group = group;
        self.preview = 0.;
        if let Some(j) = self
            .project
            .groups
            .iter()
            .find(|g| g.id == group)
            .and_then(|g| g.joint.as_ref())
        {
            self.pivot = j
                .pivot_m
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ");
            self.axis = j
                .axis
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ");
            self.limits = j
                .limits
                .map(|v| format!("{}, {}", v[0], v[1]))
                .unwrap_or_default();
        }
        self.revision += 1;
    }
    fn select_part(&mut self, index: usize) {
        self.selected_part = index;
        self.explicit_mass.clear();
        self.explicit_com = "0, 0, 0".into();
        self.explicit_tensor.clear();
        if let Some(m) = &self.project.parts[index].override_mass {
            self.explicit_mass = m.mass_kg.to_string();
            self.explicit_com = m
                .com_m
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ");
            let a = m.inertia_kg_m2;
            self.explicit_tensor = [a[0][0], a[1][1], a[2][2], a[0][1], a[0][2], a[1][2]]
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ");
        }
        self.select(self.project.parts[index].group);
    }
    fn new_group(&mut self) {
        let Some(id) = self
            .project
            .groups
            .iter()
            .map(|g| g.id)
            .max()
            .unwrap_or(0)
            .checked_add(1)
        else {
            self.status = "Group ID space exhausted".into();
            return;
        };
        let part = &mut self.project.parts[self.selected_part];
        let parent = part.group;
        part.group = id;
        self.pivot = part
            .pose
            .translation_m
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(", ");
        self.project.groups.push(Group {
            id,
            name: format!("Moving group {id}"),
            pose: part.pose,
            fixed: false,
            proxy: Proxy::PerPartBoxes,
            mass_mode: MassMode::Unspecified,
            joint: Some(Joint {
                parent,
                motion: Motion::Fixed,
                pivot_m: part.pose.translation_m,
                axis: [0., 0., 1.],
                limits: None,
                reviewed: false,
                evidence: String::new(),
            }),
        });
        self.selected_group = id;
        self.revision += 1;
        self.status="New group keeps the part's source placement. Review its parent joint; nothing was inferred.".into();
    }
    fn edit_group(&mut self, edit: impl FnOnce(&mut Group)) {
        if let Some(group) = self
            .project
            .groups
            .iter_mut()
            .find(|g| g.id == self.selected_group)
        {
            edit(group);
            self.revision += 1;
        }
    }
    fn merge_into(&mut self, target: u64) {
        let source = self.selected_group;
        if source == target || !self.project.groups.iter().any(|g| g.id == target) {
            return;
        }
        if self
            .project
            .groups
            .iter()
            .any(|g| g.joint.as_ref().is_some_and(|j| j.parent == source))
        {
            self.status = "Reparent child groups explicitly before merging their parent.".into();
            return;
        }
        for part in &mut self.project.parts {
            if part.group == source {
                part.group = target;
            }
        }
        self.project.groups.retain(|g| g.id != source);
        self.project
            .groups
            .iter_mut()
            .find(|g| g.id == target)
            .unwrap()
            .mass_mode = MassMode::Unspecified;
        self.select(target);
        self.status =
            "Merged source placements without moving Parts; review the target mass basis again."
                .into();
    }
    fn remove_empty(&mut self) {
        use std::collections::BTreeSet;
        let before = self.project.groups.len();
        loop {
            let used: BTreeSet<_> = self.project.parts.iter().map(|p| p.group).collect();
            let parents: BTreeSet<_> = self
                .project
                .groups
                .iter()
                .filter_map(|g| g.joint.as_ref().map(|j| j.parent))
                .collect();
            let n = self.project.groups.len();
            self.project
                .groups
                .retain(|g| used.contains(&g.id) || parents.contains(&g.id));
            if n == self.project.groups.len() {
                break;
            }
        }
        if !self
            .project
            .groups
            .iter()
            .any(|g| g.id == self.selected_group)
        {
            self.select(self.project.groups[0].id);
        }
        self.revision += 1;
        self.status = format!(
            "Removed {} empty leaf groups and their parent joints; populated children require explicit reparenting.",
            before - self.project.groups.len()
        );
    }
    fn joint(&mut self, motion: Motion) {
        let result = (|| {
            let pivot = triple(&self.pivot)?;
            let axis = triple(&self.axis)?;
            let limits = if matches!(motion, Motion::Fixed) || self.limits.trim().is_empty() {
                None
            } else {
                let v = numbers(&self.limits)?;
                if v.len() != 2 || v[0] > v[1] {
                    return Err("limits need two ordered values".to_owned());
                }
                Some([v[0], v[1]])
            };
            let group = self
                .project
                .groups
                .iter_mut()
                .find(|g| g.id == self.selected_group)
                .ok_or("missing group")?;
            let joint = group
                .joint
                .as_mut()
                .ok_or("root has no parent; choose a moving group")?;
            if axis.iter().map(|x| x * x).sum::<f64>() < 1e-20 {
                return Err("axis must be nonzero".into());
            }
            joint.pivot_m = pivot;
            joint.axis = axis;
            joint.limits = limits;
            joint.motion = motion;
            joint.reviewed = true;
            joint.evidence = "manually reviewed in RXUI; not native mate inference".into();
            self.project.validate()?;
            Ok(())
        })();
        self.status = match result {
            Ok(()) => {
                "Joint frames applied; saved source placements retained at zero coordinate.".into()
            }
            Err(e) => e,
        };
        self.revision += 1;
    }
}
fn numbers(text: &str) -> Result<Vec<f64>, String> {
    text.split(',')
        .map(|s| {
            s.trim()
                .parse::<f64>()
                .map_err(|_| "expected comma-separated numbers".into())
                .and_then(|x| {
                    if x.is_finite() {
                        Ok(x)
                    } else {
                        Err("nonfinite quantity".into())
                    }
                })
        })
        .collect()
}
fn triple(text: &str) -> Result<[f64; 3], String> {
    let values = numbers(text)?;
    values
        .try_into()
        .map_err(|_| "expected three numbers".into())
}

impl Panel {
    fn review(&mut self) -> &mut Review {
        self.import
            .as_mut()
            .expect("import controls only exist in import mode")
    }
    pub(super) fn import_view(&self, cx: &mut ViewContext<'_, Self>) -> Element {
        let r = self.import.as_ref().unwrap();
        let mut parts = column().gap(6.).fill_width().max_height(180.).scroll_y();
        for (i, p) in r.project.parts.iter().enumerate() {
            parts = parts.child(
                button(format!(
                    "{} #{} {} [g{}]",
                    if i == r.selected_part { ">" } else { "" },
                    p.id,
                    p.name.chars().take(15).collect::<String>(),
                    p.group
                ))
                .key(format!("part-{i}"))
                .on_click(cx.listener(move |s, _, _| {
                    let r = s.review();
                    r.select_part(i);
                })),
            );
        }
        let mut groups = column().gap(6.).fill_width().max_height(180.).scroll_y();
        for g in &r.project.groups {
            let id = g.id;
            groups =
                groups.child(
                    column()
                        .fill_width()
                        .gap(4.)
                        .child(
                            button(format!(
                                "{} {}: {}",
                                if id == r.selected_group { ">" } else { "" },
                                id,
                                g.name.chars().take(24).collect::<String>()
                            ))
                            .key(format!("group-{id}"))
                            .fill_width()
                            .on_click(cx.listener(move |s, _, _| s.review().select(id))),
                        )
                        .child(
                            row()
                                .gap(6.)
                                .child(button("Assign part").key(format!("assign-{id}")).on_click(
                                    cx.listener(move |s, _, _| {
                                        let r = s.review();
                                        r.project.parts[r.selected_part].group = id;
                                        r.select(id);
                                    }),
                                ))
                                .child(button("Parent").disabled(id == r.selected_group).on_click(
                                    cx.listener(move |s, _, _| {
                                        let r = s.review();
                                        r.edit_group(|g| {
                                            if let Some(j) = &mut g.joint {
                                                j.parent = id;
                                                j.reviewed = false;
                                            } else {
                                                g.joint = Some(Joint {
                                                    parent: id,
                                                    motion: Motion::Fixed,
                                                    pivot_m: g.pose.translation_m,
                                                    axis: [0., 0., 1.],
                                                    limits: None,
                                                    reviewed: false,
                                                    evidence: String::new(),
                                                });
                                            }
                                        });
                                        r.select(r.selected_group);
                                    }),
                                ))
                                .child(button("Merge").disabled(id == r.selected_group).on_click(
                                    cx.listener(move |s, _, _| s.review().merge_into(id)),
                                )),
                        ),
                );
        }
        let group = r
            .project
            .groups
            .iter()
            .find(|g| g.id == r.selected_group)
            .unwrap();
        let preview_limits = group
            .joint
            .as_ref()
            .and_then(|j| j.limits)
            .unwrap_or([-1., 1.]);
        let mass = match r.project.mass(group) {
            Ok((m, _)) => format!(
                "Mass {:.6} kg\nCOM [{:.4}, {:.4}, {:.4}] m\nTensor kg m²:\n{:?}\n{:?}\n{:?}",
                m.mass_kg,
                m.com_m[0],
                m.com_m[1],
                m.com_m[2],
                m.inertia_kg_m2[0],
                m.inertia_kg_m2[1],
                m.inertia_kg_m2[2]
            ),
            Err(e) => format!("Mass unavailable: {e}"),
        };
        let controls=column().width(350.).fill_height().scroll_y().padding(10.).gap(10.)
            .child(label("SOURCE OCCURRENCES").font_size(14.))
            .child(button("Omit missing references").on_click(cx.listener(|s,_,_| {let r=s.review();let n=r.project.omit_missing_references();r.status=format!("Omitted {n} missing occurrences; no dependent relations were imported.");r.revision+=1;}))).child(parts)
            .child(button("New moving group from selected part").key("new-group").on_click(cx.listener(|s,_,_|s.review().new_group())))
            .child(label("RIGID GROUPS").font_size(14.)).child(groups)
            .child(button("Remove empty groups").on_click(cx.listener(|s,_,_|s.review().remove_empty())))
            .child(label("Empty groups block export; assign at least one part to each group."))
            .child(label(format!("Selected group {} · {:?}",group.id,group.proxy)))
            .child(row().gap(4.)
                .child(button("Part boxes").on_click(cx.listener(|s,_,_|s.review().edit_group(|g|g.proxy=Proxy::PerPartBoxes))))
                .child(button("Box").on_click(cx.listener(|s,_,_|s.review().edit_group(|g|g.proxy=Proxy::Box))))
                .child(button("Sphere").on_click(cx.listener(|s,_,_|s.review().edit_group(|g|g.proxy=Proxy::Sphere))))
                .child(button("Hull").on_click(cx.listener(|s,_,_|s.review().edit_group(|g|g.proxy=Proxy::Hull)))))
            .child(button(if r.proxies {"Show source surfaces"}else{"Show collision proxies"}).key("proxy-view").on_click(cx.listener(|s,_,_|{let r=s.review();r.proxies = !r.proxies;r.revision+=1;})))
            .child(button(if group.fixed {"Body: fixed (make dynamic)"}else{"Body: dynamic (make fixed)"}).on_click(cx.listener(|s,_,_|s.review().edit_group(|g|g.fixed = !g.fixed))))
            .child(label("MASS BASIS — independent of collision proxy"))
            .child(button("Review cached SI; off-diagonals = −products").on_click(cx.listener(|s,_,_|s.review().edit_group(|g|g.mass_mode=MassMode::ReviewedCache {products_sign:-1.}))))
            .child(button("Review cached SI; off-diagonals = +products").on_click(cx.listener(|s,_,_|s.review().edit_group(|g|g.mass_mode=MassMode::ReviewedCache {products_sign:1.}))))
            .child(label("Cached values are user-reviewed, not independently verified.\nAlternative: estimate solid part boxes at density kg/m³:"))
            .child(text_input(r.density.clone()).key("density").fill_width().on_change(cx.listener(|s,e:&TextChangeEvent,_|s.review().density=e.value.clone())))
            .child(button("Use explicit box-density estimate").on_click(cx.listener(|s,_,_|{let r=s.review();match r.density.parse::<f64>() {Ok(v) if v.is_finite() && v>0.=>r.edit_group(|g|g.mass_mode=MassMode::EstimatedBoxDensity {density_kg_m3:v}),_=>r.status="Density must be finite and positive".into()}})))
            .child(label(format!("EXPLICIT PART #{} OVERRIDE (SI)",r.project.parts[r.selected_part].id)))
            .child(text_input(r.explicit_mass.clone()).key("part-mass").fill_width().accessibility_label("Selected part mass kg").on_change(cx.listener(|s,e:&TextChangeEvent,_|s.review().explicit_mass=e.value.clone())))
            .child(text_input(r.explicit_com.clone()).key("part-com").fill_width().accessibility_label("Part-local COM x,y,z metres").on_change(cx.listener(|s,e:&TextChangeEvent,_|s.review().explicit_com=e.value.clone())))
            .child(text_input(r.explicit_tensor.clone()).key("part-tensor").fill_width().accessibility_label("Central tensor xx,yy,zz,xy,xz,yz kg m2").on_change(cx.listener(|s,e:&TextChangeEvent,_|s.review().explicit_tensor=e.value.clone())))
            .child(button("Apply explicit part mass / COM / tensor").on_click(cx.listener(|s,_,_| {let r=s.review();let result=(|| {let mass=r.explicit_mass.trim().parse::<f64>().map_err(|_|"invalid mass".to_owned())?;let com=triple(&r.explicit_com)?;let t=numbers(&r.explicit_tensor)?;if t.len()!=6 {return Err("tensor requires xx,yy,zz,xy,xz,yz (actual off-diagonals)".into());}r.project.set_mass_override(r.selected_part,Mass {mass_kg:mass,com_m:com,inertia_kg_m2:[[t[0],t[3],t[4]],[t[3],t[1],t[5]],[t[4],t[5],t[2]]]})})();r.status=match result {Ok(())=>"Explicit part-local physical properties applied".into(),Err(e)=>e};})))
            .child(label(mass))
            .child(label("Joint preview coordinate (kinematic, not physics)"))
            .child(slider(r.preview as f32).range(preview_limits[0] as f32,preview_limits[1] as f32).disabled(group.joint.as_ref().is_none_or(|j|matches!(j.motion,Motion::Fixed)))
                .on_change(cx.listener(|s,e:&SliderEvent,_| {let r=s.review();r.preview=f64::from(e.value);r.revision+=1;})))
            .child(label("PARENT JOINT — source/world coordinates; zero retains CAD pose"))
            .child(label(format!("Parent: {:?}; type/limits require explicit review",group.joint.as_ref().map(|j|j.parent))))
            .child(text_input(r.pivot.clone()).key("pivot").fill_width().accessibility_label("Joint pivot metres").on_change(cx.listener(|s,e:&TextChangeEvent,_|s.review().pivot=e.value.clone())))
            .child(text_input(r.axis.clone()).key("axis").fill_width().accessibility_label("Joint axis").on_change(cx.listener(|s,e:&TextChangeEvent,_|s.review().axis=e.value.clone())))
            .child(text_input(r.limits.clone()).key("limits").fill_width().accessibility_label("Joint limits radians or metres").on_change(cx.listener(|s,e:&TextChangeEvent,_|s.review().limits=e.value.clone())))
            .child(row().gap(6.)
                .child(button("Fixed").on_click(cx.listener(|s,_,_|s.review().joint(Motion::Fixed))))
                .child(button("Hinge").on_click(cx.listener(|s,_,_|s.review().joint(Motion::Revolute))))
                .child(button("Slider").on_click(cx.listener(|s,_,_|s.review().joint(Motion::Prismatic)))))
            .child(label("General SolidWorks mate solving is unavailable.\nParent buttons select a group; overrides use the selected Part frame."));
        let mut scene = column()
            .flex_grow(1.)
            .flex_basis(0.)
            .min_width(0.)
            .min_height(0.)
            .fill_height()
            .gap(8.)
            .child(
                row()
                    .gap(8.)
                    .child(button("Orbit left").on_click(cx.listener(|s, _, _| s.yaw -= 0.2)))
                    .child(button("Orbit right").on_click(cx.listener(|s, _, _| s.yaw += 0.2))),
            )
            .child(label(if r.proxies {
                "Orange: selected group · Collision proxies (approximations)"
            } else {
                "Orange: selected rigid group · Teal: other groups · Native display geometry"
            }));
        if let Some(image) = &self.image {
            scene = scene.child(
                rxui::image(image.clone())
                    .key("viewport")
                    .fill_width()
                    .flex_grow(1.)
                    .flex_basis(0.)
                    .min_width(0.)
                    .min_height(0.)
                    .fit(ImageFit::Contain),
            );
        }
        column()
            .fill_width()
            .fill_height()
            .padding(12.)
            .gap(8.)
            .child(label("SolidWorks → reviewed physical model").font_size(22.))
            .child(label(format!(
                "{} parts · {} meshes · {} groups · {} diagnostics · {} omitted · {}",
                r.project.parts.len(),
                if r.project.meshes.is_empty() {
                    r.project.parts.len()
                } else {
                    r.project.meshes.len()
                },
                r.project.groups.len(),
                r.project.diagnostics.len(),
                r.project.omitted_occurrences.len(),
                r.project
                    .source
                    .rsplit('/')
                    .next()
                    .unwrap_or(&r.project.source)
            )))
            .child(
                row()
                    .gap(8.)
                    .child(
                        button(if r.project.source_frames_and_geometry_reviewed {
                            "Source frames/geometry reviewed"
                        } else {
                            "Acknowledge source frames + configuration geometry review"
                        })
                        .on_click(cx.listener(|s, _, _| {
                            let r = s.review();
                            r.project.source_frames_and_geometry_reviewed =
                                !r.project.source_frames_and_geometry_reviewed;
                        })),
                    )
                    .child(button("Save review").on_click(cx.listener(|s, _, _| {
                        let r = s.review();
                        r.status = match r.project.save(&r.save) {
                            Ok(()) => format!("Saved {:?}", r.save),
                            Err(e) => e,
                        };
                    })))
                    .child(
                        button("Validate + export recipe").on_click(cx.listener(|s, _, _| {
                            let r = s.review();
                            r.status = match r.project.export(&r.export) {
                                Ok(()) => {
                                    format!("Cooked/installed successfully; saved {:?}", r.export)
                                }
                                Err(e) => e,
                            };
                        })),
                    ),
            )
            .child(
                row()
                    .fill_width()
                    .flex_grow(1.)
                    .flex_basis(0.)
                    .min_height(0.)
                    .gap(12.)
                    .child(controls)
                    .child(scene),
            )
            .child(label(self.error.as_ref().unwrap_or(&r.status).clone()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn grouping_preserves_occurrence_poses_and_requires_explicit_joint_review() {
        let project = super::super::importer::tests::fixture();
        let source_poses: Vec<_> = project.parts.iter().map(|p| p.pose).collect();
        let mut review = Review::new(project, "unused.review.json".into());
        review.new_group();
        assert_eq!(review.selected_group, 2);
        assert_eq!(review.project.parts[0].group, 2);
        assert_eq!(
            review
                .project
                .parts
                .iter()
                .map(|p| p.pose)
                .collect::<Vec<_>>(),
            source_poses
        );
        assert!(review.project.check_runtime().is_err());
        review.joint(Motion::Revolute);
        review.project.check_runtime().unwrap();
        assert_eq!(
            review
                .project
                .parts
                .iter()
                .map(|p| p.pose)
                .collect::<Vec<_>>(),
            source_poses
        );
    }

    #[test]
    fn bulk_merge_and_empty_cleanup_preserve_poses_and_do_not_orphan_children() {
        let project = super::super::importer::tests::shared_fixture();
        let poses: Vec<_> = project.parts.iter().map(|p| p.pose).collect();
        let mut r = Review::new(project, "unused.json".into());
        r.new_group();
        r.select(1);
        r.merge_into(2);
        assert_eq!(r.project.groups.len(), 2);
        r.select(2);
        r.merge_into(1);
        assert_eq!(r.project.groups.len(), 1);
        assert!(r.project.parts.iter().all(|p| p.group == 1));
        assert_eq!(
            r.project.parts.iter().map(|p| p.pose).collect::<Vec<_>>(),
            poses
        );
        r.new_group();
        r.project.parts[0].group = 1;
        r.remove_empty();
        assert_eq!(r.project.groups.len(), 1);
        assert_eq!(r.selected_group, 1);
        r.project.check_runtime().unwrap();
    }
}
