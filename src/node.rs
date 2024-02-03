use crate::{Axes, Constraint, Vec2};

#[derive(Clone, Default)]
pub struct Node {
    pub p: Vec2,
    pub v: Vec2,
    pub constraint_len: u32,
    pub constraints: [Constraint; 32],
}

impl Node {
    pub fn new_ghost(p: Vec2, constraint: Constraint) -> Self {
        let mut slf = Self {
            p,
            v: Vec2::ZERO,
            constraint_len: 1,
            constraints: Default::default(),
        };
        slf.constraints[0] = constraint;
        slf
    }

    pub fn add_constraint(&mut self, constraint: Constraint) {
        match &constraint {
            Constraint::None => {
                return;
            }
            Constraint::Freeze { axes, .. } => {
                if !(axes.contains(Axes::X) || axes.contains(Axes::Y)) {
                    return;
                }
            }
            Constraint::Link {
                node: _, distance, ..
            }
            | Constraint::Rope {
                node: _, distance, ..
            } => {
                if *distance <= 0.0 {
                    return;
                }
            }
            Constraint::Hydraulic {
                node: _, distance, ..
            } => {
                if *distance <= 0.0 {
                    return;
                }
            }
            Constraint::Spring {
                node: _,
                distance,
                stiffness,
            } => {
                if *distance <= 0.0 || *stiffness == 0.0 {
                    return;
                }
            }
            Constraint::Range { axes, start, end } => {
                if start >= end || !(axes.contains(Axes::X) || axes.contains(Axes::Y)) {
                    return;
                }
            }
            Constraint::Rotor { speed } => {
                if *speed == 0.0 {
                    return;
                }
            }
        }
        self.constraints[self.constraint_len as usize] = constraint;
        self.constraint_len = (self.constraint_len + 1) % self.constraints.len() as u32;
    }

    pub fn remove_constraint(&mut self, constraint_idx: u32) {
        if self.constraint_len == 0 {
            return;
        }
        let p = self.constraints.as_mut_ptr();
        std::mem::swap(unsafe { &mut *p.add(constraint_idx as usize) }, unsafe {
            &mut *p.add(self.constraint_len as usize - 1)
        });
        self.constraints[self.constraint_len as usize - 1] = Constraint::None;
        self.constraint_len -= 1;
    }

    pub fn unlinked(&self) -> bool {
        let mut unlinked = true;
        for i in 0..self.constraint_len {
            let c = &self.constraints[i as usize];
            match *c {
                Constraint::None | Constraint::Range { .. } => {}
                _ => {
                    unlinked = false;
                    break;
                }
            }
        }
        unlinked
    }

    pub fn move_by(&mut self, x: f32, y: f32) {
        self.p += Vec2::new(x, y);
        for i in 0..self.constraint_len {
            let c = &mut self.constraints[i as usize];
            if let Constraint::Freeze { axes, x: fx, y: fy } = c {
                if axes.contains(Axes::X) {
                    *fx += x;
                }
                if axes.contains(Axes::Y) {
                    *fy += y;
                }
            }
        }
    }

    pub fn kinetic_energy(&self) -> f32 {
        0.5 * self.v.len2()
    }

    // Iterates over node's constraints
    pub fn iter(&self) -> std::iter::Take<std::slice::Iter<'_, Constraint>> {
        self.constraints.iter().take(self.constraint_len as usize)
    }

    // Iterates mutably over node's constraints
    pub fn iter_mut(&mut self) -> std::iter::Take<std::slice::IterMut<'_, Constraint>> {
        self.constraints
            .iter_mut()
            .take(self.constraint_len as usize)
    }

    pub fn iter_link_nodes(&self) -> impl Iterator<Item = &'_ u32> {
        self.iter().filter_map(|c| c.link_node())
    }

    pub fn iter_mut_link_nodes(&mut self) -> impl Iterator<Item = &'_ mut u32> {
        self.iter_mut().filter_map(|c| c.link_node_mut())
    }

    pub fn iter_link_nodes_dist(&self) -> impl Iterator<Item = (&'_ u32, &'_ f32)> {
        self.iter().filter_map(|c| c.link_node_dist())
    }

    pub fn iter_mut_link_nodes_dist(&mut self) -> impl Iterator<Item = (&'_ mut u32, &'_ mut f32)> {
        self.iter_mut().filter_map(|c| c.link_node_dist_mut())
    }
}
