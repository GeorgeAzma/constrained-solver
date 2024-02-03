use crate::{app::renderer::Renderer, Axes, Constraint, HashGrid, Integrator, Node, Vec2};
use std::{
    collections::{HashMap, HashSet},
    io::{self, Read, Write},
};

// TODO: Add fast linked nodes query, currently have to iterate over all nodes
pub struct World {
    pub nodes: Vec<Node>,
    pub radius: f32,
    pub dt: f32,
    pub remove_queue: Vec<u32>,
    pub energy: f32,
}

impl Default for World {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            radius: 0.05,
            dt: 0.0,
            remove_queue: Vec::new(),
            energy: 0.0,
        }
    }
}

impl World {
    pub fn add_node(&mut self, x: f32, y: f32) -> u32 {
        self.nodes.push(Node {
            p: Vec2::new(x, y),
            v: Vec2::ZERO,
            constraint_len: 0,
            constraints: Default::default(),
        });
        (self.nodes.len() - 1) as u32
    }

    pub fn select_nodes(&mut self, nodes: &Vec<u32>) -> Vec<Node> {
        let mut dest_nodes = Vec::new();
        for i in nodes {
            let mut n = self.nodes[*i as usize].clone();
            let mut constraints_to_remove = Vec::new();
            for (i, c) in n.iter_mut().enumerate() {
                if let Some(node) = c.link_node_mut() {
                    if let Some(found_node) = nodes.iter().position(|n| *n == *node) {
                        *node = found_node as u32;
                    } else {
                        constraints_to_remove.push(i);
                    }
                }
            }
            for i in constraints_to_remove {
                n.remove_constraint(i as u32);
            }
            dest_nodes.push(n);
        }
        dest_nodes
    }

    pub fn copy_nodes_as_vec(&mut self, nodes: &Vec<u32>, x: f32, y: f32) -> Vec<Node> {
        let mut dest_nodes = self.select_nodes(nodes);
        let mut min = Vec2::splat(f32::MAX);
        for n in dest_nodes.iter() {
            min = min.min(&n.p);
        }
        for n in dest_nodes.iter_mut() {
            n.move_by(x - min.x, y - min.y);
        }
        dest_nodes
    }

    pub fn copy_nodes(&mut self, nodes: &Vec<u32>, x: f32, y: f32) {
        let mut copied_nodes = self.copy_nodes_as_vec(nodes, x, y);

        // TODO:
        // let points: Vec<(f32, f32)> = self.nodes.iter().map(|n| (n.p.x, n.p.y)).collect();
        // let hash_grid = HashGrid::new(&points, self.radius);

        // Remap all intersecting copied node indices to intersected world node indices
        // let mut intersecting_copied_nodes = HashMap::new();
        // for (i, a) in copied_nodes.iter().enumerate() {
        //     let collisions = hash_grid.find(a.p.x, a.p.y);
        //     for bi in collisions {
        //         let b = &self.nodes[bi as usize];
        //         let dist = a.p.dist(&b.p);
        //         if dist < self.radius * 2.0 {
        //             let prev_bi = intersecting_copied_nodes.entry(i).or_insert(bi);
        //             let prev_dist = a.p.dist(&self.nodes[*prev_bi as usize].p);
        //             if dist < prev_dist {
        //                 *prev_bi = bi;
        //             }
        //         }
        //     }
        // }

        // Iterate over all copied node links and change their destination/distance to world node if link is intersecting world node
        // If current copied node is intersecting transfer its links to intersected world node

        // If copied node is not intersecting then update its link node indices
        // Based on how many copied nodes where removed before it
        // (Copied nodes get removed if they intersect)
        // Unless link node is intersecting in which case
        // update world node's link indices where intersecting copied node got transfered

        // let len = self.nodes.len() as u32;
        // for copied_node in copied_nodes.iter_mut() {
        //     let copied_node_p = copied_node.p;
        //     for (node, distance) in copied_node.iter_mut_link_nodes_dist() {
        //         if let Some(world_node_idx) = intersecting_copied_nodes.get(&(*node as usize)) {
        //             *node = *world_node_idx;
        //             *distance = copied_node_p.dist(&self.nodes[*world_node_idx as usize].p);
        //         } else {
        //             *node += len;
        //         }
        //     }
        // }

        // let mut removed_before_me = 0;
        // for (i, copied_node) in copied_nodes.iter_mut().enumerate() {
        //     if let Some(world_node_idx) = intersecting_copied_nodes.get(&i) {
        //         let world_node = &mut self.nodes[*world_node_idx as usize];
        //         for c in copied_node.iter() {
        //             if c.link_node().is_some() {
        //                 world_node.add_constraint(c.clone());
        //             }
        //         }
        //         removed_before_me += 1;
        //     }
        //     for c in copied_node.iter() {
        //         if c.link_node().is_some() {
        //             world_node.add_constraint(c.clone());
        //         }
        //     }
        // }

        // let mut copied_idx = 0;
        // copied_nodes.retain(|_| {
        //     let retain = !intersecting_copied_nodes.contains_key(&copied_idx);
        //     copied_idx += 1;
        //     retain
        // });

        for n in copied_nodes.iter_mut() {
            for node in n.iter_mut_link_nodes() {
                *node += self.nodes.len() as u32;
            }
        }

        self.nodes.append(&mut copied_nodes);
    }

    pub fn link_node(&mut self, node1: u32, node2: u32) {
        if self.linked_to_node(node1, node2).is_some() {
            return;
        }
        let distance = self.nodes[node1 as usize]
            .p
            .dist(&self.nodes[node2 as usize].p);
        self.nodes[node1 as usize].add_constraint(Constraint::Link {
            node: node2,
            distance,
        });
    }

    pub fn hydraulic_node(&mut self, node1: u32, node2: u32, speed: f32) {
        if self.linked_to_node(node1, node2).is_some() {
            return;
        }
        let distance = self.nodes[node1 as usize]
            .p
            .dist(&self.nodes[node2 as usize].p);
        self.nodes[node1 as usize].add_constraint(Constraint::Hydraulic {
            node: node2,
            distance,
            speed,
        });
    }

    pub fn spring_node(&mut self, node1: u32, node2: u32, stiffness: f32) {
        if self.linked_to_node(node1, node2).is_some() {
            return;
        }
        let distance = self.nodes[node1 as usize]
            .p
            .dist(&self.nodes[node2 as usize].p);
        self.nodes[node1 as usize].add_constraint(Constraint::Spring {
            node: node2,
            distance,
            stiffness,
        });
    }

    pub fn rope_node(&mut self, node1: u32, node2: u32) {
        if self.linked_to_node(node1, node2).is_some() {
            return;
        }
        let distance = self.nodes[node1 as usize]
            .p
            .dist(&self.nodes[node2 as usize].p);
        self.nodes[node1 as usize].add_constraint(Constraint::Rope {
            node: node2,
            distance,
        });
    }

    pub fn linked_to_node(&self, node1: u32, node2: u32) -> Option<u32> {
        let n = &self.nodes[node1 as usize];
        for (i, c) in n.iter().enumerate() {
            if let Some(node) = c.link_node() {
                if *node == node2 {
                    return Some(i as u32);
                }
            }
        }
        None
    }

    fn unlink_node(&mut self, node1: u32, node2: u32) {
        let mut unlink = |node1: u32, node2: u32| {
            if let Some(linked_constraint) = self.linked_to_node(node1, node2) {
                self.nodes[node1 as usize].remove_constraint(linked_constraint);
            }
        };
        unlink(node1, node2);
        unlink(node2, node1);
    }

    pub fn node_linked(&self, node_idx: u32) -> bool {
        if self.nodes[node_idx as usize].unlinked() {
            for j in 0..self.nodes.len() as u32 {
                if self.linked_to_node(j, node_idx).is_some() {
                    return true;
                }
            }
            return false;
        }
        true
    }

    fn swap_node(&mut self, node_idx: u32, swap_idx: u32) {
        if node_idx == swap_idx {
            return;
        }

        for i in 0..self.nodes.len() {
            if let Some(linked_constraint) = self.linked_to_node(i as u32, node_idx) {
                if let Some(node) =
                    self.nodes[i].constraints[linked_constraint as usize].link_node_mut()
                {
                    *node = swap_idx;
                }
            }
        }
    }

    pub fn remove_node(&mut self, node_idx: u32) {
        self.remove_queue.push(node_idx);
        for n_idx in 0..self.nodes.len() as u32 {
            if n_idx == node_idx {
                continue;
            }
            self.unlink_node(node_idx, n_idx);
            if !self.node_linked(n_idx) {
                self.remove_queue.push(n_idx);
            }
        }
    }

    pub fn move_node(&mut self, node_idx: u32, x: f32, y: f32, update_constraints: bool) {
        let a = unsafe { &mut *self.nodes.as_mut_ptr().add(node_idx as usize) };
        a.move_by(x, y);
        a.v = Vec2::ZERO;
        let a_p = a.p;
        if update_constraints {
            for (node, distance) in a.iter_mut_link_nodes_dist() {
                *distance = self.nodes[*node as usize].p.dist(&a_p);
            }
        }

        if update_constraints {
            for b in self.nodes.iter_mut() {
                let b_p = b.p;
                for (node, distance) in b.iter_mut_link_nodes_dist() {
                    if *node == node_idx {
                        *distance = b_p.dist(&a.p);
                    }
                }
            }
        }
    }

    pub fn scale(&self) -> f32 {
        return self.radius * 20.0;
    }

    pub fn link_width(&self) -> f32 {
        return self.radius * 0.65;
    }

    pub fn move_all(&mut self, x: f32, y: f32) {
        for i in 0..self.nodes.len() {
            let old_v = self.nodes[i].v;
            self.move_node(i as u32, x, y, false);
            self.nodes[i].v = old_v;
        }
    }

    pub fn set_scale(&mut self, scale: f32) {
        self.radius = scale * 0.05;
    }

    pub fn remove_link(&mut self, node_idx: u32, link_idx: u32) {
        let c = &self.nodes[node_idx as usize].constraints[link_idx as usize];
        if let Some(&node) = c.link_node() {
            self.unlink_node(node_idx, node);
            if !self.node_linked(node_idx) {
                self.remove_node(node_idx);
            }
            if !self.node_linked(node) {
                self.remove_node(node);
            }
        }
    }

    pub fn point_inside_node(&mut self, x: f32, y: f32) -> Option<u32> {
        let mut n_idx = u32::MAX;
        let p = Vec2::new(x, y);
        let mut min = f32::MAX;
        for (i, n) in self.nodes.iter().enumerate() {
            let d = n.p.dist(&p);
            if d < min {
                n_idx = i as u32;
                min = d;
            }
        }

        if n_idx < self.nodes.len() as u32 {
            if min <= self.radius / self.scale() {
                return Some(n_idx);
            }
        }

        None
    }

    pub fn point_inside_link(&self, x: f32, y: f32) -> Option<(u32, u32)> {
        let p = Vec2::new(x, y);
        let line_sdf = |a: Vec2, b: Vec2| {
            let pa = p - a;
            let ba = b - a;
            let h = (pa.dot(&ba) / ba.len2()).clamp(0.0, 1.0);
            return (pa - ba * h).len();
        };
        for (i, n) in self.nodes.iter().enumerate() {
            let a = n.p;
            for j in 0..n.constraint_len {
                if let Some(node) = n.constraints[j as usize].link_node() {
                    let b = self.nodes[*node as usize].p;
                    if line_sdf(a, b) <= self.link_width() / self.scale() {
                        return Some((i as u32, j));
                    }
                }
            }
        }
        None
    }

    // TODO:
    pub fn clear_rect(&self, min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Option<(u32, u32)> {
        let line_rect_intersection = |x1: f32,
                                      y1: f32,
                                      x2: f32,
                                      y2: f32,
                                      min_x: f32,
                                      min_y: f32,
                                      max_x: f32,
                                      max_y: f32|
         -> bool {
            if (min_x <= x1 && x1 <= max_x && min_y <= y1 && y1 <= max_y)
                || (min_x <= x2 && x2 <= max_x && min_y <= y2 && y2 <= max_y)
            {
                true
            } else if (x1 < min_x && x2 < min_x)
                || (x1 > max_x && x2 > max_x)
                || (y1 < min_y && y2 < min_y)
                || (y1 > max_y && y2 > max_y)
            {
                false
            } else {
                false
            }
        };

        for (i, n) in self.nodes.iter().enumerate() {
            let a = n.p;
            if a.x >= min_x && a.y >= min_y && a.x <= max_x && a.y <= max_y {}
            for j in 0..n.constraint_len {
                if let Some(node) = n.constraints[j as usize].link_node() {
                    let b = self.nodes[*node as usize].p;
                    if line_rect_intersection(a.x, a.y, b.x, b.y, min_x, min_y, max_x, max_y) {
                        return Some((i as u32, j));
                    }
                }
            }
        }
        None
    }

    pub fn update(&mut self, integrator: &mut impl Integrator, dt: f32, steps: u32) {
        self.dt = dt / steps as f32;

        if self.dt != 0.0 {
            for _ in 0..steps {
                integrator.solve(self);
            }

            self.energy = 0.0;
            for n in self.nodes.iter() {
                self.energy += n.kinetic_energy();
            }
        }
    }

    pub fn flush(&mut self) {
        let mut unique_set = HashSet::new();
        self.remove_queue.retain(|e| unique_set.insert(*e));
        let mut idx_map = std::collections::HashMap::new();
        for idx in self.remove_queue.iter() {
            let idx = idx_map.remove(idx).unwrap_or(*idx);
            self.nodes.swap_remove(idx as usize);
            if idx != self.nodes.len() as u32 {
                idx_map.insert(self.nodes.len() as u32, idx);
            }
        }
        self.remove_queue.clear();
        for (idx, swap_idx) in idx_map {
            self.swap_node(idx, swap_idx);
        }
    }

    pub fn step(&mut self) {
        let points: Vec<(f32, f32)> = self.nodes.iter().map(|n| (n.p.x, n.p.y)).collect();
        let hash_grid = HashGrid::new(&points, self.radius);

        for n in self.nodes.iter_mut() {
            n.v.y -= 6.0 * self.dt; // Gravity
        }
        for i in 0..self.nodes.len() {
            let nodes = self.nodes.as_mut_ptr();
            let a = unsafe { &mut *nodes.add(i as usize) };
            let collisions = hash_grid.find(a.p.x, a.p.y);
            for bi in collisions {
                if bi == i as u32 {
                    continue;
                }
                let b = unsafe { nodes.add(bi as usize).as_mut().unwrap() };
                let dist = a.p.dist(&b.p);
                if dist < self.radius * 2.0 {
                    // Resolution
                    let to_b = b.p - a.p;
                    let push = to_b * (dist - self.radius * 2.0) * 0.5;
                    a.p += push;
                    b.p -= push;

                    // Linear impulse
                    let impulse_mag = to_b * (a.v - b.v).dot(&to_b) / (dist * dist);
                    a.v -= impulse_mag;
                    b.v += impulse_mag;
                }
            }

            let mut rotor_speed = 0.0;
            for c in a.iter() {
                if let Constraint::Rotor { speed } = c {
                    rotor_speed = *speed;
                    break;
                }
            }

            const LINK_STIFFNESS: f32 = 32.0;
            for c in 0..a.constraint_len {
                match &mut a.constraints[c as usize] {
                    Constraint::None => {}
                    Constraint::Freeze { axes, x, y } => {
                        if axes.contains(Axes::X) {
                            a.p.x = *x;
                            a.v.x = 0.0;
                        }
                        if axes.contains(Axes::Y) {
                            a.p.y = *y;
                            a.v.y = 0.0;
                        }
                    }
                    Constraint::Link { node, distance } => {
                        let b = unsafe { nodes.add(*node as usize).as_mut().unwrap() };
                        let dist = a.p.dist(&b.p);
                        let d = (a.p - b.p) / dist * (*distance - dist);
                        a.v += d * LINK_STIFFNESS;
                        b.v -= d * LINK_STIFFNESS;
                        a.p += d * 0.5;
                        b.p -= d * 0.5;
                        if rotor_speed > 0.0 {
                            let c = (b.p - a.p).norm().rot90();
                            b.v += c * rotor_speed * dist * self.dt * 64.0;
                        }
                    }
                    Constraint::Hydraulic {
                        node,
                        distance,
                        speed,
                    } => {
                        *distance += *speed * self.dt;
                        let b = unsafe { nodes.add(*node as usize).as_mut().unwrap() };
                        let dist = a.p.dist(&b.p);
                        let d = (a.p - b.p) / dist * (*distance - dist);
                        a.v += d * LINK_STIFFNESS;
                        b.v -= d * LINK_STIFFNESS;
                        a.p += d * 0.5;
                        b.p -= d * 0.5;
                        if rotor_speed > 0.0 {
                            let c = (b.p - a.p).norm().rot90();
                            b.v += c * rotor_speed * dist * self.dt * 64.0;
                        }
                    }
                    Constraint::Spring {
                        node,
                        distance,
                        stiffness,
                    } => {
                        let b = unsafe { nodes.add(*node as usize).as_mut().unwrap() };
                        let dist = a.p.dist(&b.p);
                        let n = (a.p - b.p) / dist;
                        let push = *distance - dist;
                        a.p += n * push.abs().sqrt() * push.signum() * *stiffness * 8.0 * self.dt;
                        b.p -= n * push.abs().sqrt() * push.signum() * *stiffness * 8.0 * self.dt;
                        a.v += n * push * *stiffness * 512.0 * self.dt;
                        b.v -= n * push * *stiffness * 512.0 * self.dt;
                        if rotor_speed > 0.0 {
                            let c = (b.p - a.p).norm().rot90();
                            b.v += c * rotor_speed * dist * self.dt * 64.0;
                        }
                    }
                    Constraint::Range { axes, start, end } => {
                        if axes.contains(Axes::X) {
                            let clamped_x = a.p.x.clamp(*start, *end);
                            if clamped_x != a.p.x {
                                a.v.x = (clamped_x - a.p.x).signum() * a.v.x.abs();
                                a.p.x = clamped_x;
                            }
                        }
                        if axes.contains(Axes::Y) {
                            let clamped_y = a.p.y.clamp(*start, *end);
                            if clamped_y != a.p.y {
                                a.v.y = (clamped_y - a.p.y).signum() * a.v.y.abs();
                                a.p.y = clamped_y;
                            }
                        }
                    }
                    Constraint::Rotor { speed: _ } => {}
                    Constraint::Rope { node, distance } => {
                        let b = unsafe { nodes.add(*node as usize).as_mut().unwrap() };
                        let dist = a.p.dist(&b.p);
                        let d = (a.p - b.p) / dist * (*distance - dist);
                        if dist > *distance {
                            a.v += d * LINK_STIFFNESS * 0.5; // Ropes are slightly less stiff
                            b.v -= d * LINK_STIFFNESS * 0.5;
                            a.p += d * 0.5;
                            b.p -= d * 0.5;
                            if rotor_speed > 0.0 {
                                let c = (b.p - a.p).norm().rot90();
                                b.v += c * rotor_speed * dist * self.dt * 64.0;
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn render_node(&self, node: &Node, gfx: &mut Renderer) {
        let old_col = gfx.color;
        let old_stroke_col = gfx.stroke_color;
        let old_stroke_width = gfx.stroke_width;
        let Vec2 { x, y } = node.p;
        let mut color = [240, 200, 64];
        let mut is_rotor = false;
        let mut freeze_axes = None;
        for i in 0..node.constraint_len {
            let c = &node.constraints[i as usize];
            match c {
                Constraint::Freeze { axes, .. } => {
                    color = [64, 180, 255];
                    freeze_axes = Some(axes);
                    break;
                }
                Constraint::Rope { .. } => {
                    color = [160, 130, 100];
                    break;
                }
                Constraint::Spring { .. } => {
                    color = [255, 255, 128];
                    break;
                }
                Constraint::Hydraulic { .. } => {
                    color = [32, 72, 180];
                    break;
                }
                Constraint::Rotor { .. } => {
                    color = [180, 255, 64];
                    is_rotor = true;
                    break;
                }
                _ => {}
            }
        }

        gfx.color = [
            (gfx.color[0] as f32 * color[0] as f32 / 255.0) as u8,
            (gfx.color[1] as f32 * color[1] as f32 / 255.0) as u8,
            (gfx.color[2] as f32 * color[2] as f32 / 255.0) as u8,
            gfx.color[3],
        ];

        gfx.stroke_color = [
            (gfx.color[0] as f32 * 0.7) as u8,
            (gfx.color[1] as f32 * 0.7) as u8,
            (gfx.color[2] as f32 * 0.7) as u8,
            gfx.color[3],
        ];
        gfx.stroke_width = 0.08 / self.radius.sqrt();
        gfx.circle(x * self.scale(), y * self.scale(), self.radius);

        if let Some(axes) = freeze_axes {
            gfx.color = gfx.stroke_color;
            if axes.contains(Axes::ALL) {
                gfx.circle(x * self.scale(), y * self.scale(), self.radius * 0.45);
            } else if axes.contains(Axes::X) {
                gfx.line(
                    x * self.scale(),
                    y * self.scale() - self.radius * 0.8,
                    x * self.scale(),
                    y * self.scale() + self.radius * 0.8,
                    self.radius * 0.2,
                );
            } else if axes.contains(Axes::Y) {
                gfx.line(
                    x * self.scale() - self.radius * 0.8,
                    y * self.scale(),
                    x * self.scale() + self.radius * 0.8,
                    y * self.scale(),
                    self.radius * 0.2,
                );
            }
        }
        if is_rotor {
            gfx.color = gfx.stroke_color;
            gfx.line(
                x * self.scale() - self.radius * 0.8,
                y * self.scale(),
                x * self.scale() + self.radius * 0.8,
                y * self.scale(),
                self.radius * 0.2,
            );
            gfx.line(
                x * self.scale(),
                y * self.scale() - self.radius * 0.8,
                x * self.scale(),
                y * self.scale() + self.radius * 0.8,
                self.radius * 0.2,
            );
        }
        gfx.color = old_col;
        gfx.stroke_color = old_stroke_col;
        gfx.stroke_width = old_stroke_width;
    }

    pub fn render_link(&self, a: &Node, link: &Constraint, nodes: &Vec<Node>, gfx: &mut Renderer) {
        let old_col = gfx.color;
        let old_stroke_col = gfx.stroke_color;
        let old_stroke_width = gfx.stroke_width;
        match link {
            Constraint::Link { node, .. } => {
                let b = &nodes[*node as usize];
                let to_a = (a.p - b.p).norm();
                let mut d = 4.0 * ((-to_a).dot(&b.v) + to_a.dot(&a.v));
                d = d.abs().sqrt() * d.signum();
                {
                    let r = 1.0 - d.max(0.0);
                    let b = 1.0 + d.min(0.0);
                    let g = 1.0 - d.max(0.0) + d.min(0.0);
                    gfx.color = [
                        (gfx.color[0] as f32 * r) as u8,
                        (gfx.color[1] as f32 * g) as u8,
                        (gfx.color[2] as f32 * b) as u8,
                        gfx.color[3],
                    ];
                }
                gfx.stroke_color = [
                    gfx.color[0] / 2,
                    gfx.color[1] / 2,
                    gfx.color[2] / 2,
                    gfx.color[3],
                ];
                gfx.stroke_width = 0.5;
                gfx.line(
                    a.p.x * self.scale(),
                    a.p.y * self.scale(),
                    b.p.x * self.scale(),
                    b.p.y * self.scale(),
                    self.link_width(),
                );
            }
            Constraint::Rope { node, distance } => {
                let b = &nodes[*node as usize];
                let to_a = (a.p - b.p).norm();
                let mut d = 4.0 * ((-to_a).dot(&b.v) + to_a.dot(&a.v));
                d = d.abs().sqrt() * d.signum();
                {
                    let r = 1.0 - d.max(0.0);
                    let b = 1.0 + d.min(0.0);
                    let g = 1.0 - d.max(0.0) + d.min(0.0);
                    gfx.color = [
                        (gfx.color[0] as f32 * r) as u8,
                        (gfx.color[1] as f32 * g) as u8,
                        (gfx.color[2] as f32 * b) as u8,
                        gfx.color[3],
                    ];
                }
                gfx.stroke_color = [
                    gfx.color[0] / 3,
                    gfx.color[1] / 3,
                    gfx.color[2] / 3,
                    gfx.color[3],
                ];
                gfx.stroke_width = 0.5;
                gfx.line(
                    a.p.x * self.scale(),
                    a.p.y * self.scale(),
                    b.p.x * self.scale(),
                    b.p.y * self.scale(),
                    self.link_width() * 0.7,
                );

                gfx.color = [
                    gfx.color[0] / 3,
                    gfx.color[1] / 3,
                    gfx.color[2] / 3,
                    gfx.color[3],
                ];
                let to_b = b.p - a.p;
                let c = to_b.norm().rot90() * self.link_width() / self.scale() * 0.45;
                let windings = (distance * 32.0) as u32;
                let inv = 1.0 / windings as f32;
                for i in 0..windings {
                    let d = i as f32 * inv;
                    let p1 = (a.p - c + to_b * d) * self.scale();
                    let p2 = (a.p + c + to_b * (d + inv * 0.5)) * self.scale();
                    gfx.line(p1.x, p1.y, p2.x, p2.y, self.link_width() * 0.25);
                }
            }
            Constraint::Hydraulic { node, .. } => {
                let b = &nodes[*node as usize];
                let to_a = (a.p - b.p).norm();
                let mut d = 4.0 * ((-to_a).dot(&b.v) + to_a.dot(&a.v));
                d = d.abs().sqrt() * d.signum();
                {
                    let r = 1.0 - d.max(0.0);
                    let b = 1.0 + d.min(0.0);
                    let g = 1.0 - d.max(0.0) + d.min(0.0);
                    gfx.color = [
                        (gfx.color[0] as f32 * r) as u8,
                        (gfx.color[1] as f32 * g) as u8,
                        (gfx.color[2] as f32 * b) as u8,
                        gfx.color[3],
                    ];
                }
                gfx.stroke_color = [
                    gfx.color[0] / 3,
                    gfx.color[1] / 3,
                    gfx.color[2] / 3,
                    gfx.color[3],
                ];
                gfx.stroke_width = 0.5;
                gfx.line(
                    a.p.x * self.scale(),
                    a.p.y * self.scale(),
                    b.p.x * self.scale(),
                    b.p.y * self.scale(),
                    self.link_width(),
                );
                gfx.color = [
                    gfx.color[0] / 2,
                    gfx.color[1] / 2,
                    gfx.color[2] / 2,
                    gfx.color[3],
                ];
                let to_b = b.p - (b.p - a.p) * 0.5;
                gfx.line(
                    a.p.x * self.scale(),
                    a.p.y * self.scale(),
                    to_b.x * self.scale(),
                    to_b.y * self.scale(),
                    self.link_width(),
                );
            }
            Constraint::Spring {
                node,
                distance,
                stiffness,
            } => {
                let b = &nodes[*node as usize];
                let to_a = (a.p - b.p).norm();
                let mut d = (-to_a).dot(&b.v) + to_a.dot(&a.v);
                d *= 0.25;
                {
                    let r = 1.0 - d.max(0.0);
                    let b = 1.0 + d.min(0.0);
                    let g = 1.0 - d.max(0.0) + d.min(0.0);
                    gfx.color = [
                        (gfx.color[0] as f32 * r) as u8,
                        (gfx.color[1] as f32 * g) as u8,
                        (gfx.color[2] as f32 * b) as u8,
                        gfx.color[3],
                    ];
                }

                gfx.stroke_color = [
                    gfx.color[0] / 3,
                    gfx.color[1] / 3,
                    gfx.color[2] / 3,
                    gfx.color[3],
                ];
                gfx.stroke_width = 0.5;
                let to_b = b.p - a.p;
                let c = to_b.norm().rot90() * self.link_width() / self.scale();
                let windings = (distance * stiffness * 32.0) as u32;
                let inv = 1.0 / windings as f32;
                for i in 0..windings {
                    let d = i as f32 * inv;
                    let p1 = (a.p - c + to_b * d) * self.scale();
                    let p2 = (a.p + c + to_b * (d + inv)) * self.scale();
                    gfx.line(p1.x, p1.y, p2.x, p2.y, self.link_width() * 0.25);
                    let p2 = p2 - to_b * inv * self.scale();
                    gfx.line(p1.x, p1.y, p2.x, p2.y, self.link_width() * 0.25);
                }
            }
            _ => {}
        }
        gfx.color = old_col;
        gfx.stroke_color = old_stroke_col;
        gfx.stroke_width = old_stroke_width;
    }

    pub fn render_nodes(&self, nodes: &Vec<Node>, gfx: &mut Renderer) {
        for n in nodes.iter() {
            for i in 0..n.constraint_len {
                let c = &n.constraints[i as usize];
                self.render_link(n, c, nodes, gfx);
            }
        }

        for n in nodes.iter() {
            self.render_node(&n, gfx);
        }
    }

    pub fn render(&self, gfx: &mut Renderer) {
        self.render_nodes(&self.nodes, gfx);
    }

    pub fn serealize<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&self.radius.to_le_bytes())?;
        writer.write_all(&(self.nodes.len() as u32).to_le_bytes())?;
        for n in self.nodes.iter() {
            writer.write_all(&n.p.x.to_le_bytes())?;
            writer.write_all(&n.p.y.to_le_bytes())?;
            writer.write_all(&n.v.x.to_le_bytes())?;
            writer.write_all(&n.v.y.to_le_bytes())?;
            writer.write_all(&n.constraint_len.to_le_bytes())?;
            for i in 0..n.constraint_len {
                let c = &n.constraints[i as usize];
                let t: u32 = c.into();
                writer.write_all(&[t as u8])?;
                match *c {
                    Constraint::None => {}
                    Constraint::Freeze { axes, x, y } => {
                        let a: u32 = axes.into();
                        writer.write_all(&a.to_le_bytes())?;
                        writer.write_all(&x.to_le_bytes())?;
                        writer.write_all(&y.to_le_bytes())?;
                    }
                    Constraint::Link { node, distance } => {
                        writer.write_all(&node.to_le_bytes())?;
                        writer.write_all(&distance.to_le_bytes())?;
                    }
                    Constraint::Hydraulic {
                        node,
                        distance,
                        speed,
                    } => {
                        writer.write_all(&node.to_le_bytes())?;
                        writer.write_all(&distance.to_le_bytes())?;
                        writer.write_all(&speed.to_le_bytes())?;
                    }
                    Constraint::Spring {
                        node,
                        distance,
                        stiffness,
                    } => {
                        writer.write_all(&node.to_le_bytes())?;
                        writer.write_all(&distance.to_le_bytes())?;
                        writer.write_all(&stiffness.to_le_bytes())?;
                    }
                    Constraint::Range { axes, start, end } => {
                        let a: u32 = axes.into();
                        writer.write_all(&a.to_le_bytes())?;
                        writer.write_all(&start.to_le_bytes())?;
                        writer.write_all(&end.to_le_bytes())?;
                    }
                    Constraint::Rotor { speed } => {
                        writer.write_all(&speed.to_le_bytes())?;
                    }
                    Constraint::Rope { node, distance } => {
                        writer.write_all(&node.to_le_bytes())?;
                        writer.write_all(&distance.to_le_bytes())?;
                    }
                }
            }
        }
        Ok(())
    }

    pub fn deserialize<R: Read>(reader: &mut R) -> io::Result<Self> {
        let mut buf = [0u8; 4];
        reader.read_exact(&mut buf)?;
        let radius = f32::from_le_bytes(buf);
        reader.read_exact(&mut buf)?;
        let nodes_len = u32::from_le_bytes(buf);
        let mut nodes = Vec::new();
        nodes.resize(nodes_len as usize, Default::default());
        for i in 0..nodes_len {
            nodes[i as usize] = Node::default();
            let n = &mut nodes[i as usize];
            reader.read_exact(&mut buf)?;
            n.p.x = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            n.p.y = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            n.v.x = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            n.v.y = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            n.constraint_len = u32::from_le_bytes(buf);
            for _ in 0..n.constraint_len {
                let mut c = [0u8; 1];
                reader.read_exact(&mut c)?;
                let mut c = Constraint::from(c[0] as u32);
                match &mut c {
                    Constraint::None => continue,
                    Constraint::Freeze { axes, x, y } => {
                        reader.read_exact(&mut buf)?;
                        *axes = Axes::from_bits_retain(u32::from_le_bytes(buf));
                        reader.read_exact(&mut buf)?;
                        *x = f32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        *y = f32::from_le_bytes(buf);
                    }
                    Constraint::Link { node, distance } | Constraint::Rope { node, distance } => {
                        reader.read_exact(&mut buf)?;
                        *node = u32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        *distance = f32::from_le_bytes(buf);
                    }
                    Constraint::Hydraulic {
                        node,
                        distance,
                        speed,
                    } => {
                        reader.read_exact(&mut buf)?;
                        *node = u32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        *distance = f32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        *speed = f32::from_le_bytes(buf);
                    }
                    Constraint::Spring {
                        node,
                        distance,
                        stiffness,
                    } => {
                        reader.read_exact(&mut buf)?;
                        *node = u32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        *distance = f32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        *stiffness = f32::from_le_bytes(buf);
                    }
                    Constraint::Range { axes, start, end } => {
                        reader.read_exact(&mut buf)?;
                        *axes = Axes::from_bits_retain(u32::from_le_bytes(buf));
                        reader.read_exact(&mut buf)?;
                        *start = f32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        *end = f32::from_le_bytes(buf);
                    }
                    Constraint::Rotor { speed } => {
                        reader.read_exact(&mut buf)?;
                        *speed = f32::from_le_bytes(buf);
                    }
                }
                n.add_constraint(c);
            }
        }

        Ok(Self {
            nodes,
            radius,
            dt: 0.0,
            remove_queue: Vec::new(),
            energy: 0.0,
        })
    }
}
