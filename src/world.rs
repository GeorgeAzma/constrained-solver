use crate::{app::renderer::Renderer, Axes, Constraint, HashGrid, Integrator, Vec2};

#[repr(u32)]
#[derive(PartialEq, Eq)]
pub enum ColliderType {
    Node,
    Road,
    Polygon,
}

pub struct Node {
    pub p: Vec2,
    pub v: Vec2,
    constraint_len: u32,
    constraints: [Constraint; 32],
    collider_type: ColliderType,
}

impl Node {
    pub const RADIUS: f32 = 0.05;
    pub const DIAMETER: f32 = Self::RADIUS * 2.0;
    pub const LINK_WIDTH: f32 = Self::RADIUS * 0.5;

    pub fn add_constraint(&mut self, constraint: Constraint) {
        // Validating, TODO: return if linked node == self
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

    pub fn kinetic_energy(&self) -> f32 {
        0.5 * self.v.len2()
    }
}

// TODO: Add fast linked nodes query, currently have to iterate over all nodes
#[derive(Default)]
pub struct World {
    pub nodes: Vec<Node>,
    pub dt: f32,
}

impl World {
    pub fn add_node(&mut self, x: f32, y: f32) -> u32 {
        self.nodes.push(Node {
            p: Vec2::new(x, y),
            v: Vec2::ZERO,
            constraint_len: 0,
            constraints: Default::default(),
            collider_type: ColliderType::Node,
        });
        (self.nodes.len() - 1) as u32
    }

    pub fn link_node(&mut self, node1: u32, node2: u32) {
        if self.linked_to_node(node1, node2).is_some() {
            return;
        }
        let dist = self.nodes[node1 as usize]
            .p
            .dist(&self.nodes[node2 as usize].p);
        self.nodes[node1 as usize].add_constraint(Constraint::Link {
            node: node2,
            distance: dist,
            hydraulic_speed: 0.0,
        });
    }

    pub fn spring_node(&mut self, node1: u32, node2: u32, stiffness: f32) {
        if self.linked_to_node(node1, node2).is_some() {
            return;
        }
        let dist = self.nodes[node1 as usize]
            .p
            .dist(&self.nodes[node2 as usize].p);
        self.nodes[node1 as usize].add_constraint(Constraint::Spring {
            node: node2,
            distance: dist,
            stiffness,
        });
    }

    pub fn linked_to_node(&self, node1: u32, node2: u32) -> Option<u32> {
        let n = &self.nodes[node1 as usize];
        for i in 0..n.constraint_len {
            let c = &n.constraints[i as usize];
            match c {
                Constraint::Link { node, .. } => {
                    if *node == node2 {
                        return Some(i);
                    }
                }
                Constraint::Spring { node, .. } => {
                    if *node == node2 {
                        return Some(i);
                    }
                }
                _ => {}
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

    pub fn swap_node(&mut self, node_idx: u32, swap_idx: u32) {
        if node_idx == swap_idx {
            return;
        }

        for i in 0..self.nodes.len() {
            if let Some(linked_constraint) = self.linked_to_node(i as u32, node_idx) {
                let c = &mut self.nodes[i].constraints[linked_constraint as usize];
                match c {
                    Constraint::Link { node, .. } => {
                        *node = swap_idx;
                    }
                    Constraint::Spring { node, .. } => {
                        *node = swap_idx;
                    }
                    _ => {}
                }
            }
        }
    }

    pub fn remove_node(&mut self, node_idx: u32) {
        for n_idx in 0..self.nodes.len() as u32 {
            if n_idx == node_idx {
                continue;
            }
            self.unlink_node(node_idx, n_idx);
        }

        self.swap_node(self.nodes.len() as u32 - 1, node_idx as u32);
        self.nodes.swap_remove(node_idx as usize);

        for i in 0..self.nodes.len() as u32 {
            if !self.node_linked(i) {
                self.remove_node(i);
                break;
            }
        }
    }

    pub fn move_node(&mut self, node_idx: u32, x: f32, y: f32) {
        let a = unsafe { &mut *self.nodes.as_mut_ptr().add(node_idx as usize) };
        a.p = Vec2::new(x, y);
        for i in 0..a.constraint_len {
            match &mut a.constraints[i as usize] {
                Constraint::Link { node, distance, .. } => {
                    if self.dt == 0.0 {
                        *distance = self.nodes[*node as usize].p.dist(&a.p);
                    }
                }
                Constraint::Spring { node, distance, .. } => {
                    if self.dt == 0.0 {
                        *distance = self.nodes[*node as usize].p.dist(&a.p);
                    }
                }
                Constraint::Freeze { axes: _, x, y } => {
                    *x = a.p.x;
                    *y = a.p.y;
                }
                _ => {}
            }
        }

        if self.dt == 0.0 {
            for b in self.nodes.iter_mut() {
                for i in 0..b.constraint_len {
                    match &mut b.constraints[i as usize] {
                        Constraint::Link { node, distance, .. } => {
                            if *node == node_idx {
                                *distance = b.p.dist(&a.p);
                            }
                        }
                        Constraint::Spring { node, distance, .. } => {
                            if *node == node_idx {
                                *distance = b.p.dist(&a.p);
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    // FIX:
    pub fn remove_link(&mut self, node_idx: u32, link_idx: u32) {
        let c = &self.nodes[node_idx as usize].constraints[link_idx as usize];
        match *c {
            Constraint::Link { node, .. } => {
                self.unlink_node(node_idx, node);
                // if !self.node_linked(node_idx) {
                //     self.remove_node(node_idx);
                // } // FIX: Above invalidates below
                // if !self.node_linked(node) {
                //     self.remove_node(node);
                // }
            }
            Constraint::Spring { node, .. } => {
                self.unlink_node(node_idx, node);
                // if !self.node_linked(node_idx) {
                //     self.remove_node(node_idx);
                // }
                // if !self.node_linked(node) {
                //     self.remove_node(node);
                // }
            }
            _ => {}
        }
    }

    pub fn get_intersecting_node(&mut self, x: f32, y: f32) -> Option<u32> {
        let mut n_idx = u32::MAX;
        let p = Vec2::new(x, y);
        let mut min = f32::MAX;
        for (i, n) in self.nodes.iter().enumerate() {
            let d = n.p.dist2(&p);
            if d < min {
                n_idx = i as u32;
                min = d;
            }
        }

        if n_idx < self.nodes.len() as u32 {
            if min <= Node::RADIUS * Node::RADIUS {
                return Some(n_idx);
            }
        }

        None
    }

    pub fn get_intersecting_link(&self, x: f32, y: f32) -> Option<(u32, u32)> {
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
                let c = &n.constraints[j as usize];
                match *c {
                    Constraint::Link { node, .. } => {
                        let b = self.nodes[node as usize].p;
                        if line_sdf(a, b) <= Node::LINK_WIDTH {
                            return Some((i as u32, j));
                        }
                    }
                    Constraint::Spring { node, .. } => {
                        let b = self.nodes[node as usize].p;
                        if line_sdf(a, b) <= Node::LINK_WIDTH {
                            return Some((i as u32, j));
                        }
                    }
                    _ => {}
                }
            }
        }
        None
    }

    pub fn update(&mut self, integrator: &mut impl Integrator, dt: f32, steps: u32) {
        self.dt = dt / steps as f32;

        let points: Vec<(f32, f32)> = self.nodes.iter().map(|n| (n.p.x, n.p.y)).collect();
        let hash_grid = HashGrid::new(&points);

        for _ in 0..steps {
            for n in self.nodes.iter_mut() {
                // Gravity
                n.v.y -= 3.0 * self.dt;
                // Drag
                n.v -= n.v * 0.25 * self.dt;
            }
            for i in 0..self.nodes.len() {
                let nodes = self.nodes.as_mut_ptr();
                let a = unsafe { nodes.add(i as usize).as_mut().unwrap() };
                // Should be !=, but for debugging purposes it's ==
                if a.collider_type != ColliderType::Node {
                    let collisions = hash_grid.find(a.p.x, a.p.y);
                    for b_idx in collisions {
                        if b_idx == i as u32 {
                            continue;
                        }
                        let b = unsafe { nodes.add(b_idx as usize).as_mut().unwrap() };
                        if a.collider_type == ColliderType::Road
                            && b.collider_type == ColliderType::Road
                        {
                            continue;
                        }
                        let dist = a.p.dist(&b.p);
                        if dist < Node::DIAMETER {
                            // Resolution
                            let to_b = b.p - a.p;
                            let push = to_b * (dist - Node::DIAMETER) * 0.5;
                            a.p += push;
                            b.p -= push;

                            // Linear impulse
                            let impulse_mag = to_b * (a.v - b.v).dot(&to_b) / (dist * dist);
                            a.v -= impulse_mag;
                            b.v += impulse_mag;
                        }
                    }
                }

                const LINK_STIFFNESS: f32 = 65536.0;
                for c in 0..a.constraint_len {
                    match a.constraints[c as usize].clone() {
                        Constraint::None => {}
                        Constraint::Freeze { axes, x, y } => {
                            if axes.contains(Axes::X) {
                                a.p.x = x;
                                a.v.x = 0.0;
                            }
                            if axes.contains(Axes::Y) {
                                a.p.y = y;
                                a.v.y = 0.0;
                            }
                        }
                        Constraint::Link {
                            node,
                            distance,
                            hydraulic_speed,
                        } => {
                            let b = unsafe { nodes.add(node as usize).as_mut().unwrap() };
                            let dist = a.p.dist(&b.p);
                            let d = (a.p - b.p) / dist * (distance - dist);
                            a.v += d * LINK_STIFFNESS * self.dt;
                            b.v -= d * LINK_STIFFNESS * self.dt;
                            a.p += d * 0.5;
                            b.p -= d * 0.5;
                            if hydraulic_speed > 0.0 {
                                // TODO:
                            }
                        }
                        Constraint::Spring {
                            node,
                            distance,
                            stiffness,
                        } => {
                            let b = unsafe { nodes.add(node as usize).as_mut().unwrap() };
                            let dist = a.p.dist(&b.p);
                            let n = (a.p - b.p) / dist;
                            let push = (distance - dist) * 0.5;
                            a.v += n * push * stiffness * self.dt;
                            b.v -= n * push * stiffness * self.dt;
                        }
                        Constraint::Range { axes, start, end } => {
                            if axes.contains(Axes::X) {
                                let clamped_x = a.p.x.clamp(start, end);
                                if clamped_x != a.p.x {
                                    a.v.x = (clamped_x - a.p.x).signum() * a.v.x.abs();
                                    a.p.x = clamped_x;
                                }
                            }
                            if axes.contains(Axes::Y) {
                                let clamped_y = a.p.y.clamp(start, end);
                                if clamped_y != a.p.y {
                                    a.v.y = (clamped_y - a.p.y).signum() * a.v.y.abs();
                                    a.p.y = clamped_y;
                                }
                            }
                        }
                        // TODO:
                        Constraint::Rotor { speed: _ } => {}
                    }
                }
            }
            integrator.solve(self);
        }
    }

    pub fn render(&self, gfx: &mut Renderer) {
        // Render links and springs
        for a in self.nodes.iter() {
            for c in (&a.constraints).iter() {
                match c {
                    Constraint::Link { node, .. } => {
                        gfx.color = [255, 255, 255, 255];
                        gfx.stroke_color = [0, 0, 0, 255];
                        gfx.stroke_width = 0.6;
                        let b = &self.nodes[*node as usize];
                        gfx.line(a.p.x, a.p.y, b.p.x, b.p.y, Node::LINK_WIDTH);
                    }
                    Constraint::Spring { node, .. } => {
                        gfx.color = [255, 200, 200, 255];
                        gfx.stroke_color = [0, 0, 0, 255];
                        gfx.stroke_width = 0.6;
                        let b = &self.nodes[*node as usize];
                        gfx.line(a.p.x, a.p.y, b.p.x, b.p.y, Node::LINK_WIDTH);
                    }
                    _ => {}
                }
            }
        }

        let mut energy = 0.0;
        // Render nodes
        for a in self.nodes.iter() {
            energy += a.kinetic_energy();
            let mut color = [240, 200, 64, 255];
            for c in (&a.constraints).iter() {
                match c {
                    Constraint::Freeze { .. } => color = [64, 180, 255, 255],
                    _ => {}
                }
            }

            gfx.reset();
            gfx.color = color;
            let Vec2 { x, y } = a.p;

            gfx.stroke_color = [
                (color[0] as f32 * 0.7) as u8,
                (color[1] as f32 * 0.7) as u8,
                (color[2] as f32 * 0.7) as u8,
                255,
            ];
            gfx.stroke_width = 0.08 / Node::RADIUS.sqrt();
            gfx.circle(x, y, Node::RADIUS);
        }

        gfx.color = [255, 255, 255, 255];
        gfx.stroke_color = [32, 32, 32, 255];
        gfx.stroke_width = 0.45;
        gfx.bold = 0.9;
        gfx.text(format!("Energy: {:.4}", energy).as_str(), -0.95, 0.9, 0.04);
    }
}
