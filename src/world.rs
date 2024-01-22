use crate::{app::renderer::Renderer, Axes, Constraint, HashGrid, Integrator, Vec2};
use std::{
    collections::HashMap,
    io::{self, Read, Write},
};

#[repr(u32)]
#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub enum ColliderType {
    #[default]
    None,
    Node,
}

#[derive(Clone, Default)]
pub struct Node {
    pub p: Vec2,
    pub v: Vec2,
    constraint_len: u32,
    constraints: [Constraint; 32],
    collider_type: ColliderType,
}

impl Node {
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

    pub fn kinetic_energy(&self) -> f32 {
        0.5 * self.v.len2()
    }
}

pub struct RemoveNodeEvent(u32);

// TODO: Add fast linked nodes query, currently have to iterate over all nodes
pub struct World {
    pub nodes: Vec<Node>,
    pub radius: f32,
    pub dt: f32,
    pub remove_queue: Vec<u32>,
}

impl Default for World {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            radius: 0.05,
            dt: 0.0,
            remove_queue: Vec::new(),
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
        });
    }

    pub fn hydraulic_node(&mut self, node1: u32, node2: u32, speed: f32) {
        if self.linked_to_node(node1, node2).is_some() {
            return;
        }
        let dist = self.nodes[node1 as usize]
            .p
            .dist(&self.nodes[node2 as usize].p);
        self.nodes[node1 as usize].add_constraint(Constraint::Hydraulic {
            node: node2,
            distance: dist,
            speed,
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
                Constraint::Link { node, .. }
                | Constraint::Hydraulic { node, .. }
                | Constraint::Spring { node, .. } => {
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

    fn swap_node(&mut self, node_idx: u32, swap_idx: u32) {
        if node_idx == swap_idx {
            return;
        }

        for i in 0..self.nodes.len() {
            if let Some(linked_constraint) = self.linked_to_node(i as u32, node_idx) {
                let c = &mut self.nodes[i].constraints[linked_constraint as usize];
                match c {
                    Constraint::Link { node, .. }
                    | Constraint::Hydraulic { node, .. }
                    | Constraint::Spring { node, .. } => {
                        *node = swap_idx;
                    }
                    _ => {}
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

    pub fn move_node(&mut self, node_idx: u32, x: f32, y: f32) {
        let a = unsafe { &mut *self.nodes.as_mut_ptr().add(node_idx as usize) };
        a.p = Vec2::new(x, y);
        for i in 0..a.constraint_len {
            match &mut a.constraints[i as usize] {
                Constraint::Link { node, distance }
                | Constraint::Hydraulic { node, distance, .. }
                | Constraint::Spring { node, distance, .. } => {
                    if self.dt == 0.0 {
                        *distance = self.nodes[*node as usize].p.dist(&a.p);
                    }
                }
                Constraint::Freeze { axes, x, y } => {
                    if axes.contains(Axes::X) {
                        *x = a.p.x;
                    }
                    if axes.contains(Axes::Y) {
                        *y = a.p.y;
                    }
                }
                _ => {}
            }
        }

        if self.dt == 0.0 {
            for b in self.nodes.iter_mut() {
                for i in 0..b.constraint_len {
                    match &mut b.constraints[i as usize] {
                        Constraint::Link { node, distance, .. }
                        | Constraint::Hydraulic { node, distance, .. }
                        | Constraint::Spring { node, distance, .. } => {
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

    pub fn scale(&self) -> f32 {
        return self.radius * 20.0;
    }

    pub fn link_width(&self) -> f32 {
        return self.radius * 0.65;
    }

    pub fn move_all(&mut self, x: f32, y: f32) {
        for i in 0..self.nodes.len() {
            let p = self.nodes[i].p;
            self.move_node(i as u32, p.x + x, p.y + y);
        }
    }

    pub fn set_scale(&mut self, scale: f32) {
        self.radius = scale * 0.05;
    }

    pub fn remove_link(&mut self, node_idx: u32, link_idx: u32) {
        let c = &self.nodes[node_idx as usize].constraints[link_idx as usize];
        match *c {
            Constraint::Link { node, .. }
            | Constraint::Hydraulic { node, .. }
            | Constraint::Spring { node, .. } => {
                self.unlink_node(node_idx, node);
                if !self.node_linked(node_idx) {
                    self.remove_node(node_idx);
                }
                if !self.node_linked(node) {
                    self.remove_node(node);
                }
            }
            _ => {}
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
                let c = &n.constraints[j as usize];
                match *c {
                    Constraint::Link { node, .. }
                    | Constraint::Hydraulic { node, .. }
                    | Constraint::Spring { node, .. } => {
                        let b = self.nodes[node as usize].p;
                        if line_sdf(a, b) <= self.link_width() / self.scale() {
                            return Some((i as u32, j));
                        }
                    }
                    _ => {}
                }
            }
        }
        None
    }

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
                let c = &n.constraints[j as usize];
                match *c {
                    Constraint::Link { node, .. }
                    | Constraint::Hydraulic { node, .. }
                    | Constraint::Spring { node, .. } => {
                        let b = self.nodes[node as usize].p;
                        if line_rect_intersection(a.x, a.y, b.x, b.y, min_x, min_y, max_x, max_y) {
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

        use std::collections::{HashMap, HashSet};
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

        if self.dt != 0.0 {
            for _ in 0..steps {
                self.step();
                integrator.solve(self);
            }
        }
    }

    pub fn step(&mut self) {
        let points: Vec<(f32, f32)> = self.nodes.iter().map(|n| (n.p.x, n.p.y)).collect();
        let hash_grid = HashGrid::new(&points, self.radius / self.scale());

        for n in self.nodes.iter_mut() {
            n.v.y -= 6.0 * self.dt; // Gravity
                                    // n.v -= n.v * 0.3 * self.dt; // Drag
        }
        for i in 0..self.nodes.len() {
            let nodes = self.nodes.as_mut_ptr();
            let a = unsafe { nodes.add(i as usize).as_mut().unwrap() };
            if a.collider_type == ColliderType::None {
                continue;
            }
            let collisions = hash_grid.find(a.p.x, a.p.y);
            for b_idx in collisions {
                if b_idx == i as u32 {
                    continue;
                }
                let b = unsafe { nodes.add(b_idx as usize).as_mut().unwrap() };
                if a.collider_type == ColliderType::None {
                    continue;
                }
                let dist = a.p.dist(&b.p);
                if dist < self.radius * 2.0 / self.scale() {
                    // Resolution
                    let to_b = b.p - a.p;
                    let push = to_b * (dist - self.radius * 2.0 / self.scale()) * 0.5;
                    a.p += push;
                    b.p -= push;

                    // Linear impulse
                    let impulse_mag = to_b * (a.v - b.v).dot(&to_b) / (dist * dist);
                    a.v -= impulse_mag;
                    b.v += impulse_mag;
                }
            }

            const LINK_STIFFNESS: f32 = 32.0;

            let mut rotor_speed = 0.0;
            for c in 0..a.constraint_len {
                match a.constraints[c as usize] {
                    Constraint::Rotor { speed } => {
                        rotor_speed = speed;
                        break;
                    }
                    _ => {}
                }
            }
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
                            let c = (b.p - a.p).norm().cross();
                            b.p += c * rotor_speed * dist * 0.0025 * self.dt;
                            b.v += c * rotor_speed * dist * 0.0025;
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
                            let c = (b.p - a.p).norm().cross();
                            b.p += c * rotor_speed * dist * 0.0025 * self.dt;
                            b.v += c * rotor_speed * dist * 0.0025;
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
                        a.p += n
                            * (push.abs().sqrt() + push * push)
                            * push.signum()
                            * *stiffness
                            * 2.0
                            * self.dt;
                        b.p -= n
                            * (push.abs().sqrt() + push * push)
                            * push.signum()
                            * *stiffness
                            * 2.0
                            * self.dt;
                        a.v += n * push * *stiffness * 128.0 * self.dt;
                        b.v -= n * push * *stiffness * 128.0 * self.dt;
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
                }
            }
        }
    }

    pub fn render(&self, gfx: &mut Renderer) {
        // Render links and springs
        let s = self.scale();
        for a in self.nodes.iter() {
            for c in (&a.constraints).iter() {
                match c {
                    Constraint::Link { node, .. } => {
                        let b = &self.nodes[*node as usize];
                        let to_a = (a.p - b.p).norm();
                        let mut d = 4.0 * ((-to_a).dot(&b.v) + to_a.dot(&a.v));
                        d = d.abs().sqrt() * d.signum();
                        {
                            let r = 1.0 - d.max(0.0);
                            let b = 1.0 + d.min(0.0);
                            let g = 1.0 - d.max(0.0) + d.min(0.0);
                            gfx.color = [
                                (r * 255.0 + 0.5) as u8,
                                (g * 255.0 + 0.5) as u8,
                                (b * 255.0 + 0.5) as u8,
                                255,
                            ];
                        }
                        gfx.stroke_color =
                            [gfx.color[0] / 3, gfx.color[1] / 3, gfx.color[2] / 3, 255];
                        gfx.stroke_width = 0.5;
                        gfx.line(
                            a.p.x * s,
                            a.p.y * s,
                            b.p.x * s,
                            b.p.y * s,
                            self.link_width(),
                        );
                    }
                    Constraint::Hydraulic { node, .. } => {
                        let b = &self.nodes[*node as usize];
                        let to_a = (a.p - b.p).norm();
                        let mut d = 4.0 * ((-to_a).dot(&b.v) + to_a.dot(&a.v));
                        d = d.abs().sqrt() * d.signum();
                        {
                            let r = 1.0 - d.max(0.0);
                            let b = 1.0 + d.min(0.0);
                            let g = 1.0 - d.max(0.0) + d.min(0.0);
                            gfx.color = [
                                (r * 255.0 + 0.5) as u8,
                                (g * 255.0 + 0.5) as u8,
                                (b * 255.0 + 0.5) as u8,
                                255,
                            ];
                        }
                        gfx.stroke_color =
                            [gfx.color[0] / 3, gfx.color[1] / 3, gfx.color[2] / 3, 255];
                        gfx.stroke_width = 0.5;
                        gfx.line(
                            a.p.x * s,
                            a.p.y * s,
                            b.p.x * s,
                            b.p.y * s,
                            self.link_width(),
                        );
                        gfx.color = [gfx.color[0] / 2, gfx.color[1] / 2, gfx.color[2] / 2, 255];
                        let to_b = b.p - (b.p - a.p) * 0.5;
                        gfx.line(
                            a.p.x * s,
                            a.p.y * s,
                            to_b.x * s,
                            to_b.y * s,
                            self.link_width(),
                        );
                    }
                    Constraint::Spring {
                        node,
                        distance,
                        stiffness,
                    } => {
                        let b = &self.nodes[*node as usize];
                        let to_a = (a.p - b.p).norm();
                        let mut d = (-to_a).dot(&b.v) + to_a.dot(&a.v);
                        d = d * 0.25;
                        {
                            let r = 1.0 - d.max(0.0);
                            let b = 1.0 + d.min(0.0);
                            let g = 1.0 - d.max(0.0) + d.min(0.0);
                            gfx.color = [
                                (r * 255.0 + 0.5) as u8,
                                (g * 255.0 + 0.5) as u8,
                                (b * 255.0 + 0.5) as u8,
                                255,
                            ];
                        }

                        gfx.stroke_color =
                            [gfx.color[0] / 3, gfx.color[1] / 3, gfx.color[2] / 3, 255];
                        gfx.stroke_width = 0.5;
                        let to_b = b.p - a.p;
                        let c = to_b.norm().cross() * self.link_width();
                        let windings = (distance * stiffness * 32.0) as u32;
                        let inv = 1.0 / windings as f32;
                        for i in 0..windings {
                            let d = i as f32 * inv;
                            let p1 = (a.p - c + to_b * d) * s;
                            let p2 = (a.p + c + to_b * (d + inv)) * s;
                            gfx.line(p1.x, p1.y, p2.x, p2.y, self.link_width() * 0.25);
                            let p2 = p2 - to_b * inv * s;
                            gfx.line(p1.x, p1.y, p2.x, p2.y, self.link_width() * 0.25);
                        }
                    }
                    _ => {}
                }
            }
        }

        let mut energy = 0.0;
        // Render nodes
        for a in self.nodes.iter() {
            energy += a.kinetic_energy();
            let Vec2 { x, y } = a.p;
            let mut color = [240, 200, 64, 255];
            let mut is_rotor = false;
            let mut freeze_axes = None;
            for c in (&a.constraints).iter() {
                match c {
                    Constraint::Freeze { axes, .. } => {
                        color = [64, 180, 255, 255];
                        freeze_axes = Some(axes);
                    }
                    Constraint::Rotor { .. } => {
                        is_rotor = true;
                    }
                    _ => {}
                }
            }

            if is_rotor {
                color = [180, 255, 64, 255];
            }

            gfx.color = color;

            gfx.stroke_color = [
                (color[0] as f32 * 0.7) as u8,
                (color[1] as f32 * 0.7) as u8,
                (color[2] as f32 * 0.7) as u8,
                255,
            ];
            gfx.stroke_width = 0.08 / self.radius.sqrt();
            gfx.circle(x * s, y * s, self.radius);

            if let Some(axes) = freeze_axes {
                gfx.color = gfx.stroke_color;
                if axes.contains(Axes::ALL) {
                    gfx.circle(x * s, y * s, self.radius * 0.5);
                } else if axes.contains(Axes::X) {
                    gfx.line(
                        x * s,
                        y * s - self.radius * 0.8,
                        x * s,
                        y * s + self.radius * 0.8,
                        self.radius * 0.2,
                    );
                } else if axes.contains(Axes::Y) {
                    gfx.line(
                        x * s - self.radius * 0.8,
                        y * s,
                        x * s + self.radius * 0.8,
                        y * s,
                        self.radius * 0.2,
                    );
                }
            }
            if is_rotor {
                gfx.color = gfx.stroke_color;
                gfx.line(
                    x * s - self.radius * 0.8,
                    y * s,
                    x * s + self.radius * 0.8,
                    y * s,
                    self.radius * 0.2,
                );
                gfx.line(
                    x * s,
                    y * s - self.radius * 0.8,
                    x * s,
                    y * s + self.radius * 0.8,
                    self.radius * 0.2,
                );
            }
        }

        gfx.color = [255, 255, 255, 255];
        gfx.stroke_color = [32, 32, 32, 255];
        gfx.stroke_width = 0.45;
        gfx.bold = 0.9;
        gfx.text(format!("Energy: {:.4}", energy).as_str(), -0.95, 0.9, 0.04);
    }

    pub fn serealize<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&self.radius.to_le_bytes())?;
        writer.write_all(&(self.nodes.len() as u32).to_le_bytes())?;
        for n in self.nodes.iter() {
            writer.write_all(&n.p.x.to_le_bytes())?;
            writer.write_all(&n.p.y.to_le_bytes())?;
            writer.write_all(&n.v.x.to_le_bytes())?;
            writer.write_all(&n.v.y.to_le_bytes())?;
            let collider_type = match n.collider_type {
                ColliderType::None => 0u32,
                ColliderType::Node => 1u32,
            };
            writer.write_all(&collider_type.to_le_bytes())?;
            writer.write_all(&n.constraint_len.to_le_bytes())?;
            for i in 0..n.constraint_len {
                let c = &n.constraints[i as usize];
                match *c {
                    Constraint::None => writer.write_all(&[0u8])?,
                    Constraint::Freeze { axes, x, y } => {
                        writer.write_all(&[1u8])?;
                        let a: u32 = axes.into();
                        writer.write_all(&a.to_le_bytes())?;
                        writer.write_all(&x.to_le_bytes())?;
                        writer.write_all(&y.to_le_bytes())?;
                    }
                    Constraint::Link { node, distance } => {
                        writer.write_all(&[2u8])?;
                        writer.write_all(&node.to_le_bytes())?;
                        writer.write_all(&distance.to_le_bytes())?;
                    }
                    Constraint::Hydraulic {
                        node,
                        distance,
                        speed,
                    } => {
                        writer.write_all(&[3u8])?;
                        writer.write_all(&node.to_le_bytes())?;
                        writer.write_all(&distance.to_le_bytes())?;
                        writer.write_all(&speed.to_le_bytes())?;
                    }
                    Constraint::Spring {
                        node,
                        distance,
                        stiffness,
                    } => {
                        writer.write_all(&[4u8])?;
                        writer.write_all(&node.to_le_bytes())?;
                        writer.write_all(&distance.to_le_bytes())?;
                        writer.write_all(&stiffness.to_le_bytes())?;
                    }
                    Constraint::Range { axes, start, end } => {
                        writer.write_all(&[5u8])?;
                        let a: u32 = axes.into();
                        writer.write_all(&a.to_le_bytes())?;
                        writer.write_all(&start.to_le_bytes())?;
                        writer.write_all(&end.to_le_bytes())?;
                    }
                    Constraint::Rotor { speed } => {
                        writer.write_all(&[6u8])?;
                        writer.write_all(&speed.to_le_bytes())?;
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
            let collider_type = u32::from_le_bytes(buf);
            n.collider_type = match collider_type {
                1 => ColliderType::Node,
                _ => ColliderType::None,
            };
            reader.read_exact(&mut buf)?;
            n.constraint_len = u32::from_le_bytes(buf);
            for _ in 0..n.constraint_len {
                let mut c = [0u8; 1];
                reader.read_exact(&mut c)?;
                match c[0] {
                    0 => continue,
                    1 => {
                        reader.read_exact(&mut buf)?;
                        let a = u32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let x = f32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let y = f32::from_le_bytes(buf);
                        n.add_constraint(Constraint::Freeze {
                            axes: Axes::from_bits_retain(a),
                            x,
                            y,
                        });
                    }
                    2 => {
                        reader.read_exact(&mut buf)?;
                        let node = u32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let distance = f32::from_le_bytes(buf);
                        n.add_constraint(Constraint::Link { node, distance });
                    }
                    3 => {
                        reader.read_exact(&mut buf)?;
                        let node = u32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let distance = f32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let speed = f32::from_le_bytes(buf);
                        n.add_constraint(Constraint::Hydraulic {
                            node,
                            distance,
                            speed,
                        });
                    }
                    4 => {
                        reader.read_exact(&mut buf)?;
                        let node = u32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let distance = f32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let stiffness = f32::from_le_bytes(buf);
                        n.add_constraint(Constraint::Spring {
                            node,
                            distance,
                            stiffness,
                        });
                    }
                    5 => {
                        reader.read_exact(&mut buf)?;
                        let a = u32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let start = f32::from_le_bytes(buf);
                        reader.read_exact(&mut buf)?;
                        let end = f32::from_le_bytes(buf);
                        n.add_constraint(Constraint::Range {
                            axes: Axes::from_bits_retain(a),
                            start,
                            end,
                        });
                    }
                    6 => {
                        reader.read_exact(&mut buf)?;
                        let speed = f32::from_le_bytes(buf);
                        n.add_constraint(Constraint::Rotor { speed });
                    }
                    _ => {}
                }
            }
        }

        Ok(Self {
            nodes,
            radius,
            dt: 0.0,
            remove_queue: Vec::new(),
        })
    }
}
