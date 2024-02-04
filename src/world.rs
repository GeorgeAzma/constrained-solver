use crate::{app::renderer::Renderer, Axes, HashGrid, Integrator, Link, Node, Vec2};
use std::{
    collections::{HashMap, HashSet},
    io::{self, Read, Write},
};

// TODO: Add fast linked nodes query, currently have to iterate over all nodes
pub struct World {
    pub nodes: Vec<Node>,
    pub links: Vec<Link>,
    pub radius: f32,
    pub dt: f32,
    pub energy: f32,
    pub node_links: HashMap<u32, Vec<u32>>,
    pub node_remove_queue: Vec<u32>,
    pub link_remove_queue: Vec<u32>,
}

impl Default for World {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            links: Vec::new(),
            radius: 0.05,
            dt: 0.0,
            energy: 0.0,
            node_links: HashMap::new(),
            node_remove_queue: Vec::new(),
            link_remove_queue: Vec::new(),
        }
    }
}

impl World {
    pub const NODE_COLOR: [u8; 3] = [240, 200, 64];

    pub fn add(&mut self, node: Node) -> u32 {
        self.nodes.push(node);
        (self.nodes.len() - 1) as u32
    }

    pub fn select(&mut self, nodes: &Vec<u32>) -> (Vec<Node>, Vec<Link>) {
        let mut selected = (Vec::new(), Vec::new());
        let mut selected_node_indices = Vec::new();
        let mut unlinked_nodes = HashSet::new();
        let mut set = HashSet::new();
        for &i in nodes {
            if set.insert(i) {
                selected_node_indices.push(i);
                selected.0.push(self.nodes[i as usize].clone());
            }
            let links = self.node_links.get(&i);
            if let Some(links) = links {
                for &link_idx in links.iter() {
                    let link = &self.links[link_idx as usize];
                    if link.n1() == i {
                        if set.insert(link.n2()) {
                            selected_node_indices.push(link.n2());
                            selected.0.push(self.nodes[link.n2() as usize].clone());
                            unlinked_nodes.insert(link.n2());
                        }
                    } else {
                        if set.insert(link.n1()) {
                            selected_node_indices.push(link.n1());
                            selected.0.push(self.nodes[link.n1() as usize].clone());
                            unlinked_nodes.insert(link.n1());
                        }
                    }
                }
            }
        }
        for (new_idx, idx) in selected_node_indices.iter().enumerate() {
            if unlinked_nodes.contains(idx) && !nodes.contains(idx) {
                continue;
            }
            let links = self.node_links.get(&idx);
            if let Some(links) = links {
                for i in links.iter() {
                    let mut link = self.links[*i as usize].clone();
                    if link.n1() == *idx {
                        let n2 = selected_node_indices.iter().position(|n| *n == link.n2());
                        if let Some(n2) = n2 {
                            link.set_n1(new_idx as u32);
                            link.set_n2(n2 as u32);
                            selected.1.push(link);
                        }
                    } else if link.n2() == *idx {
                        let n1 = selected_node_indices.iter().position(|n| *n == link.n1());
                        if let Some(n1) = n1 {
                            link.set_n1(n1 as u32);
                            link.set_n2(new_idx as u32);
                            selected.1.push(link);
                        }
                    }
                }
            }
        }
        selected
    }

    pub fn select_moved(&mut self, nodes: &Vec<u32>, x: f32, y: f32) -> (Vec<Node>, Vec<Link>) {
        let mut selected = self.select(nodes);
        let mut min = Vec2::splat(f32::MAX);
        for n in selected.0.iter() {
            min = min.min(&n.p);
        }
        for n in selected.0.iter_mut() {
            n.move_by(x - min.x, y - min.y);
        }
        selected
    }

    pub fn copy_nodes(&mut self, nodes: &Vec<u32>, x: f32, y: f32) {
        let mut selected = self.select_moved(nodes, x, y);

        // TODO: Join intersecting nodes

        let len = self.nodes.len() as u32;
        self.nodes.append(&mut selected.0);

        for l in selected.1.iter_mut() {
            l.set_n1(l.n1() + len);
            l.set_n2(l.n2() + len);
            self.link_node(l.clone());
        }
    }

    pub fn link_node(&mut self, mut link: Link) {
        if link.n1() == link.n2()
            || self
                .node_links
                .get(&link.n1())
                .unwrap_or(&Vec::new())
                .iter()
                .find(|&l| {
                    self.links[*l as usize].n1() == link.n2()
                        || self.links[*l as usize].n2() == link.n2()
                })
                .is_some()
        {
            return;
        }

        self.node_links
            .entry(link.n1())
            .and_modify(|e| e.push(self.links.len() as u32))
            .or_insert(vec![self.links.len() as u32]);
        self.node_links
            .entry(link.n2())
            .and_modify(|e| e.push(self.links.len() as u32))
            .or_insert(vec![self.links.len() as u32]);
        link.set_dist(
            self.nodes[link.n1() as usize]
                .p
                .dist(&self.nodes[link.n2() as usize].p),
        );
        self.links.push(link);
    }

    pub fn nodes_link(&self, node1: u32, node2: u32) -> Option<u32> {
        let links = self.node_links.get(&node1);
        if let Some(links) = links {
            if let Some(link) = links
                .iter()
                .find(|&l| self.links[*l as usize].linked_to(node2))
            {
                return Some(*link);
            }
        }
        None
    }

    pub fn remove_link(&mut self, link_idx: u32) {
        self.link_remove_queue.push(link_idx);

        let n1 = self.links[link_idx as usize].n1();
        if !(self.nodes[n1 as usize].fixed_x() || self.nodes[n1 as usize].fixed_y())
            && self.node_links[&n1].len() <= 1
        {
            self.node_remove_queue.push(n1);
        }

        let n2 = self.links[link_idx as usize].n2();
        if !(self.nodes[n2 as usize].fixed_x() || self.nodes[n2 as usize].fixed_y())
            && self.node_links[&n2].len() <= 1
        {
            self.node_remove_queue.push(n2);
        }
    }

    pub fn unlink_nodes(&mut self, node1: u32, node2: u32) {
        if let Some(link) = self.nodes_link(node1, node2) {
            self.remove_link(link);
        }
    }

    pub fn node_linked(&self, node_idx: u32) -> bool {
        let node = &self.nodes[node_idx as usize];
        node.fixed_x()
            || node.fixed_y()
            || self.node_links.get(&node_idx).unwrap_or(&Vec::new()).len() > 0
    }

    pub fn remove_node(&mut self, node_idx: u32) {
        self.node_remove_queue.push(node_idx);
        for link in self.links.iter() {
            if link.n1() == node_idx {
                let n2 = &self.nodes[link.n2() as usize];
                if self.node_links[&link.n2()].len() <= 1 && !(n2.fixed_x() || n2.fixed_y()) {
                    self.node_remove_queue.push(link.n2());
                }
            } else if link.n2() == node_idx {
                let n1 = &self.nodes[link.n1() as usize];
                if self.node_links[&link.n1()].len() <= 1 && !(n1.fixed_x() || n1.fixed_y()) {
                    self.node_remove_queue.push(link.n1());
                }
            }
        }
    }

    pub fn move_node(&mut self, node_idx: u32, x: f32, y: f32, update_constraints: bool) {
        let a = &mut self.nodes[node_idx as usize];
        a.move_by(x, y);
        a.v = Vec2::ZERO;
        if update_constraints || a.fixed() {
            let links = self.node_links.get(&node_idx);
            if let Some(links) = links {
                for link in links.iter() {
                    let link = &mut self.links[*link as usize];
                    let n1 = &self.nodes[link.n1() as usize];
                    let n2 = &self.nodes[link.n2() as usize];
                    if update_constraints {
                        link.set_dist(n1.p.dist(&n2.p));
                    }
                    if n1.fixed() && n2.fixed() {
                        link.set_dist(n1.fixed_p.dist(&n2.fixed_p));
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
        if n_idx < self.nodes.len() as u32 && min <= self.radius / self.scale() {
            Some(n_idx)
        } else {
            None
        }
    }

    pub fn point_inside_link(&self, x: f32, y: f32) -> Option<u32> {
        let p = Vec2::new(x, y);
        let line_sdf = |a: Vec2, b: Vec2| {
            let pa = p - a;
            let ba = b - a;
            let h = (pa.dot(&ba) / ba.len2()).clamp(0.0, 1.0);
            return (pa - ba * h).len();
        };
        for (i, link) in self.links.iter().enumerate() {
            if line_sdf(
                self.nodes[link.n1() as usize].p,
                self.nodes[link.n2() as usize].p,
            ) <= self.link_width() / self.scale()
            {
                return Some(i as u32);
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

    pub fn flush(&mut self) -> Vec<(u32, u32)> {
        let mut unique_set = HashSet::new();
        self.node_remove_queue.retain(|e| unique_set.insert(*e));
        let mut idx_map = HashMap::new();
        for idx in self.node_remove_queue.iter() {
            let idx = idx_map.remove(idx).unwrap_or(*idx);
            self.nodes.swap_remove(idx as usize);
            let last = self.nodes.len() as u32;
            let last_links = self.node_links.remove(&idx);
            if idx != last {
                if let Some(a) = self.node_links.remove(&last) {
                    self.node_links.insert(idx, a);
                }
            }
            if let Some(remove_links) = last_links {
                self.link_remove_queue.append(&mut remove_links.clone());
            }
            if idx != last {
                idx_map.insert(last, idx);
            }
        }
        self.node_remove_queue.clear();

        let node_swaps: Vec<(u32, u32)> = idx_map.iter().map(|(&a, &b)| (a, b)).collect();

        unique_set.clear();
        self.link_remove_queue.retain(|&e| unique_set.insert(e));
        let mut idx_map2 = HashMap::new();
        for idx in self.link_remove_queue.iter() {
            let idx = idx_map2.remove(idx).unwrap_or(*idx);
            self.links.swap_remove(idx as usize);
            if idx != self.links.len() as u32 {
                idx_map2.insert(self.links.len() as u32, idx);
            }
        }
        self.link_remove_queue.clear();

        // TODO: Avoid this
        self.node_links.clear();
        for (i, link) in self.links.iter_mut().enumerate() {
            if let Some(swap_idx) = idx_map.get(&link.n1()) {
                link.set_n1(*swap_idx);
            }
            if let Some(swap_idx) = idx_map.get(&link.n2()) {
                link.set_n2(*swap_idx);
            }
            self.node_links
                .entry(link.n1())
                .and_modify(|e| e.push(i as u32))
                .or_insert(vec![i as u32]);
            self.node_links
                .entry(link.n2())
                .and_modify(|e| e.push(i as u32))
                .or_insert(vec![i as u32]);
        }

        node_swaps
    }

    pub fn step(&mut self) {
        let points: Vec<(f32, f32)> = self.nodes.iter().map(|n| (n.p.x, n.p.y)).collect();
        let r = self.radius / self.scale();
        let hash_grid = HashGrid::new(&points, r);

        for n in self.nodes.iter_mut() {
            // Gravity
            n.v.y -= 6.0 * self.dt;
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
                if dist < r * 2.0 {
                    // Resolution
                    let to_b = b.p - a.p;
                    let push = to_b * (dist - r * 2.0) * 0.5;
                    a.p += push;
                    b.p -= push;

                    // Linear impulse
                    let impulse_mag = to_b * (a.v - b.v).dot(&to_b) / (dist * dist);
                    a.v -= impulse_mag;
                    b.v += impulse_mag;
                }
            }

            if a.fixed_x() {
                a.p.x = a.fixed_p.x;
                a.v.x = 0.0;
            }
            if a.fixed_y() {
                a.p.y = a.fixed_p.y;
                a.v.y = 0.0;
            }
        }

        const LINK_STIFFNESS: f32 = 32.0;
        const ROTOR_SPEED: f32 = 64.0;
        for link in self.links.iter_mut() {
            let nodes_ptr = self.nodes.as_mut_ptr();
            let a = unsafe { &mut *nodes_ptr.add(link.n1() as usize) };
            let b = unsafe { &mut *nodes_ptr.add(link.n2() as usize) };
            let real_dist = a.p.dist(&b.p);
            let dist = link.dist();
            let to_a = a.p - b.p;
            let inside = dist - real_dist;
            let d = to_a / real_dist * inside;

            match link.clone() {
                Link::Link { .. } => {
                    a.v += d * LINK_STIFFNESS;
                    b.v -= d * LINK_STIFFNESS;
                    a.p += d * 0.5;
                    b.p -= d * 0.5;
                }
                Link::Rope { .. } => {
                    if real_dist > dist {
                        a.v += d * LINK_STIFFNESS * 0.5; // Ropes are slightly less stiff
                        b.v -= d * LINK_STIFFNESS * 0.5;
                        a.p += d * 0.5;
                        b.p -= d * 0.5;
                    }
                }
                Link::Hydraulic { speed, .. } => {
                    link.set_dist(dist + speed * self.dt);
                    a.v += d * LINK_STIFFNESS;
                    b.v -= d * LINK_STIFFNESS;
                    a.p += d * 0.5;
                    b.p -= d * 0.5;
                }
                Link::Spring { stiffness, .. } => {
                    let n = to_a / real_dist;
                    let push = dist - real_dist;
                    a.p += n * push.abs().sqrt() * push.signum() * stiffness * 8.0 * self.dt;
                    b.p -= n * push.abs().sqrt() * push.signum() * stiffness * 8.0 * self.dt;
                    a.v += d * stiffness * 512.0 * self.dt;
                    b.v -= d * stiffness * 512.0 * self.dt;
                }
            }

            if a.rotor_speed > 0.0 {
                let c = -to_a.norm().rot90();
                b.v += c * a.rotor_speed * real_dist * self.dt * ROTOR_SPEED;
            }
            if b.rotor_speed > 0.0 {
                let c = to_a.norm().rot90();
                a.v += c * b.rotor_speed * real_dist * self.dt * ROTOR_SPEED;
            }
        }
    }

    pub fn render_node(&self, node: &Node, mut color: [u8; 3], gfx: &mut Renderer) {
        let old_col = gfx.color;
        let old_stroke_col = gfx.stroke_color;
        let old_stroke_width = gfx.stroke_width;
        let Vec2 { x, y } = node.p;

        if node.fixed_x() || node.fixed_y() {
            color = [64, 180, 255];
        }
        if node.rotor_speed != 0.0 {
            color = [180, 255, 64];
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

        gfx.color = gfx.stroke_color;
        if node.fixed() {
            gfx.circle(x * self.scale(), y * self.scale(), self.radius * 0.45);
        } else if node.fixed_x() {
            gfx.line(
                x * self.scale(),
                y * self.scale() - self.radius * 0.8,
                x * self.scale(),
                y * self.scale() + self.radius * 0.8,
                self.radius * 0.2,
            );
        } else if node.fixed_y() {
            gfx.line(
                x * self.scale() - self.radius * 0.8,
                y * self.scale(),
                x * self.scale() + self.radius * 0.8,
                y * self.scale(),
                self.radius * 0.2,
            );
        }

        gfx.color = old_col;
        gfx.stroke_color = old_stroke_col;
        gfx.stroke_width = old_stroke_width;
    }

    pub fn render_link(&self, link: &Link, nodes: &Vec<Node>, gfx: &mut Renderer) {
        let old_col = gfx.color;
        let old_stroke_col = gfx.stroke_color;
        let old_stroke_width = gfx.stroke_width;

        if link.n1() >= nodes.len() as u32 || link.n2() >= nodes.len() as u32 {
            println!("Invalid link, can't render it");
            return;
        }

        let a = &nodes[link.n1() as usize];
        let b = &nodes[link.n2() as usize];

        let to_a = (a.p - b.p).norm();
        let mut d = 4.0 * ((-to_a).dot(&b.v) + to_a.dot(&a.v));
        d = d.abs().sqrt() * d.signum();

        let rr = 1.0 - d.max(0.0);
        let gg = rr + d.min(0.0);
        let bb = 1.0 + d.min(0.0);

        let color = [
            (gfx.color[0] as f32 * rr) as u8,
            (gfx.color[1] as f32 * gg) as u8,
            (gfx.color[2] as f32 * bb) as u8,
            gfx.color[3],
        ];
        gfx.color = color;

        gfx.stroke_color = [color[0] / 2, color[1] / 2, color[2] / 2, color[3]];
        gfx.stroke_width = 0.5;

        match link {
            Link::Link { .. } => {
                gfx.line(
                    a.p.x * self.scale(),
                    a.p.y * self.scale(),
                    b.p.x * self.scale(),
                    b.p.y * self.scale(),
                    self.link_width(),
                );
            }
            Link::Hydraulic { .. } => {
                gfx.line(
                    a.p.x * self.scale(),
                    a.p.y * self.scale(),
                    b.p.x * self.scale(),
                    b.p.y * self.scale(),
                    self.link_width(),
                );
                gfx.color = [color[0] / 2, color[1] / 2, color[2] / 2, color[3]];
                let to_b = b.p - (b.p - a.p) * 0.5;
                gfx.line(
                    a.p.x * self.scale(),
                    a.p.y * self.scale(),
                    to_b.x * self.scale(),
                    to_b.y * self.scale(),
                    self.link_width(),
                );
            }
            Link::Spring { stiffness, .. } => {
                let c = (-to_a).rot90() * self.link_width() / self.scale();
                let windings = (link.dist() * stiffness * 32.0) as u32;
                let inv = 1.0 / windings as f32;
                let to_b = b.p - a.p;
                gfx.stroke_width = 0.7;
                for i in 0..windings {
                    let d = i as f32 * inv;
                    let p1 = (a.p - c + to_b * d) * self.scale();
                    let p2 = (a.p + c + to_b * (d + inv)) * self.scale();
                    gfx.line(p1.x, p1.y, p2.x, p2.y, self.link_width() * 0.25);
                    let p2 = p2 - to_b * inv * self.scale();
                    gfx.line(p1.x, p1.y, p2.x, p2.y, self.link_width() * 0.25);
                }
            }
            Link::Rope { .. } => {
                gfx.line(
                    a.p.x * self.scale(),
                    a.p.y * self.scale(),
                    b.p.x * self.scale(),
                    b.p.y * self.scale(),
                    self.link_width() * 0.7,
                );

                gfx.color = [color[0] / 2, color[1] / 2, color[2] / 2, color[3]];
                let to_b = b.p - a.p;
                let c = to_b.norm().rot90() * self.link_width() / self.scale() * 0.45;
                let windings = (link.dist() * 32.0) as u32;
                let inv = 1.0 / windings as f32;
                for i in 0..windings {
                    let d = i as f32 * inv;
                    let p1 = (a.p - c + to_b * d) * self.scale();
                    let p2 = (a.p + c + to_b * (d + inv * 0.5)) * self.scale();
                    gfx.line(p1.x, p1.y, p2.x, p2.y, self.link_width() * 0.25);
                }
            }
        }
        gfx.color = old_col;
        gfx.stroke_color = old_stroke_col;
        gfx.stroke_width = old_stroke_width;
    }

    pub fn render_links(&self, links: &Vec<Link>, nodes: &Vec<Node>, gfx: &mut Renderer) {
        for l in links.iter() {
            self.render_link(l, nodes, gfx);
        }
    }

    pub fn render_nodes(&self, nodes: &Vec<Node>, gfx: &mut Renderer) {
        for n in nodes.iter() {
            self.render_node(&n, Self::NODE_COLOR, gfx);
        }
    }

    pub fn render_structure(&self, links: &Vec<Link>, nodes: &Vec<Node>, gfx: &mut Renderer) {
        self.render_links(&links, &nodes, gfx);
        self.render_nodes(&nodes, gfx);
    }

    pub fn render(&self, gfx: &mut Renderer) {
        self.render_structure(&self.links, &self.nodes, gfx);
    }

    pub fn serealize<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&self.radius.to_le_bytes())?;

        writer.write_all(&(self.nodes.len() as u32).to_le_bytes())?;
        for n in self.nodes.iter() {
            writer.write_all(&n.p.x.to_le_bytes())?;
            writer.write_all(&n.p.y.to_le_bytes())?;
            writer.write_all(&n.v.x.to_le_bytes())?;
            writer.write_all(&n.v.y.to_le_bytes())?;
            writer.write_all(&n.fixed_p.x.to_ne_bytes())?;
            writer.write_all(&n.fixed_p.y.to_ne_bytes())?;
            writer.write_all(&n.rotor_speed.to_ne_bytes())?;
        }

        writer.write_all(&(self.links.len() as u32).to_le_bytes())?;
        for l in self.links.iter() {
            writer.write_all(&l.n1().to_le_bytes())?;
            writer.write_all(&l.n2().to_le_bytes())?;
            writer.write_all(&l.dist().to_le_bytes())?;
            match *l {
                Link::Link { .. } => {
                    writer.write_all(&[0])?;
                }
                Link::Rope { .. } => {
                    writer.write_all(&[1])?;
                }
                Link::Hydraulic { speed, .. } => {
                    writer.write_all(&[2])?;
                    writer.write_all(&speed.to_le_bytes())?;
                }
                Link::Spring { stiffness, .. } => {
                    writer.write_all(&[3])?;
                    writer.write_all(&stiffness.to_le_bytes())?;
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
        nodes.resize(nodes_len as usize, Node::default());
        for node in nodes.iter_mut() {
            reader.read_exact(&mut buf)?;
            node.p.x = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            node.p.y = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            node.v.x = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            node.v.y = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            node.fixed_p.x = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            node.fixed_p.y = f32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            node.rotor_speed = f32::from_le_bytes(buf);
        }

        reader.read_exact(&mut buf)?;
        let links_len = u32::from_le_bytes(buf);
        let mut links = Vec::new();
        links.resize(links_len as usize, Link::default());

        for link in links.iter_mut() {
            reader.read_exact(&mut buf)?;
            let n1 = u32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            let n2 = u32::from_le_bytes(buf);
            reader.read_exact(&mut buf)?;
            let dist = f32::from_le_bytes(buf);

            let mut c = [0u8];
            reader.read_exact(&mut c)?;
            *link = match c[0] {
                0 => Link::Link { n1, n2, dist },
                1 => Link::Rope { n1, n2, dist },
                2 => {
                    reader.read_exact(&mut buf)?;
                    let speed = f32::from_le_bytes(buf);
                    Link::Hydraulic {
                        n1,
                        n2,
                        dist,
                        speed,
                    }
                }
                3 => {
                    reader.read_exact(&mut buf)?;
                    let stiffness = f32::from_le_bytes(buf);
                    Link::Spring {
                        n1,
                        n2,
                        dist,
                        stiffness,
                    }
                }
                _ => {
                    println!("Invalid constraint encountered while loading the world");
                    Link::default()
                }
            };
        }

        let mut world = Self {
            nodes,
            links: Vec::new(),
            radius,
            dt: 0.0,
            energy: 0.0,
            node_links: HashMap::new(),
            node_remove_queue: Vec::new(),
            link_remove_queue: Vec::new(),
        };

        for link in links {
            world.link_node(link);
        }

        Ok(world)
    }
}
