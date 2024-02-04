use super::renderer;
use super::App;
use crate::Node;
use crate::{integrator::*, Axes, Cooldown, Link, Vec2, World};
use owned_ttf_parser::name::Name;
use rand::Rng;
use std::fs::File;
use std::rc::Rc;
use winit::event::{MouseButton, WindowEvent};
use winit::keyboard::KeyCode;
use winit::window::Window;

#[repr(u32)]
#[derive(Clone, Copy, PartialEq, Eq)]
enum Material {
    Node,
    Fixed,
    Rotor,
    Hydraulic,
    Spring,
    Roller,
    Rope,
}
const MATERIAL_LEN: u32 = Material::Rope as u32 + 1;

impl From<u32> for Material {
    fn from(value: u32) -> Self {
        match value {
            1 => Material::Fixed,
            2 => Material::Rotor,
            3 => Material::Hydraulic,
            4 => Material::Spring,
            5 => Material::Roller,
            6 => Material::Rope,
            _ => Material::Node,
        }
    }
}

pub struct SimpleApp {
    pub app: *const App,
    world: World,
    integrator: Euler,
    selected_node: Option<u32>,
    selected_nodes: Vec<u32>,
    selection_start: Option<Vec2>,
    selected_material: Material,
    time_scale: f32,
    physics_cooldown: Cooldown,
    move_start_pos: Vec2,
    scale: f32,
}

impl SimpleApp {
    pub fn new(app: &App) -> Self {
        let world;
        let file = File::open("assets/save.dat");
        if let Ok(mut file) = file {
            world = World::deserialize(&mut file).expect("Failed to load save file");
        } else {
            world = World::default();
        }

        Self {
            app,
            world,
            integrator: Default::default(),
            selected_node: None,
            selected_nodes: Vec::new(),
            selection_start: None,
            selected_material: Material::Node,
            time_scale: 1.0,
            physics_cooldown: Cooldown::new(std::time::Duration::from_secs_f32(1.0 / 256.0)),
            move_start_pos: Vec2::ZERO,
            scale: 1.0,
        }
    }

    fn material_node(material: Material, x: f32, y: f32) -> Node {
        match material {
            Material::Node | Material::Hydraulic | Material::Spring | Material::Rope => {
                Node::new(x, y)
            }
            Material::Fixed => Node::new_fixed(x, y),
            Material::Rotor => Node::new_rotor(x, y, 1.0),
            Material::Roller => Node::new_fixed_y(x, y),
        }
    }

    fn material_link(material: Material, dist: f32) -> Link {
        match material {
            Material::Rope { .. } => Link::Rope { n1: 0, n2: 1, dist },
            Material::Hydraulic { .. } => Link::Hydraulic {
                n1: 0,
                n2: 1,
                dist,
                speed: 1.0,
            },
            Material::Spring { .. } => Link::Spring {
                n1: 0,
                n2: 1,
                dist,
                stiffness: 1.0,
            },
            _ => Link::Link { n1: 0, n2: 1, dist },
        }
    }

    const fn material_color(material: Material) -> [u8; 3] {
        match material {
            Material::Rope { .. } => [160, 130, 100],
            Material::Hydraulic { .. } => [32, 72, 180],
            Material::Spring { .. } => [255, 255, 128],
            _ => World::NODE_COLOR,
        }
    }

    fn add_node(&mut self, x: f32, y: f32) -> u32 {
        self.world
            .add(Self::material_node(self.selected_material, x, y))
    }

    fn link_nodes(&mut self, node1: u32, node2: u32) {
        match self.selected_material {
            Material::Node | Material::Fixed | Material::Rotor | Material::Roller => {
                self.world.link_node(Link::Link {
                    n1: node1,
                    n2: node2,
                    dist: 0.0,
                });
            }
            Material::Hydraulic => {
                self.world.link_node(Link::Hydraulic {
                    n1: node1,
                    n2: node2,
                    dist: 0.0,
                    speed: 1.0,
                });
            }
            Material::Spring => {
                self.world.link_node(Link::Spring {
                    n1: node1,
                    n2: node2,
                    dist: 0.0,
                    stiffness: 1.0,
                });
            }
            Material::Rope => {
                self.world.link_node(Link::Rope {
                    n1: node1,
                    n2: node2,
                    dist: 0.0,
                });
            }
        }
    }

    pub fn update(&mut self) {
        std::thread::sleep(std::time::Duration::from_millis(4));

        let app = unsafe { self.app.as_ref().unwrap() };
        let mx = app.mouse_x / self.world.scale();
        let my = app.mouse_y / self.world.scale();
        while self.physics_cooldown.ready() {
            let dt = self.physics_cooldown.delay.as_secs_f32();
            self.world
                .update(&mut self.integrator, dt * self.time_scale, 1);
            self.physics_cooldown.next();
        }
        if app.key_pressed(KeyCode::Digit1) {
            self.selected_material = Material::Node;
        } else if app.key_pressed(KeyCode::Digit2) {
            self.selected_material = Material::Fixed;
        } else if app.key_pressed(KeyCode::Digit3) {
            self.selected_material = Material::Rotor;
        } else if app.key_pressed(KeyCode::Digit4) {
            self.selected_material = Material::Hydraulic;
        } else if app.key_pressed(KeyCode::Digit5) {
            self.selected_material = Material::Spring;
        } else if app.key_pressed(KeyCode::Digit6) {
            self.selected_material = Material::Roller;
        } else if app.key_pressed(KeyCode::Digit7) {
            self.selected_material = Material::Rope;
        }
        let intersecting_node = self.world.point_inside_node(mx, my);
        let intersecting_link = self.world.point_inside_link(mx, my);
        if app.mouse_pressed(MouseButton::Left) {
            if let Some(intersecting_node) = intersecting_node {
                self.selected_node = Some(intersecting_node);
            } else if !app.key_down(KeyCode::ShiftLeft) {
                self.selected_node = Some(self.add_node(mx, my));
            }
        } else if app.mouse_released(MouseButton::Left) {
            if let Some(selected_node) = self.selected_node {
                if let Some(intersecting_node) = intersecting_node {
                    if intersecting_node == selected_node
                        && !app.key_down(KeyCode::ShiftLeft)
                        && !self.world.node_linked(selected_node)
                    {
                        self.world.remove_node(selected_node);
                    } else {
                        self.link_nodes(selected_node, intersecting_node);
                    }
                } else if !app.key_down(KeyCode::ShiftLeft) {
                    self.add_node(mx, my);
                    self.link_nodes(selected_node, self.world.nodes.len() as u32 - 1);
                }
                self.selected_node = None;
            }
        } else if app.mouse_pressed(MouseButton::Right)
            && intersecting_node.is_none()
            && intersecting_link.is_none()
            && self.selected_node.is_none()
        {
            self.selection_start = Some(Vec2::new(mx, my));
        } else if app.mouse_released(MouseButton::Right) {
            if let Some(selection_start) = self.selection_start {
                self.selected_nodes.clear();
                let selection_end = Vec2::new(mx, my);
                let min = selection_start.min(&selection_end);
                let max = selection_start.max(&selection_end);
                for i in 0..self.world.nodes.len() {
                    let n = &self.world.nodes[i];
                    if n.p.x >= min.x && n.p.y >= min.y && n.p.x <= max.x && n.p.y <= max.y {
                        self.selected_nodes.push(i as u32);
                    }
                }
                self.selection_start = None;
            } else if let Some(intersecting_node) = intersecting_node {
                self.world.remove_node(intersecting_node);
            } else if let Some(link_idx) = intersecting_link {
                self.world.remove_link(link_idx);
            }
        }
        if app.mouse_down(MouseButton::Left) && app.key_down(KeyCode::ShiftLeft) {
            if let Some(selected_node) = self.selected_node {
                let Vec2 { x, y } = self.world.nodes[selected_node as usize].p;
                self.world
                    .move_node(selected_node, mx - x, my - y, self.time_scale == 0.0);
            } else if let Some(intersecting_node) = intersecting_node {
                let Vec2 { x, y } = self.world.nodes[intersecting_node as usize].p;
                self.world
                    .move_node(intersecting_node, mx - x, my - y, self.time_scale == 0.0);
            }
        }
        if app.key_pressed(KeyCode::Space) {
            if self.time_scale == 0.0 {
                self.time_scale = 1.0;
            } else {
                self.time_scale = 0.0;
            }
        }
        if app.key_pressed(KeyCode::KeyD) {
            for n in self.selected_nodes.iter() {
                self.world.remove_node(*n);
            }
            self.selected_nodes.clear();
        } else if app.key_released(KeyCode::KeyC) {
            self.world.copy_nodes(&self.selected_nodes, mx, my);
            self.selected_nodes.clear();
        }

        for i in 0..self.world.nodes.len() {
            let p = self.world.nodes[i].p;
            if p.len() > 512.0 {
                self.world.remove_node(i as u32);
                break;
            }
        }

        for &idx in self.world.node_remove_queue.iter() {
            self.selected_nodes.retain(|n| *n != idx);
            if let Some(selected_node) = self.selected_node {
                if idx == selected_node {
                    self.selected_node = None;
                }
            }
        }

        let node_swaps = self.world.flush();

        for (idx, swap_idx) in node_swaps {
            for selected_node in self.selected_nodes.iter_mut() {
                if *selected_node == idx {
                    *selected_node = swap_idx;
                } else if *selected_node == swap_idx {
                    *selected_node = idx;
                }
            }
            if let Some(selected_node) = self.selected_node {
                if selected_node == idx {
                    self.selected_node = Some(swap_idx);
                } else if selected_node == swap_idx {
                    self.selected_node = Some(idx);
                }
            }
        }
    }

    pub fn event(&mut self, _event: &WindowEvent) {}

    pub fn render(&mut self, gfx: &mut renderer::Renderer) {
        let app = unsafe { self.app.as_ref().unwrap() };
        let mx = app.mouse_x / self.world.scale();
        let my = app.mouse_y / self.world.scale();
        if app.mouse_pressed(MouseButton::Middle) {
            self.move_start_pos = Vec2::new(app.mouse_x, app.mouse_y);
        } else if app.mouse_down(MouseButton::Middle) {
            let pos = Vec2::new(app.mouse_x, app.mouse_y);
            let d = pos - self.move_start_pos;
            gfx.position[0] = d.x;
            gfx.position[1] = d.y;
        } else if app.mouse_released(MouseButton::Middle) {
            self.world.dt = 0.0;
            self.world.move_all(
                mx - self.move_start_pos.x / self.world.scale(),
                my - self.move_start_pos.y / self.world.scale(),
            );
        }
        self.scale *= 1.0 + app.mouse_scroll * 0.1;
        self.world.set_scale(self.scale);
        gfx.color = [64, 72, 96, 255];
        gfx.rect(0.0, 0.0, 1000.0, 1000.0);
        gfx.stroke_color = [255, 255, 255, 255];
        gfx.color = [255, 255, 255, 255];
        self.world.render(gfx);

        let (selected_nodes, selected_links) = self.world.select(&self.selected_nodes);
        gfx.color = [160, 190, 255, 255];
        self.world
            .render_structure(&selected_links, &selected_nodes, gfx);
        gfx.color = [255, 255, 255, 255];

        // Copying Structure Ghost
        if app.key_down(KeyCode::KeyC) {
            let (selected_ghost_nodes, selected_ghost_links) =
                self.world.select_moved(&self.selected_nodes, mx, my);
            gfx.color[3] = 64;
            self.world
                .render_structure(&selected_ghost_links, &selected_ghost_nodes, gfx);
            gfx.color[3] = 255;
        }

        // Ghost Placement
        gfx.reset();
        if let Some(selected_node) = self.selected_node {
            if app.mouse_down(MouseButton::Left) {
                gfx.color[3] = 64;
                let node = Self::material_node(self.selected_material, mx, my);
                let color = Self::material_color(self.selected_material);
                self.world.render_node(&node, color, gfx);
                let selected_node = self.world.nodes[selected_node as usize].clone();
                self.world.render_link(
                    &Self::material_link(self.selected_material, selected_node.p.dist(&node.p)),
                    &vec![selected_node, node],
                    gfx,
                );
                gfx.color[3] = 255;
            }
        }

        // Selection GUI
        gfx.reset();
        let old_scale = self.world.scale();
        self.world.set_scale(1.0);
        const OFF: Vec2 = Vec2::splat(-0.9);
        let gap = self.world.radius * 2.5;

        for i in 0..MATERIAL_LEN {
            let mat = Material::from(i);
            if mat == self.selected_material {
                gfx.scale[0] = 1.2;
                gfx.scale[1] = 1.2;
                gfx.color = [255, 255, 255, 255];
            } else {
                gfx.scale[0] = 0.8;
                gfx.scale[1] = 0.8;
                gfx.color = [200, 200, 200, 128];
            }

            let p = OFF + Vec2::new(gap * i as f32, 0.0);
            self.world.render_node(
                &Self::material_node(mat, p.x, p.y),
                Self::material_color(mat),
                gfx,
            );
        }
        self.world.set_scale(old_scale);

        gfx.reset();
        gfx.color = [255, 255, 255, 255];
        gfx.stroke_color = [32, 32, 32, 255];
        gfx.stroke_width = 0.45;
        gfx.bold = 0.9;
        gfx.text(
            format!("Energy: {:.2}", self.world.energy).as_str(),
            -0.95,
            0.9,
            0.04,
        );

        gfx.text(
            format!("Frame: {:.2} ms", app.dt * 1000.0).as_str(),
            -0.95,
            0.8,
            0.04,
        );

        if let Some(selection_start) = self.selection_start {
            let selection_end = Vec2::new(app.mouse_x, app.mouse_y) / self.world.scale();
            let min = selection_start.min(&selection_end) * 0.5 * self.world.scale();
            let max = selection_start.max(&selection_end) * 0.5 * self.world.scale();
            gfx.color = [255, 255, 255, 32];
            gfx.stroke_color = [255, 255, 255, 48];
            gfx.stroke_width = 0.01 / (max.x - min.x).min(max.y - min.y);
            gfx.rect(min.x + max.x, min.y + max.y, max.x - min.x, max.y - min.y);
        }
    }
}

impl Drop for SimpleApp {
    fn drop(&mut self) {
        let mut file = File::create("assets/save.dat").expect("Failed to open save file to save");
        self.world
            .serealize(&mut file)
            .expect("Failed to save game");
    }
}
