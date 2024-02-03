use super::renderer;
use super::App;
use crate::Node;
use crate::{integrator::*, Axes, Constraint, Cooldown, Vec2, World};
use owned_ttf_parser::name::Name;
use rand::Rng;
use std::fs::File;
use std::rc::Rc;
use winit::event::{MouseButton, WindowEvent};
use winit::keyboard::KeyCode;
use winit::window::Window;

const CONSTRAINTS: [Constraint; 7] = [
    Constraint::Link {
        node: 0,
        distance: 0.0,
    },
    Constraint::Freeze {
        axes: Axes::ALL,
        x: 0.0,
        y: 0.0,
    },
    Constraint::Rotor { speed: 0.0 },
    Constraint::Hydraulic {
        node: 0,
        distance: 0.0,
        speed: 0.0,
    },
    Constraint::Spring {
        node: 0,
        distance: 0.0,
        stiffness: 1.0,
    },
    Constraint::Freeze {
        axes: Axes::Y,
        x: 0.0,
        y: 0.0,
    },
    Constraint::Rope {
        node: 0,
        distance: 0.0,
    },
];

#[repr(u32)]
#[derive(Clone, Copy)]
enum Material {
    Node,
    Fixed,
    Rotor,
    Hydraulic,
    Spring,
    Roller,
    Rope,
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

    fn add_node(&mut self, x: f32, y: f32) -> u32 {
        match self.selected_material {
            Material::Node | Material::Hydraulic | Material::Spring | Material::Rope => {
                self.world.add_node(x, y)
            }
            Material::Fixed => {
                let n = self.world.add_node(x, y);
                self.world.nodes[n as usize].add_constraint(Constraint::Freeze {
                    axes: Axes::ALL,
                    x,
                    y,
                });
                n
            }
            Material::Rotor => {
                let n = self.world.add_node(x, y);
                self.world.nodes[n as usize].add_constraint(Constraint::Rotor { speed: 1.0 });
                self.world.nodes[n as usize].add_constraint(Constraint::Freeze {
                    axes: Axes::ALL,
                    x,
                    y,
                });
                n
            }
            Material::Roller => {
                let n = self.world.add_node(x, y);
                self.world.nodes[n as usize].add_constraint(Constraint::Freeze {
                    axes: Axes::Y,
                    x,
                    y,
                });
                n
            }
        }
    }

    fn link_nodes(&mut self, node1: u32, node2: u32) {
        match self.selected_material {
            Material::Node | Material::Fixed | Material::Rotor | Material::Roller => {
                self.world.link_node(node1, node2);
            }
            Material::Hydraulic => {
                self.world.hydraulic_node(node1, node2, 1.0);
            }
            Material::Spring => {
                self.world.spring_node(node1, node2, 1.0);
            }
            Material::Rope => {
                self.world.rope_node(node1, node2);
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
            } else {
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
            } else if let Some((node_idx, link_idx)) = intersecting_link {
                self.world.remove_link(node_idx, link_idx);
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

        if let Some(selected_node) = self.selected_node {
            for &idx in self.world.remove_queue.iter() {
                self.selected_nodes.retain(|n| *n != idx);
                if idx == selected_node {
                    self.selected_node = None;
                }
            }
        }
        self.world.flush();
    }

    pub fn event(&mut self, _event: &WindowEvent) {}

    fn render_ghost_link(
        &self,
        x: f32,
        y: f32,
        link: Constraint,
        connected_node: u32,
        gfx: &mut renderer::Renderer,
    ) {
        let mut link2 = link.clone();
        match &mut link2 {
            Constraint::Link { node, distance, .. }
            | Constraint::Rope { node, distance, .. }
            | Constraint::Spring { node, distance, .. }
            | Constraint::Hydraulic { node, distance, .. } => {
                *node = connected_node;
                *distance = Vec2::new(x, y).dist(&self.world.nodes[connected_node as usize].p);
            }
            _ => {
                link2 = Constraint::Link {
                    node: connected_node,
                    distance: Vec2::new(x, y).dist(&self.world.nodes[connected_node as usize].p),
                };
            }
        }
        gfx.color[3] = 64;
        self.world.render_link(
            &Node::new_ghost(Vec2::new(x, y), link2.clone()),
            &link2,
            &self.world.nodes,
            gfx,
        );
        self.world
            .render_node(&Node::new_ghost(Vec2::new(x, y), link.clone()), gfx);
        gfx.color[3] = 255;
    }

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

        let selected_nodes = self.world.select_nodes(&self.selected_nodes);
        gfx.color = [128, 135, 150, 255];
        self.world.render_nodes(&selected_nodes, gfx);
        gfx.color = [255, 255, 255, 255];

        // Copying Structure Ghost
        if app.key_down(KeyCode::KeyC) {
            let selected_node_ghosts = self.world.copy_nodes_as_vec(&self.selected_nodes, mx, my);
            gfx.color[3] = 64;
            self.world.render_nodes(&selected_node_ghosts, gfx);
            gfx.color[3] = 255;
        }

        // Ghost Placement
        gfx.reset();
        if let Some(selected_node) = self.selected_node {
            if app.mouse_down(MouseButton::Left) {
                for i in 0..CONSTRAINTS.len() {
                    if i as u32 == self.selected_material as u32 {
                        let mx = app.mouse_x / self.world.scale();
                        let my = app.mouse_y / self.world.scale();
                        self.render_ghost_link(mx, my, CONSTRAINTS[i].clone(), selected_node, gfx);
                    }
                }
            }
        }

        // Selection GUI
        gfx.reset();
        let old_scale = self.world.scale();
        self.world.set_scale(1.0);
        const OFF: Vec2 = Vec2::splat(-0.9);
        let gap = self.world.radius * 2.5;

        for i in 0..CONSTRAINTS.len() {
            if i as u32 == self.selected_material as u32 {
                gfx.scale[0] = 1.2;
                gfx.scale[1] = 1.2;
                gfx.color = [255, 255, 255, 255];
            } else {
                gfx.scale[0] = 0.8;
                gfx.scale[1] = 0.8;
                gfx.color = [200, 200, 200, 128];
            }

            self.world.render_node(
                &Node::new_ghost(OFF + Vec2::new(gap * i as f32, 0.0), CONSTRAINTS[i].clone()),
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
