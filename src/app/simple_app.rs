use super::renderer;
use super::App;
use crate::integrator::*;
use crate::{Axes, Constraint, Vec2, World};
use owned_ttf_parser::name::Name;
use rand::Rng;
use std::fs::File;
use std::rc::Rc;
use winit::event::{MouseButton, WindowEvent};
use winit::keyboard::KeyCode;
use winit::window::Window;

#[repr(u32)]
enum Material {
    Node,
    Fixed,
    Rotor,
    Hydraulic,
    Spring,
    Roller,
}

pub struct SimpleApp {
    pub app: *const App,
    world: World,
    integrator: Euler,
    selected_node: Option<u32>,
    selection_start: Option<Vec2>,
    time_scale: f32,
    selected_material: Material,
    physics_dt: f32,
    move_start_pos: Vec2,
    scale: f32,
}

// TODO: selected_node index and such can get invalidated
// So maybe have world.remove_node() listener which updates
// selected_node and such dependencies
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
            selection_start: None,
            time_scale: 1.0,
            selected_material: Material::Node,
            physics_dt: 0.0,
            move_start_pos: Vec2::ZERO,
            scale: 1.0,
        }
    }

    fn add_node(&mut self, x: f32, y: f32) -> u32 {
        match self.selected_material {
            Material::Node | Material::Hydraulic | Material::Spring => self.world.add_node(x, y),
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
        }
    }

    pub fn update(&mut self) {
        // std::thread::sleep(std::time::Duration::from_millis(20));
        let app = unsafe { self.app.as_ref().unwrap() };
        let s = self.world.scale();
        let mx = app.mouse_x / s;
        let my = app.mouse_y / s;
        if let Some(selected_node) = self.selected_node {
            for &idx in self.world.remove_queue.iter() {
                if idx == selected_node {
                    self.selected_node = None;
                }
            }
        }
        self.world.update(
            &mut self.integrator,
            app.dt * self.time_scale,
            (app.dt * 60.0) as u32 + 32,
        );
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
        } else if app.mouse_released(MouseButton::Right) {
            if let Some(selection_start) = self.selection_start {
                let selection_end = Vec2::new(mx, my);
                let min = selection_start.min(&selection_end);
                let max = selection_start.max(&selection_end);
                for i in 0..self.world.nodes.len() {
                    let n = &self.world.nodes[i];
                    if n.p.x >= min.x && n.p.y >= min.y && n.p.x <= max.x && n.p.y <= max.y {
                        self.world.remove_node(i as u32);
                    }
                }
                self.selection_start = None;
            } else if let Some(intersecting_node) = intersecting_node {
                self.world.remove_node(intersecting_node);
            } else if let Some((node_idx, link_idx)) = intersecting_link {
                self.world.remove_link(node_idx, link_idx);
            }
        } else if app.mouse_pressed(MouseButton::Right)
            && intersecting_node.is_none()
            && intersecting_link.is_none()
            && self.selected_node.is_none()
        {
            self.selection_start = Some(Vec2::new(mx, my));
        }
        if app.mouse_down(MouseButton::Left) && app.key_down(KeyCode::ShiftLeft) {
            if let Some(selected_node) = self.selected_node {
                self.world.move_node(selected_node, mx, my);
            } else if let Some(intersecting_node) = intersecting_node {
                self.world.move_node(intersecting_node, mx, my);
            }
        }
        if app.key_pressed(KeyCode::Space) {
            if self.time_scale == 0.0 {
                self.time_scale = 1.0;
            } else if self.time_scale == 1.0 {
                self.time_scale = 0.0;
            }
        }
        if app.key_pressed(KeyCode::Digit1) {}

        for i in 0..self.world.nodes.len() {
            let p = self.world.nodes[i].p;
            if p.len() > 512.0 {
                self.world.remove_node(i as u32);
                break;
            }
        }
    }

    pub fn event(&mut self, _event: &WindowEvent) {}

    pub fn render(&mut self, gfx: &mut renderer::Renderer) {
        let app = unsafe { self.app.as_ref().unwrap() };
        let s = self.world.scale();
        if app.mouse_pressed(MouseButton::Middle) {
            self.move_start_pos = Vec2::new(app.mouse_x, app.mouse_y);
        } else if app.mouse_down(MouseButton::Middle) {
            let pos = Vec2::new(app.mouse_x, app.mouse_y);
            let d = pos - self.move_start_pos;
            gfx.position[0] = d.x;
            gfx.position[1] = d.y;
        } else if app.mouse_released(MouseButton::Middle) {
            let pos = Vec2::new(app.mouse_x, app.mouse_y);
            let d = pos - self.move_start_pos;
            self.world.dt = 0.0;
            self.world.move_all(d.x / s, d.y / s);
        }
        self.scale *= 1.0 + app.mouse_scroll * 0.1;
        self.world.set_scale(self.scale);
        gfx.color = [64, 72, 96, 255];
        gfx.rect(0.0, 0.0, 1000.0, 1000.0);
        gfx.line(0.0, -1000.0, 0.0, 1000.0, 0.01);
        self.world.render(gfx);
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
