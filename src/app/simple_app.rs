use super::renderer;
use super::App;
use crate::integrator::*;
use crate::{Axes, Constraint, Vec2, World};
use owned_ttf_parser::name::Name;
use rand::Rng;
use std::rc::Rc;
use winit::event::{MouseButton, WindowEvent};
use winit::keyboard::KeyCode;
use winit::window::Window;

#[repr(u32)]
enum Material {
    Link,
    Fixed,
    Rotor,
    Hydraulic,
}

pub struct SimpleApp {
    pub app: *const App,
    world: World,
    integrator: Euler,
    selected_node: Option<u32>,
    selection_start: Option<Vec2>,
    time_scale: f32,
    selected_material: Material,
}

// TODO: selected_node index and such can get invalidated
// So maybe have world.remove_node() listener which updates
// selected_node and such dependencies
impl SimpleApp {
    pub fn new(app: &App) -> Self {
        let world = World::default();

        Self {
            app,
            world,
            integrator: Default::default(),
            selected_node: None,
            selection_start: None,
            time_scale: 1.0,
            selected_material: Material::Link,
        }
    }

    pub fn update(&mut self) {
        let app = unsafe { self.app.as_ref().unwrap() };
        self.world
            .update(&mut self.integrator, app.dt * self.time_scale, 1);
        let intersecting_node = self.world.get_intersecting_node(app.mouse_x, app.mouse_y);
        let intersecting_link = self.world.get_intersecting_link(app.mouse_x, app.mouse_y);
        if app.mouse_pressed(MouseButton::Left) {
            if let Some(intersecting_node) = intersecting_node {
                self.selected_node = Some(intersecting_node);
            } else {
                self.selected_node = Some(self.world.add_node(app.mouse_x, app.mouse_y));
            }
        } else if app.mouse_released(MouseButton::Left) {
            if let Some(selected_node) = self.selected_node {
                if let Some(intersecting_node) = intersecting_node {
                    if intersecting_node == selected_node {
                        self.world.remove_node(selected_node);
                    } else {
                        self.world.link_node(selected_node, intersecting_node);
                    }
                } else {
                    self.world.add_node(app.mouse_x, app.mouse_y);
                    self.world
                        .link_node(selected_node, self.world.nodes.len() as u32 - 1);
                }
                self.selected_node = None;
            } else {
            }
        } else if app.mouse_released(MouseButton::Right) {
            if let Some(selection_start) = self.selection_start {
                let selection_end = Vec2::new(app.mouse_x, app.mouse_y);
                let min = selection_start.min(&selection_end);
                let max = selection_start.max(&selection_end);
                let mut i = 0;
                loop {
                    let p = self.world.nodes[i].p;
                    if p.x >= min.x && p.y >= min.y && p.x <= max.x && p.y <= max.y {
                        self.world.remove_node(i as u32);
                        i -= 1;
                    }
                    i += 1;
                    if i >= self.world.nodes.len() {
                        break;
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
        {
            self.selection_start = Some(Vec2::new(app.mouse_x, app.mouse_y));
        } else if app.mouse_pressed(MouseButton::Middle) {
            if intersecting_node.is_none() {
                let n = self.world.add_node(app.mouse_x, app.mouse_y);
                self.world.nodes[n as usize].add_constraint(Constraint::Freeze {
                    axes: Axes::ALL,
                    x: app.mouse_x,
                    y: app.mouse_y,
                });
            }
        }
        if app.mouse_down(MouseButton::Middle) {
            if let Some(intersecting_node) = intersecting_node {
                self.world
                    .move_node(intersecting_node, app.mouse_x, app.mouse_y);
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
            if p.x > 2.0 || p.y > 2.0 || p.x < -2.0 || p.y < -2.0 {
                self.world.remove_node(i as u32);
                break;
            }
        }
    }

    pub fn event(&mut self, _event: &WindowEvent) {}

    pub fn render(&mut self, gfx: &mut renderer::Renderer) {
        let app = unsafe { self.app.as_ref().unwrap() };
        gfx.color = [64, 72, 96, 255];
        gfx.rect(0.0, 0.0, 1.0, 1.0);
        self.world.render(gfx);
        gfx.text(
            format!("Frame: {:.2} ms", app.dt * 1000.0).as_str(),
            -0.95,
            0.8,
            0.04,
        );

        if let Some(selection_start) = self.selection_start {
            let selection_end = Vec2::new(app.mouse_x, app.mouse_y);
            let min = selection_start.min(&selection_end) * 0.5;
            let max = selection_start.max(&selection_end) * 0.5;
            gfx.color = [255, 255, 255, 32];
            gfx.stroke_color = [255, 255, 255, 48];
            gfx.stroke_width = 0.01 / (max.x - min.x).min(max.y - min.y);
            gfx.rect(min.x + max.x, min.y + max.y, max.x - min.x, max.y - min.y);
        }
    }
}
