use crate::Vec2;

bitflags::bitflags! {
    #[derive(Default, Clone, Copy)]
    pub struct Axes: u32 {
        const X = 0b01;
        const Y = 0b10;
        const ALL = 0b11;
    }
}

impl From<Axes> for u32 {
    fn from(axes: Axes) -> u32 {
        axes.bits()
    }
}

#[derive(Clone)]
pub struct Node {
    pub p: Vec2,
    pub v: Vec2,
    pub fixed_p: Vec2,
    pub rotor_speed: f32,
}

impl Default for Node {
    fn default() -> Self {
        Self {
            p: Vec2::ZERO,
            v: Vec2::ZERO,
            fixed_p: Vec2::splat(f32::MAX),
            rotor_speed: 0.0,
        }
    }
}

impl Node {
    pub fn new(x: f32, y: f32) -> Self {
        let mut node = Self::default();
        node.p = Vec2::new(x, y);
        node
    }

    pub fn new_fixed_x(x: f32, y: f32) -> Self {
        let mut node = Self::new(x, y);
        node.fixed_p.x = x;
        node
    }

    pub fn new_fixed_y(x: f32, y: f32) -> Self {
        let mut node = Self::new(x, y);
        node.fixed_p.y = y;
        node
    }

    pub fn new_fixed(x: f32, y: f32) -> Self {
        let mut node = Self::new(x, y);
        node.fixed_p = node.p;
        node
    }

    pub fn new_rotor(x: f32, y: f32, speed: f32) -> Self {
        let mut node = Self::new(x, y);
        node.fixed_p = node.p;
        node.rotor_speed = speed;
        node
    }

    pub fn fixed_x(&self) -> bool {
        self.fixed_p.x != f32::MAX
    }

    pub fn fixed_y(&self) -> bool {
        self.fixed_p.y != f32::MAX
    }

    pub fn fixed(&self) -> bool {
        self.fixed_x() && self.fixed_y()
    }

    pub fn rotor(&self) -> bool {
        self.rotor_speed != 0.0
    }

    pub fn move_by(&mut self, x: f32, y: f32) {
        self.p += Vec2::new(x, y);
        if self.fixed_x() {
            self.fixed_p.x += x;
        }
        if self.fixed_y() {
            self.fixed_p.y += y;
        }
    }

    pub fn kinetic_energy(&self) -> f32 {
        0.5 * self.v.len2()
    }
}
