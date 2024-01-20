use crate::World;

pub trait Integrator {
    fn new() -> Self;
    fn solve(&mut self, world: &mut World);
}

#[derive(Default)]
pub struct Euler;
impl Integrator for Euler {
    fn new() -> Self {
        Self {}
    }
    fn solve(&mut self, world: &mut World) {
        for obj in world.nodes.iter_mut() {
            obj.p += obj.v * world.dt;
        }
    }
}

#[derive(Default)]
pub struct RK4;

impl Integrator for RK4 {
    fn new() -> Self {
        Self
    }

    fn solve(&mut self, _world: &mut World) {}
}
