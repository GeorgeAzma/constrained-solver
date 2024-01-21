use crate::World;

pub trait Integrator {
    fn new() -> Self;
    fn solve(&mut self, world: &mut World);
}

#[derive(Default)]
pub struct Euler;
impl Integrator for Euler {
    fn new() -> Self {
        Self
    }
    fn solve(&mut self, world: &mut World) {
        for n in world.nodes.iter_mut() {
            n.p += n.v * world.dt;
        }
    }
}

// TODO: Figure out other integrators, tho it's working well for now
#[derive(Default)]
pub struct RK4;

impl Integrator for RK4 {
    fn new() -> Self {
        Self
    }

    fn solve(&mut self, world: &mut World) {
        for i in 0..world.nodes.len() {
            let k1 = world.nodes[i].v * world.dt;
            let k2 = (world.nodes[i].v + k1 * 0.5) * world.dt;
            let k3 = (world.nodes[i].v + k2 * 0.5) * world.dt;
            let k4 = (world.nodes[i].v + k3) * world.dt;
            world.nodes[i].p += (k1 + k2 * 2.0 + k3 * 2.0 + k4) / 6.0;
        }
    }
}
