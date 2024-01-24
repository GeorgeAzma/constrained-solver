use crate::{Vec2, World};

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
        world.step();
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
        let p0: Vec<Vec2> = world.nodes.iter().map(|n| n.p).collect();
        let v0: Vec<Vec2> = world.nodes.iter().map(|n| n.v).collect();
        world.step();
        let p1: Vec<Vec2> = world
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.p - p0[i]))
            .collect();
        let v1: Vec<Vec2> = world
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.v - v0[i]))
            .collect();
        let mut step = |p1: &[Vec2], v1: &[Vec2], m: f32| -> (Vec<Vec2>, Vec<Vec2>) {
            for i in 0..world.nodes.len() {
                world.nodes[i].p = p0[i] + p1[i] * m;
                world.nodes[i].v = v0[i] + v1[i] * m;
            }
            world.dt *= m;
            world.step();
            let p1 = world
                .nodes
                .iter()
                .enumerate()
                .map(|(i, n)| (n.p - p0[i] - p1[i] * m))
                .collect();
            let v1 = world
                .nodes
                .iter()
                .enumerate()
                .map(|(i, n)| (n.v - v0[i] - v1[i] * m))
                .collect();
            world.dt /= m;
            (p1, v1)
        };
        let (p2, v2) = step(&p1, &v1, 0.5);
        let (p3, v3) = step(&p2, &v2, 0.5);
        let (p4, v4) = step(&p3, &v3, 1.0);
        world.nodes.iter_mut().enumerate().for_each(|(i, n)| {
            n.p = p0[i] + (p1[i] + p2[i] * 2.0 + p3[i] * 2.0 + p4[i]) / 6.0;
            n.v = v0[i] + (v1[i] + v2[i] * 2.0 + v3[i] * 2.0 + v4[i]) / 6.0;
            n.p += n.v * world.dt;
        });
    }
}
