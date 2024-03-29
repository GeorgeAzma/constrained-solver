#![allow(dead_code, unused_imports)]
pub mod app;
pub mod assets;
pub mod cooldown;
pub use cooldown::*;
pub mod vec2;
pub use vec2::*;
pub mod node;
pub use node::*;
pub mod link;
pub use link::*;
pub mod world;
pub use world::*;
pub mod integrator;
pub use integrator::*;
pub mod hash_grid;
pub use hash_grid::*;
pub mod event;
pub use event::*;

#[tokio::main]
async fn main() {
    app::run().await;
}
