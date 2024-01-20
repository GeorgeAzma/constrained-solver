#[allow(dead_code, unused_imports)]
pub mod app;
pub mod assets;
pub mod cooldown;
pub mod vec2;
pub use vec2::*;
pub mod world;
pub use world::*;
pub mod integrator;
pub use integrator::*;
pub mod constraint;
pub use constraint::*;
pub mod hash_grid;
pub use hash_grid::*;

#[tokio::main]
async fn main() {
    app::run().await;
}
