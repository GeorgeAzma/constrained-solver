bitflags::bitflags! {
    #[derive(Clone, Copy)]
    pub struct Axes: u32 {
        const X = 0b01;
        const Y = 0b10;
        const ALL = 0b11;
    }
}

#[repr(u32, align(4))]
#[derive(Clone, Default)]
pub enum Constraint {
    #[default]
    None,
    Freeze {
        axes: Axes,
        x: f32,
        y: f32,
    },
    Link {
        node: u32,
        distance: f32,
        hydraulic_speed: f32, // <=0 if not hydraulic
    },
    Spring {
        node: u32,
        distance: f32,
        stiffness: f32,
    },
    Range {
        axes: Axes,
        start: f32,
        end: f32,
    },
    Rotor {
        speed: f32,
    },
}
