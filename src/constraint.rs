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
    },
    Hydraulic {
        node: u32,
        distance: f32,
        speed: f32,
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
    Rope {
        node: u32,
        distance: f32,
    },
}

impl Constraint {
    pub fn link_node(&self) -> Option<&u32> {
        match self {
            Constraint::Link { node, .. }
            | Constraint::Hydraulic { node, .. }
            | Constraint::Spring { node, .. }
            | Constraint::Rope { node, .. } => Some(node),
            _ => None,
        }
    }

    pub fn link_node_mut(&mut self) -> Option<&mut u32> {
        match self {
            Constraint::Link { node, .. }
            | Constraint::Hydraulic { node, .. }
            | Constraint::Spring { node, .. }
            | Constraint::Rope { node, .. } => Some(node),
            _ => None,
        }
    }

    pub fn link_node_dist(&self) -> Option<(&u32, &f32)> {
        match self {
            Constraint::Link { node, distance, .. }
            | Constraint::Hydraulic { node, distance, .. }
            | Constraint::Spring { node, distance, .. }
            | Constraint::Rope { node, distance, .. } => Some((node, distance)),
            _ => None,
        }
    }

    pub fn link_node_dist_mut(&mut self) -> Option<(&mut u32, &mut f32)> {
        match self {
            Constraint::Link { node, distance, .. }
            | Constraint::Hydraulic { node, distance, .. }
            | Constraint::Spring { node, distance, .. }
            | Constraint::Rope { node, distance, .. } => Some((node, distance)),
            _ => None,
        }
    }
}

impl From<u32> for Constraint {
    fn from(value: u32) -> Self {
        match value {
            0 => Constraint::None,
            1 => Constraint::Freeze {
                axes: Default::default(),
                x: 0.0,
                y: 0.0,
            },
            2 => Constraint::Link {
                node: 0,
                distance: 0.0,
            },
            3 => Constraint::Hydraulic {
                node: 0,
                distance: 0.0,
                speed: 0.0,
            },
            4 => Constraint::Spring {
                node: 0,
                distance: 0.0,
                stiffness: 0.0,
            },
            5 => Constraint::Range {
                axes: Axes::default(),
                start: 0.0,
                end: 0.0,
            },
            6 => Constraint::Rotor { speed: 0.0 },
            7 => Constraint::Rope {
                node: 0,
                distance: 0.0,
            },
            _ => Constraint::None,
        }
    }
}

impl From<&Constraint> for u32 {
    fn from(value: &Constraint) -> Self {
        match value {
            Constraint::None => 0,
            Constraint::Freeze { .. } => 1,
            Constraint::Link { .. } => 2,
            Constraint::Hydraulic { .. } => 3,
            Constraint::Spring { .. } => 4,
            Constraint::Range { .. } => 5,
            Constraint::Rotor { .. } => 6,
            Constraint::Rope { .. } => 7,
        }
    }
}
