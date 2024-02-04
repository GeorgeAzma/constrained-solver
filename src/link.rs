#[repr(u32, align(4))]
#[derive(Clone)]
pub enum Link {
    Link {
        n1: u32,
        n2: u32,
        dist: f32,
    },
    Rope {
        n1: u32,
        n2: u32,
        dist: f32,
    },
    Hydraulic {
        n1: u32,
        n2: u32,
        dist: f32,
        speed: f32,
    },
    Spring {
        n1: u32,
        n2: u32,
        dist: f32,
        stiffness: f32,
    },
}

impl Link {
    pub fn default() -> Self {
        Self::Link {
            n1: 0,
            n2: 0,
            dist: 0.0,
        }
    }

    pub fn n1(&self) -> u32 {
        match self {
            Link::Link { n1, .. }
            | Link::Hydraulic { n1, .. }
            | Link::Spring { n1, .. }
            | Link::Rope { n1, .. } => *n1,
        }
    }

    pub fn n2(&self) -> u32 {
        match self {
            Link::Link { n2, .. }
            | Link::Hydraulic { n2, .. }
            | Link::Spring { n2, .. }
            | Link::Rope { n2, .. } => *n2,
        }
    }

    pub fn dist(&self) -> f32 {
        match self {
            Link::Link { dist, .. }
            | Link::Hydraulic { dist, .. }
            | Link::Spring { dist, .. }
            | Link::Rope { dist, .. } => *dist,
        }
    }

    pub fn set_n1(&mut self, node1: u32) {
        match self {
            Link::Link { n1, .. }
            | Link::Hydraulic { n1, .. }
            | Link::Spring { n1, .. }
            | Link::Rope { n1, .. } => *n1 = node1,
        }
    }

    pub fn set_n2(&mut self, node2: u32) {
        match self {
            Link::Link { n2, .. }
            | Link::Hydraulic { n2, .. }
            | Link::Spring { n2, .. }
            | Link::Rope { n2, .. } => *n2 = node2,
        }
    }

    pub fn set_dist(&mut self, distance: f32) {
        match self {
            Link::Link { dist, .. }
            | Link::Hydraulic { dist, .. }
            | Link::Spring { dist, .. }
            | Link::Rope { dist, .. } => *dist = distance,
        }
    }

    pub fn linked_to(&self, n: u32) -> bool {
        self.n1() == n || self.n2() == n
    }
}
