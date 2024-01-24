pub struct HashGrid {
    cell_size: f32,
    point_cell_keys: Vec<(u32, u32)>,
    cell_start_indices: Vec<u32>,
}

impl HashGrid {
    pub fn new(points: &Vec<(f32, f32)>, cell_size: f32) -> Self {
        if points.len() == 0 {
            return Self {
                cell_size,
                point_cell_keys: Vec::new(),
                cell_start_indices: Vec::new(),
            };
        }

        let mut point_cell_keys = Vec::new();
        let mut cell_start_indices = Vec::new();
        point_cell_keys.resize(points.len(), Default::default());
        cell_start_indices.resize(points.len(), u32::MAX);

        for (i, &(x, y)) in points.iter().enumerate() {
            let (cx, cy) = ((x / cell_size) as i32, (y / cell_size) as i32);
            let key = Self::cell_key(cx, cy, points.len() as i32);
            point_cell_keys[i] = (i as u32, key);
        }

        point_cell_keys.sort_unstable_by(|a, b| a.1.cmp(&b.1));

        let (_, last_key) = point_cell_keys[0];
        cell_start_indices[last_key as usize] = 0;
        for i in 1..point_cell_keys.len() {
            let (_, key) = point_cell_keys[i];
            if last_key != key {
                cell_start_indices[key as usize] = i as u32;
            }
        }

        Self {
            cell_size,
            point_cell_keys,
            cell_start_indices,
        }
    }

    fn to_cell_coord(&self, x: f32, y: f32) -> (i32, i32) {
        ((x / self.cell_size) as i32, (y / self.cell_size) as i32)
    }

    fn cell_key(cx: i32, cy: i32, size: i32) -> u32 {
        (cx.wrapping_mul(15823)
            .wrapping_add(cy.wrapping_mul(9737333))
            % size)
            .abs() as u32
    }

    pub fn find(&self, x: f32, y: f32) -> Vec<u32> {
        if self.point_cell_keys.len() == 0 {
            return Vec::new();
        }

        let mut indices = Vec::new();
        for i in -1..=1 {
            for j in -1..=1 {
                let (cx, cy) = self.to_cell_coord(x, y);
                let (cx, cy) = (cx + i, cy + j);
                let key = Self::cell_key(cx, cy, self.point_cell_keys.len() as i32);
                let start = self.cell_start_indices[key as usize];
                if start == u32::MAX {
                    continue;
                }
                for i in 0..(self.point_cell_keys.len() as u32 - start) {
                    let (point_idx, cell_key) = self.point_cell_keys[(start + i) as usize];
                    if cell_key != key {
                        break;
                    }
                    indices.push(point_idx);
                }
            }
        }
        indices
    }
}
