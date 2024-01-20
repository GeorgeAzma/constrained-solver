use std::ops;

#[derive(Clone, Copy, Default, Debug)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl ops::Add<Vec2> for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl ops::Add<f32> for Vec2 {
    type Output = Self;
    fn add(self, rhs: f32) -> Self {
        Self {
            x: self.x + rhs,
            y: self.y + rhs,
        }
    }
}

impl ops::AddAssign<Vec2> for Vec2 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl ops::AddAssign<f32> for Vec2 {
    fn add_assign(&mut self, rhs: f32) {
        self.x += rhs;
        self.y += rhs;
    }
}

impl ops::Sub<Vec2> for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl ops::Sub<f32> for Vec2 {
    type Output = Self;
    fn sub(self, rhs: f32) -> Self {
        Self {
            x: self.x - rhs,
            y: self.y - rhs,
        }
    }
}

impl ops::SubAssign<Vec2> for Vec2 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl ops::SubAssign<f32> for Vec2 {
    fn sub_assign(&mut self, rhs: f32) {
        self.x -= rhs;
        self.y -= rhs;
    }
}

impl ops::Mul<Vec2> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
        }
    }
}

impl ops::Mul<f32> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl ops::MulAssign<Vec2> for Vec2 {
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
    }
}

impl ops::MulAssign<f32> for Vec2 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl ops::Div<Vec2> for Vec2 {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        Self {
            x: self.x / rhs.x,
            y: self.y / rhs.y,
        }
    }
}

impl ops::Div<f32> for Vec2 {
    type Output = Self;
    fn div(self, rhs: f32) -> Self {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl ops::DivAssign<Vec2> for Vec2 {
    fn div_assign(&mut self, rhs: Self) {
        self.x /= rhs.x;
        self.y /= rhs.y;
    }
}

impl ops::DivAssign<f32> for Vec2 {
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
    }
}

impl ops::Neg for Vec2 {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

trait Funcs {
    fn step(self, b: Self) -> Self;
    fn rand(self) -> f32;
    fn noise(self) -> f32;
    fn fbm(self, oct: u32) -> f32;
    fn smooth(self) -> Self;
    fn lerp(self, other: Self, k: Self) -> Self;
    fn slerp(self, b: Self, k: Self) -> Self;
    fn sstep(self, b: Self, k: Self) -> Self;
}

impl Funcs for f32 {
    fn step(self, b: Self) -> Self {
        (b > self) as i32 as f32
    }

    fn rand(self) -> f32 {
        let mut x: u32 = unsafe { std::mem::transmute(self) };
        x ^= x >> 16;
        x = x.wrapping_mul(0x7feb352d);
        x ^= x >> 15;
        x = x.wrapping_mul(0x846ca68b);
        x ^= x >> 16;
        x as f32 / std::u32::MAX as f32
    }

    fn noise(self) -> f32 {
        let fl = self.floor();
        return fl.rand().lerp((fl + 1.0).rand(), self.fract());
    }

    fn fbm(self, oct: u32) -> f32 {
        let mut s = 0.0;
        let mut m = 0.0;
        let mut a = 0.5;
        let mut p = self;

        for _ in 0..oct {
            s += a * p.noise();
            m += a;
            a *= 0.5;
            p *= 2.0;
        }
        return s / m;
    }

    fn smooth(self) -> Self {
        self * self * (3.0 - 2.0 * self)
    }

    fn lerp(self, other: Self, k: Self) -> Self {
        self + (other - self) * k
    }

    fn slerp(self, b: Self, k: Self) -> Self {
        self.lerp(b, k.smooth())
    }

    fn sstep(self, e0: Self, e1: Self) -> Self {
        ((self - e0) / (e1 - e0)).clamp(0.0, 1.0).smooth()
    }
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };
    pub const ONE: Self = Self { x: 1.0, y: 1.0 };
    pub const NEG_ONE: Self = Self { x: -1.0, y: -1.0 };
    pub const X: Self = Self { x: 1.0, y: 0.0 };
    pub const Y: Self = Self { x: 0.0, y: 1.0 };
    pub const NEG_X: Self = Self { x: -1.0, y: 0.0 };
    pub const NEG_Y: Self = Self { x: 0.0, y: -1.0 };

    pub fn splat(v: f32) -> Self {
        Self::new(v, v)
    }

    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn angle(a: f32) -> Self {
        Self::new(a.cos(), a.sin())
    }

    pub fn dot(&self, rhs: &Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    pub fn len2(&self) -> f32 {
        self.dot(self)
    }

    pub fn len(&self) -> f32 {
        self.len2().sqrt()
    }

    pub fn norm(&self) -> Self {
        self.clone() / self.len()
    }

    pub fn dist2(&self, rhs: &Self) -> f32 {
        (rhs.clone() - *self).len2()
    }

    pub fn dist(&self, rhs: &Self) -> f32 {
        (rhs.clone() - *self).len()
    }

    pub fn sign(&self) -> Self {
        Self::new(self.x.signum(), self.y.signum())
    }

    pub fn abs(&self) -> Self {
        Self::new(self.x.abs(), self.y.abs())
    }

    pub fn floor(&self) -> Self {
        Self::new(self.x.floor(), self.y.floor())
    }

    pub fn round(&self) -> Self {
        Self::new(self.x.round(), self.y.round())
    }

    pub fn ceil(&self) -> Self {
        Self::new(self.x.ceil(), self.y.ceil())
    }

    pub fn fract(&self) -> Self {
        Self::new(self.x.fract(), self.y.fract())
    }

    pub fn exp(&self) -> Self {
        Self::new(self.x.exp(), self.y.exp())
    }

    pub fn pow(&self, p: f32) -> Self {
        Self::new(self.x.powf(p), self.y.powf(p))
    }

    pub fn lerp(&self, rhs: &Self, k: f32) -> Self {
        Self::new(self.x.lerp(rhs.x, k), self.y.lerp(rhs.y, k))
    }

    pub fn slerp(&self, rhs: &Self, k: &Self) -> Self {
        Self::new(self.x.slerp(rhs.x, k.x), self.y.slerp(rhs.y, k.y))
    }

    pub fn sstep(self, e0: Self, e1: Self) -> Self {
        Self::new(self.x.sstep(e0.x, e1.x), self.y.sstep(e0.y, e1.y))
    }

    pub fn step(&self, rhs: &Self) -> Self {
        Self::new(self.x.step(rhs.x), self.y.step(rhs.y))
    }

    pub fn angle_between(&self, rhs: &Self) -> f32 {
        self.norm().dot(&rhs.norm())
    }

    pub fn cross(&self) -> Self {
        Self::new(-self.y, self.x)
    }

    pub fn max(&self, rhs: &Self) -> Self {
        Self::new(self.x.max(rhs.x), self.y.max(rhs.y))
    }

    pub fn max_elem(&self) -> f32 {
        self.x.max(self.y)
    }

    pub fn min(&self, rhs: &Self) -> Self {
        Self::new(self.x.min(rhs.x), self.y.min(rhs.y))
    }

    pub fn min_elem(&self) -> f32 {
        self.x.min(self.y)
    }

    pub fn sin(&self) -> Self {
        Self::new(self.x.sin(), self.y.sin())
    }

    pub fn cos(&self) -> Self {
        Self::new(self.x.cos(), self.y.cos())
    }

    pub fn atan2(&self) -> f32 {
        self.y.atan2(self.x)
    }

    pub fn clamp(&self, min: &Self, max: &Self) -> Self {
        Self::new(self.x.max(min.x).min(max.x), self.y.max(min.y).min(max.y))
    }
}
