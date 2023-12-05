use core::ops;
#[cfg(feature = "fusion-use-normal-sqrt")]
use libm::sqrtf;

use crate::{fusion_fast_inverse_sqrt, FusionVector};

impl FusionVector {
    pub fn get(&self, x: &mut f32, y: &mut f32, z: &mut f32) {
        *x = self.x;
        *y = self.y;
        *z = self.z;
    }
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            x,
            y,
            z,
        }
    }
    pub fn zero() -> Self {
        const VALUE: FusionVector =
            FusionVector {
                x: 0.0f32,
                y: 0.0f32,
                z: 0.0f32,
            };
        VALUE
    }

    pub fn ones() -> Self {
        const VALUE: FusionVector =
            FusionVector {
                x: 1.0f32,
                y: 1.0f32,
                z: 1.0f32,
            };
        VALUE
    }

    pub fn is_zero(&self) -> bool {
        self.x == 0.0f32 && self.y == 0.0f32 && self.z == 0.0f32
    }
    pub fn sum(&self) -> f32 {
        self.x + self.y + self.z
    }

    pub fn cross_product(&self, rhs: &Self) -> Self {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    pub fn dot_product(&self, rhs: &Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn magnitude_squared(&self, rhs: &Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn magnitude(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn normalize(&self) -> Self {
        #[cfg(feature = "fusion-use-normal-sqrt")]
        {
            *self * (1.0f32 / sqrtf(self.magnitude()))
        }
        #[cfg(not(feature = "fusion-use-normal-sqrt"))]
        {
            *self * fusion_fast_inverse_sqrt(self.magnitude())
        }
    }
}

impl ops::Add for FusionVector {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl ops::Sub for FusionVector {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl ops::AddAssign for FusionVector {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl ops::SubAssign for FusionVector {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl ops::Mul for FusionVector {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }
}

impl ops::Mul<f32> for FusionVector {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl ops::MulAssign for FusionVector {
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
        self.z *= rhs.z;
    }
}

#[test]
fn add_test() {
    let a = FusionVector {
        x: 1.0f32,
        y: 2.0f32,
        z: 3.0f32,
    };
    let b = FusionVector {
        x: 4.0f32,
        y: 5.0f32,
        z: 6.0f32,
    };
    let c = a + b;
    assert_eq!(c.x, 5.0f32);
    assert_eq!(c.y, 7.0f32);
    assert_eq!(c.z, 9.0f32);
}