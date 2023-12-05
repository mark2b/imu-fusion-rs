use core::ops;
#[cfg(feature = "fusion-use-normal-sqrt")]
use libm::sqrtf;

use crate::{Axis, fusion_fast_inverse_sqrt, FusionVector};

impl FusionVector {
    pub fn get(&self, x: &mut f32, y: &mut f32, z: &mut f32) {
        unsafe {
            *x = self.axis.x;
            *y = self.axis.y;
            *z = self.axis.z;
        };
    }
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            axis: Axis {
                x,
                y,
                z,
            }
        }
    }
    pub fn zero() -> Self {
        const VALUE: FusionVector =
            FusionVector {
                axis: Axis {
                    x: 0.0f32,
                    y: 0.0f32,
                    z: 0.0f32,
                }
            };
        VALUE
    }

    pub fn ones() -> Self {
        const VALUE: FusionVector =
            FusionVector {
                axis: Axis {
                    x: 1.0f32,
                    y: 1.0f32,
                    z: 1.0f32,
                }
            };
        VALUE
    }

    pub fn is_zero(&self) -> bool {
        unsafe {
            self.axis.x == 0.0f32 && self.axis.y == 0.0f32 && self.axis.z == 0.0f32
        }
    }
    pub fn sum(&self) -> f32 {
        unsafe {
            self.axis.x + self.axis.y + self.axis.z
        }
    }

    pub fn cross_product(&self, rhs: &Self) -> Self {
        unsafe {
            Self {
                axis: Axis {
                    x: self.axis.y * rhs.axis.z - self.axis.z * rhs.axis.y,
                    y: self.axis.z * rhs.axis.x - self.axis.x * rhs.axis.z,
                    z: self.axis.x * rhs.axis.y - self.axis.y * rhs.axis.x,
                }
            }
        }
    }

    pub fn dot_product(&self, rhs: &Self) -> f32 {
        unsafe {
            self.axis.x * rhs.axis.x + self.axis.y * rhs.axis.y + self.axis.z * rhs.axis.z
        }
    }

    pub fn magnitude_squared(&self, rhs: &Self) -> f32 {
        unsafe {
            self.axis.x * rhs.axis.x + self.axis.y * rhs.axis.y + self.axis.z * rhs.axis.z
        }
    }

    pub fn magnitude(&self) -> f32 {
        unsafe {
            self.axis.x * self.axis.x + self.axis.y * self.axis.y + self.axis.z * self.axis.z
        }
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
        unsafe {
            Self {
                axis: Axis {
                    x: self.axis.x + rhs.axis.x,
                    y: self.axis.y + rhs.axis.y,
                    z: self.axis.z + rhs.axis.z,
                }
            }
        }
    }
}

impl ops::Sub for FusionVector {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        unsafe {
            Self {
                axis: Axis {
                    x: self.axis.x - rhs.axis.x,
                    y: self.axis.y - rhs.axis.y,
                    z: self.axis.z - rhs.axis.z,
                }
            }
        }
    }
}

impl ops::AddAssign for FusionVector {
    fn add_assign(&mut self, rhs: Self) {
        unsafe {
            self.axis.x += rhs.axis.x;
            self.axis.y += rhs.axis.y;
            self.axis.z += rhs.axis.z;
        }
    }
}

impl ops::SubAssign for FusionVector {
    fn sub_assign(&mut self, rhs: Self) {
        unsafe {
            self.axis.x -= rhs.axis.x;
            self.axis.y -= rhs.axis.y;
            self.axis.z -= rhs.axis.z;
        }
    }
}

impl ops::Mul for FusionVector {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        unsafe {
            Self {
                axis: Axis {
                    x: self.axis.x * rhs.axis.x,
                    y: self.axis.y * rhs.axis.y,
                    z: self.axis.z * rhs.axis.z,
                }
            }
        }
    }
}

impl ops::Mul<f32> for FusionVector {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        unsafe {
            Self {
                axis: Axis {
                    x: self.axis.x * rhs,
                    y: self.axis.y * rhs,
                    z: self.axis.z * rhs,
                }
            }
        }
    }
}

impl ops::MulAssign for FusionVector {
    fn mul_assign(&mut self, rhs: Self) {
        unsafe {
            self.axis.x *= rhs.axis.x;
            self.axis.y *= rhs.axis.y;
            self.axis.z *= rhs.axis.z;
        }
    }
}

#[test]
fn add_test() {
    let a = FusionVector {
        axis: Axis {
            x: 1.0f32,
            y: 2.0f32,
            z: 3.0f32,
        }
    };
    let b = FusionVector {
        axis: Axis {
            x: 4.0f32,
            y: 5.0f32,
            z: 6.0f32,
        }
    };
    let c = a + b;
    unsafe {
        assert_eq!(c.axis.x, 5.0f32);
        assert_eq!(c.axis.y, 7.0f32);
        assert_eq!(c.axis.z, 9.0f32);
    }
}