use core::ops;
#[allow(unused_imports)]
use libm::{asinf, atan2f, sqrtf};
use crate::{Angle, asin_safe, fusion_fast_inverse_sqrt, fusion_radians_to_degrees, FusionEuler, FusionMatrix, FusionQuaternion, FusionVector};

impl FusionQuaternion {
    pub fn identity() -> Self {
        const VALUE: FusionQuaternion =
            FusionQuaternion {
                w: 1.0f32,
                x: 0.0f32,
                y: 0.0f32,
                z: 0.0f32,
            };
        VALUE
    }

    pub fn normalize(&self) -> Self {
        #[cfg(feature = "fusion-use-normal-sqrt")]
        {
            *self * 1.0f32 / sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        }
        #[cfg(not(feature = "fusion-use-normal-sqrt"))]
        {
            *self * fusion_fast_inverse_sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        }
    }

    pub fn euler(self) -> FusionEuler {
        // calculate common terms to avoid repeated operations
        let q = self;
        let half_minus_qy_squared = 0.5f32 - q.y * q.y;
        let roll = fusion_radians_to_degrees(atan2f(q.w * q.x + q.y * q.z, half_minus_qy_squared - q.x * q.x));
        let pitch = fusion_radians_to_degrees(asin_safe(2.0f32 * (q.w * q.y - q.z * q.x)));
        let yaw = fusion_radians_to_degrees(atan2f(q.w * q.z + q.x * q.y, half_minus_qy_squared - q.z * q.z));

        FusionEuler {
            angle: Angle {
                roll,
                pitch,
                yaw,
            }
        }
    }

    pub fn rotation(self) -> FusionMatrix {
        // calculate common terms to avoid repeated operations
        let qwqw = self.w * self.w;
        let qwqx = self.w * self.x;
        let qwqy = self.w * self.y;
        let qwqz = self.w * self.z;
        let qxqx = self.x * self.x;
        let qxqy = self.x * self.y;
        let qxqz = self.x * self.z;
        let qyqy = self.y * self.y;
        let qyqz = self.y * self.z;
        let qzqz = self.z * self.z;
        FusionMatrix {
            xx: 2.0f32 * (qwqw - 0.5f32 + qxqx),
            xy: 2.0f32 * (qxqy - qwqz),
            xz: 2.0f32 * (qxqz + qwqy),
            yx: 2.0f32 * (qxqy + qwqz),
            yy: 2.0f32 * (qwqw - 0.5f32 + qyqy),
            yz: 2.0f32 * (qyqz - qwqx),
            zx: 2.0f32 * (qxqz - qwqy),
            zy: 2.0f32 * (qyqz + qwqx),
            zz: 2.0f32 * (qwqw - 0.5f32 + qzqz),

        }
    }
}

impl ops::Add for FusionQuaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl ops::Mul for FusionQuaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl ops::Mul<FusionVector> for FusionQuaternion {
    type Output = Self;
    fn mul(self, rhs: FusionVector) -> Self::Output {
        Self {
            w: -self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x,
        }
    }
}

impl ops::Mul<f32> for FusionQuaternion {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            w: self.w * rhs,
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

