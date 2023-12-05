use core::ops;
#[allow(unused_imports)]
use libm::{asinf, atan2f, sqrtf};
use crate::{Angle, asin_safe, fusion_fast_inverse_sqrt, fusion_radians_to_degrees, FusionEuler, FusionMatrix, FusionQuaternion, FusionVector, Quaternion};

impl FusionQuaternion {
    pub fn identity() -> Self {
        const VALUE: FusionQuaternion =
            FusionQuaternion {
                data: [1.0f32, 0.0f32, 0.0f32, 0.0f32],
            };
        VALUE
    }

    pub fn normalize(&self) -> Self {
        unsafe {
            #[cfg(feature = "fusion-use-normal-sqrt")]
            {
                *self * 1.0f32 / sqrtf(self.element.w * self.element.w + self.element.x * self.element.x + self.element.y * self.element.y + self.element.z * self.element.z)
            }
            #[cfg(not(feature = "fusion-use-normal-sqrt"))]
            {
                *self * fusion_fast_inverse_sqrt(self.element.w * self.element.w + self.element.x * self.element.x + self.element.y * self.element.y + self.element.z * self.element.z)
            }
        }
    }

    pub fn euler(self) -> FusionEuler {
        unsafe {
            // calculate common terms to avoid repeated operations
            let q = self.element;
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
    }

    pub fn rotation(self) -> FusionMatrix {
        unsafe {
            // calculate common terms to avoid repeated operations
            let qwqw = self.element.w * self.element.w;
            let qwqx = self.element.w * self.element.x;
            let qwqy = self.element.w * self.element.y;
            let qwqz = self.element.w * self.element.z;
            let qxqx = self.element.x * self.element.x;
            let qxqy = self.element.x * self.element.y;
            let qxqz = self.element.x * self.element.z;
            let qyqy = self.element.y * self.element.y;
            let qyqz = self.element.y * self.element.z;
            let qzqz = self.element.z * self.element.z;
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
}

impl ops::Add for FusionQuaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        unsafe {
            Self {
                element: Quaternion {
                    w: self.element.w + rhs.element.w,
                    x: self.element.x + rhs.element.x,
                    y: self.element.y + rhs.element.y,
                    z: self.element.z + rhs.element.z,
                }
            }
        }
    }
}

impl ops::Mul for FusionQuaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        unsafe {
            Self {
                element: Quaternion {
                    w: self.element.w * rhs.element.w - self.element.x * rhs.element.x - self.element.y * rhs.element.y - self.element.z * rhs.element.z,
                    x: self.element.w * rhs.element.x + self.element.x * rhs.element.w + self.element.y * rhs.element.z - self.element.z * rhs.element.y,
                    y: self.element.w * rhs.element.y - self.element.x * rhs.element.z + self.element.y * rhs.element.w + self.element.z * rhs.element.x,
                    z: self.element.w * rhs.element.z + self.element.x * rhs.element.y - self.element.y * rhs.element.x + self.element.z * rhs.element.w,
                }
            }
        }
    }
}

impl ops::Mul<FusionVector> for FusionQuaternion {
    type Output = Self;
    fn mul(self, rhs: FusionVector) -> Self::Output {
        unsafe {
            Self {
                element: Quaternion {
                    w: -self.element.x * rhs.x - self.element.y * rhs.y - self.element.z * rhs.z,
                    x: self.element.w * rhs.x + self.element.y * rhs.z - self.element.z * rhs.y,
                    y: self.element.w * rhs.y - self.element.x * rhs.z + self.element.z * rhs.x,
                    z: self.element.w * rhs.z + self.element.x * rhs.y - self.element.y * rhs.x,
                }
            }
        }
    }
}

impl ops::Mul<f32> for FusionQuaternion {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        unsafe {
            Self {
                element: Quaternion {
                    w: self.element.w * rhs,
                    x: self.element.x * rhs,
                    y: self.element.y * rhs,
                    z: self.element.z * rhs,
                }
            }
        }
    }
}

