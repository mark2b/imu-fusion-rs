use core::ops;
use crate::{Axis, Axis2, FusionMatrix, FusionQuaternion, FusionVector};

impl FusionMatrix {
    pub fn identity() -> Self {
        const VALUE: FusionMatrix =
            FusionMatrix {
                data: [[1.0f32, 0.0f32, 0.0f32], [0.0f32, 1.0f32, 0.0f32], [0.0f32, 0.0f32, 1.0f32]],
            };
        VALUE
    }
}

impl ops::Mul<FusionVector> for FusionMatrix {
    type Output = FusionVector;
    fn mul(self, rhs: FusionVector) -> Self::Output {
        unsafe {
            FusionVector {
                axis: Axis {
                    x: self.element.x.x * rhs.axis.x + self.element.x.y * rhs.axis.y + self.element.x.z * rhs.axis.z,
                    y: self.element.y.x * rhs.axis.x + self.element.y.y * rhs.axis.y + self.element.y.z * rhs.axis.z,
                    z: self.element.z.x * rhs.axis.x + self.element.z.y * rhs.axis.y + self.element.z.z * rhs.axis.z,
                }
            }
        }
    }
}

impl From<FusionQuaternion> for FusionMatrix {
    fn from(q: FusionQuaternion) -> Self {
        unsafe {
            let qwqw = q.element.w * q.element.w;
            let qwqx = q.element.w * q.element.x;
            let qwqy = q.element.w * q.element.y;
            let qwqz = q.element.w * q.element.z;
            let qxqx = q.element.x * q.element.x;
            let qxqy = q.element.x * q.element.y;
            let qxqz = q.element.x * q.element.z;
            let qyqy = q.element.y * q.element.y;
            let qyqz = q.element.y * q.element.z;
            let qzqz = q.element.z * q.element.z;
            Self {
                element: Axis2 {
                    x: Axis {
                        x: 2.0f32 * (qwqw - 0.5f32 + qxqx),
                        y: 2.0f32 * (qxqy - qwqz),
                        z: 2.0f32 * (qxqz + qwqy),
                    },
                    y: Axis {
                        x: 2.0f32 * (qxqy + qwqz),
                        y: 2.0f32 * (qwqw - 0.5f32 + qyqy),
                        z: 2.0f32 * (qyqz - qwqx),
                    },
                    z: Axis {
                        x: 2.0f32 * (qxqz - qwqy),
                        y: 2.0f32 * (qyqz + qwqx),
                        z: 2.0f32 * (qwqw - 0.5f32 + qzqz),
                    },
                }
            }
        }
    }
}