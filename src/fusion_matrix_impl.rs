use core::ops;
use crate::{FusionMatrix, FusionQuaternion, FusionVector};

impl FusionMatrix {
    pub fn new(xx: f32, xy: f32, xz: f32, yx: f32, yy: f32, yz: f32, zx: f32, zy: f32, zz: f32) -> Self {
        Self {
            xx,
            xy,
            xz,
            yx,
            yy,
            yz,
            zx,
            zy,
            zz,
        }
    }


    pub fn identity() -> Self {
        const VALUE: FusionMatrix =
            FusionMatrix {
                xx: 1.0f32,
                xy: 0.0f32,
                xz: 0.0f32,
                yx: 0.0f32,
                yy: 1.0f32,
                yz: 0.0f32,
                zx: 0.0f32,
                zy: 0.0f32,
                zz: 1.0f32,
            };
        VALUE
    }
}

impl ops::Mul<FusionVector> for FusionMatrix {
    type Output = FusionVector;
    fn mul(self, rhs: FusionVector) -> Self::Output {
        FusionVector {
            x: self.xx * rhs.x + self.xy * rhs.y + self.xz * rhs.z,
            y: self.yx * rhs.x + self.yy * rhs.y + self.yz * rhs.z,
            z: self.zx * rhs.x + self.zy * rhs.y + self.zz * rhs.z,
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
