use crate::FusionVector;
use nalgebra::{Vector3};

impl Into<FusionVector> for Vector3<f32> {
    fn into(self) -> FusionVector {
        FusionVector {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}
