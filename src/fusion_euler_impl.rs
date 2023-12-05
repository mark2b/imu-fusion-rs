use crate::{Angle, FusionEuler};

impl FusionEuler {
    pub fn zero() -> Self {
        const VALUE: FusionEuler =
            FusionEuler {
                angle: Angle {
                    roll: 0.0f32,
                    pitch: 0.0f32,
                    yaw: 0.0f32,
                }
            };
        VALUE
    }

    pub fn get_roll(&self) -> f32 {
        self.angle.roll
    }
    pub fn get_pitch(&self) -> f32 {
        self.angle.pitch
    }
    pub fn get_yaw(&self) -> f32 {
        self.angle.yaw
    }
}
