#![no_std]

mod fusion_vector_impl;
mod fusion_quaternion_impl;
mod fusion_matrix_impl;
mod fusion_euler_impl;
mod fusion_impl;
mod fusion_ahrs_impl;
mod fusion_gyr_offset_impl;
mod nalgebra;

pub enum FusionConvention {
    /* North-West-Up */
    NWU,
    /* East-North-Up */
    ENU,
    /* East-North-Up */
    NED,
}

pub struct Fusion {
    pub(crate) gyr_misalignment: FusionMatrix,
    pub(crate) gyr_sensitivity: FusionVector,
    pub(crate) gyr_offset: FusionVector,
    pub(crate) acc_misalignment: FusionMatrix,
    pub(crate) acc_sensitivity: FusionVector,
    pub(crate) acc_offset: FusionVector,
    pub(crate) soft_iron_matrix: FusionMatrix,
    pub(crate) hard_iron_offset: FusionVector,
    pub(crate) ahrs: FusionAhrs,
    pub(crate) offset: FusionGyrOffset,
    pub(crate) last_dt: f32,
}

pub struct FusionAhrs {
    settings: FusionAhrsSettings,
    quaternion: FusionQuaternion,
    acc: FusionVector,
    initialising: bool,
    ramped_gain: f32,
    ramped_gain_step: f32,
    angular_rate_recovery: bool,
    half_accelerometer_feedback: FusionVector,
    half_magnetometer_feedback: FusionVector,
    accelerometer_ignored: bool,
    acceleration_recovery_trigger: i32,
    acceleration_recovery_timeout: i32,
    magnetometer_ignored: bool,
    magnetic_recovery_trigger: i32,
    magnetic_recovery_timeout: i32,
}

pub struct FusionAhrsSettings {
    pub convention: FusionConvention,
    pub gain: f32,
    pub gyroscope_range: f32,
    pub acceleration_rejection: f32,
    pub magnetic_rejection: f32,
    pub recovery_trigger_period: i32,
}

#[derive(Copy, Clone)]
pub struct Axis {
    pub(crate) x: f32,
    pub(crate) y: f32,
    pub(crate) z: f32,
}

#[derive(Copy, Clone)]
struct Axis2 {
    x: Axis,
    y: Axis,
    z: Axis,
}

#[derive(Copy, Clone)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Copy, Clone)]
pub struct Angle {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Copy, Clone)]
#[allow(dead_code)]
pub union FusionVector {
    pub(crate) data: [f32; 3],
    pub axis: Axis,
}

#[derive(Copy, Clone)]
#[allow(dead_code)]
pub union FusionMatrix {
    data: [[f32; 3]; 3],
    element: Axis2,
}

#[derive(Copy, Clone)]
#[allow(dead_code)]
pub union FusionQuaternion {
    pub data: [f32; 4],
    pub element: Quaternion,
}

#[derive(Copy, Clone)]
#[allow(dead_code)]
pub struct FusionEuler {
    pub angle: Angle,
}

pub struct FusionGyrOffset {
    filter_coefficient: f32,
    timeout: u32,
    timer: u32,
    gyroscope_offset: FusionVector,
}

// Timeout in seconds.
const TIMEOUT: u32 = 5;

// Cutoff frequency in Hz.
const CUTOFF_FREQUENCY: f32 = 0.02f32;

// Threshold in degrees per second.
const THRESHOLD: f32 = 3f32;

fn fusion_degrees_to_radians(degrees: f32) -> f32 {
    degrees * (core::f32::consts::PI / 180.0f32)
}

fn fusion_radians_to_degrees(radians: f32) -> f32 {
    radians * (180.0f32 / core::f32::consts::PI)
}

fn asin_safe(value: f32) -> f32 {
    use libm::{asinf};
    if value <= -1.0f32 {
        return core::f32::consts::PI / -2.0f32;
    }
    if value >= 1.0f32 {
        return core::f32::consts::PI / 2.0f32;
    }
    asinf(value)
}

fn fusion_fast_inverse_sqrt(x: f32) -> f32 {
    union Union32 {
        f: f32,
        i: i32,
    }

    let mut union32 = Union32 { f: x };
    unsafe {
        union32.i = 0x5F1F1412 - (union32.i >> 1);
        union32.f * (1.69000231f32 - 0.714158168f32 * x * union32.f * union32.f)
    }
}


#[test]
fn fusion_fast_inverse_sqrt_test() {
    use libm::{fabsf};
    let result = fusion_fast_inverse_sqrt(9.0f32);
    let actual = 1f32 / result;
    let expected = 3f32;
    assert!(fabsf(actual - expected) < 0.01f32);
}
