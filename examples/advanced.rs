extern crate core;

use std::time;
use imu_fusion::{FusionAhrsSettings, FusionGyrOffset};

const SAMPLE_RATE_HZ: u32 = 100;

fn main() {
    // Define calibration (replace with actual calibration data if available)
    let gyr_misalignment = imu_fusion::FusionMatrix::new(1.0f32, 0.0f32, 0.0f32, 0.0f32, 1.0f32, 0.0f32, 0.0f32, 0.0f32, 1.0f32);
    let gyr_sensitivity = imu_fusion::FusionVector::new(1.0f32, 1.0f32, 1.0f32);
    let gyr_offset = imu_fusion::FusionVector::new(0.0f32, 0.0f32, 0.0f32);
    let acc_misalignment = imu_fusion::FusionMatrix::new(1.0f32, 0.0f32, 0.0f32, 0.0f32, 1.0f32, 0.0f32, 0.0f32, 0.0f32, 1.0f32);
    let acc_sensitivity = imu_fusion::FusionVector::new(1.0f32, 1.0f32, 1.0f32);
    let acc_offset = imu_fusion::FusionVector::new(0.0f32, 0.0f32, 0.0f32);
    let soft_iron_matrix = imu_fusion::FusionMatrix::new(1.0f32, 0.0f32, 0.0f32, 0.0f32, 1.0f32, 0.0f32, 0.0f32, 0.0f32, 1.0f32);
    let hard_iron_offset = imu_fusion::FusionVector::new(0.0f32, 0.0f32, 0.0f32);
    // Initialise algorithms
    let mut ahrs_settings = FusionAhrsSettings::new();

    // Set AHRS algorithm settings
    ahrs_settings.convention = imu_fusion::FusionConvention::NWU;
    ahrs_settings.gain = 0.5f32;
    ahrs_settings.gyr_range = 2000.0f32; // replace this with actual gyroscope range in degrees/s
    ahrs_settings.acc_rejection = 10.0f32;
    ahrs_settings.mag_rejection = 10.0f32;
    ahrs_settings.recovery_trigger_period = 5 * SAMPLE_RATE_HZ as i32;

    let mut fusion = imu_fusion::Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    let mut offset = FusionGyrOffset::new(SAMPLE_RATE_HZ);

    // This loop should repeat each time new sensors data is available
    let start_time = time::Instant::now();
    loop {
        // Acquire latest sensor data
        let gyr = imu_fusion::FusionVector::new(0.0f32, 0.0f32, 0.0f32); // replace this with actual gyroscope data in degrees/s
        let acc = imu_fusion::FusionVector::new(0.0f32, 0.0f32, 1.0f32); // replace this with actual accelerometer data in g
        let mag = imu_fusion::FusionVector::new(1.0f32, 0.0f32, 0.0f32); // replace this with actual magnetometer data in arbitrary units

        // Apply calibration
        let mut gyr = fusion.inertial_calibration(gyr, gyr_misalignment, gyr_sensitivity, gyr_offset);
        let acc = fusion.inertial_calibration(acc, acc_misalignment, acc_sensitivity, acc_offset);
        let mag = fusion.magnetic_calibration(mag, soft_iron_matrix, hard_iron_offset);

        // Update gyroscope offset correction algorithm
        gyr = offset.update(gyr);

        // obtain a timestamp since the start of measurements as f32 in seconds
        let timestamp = start_time.elapsed().as_secs_f32();

        // Update gyroscope AHRS algorithm
        fusion.update(gyr, acc, mag, timestamp);

        // Print algorithm outputs
        let euler = fusion.euler();
        let earth = fusion.earth_acc();
        println!("Roll {}, Pitch {}, Yaw {}, X {}, Y {}, Z {}", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, earth.x, earth.y, earth.z);
    }

}