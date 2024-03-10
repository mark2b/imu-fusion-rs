use imu_fusion::FusionAhrsSettings;
use std::time::{self, Instant};

const SAMPLE_RATE_HZ: u32 = 100;

fn main() {
    let ahrs_settings = FusionAhrsSettings::new();
    let mut fusion = imu_fusion::Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    let mut last_timestamp: time::Instant = time::Instant::now();
    loop {
        let gyr = imu_fusion::FusionVector::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
        let acc = imu_fusion::FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g
        
        // obtain a timestamp since the start of measurements as f32 in seconds
        let delta_t = last_timestamp.elapsed().as_secs_f32();

        fusion.update_no_mag_by_duration_seconds(gyr, acc, delta_t);
        let euler = fusion.euler();
        println!("Roll {}, Pitch {}, Yaw {}", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
        last_timestamp = Instant::now();
    }
}