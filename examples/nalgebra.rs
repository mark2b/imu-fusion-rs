use imu_fusion::FusionAhrsSettings;
use nalgebra::Vector3;
use std::time;

const SAMPLE_RATE_HZ: u32 = 100;

fn main() {
    let ahrs_settings = FusionAhrsSettings::new();
    let mut fusion = imu_fusion::Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    let start_time: time::Instant = time::Instant::now();
    loop {
        let gyr = Vector3::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
        let acc = Vector3::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g

        // obtain a timestamp since the start of measurements as f32 in seconds
        let timestamp = start_time.elapsed().as_secs_f32();

        fusion.update_no_mag(gyr.into(), acc.into(), timestamp);
        let euler = fusion.euler();
        println!("Roll {}, Pitch {}, Yaw {}", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    }
}