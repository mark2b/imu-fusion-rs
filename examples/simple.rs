use imu_fusion::FusionAhrsSettings;

const SAMPLE_RATE_HZ: u32 = 100;

fn main() {
    let ahrs_settings = FusionAhrsSettings::new();
    let mut fusion = imu_fusion::Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    loop {
        let gyr = imu_fusion::FusionVector::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
        let acc = imu_fusion::FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g
        fusion.update_no_mag(gyr, acc, 1.0 / SAMPLE_RATE_HZ as f32);
        let euler = fusion.euler();
        println!("Roll {}, Pitch {}, Yaw {}", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    }
}