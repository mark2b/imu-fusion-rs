use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
use imu_fusion::FusionConvention::NWU;

fn main() {
    let mut reader = csv::Reader::from_path("examples/raw_data/sensors_raw_data.csv").unwrap();
    let mut ahrs_settings = FusionAhrsSettings::new();
    ahrs_settings.convention = NWU;
    ahrs_settings.gain = 0.5f32;
    ahrs_settings.acceleration_rejection = 10.0f32;
    ahrs_settings.magnetic_rejection = 20.0f32;

    let mut fusion = Fusion::new(25, ahrs_settings);

    for result in reader.deserialize() {
        let record: [f32; 10] = result.unwrap();
        let dt = record[0];
        let gyr = FusionVector::new(record[1], record[2], record[3]);
        let acc = FusionVector::new(record[4], record[5], record[6]);
        let mag = FusionVector::new(record[7], record[8], record[9]);

        fusion.update(gyr, acc, mag, dt);
        let euler = fusion.euler();
        let earth_acc = fusion.earth_acc();
        unsafe {
            // println!("{}, {},{},{},{}", dt, q.data[0], q.data[1], q.data[2], q.data[3]);
            // let euler = q.euler();
            println!("{}, {}, {} {}, {}, {} {}", dt, euler.angle.yaw, euler.angle.pitch, euler.angle.roll, earth_acc.axis.x, earth_acc.axis.y, earth_acc.axis.z);
        }
    }
}