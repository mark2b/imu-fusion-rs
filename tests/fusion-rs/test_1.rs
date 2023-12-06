#[cfg(test)]
mod tests {
    use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
    use imu_fusion::FusionConvention::NWU;

    #[test]
    fn test1() {
        let mut reader = csv::Reader::from_path("tests/fusion_in.csv").unwrap();
        let mut writer = csv::Writer::from_path("tests/fusion_rs_out.csv").unwrap();
        let mut ahrs_settings = FusionAhrsSettings::new();
        ahrs_settings.convention = NWU;
        ahrs_settings.gain = 0.5f32;
        ahrs_settings.acc_rejection = 10.0f32;
        ahrs_settings.mag_rejection = 20.0f32;

        let mut fusion = Fusion::new(25, ahrs_settings);
        writer.write_record(&["dt", "euler_yaw", "euler_pitch", "euler_roll", "earth_acc_x", "earth_acc_y", "earth_acc_z", "q_w", "q_x", "q_y", "q_z"]).unwrap();
        let (mut acc_x, mut acc_y, mut acc_z) = (0f32, 0f32, 0f32);
        for result in reader.deserialize() {
            let record: [f32; 10] = result.unwrap();
            let dt = record[0];
            let gyr = FusionVector::new(record[1], record[2], record[3]);
            let acc = FusionVector::new(record[4], record[5], record[6]);
            let mag = FusionVector::new(record[7], record[8], record[9]);

            fusion.update(gyr, acc, mag, dt);
            let q = fusion.quaternion();
            let euler = fusion.euler();
            let earth_acc = fusion.earth_acc();
            earth_acc.get(&mut acc_x, &mut acc_y, &mut acc_z);
            writer.write_record(&[format!("{:.8}", dt), format!("{:.8}", euler.angle.yaw), format!("{:.8}", euler.angle.pitch), format!("{:.8}", euler.angle.roll), format!("{:.8}", acc_x), format!("{:.8}", acc_y), format!("{:.8}", acc_z), format!("{:.8}", q.w), format!("{:.8}", q.x), format!("{:.8}", q.y), format!("{:.8}", q.z)]).unwrap();
        }
        _ = writer.flush();
        compare_results();
    }

    fn compare_results() {
        let mut reader_c = csv::Reader::from_path("tests/fusion_c_out.csv").unwrap();
        let mut reader_rs = csv::Reader::from_path("tests/fusion_rs_out.csv").unwrap();
        loop {
            let record_c = reader_c.deserialize::<[f32; 3]>().next();
            let record_rs = reader_rs.deserialize::<[f32; 3]>().next();
            match record_c {
                Some(Ok(record_c)) => {
                    match record_rs {
                        Some(Ok(record_rs)) => {
                            for i in 0..record_c.len() {
                                assert!((record_c[i] - record_rs[i]).abs() < 0.0001f32);
                            }
                        }
                        _ => panic!("Error reading [rs] record"),
                    }
                }
                None => break,
                _ => panic!("Error reading [c] record"),
            }
        }
    }
}

