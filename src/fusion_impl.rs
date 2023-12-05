use crate::{Fusion, FusionAhrs, FusionAhrsSettings, FusionEuler, FusionGyrOffset, FusionMatrix, FusionQuaternion, FusionVector};

impl Fusion {
    pub fn new(sample_rate: u32, ahrs_settings: FusionAhrsSettings) -> Self {
        let mut ahrs = FusionAhrs::new();
        ahrs.update_settings(ahrs_settings);
        Self {
            gyr_misalignment: FusionMatrix::identity(),
            gyr_sensitivity: FusionVector::ones(),
            gyr_offset: FusionVector::zero(),
            acc_misalignment: FusionMatrix::identity(),
            acc_sensitivity: FusionVector::ones(),
            acc_offset: FusionVector::zero(),
            soft_iron_matrix: FusionMatrix::identity(),
            hard_iron_offset: FusionVector::zero(),
            ahrs,
            offset: FusionGyrOffset::new(sample_rate),
            last_dt: 0f32,
        }
    }

    pub fn inertial_calibration(&self, uncalibrated: FusionVector, misalignment: FusionMatrix, sensitivity: FusionVector, offset: FusionVector) -> FusionVector {
        misalignment * ((uncalibrated - offset) * sensitivity)
    }

    pub fn magnetic_calibration(&self, uncalibrated: FusionVector, soft_iron_matrix: FusionMatrix, hard_iron_offset: FusionVector) -> FusionVector {
        soft_iron_matrix * (uncalibrated - hard_iron_offset)
    }

    pub fn update_no_mag(&mut self, gyr: FusionVector, acc: FusionVector, dt: f32) {
        let delta_t = dt - self.last_dt;
        // Apply calibration
        let mut gyr = self.inertial_calibration(gyr, self.gyr_misalignment, self.gyr_sensitivity, self.gyr_offset);
        let acc = self.inertial_calibration(acc, self.acc_misalignment, self.acc_sensitivity, self.acc_offset);

        // Update gyroscope offset correction algorithm
        gyr = self.offset.update(gyr);

        self.ahrs.update_no_mag(gyr, acc, delta_t);
        self.last_dt = dt;
    }

    pub fn update_external_heading(&mut self, gyr: FusionVector, acc: FusionVector, heading: f32, dt: f32) {
        // Apply calibration
        let mut gyr = self.inertial_calibration(gyr, self.gyr_misalignment, self.gyr_sensitivity, self.gyr_offset);
        let acc = self.inertial_calibration(acc, self.acc_misalignment, self.acc_sensitivity, self.acc_offset);

        // Update gyroscope offset correction algorithm
        gyr = self.offset.update(gyr);

        self.ahrs.update_external_heading(gyr, acc, heading, dt);
        self.last_dt = dt;
    }

    pub fn update(&mut self, gyr: FusionVector, acc: FusionVector, mag: FusionVector, dt: f32) {
        let delta_t = dt - self.last_dt;
        // Apply calibration
        let mut gyr = self.inertial_calibration(gyr, self.gyr_misalignment, self.gyr_sensitivity, self.gyr_offset);
        let acc = self.inertial_calibration(acc, self.acc_misalignment, self.acc_sensitivity, self.acc_offset);
        let mag = self.magnetic_calibration(mag, self.soft_iron_matrix, self.hard_iron_offset);

        // Update gyroscope offset correction algorithm
        gyr = self.offset.update(gyr);

        self.ahrs.update(gyr, acc, mag, delta_t);
        self.last_dt = dt;
    }

    pub fn euler(&self) -> FusionEuler {
        self.ahrs.quaternion.euler()
    }

    pub fn earth_acc(&self) -> FusionVector {
        self.ahrs.earth_acc()
    }

    pub fn quaternion(&self) -> FusionQuaternion {
        self.ahrs.quaternion
    }
}
