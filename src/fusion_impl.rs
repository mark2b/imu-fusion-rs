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
            last_timestamp: 0f32,
        }
    }

    pub fn inertial_calibration(&self, uncalibrated: FusionVector, misalignment: FusionMatrix, sensitivity: FusionVector, offset: FusionVector) -> FusionVector {
        misalignment * ((uncalibrated - offset) * sensitivity)
    }

    pub fn magnetic_calibration(&self, uncalibrated: FusionVector, soft_iron_matrix: FusionMatrix, hard_iron_offset: FusionVector) -> FusionVector {
        soft_iron_matrix * (uncalibrated - hard_iron_offset)
    }

    /// Updates the AHRS algorithm based on gyroscope data in degrees/s and acceleration data in g force.
    ///
    /// The time is provided using an absolute timestamp in seconds since the first measurement.
    /// Note that if you provide unix timestamps, the precision of f32 will not be enough to correctly compute the time difference.
    ///
    /// # Examples
    /// ```no_run
    /// use imu_fusion::{Fusion, FusionVector, FusionAhrsSettings};
    ///
    /// const SAMPLE_RATE_HZ: u32 = 100;
    ///
    /// let ahrs_settings = FusionAhrsSettings::new();
    /// let mut fusion = imu_fusion::Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    /// let mut current_ts = 0f32;
    ///
    /// loop {
    ///     let gyr = FusionVector::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
    ///     let acc = FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g
    ///     fusion.update_no_mag(gyr, acc, current_ts);
    ///     current_ts += 1.0 / SAMPLE_RATE_HZ as f32
    /// }
    ///
    /// ```
    pub fn update_no_mag(&mut self, gyr: FusionVector, acc: FusionVector, timestamp: f32) {
        let delta_t = timestamp - self.last_timestamp;
        self.update_no_mag_by_duration_seconds(gyr, acc, delta_t);
        self.last_timestamp = timestamp;
    }

    /// Updates the AHRS algorithm based on gyroscope data in degrees/s and acceleration data in g force.
    ///
    /// The time is provided as a duration in seconds since the last measurement.
    /// Note that this won't increase the internal timestamp and using the timestamp version of update functions will produce incorrect results.
    ///
    /// # Examples
    /// ```no_run
    /// use imu_fusion::{Fusion, FusionVector, FusionAhrsSettings};
    /// use std::time::{Duration, Instant};
    ///
    /// const SAMPLE_RATE_HZ: u32 = 100;
    ///
    /// let ahrs_settings = FusionAhrsSettings::new();
    /// let mut fusion = imu_fusion::Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    /// let mut last_ts = Instant::now();
    ///
    /// loop {
    ///     let gyr = FusionVector::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
    ///     let acc = FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g
    ///     fusion.update_no_mag_by_duration_seconds(gyr, acc, last_ts.elapsed().as_secs_f32());
    ///     last_ts = Instant::now();
    /// }
    ///
    /// ```
    pub fn update_no_mag_by_duration_seconds(&mut self, gyr: FusionVector, acc: FusionVector, delta_t: f32) {
        // Apply calibration
        let mut gyr = self.inertial_calibration(gyr, self.gyr_misalignment, self.gyr_sensitivity, self.gyr_offset);
        let acc = self.inertial_calibration(acc, self.acc_misalignment, self.acc_sensitivity, self.acc_offset);

        // Update gyroscope offset correction algorithm
        gyr = self.offset.update(gyr);

        self.ahrs.update_no_mag(gyr, acc, delta_t);
    }

    /// Updates the AHRS algorithm based on gyroscope data in degrees/s, acceleration data in g force and a heading in degrees.
    ///
    /// The time is provided using an absolute timestamp in seconds since the first measurement.
    /// Note that if you provide unix timestamps, the precision of f32 will not be enough to correctly compute the time difference.
    ///
    /// # Examples
    /// ```no_run
    /// use imu_fusion::{Fusion, FusionVector, FusionAhrsSettings};
    ///
    /// const SAMPLE_RATE_HZ: u32 = 100;
    ///
    /// let ahrs_settings = FusionAhrsSettings::new();
    /// let mut fusion = Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    /// let mut current_ts = 0f32;
    ///
    /// loop {
    ///     let gyr = FusionVector::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
    ///     let acc = FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g
    ///     let external_heading = 0f32; // replace this with actual heading in degrees
    ///     fusion.update_external_heading(gyr, acc, external_heading, current_ts);
    ///     current_ts += 1.0 / SAMPLE_RATE_HZ as f32
    /// }
    ///
    /// ```
    pub fn update_external_heading(
        &mut self,
        gyr: FusionVector,
        acc: FusionVector,
        heading: f32,
        timestamp: f32,
    ) {
        let delta_t = timestamp - self.last_timestamp;
        self.update_external_heading_by_duration_seconds(gyr, acc, heading, delta_t);
        self.last_timestamp = timestamp;
    }

    /// Updates the AHRS algorithm based on gyroscope data in degrees/s, acceleration data in g force and a heading in degrees.
    ///
    /// The time is provided using a duration in seconds since the last measurement.
    /// Note that this won't increase the internal timestamp and using the timestamp version of update functions will produce incorrect results.
    ///
    /// # Examples
    /// ```no_run
    /// use imu_fusion::{Fusion, FusionVector, FusionAhrsSettings};
    /// use std::time::{Duration, Instant};
    ///
    /// const SAMPLE_RATE_HZ: u32 = 100;
    ///
    /// let ahrs_settings = FusionAhrsSettings::new();
    /// let mut fusion = Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    /// let mut last_ts = Instant::now();
    ///
    /// loop {
    ///     let gyr = FusionVector::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
    ///     let acc = FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g
    ///     let external_heading = 0f32; // replace this with actual heading in degrees
    ///     fusion.update_external_heading(gyr, acc, external_heading, last_ts.elapsed().as_secs_f32());
    ///     last_ts = Instant::now();
    /// }
    ///
    /// ```
    pub fn update_external_heading_by_duration_seconds(
        &mut self,
        gyr: FusionVector,
        acc: FusionVector,
        heading: f32,
        delta_t: f32,
    ) {
        // Apply calibration
        let mut gyr = self.inertial_calibration(gyr, self.gyr_misalignment, self.gyr_sensitivity, self.gyr_offset);
        let acc = self.inertial_calibration(acc, self.acc_misalignment, self.acc_sensitivity, self.acc_offset);

        // Update gyroscope offset correction algorithm
        gyr = self.offset.update(gyr);

        self.ahrs.update_external_heading(gyr, acc, heading, delta_t);
    }

    /// Updates the AHRS algorithm based on gyroscope data in degrees/s, acceleration data in g force and magnetic measurements in degrees.
    ///
    /// The time is provided using an absolute timestamp in seconds since the first measurement.
    /// Note that if you provide unix timestamps, the precision of f32 will not be enough to correctly compute the time difference.
    ///
    /// # Examples
    /// ```no_run
    /// use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
    ///
    /// const SAMPLE_RATE_HZ: u32 = 100;
    ///
    /// let ahrs_settings = FusionAhrsSettings::new();
    /// let mut fusion = Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    /// let mut current_ts = 0f32;
    ///
    /// loop {
    ///     let gyr = FusionVector::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
    ///     let acc = FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g
    ///     let mag = FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual magnetic measurement in degrees
    ///     fusion.update(gyr, acc, mag, current_ts);
    ///     current_ts += 1.0 / SAMPLE_RATE_HZ as f32
    /// }
    ///
    /// ```
    pub fn update(
        &mut self,
        gyr: FusionVector,
        acc: FusionVector,
        mag: FusionVector,
        timestamp: f32,
    ) {
        let delta_t = timestamp - self.last_timestamp;
        self.update_by_duration_seconds(gyr, acc, mag, delta_t);
        self.last_timestamp = timestamp;
    }

    /// Updates the AHRS algorithm based on gyroscope data in degrees/s, acceleration data in g force and magnetic measurements in degrees.
    ///
    /// The time is provided using a duration in seconds since the last measurement.
    /// Note that this won't increase the internal timestamp and using the timestamp version of update functions will produce incorrect results.
    /// # Examples
    /// ```no_run
    /// use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
    /// use std::time::{Duration, Instant};
    ///
    /// const SAMPLE_RATE_HZ: u32 = 100;
    ///
    /// let ahrs_settings = FusionAhrsSettings::new();
    /// let mut fusion = Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    /// let mut last_ts = Instant::now();
    ///
    /// loop {
    ///     let gyr = FusionVector::new(0f32, 0f32, 0f32); // replace this with actual gyroscope data in degrees/s
    ///     let acc = FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual accelerometer data in g
    ///     let mag = FusionVector::new(0f32, 0f32, 1.0f32); // replace this with actual magnetic measurement in degrees
    ///     fusion.update_by_duration_seconds(gyr, acc, mag, last_ts.elapsed().as_secs_f32());
    ///     last_ts = Instant::now();
    /// }
    ///
    /// ```
    pub fn update_by_duration_seconds(
        &mut self,
        gyr: FusionVector,
        acc: FusionVector,
        mag: FusionVector,
        delta_t: f32,
    ) {
        // Apply calibration
        let mut gyr = self.inertial_calibration(gyr, self.gyr_misalignment, self.gyr_sensitivity, self.gyr_offset);
        let acc = self.inertial_calibration(acc, self.acc_misalignment, self.acc_sensitivity, self.acc_offset);
        let mag = self.magnetic_calibration(mag, self.soft_iron_matrix, self.hard_iron_offset);

        // Update gyroscope offset correction algorithm
        gyr = self.offset.update(gyr);

        self.ahrs.update(gyr, acc, mag, delta_t);
    }

    /// Obtain euler angle current sensor position
    ///
    /// Euler angles are provided in degrees
    ///
    /// # Examples
    ///
    /// ```
    /// use imu_fusion::{Fusion, FusionAhrsSettings};
    ///
    /// const SAMPLE_RATE_HZ: u32 = 100;
    ///
    /// let ahrs_settings = FusionAhrsSettings::new();
    /// let mut fusion = Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    ///
    /// // ...update sensor values
    ///
    /// let euler = fusion.euler();
    /// println!("Roll {}, Pitch {}, Yaw {}", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    /// ```
    pub fn euler(&self) -> FusionEuler {
        self.ahrs.quaternion.euler()
    }

    /// Obtain acceleration of sensor in earth's frame of reference
    ///
    /// The values returned are provided in g force
    ///
    /// # Examples
    ///
    /// ```
    /// use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
    ///
    /// const SAMPLE_RATE_HZ: u32 = 100;
    ///
    /// let ahrs_settings = FusionAhrsSettings::new();
    /// let mut fusion = Fusion::new(SAMPLE_RATE_HZ, ahrs_settings);
    ///
    /// // ...update sensor values
    ///
    /// let acc = fusion.earth_acc();
    /// println!("x {}, y {}, z {}", acc.x, acc.y, acc.z);
    /// ```
    pub fn earth_acc(&self) -> FusionVector {
        self.ahrs.earth_acc()
    }

    pub fn quaternion(&self) -> FusionQuaternion {
        self.ahrs.quaternion
    }
}
