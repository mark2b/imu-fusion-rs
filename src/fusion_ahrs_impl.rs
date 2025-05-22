use libm::{atan2f, cosf, fabsf, sinf};
use crate::{fusion_degrees_to_radians, FusionAhrs, FusionAhrsFlags, FusionAhrsSettings, FusionConvention, FusionQuaternion, FusionVector};
use crate::FusionConvention::NWU;

/**
 * Initial gain used during the initialisation.
 */
const INITIAL_GAIN: f32 = 10.0f32;
/**
 * Initialisation period in seconds.
 */
const INITIALISATION_PERIOD: f32 = 3.0f32;

impl FusionAhrs {
    pub fn new() -> Self {
        let settings = FusionAhrsSettings::new();
        let gain = settings.gain;
        let recovery_trigger_period = settings.recovery_trigger_period;
        Self {
            settings,
            quaternion: FusionQuaternion::identity(),
            acc: FusionVector::zero(),
            initialising: true,
            ramped_gain: INITIAL_GAIN,
            ramped_gain_step: (INITIAL_GAIN - gain) / INITIALISATION_PERIOD,
            angular_rate_recovery: false,
            half_accelerometer_feedback: FusionVector::zero(),
            half_magnetometer_feedback: FusionVector::zero(),
            accelerometer_ignored: false,
            acceleration_recovery_trigger: 0,
            acceleration_recovery_timeout: recovery_trigger_period,
            magnetometer_ignored: false,
            magnetic_recovery_trigger: 0,
            magnetic_recovery_timeout: recovery_trigger_period,
        }
    }

    pub fn update_settings(&mut self, settings: FusionAhrsSettings) {
        self.settings.convention = settings.convention;
        self.settings.gain = settings.gain;
        self.settings.gyr_range = if settings.gyr_range == 0.0f32 { f32::MAX } else { 0.98f32 * settings.gyr_range };
        self.settings.acc_rejection = if settings.acc_rejection == 0.0f32 {
            f32::MAX
        } else {
            let v = 0.5f32 * sinf(fusion_degrees_to_radians(settings.acc_rejection));
            v * v
        };
        self.settings.mag_rejection = if settings.mag_rejection == 0.0f32 {
            f32::MAX
        } else {
            let v = 0.5f32 * sinf(fusion_degrees_to_radians(settings.mag_rejection));
            v * v
        };
        self.settings.recovery_trigger_period = settings.recovery_trigger_period;
        self.acceleration_recovery_timeout = self.settings.recovery_trigger_period;
        self.magnetic_recovery_timeout = self.settings.recovery_trigger_period;
        if settings.gain == 0.0f32 || settings.recovery_trigger_period == 0 { // disable acceleration and magnetic rejection features if gain is zero
            self.settings.acc_rejection = f32::MAX;
            self.settings.mag_rejection = f32::MAX;
        }
        if !self.initialising {
            self.ramped_gain = settings.gain;
        }
        self.ramped_gain_step = (INITIAL_GAIN - self.settings.gain) / INITIALISATION_PERIOD;
    }

    pub fn update_no_mag(&mut self, gyr: FusionVector, acc: FusionVector, dt: f32) {
        self.update(gyr, acc, FusionVector::zero(), dt);
        // Zero heading during initialisation
        if self.initialising == true {
            self.set_heading(0.0f32);
        }
    }

    pub fn update_external_heading(&mut self, gyr: FusionVector, acc: FusionVector, heading: f32, dt: f32) {
        let q = self.quaternion;
        // Calculate roll
        let roll = atan2f(q.w * q.x + q.y * q.z, 0.5f32 - q.y * q.y - q.x * q.x);
        // Calculate magnetometer// Calculate magnetometer
        let heading_radians = fusion_degrees_to_radians(heading);
        let sin_heading_radians = sinf(heading_radians);
        let magnetometer = FusionVector {
            x: cosf(heading_radians),
            y: -1.0f32 * cosf(roll) * sin_heading_radians,
            z: sin_heading_radians * sinf(roll),
        };
        // Update AHRS algorithm
        self.update(gyr, acc, magnetometer, dt);
    }

    pub fn update(&mut self, gyr: FusionVector, acc: FusionVector, mag: FusionVector, dt: f32) {
        // Store accelerometer
        self.acc = acc;
        // Reinitialise if gyroscope range exceeded
        if fabsf(gyr.x) > self.settings.gyr_range || fabsf(gyr.y) > self.settings.gyr_range || fabsf(gyr.z) > self.settings.gyr_range {
            let quaternion = self.quaternion;
            self.reset();
            self.quaternion = quaternion;
            self.angular_rate_recovery = true;
        }
        // Ramp down gain during initialisation
        if self.initialising == true {
            self.ramped_gain -= self.ramped_gain_step * dt;
            if self.ramped_gain < self.settings.gain || self.settings.gain == 0.0f32 {
                self.ramped_gain = self.settings.gain;
                self.initialising = false;
                self.angular_rate_recovery = false;
            }
        }
        // Calculate direction of gravity indicated by algorithm
        let half_gravity = self.calculate_half_gravity();

        // Calculate accelerometer feedback
        let mut half_accelerometer_feedback = FusionVector::zero();
        self.accelerometer_ignored = true;
        if !self.acc.is_zero() {
            // Calculate accelerometer feedback scaled by 0.5
            self.half_accelerometer_feedback = self.feedback(self.acc.normalize(), half_gravity);
            // Don't ignore accelerometer if acceleration error below threshold
            if self.initialising == true || self.half_accelerometer_feedback.magnitude() <= self.settings.acc_rejection {
                self.accelerometer_ignored = false;
                self.acceleration_recovery_trigger -= 9;
            } else {
                self.acceleration_recovery_trigger += 1;
            }
            // Don't ignore accelerometer during acceleration recovery
            if self.acceleration_recovery_trigger > self.acceleration_recovery_timeout {
                self.acceleration_recovery_timeout = 0;
                self.accelerometer_ignored = false;
            } else {
                self.acceleration_recovery_timeout = self.settings.recovery_trigger_period;
            }
            self.acceleration_recovery_trigger = clamp(self.acceleration_recovery_trigger, 0, self.settings.recovery_trigger_period);
            // Apply accelerometer feedback
            if !self.accelerometer_ignored {
                half_accelerometer_feedback = self.half_accelerometer_feedback;
            }
        }
        // Calculate magnetometer feedback
        let mut half_magnetometer_feedback = FusionVector::zero();
        self.magnetometer_ignored = true;
        if !mag.is_zero() {
            // Calculate direction of magnetic field indicated by algorithm
            let half_magnetic = self.calculate_half_magnetic();
            // Calculate magnetometer feedback scaled by 0.5

            self.half_magnetometer_feedback = self.feedback(half_gravity.cross_product(&mag).normalize(), half_magnetic);
            // Don't ignore magnetometer if magnetic error below threshold
            if self.initialising == true || self.half_magnetometer_feedback.magnitude() <= self.settings.mag_rejection {
                self.magnetometer_ignored = false;
                self.magnetic_recovery_trigger -= 9;
            } else {
                self.magnetic_recovery_trigger += 1;
            }
            // Don't ignore magnetometer during magnetic recovery
            if self.magnetic_recovery_trigger > self.magnetic_recovery_timeout {
                self.magnetic_recovery_timeout = 0;
                self.magnetometer_ignored = false;
            } else {
                self.magnetic_recovery_timeout = self.settings.recovery_trigger_period;
            }
            self.magnetic_recovery_trigger = clamp(self.magnetic_recovery_trigger, 0, self.settings.recovery_trigger_period);
            // Apply magnetometer feedback
            if !self.magnetometer_ignored {
                half_magnetometer_feedback = self.half_magnetometer_feedback;
            }
        }

        // Convert gyroscope to radians per second scaled by 0.5
        let half_gyroscope = gyr * fusion_degrees_to_radians(0.5f32);
        // Apply feedback to gyroscope
        let adjusted_half_gyroscope = half_gyroscope + (half_accelerometer_feedback + half_magnetometer_feedback) * self.ramped_gain;
        // Integrate rate of change of quaternion
        self.quaternion = self.quaternion + self.quaternion * (adjusted_half_gyroscope * dt);
        // Normalise quaternion
        self.quaternion = self.quaternion.normalize();
    }


    fn feedback(&self, sensor: FusionVector, reference: FusionVector) -> FusionVector {
        if sensor.dot_product(&reference) < 0.0f32 { // if error is >90 degrees
            sensor.cross_product(&reference).normalize()
        } else {
            sensor.cross_product(&reference)
        }
    }

    fn set_heading(&mut self, heading: f32) {
        let q = self.quaternion;
        let yaw = atan2f(q.w * q.z + q.x * q.y, 0.5f32 - q.y * q.y - q.z * q.z);
        let half_yaw_minus_heading = 0.5f32 * (yaw - fusion_degrees_to_radians(heading));
        let rotation = FusionQuaternion {
            w: cosf(half_yaw_minus_heading),
            x: 0.0f32,
            y: 0.0f32,
            z: -1.0f32 * sinf(half_yaw_minus_heading),
        };
        self.quaternion = rotation * self.quaternion;
    }

    pub fn reset(&mut self) {
        self.quaternion = FusionQuaternion::identity();
        self.acc = FusionVector::zero();
        self.initialising = true;
        self.ramped_gain = INITIAL_GAIN;
        self.angular_rate_recovery = false;
        self.half_accelerometer_feedback = FusionVector::zero();
        self.half_magnetometer_feedback = FusionVector::zero();
        self.accelerometer_ignored = false;
        self.acceleration_recovery_trigger = 0;
        self.acceleration_recovery_timeout = self.settings.recovery_trigger_period;
        self.magnetometer_ignored = false;
        self.magnetic_recovery_trigger = 0;
        self.magnetic_recovery_timeout = self.settings.recovery_trigger_period;
    }

    pub fn calculate_half_gravity(&self) -> FusionVector {
        let q = self.quaternion;
        match self.settings.convention {
            FusionConvention::ENU | FusionConvention::NWU => {
                FusionVector {
                    x: q.x * q.z - q.w * q.y,
                    y: q.y * q.z + q.w * q.x,
                    z: q.w * q.w - 0.5f32 + q.z * q.z,
                }
            }
            FusionConvention::NED => {
                FusionVector {
                    x: q.w * q.y - q.x * q.z,
                    y: -1.0f32 * (q.y * q.z + q.w * q.x),
                    z: 0.5f32 - q.w * q.w - q.z * q.z,
                }
            }
        }
    }

    pub fn calculate_half_magnetic(&self) -> FusionVector {
        let q = self.quaternion;
        match self.settings.convention {
            FusionConvention::NWU => {
                let half_magnetic = FusionVector {
                    x: q.x * q.y + q.w * q.z,
                    y: q.w * q.w - 0.5f32 + q.y * q.y,
                    z: q.y * q.z - q.w * q.x,
                };
                half_magnetic
            }
            FusionConvention::ENU => {
                let half_magnetic = FusionVector {
                    x: 0.5f32 - q.w * q.w - q.x * q.x,
                    y: q.w * q.z - q.x * q.y,
                    z: -1.0f32 * (q.x * q.z + q.w * q.y),
                };
                half_magnetic
            }
            FusionConvention::NED => {
                let half_magnetic = FusionVector {
                    x: -1.0f32 * (q.x * q.y + q.w * q.z),
                    y: 0.5f32 - q.w * q.w - q.y * q.y,
                    z: q.w * q.x - q.y * q.z,
                };
                half_magnetic
            }
        }
    }
    pub fn earth_acc(&self) -> FusionVector {
        // Calculate accelerometer measurement in the Earth coordinate frame
        let q = self.quaternion;
        let a = self.acc;
        let qwqw = q.w * q.w; // calculate common terms to avoid repeated operations
        let qwqx = q.w * q.x;
        let qwqy = q.w * q.y;
        let qwqz = q.w * q.z;
        let qxqy = q.x * q.y;
        let qxqz = q.x * q.z;
        let qyqz = q.y * q.z;
        let mut accelerometer = FusionVector {
            x: 2.0f32 * ((qwqw - 0.5f32 + q.x * q.x) * a.x + (qxqy - qwqz) * a.y + (qxqz + qwqy) * a.z),
            y: 2.0f32 * ((qxqy + qwqz) * a.x + (qwqw - 0.5f32 + q.y * q.y) * a.y + (qyqz - qwqx) * a.z),
            z: 2.0f32 * ((qxqz - qwqy) * a.x + (qyqz + qwqx) * a.y + (qwqw - 0.5f32 + q.z * q.z) * a.z),
        };
        // Remove gravity from accelerometer measurement
        match self.settings.convention {
            FusionConvention::ENU | FusionConvention::NWU => {
                accelerometer.z -= 1.0f32;
                accelerometer
            }
            FusionConvention::NED => {
                accelerometer.z += 1.0f32;
                accelerometer
            }
        }
    }

    pub fn linear_acc(&self) -> FusionVector {
        // Calculate accelerometer measurement in the Earth coordinate frame
        let q = self.quaternion;
        // Calculate gravity in the sensor coordinate frame
        let gravity = FusionVector {
            x: 2.0f32 * (q.x * q.z - q.w * q.y),
            y: 2.0f32 * (q.y * q.z + q.w * q.x),
            z: 2.0f32 * (q.w * q.w - 0.5f32 + q.z * q.z),
        };

        // Remove gravity from accelerometer measurement
        match self.settings.convention {
            FusionConvention::ENU | FusionConvention::NWU => {
                self.acc - gravity
            }
            FusionConvention::NED => {
                self.acc + gravity
            }
        }
    }

    pub fn flags(&self) -> FusionAhrsFlags {
        FusionAhrsFlags {
            initializing: self.initialising,
            angular_rate_recovery: self.angular_rate_recovery,
            acceleration_recovery: self.acceleration_recovery_trigger
                > self.acceleration_recovery_timeout,
            magnetic_recovery: self.magnetic_recovery_trigger > self.magnetic_recovery_timeout,
        }
    }
}

impl FusionAhrsSettings {
    pub fn new() -> Self {
        Self {
            convention: NWU,
            gain: 0.5f32,
            gyr_range: 0.0f32,
            acc_rejection: 90.0f32,
            mag_rejection: 90.0f32,
            recovery_trigger_period: 0,
        }
    }
}

fn clamp<T: Ord>(value: T, min: T, max: T) -> T {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}
