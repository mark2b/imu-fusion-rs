use libm::{fabsf};
use crate::{CUTOFF_FREQUENCY, FusionGyrOffset, FusionVector, THRESHOLD, TIMEOUT};

impl FusionGyrOffset {
    pub fn new(sample_rate: u32) -> Self {
        Self {
            filter_coefficient: 2.0f32 * core::f32::consts::PI * CUTOFF_FREQUENCY * (1.0f32 / sample_rate as f32),
            gyroscope_offset: FusionVector::zero(),
            timeout: TIMEOUT * sample_rate,
            timer: 0,
        }
    }

    pub fn update(&mut self, mut gyr: FusionVector) -> FusionVector{
        unsafe {

            // Subtract offset from gyroscope measurement
            gyr = gyr - self.gyroscope_offset;
            // Reset timer if gyroscope not stationary

            if fabsf(gyr.axis.x) > THRESHOLD || fabsf(gyr.axis.y) > THRESHOLD || fabsf(gyr.axis.z) > THRESHOLD {
                self.timer = 0;
                return gyr;
            }

            // Increment timer while gyroscope stationary
            if self.timer < self.timeout {
                self.timer += 1;
                return gyr;
            }
            // Adjust offset if timer has elapsed
            self.gyroscope_offset = self.gyroscope_offset + (gyr * self.filter_coefficient);
            gyr
        }
    }
}