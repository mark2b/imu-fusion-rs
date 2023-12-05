#include <Fusion.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include<unistd.h>               // for linux

#define SAMPLE_RATE (25) // replace this with actual sample rate
const char* in_filename = "../fusion_in.csv";
const char* out_filename = "../fusion_c_out.csv";

int main(void) {

    FILE* in = fopen(in_filename, "r");
    if (in == NULL) {
        printf("Input file not found. %s", in_filename);
        return -1;
    }
    FILE* out = fopen(out_filename, "w");
    if (out == NULL) {
        printf("Output file not found. %s", out_filename);
        return -1;
    }
    
    
    // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    float t, last_t = 0, ax, ay, az, gx, gy, gz, mx, my, mz;
    // This loop should repeat each time new gyroscope data is available

    fprintf(out, "dt,euler_yaw,euler_pitch,euler_roll,earth_x,earth_y,earth_z,q_w,q_x,q_y,q_z\n");
    while (fscanf(in, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &t, &gx, &gy, &gz, &ax, &ay, &az, &mx, &my, &mz) != -1) {
        if (t == 0) {
            last_t = t;
            continue;
        }
        float delta = t - last_t;
        // Acquire latest sensor data
        const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
        FusionVector gyroscope = {gx, gy, gz}; // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {ax, ay, az}; // replace this with actual accelerometer data in g
        FusionVector magnetometer = {mx, my, mz}; // replace this with actual magnetometer data in arbitrary units

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);
        
        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta);

        FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
        
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        fprintf(out, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", t,
               euler.angle.yaw, euler.angle.pitch, euler.angle.roll,
               earth.x, earth.y, earth.z, q.array[0], q.array[1], q.array[2], q.array[3]);
        printf("%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", t,
               euler.angle.yaw, euler.angle.pitch, euler.angle.roll,
               earth.x, earth.y, earth.z, q.array[0], q.array[1], q.array[2], q.array[3]);
        last_t = t;
    }
}
