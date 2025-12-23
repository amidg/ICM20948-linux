#include "icm20948.h"

#define NUM_POINTS_GYRO_CALIBRATION 500
#define CALIBRATION_CYCLE_DELAY_US  1000
#define MAX_CALIBRATION_STEPS       4

int calibrate_gyro(const icm20948* device) {
    // check if device is valid
    if (!device || !device->data || !device->offset) return -1;

    // check if init was called before:
    // - i2c_fd should be >= 0
    // - i2c_device_path is not NULL
    if (device->i2c_fd < 0 || !device->i2c_device_path) return -1;

    // perform calibration based on the average offset
    device->offset->gx = 0;
    device->offset->gy = 0;
    device->offset->gz = 0;

    for (uint16_t i = 0; i < NUM_POINTS_GYRO_CALIBRATION; ++i) {
        if (icm20948_get_sensors(device) != 0) return -1;
        device->offset->gx += device->data->gx;
        device->offset->gy += device->data->gy;
        device->offset->gz += device->data->gz;
        usleep(CALIBRATION_CYCLE_DELAY_US); // 1ms
    }

    device->offset->gx /= NUM_POINTS_GYRO_CALIBRATION;
    device->offset->gy /= NUM_POINTS_GYRO_CALIBRATION;
    device->offset->gz /= NUM_POINTS_GYRO_CALIBRATION;

    return 0;
}

int get_accel_values(const icm20948* device) {
    // check if device is valid
    if (!device || !device->data || !device->offset) return -1;

    // check if init was called before:
    // - i2c_fd should be >= 0
    // - i2c_device_path is not NULL
    if (device->i2c_fd < 0 || !device->i2c_device_path) return -1;

    double ax = 0;
    double ay = 0;
    double az = 0;
        
    for (uint16_t i = 0; i < NUM_POINTS_GYRO_CALIBRATION; ++i) {
        if (icm20948_get_sensors(device) != 0) return -1;
        ax += device->data->ax;
        ay += device->data->ay;
        az += device->data->az;
        usleep(CALIBRATION_CYCLE_DELAY_US); // 1ms
    }

    device->data->ax = ax / NUM_POINTS_GYRO_CALIBRATION;
    device->data->ay = ay / NUM_POINTS_GYRO_CALIBRATION;
    device->data->az = az / NUM_POINTS_GYRO_CALIBRATION;

    return 0;
}

int calibrate_accel(const icm20948* device) {
    // check if device is valid
    if (!device || !device->data || !device->offset) return -1;

    // check if init was called before:
    // - i2c_fd should be >= 0
    // - i2c_device_path is not NULL
    if (device->i2c_fd < 0 || !device->i2c_device_path) return -1;

    int status = 0;
    size_t direction = 0; // 0, 1, 2
    double ax[MAX_CALIBRATION_STEPS];
    double ay[MAX_CALIBRATION_STEPS];
    double az[MAX_CALIBRATION_STEPS];
    while (direction < MAX_CALIBRATION_STEPS) {
        // wait for input
        char input;
        printf("Press SPACE to calibrate");
        input = getchar();
        if (input != 32) continue; // space

        // measure
        switch (direction) {
            case 0: // Z upward
                if (get_accel_values(device) != 0) return -1;
                break;
            case 1: // Z side, X down
                if (get_accel_values(device) != 0) return -1;
                break;
            case 2: // Z down
                if (get_accel_values(device) != 0) return -1;
                break;
            case 3: // Y up, Z side
                if (get_accel_values(device) != 0) return -1;
                break;
        };

        ax[direction] = device->data->ax;
        ay[direction] = device->data->ay;
        az[direction] = device->data->az;
        printf(
            "ax %f, ay %f, az %f\n",
            ax[direction],
            ay[direction],
            az[direction]
        );

        ++direction;
    }

    // calculate linear fit for X
    const double target_x[MAX_CALIBRATION_STEPS] = {0.0, -1.0, 0.0, 0.0};
    double sumsq;
    status = gsl_fit_linear(
        ax, 1,
        target_x, 1,
        MAX_CALIBRATION_STEPS,
        &(device->offset->ax_c0),
        &(device->offset->ax_c1),
        &(device->offset->ax_cov00),
        &(device->offset->ax_cov01),
        &(device->offset->ax_cov11),
        &sumsq
    );

    // linear fit y
    const double target_y[MAX_CALIBRATION_STEPS] = {0.0, 0.0, 0.0, -1.0};
    status = gsl_fit_linear(
        ay, 1,
        target_y, 1,
        MAX_CALIBRATION_STEPS,
        &(device->offset->ay_c0),
        &(device->offset->ay_c1),
        &(device->offset->ay_cov00),
        &(device->offset->ay_cov01),
        &(device->offset->ay_cov11),
        &sumsq
    );

    // linear fit z
    const double target_z[MAX_CALIBRATION_STEPS] = {1.0, 0.0, -1.0, 0.0};
    status = gsl_fit_linear(
        az, 1,
        target_z, 1,
        MAX_CALIBRATION_STEPS,
        &(device->offset->az_c0),
        &(device->offset->az_c1),
        &(device->offset->az_cov00),
        &(device->offset->az_cov01),
        &(device->offset->az_cov11),
        &sumsq
    );
    
    return status;
}

int main(void) {
    icm20948_data_t data;
    icm20948_offset_t offset;
    icm20948_config_t config = {1, LOW_PASS_ORDER_2, GYRO_RANGE_500_DEG, ACCEL_RANGE_2G};
    icm20948 imu = {-1, "/dev/i2c-1", &config, &data, &offset};

    // Initialize MPU6050
    if (icm20948_init(&imu) < 0) {
        printf("Failed to initialize IMU\n");
        return 1;
    }
    
    printf("IMU initialized successfully\n");

    // do calibration
    if (calibrate_gyro(&imu) != 0) {
        printf("Failed to calibrate gyroscope\n");
        return 1;
    }

    // calibrate accelerometer
    if (calibrate_accel(&imu) != 0) {
        printf("Failed to calibrate accelerometer\n");
        return 1;
    }

    // show offsets
    printf(
        "IMU calibrated successfully:\n"
        "gyro x: %f\n"
        "gyro y: %f\n"
        "gyro z: %f\n"
        "------------\n"
        "accel x: c0 %f, c1 %f, cov00 %f, cov01 %f, cov11 %f\n"
        "accel y: c0 %f, c1 %f, cov00 %f, cov01 %f, cov11 %f\n"
        "accel z: c0 %f, c1 %f, cov00 %f, cov01 %f, cov11 %f\n",
        imu.offset->gx,
        imu.offset->gy,
        imu.offset->gz,
        imu.offset->ax_c0,
        imu.offset->ax_c1,
        imu.offset->ax_cov00,
        imu.offset->ax_cov01,
        imu.offset->ax_cov11,
        imu.offset->ay_c0,
        imu.offset->ay_c1,
        imu.offset->ay_cov00,
        imu.offset->ay_cov01,
        imu.offset->ay_cov11,
        imu.offset->az_c0,
        imu.offset->az_c1,
        imu.offset->az_cov00,
        imu.offset->az_cov01,
        imu.offset->az_cov11
    );

    icm20948_close(&imu);
    return 0;
}
