#include "icm20948.h"

int main(void) {
    icm20948_data_t data;

    icm20948_config_t config = {1, LOW_PASS_ORDER_2, GYRO_RANGE_500_DEG, ACCEL_RANGE_2G};

    icm20948_offset_t offset;

    icm20948 imu = {-1, "/dev/i2c-1", &config, &data, &offset};

    // Initialize MPU6050
    if (icm20948_init(&imu) < 0) {
        printf("Failed to initialize IMU\n");
        return 1;
    }
    
    printf("Successfully initialized IMU\n");
    
    // Read data continuously
    while (1) {
	// read temperature
	if (icm20948_get_sensors_calibrated(&imu) == 0) {
		printf("Accel X / Y / Z (G): %f / %f / %f\n",
            data.ax, data.ay, data.az);
		printf("Gyro X / Y / Z (rad/s): %f / %f / %f\n",
            data.gx, data.gy, data.gz);
        printf("Magnetometer X / Y / Z (uT): %f / %f / %f\n",
            data.mx, data.my, data.mz);
	}
        
        usleep(100000); // 10ms delay
    }
    
    icm20948_close(&imu);
    return 0;
}
