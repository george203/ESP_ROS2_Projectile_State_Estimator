#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8

#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_WHO_AM_I        0x75
#define REG_ACCEL_XOUT_H    0x3B

static const char* TAG = "IMU_Node";

// data structure for accel/gyro tracking
typedef struct {
    float accel_x, accel_y, accel_z;  // m/s^2
    float gyro_x, gyro_y, gyro_z;     // rad/s
} mpu_data_t;

// I2C Master Bus Config
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;

// I2C Device Config (MPU)
i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU6050_ADDR,
    .scl_speed_hz = 100000,
};
i2c_master_dev_handle_t dev_handle;

// micro ros publishing vars
rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;

// helper function for writing an mpu reg
esp_err_t mpu_write_reg(uint8_t reg, uint8_t value) {
    // Build a 2-byte buffer: [register_address, value_to_write]
    // Call i2c_master_transmit(dev_handle, buffer, 2, timeout_ms)
    // Return the esp_err_t
    uint8_t buffer[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(dev_handle, buffer, 2, -1);
    return err;
}

// helper function for reading an mpu reg
esp_err_t mpu_read_reg(uint8_t reg, uint8_t *value) {
    // Call i2c_master_transmit_receive(dev_handle, &reg, 1, value, 1, timeout_ms)
    // Return the esp_err_t
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, value, 1, 1000);
    return err;
}

// mpu init function
esp_err_t mpu_init(void) {
    uint8_t id;

    // check device id against expected id
    esp_err_t err = mpu_read_reg(REG_WHO_AM_I, &id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(err));
        return err;
    }
    if(id != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I returned 0x%02X, expected 0x68", id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (OK)", id);

    // set init values for regs
    err = mpu_write_reg(REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to REG_PWR_MGMT_1: %s", esp_err_to_name(err));
        return err;
    }

    err = mpu_write_reg(REG_GYRO_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to REG_GYRO_CONFIG: %s", esp_err_to_name(err));
        return err;
    }

    err = mpu_write_reg(REG_ACCEL_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to REG_ACCEL_CONFIG: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

// helper function for reading bulk data
esp_err_t mpu_read_data(uint8_t *raw_buffer) {
    uint8_t reg = REG_ACCEL_XOUT_H;
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, raw_buffer, 14, 1000);
    return err;
}

// helper function for putting accel/gyro values in struct
void mpu_parse_data(mpu_data_t *mpu_dat, uint8_t *raw_buffer) {
    mpu_dat->accel_x = (int16_t)((raw_buffer[0] << 8) | raw_buffer[1]) / 16384.0f * 9.80665f;
    mpu_dat->accel_y = (int16_t)((raw_buffer[2] << 8) | raw_buffer[3]) / 16384.0f * 9.80665f;
    mpu_dat->accel_z = (int16_t)((raw_buffer[4] << 8) | raw_buffer[5]) / 16384.0f * 9.80665f;
    mpu_dat->gyro_x = (int16_t)((raw_buffer[8] << 8) | raw_buffer[9]) / 131.0 * (M_PI / 180.0);
    mpu_dat->gyro_y = (int16_t)((raw_buffer[10] << 8) | raw_buffer[11]) / 131.0 * (M_PI / 180.0);
    mpu_dat->gyro_z = (int16_t)((raw_buffer[12] << 8) | raw_buffer[13]) / 131.0 * (M_PI / 180.0);
}

// 
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;
    
    // Fill the message
    int64_t time_ns = rmw_uros_epoch_nanos();
    msg.header.stamp.sec = time_ns / 1000000000LL;
    msg.header.stamp.nanosec = time_ns % 1000000000LL;
    
    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 9.81;
    
    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = 0.0;
    
    msg.orientation_covariance[0] = -1.0;
    
    // Publish once, at the end
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}

//
void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "imu_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"imu/raw"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback,
		true));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}


void app_main(void) {
    // Configure and create the I2C bus (Step 2)
    // Add the MPU as a device (Step 3)
    // Call mpu_init() (Step 5)
    // For now, just log "MPU initialized" and exit — we'll add the read loop next
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    if (mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU init failed");
        return;
    }

    ESP_LOGI(TAG, "MPU Initialized");

    uros_network_interface_initialize();
    xTaskCreate(micro_ros_task,
            "uros_task",
            16000,
            NULL,
            5,
            NULL
    );

    // static int counter = 0;
    // while(1) {
    //     uint8_t raw_buffer[14];
    //     mpu_data_t data;

    //     mpu_read_data(raw_buffer);
    //     mpu_parse_data(&data, raw_buffer);

    //     if (++counter % 10 == 0) {  // print at 10 Hz
    //         ESP_LOGI(TAG, "accel: %.2f %.2f %.2f | gyro: %.2f %.2f %.2f",
    //          data.accel_x, data.accel_y, data.accel_z,
    //          data.gyro_x, data.gyro_y, data.gyro_z);
    //     }
    
    //     vTaskDelay(pdMS_TO_TICKS(10));  // ~100 Hz
    // }

    return;
}
