idf_component_register(
    SRCS "imu_node.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component
    PRIV_REQUIRES esp_driver_i2c
)
