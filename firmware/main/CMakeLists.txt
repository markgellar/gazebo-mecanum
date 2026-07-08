file(GLOB VL53L0X_SRC "libraries/Adafruit_VL53L0X/*.cpp")
file(GLOB BUSIO_SRC "libraries/Adafruit_BusIO/*.cpp")

idf_component_register(
  SRCS "main_controller.cpp" ${VL53L0X_SRC} ${BUSIO_SRC}
  INCLUDE_DIRS "."
    "libraries/Adafruit_VL53L0X"
    "libraries/Adafruit_BusIO"
    "libraries/Adafruit_Sensor"
  REQUIRES arduino micro_ros_espidf_component
)
