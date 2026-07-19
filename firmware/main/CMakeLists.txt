file(GLOB VL53L0X_SRC
  "libraries/Adafruit_VL53L0X/src/*.cpp"
  "libraries/Adafruit_VL53L0X/src/core/src/*.cpp"
  "libraries/Adafruit_VL53L0X/src/platform/src/*.cpp"
)
file(GLOB BUSIO_SRC "libraries/Adafruit_BusIO/*.cpp")

idf_component_register(
  SRCS "main_controller.cpp" ${VL53L0X_SRC} ${BUSIO_SRC}
  INCLUDE_DIRS "."
    "libraries/Adafruit_VL53L0X/src"
    "libraries/Adafruit_BusIO"
    "libraries/Adafruit_Sensor"
  REQUIRES arduino micro_ros_espidf_component
)

set_source_files_properties(main_controller.cpp PROPERTIES COMPILE_FLAGS "-Wno-error")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=unused-result)

set_source_files_properties(${VL53L0X_SRC} PROPERTIES COMPILE_FLAGS "-Wno-error=maybe-uninitialized -Wno-error")
set_source_files_properties(${BUSIO_SRC} PROPERTIES COMPILE_FLAGS "-Wno-error")
